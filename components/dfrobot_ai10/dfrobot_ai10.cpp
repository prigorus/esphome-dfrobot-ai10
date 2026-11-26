#include "dfrobot_ai10.h"

namespace esphome {
namespace dfrobot_ai10 {

static const char *const TAG = "dfrobot_ai10";

void DFRobotAI10Component::setup() {
  ESP_LOGI(TAG, "SEN0677 AI Vision Sensor — initializing...");
  // Boot-up delay (datasheet requirement: 900ms–2.5s)
  // This is in setup(), so blocking here is acceptable and safer for init sequence.
  this->set_timeout("ai10_boot", 3000, [this]() {
    this->setup_done_ = true;
    this->send_reset();
  });

  // Send initial reset (mirrors Arduino library begin())

}

void DFRobotAI10Component::dump_config() {
  ESP_LOGCONFIG(TAG, "DFRobot AI10 (SEN0677):");
  ESP_LOGCONFIG(TAG, "  Protocol: SyncWord=0xEF 0xAA, XOR checksum");
  this->check_uart_settings(115200);
}

void DFRobotAI10Component::loop() {
  // Read bytes one-by-one and feed into state machine parser
  uint32_t now = millis();
  if (parser_state_ != STATE_WAIT_SYNC_H && now - last_byte_ > 250)
    parser_state_ = STATE_WAIT_SYNC_H;
  for (size_t budget = 0; budget < 2048 && this->available(); ++budget) {
    uint8_t byte;
    if (!this->read_byte(&byte)) break;
    last_byte_ = millis();

    switch (this->parser_state_) {
      case STATE_WAIT_SYNC_H:
        if (byte == SYNC_H) {
          this->parser_state_ = STATE_WAIT_SYNC_L;
        }
        break;

      case STATE_WAIT_SYNC_L:
        if (byte == SYNC_L) {
          this->parser_state_ = STATE_READ_HEADER;
          this->header_pos_ = 0;
        } else if (byte == SYNC_H) {
          // Another 0xEF — stay in WAIT_SYNC_L
        } else {
          this->parser_state_ = STATE_WAIT_SYNC_H;
        }
        break;

      case STATE_READ_HEADER:
        this->header_buf_[this->header_pos_++] = byte;
        if (this->header_pos_ >= 3) {
          // Header complete: msg_id(1) + lenH(1) + lenL(1)
          this->payload_len_ = (this->header_buf_[1] << 8) | this->header_buf_[2];
          this->payload_pos_ = 0;

          if (this->payload_len_ >= sizeof(this->payload_buf_)) {
            ESP_LOGW(TAG, "Payload too large: %d bytes, resetting parser", this->payload_len_);
            this->parser_state_ = STATE_WAIT_SYNC_H;
          } else {
            this->parser_state_ = STATE_READ_PAYLOAD;
          }
        }
        break;

      case STATE_READ_PAYLOAD:
        this->payload_buf_[this->payload_pos_++] = byte;
        if (this->payload_pos_ >= this->payload_len_ + 1) {
          // Packet complete — verify XOR checksum
          uint8_t xor_calc = 0;
          for (uint8_t i = 0; i < 3; i++) xor_calc ^= this->header_buf_[i];
          for (uint16_t i = 0; i < this->payload_len_; i++) xor_calc ^= this->payload_buf_[i];

          uint8_t xor_recv = this->payload_buf_[this->payload_len_];

          if (xor_calc == xor_recv) {
            uint8_t msg_id = this->header_buf_[0];
            // ESP_LOGV(TAG, "Packet received: MsgID=0x%02X, Len=%d, XOR OK", msg_id, this->payload_len_);
            this->process_packet_(msg_id, this->payload_buf_, this->payload_len_);
          } else {
            ESP_LOGW(TAG, "XOR checksum mismatch! calc=0x%02X recv=0x%02X", xor_calc, xor_recv);
          }
          this->parser_state_ = STATE_WAIT_SYNC_H;
        }
        break;
    }
  }

  // Status log every 10 seconds
  now = millis();
  if ((waiting_reset_ || pending_cmd_) && now - command_started_ > command_timeout_ms_) {
    uint8_t cmd = waiting_reset_ ? CMD_RESET : pending_cmd_;
    fail_command_(cmd, RES_FAILED_TIMEOUT);
  }
  if (config_test_phase_ == 3 && now - config_test_hold_started_ >= 80) {
    config_test_phase_ = 4;
    command_started_ = now; command_timeout_ms_ = 3000;
    send_extended_packet_(0xF9, {3});
  }
  if (refresh_queued_ && setup_done_ && !waiting_reset_ && (!pending_cmd_ || pending_cmd_ == CMD_VERIFY)) {
    refresh_queued_ = false;
    refresh_users();
  }
  if (repeat_wait_ && !waiting_reset_ && pending_cmd_ == CMD_VERIFY &&
      stored_continuous_ && !suppress_verify_ && now - repeat_started_ >= 1000) {
    repeat_wait_ = false;
    pending_action_ = PENDING_VERIFY;
    execute_pending_action_();
  }
  if (this->setup_done_ && now - this->last_status_log_ > 10000) {
    this->last_status_log_ = now;
    ESP_LOGD(TAG, "Parser state: %s, last result: %s",
             this->parser_state_ == STATE_WAIT_SYNC_H ? "IDLE" : "RECEIVING",
             result_to_str_(this->last_result_));
  }
}

// ===== Public API (Non-blocking State Machine) =====

bool DFRobotAI10Component::enrollment_command_(uint8_t cmd) {
  return cmd == CMD_ENROLL_SINGLE || cmd == CMD_ENROLL_DIRECTION || cmd == CMD_ENROLL_INTERACTIVE || cmd == 0xFB || cmd == 0xF7;
}
void DFRobotAI10Component::invalidate_auth_() {
  recognized_ = false; face_detected_ = false; last_uid_ = 0;
  last_user_name_.clear(); last_type_ = TYPE_NONE; last_admin_ = false;
}
void DFRobotAI10Component::cancel_stop_() {
  cancel_timeout("ai10_stop_override");
  cancel_timeout("ai10_stop_final");
  stop_in_progress_ = false;
}
void DFRobotAI10Component::fail_command_(uint8_t cmd, uint8_t result) {
  repeat_wait_ = false;
  if (config_test_phase_) {
    ESP_LOGW(TAG, "AI10 test interrupted: cmd=0x%02X result=0x%02X; no automatic restart after transport failure", cmd, result);
    cancel_config_test_();
  }
  bool failed_directory = directory_active_;
  cancel_directory_();
  bool enroll = enrollment_command_(cmd) || (cmd == CMD_RESET && (pending_action_ == PENDING_ENROLL || (pending_action_ == PENDING_EXTENDED && enrollment_command_(extended_cmd_))));
  bool verify = cmd == CMD_VERIFY || (cmd == CMD_RESET && pending_action_ == PENDING_VERIFY);
  waiting_reset_ = false; pending_action_ = PENDING_NONE; pending_cmd_ = 0;
  clear_transfer_();
  suppress_verify_ = true; invalidate_auth_(); last_result_ = result;
  if (enroll) enrollment_failed_.call(result);
  if (verify) recognition_failed_.call(result);
  if (failed_directory) user_directory_failed_.call(result);
  command_result_.call(cmd, result);
}
void DFRobotAI10Component::transmit_reset_() {
  uint8_t data[] = {CMD_RESET, 0, 0};
  waiting_reset_ = true; command_started_ = millis(); command_timeout_ms_ = 3000;
  send_packet_(data, sizeof(data));
}
void DFRobotAI10Component::send_reset() {
  repeat_wait_ = false;
  cancel_config_test_();
  cancel_stop_(); cancel_directory_(); pending_action_ = PENDING_NONE; pending_cmd_ = 0;
  clear_transfer_();
  suppress_verify_ = true; invalidate_auth_();
  if (setup_done_) transmit_reset_();
}
bool DFRobotAI10Component::prepare_(PendingAction action) {
  if (!setup_done_ || waiting_reset_ || (pending_cmd_ && pending_cmd_ != CMD_VERIFY)) {
    ESP_LOGW(TAG, "Module busy or booting; command rejected");
    return false;
  }
  if (action == PENDING_ENROLL || action == PENDING_DELETE_USER || action == PENDING_DELETE_ALL ||
      (action == PENDING_EXTENDED && auto_refresh_users_))
    remember_recognition_();
  else if (!directory_active_) resume_pending_ = false;
  repeat_wait_ = false;
  cancel_stop_(); suppress_verify_ = true; invalidate_auth_();
  pending_cmd_ = 0; pending_action_ = action;
  return true;
}
void DFRobotAI10Component::stop_recognition() {
  send_reset();
  stop_in_progress_ = true;
  // Preserve the upstream firmware workaround without treating its reply as authorization.
  set_timeout("ai10_stop_override", 80, [this]() {
    if (!setup_done_) return;
    uint8_t data[] = {CMD_VERIFY, 0, 2, 0, 1};
    send_packet_(data, sizeof(data));
  });
  set_timeout("ai10_stop_final", 1500, [this]() { send_reset(); });
}
void DFRobotAI10Component::start_recognition(uint8_t timeout, bool continuous) {
  if (!prepare_(PENDING_VERIFY)) return;
  stored_timeout_ = timeout; stored_continuous_ = continuous; max_attempts_ = -1;
  transmit_reset_();
}
void DFRobotAI10Component::start_recognition(uint8_t timeout, bool continuous, uint8_t max_attempts) {
  if (!prepare_(PENDING_VERIFY)) return;
  stored_timeout_ = timeout; stored_continuous_ = continuous; max_attempts_ = max_attempts;
  transmit_reset_();
}
void DFRobotAI10Component::enroll_user(uint8_t admin, const char *name, uint8_t timeout) {
  if (!name || !prepare_(PENDING_ENROLL)) return;
  stored_admin_ = admin; stored_name_ = name; stored_timeout_ = timeout;
  stored_enroll_direction_ = 0x00;
  transmit_reset_();
}
void DFRobotAI10Component::start_configured_recognition(uint8_t timeout, bool continuous, int attempts) {
  if (attempts == -1) start_recognition(timeout, continuous);
  else if (attempts >= 0 && attempts <= 255) start_recognition(timeout, continuous, uint8_t(attempts));
  else ESP_LOGW(TAG, "Recognition attempts must be -1 or 0..255");
}
void DFRobotAI10Component::enroll_palm(uint8_t admin, const char *name, uint8_t timeout) {
  if (!name || !*name || strlen(name)>31 || admin>1 || !timeout || !prepare_(PENDING_ENROLL)) return;
  stored_admin_ = admin; stored_name_ = name; stored_timeout_ = timeout;
  stored_enroll_direction_ = 0xFD;
  transmit_reset_();
}
void DFRobotAI10Component::delete_user(uint16_t uid) {
  if (!prepare_(PENDING_DELETE_USER)) return;
  stored_uid_ = uid; transmit_reset_();
}
void DFRobotAI10Component::delete_all_users() {
  if (!prepare_(PENDING_DELETE_ALL)) return;
  transmit_reset_();
}
void DFRobotAI10Component::remember_recognition_() {
  if (resume_recognition_after_refresh_ && pending_cmd_ == CMD_VERIFY && stored_continuous_ && !suppress_verify_) {
    resume_pending_ = true;
    resume_timeout_ = stored_timeout_;
    resume_attempts_ = max_attempts_;
  }
}
void DFRobotAI10Component::invalidate_directory_() {
  directory_complete_ = false;
  users_.clear();
}
void DFRobotAI10Component::cancel_directory_() {
  if (directory_active_) {
    invalidate_directory_();
    user_list_complete_ = false; user_ids_.clear(); user_count_ = 0;
  }
  directory_active_ = false;
  refresh_queued_ = false;
  resume_pending_ = false;
  staged_users_.clear();
}
void DFRobotAI10Component::schedule_directory_refresh_() {
  invalidate_directory_();
  if (auto_refresh_users_) refresh_queued_ = true;
  else resume_pending_ = false;
}
void DFRobotAI10Component::refresh_users() {
  // Keep one outstanding request; never overlap user-detail replies.
  if (!setup_done_ || waiting_reset_ || directory_active_ || (pending_cmd_ && pending_cmd_ != CMD_VERIFY)) {
    ESP_LOGW(TAG, "User directory refresh rejected: module busy or booting");
    return;
  }
  remember_recognition_();
  directory_active_ = true;
  if (!prepare_(PENDING_GET_USERS)) { cancel_directory_(); return; }
  refresh_queued_ = false; directory_index_ = 0;
  invalidate_directory_(); staged_users_.clear();
  user_ids_.clear(); user_count_ = 0; user_list_complete_ = false;
  transmit_reset_();
}
void DFRobotAI10Component::request_next_user_() {
  if (directory_index_ >= user_ids_.size()) { finish_directory_(); return; }
  uint16_t uid = user_ids_[directory_index_];
  extended_data_ = {uint8_t(uid >> 8), uint8_t(uid)};
  pending_cmd_ = CMD_GET_USER_INFO;
  command_started_ = millis(); command_timeout_ms_ = 3000;
  send_extended_packet_(pending_cmd_,extended_data_);
}
void DFRobotAI10Component::finish_directory_() {
  users_.swap(staged_users_); staged_users_.clear();
  directory_complete_ = true; directory_active_ = false; pending_cmd_ = 0;
  bool resume = resume_pending_; resume_pending_ = false;
  uint8_t timeout = resume_timeout_;
  int16_t attempts = resume_attempts_;
  // Resume only after successful refresh; authorization remains cleared.
  if (resume) {
    if (attempts < 0) start_recognition(timeout,true);
    else start_recognition(timeout,true,uint8_t(attempts));
  }
  user_directory_.call(uint16_t(users_.size()));
}
std::string DFRobotAI10Component::get_user_summary() const {
  if (!directory_complete_) return {};
  // ESPHome text sensors have a short state limit. Use per-user events for large directories.
  std::string out;
  for (const auto &u : users_) {
    std::string row = std::to_string(u.uid) + ": " + u.name +
        (u.is_palm() ? " [palm]" : " [face]") + (u.admin ? " [admin]" : "");
    if (out.size()+row.size()+2 > 240) { out += "..."; break; }
    if (!out.empty()) out += "; ";
    out += row;
  }
  return out;
}
void DFRobotAI10Component::get_all_user_ids() {
  if (!prepare_(PENDING_GET_USERS)) return;
  invalidate_directory_(); user_ids_.clear(); user_count_ = 0; user_list_complete_ = false; transmit_reset_();
}
void DFRobotAI10Component::scan_qr_code(uint8_t timeout) {
  if (!prepare_(PENDING_SCAN_QR)) return;
  stored_timeout_ = timeout; transmit_reset_();
}
void DFRobotAI10Component::queue_extended_(uint8_t cmd, std::vector<uint8_t> data, uint32_t timeout_ms) {
  if (!prepare_(PENDING_EXTENDED)) return;
  extended_cmd_ = cmd; extended_data_ = std::move(data); extended_timeout_ms_ = timeout_ms;
  transmit_reset_();
}
void DFRobotAI10Component::cancel_config_test_() {
  config_test_phase_ = 0;
  config_test_resume_ = false;
}
void DFRobotAI10Component::queue_config_test_(uint8_t cmd, std::vector<uint8_t> data, uint8_t phase, bool idle_only) {
  if (config_test_phase_ || stop_in_progress_ || refresh_queued_ || directory_active_ ||
      (is_busy() && (idle_only || !is_continuous_recognition_active()))) {
    ESP_LOGW(TAG, "AI10 test 0x%02X rejected: module busy; IR/UVC initialization requires idle", cmd);
    return;
  }
  bool resume = is_continuous_recognition_active();
  uint8_t timeout = stored_timeout_;
  int16_t attempts = max_attempts_;
  if (!prepare_(PENDING_EXTENDED)) return;
  // Test recovery is independent of user-directory refresh recovery.
  resume_pending_ = false;
  config_test_resume_ = resume;
  config_test_timeout_ = timeout;
  config_test_attempts_ = attempts;
  config_test_phase_ = phase;
  extended_cmd_ = cmd; extended_data_ = std::move(data); extended_timeout_ms_ = 3000;
  ESP_LOGI(TAG, "AI10 test cmd=0x%02X bytes=%u resume=%d", cmd, unsigned(extended_data_.size()), resume);
  transmit_reset_();
}
void DFRobotAI10Component::finish_config_test_(uint8_t cmd, uint8_t result) {
  bool resume = config_test_resume_;
  uint8_t timeout = config_test_timeout_;
  int16_t attempts = config_test_attempts_;
  cancel_config_test_();
  pending_cmd_ = 0; last_result_ = result;
  command_result_.call(cmd,result);
  // Only resume after a matched reply, never after missing/corrupt replies.
  if (resume && !is_busy() && !stop_in_progress_) start_configured_recognition(timeout,true,attempts);
}
void DFRobotAI10Component::test_white_led() { queue_config_test_(0xF9,{2},2); }
void DFRobotAI10Component::test_white_led_off() { queue_config_test_(0xF9,{3}); }
void DFRobotAI10Component::test_auto_light(int state) {
  if (state == -1) queue_config_test_(0xDB,{});
  else if (state == 0 || state == 1) queue_config_test_(0xDB,{uint8_t(state)});
}
void DFRobotAI10Component::test_read_uvc() { queue_config_test_(CMD_UVC_READ,{}); }
void DFRobotAI10Component::test_write_uvc(int usb,int bitrate,int quality,int attributes) {
  if ((usb!=0x11 && usb!=0x20 && usb!=0x91 && usb!=0xA0) ||
      bitrate<4 || bitrate>255 || quality<10 || quality>100 || attributes<0 || attributes>3) {
    ESP_LOGW(TAG,"Invalid UVC parameters"); return;
  }
  queue_config_test_(CMD_UVC_WRITE,{uint8_t(usb),uint8_t(bitrate),uint8_t(quality),uint8_t(attributes)});
}
void DFRobotAI10Component::test_ir_uvc(bool enabled) { queue_config_test_(enabled?0xF4:0xF5,{},1,true); }
void DFRobotAI10Component::query_status() { queue_extended_(CMD_STATUS, {}); }
void DFRobotAI10Component::query_version() { queue_extended_(CMD_VERSION, {}); }
void DFRobotAI10Component::query_serial() { queue_extended_(CMD_SERIAL, {}); }
void DFRobotAI10Component::query_user(uint16_t uid) {
  stored_uid_ = uid;
  queue_extended_(CMD_GET_USER_INFO, {uint8_t(uid >> 8), uint8_t(uid)});
}
void DFRobotAI10Component::clear_enrollment() { queue_extended_(CMD_FACE_RESET, {}); }
void DFRobotAI10Component::enroll_direction(uint8_t admin, const char *name, uint8_t direction, uint8_t timeout) {
  if (!name || admin > 1 || direction > 31) return;
  std::vector<uint8_t> data(35, 0); data[0] = admin;
  memcpy(data.data()+1, name, std::min<size_t>(strlen(name), 31));
  data[33] = direction; data[34] = timeout;
  queue_extended_(CMD_ENROLL_DIRECTION, std::move(data), uint32_t(timeout)*1000+3000);
}
void DFRobotAI10Component::enroll_interactive(uint8_t admin, const char *name, uint8_t type, uint8_t duplicate, uint8_t timeout) {
  if (!name || admin > 1 || type > 1 || duplicate > 2) return;
  std::vector<uint8_t> data(40, 0); data[0] = admin;
  memcpy(data.data()+1, name, std::min<size_t>(strlen(name), 31));
  data[34] = type; data[35] = duplicate; data[36] = timeout;
  queue_extended_(CMD_ENROLL_INTERACTIVE, std::move(data), uint32_t(timeout)*1000+3000);
}
void DFRobotAI10Component::query_uvc() { queue_extended_(CMD_UVC_READ, {}); }
void DFRobotAI10Component::configure_uvc(uint8_t usb, uint8_t bitrate, uint8_t quality, uint8_t attributes) {
  if ((usb != 0x11 && usb != 0x20 && usb != 0x91 && usb != 0xA0) || quality < 10 || quality > 100 || attributes > 3) return;
  queue_extended_(CMD_UVC_WRITE, {usb, bitrate, quality, attributes});
}

// Sensitive operations are opt-in, stream downloads rather than allocating whole images.
void DFRobotAI10Component::clear_transfer_() {
  transfer_sink_ = nullptr;
  upload_active_ = false;
  std::fill(upload_bytes_.begin(), upload_bytes_.end(), 0);
  upload_bytes_.clear();
}
void DFRobotAI10Component::send_extended_packet_(uint8_t cmd, const std::vector<uint8_t> &data) {
  std::vector<uint8_t> packet{cmd,uint8_t(data.size() >> 8),uint8_t(data.size())};
  packet.insert(packet.end(),data.begin(),data.end());
  send_packet_(packet.data(),packet.size());
}
bool DFRobotAI10Component::capture_image(uint8_t kind, uint8_t resolution, ChunkSink sink, uint32_t first_reply_timeout_ms) {
  if (!allow_sensitive_transfers_ || !sink || (kind != 0x71 && kind != 0x72 && kind != 0x74) || resolution > 2 || !prepare_(PENDING_EXTENDED)) return false;
  transfer_sink_ = std::move(sink); transfer_size_ = transfer_received_ = 0; transfer_seq_ = 0; capture_uid_ = 0;
  extended_cmd_ = kind; extended_data_ = {0};
  if (kind == 0x74) extended_data_.push_back(resolution);
  extended_timeout_ms_ = std::max<uint32_t>(1000, first_reply_timeout_ms); transmit_reset_();
  return true;
}
void DFRobotAI10Component::export_template(uint16_t uid, uint8_t type, ChunkSink sink) {
  if (!allow_sensitive_transfers_ || !sink || uid < 1 || uid > 2000 || type < 1 || type > 3 || !prepare_(PENDING_EXTENDED)) return;
  transfer_sink_ = std::move(sink); transfer_size_ = transfer_received_ = 0; transfer_seq_ = 0; transfer_uid_ = uid;
  extended_cmd_ = 0xFA; extended_data_ = {0,0,uint8_t(uid>>8),uint8_t(uid),type};
  extended_timeout_ms_ = 10000; transmit_reset_();
}
void DFRobotAI10Component::import_template(uint8_t type, const std::string &name, std::vector<uint8_t> bytes) {
  // The protocol uses short final chunks; exact multiples need vendor clarification.
  if (!allow_sensitive_transfers_ || type < 1 || type > 3 || bytes.empty() || bytes.size()>16384 || bytes.size()%246==0 || name.size()>20 || !prepare_(PENDING_EXTENDED)) return;
  upload_bytes_ = std::move(bytes); upload_active_ = true; transfer_type_ = type;
  transfer_size_ = upload_bytes_.size(); transfer_received_=0; transfer_seq_=0;
  extended_cmd_=0xFB; extended_data_={0,0,type};
  if (!name.empty()) { extended_data_.push_back(name.size()); extended_data_.insert(extended_data_.end(),name.begin(),name.end()); }
  extended_timeout_ms_=10000; transmit_reset_();
}
void DFRobotAI10Component::enroll_photo(uint8_t bio_type, const std::string &name, std::vector<uint8_t> bytes) {
  if (!allow_sensitive_transfers_ || bio_type > 3 || bytes.empty() || bytes.size()>65536 || name.size()>20 || !prepare_(PENDING_EXTENDED)) return;
  upload_bytes_=std::move(bytes); upload_active_=true;
  transfer_size_=upload_bytes_.size(); transfer_received_=0; transfer_seq_=0;
  uint32_t n=transfer_size_;
  extended_cmd_=0xF7; extended_data_={0,0,uint8_t(n>>24),uint8_t(n>>16),uint8_t(n>>8),uint8_t(n),bio_type};
  if (!name.empty()) { extended_data_.push_back(name.size()); extended_data_.insert(extended_data_.end(),name.begin(),name.end()); }
  extended_timeout_ms_=10000; transmit_reset_();
}
void DFRobotAI10Component::set_duplicate_check(bool enabled) { queue_extended_(0xFC,{uint8_t(enabled)}); }
void DFRobotAI10Component::request_chunk_() {
  ++transfer_seq_; command_started_=millis(); command_timeout_ms_=10000;
  if (transfer_sink_) {
    if (pending_cmd_ == 0xFA)
      send_extended_packet_(pending_cmd_,{uint8_t(transfer_seq_>>8),uint8_t(transfer_seq_),uint8_t(transfer_uid_>>8),uint8_t(transfer_uid_)});
    else send_extended_packet_(pending_cmd_,{uint8_t(transfer_seq_)});
  } else if (upload_active_) {
    std::vector<uint8_t> data{uint8_t(transfer_seq_>>8),uint8_t(transfer_seq_)};
    if (pending_cmd_==0xFB) data.push_back(transfer_type_);
    size_t n=std::min<size_t>(246,transfer_size_-transfer_received_);
    data.insert(data.end(),upload_bytes_.begin()+transfer_received_,upload_bytes_.begin()+transfer_received_+n);
    transfer_received_+=n;
    send_extended_packet_(pending_cmd_,data);
    std::fill(data.begin(),data.end(),0);
  }
}
void DFRobotAI10Component::handle_chunk_(uint8_t cmd,const uint8_t *data,uint16_t len) {
  if (waiting_reset_ || cmd!=pending_cmd_ || !transfer_sink_ || !transfer_seq_ || transfer_received_>=transfer_size_) return;
  size_t expected=std::min<uint32_t>(cmd==0xFA?246:1024,transfer_size_-transfer_received_);
  if (len!=expected || !transfer_sink_(data,len)) { fail_command_(cmd,RES_FAILED_INVALID_PARAM); return; }
  transfer_received_+=len;
  if (transfer_received_==transfer_size_) {
    pending_cmd_=0; clear_transfer_(); last_result_=RES_SUCCESS; command_result_.call(cmd,RES_SUCCESS);
  } else request_chunk_();
}

// ===== Internal Helper: Execute Pending Action =====

void DFRobotAI10Component::execute_pending_action_() {
  if (this->pending_action_ == PENDING_NONE) return;

  command_started_ = millis();
  command_timeout_ms_ = 3000;
  switch (this->pending_action_) {
    case PENDING_VERIFY: {
      uint8_t data[] = {CMD_VERIFY, 0, uint8_t(max_attempts_ < 0 ? 2 : 3),
        uint8_t(stored_continuous_ && !full_scan_ ? 1 : 0), stored_timeout_, uint8_t(max_attempts_)};
      suppress_verify_ = false;
      command_timeout_ms_ = stored_continuous_ && !full_scan_ ? 0x7FFFFFFFUL : uint32_t(stored_timeout_)*1000+3000;
      send_packet_(data, max_attempts_ < 0 ? 5 : 6);
      this->pending_cmd_ = CMD_VERIFY;
      this->recognized_ = false; // Reset auth state on new verify start
      ESP_LOGI(TAG, "CMD: VERIFY (0x12) continuous=%d timeout=%ds", this->stored_continuous_, this->stored_timeout_);
      break;
    }

    case PENDING_ENROLL: {
      command_timeout_ms_ = uint32_t(stored_timeout_)*1000+3000;
      ESP_LOGI(TAG, "CMD: ENROLL timeout=%ds", stored_timeout_);
      
      EnrollData ed;
      memset(&ed, 0, sizeof(ed));
      ed.admin = this->stored_admin_;
      strncpy((char *)ed.user_name, this->stored_name_.c_str(), 31);
      ed.face_dir = stored_enroll_direction_;
      ed.timeout = this->stored_timeout_;

      uint16_t struct_size = sizeof(EnrollData);
      std::vector<uint8_t> data;
      data.push_back(CMD_ENROLL_SINGLE);
      data.push_back((struct_size >> 8) & 0xFF);
      data.push_back(struct_size & 0xFF);
      for (uint16_t i = 0; i < struct_size; i++) {
        data.push_back(((uint8_t *)&ed)[i]);
      }
      this->send_packet_(data.data(), data.size());
      this->pending_cmd_ = CMD_ENROLL_SINGLE;
      break;
    }

    case PENDING_DELETE_USER: {
      uint8_t data[] = {
        CMD_DEL_USER,
        0x00, 0x02,
        (uint8_t)((this->stored_uid_ >> 8) & 0xFF),
        (uint8_t)(this->stored_uid_ & 0xFF)
      };
      this->send_packet_(data, sizeof(data));
      this->pending_cmd_ = CMD_DEL_USER;
      ESP_LOGI(TAG, "CMD: DELETE USER uid=%d", this->stored_uid_);
      break;
    }

    case PENDING_DELETE_ALL: {
      uint8_t data[] = {CMD_DEL_ALL_USER, 0x00, 0x00};
      this->send_packet_(data, sizeof(data));
      this->pending_cmd_ = CMD_DEL_ALL_USER;
      ESP_LOGI(TAG, "CMD: DELETE ALL USERS");
      break;
    }

    case PENDING_GET_USERS: {
      user_page_ = 0;
      uint8_t data[] = {CMD_GET_ALL_USERID, 0x00, 0x01, 0x00};
      this->send_packet_(data, sizeof(data));
      this->pending_cmd_ = CMD_GET_ALL_USERID;
      ESP_LOGI(TAG, "CMD: GET ALL USER IDs");
      break;
    }

    case PENDING_SCAN_QR: {
      command_timeout_ms_ = uint32_t(stored_timeout_)*1000+3000;
      uint8_t data[] = {CMD_SCAN_QR_CODE, 0, 2, 0, stored_timeout_};
      uint8_t legacy[] = {CMD_SCAN_QR_CODE, 0, 1, stored_timeout_};
      if (documented_qr_) send_packet_(data, sizeof(data));
      else send_packet_(legacy, sizeof(legacy));
      this->pending_cmd_ = CMD_SCAN_QR_CODE;
      ESP_LOGI(TAG, "CMD: SCAN QR CODE timeout=%ds", this->stored_timeout_);
      break;
    }
    
    case PENDING_EXTENDED: {
      std::vector<uint8_t> packet{extended_cmd_, uint8_t(extended_data_.size() >> 8), uint8_t(extended_data_.size())};
      packet.insert(packet.end(), extended_data_.begin(), extended_data_.end());
      pending_cmd_ = extended_cmd_; command_timeout_ms_ = extended_timeout_ms_;
      send_packet_(packet.data(), packet.size());
      break;
    }
    default:
      break;
  }

  // Clear pending action after execution
  this->pending_action_ = PENDING_NONE;
}

// ===== Protocol Implementation =====

void DFRobotAI10Component::send_packet_(const uint8_t *data, size_t len) {
  uint8_t xor_check = xor_checksum_(data, len);

  // Wire format: [SYNC_H][SYNC_L][data...][XOR]
  uint8_t sync[2] = {SYNC_H, SYNC_L};
  this->write_array(sync, 2);
  this->write_array(data, len);
  this->write_byte(xor_check);
  // UART driver buffers outgoing bytes; avoid a blocking flush here.

  // Debug log (hex string construction omitted for brevity/performance in standard log level)
  if (len <= 8) {
     // Only full hex dump for short packets
     char buf[32] = {0};
     for(size_t i=0; i<len; i++) sprintf(buf + i*3, "%02X ", data[i]);
     ESP_LOGD(TAG, "TX: EF AA %s%02X", buf, xor_check);
  } else {
     ESP_LOGD(TAG, "TX: EF AA [Payload %d bytes] %02X", len, xor_check);
  }
}

uint8_t DFRobotAI10Component::xor_checksum_(const uint8_t *data, size_t len) {
  uint8_t xor_val = 0;
  for (size_t i = 0; i < len; i++) {
    xor_val ^= data[i];
  }
  return xor_val;
}

void DFRobotAI10Component::process_packet_(uint8_t msg_id, const uint8_t *payload, uint16_t len) {
  if (msg_id == MID_REPLY) {
    this->handle_reply_(payload, len);
  } else if (msg_id == MID_NOTE) {
    this->handle_note_(payload, len);
  } else if (msg_id == 0x71 || msg_id == 0x72 || msg_id == 0x74 || msg_id == 0xFA) {
    handle_chunk_(msg_id,payload,len);
  } else {
    ESP_LOGW(TAG, "Unknown MsgID: 0x%02X", msg_id);
    // Do not log unsolicited payloads.
  }
}

void DFRobotAI10Component::schedule_repeat_() {
  // Hold the logical VERIFY operation active so shutters and directory refresh can pause it.
  // Ignore late replies/notes between single-shot cycles.
  repeat_wait_ = true;
  repeat_started_ = command_started_ = millis();
  command_timeout_ms_ = 4000;
}
void DFRobotAI10Component::handle_reply_(const uint8_t *payload, uint16_t len) {
  if (len < 2) {
    ESP_LOGW(TAG, "Reply too short: %d bytes", len);
    return;
  }

  uint8_t cmd = payload[0], result = payload[1];
  const uint8_t *data = payload + 2;
  uint16_t n = len - 2;
  if (cmd == CMD_RESET) {
    if (!waiting_reset_) return;
    if (result != RES_SUCCESS) { fail_command_(cmd, result); return; }
    waiting_reset_ = false; last_result_ = result;
    execute_pending_action_();
    return;
  }
  if (waiting_reset_ || cmd != pending_cmd_ || !pending_cmd_ || (cmd == CMD_VERIFY && (suppress_verify_ || repeat_wait_))) return;
  if (config_test_phase_) {
    if (config_test_phase_ == 3) return; // Ignore duplicate replies during the hold.
    ESP_LOGI(TAG, "AI10 test reply: cmd=0x%02X result=0x%02X bytes=%u",cmd,result,unsigned(n));
    if (n && n <= 16) {
      char buf[49] = {};
      for (uint16_t i=0;i<n;++i) snprintf(buf+i*3,sizeof(buf)-i*3,"%02X ",data[i]);
      ESP_LOGI(TAG, "AI10 test response data: %s",buf);
    }
    if (result == RES_SUCCESS && ((cmd==0xDB && n==1 && data[0]<=1) ||
        (cmd==CMD_UVC_READ && n==4))) {
      if (cmd==0xDB) ESP_LOGI(TAG,"Auto light: %s",data[0]?"ON":"OFF");
      else ESP_LOGI(TAG,"UVC: USB=0x%02X bitrate=%u quality=%u attributes=0x%02X",data[0],data[1],data[2],data[3]);
    }
    if (config_test_phase_ == 2 && result == RES_SUCCESS) {
      config_test_phase_ = 3;
      config_test_hold_started_ = command_started_ = millis(); command_timeout_ms_ = 3000;
      return;
    }
    finish_config_test_(cmd,result);
    return;
  }
  if (result == RES_SUCCESS && transfer_sink_ && (cmd==0x71 || cmd==0x72 || cmd==0x74 || cmd==0xFA)) {
    if (transfer_seq_ || n < (cmd==0x72?6:4)) { fail_command_(cmd,RES_FAILED_INVALID_PARAM); return; }
    transfer_size_=uint32_t(data[0])<<24 | uint32_t(data[1])<<16 | uint32_t(data[2])<<8 | data[3];
    if (!transfer_size_ || transfer_size_ > (cmd==0xFA?16384UL:255UL*1024)) {
      ESP_LOGW(TAG,"Transfer 0x%02X length %lu exceeds protocol bounds",cmd,(unsigned long)transfer_size_);
      fail_command_(cmd,RES_FAILED_INVALID_PARAM); return;
    }
    if (cmd==0x72) capture_uid_=uint16_t(data[4])<<8|data[5];
    request_chunk_(); return;
  }
  if (result == RES_SUCCESS && upload_active_ && (cmd==0xFB || cmd==0xF7)) {
    if (n!=4 || (uint16_t(data[0])<<8|data[1])!=transfer_seq_) { fail_command_(cmd,RES_FAILED_INVALID_PARAM); return; }
    uint16_t uid=uint16_t(data[2])<<8|data[3];
    if (transfer_received_==transfer_size_) {
      if (!uid || uid>2000) { fail_command_(cmd,RES_FAILED_INVALID_PARAM); return; }
      pending_cmd_=0; clear_transfer_(); last_uid_=uid; user_list_complete_=false; last_result_=result;
      schedule_directory_refresh_();
      enrollment_success_.call(uid); command_result_.call(cmd,result);
    } else {
      if (uid) { fail_command_(cmd,RES_FAILED_INVALID_PARAM); return; }
      request_chunk_();
    }
    return;
  }
  bool continuous = cmd == CMD_VERIFY && stored_continuous_;
  if (result != RES_SUCCESS) {
    if (continuous) {
      if (full_scan_) {
        // Repeat only normal negative recognition outcomes, not transport/hardware faults.
        if (result != RES_FAILED_UNKNOWN_USER && result != RES_FAILED_LIVE_CHECK &&
            result != RES_FAILED_TIMEOUT && result != 0x1A) {
          fail_command_(cmd,result); return;
        }
        schedule_repeat_();
      }
      invalidate_auth_(); last_result_ = result;
      recognition_failed_.call(result); command_result_.call(cmd, result);
    } else fail_command_(cmd, result);
    return;
  }
  auto uid_of = [](const uint8_t *p) -> uint16_t { return uint16_t(p[0]) << 8 | p[1]; };
  auto valid_uid = [](uint16_t uid) { return uid >= 1 && uid <= 2000; };
  auto bounded_string = [](const uint8_t *p, size_t size) {
    size_t length = 0; while (length < size && p[length]) ++length;
    return std::string(reinterpret_cast<const char *>(p), length);
  };
  bool valid = true;
  bool more = continuous;
  UserRecord user_record;
  switch (cmd) {
    case CMD_VERIFY:
      valid = n >= 35 && valid_uid(uid_of(data)) && data[34] <= 1;
      if (valid) {
        last_uid_ = uid_of(data); last_user_name_ = bounded_string(data+2,32);
        last_admin_ = data[34] != 0; last_type_ = last_uid_ > 1000 ? TYPE_PALM : TYPE_FACE;
        recognized_ = true; last_result_ = result;
      }
      break;
    case CMD_SCAN_QR_CODE:
      valid = n > 0;
      if (valid) { last_qr_data_.assign(reinterpret_cast<const char *>(data),n); last_type_ = TYPE_QR; }
      break;
    case CMD_ENROLL_SINGLE:
    case CMD_ENROLL_DIRECTION:
    case CMD_ENROLL_INTERACTIVE:
      valid = n >= 2 && valid_uid(uid_of(data));
      if (valid) { last_uid_ = uid_of(data); user_list_complete_ = false; schedule_directory_refresh_(); }
      break;
    case CMD_DEL_USER:
    case CMD_DEL_ALL_USER:
      user_list_complete_ = false;
      user_ids_.clear(); user_count_ = 0; schedule_directory_refresh_();
      break;
    case CMD_GET_ALL_USERID:
      valid = append_uid_page(data,n,user_ids_);
      if (valid) {
        user_count_ = user_ids_.size(); more = data[0] == 100;
        user_list_complete_ = !more;
        command_started_ = millis();
        if (more) {
          if (user_page_ >= 20) { fail_command_(cmd, RES_FAILED_INVALID_PARAM); return; }
          ++user_page_;
          send_extended_packet_(CMD_GET_ALL_USERID, {user_page_});
          return;
        }
      } else { user_ids_.clear(); user_count_ = 0; }
      break;
    case CMD_GET_USER_INFO:
      valid = decode_user_record(data,n,user_record) &&
        extended_data_.size() == 2 && user_record.uid == uid_of(extended_data_.data());
      break;
    case CMD_STATUS:
      valid = n == 1 && data[0] <= 3;
      if (valid) module_status_ = data[0];
      break;
    case CMD_VERSION:
      valid = n > 0;
      if (valid) firmware_version_ = bounded_string(data,n);
      break;
    case CMD_SERIAL:
      valid = n >= 8;
      if (valid) {
        serial_raw_.assign(data,data+n);
        serial_number_ = display_serial(data,n);
      }
      break;
    case CMD_UVC_READ:
      valid = n == 4;
      if (valid) ESP_LOGI(TAG, "UVC: USB=0x%02X bitrate=%u quality=%u attributes=0x%02X",data[0],data[1],data[2],data[3]);
      break;
    default: break;
  }
  if (!valid) { fail_command_(cmd, RES_FAILED_INVALID_PARAM); return; }
  last_result_ = result;
  if (!more) pending_cmd_ = 0;
  if (directory_active_ && cmd == CMD_GET_ALL_USERID && !more) {
    // Keep the directory operation active across individual requests.
    request_next_user_();
    users_received_.call(user_count_);
    command_result_.call(cmd,result);
    return;
  }
  if (directory_active_ && cmd == CMD_GET_USER_INFO) {
    staged_users_.push_back(user_record);
    ++directory_index_;
    // Emit the record before completion, so event consumers can assemble a full list.
    // Keep busy through callbacks and honor explicit cancellation.
    pending_cmd_ = CMD_GET_USER_INFO;
    user_info_.call(user_record.uid,user_record.name,user_record.admin);
    command_result_.call(cmd,result);
    if (directory_active_ && pending_cmd_ == CMD_GET_USER_INFO) request_next_user_();
    return;
  }
  if (cmd == CMD_VERIFY) {
    if (continuous && full_scan_) schedule_repeat_();
    on_recognized_callback_.call(last_user_name_,last_uid_);
  }
  else if (cmd == CMD_SCAN_QR_CODE) on_qr_scanned_callback_.call(last_qr_data_);
  else if (enrollment_command_(cmd)) enrollment_success_.call(last_uid_);
  else if (cmd == CMD_GET_USER_INFO) user_info_.call(user_record.uid,user_record.name,user_record.admin);
  else if (cmd == CMD_GET_ALL_USERID && !more) users_received_.call(user_count_);
  if (cmd != CMD_GET_ALL_USERID || !more) command_result_.call(cmd,result);
}

void DFRobotAI10Component::handle_note_(const uint8_t *payload, uint16_t len) {
  // Full note (sNoteData_t): nid(1) + state(2) + left(2) + top(2) + right(2) + bottom(2) + yaw(2) + pitch(2) + roll(2) = 17 bytes
  if (len == 1 && payload[0] == 0) {
    // A spontaneous READY means the previous operation is no longer valid.
    if (setup_done_ && (waiting_reset_ || pending_cmd_)) fail_command_(waiting_reset_?CMD_RESET:pending_cmd_,RES_ABORTED);
    module_status_=0; return;
  }
  if (waiting_reset_ || repeat_wait_ || (pending_cmd_ != CMD_VERIFY && !enrollment_command_(pending_cmd_))) return;
  if (pending_cmd_ == CMD_VERIFY && !suppress_verify_ && len >= 2 && len <= 251 && payload[0] == 0x0F) {
    // QR is an observation, never an authorization event.
    last_qr_data_.assign(reinterpret_cast<const char *>(payload+1),len-1);
    on_qr_scanned_callback_.call(last_qr_data_);
    return;
  }
  if (len >= 17 && payload[0]==1) {
    uint8_t nid = payload[0];
    int16_t state = (payload[1] << 8) | payload[2];
    int16_t left = (payload[3] << 8) | payload[4];
    int16_t top = (payload[5] << 8) | payload[6];
    int16_t right = (payload[7] << 8) | payload[8];
    int16_t bottom = (payload[9] << 8) | payload[10];

    const char *state_str = "unknown";
    if (state == 0x00) { state_str = "face"; this->face_detected_ = true; }
    else if (state == (int16_t)0x80) { state_str = "palm"; this->face_detected_ = true; }
    else { this->face_detected_ = false; }

    ESP_LOGD(TAG, "NOTE: nid=%d state=%s rect=[%d,%d,%d,%d]",
             nid, state_str, left, top, right, bottom);
  } else if (len >= 2) {
    // Short note — state transition updates (e.g. face exit)
    uint8_t note_type = payload[0];
    uint8_t note_val = payload[1];
    
    // 0x0A 0x00 = Face/Object left the frame
    if (note_type == 0x0A && note_val == 0x00) {
      this->face_detected_ = false;
      this->recognized_ = false; // Reset auth state when user leaves
    }
    ESP_LOGD(TAG, "NOTE: %d bytes (type=0x%02X val=0x%02X)", len, note_type, note_val);
  } else {
    ESP_LOGD(TAG, "NOTE: %d bytes", len);
    this->log_hex_("  ", payload, len);
  }
}

void DFRobotAI10Component::log_hex_(const char *prefix, const uint8_t *data, size_t len) {
  if (len == 0) return;
  std::string hex;
  hex.reserve(len * 3);
  for (size_t i = 0; i < len && i < 100; i++) {
    char tmp[4];
    sprintf(tmp, "%02X ", data[i]);
    hex += tmp;
  }
  if (len > 100) hex += "...";
  ESP_LOGD(TAG, "%s%s", prefix, hex.c_str());
}

const char *DFRobotAI10Component::result_to_str_(uint8_t result) {
  switch (result) {
    case RES_SUCCESS:              return "success";
    case RES_REJECTED:             return "rejected";
    case RES_ABORTED:              return "aborted";
    case RES_FAILED_CAMERA:        return "camera error";
    case RES_FAILED_UNKNOWN:       return "unknown error";
    case RES_FAILED_INVALID_PARAM: return "invalid parameter";
    case RES_FAILED_NO_MEMORY:     return "no memory";
    case RES_FAILED_UNKNOWN_USER:  return "unknown user";
    case RES_FAILED_MAX_USER:      return "max users reached";
    case RES_FAILED_FACE_ENROLLED: return "face already enrolled";
    case RES_FAILED_LIVE_CHECK:    return "liveness check failed";
    case RES_FAILED_TIMEOUT:       return "timeout";
    case 14: return "authorization failed";
    case 19: return "file read failed";
    case 20: return "file write failed";
    case 21: return "encryption required";
    case 23: return "RGB image unavailable";
    case 24: return "photo too large";
    case 25: return "photo too small";
    default:                       return "unknown";
  }
}

}  // namespace dfrobot_ai10
}  // namespace esphome