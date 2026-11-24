#pragma once

/*
 * DFRobot SEN0677 AI Binocular Vision Sensor - ESPHome External Component
 *
 * Protocol reverse-engineered from DFRobot_AI10 Arduino library (MIT License)
 * https://github.com/DFRobot/DFRobot_AI10
 */

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/log.h"
#include "esphome/core/automation.h"

#include <vector>
#include <string>
#include <cstring>
#include <algorithm>
#include <functional>
#include "ai10_users.h"

namespace esphome {
namespace dfrobot_ai10 {

// ===== Protocol Constants =====
static const uint8_t SYNC_H = 0xEF;
static const uint8_t SYNC_L = 0xAA;

// Message IDs (response)
static const uint8_t MID_REPLY = 0x00;  // Reply to a command
static const uint8_t MID_NOTE  = 0x01;  // Notification (face position, state)

// Command IDs
static const uint8_t CMD_STATUS = 0x11;
static const uint8_t CMD_ENROLL_DIRECTION = 0x13;
static const uint8_t CMD_FACE_RESET = 0x23;
static const uint8_t CMD_ENROLL_INTERACTIVE = 0x26;
static const uint8_t CMD_VERSION = 0x30;
static const uint8_t CMD_SERIAL = 0x93;
static const uint8_t CMD_UVC_READ = 0xB0;
static const uint8_t CMD_UVC_WRITE = 0xB1;
static const uint8_t CMD_RESET                = 0x10;
static const uint8_t CMD_VERIFY               = 0x12;
static const uint8_t CMD_ENROLL_SINGLE        = 0x1D;
static const uint8_t CMD_DEL_USER             = 0x20;
static const uint8_t CMD_DEL_ALL_USER         = 0x21;
static const uint8_t CMD_GET_USER_INFO        = 0x22;
static const uint8_t CMD_GET_ALL_USERID       = 0x24;
static const uint8_t CMD_SCAN_QR_CODE         = 0x70;
static const uint8_t CMD_SET_FACE_DISPLAY     = 0xB5;

// Result codes
static const uint8_t RES_SUCCESS              = 0x00;
static const uint8_t RES_REJECTED             = 0x01;
static const uint8_t RES_ABORTED              = 0x02;
static const uint8_t RES_FAILED_CAMERA        = 0x04;
static const uint8_t RES_FAILED_UNKNOWN       = 0x05;
static const uint8_t RES_FAILED_INVALID_PARAM = 0x06;
static const uint8_t RES_FAILED_NO_MEMORY     = 0x07;
static const uint8_t RES_FAILED_UNKNOWN_USER  = 0x08;
static const uint8_t RES_FAILED_MAX_USER      = 0x09;
static const uint8_t RES_FAILED_FACE_ENROLLED = 0x0A;
static const uint8_t RES_FAILED_LIVE_CHECK    = 0x0C;
static const uint8_t RES_FAILED_TIMEOUT       = 0x0D;

// Enroll data structure (must match Arduino struct layout: 35 bytes)
struct __attribute__((packed)) EnrollData {
  uint8_t admin;
  uint8_t user_name[32];
  uint8_t face_dir;
  uint8_t timeout;
};

// Recognition types
enum RecognitionType : uint8_t {
  TYPE_NONE = 0,
  TYPE_FACE = 1,
  TYPE_PALM = 2,
  TYPE_QR   = 3,
};

// Parser states
enum ParserState : uint8_t {
  STATE_WAIT_SYNC_H,
  STATE_WAIT_SYNC_L,
  STATE_READ_HEADER,
  STATE_READ_PAYLOAD,
};

// State Machine for Pending Actions (Non-blocking logic)
enum PendingAction : uint8_t {
  PENDING_NONE,
  PENDING_VERIFY,
  PENDING_ENROLL,
  PENDING_DELETE_USER,
  PENDING_DELETE_ALL,
  PENDING_GET_USERS,
  PENDING_SCAN_QR,
  PENDING_EXTENDED
};

class DFRobotAI10Component : public Component, public uart::UARTDevice {
 public:
  // Register callbacks for triggers
  void add_on_recognized_callback(std::function<void(std::string, uint16_t)> &&callback) {
    this->on_recognized_callback_.add(std::move(callback));
  }

  void add_on_qr_scanned_callback(std::function<void(std::string)> &&callback) {
    this->on_qr_scanned_callback_.add(std::move(callback));
  }

  float get_setup_priority() const override { return setup_priority::DATA; }
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Public API — call these from HA services or buttons
  void enroll_user(uint8_t admin, const char *name, uint8_t timeout);
  void start_recognition(uint8_t timeout, bool continuous);
  void delete_user(uint16_t uid);
  void delete_all_users();
  void get_all_user_ids();
  void refresh_users();
  void set_auto_refresh_users(bool value) { auto_refresh_users_ = value; }
  void set_resume_recognition_after_refresh(bool value) { resume_recognition_after_refresh_ = value; }
  const std::vector<UserRecord> &get_users() const { return users_; }
  bool is_user_directory_complete() const { return directory_complete_; }
  std::string get_user_summary() const;
  const std::vector<uint8_t> &get_serial_raw() const { return serial_raw_; }
  void add_on_user_directory_callback(std::function<void(uint16_t)> &&cb) { user_directory_.add(std::move(cb)); }
  void add_on_user_directory_failed_callback(std::function<void(uint8_t)> &&cb) { user_directory_failed_.add(std::move(cb)); }
  void scan_qr_code(uint8_t timeout);
  void send_reset();
  void stop_recognition();
  void start_recognition(uint8_t timeout, bool continuous, uint8_t max_attempts);
  void query_status();
  void query_version();
  void query_serial();
  void query_user(uint16_t uid);
  void clear_enrollment();
  void enroll_direction(uint8_t admin, const char *name, uint8_t direction, uint8_t timeout);
  void enroll_interactive(uint8_t admin, const char *name, uint8_t type, uint8_t duplicate, uint8_t timeout);
  void query_uvc();
  // Explicit opt-in, streaming to caller-owned storage; never expose biometric bytes to HA logs.
  using ChunkSink = std::function<bool(const uint8_t *, size_t)>;
  void set_allow_sensitive_transfers(bool value) { allow_sensitive_transfers_ = value; }
  bool capture_image(uint8_t kind, uint8_t resolution, ChunkSink sink);
  bool can_capture_snapshot() const { return setup_done_ && !is_busy() && !stop_in_progress_ && !refresh_queued_; }
  bool is_image_transfer_active() const { return bool(transfer_sink_); }
  void export_template(uint16_t uid, uint8_t type, ChunkSink sink);
  void import_template(uint8_t type, const std::string &name, std::vector<uint8_t> bytes);
  void enroll_photo(uint8_t bio_type, const std::string &name, std::vector<uint8_t> bytes);
  void set_duplicate_check(bool enabled);
  uint32_t get_transfer_size() const { return transfer_size_; }
  uint32_t get_transfer_received() const { return transfer_received_; }
  uint16_t get_capture_uid() const { return capture_uid_; }
  void configure_uvc(uint8_t usb, uint8_t bitrate, uint8_t quality, uint8_t attributes);
  void set_documented_qr(bool value) { documented_qr_ = value; }
  uint8_t get_module_status() const { return module_status_; }
  const std::string &get_firmware_version() const { return firmware_version_; }
  const std::string &get_serial_number() const { return serial_number_; }
  const std::vector<uint16_t> &get_user_ids() const { return user_ids_; }
  bool is_user_list_complete() const { return user_list_complete_; }
  bool get_last_admin() const { return last_admin_; }
  bool is_busy() const { return waiting_reset_ || pending_cmd_ != 0; }
  void add_on_enrollment_success_callback(std::function<void(uint16_t)> &&cb) { enrollment_success_.add(std::move(cb)); }
  void add_on_enrollment_failed_callback(std::function<void(uint8_t)> &&cb) { enrollment_failed_.add(std::move(cb)); }
  void add_on_recognition_failed_callback(std::function<void(uint8_t)> &&cb) { recognition_failed_.add(std::move(cb)); }
  void add_on_command_result_callback(std::function<void(uint8_t, uint8_t)> &&cb) { command_result_.add(std::move(cb)); }
  void add_on_user_info_callback(std::function<void(uint16_t, std::string, bool)> &&cb) { user_info_.add(std::move(cb)); }
  void add_on_users_received_callback(std::function<void(uint16_t)> &&cb) { users_received_.add(std::move(cb)); }

  // Getters for template sensors
  uint16_t get_last_uid() const { return this->last_uid_; }
  const std::string &get_last_user_name() const { return this->last_user_name_; }
  RecognitionType get_last_type() const { return this->last_type_; }
  uint8_t get_last_result() const { return this->last_result_; }
  const std::string &get_last_qr_data() const { return this->last_qr_data_; }
  bool is_recognized() const { return this->recognized_; }
  bool is_face_detected() const { return this->face_detected_; }
  uint16_t get_user_count() const { return this->user_count_; }

 protected:
  // CallbackManagers
  CallbackManager<void(std::string, uint16_t)> on_recognized_callback_;
  CallbackManager<void(std::string)> on_qr_scanned_callback_;

  // Protocol helpers
  void send_packet_(const uint8_t *data, size_t len);
  static uint8_t xor_checksum_(const uint8_t *data, size_t len);
  void process_packet_(uint8_t msg_id, const uint8_t *payload, uint16_t len);
  void handle_reply_(const uint8_t *payload, uint16_t len);
  void handle_note_(const uint8_t *payload, uint16_t len);
  void log_hex_(const char *prefix, const uint8_t *data, size_t len);
  static const char *result_to_str_(uint8_t result);

  // Helper for State Machine
  void execute_pending_action_();

  // Parser vars
  ParserState parser_state_{STATE_WAIT_SYNC_H};
  uint8_t header_buf_[3];
  uint8_t header_pos_{0};
  uint8_t payload_buf_[1030];
  uint16_t payload_len_{0};
  uint16_t payload_pos_{0};

  // State vars
  uint16_t last_uid_{0};
  std::string last_user_name_;
  RecognitionType last_type_{TYPE_NONE};
  uint8_t last_result_{0xFF};
  std::string last_qr_data_;
  bool recognized_{false};
  bool face_detected_{false};
  uint16_t user_count_{0};
  std::vector<uint16_t> user_ids_;
  bool user_list_complete_{false};
  std::vector<UserRecord> users_, staged_users_;
  std::vector<uint8_t> serial_raw_;
  bool directory_complete_{false}, directory_active_{false}, refresh_queued_{false};
  bool auto_refresh_users_{false}, resume_recognition_after_refresh_{false};
  bool resume_pending_{false};
  uint8_t resume_timeout_{0};
  int16_t resume_attempts_{-1};
  size_t directory_index_{0};
  CallbackManager<void(uint16_t)> user_directory_;
  CallbackManager<void(uint8_t)> user_directory_failed_;
  void remember_recognition_();
  void invalidate_directory_();
  void schedule_directory_refresh_();
  void request_next_user_();
  void finish_directory_();
  void cancel_directory_();
  bool last_admin_{false};
  bool documented_qr_{false};
  uint8_t module_status_{0xFF};
  std::string firmware_version_, serial_number_;
  uint8_t extended_cmd_{0};
  std::vector<uint8_t> extended_data_;
  uint32_t extended_timeout_ms_{3000};
  uint32_t command_started_{0}, command_timeout_ms_{3000}, last_byte_{0};
  bool waiting_reset_{false}, suppress_verify_{true}, stop_in_progress_{false};
  int16_t max_attempts_{-1};
  CallbackManager<void(uint16_t)> enrollment_success_, users_received_;
  CallbackManager<void(uint8_t)> enrollment_failed_, recognition_failed_;
  CallbackManager<void(uint8_t, uint8_t)> command_result_;
  CallbackManager<void(uint16_t, std::string, bool)> user_info_;
  bool allow_sensitive_transfers_{false};
  ChunkSink transfer_sink_;
  uint32_t transfer_size_{0}, transfer_received_{0};
  uint16_t transfer_seq_{0}, transfer_uid_{0}, capture_uid_{0};
  uint8_t transfer_type_{0};
  std::vector<uint8_t> upload_bytes_;
  bool upload_active_{false};
  void clear_transfer_();
  void request_chunk_();
  void handle_chunk_(uint8_t cmd, const uint8_t *data, uint16_t len);
  void send_extended_packet_(uint8_t cmd, const std::vector<uint8_t> &data);
  void queue_extended_(uint8_t cmd, std::vector<uint8_t> data, uint32_t timeout_ms = 3000);
  void transmit_reset_();
  void invalidate_auth_();
  void fail_command_(uint8_t cmd, uint8_t result);
  void cancel_stop_();
  bool prepare_(PendingAction action);
  static bool enrollment_command_(uint8_t cmd);

  // State Machine vars
  PendingAction pending_action_{PENDING_NONE};
  uint8_t pending_cmd_{0}; // To track current active command
  
  // Stored parameters for pending actions
  uint8_t stored_timeout_{0};
  bool stored_continuous_{false};
  uint8_t stored_admin_{0};
  std::string stored_name_;
  uint16_t stored_uid_{0};
  
  uint32_t last_status_log_{0};
  bool setup_done_{false};
};

// Trigger Classes for ESPHome Automation
class RecognizedTrigger : public Trigger<std::string, uint16_t> {
 public:
  explicit RecognizedTrigger(DFRobotAI10Component *parent) {
    parent->add_on_recognized_callback([this](std::string name, uint16_t uid) {
      this->trigger(name, uid);
    });
  }
};

class QRScannedTrigger : public Trigger<std::string> {
 public:
  explicit QRScannedTrigger(DFRobotAI10Component *parent) {
    parent->add_on_qr_scanned_callback([this](std::string data) {
      this->trigger(data);
    });
  }
};

class EnrollmentSuccessTrigger : public Trigger<uint16_t> {
 public:
  explicit EnrollmentSuccessTrigger(DFRobotAI10Component *parent) {
    parent->add_on_enrollment_success_callback([this](uint16_t uid) { this->trigger(uid); });
  }
};

class EnrollmentFailedTrigger : public Trigger<uint8_t> {
 public:
  explicit EnrollmentFailedTrigger(DFRobotAI10Component *parent) {
    parent->add_on_enrollment_failed_callback([this](uint8_t result) { this->trigger(result); });
  }
};

class RecognitionFailedTrigger : public Trigger<uint8_t> {
 public:
  explicit RecognitionFailedTrigger(DFRobotAI10Component *parent) {
    parent->add_on_recognition_failed_callback([this](uint8_t result) { this->trigger(result); });
  }
};

class CommandResultTrigger : public Trigger<uint8_t, uint8_t> {
 public:
  explicit CommandResultTrigger(DFRobotAI10Component *parent) {
    parent->add_on_command_result_callback([this](uint8_t command, uint8_t result) { this->trigger(command, result); });
  }
};

class UserInfoTrigger : public Trigger<uint16_t, std::string, bool> {
 public:
  explicit UserInfoTrigger(DFRobotAI10Component *parent) {
    parent->add_on_user_info_callback([this](uint16_t uid, std::string name, bool admin) { this->trigger(uid, name, admin); });
  }
};

class UsersReceivedTrigger : public Trigger<uint16_t> {
 public:
  explicit UsersReceivedTrigger(DFRobotAI10Component *parent) {
    parent->add_on_users_received_callback([this](uint16_t count) { this->trigger(count); });
  }
};

class UserDirectoryTrigger : public Trigger<uint16_t> {
 public:
  explicit UserDirectoryTrigger(DFRobotAI10Component *parent) {
    parent->add_on_user_directory_callback([this](uint16_t count) { this->trigger(count); });
  }
};
class UserDirectoryFailedTrigger : public Trigger<uint8_t> {
 public:
  explicit UserDirectoryFailedTrigger(DFRobotAI10Component *parent) {
    parent->add_on_user_directory_failed_callback([this](uint8_t result) { this->trigger(result); });
  }
};

}  // namespace dfrobot_ai10
}  // namespace esphome