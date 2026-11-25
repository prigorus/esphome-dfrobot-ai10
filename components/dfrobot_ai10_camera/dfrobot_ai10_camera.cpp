#include "dfrobot_ai10_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include <cstring>

namespace esphome {
namespace dfrobot_ai10_camera {
static const char *const TAG = "ai10_camera";

void AI10SnapshotCamera::setup() {
  parent_->add_on_recognized_callback([this](std::string, uint16_t) {
    failure_count_=0;
    if (phase_==QUEUED) phase_=IDLE;
  });
  parent_->add_on_recognition_failed_callback([this](uint8_t result) {
    if (!auto_enabled_ || !failure_threshold_ || phase_!=IDLE ||
        !parent_->is_continuous_recognition_active() || has_pending_archive()) return;
    if (result!=dfrobot_ai10::RES_FAILED_UNKNOWN_USER && result!=dfrobot_ai10::RES_FAILED_LIVE_CHECK) return;
    if (failure_count_<failure_threshold_) ++failure_count_;
    if (failure_count_>=failure_threshold_) set_phase_(QUEUED);
  });
  parent_->add_on_command_result_callback([this](uint8_t cmd, uint8_t result) {
    if (!capturing_) return;
    if (cmd == capture_cmd_) complete_(result);
    else if (cmd == 0x10 && result != 0) fail_("Reset failed");
  });
}
void AI10SnapshotCamera::dump_config() {
  ESP_LOGCONFIG(TAG,"AI10 UART still camera (240x320 / 320x320 face), max JPEG bytes: %u",unsigned(max_bytes_));
}
bool AI10SnapshotCamera::take_snapshot() { return take_snapshot_(0x71); }
bool AI10SnapshotCamera::take_face_snapshot() { return take_snapshot_(0x72); }
bool AI10SnapshotCamera::hold_for_archive(const std::string &token) {
  if (token.empty() || capturing_ || !image_ || !last_capture_succeeded_ || has_pending_archive()) return false;
  archive_token_=token;
  archive_attempted_=false; archive_in_flight_=false; archive_status_="Awaiting archive";
  return true;
}
bool AI10SnapshotCamera::acknowledge_archive(const std::string &token) {
  if (token.empty() || token!=archive_token_) return false;
  archive_token_.clear(); archive_in_flight_=false; archive_status_="Saved";
  return true;
}
bool AI10SnapshotCamera::take_snapshot_(uint8_t command) {
  if (capturing_) return false;
  last_capture_succeeded_=false;
  if (has_pending_archive()) {
    ESP_LOGW(TAG,"Previous archive still awaiting Home Assistant confirmation");
    return false;
  }
  if (!parent_->can_capture_snapshot()) {
    status_="Sensor busy: stop recognition first";
    ESP_LOGW(TAG,"%s",status_.c_str());
    return false;
  }
  if (image_ && image_.use_count()>1) {
    status_="Previous image still in use";
    ESP_LOGW(TAG,"%s",status_.c_str());
    return false;
  }
  // Release old JPEG before allocation; only one bounded JPEG buffer is held.
  image_.reset();
  pending_.reset();
  expected_=received_=0;
  capturing_=true; capture_cmd_=command; capture_start_=millis(); status_="Capturing";
  if (!parent_->capture_image(command,0,[this](const uint8_t *data,size_t len) {
      return accept_chunk_(data,len);
    }, command==0x72 ? 8000 : 10000)) {
    fail_("Capture rejected: enable allow_sensitive_transfers");
    return false;
  }
  return true;
}
bool AI10SnapshotCamera::accept_chunk_(const uint8_t *data,size_t len) {
  if (!capturing_) return false;
  if (!pending_) {
    expected_=parent_->get_transfer_size();
    if (expected_<4 || expected_>max_bytes_) {
      status_="Image exceeds configured size limit";
      return false;
    }
    if (len<2 || data[0]!=0xFF || data[1]!=0xD8) {
      status_="Not a JPEG: missing SOI";
      return false;
    }
    pending_.reset(static_cast<uint8_t *>(std::malloc(expected_)));
    if (!pending_) { status_="Not enough RAM for JPEG"; return false; }
  }
  if (len>expected_-received_) { status_="Image length overflow"; return false; }
  std::memcpy(pending_.get()+received_,data,len);
  received_+=len;
  return true;
}
void AI10SnapshotCamera::complete_(uint8_t result) {
  if (result!=0) {
    if (status_=="Capturing") status_="UART capture failed (result "+std::to_string(result)+")";
    fail_(status_.c_str());
    return;
  }
  if (!pending_ || received_!=expected_ || received_<4 ||
      pending_.get()[received_-2]!=0xFF || pending_.get()[received_-1]!=0xD9) {
    fail_("Incomplete or invalid JPEG (missing EOI)"); return;
  }
  image_=std::make_shared<SnapshotImage>(std::move(pending_),received_);
  capturing_=false;
  status_="Image ready";
  last_capture_succeeded_=true;
  requested_=true;
  ESP_LOGI(TAG,"JPEG ready: %u bytes",unsigned(received_));
}
void AI10SnapshotCamera::fail_(const char *reason) {
  // Copy before assigning, since reason may point into status_.
  std::string message(reason);
  capturing_=false; last_capture_succeeded_=false; pending_.reset(); expected_=received_=0;
  status_=std::move(message);
  ESP_LOGW(TAG,"%s",status_.c_str());
}
void AI10SnapshotCamera::request_image(camera::CameraRequester) { requested_=true; }
void AI10SnapshotCamera::start_stream(camera::CameraRequester requester) {
  bool was=streams_!=0;
  streams_|=uint8_t(1u<<unsigned(requester));
  requested_=true;
  if (!was) for (auto *l:listeners_) l->on_stream_start();
}
void AI10SnapshotCamera::stop_stream(camera::CameraRequester requester) {
  bool was=streams_!=0;
  streams_ &= uint8_t(~(1u<<unsigned(requester)));
  if (was && !streams_) for (auto *l:listeners_) l->on_stream_stop();
}
void AI10SnapshotCamera::set_phase_(Phase phase) {
  phase_=phase; phase_started_=millis();
}
void AI10SnapshotCamera::finish_auto_capture_() {
  if (last_capture_succeeded_) {
    char token[48];
    snprintf(token,sizeof(token),"%08lx_%08lx_%08lx",
      (unsigned long) random_uint32(), (unsigned long) random_uint32(), (unsigned long) ++archive_sequence_);
    if (!hold_for_archive(token)) ESP_LOGW(TAG,"Could not protect image for archive");
  }
  failure_count_=0;
  set_phase_(RESTART);
}
void AI10SnapshotCamera::auto_capture_loop_() {
  uint32_t elapsed=millis()-phase_started_;
  switch (phase_) {
    case IDLE: break;
    case QUEUED:
      if (!auto_enabled_ || has_pending_archive() || !parent_->is_continuous_recognition_active()) {
        cancel_auto_capture(); break;
      }
      resume_timeout_=parent_->get_recognition_timeout();
      resume_attempts_=parent_->get_recognition_max_attempts();
      parent_->stop_recognition();
      set_phase_(WAIT_STOP);
      break;
    case WAIT_STOP:
      if (parent_->can_capture_snapshot()) {
        if (take_face_snapshot()) set_phase_(FACE);
        else if (elapsed>6000) finish_auto_capture_();
      } else if (elapsed>6000) { last_capture_succeeded_=false; set_phase_(RESTART); }
      break;
    case FACE:
      if (!capturing_) {
        if (last_capture_succeeded_ || !fallback_snapshot_) finish_auto_capture_();
        else { parent_->send_reset(); set_phase_(WAIT_FALLBACK); }
      }
      break;
    case WAIT_FALLBACK:
      if (parent_->can_capture_snapshot()) {
        if (take_snapshot()) set_phase_(FULL);
        else if (elapsed>6000) finish_auto_capture_();
      } else if (elapsed>6000) finish_auto_capture_();
      break;
    case FULL:
      if (!capturing_) finish_auto_capture_();
      break;
    case RESTART:
      if (parent_->can_capture_snapshot()) {
        if (resume_attempts_<0) parent_->start_recognition(resume_timeout_,true);
        else parent_->start_recognition(resume_timeout_,true,uint8_t(resume_attempts_));
        set_phase_(WAIT_RESTART);
      } else if (elapsed>8000) {
        ESP_LOGE(TAG,"Cannot restart recognition: module still busy");
        set_phase_(IDLE);
      }
      break;
    case WAIT_RESTART:
      if (parent_->is_continuous_recognition_active()) set_phase_(IDLE);
      else if (elapsed>5000) {
        ESP_LOGE(TAG,"Recognition restart not acknowledged");
        set_phase_(IDLE);
      }
      break;
  }
}
void AI10SnapshotCamera::archive_result(const std::string &token, bool success) {
  if (token.empty() || token!=archive_token_) return;
  if (success) { acknowledge_archive(token); ESP_LOGI(TAG,"Archive saved"); }
  else { archive_in_flight_=false; archive_status_="Write failed; retry scheduled"; }
}
void AI10SnapshotCamera::archive_loop_() {
  if (!has_pending_archive()) return;
  uint32_t now=millis();
  if (archive_in_flight_) {
    if (now-archive_last_attempt_<std::max<uint32_t>(retry_ms_,120000)) return;
    archive_in_flight_=false;
    archive_status_="Write acknowledgement timed out";
  }
  if (archive_attempted_ && now-archive_last_attempt_<retry_ms_) return;
  archive_attempted_=true; archive_in_flight_=true; archive_last_attempt_=now;
  archive_status_="Saving to Home Assistant";
  archive_request_.call(archive_token_);
}
void AI10SnapshotCamera::loop() {
  uint32_t now=millis();
  if (capturing_ && !parent_->is_image_transfer_active()) fail_("Capture cancelled");
  if (capturing_ && now-capture_start_>60000) {
    parent_->send_reset();
    fail_("Capture deadline exceeded");
  }
  auto_capture_loop_();
  archive_loop_();
  // Cached image only, including when HA opens a stream; no periodic camera capture.
  if (image_ && (requested_ || (streams_ && now-last_publish_>=1000))) {
    if (now-last_publish_<250) return;
    requested_=false; last_publish_=now;
    for (auto *listener:listeners_) listener->on_camera_image(image_);
  }
}

}
}
