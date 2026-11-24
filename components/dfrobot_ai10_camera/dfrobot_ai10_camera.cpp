#include "dfrobot_ai10_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <cstring>

namespace esphome {
namespace dfrobot_ai10_camera {
static const char *const TAG = "ai10_camera";

void AI10SnapshotCamera::setup() {
  parent_->add_on_command_result_callback([this](uint8_t cmd, uint8_t result) {
    if (!capturing_) return;
    if (cmd == 0x71) complete_(result);
    else if (cmd == 0x10 && result != 0) fail_("Reset failed");
  });
}
void AI10SnapshotCamera::dump_config() {
  ESP_LOGCONFIG(TAG,"AI10 manual UART camera, 240x320, max JPEG bytes: %u",unsigned(max_bytes_));
}
bool AI10SnapshotCamera::take_snapshot() {
  if (capturing_) return false;
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
  capturing_=true; capture_start_=millis(); status_="Capturing";
  if (!parent_->capture_image(0x71,0,[this](const uint8_t *data,size_t len) {
      return accept_chunk_(data,len);
    })) {
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
  requested_=true;
  ESP_LOGI(TAG,"JPEG ready: %u bytes",unsigned(received_));
}
void AI10SnapshotCamera::fail_(const char *reason) {
  // Copy before assigning, since reason may point into status_.
  std::string message(reason);
  capturing_=false; pending_.reset(); expected_=received_=0;
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
void AI10SnapshotCamera::loop() {
  uint32_t now=millis();
  if (capturing_ && !parent_->is_image_transfer_active()) fail_("Capture cancelled");
  if (capturing_ && now-capture_start_>60000) {
    parent_->send_reset();
    fail_("Capture deadline exceeded");
  }
  // Cached image only, including when HA opens a stream; no periodic camera capture.
  if (image_ && (requested_ || (streams_ && now-last_publish_>=1000))) {
    if (now-last_publish_<250) return;
    requested_=false; last_publish_=now;
    for (auto *listener:listeners_) listener->on_camera_image(image_);
  }
}

}
}
