#pragma once
#include "esphome/components/camera/camera.h"
#include "esphome/components/dfrobot_ai10/dfrobot_ai10.h"
#include <cstdlib>
#include <memory>
#include <vector>
#include <string>

namespace esphome {
namespace dfrobot_ai10_camera {

// JPEG ownership is shared with API readers; never overwrite an in-flight buffer.
struct FreeBytes { void operator()(uint8_t *ptr) const { std::free(ptr); } };
using Bytes = std::unique_ptr<uint8_t, FreeBytes>;

class SnapshotImage : public camera::CameraImage {
 public:
  SnapshotImage(Bytes data, size_t size) : data_(std::move(data)), size_(size) {}
  uint8_t *get_data_buffer() override { return data_.get(); }
  size_t get_data_length() override { return size_; }
  bool was_requested_by(camera::CameraRequester) const override { return true; }
 protected:
  Bytes data_;
  size_t size_;
};

class SnapshotReader : public camera::CameraImageReader {
 public:
  void set_image(std::shared_ptr<camera::CameraImage> image) override {
    image_ = std::move(image); offset_ = 0;
  }
  size_t available() const override { return image_ ? image_->get_data_length()-offset_ : 0; }
  uint8_t *peek_data_buffer() override { return image_ ? image_->get_data_buffer()+offset_ : nullptr; }
  void consume_data(size_t n) override { offset_ += std::min(n,available()); }
  void return_image() override { image_.reset(); offset_ = 0; }
 protected:
  std::shared_ptr<camera::CameraImage> image_;
  size_t offset_{0};
};

class AI10SnapshotCamera : public camera::Camera {
 public:
  void set_parent(dfrobot_ai10::DFRobotAI10Component *parent) { parent_=parent; }
  void set_max_image_bytes(size_t bytes) { max_bytes_=bytes; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void add_listener(camera::CameraListener *listener) override { listeners_.push_back(listener); }
  camera::CameraImageReader *create_image_reader() override { return new SnapshotReader(); }
  void request_image(camera::CameraRequester requester) override;
  void start_stream(camera::CameraRequester requester) override;
  void stop_stream(camera::CameraRequester requester) override;
  // Explicit shutter pauses/resumes continuous recognition; API requests only serve the cached JPEG.
  void set_failed_capture_threshold(uint8_t n) { failure_threshold_=n; }
  void set_auto_capture_enabled(bool enabled) { auto_enabled_=enabled; if (!enabled) failure_count_=0; }
  bool is_auto_capture_enabled() const { return auto_enabled_; }
  uint8_t get_failed_count() const { return failure_count_; }
  bool is_auto_capture_active() const { return phase_!=IDLE; }
  void cancel_auto_capture() { phase_=IDLE; failure_count_=0; }
  void set_fallback_snapshot(bool enabled) { fallback_snapshot_=enabled; }
  void set_archive_retry_interval(uint32_t ms) { retry_ms_=ms; }
  void add_on_archive_request_callback(std::function<void(std::string)> &&cb) { archive_request_.add(std::move(cb)); }
  void archive_result(const std::string &token, bool success);
  const std::string &get_archive_status() const { return archive_status_; }
  bool take_snapshot();
  bool take_face_snapshot();
  bool has_pending_archive() const { return !archive_token_.empty(); }
  const std::string &get_archive_token() const { return archive_token_; }
  bool hold_for_archive(const std::string &token);
  bool acknowledge_archive(const std::string &token);
  bool last_capture_succeeded() const { return last_capture_succeeded_; }
  bool is_capturing() const { return capturing_; }
  size_t get_image_size() const { return image_ ? image_->get_data_length() : 0; }
  const std::string &get_capture_status() const { return status_; }
 protected:
  dfrobot_ai10::DFRobotAI10Component *parent_{nullptr};
  size_t max_bytes_{49152}, expected_{0}, received_{0};
  Bytes pending_;
  std::shared_ptr<SnapshotImage> image_;
  std::vector<camera::CameraListener *> listeners_;
  bool capturing_{false}, requested_{false}, last_capture_succeeded_{false};
  enum Phase : uint8_t { IDLE, QUEUED, WAIT_STOP, FACE, WAIT_FALLBACK, FULL, RESTART, WAIT_RESTART };
  Phase phase_{IDLE};
  uint8_t failure_threshold_{3}, failure_count_{0}, resume_timeout_{30};
  int16_t resume_attempts_{-1};
  bool auto_enabled_{false}, fallback_snapshot_{true}, archive_in_flight_{false}, archive_attempted_{false};
  bool resume_recognition_{false}, archive_capture_{false};
  uint8_t requested_capture_cmd_{0x71}, restart_attempts_{0};
  bool request_manual_snapshot_(uint8_t command);
  void begin_capture_cycle_(uint8_t command, bool archive, bool resume);
  uint32_t phase_started_{0}, retry_ms_{60000}, archive_last_attempt_{0};
  uint32_t archive_sequence_{0};
  std::string archive_status_{"No pending image"};
  CallbackManager<void(std::string)> archive_request_;
  void set_phase_(Phase phase);
  void auto_capture_loop_();
  void archive_loop_();
  void finish_auto_capture_();
  uint8_t capture_cmd_{0x71};
  std::string archive_token_;
  bool take_snapshot_(uint8_t command);
  uint8_t streams_{0};
  uint32_t last_publish_{0}, capture_start_{0};
  std::string status_{"No image"};
  bool accept_chunk_(const uint8_t *data, size_t len);
  void complete_(uint8_t result);
  void fail_(const char *reason);
};

class ArchiveRequestTrigger : public Trigger<std::string> {
 public:
  explicit ArchiveRequestTrigger(AI10SnapshotCamera *parent) {
    parent->add_on_archive_request_callback([this](std::string token) { this->trigger(token); });
  }
};

}
}
