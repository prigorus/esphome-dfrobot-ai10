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
  // Manual shutter only. API requests return cached JPEG, never interrupt recognition.
  bool take_snapshot();
  bool is_capturing() const { return capturing_; }
  size_t get_image_size() const { return image_ ? image_->get_data_length() : 0; }
  const std::string &get_capture_status() const { return status_; }
 protected:
  dfrobot_ai10::DFRobotAI10Component *parent_{nullptr};
  size_t max_bytes_{49152}, expected_{0}, received_{0};
  Bytes pending_;
  std::shared_ptr<SnapshotImage> image_;
  std::vector<camera::CameraListener *> listeners_;
  bool capturing_{false}, requested_{false};
  uint8_t streams_{0};
  uint32_t last_publish_{0}, capture_start_{0};
  std::string status_{"No image"};
  bool accept_chunk_(const uint8_t *data, size_t len);
  void complete_(uint8_t result);
  void fail_(const char *reason);
};

}
}
