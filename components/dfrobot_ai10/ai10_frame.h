#pragma once
#include <cstddef>
#include <cstdint>
namespace esphome {
namespace dfrobot_ai10 {
constexpr uint8_t frame_xor(const uint8_t *data, size_t len) {
  uint8_t x=0; for (size_t i=0;i<len;++i) x^=data[i]; return x;
}
class FrameParser {
 public:
  enum Result : uint8_t { NONE, FRAME, BAD_XOR, TOO_LARGE };
  static constexpr size_t MAX_PAYLOAD=1029;
  void reset() { state_=0; pos_=0; }
  bool idle() const { return state_==0; }
  uint8_t id() const { return header_[0]; }
  uint16_t length() const { return length_; }
  const uint8_t *data() const { return payload_; }
  Result feed(uint8_t byte) {
    switch(state_) {
      case 0: if(byte==0xEF) state_=1; break;
      case 1: if(byte==0xAA) {state_=2;pos_=0;} else if(byte!=0xEF) reset(); break;
      case 2:
        header_[pos_++]=byte;
        if(pos_==3) {
          length_=uint16_t(header_[1])<<8|header_[2];
          if(length_>MAX_PAYLOAD) {reset();return TOO_LARGE;}
          pos_=0;state_=3;
        }
        break;
      case 3:
        if(pos_<length_) {payload_[pos_++]=byte;break;}
        {uint8_t expected=frame_xor(header_,3)^frame_xor(payload_,length_);
         reset();return expected==byte?FRAME:BAD_XOR;}
      default: reset(); break;
    }
    return NONE;
  }
 private:
  uint8_t state_{0},header_[3]{},payload_[MAX_PAYLOAD]{};
  uint16_t pos_{0},length_{0};
};
}
}
