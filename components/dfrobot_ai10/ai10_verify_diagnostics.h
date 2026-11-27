#pragma once
#include <cstddef>
#include <cstdint>
namespace esphome {
namespace dfrobot_ai10 {
// NOTE 0x01 contains native little-endian signed 16-bit fields on AI10.
inline int16_t note_i16(const uint8_t *p) {
  const uint16_t value=uint16_t(p[0])|(uint16_t(p[1])<<8);
  return int16_t(value<0x8000?int32_t(value):int32_t(value)-65536);
}
struct FaceNote {
  int16_t state{0},left{0},top{0},right{0},bottom{0},yaw{0},pitch{0},roll{0};
  bool face_normal() const { return state==0; }
  bool palm_normal() const { return state==0x80; }
};
inline bool decode_face_note(const uint8_t *p,size_t n,FaceNote &out) {
  if(n!=17 || p[0]!=1) return false;
  out={note_i16(p+1),note_i16(p+3),note_i16(p+5),note_i16(p+7),note_i16(p+9),note_i16(p+11),note_i16(p+13),note_i16(p+15)};
  return true;
}
inline const char *face_note_state(int16_t state) {
  switch(state) {
    case 0: return "face normal"; case 1: return "no face";
    case 2: return "face too high"; case 3: return "face too low";
    case 4: return "face too far left"; case 5: return "face too far right";
    case 6: return "face too far"; case 7: return "face too close";
    case 8: return "eyebrow occlusion"; case 9: return "eye occlusion";
    case 10: return "face occlusion"; case 11: return "wrong direction";
    case 12: return "eyes open"; case 13: return "eyes closed";
    case 14: return "eye state unknown"; case 0x80: return "palm detected";
    default: return "unknown";
  }
}
// Keep the previous single-shot session closed while its late replies drain.
// Never infer that a late reply is a new transaction merely because it has the same echo.
struct VerifyReplyDrain {
  static constexpr uint32_t QUIET_MS=750;
  static constexpr uint32_t EXTRA_LIMIT_MS=5000;
  uint32_t started{0},last_reply{0},duplicates{0};
  void begin(uint32_t now) { started=last_reply=now;duplicates=0; }
  void observe(uint32_t now) { last_reply=now;if(duplicates<0xFFFFFFFFu)++duplicates; }
  bool ready(uint32_t now,uint32_t minimum_delay) const {
    return uint32_t(now-started)>=minimum_delay && uint32_t(now-last_reply)>=QUIET_MS;
  }
  bool expired(uint32_t now,uint32_t minimum_delay) const {
    return uint32_t(now-started)>=minimum_delay+EXTRA_LIMIT_MS;
  }
};
}
}
