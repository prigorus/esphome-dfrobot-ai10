#pragma once
#include <cstddef>
#include <cstdint>

namespace esphome {
namespace dfrobot_ai10 {
// V2.67: MID_ENROLL reply is UID (big endian) and accumulated face-direction mask.
// Deliberately do not infer missing directions from a successful result alone.
struct DirectionalEnrollment {
  static constexpr uint8_t ALL=0x1F;
  static constexpr uint8_t direction(uint8_t step) {
    return step==0?0x01:step==1?0x04:step==2?0x02:step==3?0x10:step==4?0x08:0;
  }
  uint16_t uid{0};
  uint8_t mask{0}, step{0};
  enum Outcome : uint8_t { INVALID, NEXT, COMPLETE };
  Outcome accept(const uint8_t *data,size_t n,bool existing_uid) {
    if(n!=3 || step>=5) return INVALID;
    const uint16_t received=uint16_t(data[0])<<8|data[1];
    const uint8_t expected=mask|direction(step);
    if(data[2]!=expected) return INVALID;
    // Hardware v2.30-QRXF-PV returned FFFF with a successful first direction.
    // Treat it only as a provisional UID, never as a registered identity.
    if(received==0xFFFF) {
      if(expected==ALL || uid || existing_uid) return INVALID;
    } else {
      if(!received || received>1000 || existing_uid || (uid && received!=uid)) return INVALID;
      uid=received;
    }
    mask=expected;++step;
    return mask==ALL?COMPLETE:NEXT;
  }
};
}
}
