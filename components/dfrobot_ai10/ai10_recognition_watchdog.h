#pragma once
#include <cstdint>
namespace esphome {
namespace dfrobot_ai10 {
struct RecognitionIntent {
  bool wanted{false};
  uint8_t timeout{30};
  int16_t attempts{-1};
  bool arm(uint8_t seconds,int attempts_limit) {
    if(!seconds || attempts_limit < -1 || attempts_limit > 255) return false;
    timeout=seconds; attempts=attempts_limit; wanted=true; return true;
  }
  void stop() { wanted=false; }
  bool may_recover(bool enabled,bool ready,bool suspended,bool occupied) const {
    return wanted && enabled && ready && !suspended && !occupied;
  }
};
constexpr uint32_t recovery_delay_ms(uint8_t failures) {
  return failures<3?1500u:failures==3?60000u:failures==4?120000u:failures==5?240000u:300000u;
}
constexpr uint8_t next_recovery_count(uint8_t count) {return count<6?count+1:6;}
constexpr bool elapsed_at_least(uint32_t now,uint32_t since,uint32_t interval) {
  return uint32_t(now-since)>=interval;
}
}
}
