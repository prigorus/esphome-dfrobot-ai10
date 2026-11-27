#include "ai10_recognition_watchdog.h"
#include <cassert>
using namespace esphome::dfrobot_ai10;
int main() {
  RecognitionIntent r;
  assert(!r.may_recover(true,true,false,false));
  assert(!r.arm(0,-1));
  assert(!r.arm(30,256));
  assert(r.arm(30,-1));
  assert(r.may_recover(true,true,false,false));
  assert(!r.may_recover(false,true,false,false));
  assert(!r.may_recover(true,false,false,false));
  assert(!r.may_recover(true,true,true,false));
  assert(!r.may_recover(true,true,false,true));
  assert(r.timeout==30 && r.attempts==-1);
  assert(r.arm(45,10) && r.attempts==10 && r.timeout==45);
  r.stop();assert(!r.may_recover(true,true,false,false));
  uint8_t count=0;
  for(unsigned i=0;i<30;++i) {
    const uint32_t expected=i<3?1500:i==3?60000:i==4?120000:i==5?240000:300000;
    assert(recovery_delay_ms(count)==expected);
    count=next_recovery_count(count);
  }
  assert(count==6);
  assert(elapsed_at_least(500,0xFFFFFF00u,756));
  assert(!elapsed_at_least(500,0xFFFFFF00u,757));
}
