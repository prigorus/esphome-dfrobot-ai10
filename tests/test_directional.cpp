#include "ai10_directional_enrollment.h"
#include <cassert>
using namespace esphome::dfrobot_ai10;
int main() {
  DirectionalEnrollment e;
  uint8_t p[]={0,7,1};
  assert(e.accept(p,2,false)==DirectionalEnrollment::INVALID);
  assert(e.accept(p,3,true)==DirectionalEnrollment::INVALID);
  assert(e.accept(p,3,false)==DirectionalEnrollment::NEXT);
  assert(e.uid==7 && e.step==1 && e.mask==1);
  p[1]=8;p[2]=5;
  assert(e.accept(p,3,false)==DirectionalEnrollment::INVALID);
  p[1]=7;p[2]=4;
  assert(e.accept(p,3,false)==DirectionalEnrollment::INVALID);
  p[2]=5;assert(e.accept(p,3,false)==DirectionalEnrollment::NEXT);
  p[2]=7;assert(e.accept(p,3,false)==DirectionalEnrollment::NEXT);
  p[2]=23;assert(e.accept(p,3,false)==DirectionalEnrollment::NEXT);
  p[2]=31;assert(e.accept(p,3,false)==DirectionalEnrollment::COMPLETE);
  assert(e.step==5 && e.mask==31);
  assert(e.accept(p,3,false)==DirectionalEnrollment::INVALID);
  DirectionalEnrollment palm;uint8_t palm_reply[]={3,233,1};
  assert(palm.accept(palm_reply,3,false)==DirectionalEnrollment::INVALID);
}
