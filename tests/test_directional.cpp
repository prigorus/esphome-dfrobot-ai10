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
  DirectionalEnrollment deferred;
  uint8_t interim[]={0xFF,0xFF,1};
  assert(deferred.accept(interim,3,false)==DirectionalEnrollment::NEXT);
  assert(deferred.uid==0 && deferred.mask==1 && deferred.step==1);
  interim[2]=1;assert(deferred.accept(interim,3,false)==DirectionalEnrollment::INVALID);
  interim[2]=5;assert(deferred.accept(interim,3,false)==DirectionalEnrollment::NEXT);
  interim[2]=7;assert(deferred.accept(interim,3,false)==DirectionalEnrollment::NEXT);
  interim[2]=23;assert(deferred.accept(interim,3,false)==DirectionalEnrollment::NEXT);
  interim[2]=31;assert(deferred.accept(interim,3,false)==DirectionalEnrollment::INVALID);
  assert(deferred.uid==0 && deferred.mask==23 && deferred.step==4);
  interim[0]=0;interim[1]=9;
  assert(deferred.accept(interim,3,true)==DirectionalEnrollment::INVALID);
  assert(deferred.accept(interim,3,false)==DirectionalEnrollment::COMPLETE);
  assert(deferred.uid==9 && deferred.mask==31);
  DirectionalEnrollment assigned;
  uint8_t early[]={0,9,1};
  assert(assigned.accept(early,3,false)==DirectionalEnrollment::NEXT);
  early[0]=0xFF;early[1]=0xFF;early[2]=5;
  assert(assigned.accept(early,3,false)==DirectionalEnrollment::INVALID);
  DirectionalEnrollment palm;uint8_t palm_reply[]={3,233,1};
  assert(palm.accept(palm_reply,3,false)==DirectionalEnrollment::INVALID);
}
