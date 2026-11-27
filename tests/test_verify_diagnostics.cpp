#include "ai10_verify_diagnostics.h"
#include <cassert>
#include <cstring>
using namespace esphome::dfrobot_ai10;
int main() {
  uint8_t data[]={1,1,0,0xFC,0,0x76,1,0xB2,1,0x78,2,0,0,0,0,0,0};
  FaceNote note;
  assert(decode_face_note(data,sizeof(data),note));
  assert(note.state==1 && !note.face_normal() && !note.palm_normal());
  assert(std::strcmp(face_note_state(note.state),"no face")==0);
  assert(note.left==252 && note.top==374 && note.right==434 && note.bottom==632);
  data[1]=0x80;assert(decode_face_note(data,sizeof(data),note)&&note.palm_normal());
  data[1]=0;data[3]=0xFE;data[4]=0xFF;
  assert(decode_face_note(data,sizeof(data),note)&&note.face_normal()&&note.left==-2);
  assert(!decode_face_note(data,16,note));
  VerifyReplyDrain d;d.begin(1000);
  assert(!d.ready(1999,1000));assert(d.ready(2000,1000));
  d.observe(1900);d.observe(2200);
  assert(!d.ready(2900,1000));assert(d.ready(2950,1000));
  assert(d.duplicates==2);
  assert(!d.expired(6999,1000));assert(d.expired(7000,1000));
  d.begin(0xFFFFFF00u);d.observe(0xFFFFFFF0u);
  assert(!d.ready(500,1000));assert(d.ready(1000,1000));
  assert(d.duplicates==1);
}
