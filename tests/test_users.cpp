#include "ai10_users.h"
#include <cassert>
#include <array>
#include <cstring>
using namespace esphome::dfrobot_ai10;
int main() {
  // Synthetic records reproduce measured lengths, types and flags without identifying data.
  std::vector<uint16_t> ids;
  const uint8_t page[]{6,0,7,3,0xEF,0,8,0,9,3,0xF0,0,10};
  assert(append_uid_page(page,sizeof(page),ids));
  assert(ids.size()==6 && ids[1]==1007 && ids[4]==1008);
  assert(!append_uid_page(page,sizeof(page),ids) && ids.size()==6);
  const uint8_t bad[]{2,0,11,0,0};
  assert(!append_uid_page(bad,sizeof(bad),ids) && ids.size()==6);
  assert(!append_uid_page(page,sizeof(page)-1,ids));
  const uint8_t empty[]{0};
  assert(append_uid_page(empty,sizeof(empty),ids) && ids.size()==6);
  std::array<uint8_t,35> record{};
  record[0]=3; record[1]=0xEF; record[34]=1;
  const char *name="Example palm";
  memcpy(record.data()+2,name,strlen(name));
  UserRecord r;
  assert(decode_user_record(record.data(),record.size(),r));
  assert(r.uid==1007 && r.name==name && r.admin && r.is_palm());
  assert(!decode_user_record(record.data(),34,r));
  record[34]=2; assert(!decode_user_record(record.data(),35,r));
  record[0]=0;record[1]=7;record[34]=0;
  assert(decode_user_record(record.data(),35,r) && !r.admin && !r.is_palm());
  std::array<uint8_t,32> serial{};
  memcpy(serial.data(),"TEST123",7);serial[7]=0xFF;serial[8]=0xFF;
  assert(display_serial(serial.data(),serial.size())=="TEST123");
}
