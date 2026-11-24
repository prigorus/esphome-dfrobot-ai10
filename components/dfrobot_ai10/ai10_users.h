#pragma once
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <algorithm>

namespace esphome {
namespace dfrobot_ai10 {
struct UserRecord {
  uint16_t uid{0};
  std::string name;
  bool admin{false};
  bool is_palm() const { return uid > 1000; }
};
inline uint16_t read_uid(const uint8_t *p) { return (uint16_t(p[0]) << 8) | p[1]; }
inline bool valid_user_uid(uint16_t uid) { return uid >= 1 && uid <= 2000; }
inline bool decode_user_record(const uint8_t *p, size_t n, UserRecord &record) {
  if (n != 35 || !valid_user_uid(read_uid(p)) || p[34] > 1) return false;
  size_t len=0;
  while (len < 32 && p[2+len] != 0) ++len;
  record.uid=read_uid(p);
  record.name.assign(reinterpret_cast<const char *>(p+2),len);
  record.admin=p[34]!=0;
  return true;
}
// Validate a complete page before modifying the caller's list.
inline bool append_uid_page(const uint8_t *p, size_t n, std::vector<uint16_t> &ids) {
  if (!n || p[0] > 100 || n != 1u+2u*p[0] || ids.size()+p[0]>2000) return false;
  std::vector<uint16_t> page;
  for (size_t i=0; i<p[0]; ++i) {
    uint16_t uid=read_uid(p+1+2*i);
    if (!valid_user_uid(uid) || std::find(ids.begin(),ids.end(),uid)!=ids.end() ||
        std::find(page.begin(),page.end(),uid)!=page.end()) return false;
    page.push_back(uid);
  }
  ids.insert(ids.end(),page.begin(),page.end());
  return true;
}
inline std::string display_serial(const uint8_t *p, size_t n) {
  std::string s;
  for (size_t i=0;i<std::min<size_t>(8,n);++i) {
    if (p[i]==0 || p[i]==0xFF) break;
    if (p[i]<0x20 || p[i]>0x7E) break;
    s.push_back(char(p[i]));
  }
  return s;
}
}
}
