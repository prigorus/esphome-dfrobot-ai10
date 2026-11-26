#include "ai10_frame.h"
#include <cassert>
#include <vector>
using namespace esphome::dfrobot_ai10;
int main(){
 FrameParser p;
 auto feed=[&](const std::vector<uint8_t>&b){FrameParser::Result r=FrameParser::NONE; for(auto v:b)r=p.feed(v);return r;};
 assert(feed({0xEF,0xAA,0,0,2,0x10,0,0x12})==FrameParser::FRAME);
 assert(p.id()==0 && p.length()==2 && p.data()[0]==0x10);
 assert(feed({0xEF,0xAA,0,0,2,0x10,0,0x13})==FrameParser::BAD_XOR);
 assert(feed({0xEF,0xEF,0xAA,0xDB,0,0,0xDB})==FrameParser::FRAME);
 assert(feed({0xEF,0xAA,0,0x04,0x06})==FrameParser::TOO_LARGE);
 assert(feed({0xEF,0xAA,0,0,6,0xB0})==FrameParser::NONE);
 p.reset();
 assert(feed({0xEF,0xAA,0,0,6,0xB0,0,0xA0,0x0C,0x4B,0,0x51})==FrameParser::FRAME);
 assert(p.length()==6 && p.data()[4]==0x4B);
 std::vector<uint8_t> large{0xEF,0xAA,0x74,0x04,0x00};
 large.resize(5+1024,0xA5);large.push_back(frame_xor(large.data()+2,large.size()-2));
 assert(feed(large)==FrameParser::FRAME && p.length()==1024);
 assert(feed({0xEF,0xAA,0,0,0,0})==FrameParser::FRAME);
}
