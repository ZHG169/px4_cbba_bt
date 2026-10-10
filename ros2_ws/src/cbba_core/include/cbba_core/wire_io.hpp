// 機間通訊封包的位元組讀寫：網路位元組順序（大端序）、不補位，逐欄位寫入，不直接傳 struct 的記憶體
//
// 協定版本 2（2026-10-09）改成大端序。版本 1 照「機間通訊封包規格」用小端序，見 wire_header.hpp。
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

namespace cbba_core::wire
{

class ByteWriter
{
public:
  explicit ByteWriter(std::vector<std::uint8_t> & out)
  : out_(out) {}

  void u8(std::uint8_t v) {out_.push_back(v);}
  void u16(std::uint16_t v) {put(v, 2);}
  void u32(std::uint32_t v) {put(v, 4);}
  void u64(std::uint64_t v) {put(v, 8);}
  void f32(float v)
  {
    std::uint32_t bits;
    std::memcpy(&bits, &v, sizeof(bits));
    u32(bits);
  }

private:
  void put(std::uint64_t v, int bytes)
  {
    for (int i = bytes - 1; i >= 0; --i) {
      out_.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
  }
  std::vector<std::uint8_t> & out_;
};

// 讀超過結尾時 ok() 變成 false，之後讀到的值都是 0
class ByteReader
{
public:
  ByteReader(const std::uint8_t * data, std::size_t size, std::size_t offset = 0)
  : data_(data), size_(size), pos_(offset), ok_(data != nullptr && offset <= size) {}

  bool ok() const {return ok_;}
  std::size_t remaining() const {return ok_ ? size_ - pos_ : 0;}

  std::uint8_t u8() {return static_cast<std::uint8_t>(get(1));}
  std::uint16_t u16() {return static_cast<std::uint16_t>(get(2));}
  std::uint32_t u32() {return static_cast<std::uint32_t>(get(4));}
  std::uint64_t u64() {return get(8);}
  float f32()
  {
    const std::uint32_t bits = u32();
    float v;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
  }

private:
  std::uint64_t get(int bytes)
  {
    if (!ok_ || size_ - pos_ < static_cast<std::size_t>(bytes)) {
      ok_ = false;
      return 0;
    }
    std::uint64_t v = 0;
    for (int i = 0; i < bytes; ++i) {
      v = (v << 8) | data_[pos_ + i];
    }
    pos_ += bytes;
    return v;
  }

  const std::uint8_t * data_;
  std::size_t size_;
  std::size_t pos_;
  bool ok_;
};

}  // namespace cbba_core::wire
