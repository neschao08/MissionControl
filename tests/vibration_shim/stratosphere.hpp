#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;
using s64 = int64_t;

struct Result {
    u32 value;
    u32 GetValue() const { return value; }
};

#define PACKED __attribute__((packed))
#define ALWAYS_INLINE inline __attribute__((always_inline))
#define AMS_UNREACHABLE_DEFAULT_CASE() default: break
#define R_DISCARD(x) static_cast<void>(x)
