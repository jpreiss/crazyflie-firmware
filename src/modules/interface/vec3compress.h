#pragma once

#define MAKE_FIXED_CONV(name, limit) \
static inline int16_t name ## _float_to_fix16(float x) \
{ \
  float normalized = x / limit; \
  if (normalized > 1.0f) normalized = 1.0f; \
  if (normalized < -1.0f) normalized = -1.0f; \
  return INT16_MAX * normalized; \
} \
static inline float name ## _fix16_to_float(int16_t x) \
{ \
  return (limit / INT16_MAX) * ((float)x); \
} \

MAKE_FIXED_CONV(position, 8.0f)
MAKE_FIXED_CONV(velocity, 20.0f)
MAKE_FIXED_CONV(accel, 20.0f)
MAKE_FIXED_CONV(omega, 20.0f)

#undef MAKE_FIXED_CONV
