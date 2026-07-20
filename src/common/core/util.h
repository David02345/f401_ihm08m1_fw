/*
 * util.h
 *
 *  Created on: 2026. 7. 4.
 *      Author: YDG
 */

#ifndef SRC_COMMON_CORE_UTIL_H_
#define SRC_COMMON_CORE_UTIL_H_

#include "def.h"



static inline float clampFloat(float value, float min, float max)
{
  if (value > max)
  {
    return max;
  }

  if (value < min)
  {
    return min;
  }

  return value;
}

static inline int32_t clampInt32(int32_t value, int32_t min, int32_t max)
{
  if (value > max)
  {
    return max;
  }

  if (value < min)
  {
    return min;
  }

  return value;
}

static inline  float wrapFloat(float theta, float min, float max)
{
  float range = max - min;
  if (range <= 0.0f)
  {
    return min;
  }

  while(theta >= max)
  {
    theta -= range;
  }
  while(theta < min)
  {
    theta += range;
  }

  return theta;
}

#endif /* SRC_COMMON_CORE_UTIL_H_ */
