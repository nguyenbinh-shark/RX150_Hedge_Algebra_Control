#include "hac.h"

float hac_eval(float x, float xd, float a, float b, float c) {
  return (2.0f * c / (3.0f * a)) * x + (c / (3.0f * b)) * xd;
}
