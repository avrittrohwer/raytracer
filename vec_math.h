#include <math.h>

// Squares a value
static inline float square(float a) { return a * a; }

// Finds the magnitude of a vector
static inline float magnitude(float* a) {
  return sqrtf(square(a[0]) + square(a[1]) + square(a[2]));
}

// Normalizes a vector
static inline void normalize(float* a) {
  float mag = magnitude(a);
  a[0] = a[0] / mag;
  a[1] = a[1] / mag;
  a[2] = a[2] / mag;
}

// Calculates the dot product of two vectors
static inline float dot(float* a, float* b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }

// r = a + b
static inline void vec_add(float* r, float* a, float* b) {
  r[0] = a[0] + b[0];
  r[1] = a[1] + b[1];
  r[2] = a[2] + b[2];
}

// r = a - b
static inline void vec_sub(float* r, float* a, float* b) {
  r[0] = a[0] - b[0];
  r[1] = a[1] - b[1];
  r[2] = a[2] - b[2];
}

// r = a * b
static inline void vec_mult(float* r, float a, float* b) {
  r[0] = a * b[0];
  r[1] = a * b[1];
  r[2] = a * b[2];
}
