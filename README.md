# raytracer

This is a minimal raytracer with diffuse shading and support for reflective objects, written for a Graphics class.
The raytracing logic is in `ray.c`, some minimal vector math functionality is in `vec_math.h`, and code to write a png image is in `stb_image_write.h` (I did not write this file, it was provided for this assignment).

Geometry in the scene are defined by an `Object` struct:
```
struct Object {
  // object is either a sphere or triangle
  union {
    struct Sphere sphere;
    struct Triangle triangle;
  };
  // calculates the time an intersection occurs
  float (*intersect_fn)(struct Object*, struct Ray*);
  // calculates the normal at a given point
  void (*normal_fn)(struct Object*, float* point, float* normal);
  // color of the object
  unsigned char color[3];
  int is_reflective;
};
```
I only wrote code for spheres and triangles but the tracing logic is agnostic to object types as long as an intersect function and normal function are provided.
