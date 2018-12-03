#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include "vec_math.h"

// how wide / tall the resulting image is
#define IMAGE_SIZE 512

// Sphere is defined by a center point and a radius
struct Sphere {
  float center[3];
  float radius;
};

// Triangle is defined by 3 points
struct Triangle {
  float a[3];
  float b[3];
  float c[3];
};

// Ray is defined by a vector and position
struct Ray {
  float direction[3];
  float origin[3];
};

// An object in the scene
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

// A scene is defined by a collection of objects, an image plane, and a camera position
struct Scene {
  struct Object* objects;  // collection of objects
  float upper_left[3];     // upper left position of the image plane
  float plane_size;        // how tall and wide the plane is
  float cam_pos[3];        // position of the camera
  float light_pos[3];      // Position of the light
};

// Returns time ray hits sphere or 0 if it doesn't
float sphere_intersects(struct Object* obj, struct Ray* ray) {
  // find vector from ray origin to sphere center
  float origin_center[3];
  vec_sub(origin_center, obj->sphere.center, ray->origin);

  // if the projection of ray direction onto to_center is negative, no intersections
  float t_center = dot(origin_center, ray->direction);
  if (t_center < 0) {
    return 0;
  }

  // no intersection if perpendicular dist from center sphere to ray is greater than sphere radius
  float dist_ray = sqrtf(square(magnitude(origin_center)) - square(t_center));
  if (dist_ray < 0 || dist_ray > obj->sphere.radius) {
    return 0;
  }

  // find the t values for sphere intersections
  float t_inner = sqrtf(square(obj->sphere.radius) - square(dist_ray));
  float t_hit = t_center - t_inner;

  // t_hit can be negative if we're in the sphere, return the second hit time in that case
  return t_hit < 0 ? t_center + t_inner : t_hit;
}

// Returns time ray hits triangle or 0 if it doesn't
float triangle_intersects(struct Object* obj, struct Ray* ray) {
  struct Triangle* triangle = &obj->triangle;

  // compute components of equations
  float a = triangle->a[0] - triangle->b[0];
  float b = triangle->a[1] - triangle->b[1];
  float c = triangle->a[2] - triangle->b[2];
  float d = triangle->a[0] - triangle->c[0];
  float e = triangle->a[1] - triangle->c[1];
  float f = triangle->a[2] - triangle->c[2];
  float g = ray->direction[0];
  float h = ray->direction[1];
  float i = ray->direction[2];
  float j = triangle->a[0] - ray->origin[0];
  float k = triangle->a[1] - ray->origin[1];
  float l = triangle->a[2] - ray->origin[2];

  // compute partials
  float ei_hf = e * i - h * f;
  float gf_di = g * f - d * i;
  float dh_eg = d * h - e * g;
  float ak_jb = a * k - j * b;
  float jc_al = j * c - a * l;
  float bl_kc = b * l - k * c;

  float m = a * ei_hf + b * gf_di + c * dh_eg;

  // compute t, beta, gamma, returning early if possible
  float t = -(f * ak_jb + e * jc_al + d * bl_kc) / m;
  if (t < 0) {
    return 0;
  }

  float gamma = (i * ak_jb + h * jc_al + g * bl_kc) / m;
  if (gamma < 0 || gamma > 1) return 0;

  float beta = (j * ei_hf + k * gf_di + l * dh_eg) / m;
  if (beta < 0 || beta > 1 - gamma) return 0;

  return t;
}

// Calculates the normal at the given point
void sphere_normal(struct Object* obj, float* point, float* normal) {
  // normal is point - center
  vec_sub(normal, point, obj->sphere.center);
  normalize(normal);
}

// Calculates the normal at the given point
void triangle_normal(struct Object* obj, __attribute__((unused)) float* point, float* normal) {
  // normal is the cross product of vectors ab and ac
  float ab[3];
  vec_sub(ab, obj->triangle.b, obj->triangle.a);

  float ac[3];
  vec_sub(ac, obj->triangle.c, obj->triangle.a);

  cross(normal, ab, ac);
  normalize(normal);
}

// two object lists, selectable on command line
// clang-format off
struct Object reference[] = {
  // Large reflective sphere
  {
    .sphere = {{0, 0, -16}, 2},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {0},
    .is_reflective = 1
  },
  // Smaller reflective sphere
  {
    .sphere = {{3, -1, -14}, 1},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {0},
    .is_reflective = 1
  },
  // Smaller red sphere
  {
    .sphere = {{-3, -1, -14}, 1},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {255, 0, 0},
    .is_reflective = 0
  },
  // back wall lower triangle
  {
    .triangle = {{-8, -2, -20}, {8, -2, -20}, {8, 10, -20}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {0, 0, 255},
    .is_reflective = 0
  },
  // back wall upper triangle
  {
    .triangle = {{-8, -2, -20}, {8, 10, -20}, {-8, 10, -20}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {0, 0, 255},
    .is_reflective = 0
  },
  // floor further triangle
  {
    .triangle = {{-8, -2, -20}, {8, -2, -10}, {8, -2, -20}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {255, 255, 255},
    .is_reflective = 0
  },
  // floor closer triangle
  {
    .triangle = {{-8, -2, -20}, {-8, -2, -10}, {8, -2, -10}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {255, 255, 255},
    .is_reflective = 0
  },
  // right wall lower triangle
  {
    .triangle = {{8, -2, -20}, {8, -2, -10}, {8, 10, -20}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {255, 0, 0},
    .is_reflective = 0
  }
};

struct Object custom[] = {
  // Lower sphere
  {
    .sphere = {{0, -5, -10}, 2},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {255, 0, 0},
    .is_reflective = 0
  },
  // Upper sphere
  {
    .sphere = {{0, 5, -20}, 2},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {0, 255, 0},
    .is_reflective = 0
  },
  // Left sphere
  {
    .sphere = {{-5, 0, -15}, 2},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {0, 0, 255},
    .is_reflective = 0
  },
  // Right sphere
  {
    .sphere = {{5, 0, -25}, 2},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {255, 255, 255},
    .is_reflective = 0
  },
  // Middle sphere
  {
    .sphere = {{0, 0, -16}, 2},
    .intersect_fn = sphere_intersects,
    .normal_fn = sphere_normal,
    .color = {0},
    .is_reflective = 1
  },
  // back wall lower triangle
  {
    .triangle = {{-50, -50, -100}, {50, -50, -100}, {50, 50, -100}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {20, 20, 20},
    .is_reflective = 0
  },
  // back wall upper triangle
  {
    .triangle = {{-50, -50, -100}, {50, 50, -100}, {-50, 50, -100}},
    .intersect_fn = triangle_intersects,
    .normal_fn = triangle_normal,
    .color = {20, 20, 20},
    .is_reflective = 0
  },
};
// clang-format on

// Traces a ray, writes color
void trace(struct Object* objects, int num_objects, struct Ray* ray, float* light_pos,
           unsigned char* color, int iteration) {
  // if we've bounced around 10 times, just use the color black
  if (iteration > 10) {
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    return;
  };

  // Find closest intersecting object
  float min_t = INFINITY;
  struct Object* closest_obj = NULL;

  for (int i = 0; i < num_objects; i += 1) {
    struct Object* obj = &objects[i];

    float t = obj->intersect_fn(obj, ray);
    if (t != 0 && t < min_t) {
      min_t = t;
      closest_obj = obj;
    }
  }

  // if we didn't hit anything, the color is black
  if (closest_obj == NULL) {
    color[0] = 0;
    color[1] = 0;
    color[2] = 0;
    return;
  }

  // find the point where ray hit the object
  float point_hit[3];
  vec_mult(point_hit, min_t, ray->direction);
  vec_add(point_hit, point_hit, ray->origin);

  // find the normal of the surface
  float normal[3];
  closest_obj->normal_fn(closest_obj, point_hit, normal);

  if (closest_obj->is_reflective) {
    // bounce a ray off and use that color
    // reflected = initial - 2 normal (initial dot normal)
    struct Ray reflected_ray;
    vec_mult(normal, 2 * dot(ray->direction, normal), normal);
    vec_sub(reflected_ray.direction, ray->direction, normal);
    normalize(reflected_ray.direction);
    memcpy(reflected_ray.origin, point_hit, 3 * sizeof(float));

    trace(objects, num_objects, &reflected_ray, light_pos, color, iteration += 1);
  } else {
    // shoot a ray at the light, if we hit anything sooner than the light then we're in a shadow
    struct Ray shadow_ray;
    vec_sub(shadow_ray.direction, light_pos, point_hit);
    float light_dist = magnitude(shadow_ray.direction);

    // normalize, make origin the point we hit
    normalize(shadow_ray.direction);
    memcpy(shadow_ray.origin, point_hit, 3 * sizeof(float));

    // if vector from point_hit to light intersects any objects, we're in a shadow
    int hit_something = 0;
    for (int i = 0; i < num_objects; i += 1) {
      struct Object* obj = &objects[i];
      if (obj == closest_obj) continue;

      float t = obj->intersect_fn(obj, &shadow_ray);
      if (t != 0 && t < light_dist) {
        hit_something = 1;
        break;
      }
    }

    // limit diffuse to [0.2, 1]
    float diffuse = dot(shadow_ray.direction, normal);
    if (diffuse < 0.2 || hit_something) diffuse = 0.2;

    // calculate adjusted color
    color[0] = closest_obj->color[0] * diffuse;
    color[1] = closest_obj->color[1] * diffuse;
    color[2] = closest_obj->color[2] * diffuse;
  }
}

int main(int argc, char** argv) {
  char usage_msg[] = {"usage: ray reference|custom\n"};

  // parse args for reference or custom
  if (argc < 2) {
    printf("%s", usage_msg);
    return 1;
  }

  // assign object list and figure out how many elements they have
  struct Object* objects;
  int num_objects;
  if (!strcmp("reference", argv[1])) {
    objects = reference;
    num_objects = sizeof(reference) / sizeof(struct Object);
  } else if (!strcmp("custom", argv[1])) {
    objects = custom;
    num_objects = sizeof(custom) / sizeof(struct Object);
  } else {
    printf("%s", usage_msg);
    return 1;
  }

  // Define the scene
  struct Scene scene = {
      objects,      // array of objects
      {-1, 1, -2},  // upper left corner of image plane
      2,            // plane width
      {0, 0, 0},    // cam position
      {3, 5, -15}   // light position
  };

  // Initialize the image buffer, calculate pixel width
  unsigned char image_buf[3 * IMAGE_SIZE * IMAGE_SIZE];
  float px_width = scene.plane_size / IMAGE_SIZE;
  float px_half = px_width / 2;

  // Trace the scene
  for (int y = 0; y < IMAGE_SIZE; y += 1) {
    for (int x = 0; x < IMAGE_SIZE; x += 1) {
      // find center of pixel
      // clang-format off
      float px_center[] = {
        scene.upper_left[0] + ((px_width * x) + px_half),
        scene.upper_left[1] - ((px_width * y) - px_half),
        scene.upper_left[2]
      };
      // clang-format on

      // Make ray
      struct Ray ray;
      vec_sub(ray.direction, px_center, scene.cam_pos);
      normalize(ray.direction);
      memcpy(ray.origin, scene.cam_pos, 3 * sizeof(float));

      // fill in image with color of pixel
      int index = (IMAGE_SIZE * y * 3) + (3 * x);
      trace(scene.objects, num_objects, &ray, scene.light_pos, &image_buf[index], 0);
    }
  }

  // write image buffer to file
  char file_name[14];
  sprintf(file_name, "%s.png", argv[1]);

  stbi_write_png(file_name, IMAGE_SIZE, IMAGE_SIZE, 3, image_buf, IMAGE_SIZE * 3);
  return 0;
}
