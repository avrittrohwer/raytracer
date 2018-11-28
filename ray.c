#define STB_IMAGE_WRITE_IMPLEMENTATION
#define IMAGE_SIZE 512

#include "stb_image_write.h"

int main() {
  int array_size = 3 * IMAGE_SIZE * IMAGE_SIZE;
  unsigned char image[array_size];

  int i = 0;
  while (i < array_size) {
    image[i++] = 0;
    image[i++] = 153;
    image[i++] = 255;
  }

  stbi_write_png("./image.png", IMAGE_SIZE, IMAGE_SIZE, 3, image, IMAGE_SIZE * 3);
  return 0;
}
