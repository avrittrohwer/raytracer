cc            = gcc -std=c11
warning_flags = -Wall -Wextra -Wpedantic

all: ray

ray: stb_image_write.h vec_math.h ray.c
	$(cc) $(warning_flags) stb_image_write.h vec_math.h ray.c -o ray -lm

clean:
	rm ./ray ./image.png
