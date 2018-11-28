cc     = gcc -std=gnu99
cflags = -Wall -Wextra -Wpedantic

all: ray

ray: stb_image_write.h ray.c
	$(cc) $(cflags) stb_image_write.h ray.c -o ray

clean:
	rm ./ray
