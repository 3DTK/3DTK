#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <iostream>

#include <png.h>

unsigned int WIDTH=960;
unsigned int HEIGHT=540;

png_bytep *load_png(char *filename)
{
	FILE *fp = fopen(filename, "rb");

	png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	if (!png) abort();
	png_infop info = png_create_info_struct(png);
	if (!info) abort();
	if (setjmp(png_jmpbuf(png))) abort();
	png_init_io(png, fp);
	png_read_info(png, info);
	if (png_get_image_width(png, info) != WIDTH) {
		std::cerr << "cannot handle png with a width other than " << WIDTH << std::endl;
		exit(1);
	}
	if (png_get_image_height(png, info) != HEIGHT) {
		std::cerr << "cannot handle png with a height other than " << HEIGHT << std::endl;
		exit(1);
	}
	if (png_get_bit_depth(png, info) != 8) {
		std::cerr << "cannot handle png with bit depth other than 8" << std::endl;
		exit(1);
	}
	if (png_get_color_type(png, info) != PNG_COLOR_TYPE_RGB) {
		std::cerr << "cannot handle non-RGB png" << std::endl;
		exit(1);
	}
	png_bytep *row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * HEIGHT);
	for (png_uint_32 y = 0; y < HEIGHT; y++) {
		row_pointers[y] = (png_byte*)malloc(png_get_rowbytes(png,info));
	}
	png_read_image(png, row_pointers);
	fclose(fp);

	return row_pointers;
}

int main(int argc, char* argv[]) {
	int maxerrors = atoi(argv[1]);
	png_bytep *image1 = load_png(argv[2]);
	png_bytep *image2 = load_png(argv[3]);

	unsigned int errcount = 0;

	for (unsigned int y = 0; y < HEIGHT; y++) {
		png_bytep row1 = image1[y];
		png_bytep row2 = image2[y];
		for (unsigned int x = 0; x < WIDTH; x++) {
			for (unsigned int z = 0; z < 3; z++) {
				if (row1[x*3 + z] != row2[x*3 + z]) {
					errcount += 1;
				}
			}
		}
	}

	if (errcount <= maxerrors) {
		return 0;
	} else {
		std::cerr << "errcount: " << errcount << std::endl;
		return 1;
	}
}
