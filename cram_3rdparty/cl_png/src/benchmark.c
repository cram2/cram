#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <png.h>


/* Error handling. */

const char *program_name = NULL;

void set_program_name(const char *argv0)
{
     program_name = strrchr(argv0, '/');
     if (program_name)
          program_name++;
     else
          program_name = argv0;
}

static void
verror(const char *format, va_list ap)
{
     if (program_name)
          fprintf(stderr, "%s: ", program_name);
     vfprintf(stderr, format, ap);
}

__attribute__((noreturn))
void
fatal_error(const char *format, ...)
{
     va_list ap;

     va_start(ap, format);
     verror(format, ap);
     va_end(ap);
     fprintf(stderr, "\n");
     exit(1);
}

__attribute__((noreturn))
void
fatal_perror(const char *format, ...)
{
     va_list ap;

     va_start(ap, format);
     verror(format, ap);
     va_end(ap);
     fprintf(stderr, ": %s", strerror(errno));
     fprintf(stderr, "\n");
     exit(1);
}



png_bytep
make_image(png_uint_32 height, png_uint_32 width, png_byte nchannels,
           png_byte bit_depth)
{
     png_bytep image;
     image = malloc(height * width * nchannels * bit_depth / 8);
     if (!image) fatal_perror("malloc");
     return image;
}

png_bytepp
make_row_pointers(png_bytep image, png_uint_32 height, png_uint_32 width,
                  png_byte nchannels)
{
     png_bytepp row_pointers;
     png_uint_32 i;
     row_pointers = malloc(height * sizeof(png_bytep));
     if (!row_pointers) fatal_perror("malloc");
     for (i = 0; i < height; i++)
          row_pointers[i] = image + i * width * nchannels;
     return row_pointers;
}

void
decode(FILE *input, png_bytep *image, png_uint_32 *height, png_uint_32 *width,
       png_byte *nchannels)
{
     png_structp png_ptr;
     png_infop info_ptr, end_info;
     int bit_depth, color_type;
     png_bytepp row_pointers;

     png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
     if (!png_ptr) fatal_error("png_create_read_struct");
     info_ptr = png_create_info_struct(png_ptr);
     if (!info_ptr) fatal_error("png_create_info_struct");
     end_info = png_create_info_struct(png_ptr);
     if (!end_info) fatal_error("png_create_info_struct");
     if (setjmp(png_jmpbuf(png_ptr))) fatal_error("png_error via setjmp");

     png_init_io(png_ptr, input);
     png_read_info(png_ptr, info_ptr);
     png_get_IHDR(png_ptr, info_ptr, width, height, &bit_depth, &color_type,
                  NULL, NULL, NULL);
     if (bit_depth != 8)
          fatal_error("Can only handle 8-bit images.");
     if (color_type == PNG_COLOR_TYPE_PALETTE)
          png_set_palette_to_rgb(png_ptr);
     if (!(color_type & ~PNG_COLOR_MASK_ALPHA))
          png_set_expand(png_ptr);
     if (color_type & PNG_COLOR_MASK_ALPHA)
          png_set_strip_alpha(png_ptr);

     *nchannels = color_type & ~PNG_COLOR_MASK_ALPHA ? 3 : 1;
     *image = make_image(*height, *width, *nchannels, bit_depth);
     row_pointers = make_row_pointers(*image, *height, *width, *nchannels);
     png_read_image(png_ptr, row_pointers);
     free(row_pointers);
     png_read_end(png_ptr, end_info);
     png_destroy_read_struct(&png_ptr, &info_ptr, &end_info);
}

void encode(FILE *output, png_bytep image, png_uint_32 height, 
            png_uint_32 width, png_byte nchannels)
{
     png_structp png_ptr;
     png_infop info_ptr;
     int bit_depth, color_type;
     png_bytepp row_pointers;

     png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, 
                                       NULL);
     if (!png_ptr) fatal_error("png_create_write_struct");
     info_ptr = png_create_info_struct(png_ptr);
     if (!info_ptr) fatal_error("png_create_info_struct");
     if (setjmp(png_jmpbuf(png_ptr))) fatal_error("png_error via setjmp");

     png_init_io(png_ptr, output);
     bit_depth = 8;
     switch (nchannels) {
     case 1: color_type = PNG_COLOR_TYPE_GRAY; break;
     case 3: color_type = PNG_COLOR_TYPE_RGB; break;
     default: fatal_error("Cannot write images with %d channels", nchannels);
     }
     png_set_IHDR(png_ptr, info_ptr, width, height, bit_depth, color_type,
                  PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT,
                  PNG_FILTER_TYPE_DEFAULT);
     row_pointers = make_row_pointers(image, height, width, nchannels);
     png_set_rows(png_ptr, info_ptr, row_pointers);
     png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);
     free(row_pointers);
     png_destroy_write_struct(&png_ptr, &info_ptr);
}

png_bytep to_16_bit(const png_bytep image8, png_uint_32 height, 
                    png_uint_32 width, png_byte nchannels)
{
     png_bytep image16;
     png_uint_32 i;

     image16 = make_image(height, width, nchannels, 16);
     for (i = 0; i < height * width * nchannels; i++) {
          image16[2 * i] = image8[i];
          image16[2 * i + 1] = image8[i];
     }
     return image16;
}

png_bytep to_8_bit(const png_bytep image16, png_uint_32 height, 
                   png_uint_32 width, png_byte nchannels)
{
     png_bytep image8;
     png_uint_32 i;

     image8 = make_image(height, width, nchannels, 8);
     for (i = 0; i < height * width * nchannels; i++)
          image8[i] = image16[2 * i] - (image16[2 * i + 1] < image16[2 * i]);
     return image8;
}



double elapsed_time(struct timeval *start_time, int divisor)
{
     struct timeval end_time;
     gettimeofday(&end_time, NULL);
     return (end_time.tv_sec - start_time->tv_sec +
             (end_time.tv_usec - start_time->tv_usec) / 1000000.0) / divisor;
}

double time_decode(const char *filename, int n)
{
     int i;
     png_bytep image;
     FILE *input;
     struct timeval start_time;
     png_uint_32 height, width;
     png_byte nchannels;

     gettimeofday(&start_time, NULL);
     for (i = 0; i < n; i++) {
          input = fopen(filename, "r");
          if (!input) fatal_perror("%s", filename);
          decode(input, &image, &height, &width, &nchannels);
          fclose(input);
          free(image);
     }
     return elapsed_time(&start_time, n);
}

double time_encode(const char *input_filename, const char *output_filename,
                   int n)
{
     int i;
     png_bytep image;
     FILE *input, *output;
     struct timeval start_time;
     png_uint_32 height, width;
     png_byte nchannels;

     input = fopen(input_filename, "r");
     if (!input) fatal_perror("%s", input_filename);
     decode(input, &image, &height, &width, &nchannels);
     fclose(input);

     gettimeofday(&start_time, NULL);
     for (i = 0; i < n; i++) {
          output = fopen(output_filename, "w");
          if (!output) fatal_perror("%s", output_filename);
          encode(output, image, height, width, nchannels);
          fclose(output);
     }
     free(image);
     return elapsed_time(&start_time, n);
}

double time_to_16_bit(const char *input_filename, int n)
{
     int i;
     png_bytep image8, image16;
     FILE *input;
     struct timeval start_time;
     png_uint_32 height, width;
     png_byte nchannels;

     input = fopen(input_filename, "r");
     if (!input) fatal_perror("%s", input_filename);
     decode(input, &image8, &height, &width, &nchannels);
     fclose(input);

     gettimeofday(&start_time, NULL);
     for (i = 0; i < n; i++) {
          image16 = to_16_bit(image8, height, width, nchannels);
          free(image16);
     }
     free(image8);
     return elapsed_time(&start_time, n);
}

double time_to_8_bit(const char *input_filename, int n)
{
     int i;
     png_bytep image, image8, image16;
     FILE *input;
     struct timeval start_time;
     png_uint_32 height, width;
     png_byte nchannels;

     input = fopen(input_filename, "r");
     if (!input) fatal_perror("%s", input_filename);
     decode(input, &image, &height, &width, &nchannels);
     fclose(input);

     image16 = to_16_bit(image, height, width, nchannels);

     gettimeofday(&start_time, NULL);
     for (i = 0; i < n; i++) {
          image8 = to_8_bit(image16, height, width, nchannels);
          free(image8);
     }
     free(image16);
     free(image);
     return elapsed_time(&start_time, n);
}

int main(int argc, char *argv[])
{
     set_program_name(argv[0]);
     if (argc < 3) fatal_error("Usage: %s INPUT OUTPUT", program_name);

     printf("to_16_bit: %f s\n", time_to_16_bit(argv[1], 50));
     printf("to_8_bit: %f s\n", time_to_8_bit(argv[1], 50));
     printf("decode:    %f s\n", time_decode(argv[1], 10));
     printf("encode:    %f s\n", time_encode(argv[1], argv[2], 1));
     return 0;
}
