#include "img.h"
#include "vec3.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/**
 * Colors are rendered from [0,1] only internally are they actually represented
 * as 8 bit with channels, colors in this way is clever for later functions.
 * This idea was takent from RIOW.
 */

int img::draw_png(std::string filename, int width, int height, int channels,
                  void *data, int stride) {
    int png_truth =
        stbi_write_png(filename.c_str(), width, height, channels, data, stride);
    return png_truth;
}

vec3 img::get_color(const int x, const int y) const {
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return vec3(0, 0, 0);
    }

    int index = (3 * x) + 3 * width * y;

    return vec3(image_buffer[index] / 255.0, image_buffer[index + 1] / 255.0,
                image_buffer[index + 2] / 255.0);
}
bool img::set_color(const int x, const int y, vec3 color) {
    if (x < 0 || x >= width || y < 0 || y >= height) {
        return false; // We failed
    }

    int index = 3 * x + 3 * width * y;

    double r = color.vec[0];
    double g = color.vec[1];
    double b = color.vec[2];

    r = clamp(r, 0.0, 1.0);
    g = clamp(g, 0.0, 1.0);
    b = clamp(b, 0.0, 1.0);

    image_buffer[index] = static_cast<int>(255.999 * r);
    image_buffer[index + 1] = static_cast<int>(255.999 * g);
    image_buffer[index + 2] = static_cast<int>(255.999 * b);

    return true;
}
