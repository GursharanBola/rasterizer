#include "buffer.h"
#include <Eigen/Dense>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

/**
 * Colors are rendered from [0,1] only internally are they actually represented
 * as 8 bit with channels, colors in this way is clever for later functions.
 * This idea was takent from RIOW.
 */

int image_buffer::draw_png(std::string filename, int width, int height,
                           int channels, void *data, int stride) {
    int png_truth =
        stbi_write_png(filename.c_str(), width, height, channels, data, stride);
    return png_truth;
}

Eigen::Vector3d image_buffer::get_color(const int x, const int y) const {
    if (x < 0 || x >= get_width() || y < 0 || y >= get_length()) {
        return Eigen::Vector3d(0, 0, 0);
    }

    double r = get(y, (3 * x) + 0);
    double g = get(y, (3 * x) + 1);
    double b = get(y, (3 * x) + 2);

    return Eigen::Vector3d(r / 255.0, g / 255.0, b / 255.0);
}

bool image_buffer::set_color(const int x, const int y, Eigen::Vector3d color) {
    if (x < 0 || x >= get_width() || y < 0 || y >= get_length()) {
        return false; // We failed
    }

    double r = color(0);
    double g = color(1);
    double b = color(2);

    r = clamp(r, 0.0, 1.0);
    g = clamp(g, 0.0, 1.0);
    b = clamp(b, 0.0, 1.0);

    set(x, y, static_cast<int>(255.999 * r));
    set(x + 1, y, static_cast<int>(255.999 * g));
    set(x + 1, y, static_cast<int>(255.999 * b));

    return true;
}
