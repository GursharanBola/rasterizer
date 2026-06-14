#ifndef IMG_H
#define IMG_H

#include "vec3.h"
#include <array>
#include <cstdint>
#include <string>

class img {
  public:
    static constexpr int width = 800;
    static constexpr int height = 600;
    static constexpr int channels = 3;

    std::vector<uint8_t> image_buffer;

    img() : image_buffer(width * height * channels, 0) {}

    int draw_png(std::string filename, int width, int height, int channels,
                 void *data, int stride);

    vec3 get_color(const int x, const int y) const;

    bool set_color(const int x, const int y, vec3 color);
};

inline double clamp(double x, double min, double max) {
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

#endif
