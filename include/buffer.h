#ifndef BUFFER_H
#define BUFFER_H

#include <Eigen/Dense>
#include <array>
#include <vector>

template <typename T> class buffer {
  public:
    virtual ~buffer() = default;

    virtual T get(const int i, const int j) const {
        return data[i * width + j];
    }
    virtual void set(const int i, const int j, const T &value) {
        data[i * width + j] = value;
    }

    buffer(const int length, const int width) : length(length), width(width) {
        data = std::vector<T>(length * width);
    };

    inline int get_length() const { return length; }
    inline int get_width() const { return width; }

  private:
    const int length;
    const int width;
    std::vector<T> data;
};

// each entry will store the seen_id, and the z-buffer value.
using z_entry = std::array<double, 2>;

class z_buffer : public buffer<z_entry> {
  public:
    z_buffer(const int length, const int width, const int sqrt_samples)
        : buffer(length * sqrt_samples, width * sqrt_samples),
          sqrt_samples(sqrt_samples) {};

    void set_sample(const int i, const int j, const int sam_i, const int sam_j,
                    const double val);

  private:
    int sqrt_samples;
};

class vertex_buffer : public buffer<Eigen::Vector3d> {
  public:
    vertex_buffer(const int length, const int width) : buffer(length, width) {};
};

class image_buffer : public buffer<double> {
  public:
    image_buffer(const int length, const int width, const int num_channels)
        : buffer(length, width * num_channels) {};

    int draw_png(std::string filename, int width, int height, int channels,
                 void *data, int stride);

    Eigen::Vector3d get_color(const int x, const int y) const;

    bool set_color(const int x, const int y, Eigen::Vector3d color);
};

inline double clamp(double x, double min, double max) {
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}

#endif
