#ifndef BUFFER_H
#define BUFFER_H

#include <Eigen/Dense>
#include <thread>
#include <vector>

// bound_boxes are just a container they can store integers and
// doubles
template <typename T> struct bound_box {
    T min_x, max_x = 0;
    T min_y, max_y = 0;
};

struct point {
    int x;
    int y;
};

struct tri_ref {
    int mesh_id;
    int tri_index;
};

template <typename T> class buffer {
  public:
    virtual ~buffer() = default;

    virtual T get(const int i, const int j) const {
        return data[i * width + j];
    }
    virtual void set(const int i, const int j, const T &value) {
        data[i * width + j] = value;
    }
    // default number of sqrt_samples is 1
    buffer(const int length, const int width, const int sqrt_samples = 1)
        : length(length), width(width), sqrt_samples(sqrt_samples) {
        data = std::vector<T>(length * sqrt_samples * width * sqrt_samples);
    };

    // NOTE: This must also avoid seg-fault for out of bounds requests
    void push(const std::vector<std::vector<T>> &tile, bound_box<int> &bbox,
              point indices) {};
    void pull(std::vector<std::vector<T>> &tile, bound_box<int> &bbox,
              point indices) const {};

    int get_length() const { return length; }
    int get_width() const { return width; }
    int get_sqrt_samples() const { return sqrt_samples; }
    int get_start() const { return data.begin(); }

  private:
    int length;
    int width;
    int sqrt_samples;
    std::vector<T> data;
    // copies a "line" of information, all arrays are 1d
    void cpy_line(buffer<T> buff, int start, int length,
                  std::vector<std::vector<T>> tile) {
        int begin = buff.get_start() + start;
        std::copy(begin, begin + length, tile.begin());
    }
};

class z_buffer : public buffer<double> {
  public:
    z_buffer(const int length, const int width, const int sqrt_samples)
        : buffer(length, width, sqrt_samples) {};

    void set_sample(const int i, const int j, const int sam_i, const int sam_j,
                    const double val) {

        int absolute_i = (i * get_sqrt_samples()) + sam_i;
        int absolute_j = (j * get_sqrt_samples()) + sam_j;

        set(absolute_i, absolute_j, val);
    }
};

class seen_buffer : public buffer<tri_ref> {
  public:
    seen_buffer(const int length, const int width, const int sqrt_samples)
        : buffer(length, width, sqrt_samples) {};

    void set_sample(const int i, const int j, const int sam_i, const int sam_j,
                    const tri_ref val) {

        int absolute_i = (i * get_sqrt_samples()) + sam_i;
        int absolute_j = (j * get_sqrt_samples()) + sam_j;

        set(absolute_i, absolute_j, val);
    }
};

// simple buffer, has no sub-pix
class image_buffer : public buffer<double> {
  public:
    image_buffer(const int length, const int width, const int num_channels)
        : buffer(length, width * num_channels) {};

    int draw_png(std::string filename, int width, int height, int channels,
                 void *data, int stride);

    Eigen::Vector3d get_color(const int x, const int y) const;

    bool set_color(const int x, const int y, Eigen::Vector3d color);
};

class vertex_buffer {
  public:
    vertex_buffer() = default;
    void add(const Eigen::Vector3d &v) { data.push_back(v); }
    Eigen::Vector3d get(const int i) const { return data[i]; }
    int size() const { return data.size(); }

  private:
    std::vector<Eigen::Vector3d> data;
};

inline double clamp(double x, double min, double max) {
    if (x < min)
        return min;
    if (x > max)
        return max;
    return x;
}
#endif
