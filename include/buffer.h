#ifndef BUFFER_H
#define BUFFER_H

#include <Eigen/Dense>
#include <stdexcept>
#include <vector>

struct point {
    int x;
    int y;
};

struct tri_ref {
    int mesh_id;
    int tri_index;
};

template <typename T> struct bound_box {
    T min_x, max_x = 0;
    T min_y, max_y = 0;
};

template <typename T> class buffer {
  public:
    virtual ~buffer() = default;

    T get(const int i, const int j) const { return data[i * width + j]; }
    void set(const int i, const int j, const T &value) {
        data[i * width + j] = value;
    }
    // default number of sqrt_samples is 1
    buffer(const int length, const int width, const int sqrt_samples = 1)
        : length(length), width(width), sqrt_samples(sqrt_samples) {
        data = std::vector<T>(length * sqrt_samples * width * sqrt_samples);
    };

    int get_length() const { return length; }
    int get_width() const { return width; }
    int get_length_p() const { return length / sqrt_samples; }
    int get_width_p() const { return width / sqrt_samples; }
    int get_sqrt_samples() const { return sqrt_samples; }
    int get_start() const { return data.begin(); }

  private:
    int length;
    int width;
    int sqrt_samples;
    std::vector<T> data{}; // intialize buffers as empty
};

// tiles are also buffers!
template <typename T> class tile : public buffer<T> {
  public:
    // NOTE: This must also avoid seg-fault for out of bounds requests
    // NOTE: This function must maintain, the structure of the data, this is
    // only for the ends of the tiles
    // NOTE: bbox here is at the pixel level not the sub-pixel level

    // constructor
    tile(const int sqrt_tile, const int sqrt_samples)
        : buffer<T>(sqrt_tile, sqrt_tile, sqrt_samples) {};

    // PUSH to to the BUFFER
    void push(buffer<T> &buff, const bound_box<int> &bbox,
              const point indices) {
        int sqrt_tile = this->get_length();
        int begin_x = indices.x * this->get_length();
        int begin_y = indices.y * this->get_width();
        int buff_len = buff.get_length_p();
        int buff_wid = buff.get_width_p();
        int sqrt_samples = buff.get_sqrt_samples();
        if (begin_x > buff_len || begin_y > buff_wid) {
            std::runtime_error("out of range");
        }
        // check for incomplete tiles
        int rem_x = buff_len - begin_x;
        int rem_y = buff_wid - begin_y;
        if (rem_x < sqrt_tile || rem_y < sqrt_tile) {
            // iterate safely
            for (int i = 0; i < rem_x * sqrt_samples; i++) {
                for (int j = 0; j < rem_y * sqrt_samples; j++) {
                    buff.set(begin_x + i, begin_y + j, this->data.get(i, j));
                }
            }
        }
        // base case
        for (int i = 0; i < sqrt_tile * sqrt_samples; i++) {
            for (int j = 0; j < sqrt_tile * sqrt_samples; j++) {
                buff.set(begin_x + i, begin_y + j, this->data[i][j]);
            }
        }
    };

    // PULL to the TILE
    void pull(const buffer<T> &buff, const bound_box<int> &bbox,
              const point indices) const {
        int sqrt_tile = this->get_length();
        int begin_x = indices.x * this->get_length();
        int begin_y = indices.y * this->get_width();
        int buff_len = buff.get_length_p();
        int buff_wid = buff.get_width_p();
        int sqrt_samples = buff.get_sqrt_samples();
        if (begin_x > buff_len || begin_y > buff_wid) {
            std::runtime_error("out of range");
        }
        // check for incomplete tiles
        int rem_x = buff_len - begin_x;
        int rem_y = buff_wid - begin_y;
        if (rem_x < sqrt_tile || rem_y < sqrt_tile) {
            // iterate safely
            for (int i = 0; i < rem_x * sqrt_samples; i++) {
                for (int j = 0; j < rem_y * sqrt_samples; j++) {
                    set(i, j, buff.get(begin_x + i, begin_y + j));
                }
            }
        }
        // base case
        for (int i = 0; i < sqrt_tile * sqrt_samples; i++) {
            for (int j = 0; j < sqrt_tile * sqrt_samples; j++) {
                set(i, j, buff.get(begin_x + i, begin_y + j));
            }
        }
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

// simple buffer, has no sub-pixels
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
