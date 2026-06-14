#ifndef BUFFER_H
#define BUFFER_H

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

    inline int get_length() { return length; }
    inline int get_width() { return width; }

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


#endif
