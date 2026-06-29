#ifndef ENGINE_H
#define ENGINE_H

#include "buffer.h"
#include "projector.h"
#include "scene.h"
#include <stdexcept>
#include <utility>

/**
 *
 * The program for optimal performance will store all lights in their own
 * buffers and then later use these for a shading pass. This is optimal
 * for performance since it optimizes the cache blocking. To avoid for the
 * camera's buffers it will complete a v and s buffer completion, as well as
 * a coloring pass for phong shading. The shading pass is then done using
 * tiling.
 *
 * coloring pass is done using the color_buffer
 *
 * tiling only happens on the shading pass
 */

class engine {
  public:
    engine(scene &scene, camera &camera) : scene(scene), camera(camera) {};

    // TODO: Right now this function does not need to have an tiling
    void fill_v_s(const projector &projector,
                  const std::vector<std::unique_ptr<mesh>> &meshes,
                  const vertex_buffer &v_buff, z_buffer &z_buff,
                  seen_buffer &s_buff) const;
    // TODO: determine additional functions need to be added to the engine
  private:
    scene &scene;
    camera &camera;
};

// a set of utility methods to impliment member methods
inline Eigen::Vector3d project_point(const Eigen::Vector3d p1,
                                     const Eigen::Vector3d cam_u,
                                     const Eigen::Vector3d cam_v,
                                     const Eigen::Vector3d cam_w,
                                     const Eigen::Vector3d origin) {
    Eigen::Vector3d translated = p1 - origin;

    double x_cam = translated.dot(cam_u);
    double y_cam = translated.dot(cam_v);
    double z_cam = -translated.dot(cam_w);
    return Eigen::Vector3d(x_cam, y_cam, z_cam);
}
inline std::array<Eigen::Vector3d, 3>
proj_tri(const triangle tri, const Eigen::Vector3d cam_u,
         const Eigen::Vector3d cam_v, const Eigen::Vector3d cam_w,
         const Eigen::Vector3d origin, const vertex_buffer &v_buff) {
    Eigen::Vector3d p1 = v_buff.get(tri.point1);
    Eigen::Vector3d p2 = v_buff.get(tri.point2);
    Eigen::Vector3d p3 = v_buff.get(tri.point3);

    Eigen::Vector3d proj_1 = project_point(p1, cam_u, cam_v, cam_w, origin);
    Eigen::Vector3d proj_2 = project_point(p2, cam_u, cam_v, cam_w, origin);
    Eigen::Vector3d proj_3 = project_point(p3, cam_u, cam_v, cam_w, origin);
    return std::array<Eigen::Vector3d, 3>{proj_1, proj_2, proj_3};
}
// bound_box() runs on world coordinates on the plane of interest
inline bound_box<double> w_box(const Eigen::Vector3d p1,
                               const Eigen::Vector3d p2,
                               const Eigen::Vector3d p3,
                               const double aspect_ratio) {
    double hor_min = std::min({p1[0], p2[0], p3[0]});
    double ver_min = std::min({p1[1], p2[1], p3[1]});
    double hor_max = std::max({p1[0], p2[0], p3[0]});
    double ver_max = std::max({p1[1], p2[1], p3[1]});

    // image planes are bound by the box: [-aspect_ratio, aspect_ratio, -1, 1]
    if (hor_min < -aspect_ratio) {
        hor_min = -aspect_ratio;
    }
    if (hor_max > aspect_ratio) {
        hor_max = aspect_ratio;
    }
    if (ver_min < -1) {
        ver_min = -1;
    }
    if (ver_max > 1) {
        ver_max = 1;
    }
    bound_box<double> bbox{hor_min, hor_max, ver_min, ver_max};
    return bbox;
}
// create an bound box in terms of pixels on an image_buffer
inline bound_box<int> create_box(const Eigen::Vector3d p1,
                                 const Eigen::Vector3d p2,
                                 const Eigen::Vector3d p3,
                                 const double aspect_ratio,
                                 const int img_length, const int img_width) {
    bound_box<double> w_bbox = w_box(p1, p2, p3, aspect_ratio);
    int left_pixel = std::ceil((w_bbox.min_x / aspect_ratio + 1) * img_length);
    int right_pixel = std::ceil((w_bbox.max_x / aspect_ratio + 1) * img_length);
    int top_pixel = std::ceil(1 - 0.5 * (w_bbox.min_y + 1) * img_width);
    int bot_pixel = std::ceil(1 - 0.5 * (w_bbox.max_y + 1) * img_width);
    return bound_box<int>{left_pixel, right_pixel, top_pixel, bot_pixel};
}

// checks if a point is inside of a triangle, the points must be on a plane
inline bool is_in_tri(const Eigen::Vector3d p1, const Eigen::Vector3d p2,
                      const Eigen::Vector3d p3, const Eigen::Vector3d test) {
    // 2D cross product / edge function
    auto cross2d = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
        return a.x() * b.y() - a.y() * b.x();
    };
    Eigen::Vector2d p1_2d = p1.head<2>();
    Eigen::Vector2d p2_2d = p2.head<2>();
    Eigen::Vector2d p3_2d = p3.head<2>();
    Eigen::Vector2d test_2d = test.head<2>();
    bool is_pos1 = cross2d(p2_2d - p1_2d, test_2d - p1_2d) >= 0;
    bool is_pos2 = cross2d(p3_2d - p2_2d, test_2d - p2_2d) >= 0;
    bool is_pos3 = cross2d(p1_2d - p3_2d, test_2d - p3_2d) >= 0;
    return (is_pos1 == is_pos2) && (is_pos2 == is_pos3);
}

template <typename T> struct ctx {
    int i;
    int j;
    buffer<T> &buff;
};

// functor for calls
struct rast_tri_fn {
    template <typename T>
    void operator()(ctx<T> &c, buffer<double> &z_tile,
                    const std::array<Eigen::Vector3d, 3> &p_tri, const T &val,
                    int length = 0, int width = 0) {
        if (length == 0 && width == 0)
            rast_tri(c.i, c.j, c.buff, z_tile, p_tri, val);
        else
            rast_tri(c.i, c.j, c.buff, z_tile, p_tri, val, length, width);
    }
};

template <typename T, typename Func, typename... Args>
void on_buff(buffer<T> &buff, Func &&job, Args &&...args) {
    int buff_length = buff.get_length();
    int buff_width = buff.get_width();
    for (int i = 0; i < buff_length; ++i) {
        for (int j = 0; j < buff_width; ++j) {
            ctx<T> ctx{i, j, buff};
            std::forward<Func>(job)(ctx, std::forward<Args>(args)...);
        }
    }
};

template <typename T, typename Func, typename... Args>
void on_tile(Func &&job, tile<T> &tile, Args &&...args) {
    int tile_length = tile.get_length(); // tiles are squares
    for (int i = 0; i < tile_length; ++i) {
        for (int j = 0; j < tile_length; ++j) {
            ctx<T> ctx{i, j, tile};
            std::forward<Func>(job)(ctx, std::forward<Args>(args)...);
        }
    }
}

template <typename T>
void rast_tri(const int i, const int j, buffer<T> &tile, buffer<double> &z_tile,
              const std::array<Eigen::Vector3d, 3> &p_tri, const T &val,
              const int paren_len, const int paren_wid,
              const point &top_l = point{.x = 0, .y = 0},
              const double alpha = 1.0 / 3.0, const double beta = 1.0 / 3.0,
              const double gamma = 1.0 / 3.0) {

    static double eps = 0.0001;
    static double sum = alpha + beta + gamma - 1;
    static bool sums = std::abs(sum) <= eps;
    if (!sums) {
        throw std::invalid_argument("Enter valid barycentric weights");
    }

    static bool len_eq = (tile.get_length() == z_tile.get_length());
    if (!len_eq) {
        throw std::invalid_argument("Tiles must be the same size");
    }

    const double a_ratio = (double)paren_len / paren_wid;
    int sub_pixel_x = i + top_l.x;
    int sub_pixel_y = j + top_l.y;

    double world_x = (double)sub_pixel_x / paren_len * 2 * a_ratio - a_ratio;
    double world_y = (double)sub_pixel_y / paren_wid * 2 - 1;

    Eigen::Vector3d test{world_x, world_y, 0};
    bool is_in = is_in_tri(p_tri[0], p_tri[1], p_tri[2], test);

    if (!is_in) {
        return;
    }
    double z_rep = alpha * (1.0 / p_tri[0][2]) + beta * (1.0 / p_tri[1][2]) +
                   gamma * (1.0 / p_tri[2][2]);
    double z_sub =
        (z_rep > 0) ? (1.0 / z_rep) : std::numeric_limits<double>::max();
    if (z_tile.get(i, j) > z_sub) {
        z_tile.set(i, j, z_sub);
        tile.set(i, j, val);
    }
}

// NOTE: potential bug, where this wrapper may be a recursive call
template <typename T>
void rast_tri(const int i, const int j, buffer<T> &buff, buffer<double> &z_buff,
              const std::array<Eigen::Vector3d, 3> &p_tri, const T &val,
              const point &top_l = point{.x = 0, .y = 0},
              const double alpha = 1.0 / 3.0, const double beta = 1.0 / 3.0,
              const double gamma = 1.0 / 3.0) {
    rast_tri(i, j, buff, z_buff, p_tri, val, buff.get_length(),
             buff.get_width(), top_l, alpha, beta, gamma);
}
#endif
