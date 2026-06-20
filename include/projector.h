#ifndef PROJECTOR_H
#define PROJECTOR_H

#include "buffer.h"
#include "mesh.h"
#include <Eigen/Dense>
#include <algorithm>
#include <array>
// projector acts a utility for lights and cameras
class projector {
  public:
    projector(const Eigen::Vector3d &origin, const Eigen::Vector3d &cam_u,
              const Eigen::Vector3d &cam_v, const Eigen::Vector3d cam_w,
              const double focal_dist)
        : origin(origin), cam_u(cam_u), cam_v(cam_v), cam_w(cam_w),
          focal_dist(focal_dist) {};

    inline Eigen::Vector3d project_point(Eigen::Vector3d &p1) const {
        int x = p1(0);
        int y = p1(1);
        int z = p1(2);

        return x * cam_u + y * cam_v - z * cam_w + origin;
    }

    // bounding box using the first two elements to avoid type conversion
    inline std::array<int, 4> bounding_box(Eigen::Vector3d p1,
                                           Eigen::Vector3d p2,
                                           Eigen::Vector3d p3) const {
        int hor_min = (int)std::min({p1[0], p2[0], p3[0]});
        int hor_max = (int)std::max({p1[0], p2[0], p3[0]});
        int ver_min = (int)std::min({p1[1], p2[1], p3[1]});
        int ver_max = (int)std::max({p1[1], p2[1], p3[1]});

        std::array<int, 4> res =
            std::array<int, 4>{hor_min, hor_max, ver_min, ver_max};
        return res;
    }

    // Utility
    inline Eigen::Vector3d get_u() { return cam_u; }
    inline Eigen::Vector3d get_v() { return cam_v; }
    inline Eigen::Vector3d get_w() { return cam_w; }
    inline Eigen::Vector3d get_o() { return origin; }
    inline double get_f_dist() { return focal_dist; }

  private:
    Eigen::Vector3d cam_u;
    Eigen::Vector3d cam_v;
    Eigen::Vector3d cam_w;
    Eigen::Vector3d origin;
    double focal_dist;
};

class light : public projector {
  public:
    light(const Eigen::Vector3d &color, const Eigen::Vector3d &origin,
          const Eigen::Vector3d &cam_u, const Eigen::Vector3d &cam_v,
          const Eigen::Vector3d cam_w, const double focal_dist)
        : projector(origin, cam_u, cam_v, cam_w, focal_dist), color(color) {};

    // This will populate the camera's buffers which the Scene will create.
    void cast_light(const std::vector<mesh> meshes, z_buffer &z_buff,
                    seen_buffer &s_buff, const vertex_buffer &v_buffer);

  private:
    Eigen::Vector3d color;
};

class camera : public projector {};

#endif
