#ifndef PROJECTOR_H
#define PROJECTOR_H

#include "buffer.h"
#include "mesh.h"
#include <Eigen/Dense>
#include <algorithm>

// projector acts a utility for lights and cameras
class projector {
  public:
    projector(const Eigen::Vector3d &origin, const Eigen::Vector3d &cam_u,
              const Eigen::Vector3d &cam_v, const Eigen::Vector3d cam_w,
              const double focal_dist)
        : origin(origin), cam_u(cam_u), cam_v(cam_v), cam_w(cam_w),
          focal_dist(focal_dist) {};

    Eigen::Vector3d project_point(Eigen::Vector3d &p1) const {
        int x = p1(0);
        int y = p1(1);
        int z = p1(2);

        return x * cam_u + y * cam_v - z * cam_w + origin;
    }

    // NOTE: The type here is going to be a double since that is the default
    // type for Eigen::Vector3d is a double

    // bounding box using the first two elements to avoid type conversion
    bound_box<double> bounding_box(Eigen::Vector3d p1, Eigen::Vector3d p2,
                                   Eigen::Vector3d p3) const {

        double hor_min = std::min({p1[0], p2[0], p3[0]});
        double ver_min = std::min({p1[1], p2[1], p3[1]});

        double hor_max = std::max({p1[0], p2[0], p3[0]});
        double ver_max = std::max({p1[1], p2[1], p3[1]});

        bound_box<double> bbox{hor_min, hor_max, ver_min, ver_max};

        return bbox;
    }

    // checks if a point is isnide of a triangle, the points must be on a plane
    bool is_in_tri(Eigen::Vector2d p1, Eigen::Vector2d p2, Eigen::Vector2d p3,
                   Eigen::Vector2d test) {
        // 2D cross product / edge function
        auto cross2d = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
            return a.x() * b.y() - a.y() * b.x();
        };

        bool is_pos1 = cross2d(p2 - p1, test - p1) >= 0;
        bool is_pos2 = cross2d(p3 - p2, test - p2) >= 0;
        bool is_pos3 = cross2d(p1 - p3, test - p3) >= 0;

        return (is_pos1 == is_pos2) && (is_pos2 == is_pos3);
    }
    // Utility
    Eigen::Vector3d get_u() { return cam_u; }
    Eigen::Vector3d get_v() { return cam_v; }
    Eigen::Vector3d get_w() { return cam_w; }
    Eigen::Vector3d get_o() { return origin; }
    double get_f_dist() { return focal_dist; }

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
