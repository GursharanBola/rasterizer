#ifndef ENGINE_H
#define ENGINE_H

#include "projector.h"
#include "scene.h"

class engine {
  public:
    engine(scene &scene, camera &camera) : scene(scene), camera(camera) {};
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
    int x = p1(0);
    int y = p1(1);
    int z = p1(2);
    return x * cam_u + y * cam_v - z * cam_w + origin;
}
// bound_box() runs on world coordinates on a plane
inline bound_box<double> create_box(const Eigen::Vector3d p1,
                                    const Eigen::Vector3d p2,
                                    const Eigen::Vector3d p3) {
    double hor_min = std::min({p1[0], p2[0], p3[0]});
    double ver_min = std::min({p1[1], p2[1], p3[1]});
    double hor_max = std::max({p1[0], p2[0], p3[0]});
    double ver_max = std::max({p1[1], p2[1], p3[1]});
    bound_box<double> bbox{hor_min, hor_max, ver_min, ver_max};
    return bbox;
}
// checks if a point is inside of a triangle, the points must be on a plane
inline bool is_in_tri(const Eigen::Vector2d p1, const Eigen::Vector2d p2,
                      const Eigen::Vector2d p3, const Eigen::Vector2d test) {
    // 2D cross product / edge function
    auto cross2d = [](const Eigen::Vector2d &a, const Eigen::Vector2d &b) {
        return a.x() * b.y() - a.y() * b.x();
    };
    bool is_pos1 = cross2d(p2 - p1, test - p1) >= 0;
    bool is_pos2 = cross2d(p3 - p2, test - p2) >= 0;
    bool is_pos3 = cross2d(p1 - p3, test - p3) >= 0;
    return (is_pos1 == is_pos2) && (is_pos2 == is_pos3);
}

#endif
