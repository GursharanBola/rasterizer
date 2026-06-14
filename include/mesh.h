#ifndef MESH_H
#define MESH_H

#include <Eigen/Dense>
#include <vector>

class mesh {
  public:
    std::vector<Eigen::Vector3d> list_of_triangles;

    mesh(int id, int samples,
         const Eigen::Vector3d &color = Eigen::Vector3d::Zero())
        : mesh_id(id), num_samples(samples), color(color) {}

    virtual ~mesh() = default;
    virtual void build() = 0;
    virtual Eigen::Vector3d find_normal(const Eigen::Vector3d point) const = 0;

    inline int get_id() const { return mesh_id; }
    inline int get_samples() const { return num_samples; }
    inline Eigen::Vector3d get_color() const { return color; }

  protected:
    int mesh_id;
    int num_samples;
    Eigen::Vector3d color;
};

class sphere : public mesh {
  public:
    sphere(const Eigen::Vector3d &center, double radius, int num_samples,
           int mesh_id, const Eigen::Vector3d &color)
        : mesh(mesh_id, num_samples, color), center(center), radius(radius) {}

    virtual void build() override;
    virtual Eigen::Vector3d
    find_normal(const Eigen::Vector3d point) const override;

    inline Eigen::Vector3d get_center() const { return center; }
    inline double get_radius() const { return radius; }

  private:
    Eigen::Vector3d center;
    double radius;
};

#endif
