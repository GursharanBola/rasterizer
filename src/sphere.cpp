#include "mesh.h"
#include <Eigen/Dense>

void sphere::build() {
    double delta_phi = EIGEN_PI / num_samples;
    double delta_theta = (2 * EIGEN_PI) / num_samples;

}
Eigen::Vector3d sphere::find_normal(const Eigen::Vector3d point) const {
    return (point - center).normalized();
}
