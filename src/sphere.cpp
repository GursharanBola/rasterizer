#include "buffer.h"
#include "mesh.h"
#include <Eigen/Dense>

// TODO: Debug sphere::build
void sphere::build(vertex_buffer &v_buffer) {
    int i_index = v_buffer.size(); // first open index
    double delta_phi = EIGEN_PI / num_samples;
    double delta_theta = (2 * EIGEN_PI) / num_samples;
    Eigen::Vector3d center = get_center();
    Eigen::Vector3d start_p(0, get_radius(), 0);

    // add top vertex
    v_buffer.add(start_p + center);
    for (int i = 1; i < num_samples - 1; i++) {
        double c_phi = delta_phi * i;

        Eigen::Vector3d curr_ver_cir(
            start_p(0) * std::cos(c_phi) - start_p(2) * std::sin(c_phi),
            start_p(1),
            start_p(0) * std::sin(c_phi) + start_p(2) * std::cos(c_phi));

        int curr_row = i * num_samples;

        for (int j = 0; j < num_samples; j++) {
            double c_theta = delta_theta * j;

            double x = curr_ver_cir(0);
            double y = curr_ver_cir(1);
            double z = curr_ver_cir(2);

            Eigen::Vector3d curr_hor_circ(
                x * std::cos(c_theta) - y * std::sin(c_theta),
                x * std::sin(c_theta) + y * std::cos(c_theta), z);

            v_buffer.add(curr_hor_circ + center);
        }
    }
    // add the bottom vertex
    v_buffer.add(-start_p);
    // make top triangles
    for (int k = 0; k < num_samples; k++) {
        triangle triangle;
        triangle.point1 = i_index;
        triangle.point2 = i_index + k;
        triangle.point3 = triangle.point2++;

        list_of_triangles.push_back(triangle);
    }
    // make bottom triangles
    for (int k = 0; k < num_samples; k++) {
        triangle triangle;
        triangle.point1 = i_index + num_samples * (num_samples - 2);
        triangle.point2 = i_index + num_samples * (num_samples - 1) + k;
        triangle.point3 = triangle.point2++;
    }
    // middle layers of the triangle
    for (int k = 1; k < num_samples - 1; k++) {
        for (int l = 0; l < num_samples; l++) {
            triangle triangle1;
            triangle triangle2;
            int prev_row = i_index + num_samples * (k - 1) + l;
            int curr_row = i_index + num_samples * k + l;
            triangle1.point1 = prev_row;
            triangle1.point2 = curr_row;
            triangle1.point3 = curr_row++;

            triangle2.point1 = prev_row++;
            triangle2.point2 = curr_row;
            triangle2.point3 = curr_row++;
        }
    }
}

Eigen::Vector3d sphere::find_normal(const Eigen::Vector3d point) const {
    return (point - center).normalized();
}
