#include "mesh.h"
#include <Eigen/Dense>

void quad::build(vertex_buffer &v_buffer) {
    int i_index = v_buffer.size();

    v_buffer.add(origin);
    v_buffer.add(origin + u);
    v_buffer.add(origin + v);
    v_buffer.add(origin + u + v);

    // just add the indicies in v_buffer
    triangle triangle1{
        .point1 = i_index, .point2 = i_index++, .point3 = i_index += 2};
    triangle triangle2{
        .point1 = i_index++, .point2 = i_index += 2, .point3 = i_index += 3};
}
Eigen::Vector3d quad::find_normal(const Eigen::Vector3d point) const {
    return u.cross(v).normalized();
}
