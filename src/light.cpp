#include "projector.h"

void light::cast_light(const std::vector<mesh> meshes, z_buffer &z_buff,
                       seen_buffer &s_buff, const vertex_buffer &v_buff) {

    for (const auto &mesh : meshes) {
        for (triangle tri : mesh.list_of_triangles) {
            Eigen::Vector3d p1 = v_buff.get(tri.point1);
            Eigen::Vector3d p2 = v_buff.get(tri.point2);
            Eigen::Vector3d p3 = v_buff.get(tri.point3);

            Eigen::Vector3d proj_1 = project_point(p1);
            Eigen::Vector3d proj_2 = project_point(p2);
            Eigen::Vector3d proj_3 = project_point(p3);


        }
    }

    return;
};
