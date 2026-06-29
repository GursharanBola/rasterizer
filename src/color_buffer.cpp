#include "buffer.h"

/**
 * Colors are rendered from [0,1] only internally are they actually represented
 * as 8 bit width channels, colors in this way is clever for later functions.
 * This idea was taken from RTIOW.
 */

Eigen::Vector3d color_buffer::get_color(const int i, const int j,
                                        const tri_ref tri) const {
    if (i < 0 || i >= get_width_p() || j < 0 || j >= get_length_p()) {
        return Eigen::Vector3d(0, 0, 0);
    }
    int sqrt_samples = this->get_sqrt_samples();
    int samples = sqrt_samples * sqrt_samples;
    int start_x = i * samples;

    for (int i = 0; i < samples; ++i) {
        mesh_color curr_mesh = this->get(start_x, i);
        if (tri == curr_mesh.tri) {
            return Eigen::Vector3d(curr_mesh.red / 255.0,
                                   curr_mesh.green / 255.0,
                                   curr_mesh.blue / 255.0);
        }
    }
    return Eigen::Vector3d(0, 0, 0);
}

bool color_buffer::set_color(const int i, const int j, const tri_ref tri,
                             Eigen::Vector3d color) {
    if (i < 0 || i >= get_width_p() || j < 0 || j >= get_length_p()) {
        return false;
    }
    int sqrt_samples = this->get_sqrt_samples();
    int samples = sqrt_samples * sqrt_samples;
    int start_x = i * samples;

    for (int i = 0; i < samples; ++i) {
        mesh_color curr_mesh = this->get(start_x, i);
        if (tri == curr_mesh.tri || curr_mesh.red == 0) {
            curr_mesh.tri = tri;
            curr_mesh.red = static_cast<int>(255.999 * color[0]);
            curr_mesh.green = static_cast<int>(255.999 * color[1]);
            curr_mesh.blue = static_cast<int>(255.999 * color[2]);

            return true;
        }
    }
    return false; // should never run by pidgeon hole, here for style
};
