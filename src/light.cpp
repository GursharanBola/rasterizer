#include "buffer.h"
#include "projector.h"
#include <cmath>
#include <stdexcept>
#include <vector>

// TODO: DEBUG this
void light::cast_light(const std::vector<mesh> meshes, z_buffer &z_buff,
                       seen_buffer &s_buff, const vertex_buffer &v_buff) {
    int length = z_buff.get_length();
    int width = z_buff.get_width();
    int sqrt_samples = z_buff.get_sqrt_samples();
    double aspect_ratio = static_cast<double>(length) / width;
    Eigen::Vector3d origin = get_o();

    double alpha, beta, gamma = (double)1 / 3;

    // NOTE: s_buff and z_buff are for the same object the user should never
    // be able to cause this error, it is here for debugging
    if (length != s_buff.get_length() || width != s_buff.get_width() ||
        s_buff.get_sqrt_samples() != sqrt_samples) {
        throw std::runtime_error(
            "s_buff must have same width, length, and sqrt_samples");
    }

    // NOTE: These values are hardcoded, but for good reason you may think of it
    // as the universal yard stick for this program. It must be hard coded.
    double left_bound = -aspect_ratio;
    double right_bound = aspect_ratio;
    double top_bound = 1.0;
    double bottom_bound = -1.0;

    // create a tiles so the program can use it for cache blocking purposes
    int sqrt_tile = 4; // length of tile, generates a 16 pixel tile
    int num_rows = sqrt_tile * sqrt_samples;
    int num_cols = sqrt_tile * sqrt_samples;
    std::vector<std::vector<double>> z_tile(num_rows,
                                            std::vector<double>(num_cols));
    std::vector<std::vector<tri_ref>> s_tile(num_rows,
                                             std::vector<tri_ref>(num_cols));
    for (const auto &mesh : meshes) {
        int tri_index = 0;
        for (triangle tri : mesh.list_of_triangles) {
            Eigen::Vector3d p1 = v_buff.get(tri.point1);
            Eigen::Vector3d p2 = v_buff.get(tri.point2);
            Eigen::Vector3d p3 = v_buff.get(tri.point3);

            Eigen::Vector3d proj_1 = project_point(p1);
            Eigen::Vector3d proj_2 = project_point(p2);
            Eigen::Vector3d proj_3 = project_point(p3);

            bound_box b_box = bounding_box(p1, p2, p3);
            // check if the bounding box is actually on the scene.
            if (b_box.min_x < left_bound) {
                b_box.min_x = left_bound;
            }
            if (b_box.max_x > right_bound) {
                b_box.max_x = right_bound;
            }
            if (b_box.min_y < bottom_bound) {
                b_box.min_y = bottom_bound;
            }
            if (b_box.max_y > top_bound) {
                b_box.min_y = top_bound;
            }

            // bounding box is in world coordaintes need to convert to pixels
            int box_left_pixel =
                std::ceil((b_box.min_x / aspect_ratio + 1) * length);
            int box_right_pixel =
                std::ceil((b_box.max_x / aspect_ratio + 1) * length);
            int box_top_pixel = std::ceil(1 - 0.5 * (b_box.min_y + 1) * width);
            int box_bot_pixel = std::ceil(1 - 0.5 * (b_box.max_y + 1) * width);

            bound_box<int> int_bbox = bound_box<int>{
                box_left_pixel, box_right_pixel, box_top_pixel, box_bot_pixel};

            int box_length = box_right_pixel - box_left_pixel;
            int box_width = box_top_pixel - box_bot_pixel;

            // chunk into tiles
            for (int i = 0; i < std::ceil(box_length / sqrt_tile); i++) {
                for (int j = 0; j < std::ceil(box_width / sqrt_tile); j++) {
                    point tile_coords = point{i, j};
                    // NOTE: push and pull handle seg faults
                    s_buff.pull(s_tile, int_bbox, tile_coords);
                    z_buff.pull(z_tile, int_bbox, tile_coords);

                    // deterine the location of the tile
                    double top_left_x =
                        b_box.min_x + (double)(i * sqrt_tile) / length;
                    double top_left_y =
                        b_box.max_y + (double)(j * sqrt_tile) / length;

                    // iterate over the tile and populate
                    for (int k = 0; k < sqrt_samples * sqrt_tile; k++) {
                        for (int l = 0; l < sqrt_samples * sqrt_tile; l++) {
                            // check if each sub-pixel is within the triangle on
                            // the scene.
                            double sub_pixel_x =
                                top_left_x +
                                (double)i / (sqrt_samples * length);
                            double sub_pixel_y =
                                top_left_y + (double)j / (sqrt_samples * width);

                            Eigen::Vector2d p1{proj_1[0], proj_1[1]};
                            Eigen::Vector2d p2{proj_2[0], proj_2[1]};
                            Eigen::Vector2d p3{proj_3[0], proj_3[1]};
                            Eigen::Vector2d test{sub_pixel_x, sub_pixel_y};
                            bool is_in = is_in_tri(p1, p2, p3, test);

                            double z_rep =
                                alpha *
                                (1 / proj_1[2] + 1 / proj_2[2] + 1 / proj_3[2]);
                            double z_sub =
                                (z_rep > 0)
                                    ? (1.0 / z_rep)
                                    : std::numeric_limits<double>::max();

                            if (is_in) {
                                double z_rep =
                                    alpha * (1 / proj_1[2] + 1 / proj_2[2] +
                                             1 / proj_3[2]);
                                double z_sub =
                                    (z_rep > 0)
                                        ? (1.0 / z_rep)
                                        : std::numeric_limits<double>::max();

                                if (z_tile[k][l] > z_sub) {
                                    // update the tile if so
                                    z_tile[k][l] = z_sub;
                                    s_tile[k][l] =
                                        tri_ref{mesh.get_id(), tri_index};
                                }
                            }
                        }
                    }

                    s_buff.push(s_tile, int_bbox, tile_coords);
                    z_buff.push(z_tile, int_bbox, tile_coords);
                }
            }
        }
        tri_index++;
    }
};
