#include "engine.h"
#include "buffer.h"
#include <cmath>
#include <stdexcept>
#include <vector>

// TODO: debug engine::fill_v_s and later add multithreading
void engine::fill_v_s(const projector &projector,
                      const std::vector<std::unique_ptr<mesh>> &meshes,
                      const vertex_buffer &v_buff, z_buffer &z_buff,
                      seen_buffer &s_buff) const {

    Eigen::Vector3d cam_u = projector.get_u();
    Eigen::Vector3d cam_v = projector.get_v();
    Eigen::Vector3d cam_w = projector.get_w();
    Eigen::Vector3d cam_o = projector.get_o();

    int length = z_buff.get_length();
    int width = z_buff.get_width();
    int sqrt_samples = z_buff.get_sqrt_samples();
    double aspect_ratio = static_cast<double>(length) / width;

    if (length != s_buff.get_length() || width != s_buff.get_width() ||
        s_buff.get_sqrt_samples() != sqrt_samples) {
        std::runtime_error(
            "s_buff must have same width, length, and sqrt_samples");
    }

    // create a tiles so the program can use it for cache blocking purposes
    int sqrt_tile = 4; // length of tile, generates a 16 pixel tile
    tile<double> z_tile{sqrt_tile, sqrt_samples};
    tile<tri_ref> s_tile{sqrt_tile, sqrt_samples};
    for (const auto &mesh : meshes) {
        int tri_index = 0;
        for (triangle tri : mesh->list_of_triangles) {

            std::array<Eigen::Vector3d, 3> p_tri =
                proj_tri(tri, cam_u, cam_v, cam_w, cam_o, v_buff);

            bound_box b_box = create_box(p_tri[0], p_tri[1], p_tri[2],
                                         aspect_ratio, length, width);

            int box_length = b_box.max_x - b_box.min_x;
            int box_width = b_box.max_y - b_box.min_y;

            // chunk into tiles
            for (int i = 0; i < std::ceil(box_length / sqrt_tile); i++) {
                for (int j = 0; j < std::ceil(box_width / sqrt_tile); j++) {
                    point tile_coords = point{i, j};
                    s_tile.pull(s_buff, b_box, tile_coords);
                    z_tile.pull(z_buff, b_box, tile_coords);

                    int top_l_x = b_box.min_x + i * sqrt_tile;
                    int top_l_y = b_box.max_y + j * sqrt_tile;
                    point top_l = point{top_l_x, top_l_y};

                    tri_ref tri = tri_ref{mesh->get_id(), tri_index};

                    raster_on_tile(z_tile, s_tile, p_tri, tri, length, width);

                    s_tile.push(s_buff, b_box, tile_coords);
                    z_tile.push(z_buff, b_box, tile_coords);
                }
            }
            tri_index++;
        }
    }
};
