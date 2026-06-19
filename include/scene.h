#ifndef SCENE_H
#define SCENE_H

#include "buffer.h"
#include "material.h"
#include "mesh.h"
#include <memory>
class scene {
  public:
    scene(const int image_width, const int image_height, const int num_samples)
        : image_width(image_width), image_height(image_height),
          num_samples(num_samples) {};

    void add_sphere(const Eigen::Vector3d &center, double radius,
                    const Eigen::Vector3d &color,
                    const std::shared_ptr<material> &mat);
    void add_quad(const Eigen::Vector3d &origin, const Eigen::Vector3d u,
                  const Eigen::Vector3d v, Eigen::Vector3d &color,
                  const std::shared_ptr<material> mat);

    // TODO: Create light and input constructor here
    void add_light();
    // TODO: Create the camera and input constructot here
    void render_scene();
    void clear();

  private:
    int image_width;
    int image_height;
    int num_samples;
    image_buffer img = image_buffer{image_width, image_height, num_samples};
    vertex_buffer v_buffer;
    std::vector<z_buffer> z_buffers;
    std::vector<seen_buffer> seen_buffers;
    std::vector<mesh> meshes;
};

#endif
