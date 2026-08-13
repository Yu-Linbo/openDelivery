#include <cstdint>
#include <memory>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo {

class RayCollisionFilterPlugin : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    const auto ray_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
    if (!ray_sensor) {
      gzerr << "[ray_collision_filter] parent is not a RaySensor\n";
      return;
    }

    if (!sdf || !sdf->HasElement("own_category_bits") ||
        !sdf->HasElement("shell_link")) {
      gzerr << "[ray_collision_filter] own_category_bits and shell_link are required\n";
      return;
    }

    const auto own_bits = sdf->Get<uint32_t>("own_category_bits");
    if (own_bits == 0u || (own_bits & (own_bits - 1u)) != 0u) {
      gzerr << "[ray_collision_filter] own_category_bits must contain one bit\n";
      return;
    }

    auto laser_shape = ray_sensor->LaserShape();
    auto ray_collision =
        boost::dynamic_pointer_cast<physics::Collision>(laser_shape->GetParent());
    auto world = physics::get_world(ray_sensor->WorldName());
    auto parent = world ? world->EntityByName(ray_sensor->ParentName()) : nullptr;
    auto sensor_link = boost::dynamic_pointer_cast<physics::Link>(parent);
    const auto shell_link_name = sdf->Get<std::string>("shell_link");
    physics::CollisionPtr shell_collision;
    if (sensor_link) {
      for (const auto &candidate : sensor_link->GetCollisions()) {
        if (candidate && candidate->GetName().find(shell_link_name + "_collision") !=
                             std::string::npos) {
          shell_collision = candidate;
          break;
        }
      }
    }

    if (!ray_collision || !shell_collision) {
      gzerr << "[ray_collision_filter] cannot resolve ray or shell collision for "
            << ray_sensor->ScopedName() << "\n";
      return;
    }

    shell_collision->SetCategoryBits(own_bits);
    shell_collision->SetCollideBits(GZ_ALL_COLLIDE);
    ray_collision->SetCategoryBits(GZ_SENSOR_COLLIDE);
    ray_collision->SetCollideBits(GZ_ALL_COLLIDE & ~own_bits);

    gzmsg << "[ray_collision_filter] " << ray_sensor->ScopedName()
          << " ignores own shell bit 0x" << std::hex << own_bits << std::dec
          << " and keeps peer shells visible\n";
  }
};

GZ_REGISTER_SENSOR_PLUGIN(RayCollisionFilterPlugin)

}  // namespace gazebo
