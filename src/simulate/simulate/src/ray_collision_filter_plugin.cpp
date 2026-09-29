#include <cstdint>
#include <memory>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/ode/ode_inc.h>
#include <gazebo/physics/ode/ODETypes.hh>
#include <gazebo/physics/ode/ODERayShape.hh>
#include <gazebo/sensors/RaySensor.hh>

namespace gazebo {

// Gazebo Classic's ODE multiray owns separate geoms for every beam. Changing
// only the parent collision does not change those geoms (or their ray space).
class RayCollisionFilterPlugin : public SensorPlugin {
 public:
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    const auto ray_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
    if (!ray_sensor || !sdf || !sdf->HasElement("own_category_bits")) {
      gzerr << "[ray_collision_filter] RaySensor and own_category_bits required\n";
      return;
    }
    const auto own_bits = sdf->Get<uint32_t>("own_category_bits");
    if (!own_bits || (own_bits & (own_bits - 1u)) ||
        (own_bits & (GZ_FIXED_COLLIDE | GZ_SENSOR_COLLIDE)) ||
        !(own_bits & GZ_ALL_COLLIDE)) {
      gzerr << "[ray_collision_filter] invalid/reserved robot category bit\n";
      return;
    }
    const auto world = physics::get_world(ray_sensor->WorldName());
    const auto parent = world ? world->EntityByName(ray_sensor->ParentName()) : nullptr;
    const auto link = boost::dynamic_pointer_cast<physics::Link>(parent);
    const auto shape = ray_sensor->LaserShape();
    if (!link || !shape || !shape->RayCount()) {
      gzerr << "[ray_collision_filter] cannot resolve model or laser rays\n";
      return;
    }
    std::vector<dGeomID> beams;
    for (unsigned int i = 0; i < shape->RayCount(); ++i) {
      const auto ray = boost::dynamic_pointer_cast<physics::ODERayShape>(shape->Ray(i));
      if (!ray || !ray->ODEGeomId()) {
        gzerr << "[ray_collision_filter] requires ODE ray physics\n";
        return;
      }
      beams.push_back(ray->ODEGeomId());
    }
    boost::recursive_mutex::scoped_lock lock(*world->Physics()->GetPhysicsUpdateMutex());
    // See https://www.ode.org/ode-latest-userguide.html (collision bitfields).
    // ODE uses OR for the two category/collide tests. Exclude SENSOR on the
    // body side too, or its all-bits mask admits even the robot's own rays.
    unsigned int count = 0;
    for (const auto &body_link : link->GetModel()->GetLinks()) {
      for (const auto &collision : body_link->GetCollisions()) {
        collision->SetCategoryBits(own_bits);
        collision->SetCollideBits(GZ_ALL_COLLIDE & ~GZ_SENSOR_COLLIDE);
        ++count;
      }
    }
    for (const auto beam : beams) {
      dGeomSetCategoryBits(beam, GZ_SENSOR_COLLIDE);
      dGeomSetCollideBits(beam, GZ_ALL_COLLIDE & ~own_bits & ~GZ_SENSOR_COLLIDE);
    }
    // The enclosing ray space participates in broad-phase filtering too.
    const auto space = dGeomGetSpace(beams.front());
    dGeomSetCategoryBits(reinterpret_cast<dGeomID>(space), GZ_SENSOR_COLLIDE);
    dGeomSetCollideBits(reinterpret_cast<dGeomID>(space),
                        GZ_ALL_COLLIDE & ~own_bits & ~GZ_SENSOR_COLLIDE);
    gzmsg << "[ray_collision_filter] " << ray_sensor->ScopedName()
          << " filters " << count << " own collisions across " << beams.size()
          << " beams; peer robots remain visible\n";
  }
};

GZ_REGISTER_SENSOR_PLUGIN(RayCollisionFilterPlugin)
}  // namespace gazebo
