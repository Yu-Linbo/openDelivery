// Standalone integration fixture loaded only by verify_robot_runtime.py.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/SensorManager.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/CameraSensor.hh>

namespace gazebo {
class RobotModelValidation : public WorldPlugin {
 public:
  void Load(physics::WorldPtr world, sdf::ElementPtr config) override {
    world_ = world;
    output_ = config->Get<std::string>("output_dir");
    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&RobotModelValidation::Step, this));
  }

 private:
  void Step() {
    const double time = world_->SimTime().Double();
    if (time < 2 || finished_) return;
    const auto robot = world_->ModelByName("reverse_probe");
    const auto probe = world_->ModelByName("probe");
    if (!robot || !probe) return;
    if (!captured_) {
      for (const auto &sensor : sensors::SensorManager::Instance()->GetSensors()) {
        const auto ray = std::dynamic_pointer_cast<sensors::RaySensor>(sensor);
        if (ray) rays_[sensor->ScopedName()] = ray;
      }
      if (rays_.size() != 3) return;
      for (const auto &item : rays_) {
        std::vector<double> ranges;
        item.second->Ranges(ranges);
        if (ranges.empty()) return;
        // The central rearward beam crosses our shell. Side beams can see
        // the other two robots around 2.8 m away and must NOT be filtered.
        if (item.first.find("::reverse_probe::") != std::string::npos)
          own_range_ = ranges[ranges.size() / 2];
        if (item.first.find("::probe::") != std::string::npos)
          peer_range_ = *std::min_element(ranges.begin(), ranges.end());
      }
      for (const auto &sensor : sensors::SensorManager::Instance()->GetSensors()) {
        const auto camera = std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
        if (camera && (sensor->ScopedName().find("::probe::") != std::string::npos ||
                       sensor->ScopedName().find("::inspection::") != std::string::npos)) {
          if (camera->SaveFrame(output_ + "/" + sensor->Name() + ".png")) ++frames_;
        }
      }
      initial_ = robot->WorldPose().Pos();
      captured_ = true;
    }
    double linear = 0, angular = 0;
    if (time < 5) linear = 0.3;
    else if (time < 6) linear = 0;
    else if (time < 9) linear = -0.25;
    else if (time < 14) angular = 1;
    else if (time < 17) { linear = 0.2; angular = -0.6; }
    // Exercise real wheel/ground contacts independently of ROS discovery.
    robot->GetJoint("left_wheel_joint")->SetVelocity(0, (linear-angular*0.13)/0.06);
    robot->GetJoint("right_wheel_joint")->SetVelocity(0, (linear+angular*0.13)/0.06);
    const auto pose = robot->WorldPose();
    max_tilt_ = std::max(max_tilt_, std::max(std::abs(pose.Rot().Pitch()),
                                           std::abs(pose.Rot().Roll())));
    for (const auto &item : rays_) {
      if (item.first.find("::reverse_probe::") == std::string::npos) continue;
      std::vector<double> ranges;
      item.second->Ranges(ranges);
      min_range_ = std::min(min_range_, *std::min_element(ranges.begin(), ranges.end()));
    }
    // Drive into a fixed peer to prove physical collisions still stop us.
    if (time > 19) {
      probe->GetJoint("left_wheel_joint")->SetVelocity(0, 0.4/0.06);
      probe->GetJoint("right_wheel_joint")->SetVelocity(0, 0.4/0.06);
    }
    if (time > 27) {
      const double distance = (pose.Pos()-initial_).Length();
      const double contact_x = probe->WorldPose().Pos().X();
      const bool ok = own_range_ > 11.9 && peer_range_ > 1.45 && peer_range_ < 1.7 &&
          max_tilt_ < 0.05236 && min_range_ > 0.5 && distance > 0.1 &&
          contact_x > 1.4 && contact_x < 1.58 && frames_ == 3;
      std::ofstream report(output_ + "/report.json");
      report << "{\n\"passed\":" << (ok ? "true" : "false")
             << ",\n\"peer_range\":" << peer_range_
             << ",\n\"own_body_center_range\":" << (std::isfinite(own_range_) ? std::to_string(own_range_) : "null")
             << ",\n\"own_body_clear\":" << (own_range_ > 11.9 ? "true" : "false")
             << ",\n\"motion_min_range\":" << min_range_
             << ",\n\"max_tilt_degrees\":" << max_tilt_*180/3.141592653589793
             << ",\n\"distance\":" << distance
             << ",\n\"contact_stop_x\":" << contact_x
             << ",\n\"camera_frames\":" << frames_ << "\n}\n";
      finished_ = true;
      world_->SetPaused(true);
    }
  }
  physics::WorldPtr world_;
  event::ConnectionPtr update_;
  std::string output_;
  bool captured_ = false, finished_ = false;
  unsigned int frames_ = 0;
  double max_tilt_ = 0, min_range_ = 12, peer_range_ = 0, own_range_ = 0;
  ignition::math::Vector3d initial_;
  std::map<std::string, sensors::RaySensorPtr> rays_;
};
GZ_REGISTER_WORLD_PLUGIN(RobotModelValidation)
}  // namespace gazebo
