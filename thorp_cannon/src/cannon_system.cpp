/*
 * Author: Jorge Santos
 */

#include <atomic>
#include <chrono>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace thorp::cannon
{

/**
 * Simulate Thorp's cannon firing: while the trigger topic is true, shoot the rocket model at the given rate of
 * fire, placing it at the cannon muzzle and launching it along the axis of fire. The launch speed is the one
 * that shoot_force would give the rocket if applied during one simulation step.
 * Each shot takes three steps, as physics applies link velocity commands on every step while they exist, rotated
 * by the model pose before any pose command: place the rocket at the muzzle, stopped; command the launch velocity;
 * remove the velocity commands, so the rocket flies freely.
 */
class CannonSystem : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& sdf,
                 gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*event_mgr*/) override
  {
    model_ = gz::sim::Model(entity);

    std::string axis_of_fire = sdf->Get<std::string>("axis_of_fire", "x").first;
    rate_of_fire_ = sdf->Get<double>("rate_of_fire", rate_of_fire_).first;
    shoot_force_ = sdf->Get<double>("shoot_force", shoot_force_).first;
    rocket_model_name_ = sdf->Get<std::string>("rocket_model", rocket_model_name_).first;
    std::string cannon_link_name = sdf->Get<std::string>("cannon_link", "cannon_link").first;
    std::string trigger_topic = sdf->Get<std::string>("trigger_topic", "/arbotix/cannon_trigger").first;

    cannon_link_ = model_.LinkByName(ecm, cannon_link_name);
    if (cannon_link_ == gz::sim::kNullEntity)
    {
      gzerr << "Cannon link " << cannon_link_name << " not found in model " << model_.Name(ecm)
            << "; cannon disabled" << std::endl;
      return;
    }

    if (axis_of_fire == "x")
      direction_of_fire_ = gz::math::Vector3d::UnitX;
    else if (axis_of_fire == "y")
      direction_of_fire_ = gz::math::Vector3d::UnitY;
    else if (axis_of_fire == "z")
      direction_of_fire_ = gz::math::Vector3d::UnitZ;
    else
    {
      gzerr << "Invalid axis of fire " << axis_of_fire << "; cannon disabled" << std::endl;
      return;
    }

    node_.Subscribe(trigger_topic, &CannonSystem::onTrigger, this);
    gzdbg << "Thorp cannon system ready; trigger topic: " << trigger_topic << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override
  {
    if (info.dt < std::chrono::steady_clock::duration::zero())
    {
      // time went backwards, i.e. the simulation was reset; restart firing variables
      firing_ = false;
      last_shot_time_ = -1.0;
      shot_phase_ = ShotPhase::IDLE;
    }

    if (info.paused || cannon_link_ == gz::sim::kNullEntity)
    {
      return;
    }

    switch (shot_phase_)
    {
      case ShotPhase::PLACE:
        launchRocket(ecm, std::chrono::duration<double>(info.dt).count());
        shot_phase_ = ShotPhase::LAUNCH;
        return;
      case ShotPhase::LAUNCH:
        releaseRocket(ecm);
        shot_phase_ = ShotPhase::IDLE;
        return;
      case ShotPhase::IDLE:
        break;
    }

    // Trigger pressed; fire if we meet our rate of fire
    double now = std::chrono::duration<double>(info.simTime).count();
    if (firing_ && now - last_shot_time_ >= 1.0 / rate_of_fire_ && placeRocket(ecm))
    {
      shot_phase_ = ShotPhase::PLACE;
      last_shot_time_ = now;
    }
  }

private:
  void onTrigger(const gz::msgs::Boolean& msg)
  {
    firing_ = msg.data();
    gzdbg << "Trigger! " << (firing_ ? "ON" : "OFF") << std::endl;
  }

  bool placeRocket(gz::sim::EntityComponentManager& ecm)
  {
    auto rocket_entity =
        ecm.EntityByComponents(gz::sim::components::Name(rocket_model_name_), gz::sim::components::Model());
    if (rocket_entity == gz::sim::kNullEntity)
    {
      gzerr << rocket_model_name_ << " model not loaded; firing aborted" << std::endl;
      return false;
    }
    rocket_model_ = gz::sim::Model(rocket_entity);
    rocket_link_ = gz::sim::Link(rocket_model_.LinkByName(ecm, "link"));
    if (!ecm.Component<gz::sim::components::Inertial>(rocket_link_.Entity()))
    {
      gzerr << rocket_model_name_ << " model has no link named 'link' with inertia; firing aborted" << std::endl;
      return false;
    }

    // displace to the cannon muzzle, oriented as the cannon, and stop it
    gz::math::Pose3d cannon_pose = gz::sim::worldPose(cannon_link_, ecm);
    rocket_model_.SetWorldPoseCmd(ecm, cannon_pose * gz::math::Pose3d(0.05, 0.0, 0.0, 0.0, 0.0, 0.0));
    rocket_link_.SetLinearVelocity(ecm, gz::math::Vector3d::Zero);
    rocket_link_.SetAngularVelocity(ecm, gz::math::Vector3d::Zero);
    return true;
  }

  void launchRocket(gz::sim::EntityComponentManager& ecm, double step_size)
  {
    // launch along the axis of fire, in the link frame
    auto inertial = ecm.Component<gz::sim::components::Inertial>(rocket_link_.Entity());
    double speed = shoot_force_ * step_size / inertial->Data().MassMatrix().Mass();
    rocket_link_.SetLinearVelocity(ecm, direction_of_fire_ * speed);
  }

  void releaseRocket(gz::sim::EntityComponentManager& ecm)
  {
    ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(rocket_link_.Entity());
    ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(rocket_link_.Entity());
  }

  enum class ShotPhase
  {
    IDLE,
    PLACE,
    LAUNCH
  };

  gz::sim::Model model_{ gz::sim::kNullEntity };
  gz::sim::Entity cannon_link_{ gz::sim::kNullEntity };
  gz::sim::Model rocket_model_{ gz::sim::kNullEntity };
  gz::sim::Link rocket_link_{ gz::sim::kNullEntity };
  ShotPhase shot_phase_ = ShotPhase::IDLE;
  gz::transport::Node node_;

  // Shooting configuration
  std::atomic<bool> firing_{ false };
  double shoot_force_ = 100.0;
  double rate_of_fire_ = 18.18;  // Hz, or 0.055s between shots
  double last_shot_time_ = -1.0;
  gz::math::Vector3d direction_of_fire_ = gz::math::Vector3d::UnitX;
  std::string rocket_model_name_ = "rocket";
};

}  // namespace thorp::cannon

GZ_ADD_PLUGIN(thorp::cannon::CannonSystem, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
