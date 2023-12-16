#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>

#include <gazebo/gazebo.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <arbotix_msgs/Digital.h>

namespace gazebo
{

class ThorpCannonPlugin : public ModelPlugin
{
public:
  ThorpCannonPlugin() : ModelPlugin()
  {
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized                                                                                    
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    this->model = _model;
    GZ_ASSERT(this->model != nullptr, "Got nullptr model pointer!");
    this->world = _model->GetWorld();
    this->sdf = _sdf;
    GZ_ASSERT(this->sdf != nullptr, "Got nullptr SDF element pointer!");

    if (_sdf->HasElement("axis_of_fire"))
      this->axis_of_fire = _sdf->Get<std::string>("axis_of_fire");
    if (_sdf->HasElement("rate_of_fire"))
      this->rate_of_fire = _sdf->Get<double>("rate_of_fire");
    if (_sdf->HasElement("shoot_force"))
      this->shoot_force = _sdf->Get<double>("shoot_force");
    if (_sdf->HasElement("rocket_count"))
      this->rocket_count = _sdf->Get<double>("rocket_count");

    auto cannon_link_name = _sdf->HasElement("cannon_link") ? _sdf->Get<std::string>("cannon_link") : "cannon_link";
    ROS_DEBUG("Get link %s for model %s", cannon_link_name.c_str(), this->model->GetName().c_str());
    this->cannon_link = this->model->GetLink(cannon_link_name);
    if (!this->cannon_link)
    {
      ROS_ERROR_STREAM("Got nullptr for cannon model " << cannon_link_name << "; cannon disabled");
      return;
    }

    if (this->axis_of_fire == "x")
      this->direction_of_fire = ignition::math::Vector3d(this->shoot_force, 0, 0);
    if (this->axis_of_fire == "y")
      this->direction_of_fire = ignition::math::Vector3d(0, this->shoot_force, 0);
    if (this->axis_of_fire == "z")
      this->direction_of_fire = ignition::math::Vector3d(0, 0, this->shoot_force);


    this->trigger_sub = nh.subscribe<arbotix_msgs::Digital>("arbotix/cannon_trigger", 5,
                                                            &ThorpCannonPlugin::TriggerCB, this);

    this->update_connection = event::Events::ConnectWorldUpdateBegin(std::bind(&ThorpCannonPlugin::OnUpdate, this));

    ROS_DEBUG("Thorp cannon gazebo plugin ready....");
  }

  void TriggerCB(const arbotix_msgs::Digital::ConstPtr& msg)
  {
    this->firing = msg->value > 0;
    ROS_DEBUG("Trigger! %s", this->firing ? "ON" : "OFF");
  }

  void Reset()
  {
    this->reseting_plugin = true;
    ROS_ERROR("Reseted the simulation world, we restart firing variables");

    this->firing = false;
    this->next_rocket = 1;
    this->last_shot_time = -1.0;

    this->reseting_plugin = false;
  }

  // Called by the world update start event
  public: void OnUpdate()
  {
    if (this->reseting_plugin)
    {
      ROS_ERROR("Reseting in process, please wait...");
      return;
    }

    if (this->firing && this->next_rocket <= this->rocket_count)
    {
      // Trigger pressed and still have rockets; fire if we meet our rate of fire
      double new_secs = this->world->SimTime().Float();
      double delta = new_secs - this->last_shot_time;
      if (delta >= 1.0/this->rate_of_fire)
      {
        FireRocket(this->next_rocket++);
        this->last_shot_time = new_secs;
      }
    }
  }

  void WaitForseconds(float seconds_to_wait)
  {
    unsigned int microseconds;
    microseconds = seconds_to_wait * 1e6;
    ROS_DEBUG("Waiting for %f seconds",seconds_to_wait);
    usleep(microseconds);
    ROS_DEBUG("Done waiting...");
  }

  void FireRocket(int number)
  {
    std::string model_name = this->rocket_models_base_name + std::to_string(number);
    ROS_DEBUG("Loading rocket %d model: %s", number, model_name.c_str());
    auto rocket_model = this->world->ModelByName(model_name);
    if (!rocket_model)
    {
      ROS_ERROR_STREAM("Got nullptr for rocket model " << model_name << "; firing aborted");
      return;
    }

    auto cannon_pose = this->cannon_link->WorldPose();

    this->SetRocketPose(*rocket_model, cannon_pose);
    this->SetRocketForce(*rocket_model, cannon_pose);
  }

  void SetRocketPose(gazebo::physics::Model &rocket_model, const ignition::math::Pose3d &cannon_pose)
  {
    // displace to the cannon muzzle
    ignition::math::Pose3d rocket_pose = cannon_pose * ignition::math::Pose3d(0.05, 0.0, 0.0, 0.0, 0.0, 0.0);
    ROS_DEBUG("POSE [X, Y, Z, Roll, Pitch, Yaw] = [%f, %f, %f, %f, %f, %f]",
      rocket_pose.Pos().X(), rocket_pose.Pos().Y(), rocket_pose.Pos().Z(),
      rocket_pose.Rot().Euler().X(), rocket_pose.Rot().Euler().Y(), rocket_pose.Rot().Euler().Z());
    rocket_model.SetWorldPose(rocket_pose);
  }

  void SetRocketForce(gazebo::physics::Model &rocket_model, const ignition::math::Pose3d &cannon_pose)
  {
    auto force_vector = cannon_pose.Rot() * this->direction_of_fire;

    ROS_DEBUG("FORCE APPLIED [X, Y, Z] = [%f, %f, %f]", force_vector.X(), force_vector.Y(), force_vector.Z());
    rocket_model.GetLink("link")->SetForce(force_vector);
  }


  // Pointer to the update event connection
  private: event::ConnectionPtr update_connection;
  
  /// \brief World pointer.
  protected: gazebo::physics::WorldPtr world;
  /// \brief Model pointer.
  protected: gazebo::physics::ModelPtr model;
  /// \brief Link pointer.
  protected: gazebo::physics::LinkPtr cannon_link;
  /// \brief SDF pointer.
  protected: sdf::ElementPtr sdf;

  // ROS interface
  ros::NodeHandle nh;
  ros::Subscriber trigger_sub;

  // Shooting configuration
  bool firing = false;
  uint8_t next_rocket = 1;
  uint8_t rocket_count = 6;
  double shoot_force = 100;
  double rate_of_fire = 18.18;  // Hz, or 0.055s between shots
  double last_shot_time = -1.0;
  ignition::math::Vector3d direction_of_fire;
  std::string axis_of_fire = "x";

  // Reseting Flag
  bool reseting_plugin = false;
  
  std::string rocket_models_base_name = "rocket";
};

GZ_REGISTER_MODEL_PLUGIN(ThorpCannonPlugin)
}
