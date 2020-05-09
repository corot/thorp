#include <unistd.h>
#include <boost/bind.hpp>

#include <ros/ros.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <arbotix_msgs/Digital.h>

namespace gazebo
{

class ParticleShooterPlugin : public ModelPlugin
{
public:
  ParticleShooterPlugin() : ModelPlugin()
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
    GZ_ASSERT(this->model != NULL, "Got NULL model pointer!");
    this->sdf = _sdf;
    GZ_ASSERT(this->sdf != NULL, "Got NULL SDF element pointer!");

    if (_sdf->HasElement("origin_link"))
      this->origin_link = _sdf->Get<std::string>("origin_link");
    if (_sdf->HasElement("axis_of_fire"))
      this->axis_of_fire = _sdf->Get<std::string>("axis_of_fire");
    if (_sdf->HasElement("rate_of_fire"))
      this->rate_of_fire = _sdf->Get<double>("rate_of_fire");
    if (_sdf->HasElement("shoot_force"))
      this->shoot_force = _sdf->Get<double>("shoot_force");
    if (_sdf->HasElement("rocket_count"))
      this->rocket_count = _sdf->Get<double>("rocket_count");

    if (this->axis_of_fire == "x")
      this->direction_of_fire = ignition::math::Vector3d(this->shoot_force, 0, 0);
    if (this->axis_of_fire == "y")
      this->direction_of_fire = ignition::math::Vector3d(0, this->shoot_force, 0);
    if (this->axis_of_fire == "z")
      this->direction_of_fire = ignition::math::Vector3d(0, 0, this->shoot_force);


    this->trigger_sub = nh.subscribe<arbotix_msgs::Digital>("arbotix/cannon_trigger", 5,
                                                            &ParticleShooterPlugin::triggerCB, this);

    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ParticleShooterPlugin::OnUpdate, this));

    ROS_DEBUG("Particle Shooter Ready....");
  }

  void triggerCB(const arbotix_msgs::Digital::ConstPtr& msg)
  {
    this->firing = msg->value > 0;
    ROS_ERROR("Trigger! %s", this->firing ? "ON" : "OFF");
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
      ROS_ERROR("Reseting in Process, please wait...");
      return;
    }

    if (this->firing && this->next_rocket <= this->rocket_count)
    {
      // Trigger pressed and still have rockets; fire if we meet our rate of fire
      double new_secs = this->model->GetWorld()->SimTime().Float();
      double delta = new_secs - this->last_shot_time;
      if (delta >= 1.0/this->rate_of_fire)
      {
        FireRocket(this->next_rocket++);
//        ++this->next_rocket;
        this->last_shot_time = new_secs;
      }
    }
  }


  void WaitForseconds(float seconds_to_wait)
  {
    unsigned int microseconds;
    microseconds = seconds_to_wait * 1e6;
    ROS_WARN("Waiting for %f seconds",seconds_to_wait);
    usleep(microseconds);
    ROS_WARN("Done waiting...");

  }


  void FireRocket(int number)
  {
    std::string model_name = this->particle_base_name + std::to_string(number);
    ROS_WARN("Load rocket %d model: %s", number, model_name.c_str());
    auto rocket_model = this->model->GetWorld()->ModelByName(model_name);
    ROS_WARN("Model Name=%d", rocket_model.get());

    ROS_WARN("origin model=%s  link=%s", this->model->GetName().c_str(),this->origin_link.c_str());
    auto po_link = this->model->GetLink(this->origin_link);
    ROS_WARN("origin_model model=%s", po_link->GetName().c_str());
    auto po_pose = po_link->WorldPose();

    this->SetRocketPose(rocket_model, po_pose);
    this->SetRocketForce(rocket_model, po_pose);
  }


  void SetRocketPose(boost::shared_ptr<gazebo::physics::Model> rocket_model, const ignition::math::Pose3d &cannon_pose)
  {
    ignition::math::Pose3d rocket_pose = ignition::math::Pose3d(0.1, 0.0, 0.0, 0.0, 0.0, 0.0) * cannon_pose;
    ROS_WARN("POSE [X, Y, Z, Roll, Pitch, Yaw] [%f, %f, %f, %f, %f, %f], model=%s",
      rocket_pose.Pos().X(), rocket_pose.Pos().Y(), rocket_pose.Pos().Z(),
      rocket_pose.Rot().Euler().X(), rocket_pose.Rot().Euler().Y(), rocket_pose.Rot().Euler().Z(),
             rocket_model->GetName().c_str());
    rocket_model->SetWorldPose(rocket_pose);
  }
  
  
  void SetRocketForce(boost::shared_ptr<gazebo::physics::Model> rocket_model, const ignition::math::Pose3d &cannon_pose)
  {
    auto force_vector = cannon_pose.Rot() * this->direction_of_fire;

    ROS_WARN("FORCE APPLIED[X,Y,Z]=[%f,%f,%f]", force_vector.X(), force_vector.Y(), force_vector.Z());
    rocket_model->GetLink("link")->SetForce(force_vector);
  }


  // Pointer to the update event connection
  private: event::ConnectionPtr updateConnection;
  
  /// \brief World pointer.
  protected: gazebo::physics::ModelPtr model;
  /// \brief SDF pointer.
  protected: sdf::ElementPtr sdf;
  /// \brief Maps model IDs to ModelNames
  private: std::map<int, std::string> modelIDToName;

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
  std::string origin_link = "cannon_link";

  // Reseting Flag
  bool reseting_plugin = false;
  
  std::string particle_base_name = "particle_sphere";

};

GZ_REGISTER_MODEL_PLUGIN(ParticleShooterPlugin)
}