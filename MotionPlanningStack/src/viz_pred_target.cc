#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

namespace gazebo
{
class VizPredictedTarget : public ModelPlugin
{
public:
    geometry_msgs::PoseStamped pred_pose;
    bool received_pose = false;
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;
        world_ = this->model->GetWorld();

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&VizPredictedTarget::OnUpdate, this));

        previous_time = world_->RealTime();

        this->pred_pose.pose.position.x = 0;
        this->pred_pose.pose.position.y = 0;
        this->pred_pose.pose.position.z = 0;
    }

    void pose_cb(const geometry_msgs::PoseStamped& msg)
    {
        this->pred_pose = msg;
    }


    // Called by the world update start event
public:
    void OnUpdate()
    {
        // Apply a small linear velocity to the modeltNsm=

//        if ((world_->RealTime() - previous_time) > 2.0)
//        if (this->pred_pose != NULL)
//        {
        this->model->SetGravityMode(false);
        ignition::math::Quaterniond current_rot(0, 0, 0, 1);
        auto new_pos = ignition::math::Vector3d(this->pred_pose.pose.position.x,
                                           this->pred_pose.pose.position.y,
                                           this->pred_pose.pose.position.z);
        ignition::math::Pose3d new_pose(new_pos, current_rot);
        this->model->SetWorldPose(new_pose);
        this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
        this->model->SetGravityMode(false);
        previous_time = world_->RealTime();
//        }

        //sleep(1.0);

        //current_pos[2] = 1.1;
        //current_pose.Set(current_pos, current_rot);
    }

    // Pointer to the model
private:
    physics::ModelPtr model;
    gazebo::physics::WorldPtr world_;
    common::Time previous_time;

    // Pointer to the update event connection
private:
    event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(VizPredictedTarget)
} // namespace gazebo
