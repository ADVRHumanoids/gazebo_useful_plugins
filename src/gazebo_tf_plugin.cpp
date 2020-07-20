#ifndef _GAZEBO_TF_PLUGIN_HH_
#define _GAZEBO_TF_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

#include <thread>
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
 #include <std_srvs/Empty.h>

#include <xbot_msgs/JointState.h>

using namespace std;

static bool send_fix_world=true;

namespace gazebo
{

  class GazeboTFPlugin : public ModelPlugin
  {
    public: GazeboTFPlugin() {}

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Safety check
        if (_model->GetJointCount() == 0)
        {
        std::cerr << "Invalid joint count, Centauro plugin not loaded\n";
        return;
        }

        // Store the model pointer for convenience.
        this->model = _model;
        
        
        std::string joint_name;
        
        for(int i=0;i <_model->GetJointCount();i++)
        {
        this->joint = _model->GetJoints()[i];
        joint_name=this->joint->GetScopedName();
        cout << joint_name << endl;
        }
    
        // Initialize ros, if it has not already bee initialized.
        if (!ros::isInitialized())
        {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_tfix_pub",
        ros::init_options::NoSigintHandler);
        }

        // Create our ROS node. This acts in a similar manner to
        // the Gazebo node
        this->rosNode.reset(new ros::NodeHandle("gazebo_tfix_pub"));


        // subscribe /xbotcore/joint_states.

        this->service = this->rosNode->advertiseService("pub_fix_world", pub_fix_world);
        
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboTFPlugin::TF_BroadCast, this,_1));
        
    }
    
     public: void TF_BroadCast(const common::UpdateInfo& _info){

        if(send_fix_world)
        {
            static tf::TransformBroadcaster br;
            tf::Transform transform;
            gazebo::math::Pose pose=model->GetWorldPose();
            transform.setOrigin( tf::Vector3(pose.pos.x,pose.pos.y,pose.pos.z) );
            tf::Quaternion q;
            q.setW(pose.rot.w);
            q.setX(pose.rot.x);
            q.setY(pose.rot.y);
            q.setZ(pose.rot.z);
            //q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fixed_world", "world"));
        }
    }
    public: static bool pub_fix_world(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res){
    
        send_fix_world=true;
        return true;
    }
    
    private: 
    /// \brief Pointer to the model.
    physics::ModelPtr model;
    
    event::ConnectionPtr updateConnection;

    /// \brief Pointer to the joint.
    physics::JointPtr joint;

    /// \brief A node use for ROS transport
    unique_ptr<ros::NodeHandle> rosNode;

        
    /// \brief A ROS subscriber
    ros::ServiceServer service;
    
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(GazeboTFPlugin)
}
#endif
