#ifndef _CENTAURO_PLUGIN_HH_
#define _CENTAURO_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <boost/function.hpp>
#include <boost/algorithm/string.hpp>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

#include <xbot_msgs/JointState.h>

using namespace std;

map<string,float> joint_pos_ref,joint_vel_ref;

namespace gazebo
{

  class CentauroPlugin : public ModelPlugin
  {
    public: CentauroPlugin() {}

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
        

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&CentauroPlugin::CentauroUpdate, this,_1));
        
        // Get the first joint. We are making an assumption about the model

        // Setup a P-controller, with a gain of 0.1.
        this->pid = common::PID(700, 20, 0);
        
        
        std::string joint_name;
        //std::string delimiter = "::";
        
        //size_t pos = 0;
        //std::string token;
        for(int i=0;i <_model->GetJointCount();i++)
        {
        this->joint = _model->GetJoints()[i];
        joint_name=this->joint->GetScopedName();
            
//         while ((pos = joint_name.find(delimiter)) != std::string::npos) {
//         token = joint_name.substr(0, pos);
//         joint_name.erase(0, pos + delimiter.length());
//         }
        joint_pos_ref[joint_name]=0.0;
        joint_vel_ref[joint_name]=0.0;
        }
      
      
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized())
    {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "centauro_client",
        ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("centauro_client"));


    // subscribe /xbotcore/joint_states.

    this->rosSub = this->rosNode->subscribe("/xbotcore/joint_states",5000,PositionCallback);

    }
    
    public: void CentauroUpdate(const common::UpdateInfo& _info){
      
        for(int i=0;i < model->GetJointCount();i++)
        {
        joint=model->GetJoints()[i];
        if(joint->GetScopedName()=="centauro::neck_velodyne")
            eff= 10.0*(joint_vel_ref[joint->GetScopedName()]-joint->GetVelocity(0));
        else
            eff= pid.GetPGain()*(joint_pos_ref[joint->GetScopedName()]-joint->GetAngle(0).Radian())+pid.GetDGain()*(joint_vel_ref[joint->GetScopedName()]-joint->GetVelocity(0));
        joint->SetForce(0,eff);
        }
    }
    
    
    public: static void PositionCallback(const xbot_msgs::JointState&_msg)
    {
       for (int k=0;k<_msg.link_position.size();k++)
       {
            
            string aux= "centauro::"+_msg.name[k];
            bool found=false;
            for(auto it_aux = joint_pos_ref.begin() ; it_aux != joint_pos_ref.end() && !found; it_aux++)
            {
                if(it_aux->first==aux)
                {
                    found=true;
                    joint_pos_ref[it_aux->first]=_msg.motor_position[k];
                    joint_vel_ref[it_aux->first]=_msg.motor_velocity[k];
//                     if(it_aux->first =="centauro::neck_velodyne")
//                     {
//                         cout << _msg.name[k]  <<"  R-->" << _msg.motor_position[k] << " " << _msg.motor_velocity[k] << endl;
//                         cout << it_aux->first <<"  W-->" << joint_pos_ref[it_aux->first] << " " << joint_vel_ref[it_aux->first] << endl;
//                     }
                }
            }
        }
    }
    
    private: 
    /// \brief Pointer to the model.
    physics::ModelPtr model;

    /// \brief Pointer to the joint.
    physics::JointPtr joint;

    /// \brief A PID controller for the joint.
    common::PID pid;
    
    event::ConnectionPtr updateConnection;
    double eff;
    
    /// \brief A node use for ROS transport
    unique_ptr<ros::NodeHandle> rosNode;

        
    /// \brief A ROS subscriber
    ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    
  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(CentauroPlugin)
}
#endif
