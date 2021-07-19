/*
 * Copyright 2015  Aurelien Roy
 * 
 * This file is a modified version of github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin.git
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 /**
  * @brief Parachute Plugin
  *
  * This plugin simulates parachute deployment
  *
  * @author Aurelien Roy  <aurroy@hotmail.com>
  */
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "gazebo_parachute_plugin.h"
#include <std_msgs/Bool.h>
#include <gazebo_ros_link_attacher/Attach.h>

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(StandAloneParachutePlugin)

StandAloneParachutePlugin::StandAloneParachutePlugin() : ModelPlugin()
{
}

StandAloneParachutePlugin::~StandAloneParachutePlugin()
{
  update_connection_->~Connection();
}

    void StandAloneParachutePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
      model_ = model;
//        model_ = GetModelPtr("box");
//        ROS_WARN("MODEL NAME IS %s",model->GetName());
      world_ = model_->GetWorld();

      namespace_.clear();
      if (sdf->HasElement("robotNamespace")) {
        namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
      } else {
        gzerr << "[gazebo_parachute_plugin] Please specify a robotNamespace.\n";
      }
	trigTopic_.clear();
	if (sdf->HasElement("parachuteTriggerTopic")) {
        trigTopic_ = sdf->GetElement("parachuteTriggerTopic")->Get<std::string>();
      } else {
        gzerr << "[gazebo_parachute_plugin] Please specify a parachute trigger topic.\n";
      }


      // Listen to the update event. This event is broadcast every simulation iteration.
      update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&StandAloneParachutePlugin::OnUpdate, this, _1));

      node_handle_ = transport::NodePtr(new transport::Node());
      node_handle_->Init(namespace_);

      if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc,argv,"parachute_plugin",ros::init_options::NoSigintHandler);
      }
      this->rosNode.reset(new ros::NodeHandle("parachute_plugin"));
      ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Bool>(
        trigTopic_,
        1,
        boost::bind(&StandAloneParachutePlugin::onRosMsg, this, _1),
        ros::VoidPtr(),
        &this->rosQueue
        );
       this->rosSub = this->rosNode->subscribe(so);
       this->rosQueueThread = std::thread(std::bind(&StandAloneParachutePlugin::QueueThread,this));
    }

    void StandAloneParachutePlugin::onRosMsg(const std_msgs::Bool::ConstPtr& _msg){
        this->trig = _msg->data;
    }

    void StandAloneParachutePlugin::OnUpdate(const common::UpdateInfo&){

        physics::ModelPtr parachute_model = GetModelPtr("parachute");
        if (this->trig) LoadParachute();
        if(!attached_parachute_ && parachute_model){
        AttachParachute(parachute_model); //Attach parachute to model
        attached_parachute_ = true;
        }
    }

    void StandAloneParachutePlugin::LoadParachute(){
        // Don't create duplicate paracutes
        physics::ModelPtr parachute_model = GetModelPtr("parachute");
        if(parachute_model){
		std::cout<<"*************** Parachute already exists.... *****************\n";
		return;
	}

        // Insert parachute model
        world_->InsertModelFile("model://parachute");
		
	std::cout<<"#######################PARACHUTE MODEL INSERTED!!!###################################\n";
        msgs::Int request;
        request.set_data(0);
	this->trig = 0;

    }

    physics::ModelPtr StandAloneParachutePlugin::GetModelPtr(std::string model_name){
        physics::ModelPtr model;

        #if GAZEBO_MAJOR_VERSION >= 9
        model = world_->ModelByName(model_name);
        #else
        model = world_->GetModel(model_name);
        #endif
        return model;

    }

    void StandAloneParachutePlugin::AttachParachute(physics::ModelPtr &parachute_model){
	
	ignition::math::Pose3d chute_pose = parachute_model->WorldPose();
	ROS_WARN("Parachute Spawned at [%f,%f,%f]",chute_pose.Pos().X(), chute_pose.Pos().Y(), chute_pose.Pos().Z());
        #if GAZEBO_MAJOR_VERSION >= 9
          ignition::math::Pose3d vehicle_pose = model_->WorldPose();

        #else
          ignition::math::Pose3d vehicle_pose = ignitionFromGazeboMath(model_->GetWorldPose()); //TODO(burrimi): Check tf.

        #endif
          parachute_model->SetWorldPose(ignition::math::Pose3d(vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z()+0.3, 0, 0, 0));        // or use uavPose.ros.GetYaw() ?
        ROS_WARN("Parachute reset to [%f,%f,%f]",vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z()+0.3);
	chute_pose = parachute_model->WorldPose();
	ROS_WARN("Parachute's new position: [%f,%f,%f]",chute_pose.Pos().X(), chute_pose.Pos().Y(), chute_pose.Pos().Z());
//        const gazebo::math::Pose pose;
//        pose = model_->GetWorldPose();
//        ROS_WARN("Actual Position = [%f,%f,%f]",pose.pos.x,pose.pos.y,pose.pos.z);
//        parachute_model->SetLinkWorldPose(gazebo::math::Pose(pose._x.pose._y,pose._z+0.3,0,0,0),)

//        #if GAZEBO_MAJOR_VERSION >= 9
//          gazebo::physics::JointPtr parachute_joint = world_->Physics()->CreateJoint("fixed", model_);
//        #else
//          gazebo::physics::JointPtr parachute_joint = world_->GetPhysicsEngine()->CreateJoint("fixed", model_);
//        #endif
//	
//	ROS_WARN("O");

//          parachute_joint->SetName("parachute_joint");
//	ROS_WARN("A");
//          // Attach parachute to base_link
//          gazebo::physics::LinkPtr base_link = model_->GetLink("base_link");
//	ROS_WARN("M");
//          gazebo::physics::LinkPtr parachute_link = parachute_model->GetLink("chute");
//	ROS_WARN("I");
//          parachute_joint->Attach(base_link, parachute_link);
//	ROS_WARN("PARACHUTE ATTACHED!!!");
//          // load the joint, and set up its anchor point
//          parachute_joint->Load(base_link, parachute_link, ignition::math::Pose3d(0, 0, 0.3, 0, 0, 0));
//	vehicle_pose = model_->WorldPose();
//	ROS_WARN("Probe before attachment at [%f,%f,%f]",vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z());
	chute_pose = parachute_model->WorldPose();
	ROS_WARN("Parachute before attachment at [%f,%f,%f]",chute_pose.Pos().X(), chute_pose.Pos().Y(), chute_pose.Pos().Z());
	ros::ServiceClient attachParachute = this->rosNode->serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
	gazebo_ros_link_attacher::Attach srv;
	srv.request.model_name_1 = model_->GetName();
	srv.request.link_name_1 = "base_link";
	srv.request.model_name_2 = "parachute";
	srv.request.link_name_2 = "chute";
	if (attachParachute.call(srv)){
		ROS_INFO("Parachute Attach Service call: %s!",srv.response.ok?"Success":"Failed");
	}
	else{
		ROS_INFO("Parachute Attach Service Failed!");	
	}
//	vehicle_pose = model_->WorldPose();
//	ROS_WARN("Probe after attachment at [%f,%f,%f]",vehicle_pose.Pos().X(), vehicle_pose.Pos().Y(), vehicle_pose.Pos().Z());
	chute_pose = parachute_model->WorldPose();
	ROS_WARN("Parachute after attachment at [%f,%f,%f]",chute_pose.Pos().X(), chute_pose.Pos().Y(), chute_pose.Pos().Z());
    }

    void StandAloneParachutePlugin::QueueThread()
		{
		  static const double timeout = 0.01;
		  while (this->rosNode->ok())
		  {
		    this->rosQueue.callAvailable(ros::WallDuration(timeout));
		  }
		}


} // namespace gazebo
