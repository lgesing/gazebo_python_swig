#ifndef _GAZEBO_SIM_H_
#define _GAZEBO_SIM_H_
/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
 *
*/
#include <string>
#include <iostream>
#include <cstdio>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

//#include <gazebo_msgs/SpawnModel.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/package.h>
#include <rosgraph_msgs/Clock.h>


typedef gazebo::math::Vector3 Vec3;

class Gazebo_Sim {
	private:
		gazebo::physics::WorldPtr world;

		gazebo::physics::ModelPtr robot_model;


		gazebo::sensors::SensorPtr generic_sensor;
		gazebo::sensors::ContactSensorPtr contact_sensor;

		gazebo::physics::LinkPtr phantom_link_;
		sdf::ElementPtr config;






		Vec3 last_vel_linear_;
		Vec3 last_vel_angular_;

		float input_vel_x;
		float input_vel_y;
		float input_vel_theta;



		float max_acceleration_linear_;
		float max_acceleration_angular_;

		gazebo::physics::JointPtr prismaticJoint;
		gazebo::physics::JointPtr revoluteJoint;


		bool running;
		boost::shared_ptr<ros::NodeHandle> nh_;
		ros::CallbackQueue gazebo_queue_;
		std::string robot_namespace_;
		ros::ServiceServer spawn_urdf_model_service_;

		double gazebo_maxStepSize;

	public:
		Gazebo_Sim(int argc, char *argv[]);
		~Gazebo_Sim();
	    bool makeSteps(int steps);
	    void updateRobot();
	    void executeAction(double vel_x, double vel_y, double vel_theta);


	    //void getRobotVelocity(double *output_array, int length);
	    void getRobotVelocity(double* seq, int n);

		void setRobotPose(double x, double y, double z, double roll, double pitch, double yaw);
		void setGoalPose(double x, double y, double z, double roll, double pitch, double yaw);

		void loadRobot(std::string model_file);
		bool checkCollision();
		void start();
		void stop();
		void reset();
        void loadWorld(std::string world_file);
		//bool spawnURDFModel(gazebo_msgs::SpawnModel::Request &req, gazebo_msgs::SpawnModel::Response &res);
};

#endif  // _GAZEBO_SIM_H_
