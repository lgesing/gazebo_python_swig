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

#include "string"
#include "iostream"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/sensors/ContactSensor.hh"
#include "sdf/sdf.hh"

#include "ros/ros.h"
//#include "ros/init.h"

#include "custom_main.h"





Gazebo_Sim::Gazebo_Sim(int argc, char *argv[]) {
	printf("Init Gazebo_Sim\n");

/*
	ros::init(argc, argv, "gazebo_sim");
	
	nh_.reset(new ros::NodeHandle("~"));



*/
	gazebo::setupServer(argc, argv);
	//gazebo::setupServer();

	running = false;

	robot_model = NULL;

	gazebo_maxStepSize = 0.001;




	max_acceleration_linear_  = 0.7;
	max_acceleration_angular_ = 0.7;

	prismaticJoint = NULL;
	revoluteJoint = NULL;

	contact_sensor = NULL;

	input_vel_x = 0.0;
	input_vel_y = 0.0;
	input_vel_theta = 0.0;

}

Gazebo_Sim::~Gazebo_Sim() {
	std::cout <<  "Shutdown gazebo_server" << std::endl;

	// Close everything.
	gazebo::shutdown();




	//nh_->shutdown();
}






void Gazebo_Sim::setRobotPose(double x, double y, double z, double roll, double pitch, double yaw)
{
	robot_model->SetWorldPose(gazebo::math::Pose(x, y, z, roll, pitch, yaw));

   	//std::cout << "set pos: " << x << ", " << y  << ", " << z << std::endl;
    //std::cout << "robot pos: " << robot_model->GetWorldPose().pos << std::endl;

	gazebo::physics::ModelPtr disk_blue_model =  world->GetModel("disk_blue");
	disk_blue_model->SetWorldPose(robot_model->GetWorldPose());

}


void Gazebo_Sim::setGoalPose(double x, double y, double z, double roll, double pitch, double yaw)
{

	gazebo::physics::ModelPtr disk_green_model =  world->GetModel("disk_green");
	disk_green_model->SetWorldPose(gazebo::math::Pose(x, y, z, roll, pitch, yaw));
}

void Gazebo_Sim::getRobotVelocity(double* seq, int n)
{


	seq[0] = robot_model->GetRelativeLinearVel().x;
	seq[1] = robot_model->GetRelativeLinearVel().y;
	seq[2] = robot_model->GetRelativeLinearVel().z;
}



void Gazebo_Sim::loadRobot(std::string model_file)
{
    int modelCountBefore = world->GetModelCount();

	world->InsertModelFile(model_file);
    std::cout << "Loaded robot sdf-file: " << model_file << std::endl;




    while(world->GetModelCount() < modelCountBefore + 1){
    	world->RunBlocking(1);
    	gazebo::sensors::run_once();
    }



    if(world->GetModelCount() == modelCountBefore + 1){
        std::cout << "Successfully inserted model! " << std::endl;

        robot_model = world->GetModel(modelCountBefore);


        prismaticJoint = robot_model->GetJoint("base_link_linear_joint");
        revoluteJoint  = robot_model->GetJoint("base_link_angular_joint");

        generic_sensor = gazebo::sensors::get_sensor("default::simple_robot::base_link::base_link_contact");
        contact_sensor = std::dynamic_pointer_cast<gazebo::sensors::ContactSensor>(generic_sensor);



        phantom_link_ = world->GetPhysicsEngine()->CreateLink(robot_model);




		config.reset(new sdf::Element);
		config->Copy(robot_model->GetLink("base_link")->GetSDF()->Clone());
		config->GetAttribute("name")->Set("__perfect_phantom_link__");
		// Remove visuals/collisions/inertials

		//ROS_INFO("ELEMENT Description Count: %d;", config->GetElementDescriptionCount());
		for(int h=0;h<config->GetElementDescriptionCount();++h){
			//ROS_INFO("ELEMNT NAME %s", config->GetElementDescription(h)->GetName().c_str());
		}
		while (config->HasElement("visual")) {
			config->RemoveChild(config->GetElement("visual"));
		}

		while (config->HasElement("sensor")) {
			config->RemoveChild(config->GetElement("sensor"));
		}

		while (config->HasElement("collision")) {
			config->RemoveChild(config->GetElement("collision"));
		}

		while (config->HasElement("inertial")) {
			config->RemoveChild(config->GetElement("inertial"));
		}
		//ROS_INFO("HEREHREHREHRE");
		for(int h=0;h<config->GetElementDescriptionCount();++h){
			ROS_INFO("ELEMNT NAME %s", config->GetElementDescription(h)->GetName().c_str());
		}
		phantom_link_->Load(config);
		phantom_link_->Init();


		// create prismatic joint with parent "world" and child phantomLink
		prismaticJoint = world->GetPhysicsEngine()->CreateJoint("prismatic");
		prismaticJoint->SetName(robot_model->GetName() + "__perfect_vel_lin_joint__");
		gazebo::physics::LinkPtr worldLink = boost::dynamic_pointer_cast<gazebo::physics::Link>(
				world->GetByName("world"));
		gazebo::math::Pose prismaticJointOrigin;
		prismaticJoint->Load(worldLink, phantom_link_, prismaticJointOrigin);
		prismaticJoint->Init();

		// create revolute joint with parent phantomLink and child _link
		revoluteJoint = world->GetPhysicsEngine()->CreateJoint("revolute");
		revoluteJoint->SetName(robot_model->GetName() + "__perfect_vel_ang_joint__");
		gazebo::math::Pose revoluteJointOrigin;
		revoluteJoint->Load(this->phantom_link_, robot_model->GetLink("base_link"), revoluteJointOrigin);
		revoluteJoint->Init();












    } else {
        std::cout << "ERROR: Failed inserting model" << std::endl;
    }



}


void Gazebo_Sim::loadWorld(std::string world_file)
{
    // Load a world
    world = gazebo::loadWorld(world_file);
    world->Stop();

    std::cout << "Loaded world-file: " << world_file << std::endl;
}



void Gazebo_Sim::updateRobot(){

	gazebo::math::Pose pose = robot_model->GetWorldPose();

	//Vec3 new_vel = robot_model->GetLink("base_link")->GetRelativeLinearVel() + vel_linear_cmd * gazebo_maxStepSize;
	//robot_model->GetLink("base_link")->SetLinearVel(new_vel);

/*
	Vec3 vel_linear_cmd_new_ = Vec3(0,0,0);
	Vec3 vel_angular_cmd_new_ = Vec3(0,0,0);

	float yaw = pose.rot.GetYaw();
	Vec3 vel_linear_cmd = Vec3(	vel_linear_cmd_new_.x * cosf(yaw) - vel_linear_cmd_new_.y * sinf(yaw),
													vel_linear_cmd_new_.y * cosf(yaw) + vel_linear_cmd_new_.x * sinf(yaw), 0);
	Vec3 vel_angular_cmd = Vec3(0, 0, vel_angular_cmd_new_.z);
*/


	Vec3 vel_linear_cmd_new_  = Vec3(0,0,0);
	Vec3 vel_angular_cmd_new_ = Vec3(0,0,0);


	// velocity x
	if (fabs(input_vel_x - last_vel_linear_.x) < 0.001) {
		vel_linear_cmd_new_.x = input_vel_x;
	} else {
		if (input_vel_x >= last_vel_linear_.x)
			vel_linear_cmd_new_.x = fmin(input_vel_x,  last_vel_linear_.x + max_acceleration_linear_ * gazebo_maxStepSize);
		else
			vel_linear_cmd_new_.x = fmax(input_vel_x,  last_vel_linear_.x - max_acceleration_linear_ * gazebo_maxStepSize);
	}

	// velocity y
	if (fabs(input_vel_y - last_vel_linear_.y) < 0.001) {
		vel_linear_cmd_new_.y = input_vel_y;
	} else {
		if (input_vel_y >= last_vel_linear_.y)
			vel_linear_cmd_new_.y = fmin(input_vel_y,  last_vel_linear_.y + max_acceleration_linear_ * gazebo_maxStepSize);
		else
			vel_linear_cmd_new_.y = fmax(input_vel_y,  last_vel_linear_.y - max_acceleration_linear_ * gazebo_maxStepSize);
	}

	// velocity theta
	if (fabs(input_vel_theta - last_vel_angular_.z) < 0.001) {
		vel_angular_cmd_new_.z = input_vel_theta;
	} else {
		if (input_vel_theta >= last_vel_angular_.z)
			vel_angular_cmd_new_.z = fmin(input_vel_theta, last_vel_angular_.z + max_acceleration_angular_ * gazebo_maxStepSize);
		else
			vel_angular_cmd_new_.z = fmax(input_vel_theta, last_vel_angular_.z - max_acceleration_angular_ * gazebo_maxStepSize);
	}



	float yaw = pose.rot.GetYaw();
	Vec3 vel_linear_cmd = Vec3(	vel_linear_cmd_new_.x * cosf(yaw) - vel_linear_cmd_new_.y * sinf(yaw),
								vel_linear_cmd_new_.y * cosf(yaw) + vel_linear_cmd_new_.x * sinf(yaw), 0);
	Vec3 vel_angular_cmd = Vec3(0, 0, vel_angular_cmd_new_.z);


	double linearMagnitude = vel_linear_cmd.GetLength();
	prismaticJoint->SetAxis(0, vel_linear_cmd.Normalize());
	# if GAZEBO_MAJOR_VERSION >= 5
		prismaticJoint->SetParam("fmax", 0, 100.0);
	# else
		prismaticJoint->SetMaxForce(0, 100);
	# endif
	prismaticJoint->SetVelocity(0, linearMagnitude);


	double angularMagnitude = vel_angular_cmd.GetLength();
	revoluteJoint->SetAxis(0, vel_angular_cmd.Normalize());
	# if GAZEBO_MAJOR_VERSION >= 5
		revoluteJoint->SetParam("fmax", 0, 100.0);
	# else
		revoluteJoint->SetMaxForce(0, 100);
	# endif
	revoluteJoint->SetVelocity(0, angularMagnitude);



	last_vel_linear_ = vel_linear_cmd_new_;
	last_vel_angular_ = vel_angular_cmd_new_;




}




void Gazebo_Sim::executeAction(double vel_x, double vel_y, double vel_theta){


	input_vel_x = vel_x;
	input_vel_y = vel_y;
	input_vel_theta = vel_theta;



	//vel_linear_cmd.Set(acc_x, acc_y, 0.0);
	//vel_angular_cmd.Set(0.0, 0.0, acc_theta);


}


bool Gazebo_Sim::checkCollision(){

	//std::cout <<  "   contact_sensor Number " << contact_sensor->Contacts().contact_size() <<  std::endl;
	//std::cout <<  "   contact_sensor  " << contact_sensor->GetCollisionContactCount("room_closed_10m_10m_single_obstacle::walls::collision") <<  std::endl;


	// if contact is detected
	if(contact_sensor->GetCollisionContactCount("room_closed_10m_10m_single_obstacle::walls::collision") > 0){
		return true;
		gazebo::sensors::disable();
	} else {
		return false;
	}

}





bool Gazebo_Sim::makeSteps(int steps){
	if(running){





		int i = 0;
		while(i < steps){
			updateRobot();

			gazebo::runWorld(world, 1);
			gazebo::sensors::run_once(true);

			if(checkCollision()){
				return true;
			}

			i++;
		}
		return false;
/*
		gazebo::runWorld(world, steps);
		//world->RunBlocking(steps)
		//		Was ist besser;
		//world->Step(steps);
		//gazebo::sensors::run_once(true);
		//gazebo::sensors::run_threads();


		gazebo::sensors::run_once();


		//std::cout <<  " rel_vel: "<< robot_model->GetRelativeLinearVel() <<  std::endl;

		if(checkCollision()){
			return true;
		} else {
			return false;
		}
*/







		//gazebo::common::Time::MSleep(1);
		//vel_linear_cmd.Set(0,0,0);

		//ros::spinOnce();
	}
}



void Gazebo_Sim::stop(){
	running = false;
	gazebo::sensors::disable();
}


void Gazebo_Sim::reset(){
	running = true;
	gazebo::sensors::enable();

	last_vel_linear_.Set(0.0,0.0,0.0);
	last_vel_angular_.Set(0.0,0.0,0.0);
}



//void Gazebo_Sim::start(int _argc, char **_argv){
void Gazebo_Sim::start(){
	// Initialize gazebo.


	running = true;

	// Make sure the sensors are updated once before running the world.
	// This makes sure plugins get loaded properly.
	gazebo::sensors::run_once(true);

	// Run the sensor threads
	gazebo::sensors::run_threads();


	//robot_model->SetWorldPose(gazebo::math::Pose(1.0, 5.0, 0.2, 0,0,0));




	world->GetPhysicsEngine()->SetMaxStepSize(gazebo_maxStepSize);
	world->GetPhysicsEngine()->SetRealTimeUpdateRate(0.0);



	// This is your custom main loop. In this example the main loop is just a
	// for loop with 2 iterations.
/*
	while(running)
	{
	// Run simulation for 100 steps.
	makeSteps(1);
	//std::cout << "RUN()\n";
	
	//gazebo::common::Time::MSleep(1);

	}
*/

}




