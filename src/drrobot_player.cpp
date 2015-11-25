/*!
 *  drrobot_jaguar4x4_player
 *  Copyright (c) 2011, Dr Robot Inc
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*!

@mainpage
  drrobot_jaguar4x4_player is a driver for motion control system on I90/Sentinel3/Hawk/H20/X80SV/Jaguar series mobile robot, available from
<a href="http://www.drrobot.com">Dr Robot </a>.
<hr>

@section usage Usage
@par     After start roscore, you need load robot configuration file to parameter server first.
          For example, I90 robot, you need load drrobotplayer_I90.yaml use command "rosparam load drrobotplayer_I90.yaml"
@verbatim
$ drrobot_jaguar4x4_player
@endverbatim

<hr>
@section topic ROS topics

Subscribes to (name/type):
- @b "cmd_vel"/Twist : velocity commands to differentially drive the robot.
- @b will develop other command subscribles in future, such as servo control.

Publishes to (name / type):
-@b drrobot_motor: will publish MotionInfoArray Message. Please referee the message file.
-@b drrobot_powerinfo: will publish PowerInfo Message. Please referee the message file.
-@b drrobot_ir: will publish RangeArray Message for IR sensor, and transform AD value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_sonar: will publish RangeArray Message for ultrasonic sensor, and transform value from DrRobotMotionSensorDriver to distance value in unit meter. Please referee the message file.
-@b drrobot_standardsensor: will publish StandardardSensor Message. Please referee the message file.
-@b drrobot_customsensor: will publish CustomSensor Message. Please referee the message file. Not available for standard I90/Sentinel3/Hawk/H20/X80SV robot

<hr>

@section parameters ROS parameters, please read yaml file

- @b RobotCommMethod (string) : Robot communication method, normally is "Network".
- @b RobotID (string) : specify the robot ID
- @b RobotBaseIP (string) : robot main WiFi module IP address in dot format, default is "192.168.0.201".
- @b RobotPortNum (string) : socket port number first serial port, and as default the value increased by one will be second port number.
- @b RobotSerialPort (int) : specify the serial port name if you choose serial communication in RobotCommMethod, default /dev/ttyS0"
- @b RobotType (string) : specify the robot type, now should in list: I90, Sentinel3, Hawk_H20, Jaguar, X80SV
- @b MotorDir (int) : specify the motor control direction
- @b WheelRadius (double) : wheel radius
- @b WheelDistance (double) : the distance between two driving wheels
- @b EncoderCircleCnt (int) : one circle encoder count
- @b MinSpeed (double) : minimum speed, unit is m/s.
- @b MaxSpeed (double) : maximum speed, unit is m/s.
- @b enable_ir (bool)  : Whether to enable sonar range sensors. Default: true.
- @b enable_sonar (bool)  : Whether to enable IR range sensors. Default: true.
 */

#include <assert.h>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include "tf/transform_broadcaster.h"
#include <tf/transform_listener.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>

#include <drrobot_jaguar4x4_player/MotorInfo.h>
#include <drrobot_jaguar4x4_player/MotorInfoArray.h>
#include <drrobot_jaguar4x4_player/RangeArray.h>
#include <drrobot_jaguar4x4_player/Range.h>
#include <drrobot_jaguar4x4_player/PowerInfo.h>
#include <drrobot_jaguar4x4_player/StandardSensor.h>
#include <drrobot_jaguar4x4_player/CustomSensor.h>
#include <DrRobotMotionSensorDriver.hpp>

#define MOTOR_NUM       6
#define IR_NUM          10
#define US_NUM          6
//#define M_PI   	3.1415927
using namespace std;
using namespace DrRobot_MotionSensorDriver;

class DrRobotPlayerNode
{
public:

    ros::NodeHandle node_;

    tf::TransformBroadcaster tf_;
    tf::TransformBroadcaster m_odom_broadcaster;

    ros::Publisher motorInfo_pub_;
    ros::Publisher powerInfo_pub_;
    ros::Publisher ir_pub_;
    ros::Publisher sonar_pub_;
    ros::Publisher standardSensor_pub_;
    ros::Publisher customSensor_pub_;

//19/11/2015
//additional decleration 
	ros::Publisher m_odom_pub;       
	ros::Publisher m_joint_state;
	
    ros::Subscriber cmd_vel_sub_;
    std::string robot_prefix_;
    
        // to keep track of robot:
     double m_x;
     double m_y;
     double m_theta ;


   DrRobotPlayerNode()
    {
        ros::NodeHandle private_nh("~");

        robotID_ = "drobot1";
        private_nh.getParam("RobotID",robotID_);
        ROS_INFO("I get ROBOT_ID: [%s]", robotID_.c_str());

        robotType_ = "Jaguar";
        private_nh.getParam("RobotType",robotType_);
        ROS_INFO("I get ROBOT_Type: [%s]", robotType_.c_str());

        robotCommMethod_ = "Network";
        private_nh.getParam("RobotCommMethod",robotCommMethod_);
        ROS_INFO("I get ROBOT_CommMethod: [%s]", robotCommMethod_.c_str());

        robotIP_ = "192.168.0.60";
        private_nh.getParam("RobotBaseIP",robotIP_);
        ROS_INFO("I get ROBOT_IP: [%s]", robotIP_.c_str());

        commPortNum_ = 10001;
        private_nh.getParam("RobotPortNum",commPortNum_);
        ROS_INFO("I get ROBOT_PortNum: [%d]", commPortNum_);

        robotSerialPort_ = "/dev/ttyS0";
        private_nh.getParam("RobotSerialPort",robotSerialPort_);
        ROS_INFO("I get ROBOT_SerialPort: [%s]", robotSerialPort_.c_str());

        enable_ir_ = false;
        private_nh.getParam("Enable_IR", enable_ir_);
        if (enable_ir_)
          ROS_INFO("I get Enable_IR: true");
        else
          ROS_INFO("I get Enable_IR: false");


        enable_sonar_ = false;
        private_nh.getParam("Enable_US", enable_sonar_);
        if (enable_sonar_)
          ROS_INFO("I get Enable_US: true");
        else
          ROS_INFO("I get Enable_US: false");

        motorDir_ = 1;
        private_nh.getParam("MotorDir", motorDir_);
        ROS_INFO("I get MotorDir: [%d]", motorDir_);

        wheelRadius_ = 0.135;
        private_nh.getParam("WheelRadius", wheelRadius_);
        ROS_INFO("I get Wheel Radius: [%f]", wheelRadius_);

        wheelDis_ = 0.52;
        private_nh.getParam("WheelDistance", wheelDis_);
        ROS_INFO("I get Wheel Distance: [%f]", wheelDis_);

        minSpeed_ = 0.1;
        private_nh.getParam("MinSpeed", minSpeed_);
        ROS_INFO("I get Min Speed: [%f]", minSpeed_);

        maxSpeed_ = 2.0;
        private_nh.getParam("MaxSpeed", maxSpeed_);
        ROS_INFO("I get Max Speed: [%f]", maxSpeed_);

        encoderOneCircleCnt_ = 150;
        private_nh.getParam("EncoderCircleCnt", encoderOneCircleCnt_);
        ROS_INFO("I get Encoder One Circle Count: [%d]", encoderOneCircleCnt_);

        if (robotCommMethod_ == "Network")
        {
          robotConfig1_.commMethod = Network;
          robotConfig2_.commMethod = Network;
        }
        else
        {
          robotConfig1_.commMethod = Serial;
          robotConfig2_.commMethod = Serial;
        }

        if (robotType_ == "Jaguar")
        {
          robotConfig1_.boardType = Jaguar;
        }
        else if(robotType_ == "I90")
        {
          robotConfig1_.boardType = I90_Power;
          robotConfig2_.boardType = I90_Motion;
        }
        else if (robotType_ == "Sentinel3")
        {
          robotConfig1_.boardType = Sentinel3_Power;
          robotConfig2_.boardType = Sentinel3_Motion;
        }
        else if (robotType_ == "Hawk_H20")
        {
          robotConfig1_.boardType = Hawk_H20_Power;
          robotConfig2_.boardType = Hawk_H20_Motion;
        }
        else if(robotType_ == "X80SV")
        {
          robotConfig1_.boardType = X80SV;
        }

        robotConfig1_.portNum = commPortNum_;
        robotConfig2_.portNum = commPortNum_ + 1;

      //  strcat(robotConfig1_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig1_.robotIP,robotIP_.c_str());
      //  strcat(robotConfig2_.robotIP,robotIP_.c_str());
	  strcpy(robotConfig2_.robotIP,robotIP_.c_str());

      //  strcat(robotConfig1_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig1_.serialPortName,robotSerialPort_.c_str());
      //  strcat(robotConfig2_.serialPortName,robotSerialPort_.c_str());
	  strcpy(robotConfig2_.serialPortName,robotSerialPort_.c_str());
        //create publishers for sensor data information
        motorInfo_pub_ = node_.advertise<drrobot_jaguar4x4_player::MotorInfoArray>("drrobot_motor", 1);
        powerInfo_pub_ = node_.advertise<drrobot_jaguar4x4_player::PowerInfo>("drrobot_powerinfo", 1);
        if (enable_ir_) { ir_pub_ = node_.advertise<drrobot_jaguar4x4_player::RangeArray>("drrobot_ir", 1); }
        if (enable_sonar_) { sonar_pub_ = node_.advertise<drrobot_jaguar4x4_player::RangeArray>("drrobot_sonar",1); }
        standardSensor_pub_ = node_.advertise<drrobot_jaguar4x4_player::StandardSensor>("drrobot_standardsensor", 1);
        customSensor_pub_ = node_.advertise<drrobot_jaguar4x4_player::CustomSensor>("drrobot_customsensor", 1);

		//Newly added by Abeje Y. Mersha 19/10/2015
        m_odom_pub = node_.advertise<nav_msgs::Odometry>("odom", 1);
        m_joint_state = node_.advertise<sensor_msgs::JointState>("joint_states", 1);

        drrobotPowerDriver_ = new DrRobotMotionSensorDriver();
        drrobotMotionDriver_ = new DrRobotMotionSensorDriver();
        if (  (robotType_ == "Jaguar") )
        {
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
        }
        else
        {
          drrobotPowerDriver_->setDrRobotMotionDriverConfig(&robotConfig1_);
          drrobotMotionDriver_->setDrRobotMotionDriverConfig(&robotConfig2_);
        }
        cntNum_ = 0;
    /*    
        actual_wheel_velocities_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("actual_wheel_velocities", 1);
        requested_wheel_velocities_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("requested_wheel_velocities", 1);
        smoothed_wheel_velocities_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("filtered_wheel_velocities", 1);
	    actual_wheel_positions_pub_ = node_.advertise<x80sv_driver::WheelVelocities>("actual_wheel_positions", 1);
     */

    }

    ~DrRobotPlayerNode()
    {
    }

    int start()
    {

      int res = -1;
      if (  (robotType_ == "Jaguar"))
      {
        res = drrobotMotionDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);
	if (res == 0)
	{
		ROS_INFO("open port number at: [%d]", robotConfig1_.portNum);
	}
	else
	{
		ROS_INFO("could not open network connection to [%s,%d]",  robotConfig1_.robotIP,robotConfig1_.portNum);
		//ROS_INFO("error code [%d]",  res);
	}

      }
      else
      {
        drrobotMotionDriver_->openNetwork(robotConfig2_.robotIP,robotConfig2_.portNum);
        drrobotPowerDriver_->openNetwork(robotConfig1_.robotIP,robotConfig1_.portNum);

      }

      cmd_vel_sub_ = node_.subscribe<geometry_msgs::Twist>("drrobot_cmd_vel", 1, boost::bind(&DrRobotPlayerNode::cmdVelReceived, this, _1));
        return(0);
    }

    int stop()
    {
        int status = 0;
         drrobotMotionDriver_->close();
        drrobotPowerDriver_->close();
        usleep(1000000);
        return(status);
    }

    void cmdVelReceived(const geometry_msgs::Twist::ConstPtr& cmd_vel)
    {
      double g_vel = cmd_vel->linear.x;
      double t_vel = cmd_vel->angular.z;
      if (robotConfig1_.boardType != Jaguar)
      {
        double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        //ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);
        drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity,leftWheelCmd, rightWheelCmd,NOCONTROL,NOCONTROL, NOCONTROL,NOCONTROL);
      }
      else
      {
	  
	// for Jaguar 4x4 differential drive -- old platform
// modified by Abeje Y. Mersha to vary the scalling of the velocity command according to the maximum velocity (approximately 6.5 m/s) of the setup
	     int maxEncoderVel=6 ; //in rev/s maximum velocity per second
	     int maxRobotVel=5.1 ; // in m/s maximum velocity per second
         int forwardPWM = -motorDir_ * g_vel* (maxSpeed_/maxRobotVel)* 16384 + 16384;
         int turnPWM = -motorDir_ * t_vel * (maxSpeed_/maxRobotVel)*16384 + 16384;

         if (forwardPWM > 32767) forwardPWM = 32767;
         if (forwardPWM < 0) forwardPWM = 0;
         if (turnPWM > 32767) turnPWM = 32767;
         if (turnPWM < 0) turnPWM = 0;
        
        double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);
         
         int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
         int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
         //ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);
         drrobotMotionDriver_->sendMotorCtrlAllCmd(PWM,NOCONTROL,NOCONTROL,NOCONTROL,forwardPWM,turnPWM, NOCONTROL);

/*
 * //  At the moment the independent drive does not work
	// for Jaguar 4x4 independent drive, channel 0 for left rear motor, channel 1 for right rear motor, channel 3 for left front motor, channel 4 for right front motor, here we will use velocity control

      double leftWheel = (2 * g_vel - t_vel* wheelDis_) / (2 * wheelRadius_);
        double rightWheel = (t_vel* wheelDis_ + 2 * g_vel) / (2 * wheelRadius_);

        int leftWheelCmd = motorDir_ * leftWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        int rightWheelCmd = - motorDir_ * rightWheel * encoderOneCircleCnt_ / ( 2* 3.1415927);
        ROS_INFO("Received control command: [%d, %d]", leftWheelCmd,rightWheelCmd);
        //drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity,leftWheelCmd, rightWheelCmd,NOCONTROL,leftWheelCmd, rightWheelCmd,NOCONTROL);
		//drrobotMotionDriver_->sendMotorCtrlAllCmd(Velocity,leftWheelCmd, rightWheelCmd,NOCONTROL,leftWheelCmd, rightWheelCmd*0,NOCONTROL);
		* 
*/
      }
      
      
    }
           // Convert encoder ticks to radians:
    double encoder2rad(int enc_value)
    {
        return (enc_value * 2 * M_PI) / encoderOneCircleCnt_;
    }
    
    void calculateMovementDeltaleft(drrobot_jaguar4x4_player::MotorInfo& mtr, int& encoderPrevious, double& movementDelta, double& joint_angle)
	{
        const uint encoderMax = 32768;

        if (encoderPrevious != -1)
        {
            int encoderDelta = 0;

            if (encoderPrevious < 150 && mtr.encoder_pos > 32660)
            {
                // When the encoder wrap around from lower to upper value
                // moving backwards. Encoderdelta must be negative number:
                encoderDelta = -((encoderMax - mtr.encoder_pos) + encoderPrevious);
            }
            else if (encoderPrevious > 32660 && mtr.encoder_pos < 150)
            {
                // When moving from top to bottom, we move forward, so
                // delta must be positive:
                encoderDelta = (encoderMax - encoderPrevious) + mtr.encoder_pos;
            }
            else
            {
                // The 'normal' case:
                encoderDelta = mtr.encoder_pos - encoderPrevious;
            }

            movementDelta = encoder2rad(encoderDelta);
        }
        else
        {
            movementDelta = 0;
        }

        joint_angle = encoder2rad(mtr.encoder_pos);
        encoderPrevious = mtr.encoder_pos;
	}
    
    // To be cleaned up and merged with the previous function
    void calculateMovementDeltaright(drrobot_jaguar4x4_player::MotorInfo& mtr, int& encoderPrevious, double& movementDelta, double& joint_angle)
    {
        const uint encoderMax = 32768;

        if (encoderPrevious != -1)
        {
            int encoderDelta = 0;

            if (encoderPrevious < 150 && mtr.encoder_pos > 32660)
            {
                // When the encoder wrap around from lower to upper value
                // moving backwards. Encoderdelta must be negative number:
                encoderDelta = -((encoderMax - mtr.encoder_pos) + encoderPrevious);
            }
            else if (encoderPrevious > 32660 && mtr.encoder_pos < 150)
            {
                // When moving from top to bottom, we move forward, so
                // delta must be positive:
                encoderDelta = (encoderMax - encoderPrevious) + mtr.encoder_pos;
            }
            else
            {
                // The 'normal' case:
                encoderDelta = mtr.encoder_pos - encoderPrevious;
            }

            movementDelta = encoder2rad(encoderDelta);
        }
        else
        {
            movementDelta = 0;
        }

        joint_angle = encoder2rad(mtr.encoder_pos);
        encoderPrevious = mtr.encoder_pos;
    }
    
        /*
        This function calculates odometry information from the encoderdata.  
        It creates a transform from 'odom' to 'base_footprint'

        motor 0 is the left wheel, motor 1 the right wheel.

        The x axis is forward, the y axis is to the left, and the z is upwards.

        The base footprint is exactly between the wheels. The base link is the center of the robot.
    */
    void publishOdometry(const drrobot_jaguar4x4_player::MotorInfoArray& motorInfo)
    {
        static int mEncoderPreviousLeftrear = -1; //TODO confirm left and right are correct
        static int mEncoderPreviousRightrear = -1;
        static int mEncoderPreviousLeftfront = -1; 
        static int mEncoderPreviousRightfront = -1;
        static ros::Time prev_time(0);

        ros::Time nu = ros::Time::now();

        double time_delta = (nu - prev_time).toSec();
        prev_time = nu;
        if (time_delta < 0.0001)
        {
            time_delta = 0.0001;
        }

        double leftrear_joint_angle;
        double rightrear_joint_angle;
        double leftfront_joint_angle;
        double rightfront_joint_angle;

        drrobot_jaguar4x4_player::MotorInfo mtr0 = motorInfo.motorInfos.at(0); // Leftrear motor
        drrobot_jaguar4x4_player::MotorInfo mtr1 = motorInfo.motorInfos.at(1); // Rightrear motor
        drrobot_jaguar4x4_player::MotorInfo mtr3 = motorInfo.motorInfos.at(3); // Leftfront motor
        drrobot_jaguar4x4_player::MotorInfo mtr4 = motorInfo.motorInfos.at(4); // Rightfront motor

        //ROS_INFO("Encoder values: %u, %u", mtr0.encoder_pos, mtr1.encoder_pos);
       //ROS_INFO("Motor Encoder Pos: [%u, %u, %u, %u, %u, %u]", motorInfo.motorInfos[0].encoder_pos, motorInfo.motorInfos[1].encoder_pos, motorInfo.motorInfos[3].encoder_pos, motorInfo.motorInfos[4].encoder_pos);
 /*       
             ROS_INFO("Motor Encoder Pos: [%d, %d, %d, %d, %d, %d]", msg->motorInfos[0].encoder_pos, msg->motorInfos[1].encoder_pos, msg->motorInfos[2].encoder_pos
               , msg->motorInfos[3].encoder_pos, msg->motorInfos[4].encoder_pos, msg->motorInfos[5].encoder_pos);
        
        drrobot_jaguar4x4_player::MotorInfo mtr0 = motorInfo.motorInfos.at(0); // Left motor
        drrobot_jaguar4x4_player::MotorInfo mtr1 = motorInfo.motorInfos.at(1); // Right motor

        ROS_INFO("Encoder values: %u, %u", motorInfo.motorInfos[0].encoder_pos, motorInfo.motorInfos[1].encoder_pos);
*/
        double leftrear_movement_delta;
        double rightrear_movement_delta;
        double leftfront_movement_delta;
        double rightfront_movement_delta;

        calculateMovementDeltaleft(mtr0, mEncoderPreviousLeftrear, leftrear_movement_delta, leftrear_joint_angle);
        calculateMovementDeltaright(mtr1, mEncoderPreviousRightrear, rightrear_movement_delta, rightrear_joint_angle);
		calculateMovementDeltaleft(mtr3, mEncoderPreviousLeftfront, leftfront_movement_delta, leftfront_joint_angle);
        calculateMovementDeltaright(mtr4, mEncoderPreviousRightfront, rightfront_movement_delta, rightfront_joint_angle);
        
        
        		//ROS_INFO("Angular movement: %f, %f, %f, %f", leftrear_movement_delta, rightrear_movement_delta, leftfront_movement_delta, rightfront_movement_delta);
//To do check if the wheel velocities are published!
    /*
        {
            // Publish wheel positions:
    	    drrobot_jaguar4x4_player::WheelVelocities actual_wheel_positions;
            actual_wheel_positions.left = left_joint_angle;
            actual_wheel_positions.right = right_joint_angle;
            actual_wheel_positions_pub_.publish(actual_wheel_positions);
        }
       

        {
            // Calculate and publish wheel velocities:
            drrobot_jaguar4x4_player::WheelVelocities actual_wheel_velocities;
            actual_wheel_velocities.left = left_movement_delta / time_delta;
            actual_wheel_velocities.right = right_movement_delta / time_delta;
            actual_wheel_velocities_pub_.publish(actual_wheel_velocities);
        }
	*/
//original 

        // TODO: take into account the wheel direction:
        double d_left = -((leftrear_movement_delta+leftfront_movement_delta)/2) * wheelRadius_;
        double d_right = ((rightrear_movement_delta+rightfront_movement_delta)/2) * wheelRadius_;

		int motorDir=1;
        // average distance between 2 wheels = actual distance from center
        double averageDistance = -motorDir*(d_left + d_right) / 2.0;
	
        // difference in angle from last encoder values, distance between wheel centers in m
        // When the right wheel moves forward, the angle
       ////////////// double deltaAngle = atan2((d_right - d_left), wheelDis_);
       double deltaAngle = -motorDir*(d_right - d_left)/wheelDis_;
        // ROS_INFO("Left movement %f, right %f, angle=%f", d_left, d_right, deltaAngle);
        
       /////// ROS_INFO("Left movement %f, right %f, angle=%f", d_left, d_right, deltaAngle);
     
        double delta_x = averageDistance * cos(m_theta);
        double delta_y = averageDistance * sin(m_theta);

        // x is in forward direction
        // y is sideways.
        // Angle is zero when facing forwards.

        // TODO: retrieve velocities:
        double vx = averageDistance / time_delta;
        double vy = 0;
        double vth = deltaAngle / time_delta;

        // update pose:
        m_theta += deltaAngle;
        m_x += delta_x;
        m_y += delta_y;
        
       //m_x=sin(nu.toSec());
 
       //////// ROS_INFO("Robot pos: %f, %f, %f", m_x, m_y, m_theta);
		//////////ROS_INFO("Robot delta_pos: %f, %f, %f", delta_x, delta_y, deltaAngle);
        // Send out joint state via topic:
        ////////////////// The joint name is a combination of the erratic wheel and the xacro description.
        // This is published into static tf via the robot_state_publisher.
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = nu;
        joint_state.name.resize(4);
        joint_state.position.resize(4);
        joint_state.name[0] = "base_link_leftrear_wheel_joint";
        joint_state.position[0] = leftrear_joint_angle;
        joint_state.name[1] = "base_link_rightrear_wheel_joint";
        joint_state.position[1] = rightrear_joint_angle;
        
        joint_state.name[2] = "base_link_leftfront_wheel_joint";
        joint_state.position[2] = leftfront_joint_angle;
        joint_state.name[3] = "base_link_rightfront_wheel_joint";
        joint_state.position[3] = rightfront_joint_angle;
        
        m_joint_state.publish(joint_state);

        // Grab some constant things:
        tf::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, m_theta);

        // Construct tf message:
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(m_x, m_y, 0.0));
        transform.setRotation(odom_quat);

        // Send via tf system:
        m_odom_broadcaster.sendTransform(tf::StampedTransform(transform, nu, "odom", "base_footprint"));

        // Construct odometry message:
        nav_msgs::Odometry odom;
        odom.header.stamp = nu;
        odom.header.frame_id = "odom";

        // Set position:
        odom.pose.pose.position.x = m_x;
        odom.pose.pose.position.y = m_y;

        geometry_msgs::Quaternion odom_quat2 = tf::createQuaternionMsgFromYaw(m_theta);
        odom.pose.pose.orientation = odom_quat2;

        // Set velocity:
        odom.child_frame_id = "base_footprint";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        // Publish the message:
        m_odom_pub.publish(odom);
        
     
    }

    void doUpdate()
    {
		    //The following two lines are important to declreat the motorInfoArray in the scope of this function
		      drrobot_jaguar4x4_player::MotorInfoArray motorInfoArray;
              motorInfoArray.motorInfos.resize(MOTOR_NUM);

      if ( (robotConfig1_.boardType == I90_Power) || (robotConfig1_.boardType == Sentinel3_Power)
          || (robotConfig1_.boardType == Hawk_H20_Power) )
      {
        if (drrobotPowerDriver_->portOpen())
        {
          drrobotPowerDriver_->readPowerSensorData(&powerSensorData_);
          drrobot_jaguar4x4_player::PowerInfo powerInfo;
          powerInfo.ref_vol = 1.5 * 4095 /(double)powerSensorData_.refVol;

          powerInfo.bat1_vol = (double)powerSensorData_.battery1Vol  * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.bat2_vol = (double) powerSensorData_.battery2Vol * 8 / 4095 * powerInfo.ref_vol;

          powerInfo.bat1_temp = powerSensorData_.battery1Thermo;
          powerInfo.bat2_temp = powerSensorData_.battery2Thermo;

          powerInfo.dcin_vol = (double)powerSensorData_.dcINVol * 8 / 4095 * powerInfo.ref_vol;
          powerInfo.charge_path = powerSensorData_.powerChargePath;
          powerInfo.power_path = powerSensorData_.powerPath;
          powerInfo.power_status = powerSensorData_.powerStatus;

          powerInfo_pub_.publish(powerInfo);
        }
      }
      if (drrobotMotionDriver_->portOpen())
      {
        drrobotMotionDriver_->readMotorSensorData(&motorSensorData_);
        drrobotMotionDriver_->readRangeSensorData(&rangeSensorData_);
        drrobotMotionDriver_->readStandardSensorData(&standardSensorData_);

        drrobotMotionDriver_->readCustomSensorData(&customSensorData_);
              // Translate from driver data to ROS data
            cntNum_++;
              //drrobot_jaguar4x4_player::MotorInfoArray motorInfoArray;
              //motorInfoArray.motorInfos.resize(MOTOR_NUM);
              for (uint32_t i = 0 ; i < MOTOR_NUM; ++i)
              {
                  motorInfoArray.motorInfos[i].header.stamp = ros::Time::now();
                  motorInfoArray.motorInfos[i].header.frame_id = string("drrobot_motor_");
                  motorInfoArray.motorInfos[i].header.frame_id += boost::lexical_cast<std::string>(i);
                  motorInfoArray.motorInfos[i].robot_type = robotConfig1_.boardType;
                  motorInfoArray.motorInfos[i].encoder_pos = motorSensorData_.motorSensorEncoderPos[i];
                  
                  //ROS_INFO("Left movement %u", motorInfoArray.motorInfos[i].encoder_pos);
                  motorInfoArray.motorInfos[i].encoder_vel = motorSensorData_.motorSensorEncoderVel[i];
                  motorInfoArray.motorInfos[i].encoder_dir = motorSensorData_.motorSensorEncoderDir[i];
                  if (robotConfig1_.boardType == Hawk_H20_Motion)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] * 3 /4096;;
                  }
                  else if(robotConfig1_.boardType != Jaguar)
                  {
                    motorInfoArray.motorInfos[i].motor_current = (float)motorSensorData_.motorSensorCurrent[i] / 728;
                  }
                  else
                  {
                    motorInfoArray.motorInfos[i].motor_current = 0.0;
                  }
                  motorInfoArray.motorInfos[i].motor_pwm = motorSensorData_.motorSensorPWM[i];
              }

              //ROS_INFO("publish motor info array");
              motorInfo_pub_.publish(motorInfoArray);
			  publishOdometry(motorInfoArray);
		     // ROS_INFO("Left movement %u", motorInfoArray.motorInfos[0].encoder_pos);

              drrobot_jaguar4x4_player::RangeArray rangerArray;
              rangerArray.ranges.resize(US_NUM);
	      if(enable_sonar_)
	      {
		      for (uint32_t i = 0 ; i < US_NUM; ++i)
		      {

		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_sonar_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = (float)rangeSensorData_.usRangeSensor[i]/100;     //to meters

		          // around 30 degrees
		          rangerArray.ranges[i].field_of_view = 0.5236085;
		          rangerArray.ranges[i].max_range = 2.55;
		          rangerArray.ranges[i].min_range = 0;
		          rangerArray.ranges[i].radiation_type = drrobot_jaguar4x4_player::Range::ULTRASOUND;
		      }

		      sonar_pub_.publish(rangerArray);
		}


	      if(enable_ir_)
	      {
		      rangerArray.ranges.resize(IR_NUM);
		      for (uint32_t i = 0 ; i < IR_NUM; ++i)
		      {
		          rangerArray.ranges[i].header.stamp = ros::Time::now();
		          rangerArray.ranges[i].header.frame_id = string("drrobot_ir_");
		          rangerArray.ranges[i].header.frame_id += boost::lexical_cast<std::string>(i);
		          rangerArray.ranges[i].range = ad2Dis(rangeSensorData_.irRangeSensor[i]);
		          rangerArray.ranges[i].radiation_type = drrobot_jaguar4x4_player::Range::INFRARED;
		      }

		      ir_pub_.publish(rangerArray);
	     }

              drrobot_jaguar4x4_player::StandardSensor standardSensor;
              standardSensor.humanSensorData.resize(4);
              standardSensor.tiltingSensorData.resize(2);
              standardSensor.overHeatSensorData.resize(2);
              standardSensor.header.stamp = ros::Time::now();
              standardSensor.header.frame_id = string("drrobot_standardsensor");
              for (uint32_t i = 0; i < 4; i++)
                standardSensor.humanSensorData[i] = standardSensorData_.humanSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.tiltingSensorData[i] = standardSensorData_.tiltingSensorData[i];
              for (uint32_t i = 0; i < 2; i++)
                standardSensor.overHeatSensorData[i] = standardSensorData_.overHeatSensorData[i];

              standardSensor.thermoSensorData = standardSensorData_.thermoSensorData;

              standardSensor.boardPowerVol = (double)standardSensorData_.boardPowerVol * 9 /4095;
              standardSensor.servoPowerVol = (double)standardSensorData_.servoPowerVol * 9 /4095;

              if (robotConfig1_.boardType != Jaguar)
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 24 /4095;
              }
              else
              {
                standardSensor.motorPowerVol = (double)standardSensorData_.motorPowerVol * 34.498 /4095;
              }
              standardSensor.refVol = (double)standardSensorData_.refVol / 4095 * 6;
              standardSensor.potVol = (double)standardSensorData_.potVol / 4095 * 6;
              standardSensor_pub_.publish(standardSensor);

              drrobot_jaguar4x4_player::CustomSensor customSensor;
              customSensor.customADData.resize(8);
              customSensor.header.stamp = ros::Time::now();
              customSensor.header.frame_id = string("drrobot_customsensor");

              for (uint32_t i = 0; i < 8; i ++)
              {
                customSensor.customADData[i] = customSensorData_.customADData[i];
              }
              customSensor.customIO = (uint8_t)(customSensorData_.customIO & 0xff);
              customSensor_pub_.publish(customSensor);
              
        }
                              // Publish the odometry in any case: .... This call is needed to publish the odometry message
        publishOdometry(motorInfoArray);
        //ROS_INFO("Left movement %u", motorInfoArray.motorInfos[0].encoder_pos);
    }

private:

    DrRobotMotionSensorDriver* drrobotMotionDriver_;
    DrRobotMotionSensorDriver* drrobotPowerDriver_;
    struct DrRobotMotionConfig robotConfig1_;
    struct DrRobotMotionConfig robotConfig2_;

    std::string odom_frame_id_;
    struct MotorSensorData motorSensorData_;
    struct RangeSensorData rangeSensorData_;
    struct PowerSensorData powerSensorData_;
    struct StandardSensorData standardSensorData_;
    struct CustomSensorData customSensorData_;


    std::string robotType_;
    std::string robotID_;
    std::string robotIP_;
    std::string robotCommMethod_;
    std::string robotSerialPort_;
    bool enable_ir_;
    bool enable_sonar_;
    int  commPortNum_;
    int  encoderOneCircleCnt_;
    double wheelDis_;
    double wheelRadius_;
    int motorDir_;
    double minSpeed_;
    double maxSpeed_;

    int cntNum_;
    double ad2Dis(int adValue)
    {
      double temp = 0;
      double irad2Dis = 0;

      if (adValue <= 0)
        temp = -1;
      else
        temp = 21.6 /((double)adValue * 3 /4096 - 0.17);

      if ( (temp > 80) || (temp < 0))
      {
        irad2Dis = 0.81;
      }
      else if( (temp < 10) && (temp > 0))
      {
        irad2Dis = 0.09;
      }
      else
        irad2Dis = temp /100;
      return irad2Dis;
    }
    
 
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drrobot_jaguar4x4_player");

    DrRobotPlayerNode drrobotPlayer;
    ros::NodeHandle n;
    // Start up the robot
    if (drrobotPlayer.start() != 0)
    {
        exit(-1);
    }
    /////////////////////////////////////////////////////////////////

    ros::Rate loop_rate(10);      //10Hz

    while (n.ok())
    {
      drrobotPlayer.doUpdate();
      ros::spinOnce();
     loop_rate.sleep();
    }
    /////////////////////////////////////////////////////////////////

    // Stop the robot
    drrobotPlayer.stop();

    return(0);
}

