/*
 * ymca.h
 *
 *  Created on: 19.10.2015
 *      Author: tsagi
 */

#ifndef YMCA_H_
#define YMCA_H_

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace katanafun {

    typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

    class YmcaDance {
        public:
            YmcaDance();
            virtual ~YmcaDance();

            void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal);
            control_msgs::FollowJointTrajectoryGoal makeADanceTrajectory();
            actionlib::SimpleClientGoalState getState();

        private:
            ros::NodeHandle nh_;
            TrajClient traj_client_;
            ros::Subscriber joint_state_sub_;
            std::vector<std::string> joint_names_;
            bool got_joint_state_;
            std::vector<double> current_joint_state_;
            ros::AsyncSpinner spinner_;

            void jointStateCB(const sensor_msgs::JointState::ConstPtr &msg);
    };

} /* namespace katanafun */
#endif /* YMCA_H_ */
