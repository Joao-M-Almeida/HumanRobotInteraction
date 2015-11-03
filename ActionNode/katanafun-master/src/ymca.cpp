/*
 * ymca.cpp
 *
 *  Created on: 19.10.2015
 *      Author: tsagi
 */

#include <katanafun/ymca.h>

namespace katanafun {

    YmcaDance::YmcaDance() :
        traj_client_("/katana_arm_controller/follow_joint_trajectory", true), got_joint_state_(false), spinner_(1) {
            joint_names_.push_back("katana_motor1_pan_joint");
            joint_names_.push_back("katana_motor2_lift_joint");
            joint_names_.push_back("katana_motor3_lift_joint");
            joint_names_.push_back("katana_motor4_lift_joint");
            joint_names_.push_back("katana_motor5_wrist_roll_joint");

            joint_state_sub_ = nh_.subscribe("/joint_states", 1, &YmcaDance::jointStateCB, this);
            spinner_.start();

            // wait for action server to come up
            while (!traj_client_.waitForServer(ros::Duration(5.0))) {
                ROS_INFO("Waiting for the follow_joint_trajectory server");
            }
        }

    YmcaDance::~YmcaDance() {
    }

    void YmcaDance::jointStateCB(const sensor_msgs::JointState::ConstPtr &msg) {
        std::vector<double> ordered_js;

        ordered_js.resize(joint_names_.size());

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            bool found = false;
            for (size_t j = 0; j < msg->name.size(); ++j) {
                if (joint_names_[i] == msg->name[j]) {
                    ordered_js[i] = msg->position[j];
                    found = true;
                    break;
                }
            }
            if (!found)
                return;
        }

        ROS_INFO_ONCE("Got joint state!");
        current_joint_state_ = ordered_js;
        got_joint_state_ = true;
    }

    //! Sends the command to start a given trajectory
    void YmcaDance::startTrajectory(control_msgs::FollowJointTrajectoryGoal goal) {
        // When to start the trajectory: 1s from now
        goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
        traj_client_.sendGoal(goal);
    }

    control_msgs::FollowJointTrajectoryGoal YmcaDance::makeADanceTrajectory() {
        const size_t NUM_TRAJ_POINTS = 7;
        const size_t NUM_JOINTS = 5;

        // Calibration
        std::vector<double> calibration_positions(NUM_JOINTS);
        calibration_positions[0] = 0.0;
        calibration_positions[1] = 1.57;
        calibration_positions[2] = 0.0;
        calibration_positions[3] = 0.0;
        calibration_positions[4] = 0.0;

        // Y
        std::vector<double> y_positions(NUM_JOINTS);
        y_positions[0] = 0.0;
        y_positions[1] = 1.57;
        y_positions[2] = 0.0;
        y_positions[3] = -2.0;
        y_positions[4] = 3.5;

        // M
        std::vector<double> m_positions(NUM_JOINTS);
        m_positions[0] = 0.0;
        m_positions[1] = 1.0;
        m_positions[2] = -2.0;
        m_positions[3] = -2.0;
        m_positions[4] = 3.5;

        // C
        std::vector<double> c_positions(NUM_JOINTS);
        c_positions[0] = -2.96;
        c_positions[1] = 0.0;
        c_positions[2] = 1.57;
        c_positions[3] = 0.0;
        c_positions[4] = 0.0;

        // A
        std::vector<double> a_positions(NUM_JOINTS);
        a_positions[0] = 0.0;
        a_positions[1] = 1.2;
        a_positions[2] = 0.0;
        a_positions[3] = 2.0;
        a_positions[4] = 3.5;

        trajectory_msgs::JointTrajectory trajectory;

        for (ros::Rate r = ros::Rate(10); !got_joint_state_; r.sleep()) {
            ROS_DEBUG("waiting for joint state...");

            if (!ros::ok())
                exit(-1);
        }

        // First, the joint names, which apply to all waypoints
        trajectory.joint_names = joint_names_;

        trajectory.points.resize(NUM_TRAJ_POINTS);

        // trajectory point:
        int ind = 0;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions = current_joint_state_;

        // trajectory point:
        ind++;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions = calibration_positions;

        // trajectory point:
        ind++;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions = y_positions;

        // trajectory point:
        ind++;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions = m_positions;

        //trajectory point
        ind++;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions = c_positions;

        //trajectory point
        ind++;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions = a_positions;

        // trajectory point:
        ind++;
        trajectory.points[ind].time_from_start = ros::Duration(3 * ind);
        trajectory.points[ind].positions.resize(NUM_JOINTS);
        trajectory.points[ind].positions = calibration_positions;

        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory = trajectory;
        return goal;
    }

    //! Returns the current state of the action
    actionlib::SimpleClientGoalState YmcaDance::getState() {
        return traj_client_.getState();
    }

} /* namespace katanafun */

int main(int argc, char** argv) {
    // Init the ROS node
    ros::init(argc, argv, "ymca");

    katanafun::YmcaDance arm;
    // Start the dance trajectory
    arm.startTrajectory(arm.makeADanceTrajectory());
    // Wait for trajectory completion
    while (!arm.getState().isDone() && ros::ok()) {
        usleep(50000);
    }
}
