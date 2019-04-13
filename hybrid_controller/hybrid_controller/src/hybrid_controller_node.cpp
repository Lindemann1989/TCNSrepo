#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include "hybrid_controller/CriticalEvent.h"
#include "hybrid_controller/Params.h"
#include "hybrid_controller/BarrierFunction.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include <boost/bind.hpp>
#include <string>
#include <vector>
#include <cmath>
#include <armadillo>
#include "arma_ros_std_conversions.h"
#include "BC.hpp"
#include "PFC.hpp"
using namespace std;


class ControllerNode{
	ros::Publisher control_input_pub, upfc_pub, ubc_pub, bf_pub;
	std::vector<ros::Subscriber> pose_subs;

	std::vector<geometry_msgs::PoseStamped> poses;
	arma::vec X;

	std::vector<std::vector<int>> clusters;
	std::vector<int> robots_in_cluster;

	BC barrier_controller;
	PFC potential_field_controller;
	int robot_id;
	arma::vec u_max;

	std::vector<bool> state_was_read;
	
	

public:
	ControllerNode(ros::NodeHandle nh, ros::NodeHandle priv_nh, int n_robots, int robot_id, 
			std::vector<int> V, std::vector<int> robots_in_cluster, arma::vec u_max, BC bc, PFC pfc): 
				robot_id(robot_id), u_max(u_max), barrier_controller(bc), potential_field_controller(pfc){
				
		state_was_read = std::vector<bool>(n_robots , false);
		X = arma::vec(3*(n_robots));

		control_input_pub = nh.advertise<geometry_msgs::Twist>("/cmdvel", 100);
		upfc_pub = nh.advertise<geometry_msgs::Twist>("/upfc"+std::to_string(robot_id), 100);
		ubc_pub = nh.advertise<geometry_msgs::Twist>("/ubc"+std::to_string(robot_id), 100);
		bf_pub = nh.advertise<hybrid_controller::BarrierFunction>("/barrierfunction"+std::to_string(robot_id), 100);

		poses = std::vector<geometry_msgs::PoseStamped>(n_robots);

		for(int i=0; i<n_robots; i++){
			pose_subs.push_back(
				nh.subscribe<geometry_msgs::PoseStamped>(
					"/pose_robot" + std::to_string(i),
					100,
					boost::bind(&ControllerNode::poseCallback, this, _1, i)));
		}
	}



	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, int i){
		poses[i] = *msg;
		X(arma::span(3*i, 3*i+2)) = pose_to_vec(poses[i]);
		if(!state_was_read[i]){
			state_was_read[i] = true;
			if(std::all_of(state_was_read.cbegin(), state_was_read.cend(), [](bool v){return v;})){
				double t = ros::Time::now().toSec();
				arma::vec x = X(arma::span(3*robot_id, 3*robot_id+2));
				barrier_controller.init(t, x, X);
			}
		}
	}

	void update(){
	
		if(std::any_of(state_was_read.cbegin(), state_was_read.cend(), [](bool v){return !v;})) return;
		
		std::vector<double> X_std = arma::conv_to<std::vector<double>>::from(X);
		
		arma::vec u_bc = barrier_controller.u(X_std, pose_to_std_vec(poses[robot_id]), ros::Time::now().toSec());
		arma::vec u_pfc = potential_field_controller.u(X);

		
		arma::vec u;
		u = u_bc + u_pfc; 
	
		geometry_msgs::Twist u_msg;
		double c = cos(X(robot_id*3+2));
		double s = sin(X(robot_id*3+2));
		u_msg.linear.x = (u(0)*c + u(1)*s)*450.0;
		u_msg.linear.y = (-u(0)*s + u(1)*c)*450.0;
		u_msg.angular.z = u(2);
		control_input_pub.publish(u_msg);
		
		
		geometry_msgs::Twist u1;
		u1.linear.x = u_bc(0);
		u1.linear.y = u_bc(1);
		ubc_pub.publish(u1);
		geometry_msgs::Twist u2;
		u2.linear.x = u_pfc(0);
		u2.linear.y = u_pfc(1);
		upfc_pub.publish(u2);

		hybrid_controller::BarrierFunction bf_msg;
		ros::Time stamp = ros::Time::now();
		bf_msg.stamp = stamp;
		//rho_msg.t_relative_t0 = stamp.toSec()-prescribed_performance_controller.get_t_0();
		//rho_msg.t_relative_tr = stamp.toSec()-prescribed_performance_controller.get_t_0()-prescribed_performance_controller.get_t_r();
		bf_msg.bf = barrier_controller.bf(X_std);
		bf_msg.t = barrier_controller.t(ros::Time::now().toSec());
		bf_pub.publish(bf_msg);
	}


};


void readParameters(ros::NodeHandle nh, ros::NodeHandle priv_nh, int& n_robots, int& robot_id, int& freq,
		std::vector<std::string>& s0barrier, std::vector<std::string>& s0dbarrier_t, std::vector<std::vector<std::string>>& s0dbarrier_x,
		std::vector<std::string>& s1barrier, std::vector<std::string>& s1dbarrier_t, std::vector<std::vector<std::string>>& s1dbarrier_x,
		std::vector<std::string>& s2barrier, std::vector<std::string>& s2dbarrier_t, std::vector<std::vector<std::string>>& s2dbarrier_x,
		std::vector<std::string>& s3barrier, std::vector<std::string>& s3dbarrier_t, std::vector<std::vector<std::string>>& s3dbarrier_x,
		std::vector<std::string>& s4barrier, std::vector<std::string>& s4dbarrier_t, std::vector<std::vector<std::string>>& s4dbarrier_x,
		std::vector<std::string>& s5barrier, std::vector<std::string>& s5dbarrier_t, std::vector<std::vector<std::string>>& s5dbarrier_x,
		std::vector<int>& cluster, std::vector<int>& V, std::vector<int>& robots_in_cluster, std::vector<int>& sequence,
		arma::vec& u_max, std::vector<double>& W, std::vector<double>& L, double& r, double& R, double& w){
	
	nh.param<int>("freq", freq, 100);
	nh.param<int>("n_robots", n_robots, 10);
	priv_nh.getParam("robot_id", robot_id);

	s0barrier = std::vector<std::string>(n_robots);
	s0dbarrier_t = std::vector<std::string>(n_robots);
	s1barrier = std::vector<std::string>(n_robots);
	s1dbarrier_t = std::vector<std::string>(n_robots);
	s2barrier = std::vector<std::string>(n_robots);
	s2dbarrier_t = std::vector<std::string>(n_robots);
	s3barrier = std::vector<std::string>(n_robots);
	s3dbarrier_t = std::vector<std::string>(n_robots);
	s4barrier = std::vector<std::string>(n_robots);
	s4dbarrier_t = std::vector<std::string>(n_robots);
	s5barrier = std::vector<std::string>(n_robots);
	s5dbarrier_t = std::vector<std::string>(n_robots);
	cluster = std::vector<int>(n_robots);
	
	W = std::vector<double>(n_robots);
	L = std::vector<double>(n_robots);
	
	
	for(int i=0; i<n_robots; i++){
		std::string i_str = std::to_string(i);
		nh.getParam("s0barrier"+i_str, s0barrier[i]);
		nh.getParam("s0dbarrier_t"+i_str, s0dbarrier_t[i]);
		nh.getParam("s1barrier"+i_str, s1barrier[i]);
		nh.getParam("s1dbarrier_t"+i_str, s1dbarrier_t[i]);
		nh.getParam("s2barrier"+i_str, s2barrier[i]);
		nh.getParam("s2dbarrier_t"+i_str, s2dbarrier_t[i]);
		nh.getParam("s3barrier"+i_str, s3barrier[i]);
		nh.getParam("s3dbarrier_t"+i_str, s3dbarrier_t[i]);
		nh.getParam("s4barrier"+i_str, s4barrier[i]);
		nh.getParam("s4dbarrier_t"+i_str, s4dbarrier_t[i]);
		nh.getParam("s5barrier"+i_str, s5barrier[i]);
		nh.getParam("s5dbarrier_t"+i_str, s5dbarrier_t[i]);
		nh.getParam("cluster"+i_str, cluster[i]);
		nh.getParam("L"+i_str, L[i]);
		nh.getParam("W"+i_str, W[i]);
			
		std::vector<std::string> df;
		nh.getParam("s0dbarrier_x"+i_str, df);
		s0dbarrier_x.push_back(df);
		nh.getParam("s1dbarrier_x"+i_str, df);
		s1dbarrier_x.push_back(df);
		nh.getParam("s2dbarrier_x"+i_str, df);
		s2dbarrier_x.push_back(df);
		nh.getParam("s3dbarrier_x"+i_str, df);
		s3dbarrier_x.push_back(df);
		nh.getParam("s4dbarrier_x"+i_str, df);
		s4dbarrier_x.push_back(df);
		nh.getParam("s5dbarrier_x"+i_str, df);
		s5dbarrier_x.push_back(df);

	}

	std::vector<double> u_max_stdvec;
	nh.getParam("u_max", u_max_stdvec);
	u_max = arma::vec(u_max_stdvec);


	for(int i=0; i<cluster.size(); i++){
		if(cluster[robot_id] == cluster[i]){
			robots_in_cluster.push_back(i);
		}
	}

	nh.getParam("V"+std::to_string(robot_id), V);
	nh.getParam("sequence"+std::to_string(robot_id), sequence);
	
	nh.getParam("R", R);
	nh.getParam("r", r);
	nh.getParam("w", w);
	
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "hybrid_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle priv_nh("~");

	int n_robots, robot_id, freq;
	std::vector<std::string> s0barrier, s0dbarrier_t;
	std::vector<std::vector<std::string>> s0dbarrier_x;
	std::vector<std::string> s1barrier, s1dbarrier_t;
	std::vector<std::vector<std::string>> s1dbarrier_x;
	std::vector<std::string> s2barrier, s2dbarrier_t;
	std::vector<std::vector<std::string>> s2dbarrier_x;
	std::vector<std::string> s3barrier, s3dbarrier_t;
	std::vector<std::vector<std::string>> s3dbarrier_x;
	std::vector<std::string> s4barrier, s4dbarrier_t;
	std::vector<std::vector<std::string>> s4dbarrier_x;
	std::vector<std::string> s5barrier, s5dbarrier_t;
	std::vector<std::vector<std::string>> s5dbarrier_x;
	std::vector<int> cluster, V, robots_in_cluster, sequence;
	arma::vec u_max;
	std::vector<double> W, L;
	double r, R, w;
	
	readParameters(nh, priv_nh, n_robots, robot_id, freq, s0barrier, s0dbarrier_t, s0dbarrier_x, s1barrier, s1dbarrier_t, s1dbarrier_x, s2barrier, s2dbarrier_t, s2dbarrier_x, s3barrier, s3dbarrier_t, s3dbarrier_x, s4barrier, s4dbarrier_t, s4dbarrier_x, s5barrier, s5dbarrier_t, s5dbarrier_x, cluster, V, robots_in_cluster, sequence, u_max, W, L, r, R, w);
	
	BC bc(robot_id,W[robot_id], L[robot_id], s0barrier, s0dbarrier_t, s0dbarrier_x, s1barrier, s1dbarrier_t, s1dbarrier_x, s2barrier, s2dbarrier_t, s2dbarrier_x, s3barrier, s3dbarrier_t, s3dbarrier_x, s4barrier, s4dbarrier_t, s4dbarrier_x, s5barrier, s5dbarrier_t, s5dbarrier_x, u_max, V, sequence);
	
	PFC pfc(robot_id, r, R, u_max, w);

	ControllerNode controller_node(nh, priv_nh, n_robots, robot_id, V, robots_in_cluster, u_max, bc, pfc);
	
	ros::Rate rate(freq);
	while(ros::ok()){
		ros::spinOnce();
		controller_node.update();
		rate.sleep();
	}
}
