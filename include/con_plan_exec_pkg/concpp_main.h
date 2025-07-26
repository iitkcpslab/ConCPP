/*
Purpose: Concurrent Coverage Planner
Last updated: Dynamic TH_req
Last updated on: 12 Dec 2022
To do: 
Author: Ratijit Mitra
*/


#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <algorithm>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <stack>


#include <ros/ros.h>
#include <ros/package.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <pthread.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>


#include "con_plan_exec_pkg/gap.h"


#include "con_plan_exec_pkg/PlanForHorizon.h"
#include "con_plan_exec_pkg/ShareLocalInformation.h"
#include "con_plan_exec_pkg/StopRobot.h"
#include "con_plan_exec_pkg/DiscreteClock.h"


#define RESULT_PER_CALL_FILE_NAME "resultPerCall.txt"
#define COMP_TIME_INFO_FILE_NAME "comp_time_info.txt"
#define RESULT_FILE "result.txt"


#ifdef DEBUG_RVIZ
	#include "con_plan_exec_pkg/rviz.h"
#endif


using namespace std;


class ConCPP_MAIN
{
	public:
		void initializeConCPP(ros::NodeHandle *nh);
		bool getLocalView(con_plan_exec_pkg::ShareLocalInformation::Request &req, con_plan_exec_pkg::ShareLocalInformation::Response &res);
		void handleRequest(con_plan_exec_pkg::ShareLocalInformation::Request req);
		void updateGlobalView(con_plan_exec_pkg::ShareLocalInformation::Request req);
		void printGlobalView();
		void checkCoveragePlanningCriteria();
		void stopCoveragePlanning();
		void startCoveragePlanning(float_mat ws_old, bool_mat assigned_goals_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, loc_vec req_rob_states);
		void callRobot(uint call_id_old, uint start_from_intrvl_old, uint path_len_old, loc_vec path_old, uint rob_id);
		void printCallInformation(uint req_count_old, uint TH_req_old, uint obs_count_old, uint unassigned_goal_count_old, uint cov_count_old, double comp_time, ros::Time comp_begun_at, ros::Time comp_ended_at, uint comp_begin_clk, uint comp_end_clk, double cfp_time, ros::Time cfp_begun_at, ros::Time cfp_ended_at, uint cfp_begin_clk, uint cfp_end_clk, uint fsbl_comp_intrvl, uint look_ahead_intrvl_bkp, uint cur_intrvl, uint act_rob_count, int max_pl, bool flag_send_paths);
		void writeClkFile(uint clk);
		void initializeClock();
		void writeClock();
		void incrementClock(const ros::TimerEvent &te);
		void stopRobots();

		ros::NodeHandle *nh;
		string op_dir_path;		// Output directory path

		int ws_size_x;		// Workspace size
		int ws_size_y;
		int rob_count;		// Number of robots
		int TH_req;			// Threshold on number of requesters

		float_mat ws;					// Globalview of the Workspace
		bool_mat assigned_goals;		// Goals which have already been assigned to robots
		uint obs_count;
		uint unassigned_goal_count;
		uint cov_count;

		int req_count;						// Number of requesters
		int req_count_old;
		bool_vec S_req;						// Set of requesters
		bool_vec S_req_old;
		robot_status_vec_t rob_states;		// States of the robots
		robot_status_vec_t rob_states_old;	// Last input robot states

		ros::Time mis_begun_at;			// Begining of the mission time

		uint clock;					// Clock (Discrete)
		ros::Timer clk_timer;		// Increment clock

		ros::ServiceServer lv_ss;		// Local view service server
		bool flag_running;				// Is the planner running?
		uint call_id;					// Call ID

		loc_mat paths;					// Paths
		uint start_from_intrvl;			// Start following the generated paths from the beginning of the interval
		uint_vec stops_at_clk;
		uint last_req_exp_from_rob;		// Last request expected from the robot
		uint last_req_exp_at_clk;		// Last request expected from the robot at clock
		
		double tot_comp_time;			// Total computation time

		#ifdef DEBUG_RVIZ
			cRVIZ *rviz_obj;
		#endif
};