#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include <random>
#include "vehicle.h"
#include "trajectory.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Helper {
	public:
		Helper();
		~Helper();
		double horizon = 30;
		void generate_trajectory(Vehicle& my_vehicle, vector<Vehicle> other_vehicles, double prev_last_pos);
		void generate_trajectory_1(Vehicle &my_vehicle, vector<Vehicle> other_vehicles, double prev_last_pos);
		double calculate_cost(Vehicle &my_veh, Vehicle &orig_veh,  vector<Vehicle> other_vehicles, double prev_last_pos);
		//double collision_cost(vector<vector<double>> traj, vector<Vehicle> const &vehicles);
		double exceeds_jerk_cost(Vehicle &my_veh, vector<Vehicle> const &vehicles, double prev_last_pos);
		double exceeds_accel_cost(Vehicle &my_veh, vector<Vehicle> const &vehicles, double prev_last_pos);
		double exceeds_speed_limit_cost(Vehicle &my_veh, vector<Vehicle> const &vehicles, double prev_last_pos);
		double nearest_approach(Vehicle &my_veh, Vehicle &vehicle, double time);
		double logistic(double x);
		double nearest_approach_to_any_vehicle(Vehicle &my_veh, vector<Vehicle> vehicles, double time);
		double collision_cost(Vehicle &my_veh,  vector<Vehicle> other_vehicle, double prev_last_pos);
		double lane_change_cost(Vehicle &my_veh, Vehicle &orig_veh);

};


