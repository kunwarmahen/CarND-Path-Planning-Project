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


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Helper {
	public:
		Helper();
		~Helper();
		double safe_speed = 49.5;
		double forward_safe_distance = 30;
		double back_safe_distance = -10;
		double speed_increase_inc = .224;
		void generate_trajectory(Vehicle& my_vehicle, vector<Vehicle> other_vehicles, double prev_last_pos);
		bool is_close(Vehicle &my_vehicle, vector<Vehicle> other_vehicles, int next_lane_to_try, int horizon);
		int get_next_lane(int lane);
		bool set_vehicle_lane(Vehicle &my_vehicle, bool too_close_first_lane, int first_lane, bool both_lanes_has_col, bool last_lane_changed_lane);
};


