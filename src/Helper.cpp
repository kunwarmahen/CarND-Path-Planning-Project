#include "Helper.h"

Helper::Helper() {
}

Helper::~Helper() {
}

void Helper::generate_trajectory(Vehicle& my_vehicle, vector<Vehicle>  other_vehicles, double horizon) {
	
	bool too_close = false;

	/** Reset the suggested car lane in case it is the lane which was previously suggested. 
	This was added to avoid swirling of the vehicle.*/
	my_vehicle.reset_suggested_lane();

	//Check if the vehicle is too close to any in the current lane.
	for (int i = 0; i < other_vehicles.size(); i++) {
		if (my_vehicle.get_car_lane() == other_vehicles.at(i).get_car_lane()) {
			double other_cars_s = other_vehicles.at(i).state_at(horizon).at(0);
			if ((other_cars_s > my_vehicle.car_s) && ((other_cars_s - my_vehicle.car_s) < forward_safe_distance)) {
				too_close = true;
				break;
			}
		}
	}

	if (too_close) { //If it is closer then 30 meters then we need to take elusive maneuver.
		int next_lane_to_try = get_next_lane(my_vehicle.get_car_lane());

		//check for first lane on the right if there is a possible route without collision
		int first_lane = next_lane_to_try; 
		bool too_close_first_lane = is_close(my_vehicle, other_vehicles, next_lane_to_try, horizon);
		
	
		next_lane_to_try = get_next_lane(next_lane_to_try);

		//check for second lane on the right if there is a possible route without collision
		int second_lane = next_lane_to_try;
		bool too_close_second_lane = is_close(my_vehicle, other_vehicles, next_lane_to_try, horizon);

		bool has_changed_lane = false;
		bool both_lanes_has_col = false;
		
		//If there are both the lane available then set parameter both_lanes_has_col to true
		if (!too_close_first_lane && !too_close_second_lane) {
			both_lanes_has_col = true;
		}

		// Verify and change lanes.
		has_changed_lane = set_vehicle_lane(my_vehicle, too_close_first_lane, first_lane, both_lanes_has_col, has_changed_lane);
		//If first test did not succeed then try with second
		has_changed_lane = set_vehicle_lane(my_vehicle, too_close_second_lane, second_lane, both_lanes_has_col, has_changed_lane);

		//If it was not possible to change lane, then maintain the lane and slow down
		if(!has_changed_lane) {
			my_vehicle.car_vel -= speed_increase_inc;
		}
	} else if (my_vehicle.car_vel < safe_speed) {  // Increase spead if not at the safe speed
		my_vehicle.car_vel += speed_increase_inc;
	}
}

int Helper::get_next_lane(int lane) {
	lane +=  1; //pick next lane to currnet lane
	if (lane > 2) { //reset in case it greater than lane 2
		lane = 0;
	}
	return lane;
}


bool Helper::is_close(Vehicle &my_vehicle, vector<Vehicle> other_vehicles, int next_lane_to_try, int horizon) {

	bool too_close = false;
	for (int i = 0; i < other_vehicles.size(); i++) { 
		if (next_lane_to_try == other_vehicles.at(i).get_car_lane()) {
			// check if there is a clear route in next lane with clear route for atleast 30 m at the end of  previous predicated lane and there is nothing behind it atleast 10 m
			double other_cars_s = other_vehicles.at(i).state_at(horizon).at(0);
			if (((other_cars_s > my_vehicle.car_s) && ((other_cars_s - my_vehicle.car_s) < forward_safe_distance))
				|| ((other_cars_s < my_vehicle.car_s) && ((other_cars_s - my_vehicle.car_s) >  back_safe_distance))) {
				too_close = true;
				break;
			}

			// check if there is a clear route in next next lane with clear route for atleast 30 m and there is nothing behind it atleast 10 m
			double my_car_s = my_vehicle.current_car_s;
			if (((other_vehicles.at(i).car_s > my_car_s) && ((other_vehicles.at(i).car_s - my_car_s) < forward_safe_distance))
				|| ((other_cars_s < my_vehicle.car_s) && ((other_cars_s - my_vehicle.car_s) >  back_safe_distance))) {
				too_close = true;
				break;
			}

		}
	}
	return too_close;
}


bool Helper::set_vehicle_lane(Vehicle &my_vehicle, bool too_close_first_lane, int next_lane, bool both_lanes_has_col, bool last_lane_changed_lane) {
	
	bool has_changed_lane = false;
	if (!too_close_first_lane && !last_lane_changed_lane) { //I am not too close any vehicle in next right lane
		if (my_vehicle.get_car_lane() == 0 && next_lane == 2) { //Ignore jumping across the lanes as we don't know (ateast in the logic) there is car in the middle lane
			//Ignore
		}
		else if (my_vehicle.get_car_lane() == 2 && next_lane == 0) {//Same here
			//Ignore
		}
		else {
			if (both_lanes_has_col && my_vehicle.suggested_lane != 100) { //If we are already in the process of changing lanes then continue with that rather then try to change to another lane in this instance
				my_vehicle.set_car_lane(my_vehicle.suggested_lane);
				has_changed_lane = true;
			}
			else {
				my_vehicle.set_car_lane(next_lane);
				my_vehicle.suggested_lane = next_lane;
				has_changed_lane = true;
			}
		}
	}

	return has_changed_lane;
}