
#include "bestLane.hpp"


float inefficiency_cost(float target_speed, float intended_speed) {
    /*
    Cost becomes higher for trajectories with intended lane that have traffic slower than target_speed.
    */
    float cost = fabs(target_speed - intended_speed) / target_speed;
    
    return cost;
}

tuple<int, bool> bestChangeLaneOption(const vector<vector<double>> &sensor_fusion, int myLane, int indexToExclude, 
							int prev_size, double car_s, double ref_vel){
	int start = 0;
	// if the car in front of us in is the same as `start` index, then change `start` and consider the next car
	if (indexToExclude == start){
		start++; 
	}

	unordered_map<int, int> cars_in_lane; // we save only cars in front in other lanes
	unordered_map<int, bool> allowed_transition_to_lane;
	allowed_transition_to_lane = {{0, true}, {1, true}, {2, true}};
	allowed_transition_to_lane[indexToExclude] = false;

	for (int i = start; i < sensor_fusion.size(); ++i){
		if(i == indexToExclude){
			continue;
		}

		float d = sensor_fusion[i][6];

		// if the car is in the same lane, skip it
		if(d < (2 + 4 * myLane + 2) && d > (2 + 4 * myLane - 2)){
			continue;
		}

		// get the lane number of the car
		int lane = 0;		
		if(d > 4 && d < 6){
			lane = 1;
		}
		else if(d >= 6){
			lane = 2;
		}

		// if we are in the rightmost lane, i.e 2, we should not consider the lane 0.
		// if we are in the leftmost lane, i.e 0, we should not consider the lane 2.
		if(abs(lane - myLane) > 1){
			continue;
		}

		double vx = sensor_fusion[i][3];
  		double vy = sensor_fusion[i][4];
  		double check_speed = sqrt(vx * vx + vy * vy);
		double check_car_s = sensor_fusion[i][5];

		check_car_s += ((double) prev_size * .02 * check_speed);
		// check that there is enough space for lane change, if not mark it as an option to not consider
		// We need at least 50 meters tp keep it safe
		if(fabs(check_car_s - car_s) < 50){
			allowed_transition_to_lane[lane] = false;
			continue;
		}
		else if(check_car_s > car_s){
			cars_in_lane[lane] = i;
		}
	}

	// iterate over the allowed lane transition options
	int minCost = INT_MAX;
	int targetLane = myLane;
	bool reduceSpeed = false;
	for (auto it = cars_in_lane.begin(); it != cars_in_lane.end(); ++it){
		int lane = it->first;
		// if the transition is not allowed skip it
		if(!allowed_transition_to_lane[lane]){
			continue;
		}

		// get the cost for each transition
		// vector<int> cars = it->second; // car in lane `lane`
		// int index = cars[0];
		int carIndex = it->second;
		double vx = sensor_fusion[carIndex][3];
  		double vy = sensor_fusion[carIndex][4];
  		double car_speed = sqrt(vx * vx + vy * vy);
		float cost = inefficiency_cost(49.5, car_speed);
		if (cost < minCost){
			minCost = cost;
			targetLane = lane;
			if (car_speed < ref_vel){
				reduceSpeed = true;
			}
			continue;
		}
	}

	return  std::make_tuple(targetLane, reduceSpeed);
}