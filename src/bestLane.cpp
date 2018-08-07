
#include "bestLane.hpp"
#include <iostream>

float inefficiency_cost(float intended_speed) {
    /*
    Cost becomes higher for trajectories with intended lane that have traffic slower than target_speed.
    */
    float target_speed = 50.;
    float cost = fabs(target_speed - intended_speed) / target_speed;
    
    return cost;
}

tuple<int, bool> bestChangeLaneOption(const vector<vector<double>> &sensor_fusion, int myLane, int indexToExclude, 
							int prev_size, double car_s, double ref_vel){
	// int start = 0;
	// // if the car in front of us in is the same as `start` index, then change `start` and consider the next car
	// if (indexToExclude == start){
	// 	start++; 
	// }

	unordered_map<int, int> cars_in_lane; // we save only cars in front in other lanes
	unordered_map<int, bool> allowed_transition_to_lane;
	for (int i = 0; i < 3; ++i){
		if(i == myLane){
			allowed_transition_to_lane[i] = false;
		}
		else{
			allowed_transition_to_lane[i] = true;	
		}
		
	}
	// allowed_transition_to_lane = {{0, true}, {1, true}, {2, true}};

	// allowed_transition_to_lane[indexToExclude] = false;

	for (int i = 0; i < sensor_fusion.size(); ++i){
		if(i == indexToExclude){
			continue;
		}

		float d = sensor_fusion[i][6];
		//if the car is in the same lane, skip it
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
		// if(abs(lane - myLane) > 1){
		// 	allowed_transition_to_lane[lane] = false;
		// 	continue;
		// }

		double vx = sensor_fusion[i][3];
  		double vy = sensor_fusion[i][4];
  		double check_speed = sqrt(vx * vx + vy * vy);
		double check_car_s = sensor_fusion[i][5];

		check_car_s += ((double) prev_size * .02 * check_speed);
		// check that there is enough space for lane change, if not mark it as an option to not consider
		// We need at least 50 meters to keep it safe
		
		if(check_car_s > car_s && (check_car_s - car_s) < 50){
			cout << "lane: " << lane << endl;
			std::cout << "check_car_s - car_s= " << check_car_s - car_s  << std::endl;
			allowed_transition_to_lane[lane] = false;
			continue;
		}
		else if(check_car_s < car_s && (car_s - check_car_s) < 30){
			cout << "lane: " << lane << endl;
			std::cout << "car_s - check_car_s= " << car_s - check_car_s << std::endl;
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
	// for (auto it = cars_in_lane.begin(); it != cars_in_lane.end(); ++it){
	// 	int lane = it->first;
	// 	// if the transition is not allowed skip it
	// 	if(!allowed_transition_to_lane[lane]){
	// 		continue;
	// 	}

	// 	// get the cost for each transition
	// 	// vector<int> cars = it->second; // car in lane `lane`
	// 	// int index = cars[0];
	// 	int carIndex = it->second;
		// double vx = sensor_fusion[carIndex][3];
  // 		double vy = sensor_fusion[carIndex][4];
  // 		double car_speed = sqrt(vx * vx + vy * vy);
		// float cost = inefficiency_cost(49.5, car_speed);
		// if (cost < minCost){
		// 	minCost = cost;
		// 	targetLane = lane;
		// 	if (car_speed < ref_vel){
		// 		reduceSpeed = true;
		// 	}
		// 	continue;
		// }
	// }
	for(auto it = allowed_transition_to_lane.begin(); it != allowed_transition_to_lane.end(); ++it){
		int lane = it->first;
		bool allowed = it->second;
		if(!allowed){
			std::cout << "not allowed, lane: "<< it->first << std::endl;
			continue;
		}

		int carIndex = -1;
		float cost;
		double car_speed = ref_vel;
		try{
			carIndex = cars_in_lane.at(lane);
			double vx = sensor_fusion[carIndex][3];
	  		double vy = sensor_fusion[carIndex][4];
	  		car_speed = sqrt(vx * vx + vy * vy);
			cost = inefficiency_cost(car_speed);
		}
		catch (const std::out_of_range& oor) {
			cout << "no cars found in this lane" << endl;
			cost = inefficiency_cost(49.5);
		}

		// find the min cost and also check that the lane is the nearest to ours
		if (cost < minCost && (abs(lane - myLane) <= 1)){
			minCost = cost;
			targetLane = lane;

			if (carIndex != -1 && car_speed < ref_vel){
				reduceSpeed = true;
			}
			else{
				reduceSpeed = false;
			}
		}
		
	}

	return  std::make_tuple(targetLane, reduceSpeed);
}

tuple<int, bool> changeToLane(const vector<vector<double>> &sensor_fusion, int currentLane, int prev_size, double car_s){
	bool carLeft = false;
	bool carRight = false;
	bool carAhead = false;
	double safeDist = 40; // in meters
	for(int i = 0; i < sensor_fusion.size(); ++i){
		// d coordinate (in Frenet coordinate system)
		float d = sensor_fusion[i][6];
		// get the lane number of the car
		int lane = -1;		
		if(d > 0 && lane < 4){
			lane = 0;
		}
		if(d >= 4 && d < 8){
			lane = 1;
		}
		else if(d >= 8){
			lane = 2;
		}

		if(lane == -1){
			continue;
		}

		// get the speed
	    double vx = sensor_fusion[i][3];
		double vy = sensor_fusion[i][4];
	    double check_speed = sqrt(vx * vx + vy * vy);
	    double check_car_s = sensor_fusion[i][5];
	    //let's look  where is the car in the future
		check_car_s += ((double) prev_size * .02 * check_speed);// if using previous points can project s value out.
		// cout << "car_s="<<car_s << " check_car_s=" << check_car_s << endl;
		if(lane == currentLane){
			carAhead |= check_car_s > car_s && (check_car_s - car_s) < safeDist;	
		}
		else if(lane - currentLane == -1){
			carLeft |= (car_s - safeDist) < check_car_s && (car_s + safeDist) > check_car_s;
		}
		else if(lane - currentLane == 1){
			carRight |= (car_s - safeDist) < check_car_s && (car_s + safeDist) > check_car_s;
		}		
	}

	// behaviour
	bool tooClose = false;
	int bestLane = currentLane;

	if(carAhead){
		cout << "car ahead" << endl;

		if(!carLeft && currentLane > 0){
			bestLane = currentLane - 1;
			cout << "no car left" << endl;
		}
		else if(!carRight && currentLane != 2){
			bestLane = currentLane + 1;
			cout << "no car right" << endl;
		}
		else{
			tooClose = true;
			cout << "too close" << endl;
		}
	}
	// else{
	// 	cout << "no car ahead" << endl;
	// 	if((currentLane == 0 && !carRight) || (currentLane == 2 && !carLeft)){
	// 		cout << "got to center lane" << endl;
	// 		bestLane = 1;
	// 	}
	// }

	cout << "-----" << endl;
	return std::make_tuple(bestLane, tooClose);
}