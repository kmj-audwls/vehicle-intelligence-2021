# Week 6 - Prediction & Behaviour Planning

## Assignment #2

Under the directory [./BP](./BP), you are given four Python modules:

* `simulate_behavior.py`: the main module you run. It instantiates a simple text-based simulation environment and runs it using the configuration specified in the same module.
* `road.py`: `class Road` is implemented here. It captures the state of the simulated road with a number of vehicles (including the ego) running on it, and visualizes it using terminal output.
* `vehicle.py`: `class Vehicle` implements the states of a vehicle and its transition, along with the vehicle's dynamics based on a simple kinematic assumption. Note that a vehicle's trajectory is represented by two instances of object of this class, where the first one gives the current state and the second one predicts the state that the vehicle is going to be in after one timestep.
* `cost_functions.py`: implementation of cost functions governing the state transition of the ego vehicle. The main job required for your assignment is to provide an adequate combination of cost functions by implementing them in this module.

### Task 1

Implement the method `choose_next_state()` in `vehicle.py`. It should

---
1. 'successor_states()'를 통하여 현재 상태에서 가능한 상태를 결정
2. 'choose_next_state()' 함수를 통하여 후보 trajectory를 정한뒤, 최소 cost를 구함
---

~~~python
        # TODO: implement state transition function based on the cost
        #       associated with each transition.

        # Note that the return value is a trajectory, where a trajectory
        # is a list of Vehicle objects with two elements.
        states = self.successor_states()
        costs = []
        for state in states:
            trajectory = self.generate_trajectory(state, predictions)
            if trajectory:
                cost = calculate_cost(self, trajectory, predictions)
                costs.append({"cost": cost, "state": state, "trajectory": trajectory})

        best = min(costs, key=lambda s: s['cost'])

        return best["trajectory"]
~~~
Note that you must return a planned trajectory (as described above) instead of the state that the vehicle is going to be in.

### Task 2

In `cost_functions.py`, templates for two different cost functions (`goal_distance_cost()` and `inefficiency_cost()`) are given. They are intended to capture the cost of the trajectory in terms of

* the lateral distance of the vehicle's lane selection from the goal position, and
* the time expected to be taken to reach the goal (because of different lane speeds),
---
'goal_distance_cost()'는 차량이 goal에 가까워지면 cost가 커지도록 설계
---
~~~python
def goal_distance_cost(vehicle, trajectory, predictions, data):
    '''
    Cost increases based on distance of intended lane (for planning a
    lane change) and final lane of a trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches
    the goal distance.
    '''
    distance = abs(data.end_distance_to_goal)

    if distance:
        cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data.intended_lane - data.final_lane) / distance))
    else:
        cost = 1
    return cost
~~~
---
'goal_distance_cost()'는 차량이 goal에 가까워지면 cost가 커지도록 설계
---
~~~python
def inefficiency_cost(vehicle, trajectory, predictions, data):
    '''
    Cost becomes higher for trajectories with intended lane and final lane
    that have slower traffic.
    '''
    speed_intended = velocity(predictions, data.intended_lane) or vehicle.target_speed
    speed_final = velocity(predictions, data.final_lane) or vehicle.target_speed

    cost = float(2.0 * vehicle.target_speed - speed_intended - speed_final) / vehicle.target_speed

    return cost
~~~

respectively.

Note that the range of cost functions should be carefully defined so that they can be combined by a weighted sum, which is done in the function `calculate_cost()` (to be used in `choose_next_state()` as described above). In computing the weighted sum, a set of weights are used. For example, `REACH_GOAL` and `EFFICIENCY` are already defined (but initialized to zero values). You are going to find out a good combination of weights by an empirical manner.

You are highly encouraged to experiment with your own additional cost functions. In implementing cost functions, a trajectory's summary (defined in `TrajectoryData` and given by `get_helper_data()`) can be useful.

You are also invited to experiment with a number of different simulation settings, especially in terms of

* number of lanes
* lane speed settings (all non-ego vehicles follow these)
* traffic density (governing the number of non-ego vehicles)

and so on.

Remember that our state machine should be geared towards reaching the goal in an *efficient* manner. Try to compare a behaviour that switches to the goal lane as soon as possible (note that the goal position is in the slowest lane in the given setting) and one that follows a faster lane and move to the goal lane as the remaining distance decreases. Observe different behaviour taken by the ego vehicle when different weights are given to different cost functions, and also when other cost metrics (additional cost functions) are used.
