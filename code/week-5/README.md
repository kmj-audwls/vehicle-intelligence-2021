# Week 5 - Path Planning & the A* Algorithm

---

## Examples

We have four small working examples for demonstration of basic path planning algorithms:

* `search.py`: simple shortest path search algorithm based on BFS (breadth first search) - only calculating the cost.
* `path.py`: built on top of the above, generating an optimum (in terms of action steps) path plan.
* `astar.py`: basic implementation of the A* algorithm that employs a heuristic function in expanding the search.
* `policy.py`: computation of the shortest path based on a dynamic programming technique.

These sample source can be run as they are. Explanation and test results are given in the lecture notes.

## Assignment

You will complete the implementation of a simple path planning algorithm based on the dynamic programming technique demonstrated in `policy.py`. A template code is given by `assignment.py`.

The assignmemt extends `policy.py` in two aspects:

* State space: since we now consider not only the position of the vehicle but also its orientation, the state is now represented in 3D instead of 2D.
* Cost function: we define different cost for different actions. They are:
	- Turn right and move forward
	- Move forward
	- Turn left and move forward
~~~python
	for y, x, o in p:
		# Mark the final state with a special value that we will
		# use in generating the final path policy.
		if (y, x) == goal and value[(o, y, x)] > 0:
			# TODO: implement code.
			change = True
			value[(o, y, x)] = 0
			policy[(o, y, x)] = 999
		# Try to use simple arithmetic to capture state transitions.
		elif grid[(y, x)] == 0:
			for i in range(len(action)):
				prev_o = (o + action[i]) % 4
				prev_x = x + forward[prev_o][1]
				prev_y = y + forward[prev_o][0]

				if (0 <= prev_x < grid.shape[1]) and (0 <= prev_y < grid.shape[0]) and grid[(prev_y, prev_x)] == 0:
					_cost = value[(prev_o, prev_y, prev_x)] + cost[i]

					if _cost < value[(o, y, x)]:
						change = True
						value[(o, y, x)] = _cost
						policy[(o, y, x)] = action[i]
# Now navigate through the policy table to generate a
# sequence of actions to take to follow the optimal path.
# TODO: implement code.
y, x, o = init
policy2D[(y, x)] = '#'
while policy[(o, y, x)] != 9999:
	if policy[(o, y, x)] == action[0]:
		_o = (o - 1) % 4
	elif policy[(o, y, x)] == action[1]:
		_o = o
	else:
		_o = (o + 1) % 4

	x += forward[_o][1]
	y += forward[_o][0]
	o = _o

	policy2D[(y, x)] = '*' if policy[(o, y, x)] == 999 else action_name[policy[(o, y, x)] + 1]
# Return the optimum policy generated above.
return policy2D
~~~
This example is intended to illustrate the algorithm's capability of generating an alternative (detouring) path when left turns are penalized by a higher cost than the other two possible actions. When run with the settings defined in `assignment.py` without modification, a successful implementation shall generate the following output,

```
[[' ', ' ', ' ', 'R', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', '#'],
 ['*', '#', '#', '#', '#', 'R'],
 [' ', ' ', ' ', '#', ' ', ' '],
 [' ', ' ', ' ', '#', ' ', ' ']]
```

because of the prohibitively high cost associated with a left turn.

You are highly encouraged to experiment with different (more complex) maps and different cost settings for each type of action.
