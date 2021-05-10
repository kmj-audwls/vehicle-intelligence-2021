# Week 7 - Hybrid A* Algorithm & Trajectory Generation

---

[//]: # (Image References)
[has-example]: ./hybrid_a_star/has_example.png

## Assignment: Hybrid A* Algorithm
You have to implement the following sections of code for the assignment:

* Trajectory generation: in the method `HybridAStar.expand()`, a simple one-point trajectory shall be generated based on a basic bicycle model. This is going to be used in expanding 3-D grid cells in the algorithm's search operation.
---
위에서 지정한 omega_min, max범위에서 step의 크기만큼 각도를 탐색한다.
이러한 과정을 통해 다음 state의 위치및 bicycle의 head를 계산
---
~~~python
for delta_t in range(self.omega_min, self.omega_max+1, self.omega_step):
    # TODO: implement the trajectory generation based on
    # a simple bicycle model.
    # Let theta2 be the vehicle's heading (in radian)
    # between 0 and 2 * PI.
    # Check validity and then add to the next_states list.
    delta = np.pi / 180.0 * delta_t
    omega = self.speed / self.length * np.tan(delta)

    head_theta = theta + omega

    if head_theta < 0:
        head_theta += 2 * np.pi
    elif head_theta > 2 * np.pi:
        head_theta -= 2 * np.pi

    post_x = x + self.speed * np.cos(theta)
    post_y = y + self.speed * np.sin(theta)

    if 0 <= self.idx(post_x) and self.idx(post_x) < self.dim[1] and 0 <= self.idx(post_y) and self.idx(post_y) < self.dim[2]:
        post_f = g2 + self.heuristic(post_x, post_y, goal)
        post_state = {
            'f': post_f,
            'g': g2,
            'x': post_x,
            'y': post_y,
            't': head_theta,
        }
        next_states.append(post_state)
~~~
* Hybrid A* search algorithm: in the method `HybridAStar.search()`, after expanding the states reachable from the current configuration, the algorithm must process each state (i.e., determine the grid cell, check its validity, close the visited cell, and record the path. You will have to write code in the `for n in next_states:` loop.
~~~python
# TODO: implement prioritized breadth-first search
opened.sort(key=lambda s : s['f'], reverse=True)
curr = opened.pop()
x, y = curr['x'], curr['y']
if (self.idx(x), self.idx(y)) == goal:
    self.final = curr
    found = True
    break
next_states = self.expand(curr, goal)
for state in next_states:
    next_x = self.idx(state['x'])
    next_y = self.idx(state['y'])
    next_stack = self.theta_to_stack_num(state['t'])
    if (0 <= state['x'] < grid.shape[0]) and (0 <= state['y'] < grid.shape[1]) and grid[(next_x, next_y)] == 0 and \
            self.closed[next_stack][next_x][next_y] == 0:
        if self.check_cross_obstacle(curr, state, grid):
            opened.append(state)
            self.closed[next_stack][next_x][next_y] = 1
            self.came_from[next_stack][next_x][next_y] = curr

            total_closed += 1
~~~
* Discretization of heading: in the method `HybridAStar.theta_to_stack_num()`, you will write code to map the vehicle's orientation (theta) to a finite set of stack indices.
~~~python
# TODO: implement a function that calculate the stack number
# given theta represented in radian. Note that the calculation
# should partition 360 degrees (2 * PI rad) into different
# cells whose number is given by NUM_THETA_CELLS.
return int(theta // (2 * np.pi / self.NUM_THETA_CELLS))
~~~
* Heuristic function: in the method `HybridAStar.heuristic()`, you define a heuristic function that will be used in determining the priority of grid cells to be expanded. For instance, the distance to the goal is a reasonable estimate of each cell's cost.
~~~python
    def heuristic(self, x, y, goal):
        # TODO: implement a heuristic function.
        dist = np.sqrt((goal[0] - x) * (goal[0] - x) + (goal[1] - y) * (goal[1] - y))
        return dist
~~~
![Example Output of the Hybrid A* Test Program][has-example]

---
