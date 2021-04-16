# Week 4 - Motion Model & Particle Filters

---

[//]: # (Image References)
[empty-update]: ./myresult.gif
[example]: ./myresult2.gif

## Assignment

You will complete the implementation of a simple particle filter by writing the following two methods of `class ParticleFilter` defined in `particle_filter.py`:

# This code was produced by referring https://github.com/GitHubChuanYu/T2Project3_ParticleFilterKidnappedVehicle.git

* `update_weights()`: For each particle in the sample set, calculate the probability of the set of observations based on a multi-variate Gaussian distribution.
~~~python
# 1. Select the set of landmarks that are visible
#    (within the sensor range).
for particle in self.particles:
    landmarks_in_range = []
    for id, vertice in map_landmarks.items():
        dist = distance(particle, vertice)
        if dist < sensor_range:
            landmarks_in_range.append({'id' : id, 'x' : vertice['x'], 'y' : vertice['y']})
    if len(landmarks_in_range) == 0:
        continue
~~~

~~~python
# 2. Transform each observed landmark's coordinates from the
#    particle's coordinate system to the map's coordinates.
trans_observations = []

cos_theta = np.cos(particle['t'])
sin_theta = np.sin(particle['t'])

for ob_idx, observation in enumerate(observations):
    # To map's corrdinates
    trans_x = particle['x'] + observation['x']*cos_theta - observation['y']*sin_theta
    trans_y = particle['y'] + observation['x']*sin_theta + observation['y']*cos_theta

    trans_observations.append({'x' : trans_x, 'y' : trans_y})
~~~

~~~python
# 3. Associate each transformed observation to one of the
#    predicted (selected in Step 1) landmark positions.
#    Use self.associate() for this purpose - it receives
#    the predicted landmarks and observations; and returns
#    the list of landmarks by implementing the nearest-neighbour
#    association algorithm.
ass = self.associate(landmarks_in_range, trans_observations)
particle['w'] = 1
if len(particle['assoc']) != 0:
    particle['assoc'] = []
weight_i = 1
~~~

~~~python
# 4. Calculate probability of this set of observations based on
#    a multi-variate Gaussian distribution (two variables being
#    the x and y positions with means from associated positions
#    and variances from std_landmark_x and std_landmark_y).
#    The resulting probability is the product of probabilities
#    for all the observations.
# 5. Update the particle's weight by the calculated probability.

for trans_ob, ass_pt in list(zip(trans_observations, ass)):
    trans_x = trans_ob['x']
    trans_y = trans_ob['y']
    ass_x = ass_pt['x']
    ass_y = ass_pt['y']
    sigma_x = std_landmark_x
    sigma_y = std_landmark_y
    weight_i *= 1 / (2 * np.pi * sigma_x * sigma_y) * np.exp(-0.5*(np.power((trans_x-ass_x)/sigma_x,2) + np.power((trans_y-ass_y)/sigma_y,2)))
    weight_i += 1e-60
    particle['assoc'].append(ass_pt['id'])
particle['w'] = weight_i
~~~
* `resample()`: Reconstruct the set of particles that capture the posterior belief distribution by drawing samples according to the weights.

To run the program (which generates a 2D plot), execute the following command:

```
$ python run.py
```

Without any modification to the code, you will see a resulting plot like the one below:

![Particle Filter without Proper Update & Resample][empty-update]

while a reasonable implementation of the above mentioned methods (assignments) will give you something like

![Particle Filter Example][example]

Carefully read comments in the two method bodies and write Python code that does the job.
