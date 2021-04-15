# Week 2 - Markov Localization

---

[//]: # (Image References)
[plot]: ./myresult.gif

## Assignment

You will complete the implementation of a simple Markov localizer by writing the following two functions in `markov_localizer.py`:

* `motion_model()`: For each possible prior positions, calculate the probability that the vehicle will move to the position specified by `position` given as input.
'''python
for i in range(map_size):
    prior = priors[i]
    p_trans = norm_pdf(position - i, mov, stdev)
    position_prob += prior * p_trans
'''
* `observation_model()`: Given the `observations`, calculate the probability of this measurement being observed using `pseudo_ranges`.
'''python
if len(observations) == 0 or len(observations) > len(pseudo_ranges):
    return 0

for i in range(len(observations)):
    distance_prob *= norm_pdf(observations[i], pseudo_ranges[i], stdev)
'''

TODO 모두 작성후 main 실행 결과:

![Expected Result of Markov Localization][plot]