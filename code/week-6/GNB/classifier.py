import numpy as np
import random
from math import sqrt, pi, exp

def gaussian_prob(obs, mu, sig):
    # Calculate Gaussian probability given
    # - observation
    # - mean
    # - standard deviation
    num = (obs - mu) ** 2
    denum = 2 * sig ** 2
    norm = 1 / sqrt(2 * pi * sig ** 2)
    return norm * exp(-num / denum)

# Gaussian Naive Bayes class
class GNB():
    # Initialize classification categories
    def __init__(self):
        self.classes = ['left', 'keep', 'right']

    # Given a set of variables, preprocess them for feature engineering.
    def process_vars(self, vars):
        # The following implementation simply extracts the four raw values
        # given by the input data, i.e. s, d, s_dot, and d_dot.
        s, d, s_dot, d_dot = vars
        return s, d, s_dot, d_dot

    # Train the GNB using a combination of X and Y, where
    # X denotes the observations (here we have four variables for each) and
    # Y denotes the corresponding labels ("left", "keep", "right").
    def train(self, X, Y):
        '''
        Collect the data and calculate mean and standard variation
        for each class. Record them for later use in prediction.
        '''
        # TODO: implement code.
        label = dict()

        for idx in self.classes:
            label[idx] = np.empty((4,0))

        for x,y in zip(X,Y):
            value = np.array([[x[0]], [(x[1] % 4)], [x[2]], [x[3]]])
            label[y] = np.append(label[y], value, axis=1)

        means = dict()
        stddevs = dict()

        for idx in self.classes:
            class_array = np.array(label[idx])
            means[idx] = np.mean(class_array, axis=1)
            stddevs[idx] = np.std(class_array, axis=1)

        self.means = means
        self.stddevs = stddevs


    # Given an observation (s, s_dot, d, d_dot), predict which behaviour
    # the vehicle is going to take using GNB.
    def predict(self, observation):
        '''
        Calculate Gaussian probability for each variable based on the
        mean and standard deviation calculated in the training process.
        Multiply all the probabilities for variables, and then
        normalize them to get conditional probabilities.
        Return the label for the highest conditional probability.
        '''
        # TODO: implement code.
        probs = dict()

        for idx in self.classes:
            cur_prob = 1.0
            # Multiply all the probabilities for variables
            for idx2 in range(len(observation)):
                cur_prob *= gaussian_prob(observation[idx2], self.means[idx][idx2], self.stddevs[idx][idx2])

            probs[idx] = cur_prob

        # Get hightest conditinal probability & label
        prob = 0
        predict_class = "keep"

        for idx in self.classes:
            if probs[idx] > prob:
                prob = probs[idx]
                predict_class = idx

        return predict_class

