# Week 6 - Prediction & Behaviour Planning

---
[results]: ./result.PNG

## Assignment #1
Both input files ([`train.json`](./GNB/train.json) and [`test.json`](./GNB/test.json)) have the same format, which is a JSON-encoded representation of training data set and test data set, respectively. The format is shown below:

```
{
	"states": [[s_1, d_1, s_dot_1, d_dot_1],
	           [s_2, d_2, s_dot_2, d_dot_2],
	           ...
	           [s_n, d_n, s_dot_n, d_dot_n]
	          ],
	"labels": [L_1, L_2, ..., L_n]
}
```

The array `"states"` have a total of `n` items, each of which gives a (hypothetically) measured state of a vehicle, where `s_i` and `d_i` denote its position in the Frenet coordinate system. In addition, `s_dot_i` and `d_dot_i` give their first derivates, respectively. For each measured state, a label is associated (given in the `"labels"` array) that represents the vehicle's behaviour. The label is one of `"keep"`, `"left"`, and `"right"`, which denote keeping the current lane, making a left turn, and making a right turn, respectively.

The training set has a total of 750 data points, whereas the test set contains 250 data points with the ground truth contained in `"labels"`.

The GNB classifier is trained by computing the mean and variance of each component in the state variable for each observed behaviour. Later it is used to predict the behaviour by computing the Gaussian probability of an observed state for each behaviour and taking the maximum. You are going to implement that functionality. For convcenience, a separate function `gaussian_prob()` is already given in the module `classifier.py`.

Under the directory [./GNB](./GNB), you are given two Python modules:

* `prediction.py`: the main module you run. The `main()` function does two things: (1) read an input file ([`train.json`](./GNB/train.json)) and train the GNB (Gaussian Naive Bayes) classifier using the data stored in it, and (2) read another input file ([`test.json`](./GNB/test.json)) and make predictions for a number of data points. The accuracy measure is taken and displayed.

TODO 모두 작성후 [`./prediction.py`] 실행 결과:

![Result after prediction.py][results]
* `classifier.py`: main implementation of the GNB classifier. You shall implement two methods (`train()` and `precict()`), which are used to train the classifier and make predictions, respectively.
---
1. 각 클래스에 포함되는 데이터의 평균과 분산 계산
2. 이후 'predict()'에서 observation을 'gaussian_prob()'ㅇ르 통해 계산
3. 그중 가장 큰 class를 return
---
~~~python
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
~~~

---