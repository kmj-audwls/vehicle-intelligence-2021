# Week 3 - Kalman Filters, EKF and Sensor Fusion

---

[//]: # (Image References)
[EKF-results]: ./EKF/myresult.png

## Assignment - EFK & Sensor Fusion Example
TODO 모두 작성후 [`./run.py`] 실행 결과:

![Testing of EKF with Sensor Fusion][EKF-results]

### Assignment

Complete the implementation of EKF with sensor fusion by writing the function `update_ekf()` in the module `kalman_filter`. Details are given in class and instructions are included in comments.

~~~python
        H_j = Jacobian(self.x)

        S = np.dot(np.dot(H_j,self.P),H_j.T) + self.R

        K = np.dot(np.dot(self.P, H_j.T), np.linalg.inv(S))

        px, py, vx, vy = self.x

        h_x_1 = px * px + py * py
        h_x_2 = sqrt(h_x_1)
        h_x_3 = atan2(py, px)
        h_x_4 = (px*vx+py*vy)/h_x_2

        y = z - [h_x_2, h_x_3, h_x_4]

        if y[1] > np.pi:
            y[1] = y[1] - 2*np.pi

        elif y[1] < -np.pi:
            y[1] = y[1] + 2*np.pi

        self.x = self.x + np.dot(K, y)

        self.P = np.dot((np.eye(4, dtype=float)-np.dot(K, H_j)),self.P)
~~~
