## Predictive controler implementation in C++

This is my thesis project implementation of predictive controller in C++, simple predictive controller for first order systems with dead time and online method of least squares for identify plants.

1. Every library have example .ino included.
2. ManualControl contains identifyPlant method which use online least squares.
3. PredictiveController contains logic for creating GPC matrices for predictive control law (u=K(w-fp*xp) according to [1].
4. PredictiveDTController conations simple implementation of Generalized Predictive Control for Industrial Processes from [1].
5. PIDController contains incremental PID controller.

Libraries were tested on simple plant of two RC itegrators (R=1k,C=100uF) with Arduino UNO.

[MatrixMath library](https://github.com/eecharlie/MatrixMath) were used for matrices operations.


[1] CAMACHO, E. F. a C. BORDONS. *Model predictive control*. New York: Springer, c2004. ISBN 1-85233-694-3.