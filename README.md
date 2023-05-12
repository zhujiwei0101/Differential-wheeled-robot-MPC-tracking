# Differential-wheeled-robot-MPC-tracking
## Differential wheeled robot model
### Kinematics of Differential Drive Robots
```math
\begin{bmatrix} \dot x \\ \dot y \\ \dot \theta \end{bmatrix} = 
\begin{bmatrix} \cos \theta & 0 \\ 0 & \sin\theta \\ 0 & 1 \end{bmatrix}
\begin{bmatrix} v \\ \omega \end{bmatrix}
```
where where $v$ and $\omega$  are the control variables.

### Linearization
