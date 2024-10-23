# Type of planner
import math

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

class planner:
    def __init__(self, type_, trajectory_type='parabola'): # NOTE: Change to 'sigmoid' for sigmoid trajectory
        self.type = type_
        self.trajectory_type = trajectory_type

    def plan(self, goalPoint=[-1.0, -1.0]):
        if self.type == POINT_PLANNER:
            return self.point_planner(goalPoint)
        elif self.type == TRAJECTORY_PLANNER:
            return self.trajectory_planner()

    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # DONE Part 6: Implement the trajectories here
    def trajectory_planner(self):
        trajectory_points = []

        # Parabola: y = x^2 for x in [0.0, 1.5]
        if self.trajectory_type == 'parabola':
            x_values = [i * 0.1 for i in range(16)]
            for x in x_values:
                y = x ** 2
                trajectory_points.append([x, y])

        # Sigmoid: σ(x) = 2 / (1 + e^(-2x)) - 1 for x in [0.0, 2.5]
        elif self.trajectory_type == 'sigmoid':
            x_values = [i * 0.1 for i in range(26)]
            for x in x_values:
                y = 2 / (1 + math.exp(-2 * x)) - 1
                trajectory_points.append([x, y])

        # Return the list of trajectory points: [ [x1, y1], ..., [xn, yn] ]
        return trajectory_points

