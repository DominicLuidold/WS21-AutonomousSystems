from Behavior.BaseBehavior import BaseBehavior


class FollowWallBehavior(BaseBehavior):

    def is_applicable(self, distances: list, no_detection_distance: int) -> bool:
        return distances[2] < 0.2

    def calculate_motor_value(self, distances: list, no_detection_distance: int) -> (float, float):
        # Initial values
        vel_left = 1
        vel_right = 1

        print("Sensor 2: ", distances[2])

        vel_left = vel_left + (1 - (distances[2] / no_detection_distance)) * 0.9
        vel_right = vel_right + (1 - (distances[2] / no_detection_distance)) * -0.6

        return vel_left, vel_right
