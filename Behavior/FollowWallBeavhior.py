from Behavior.BaseBehavior import BaseBehavior


class FollowWallBehavior(BaseBehavior):

    def is_applicable(self, distances: list, no_detection_distance: int) -> bool:
        return distances[2] < 0.05

    def calculate_motor_value(self, distances: list, no_detection_distance: int) -> (float, float):
        # Initial values
        vel_left = 1
        vel_right = 1

        # Calculate velocity proportionally
        print("Following Wall")
        vel_left = vel_left + (1 - (distances[2] / no_detection_distance)) * 1.0
        vel_right = vel_right + (1 - (distances[2] / no_detection_distance)) * -0.6

        # Counter-steer additionally if wall in front is detected
        vel_left = vel_left + (1 - (distances[4] / no_detection_distance)) * 0.6
        vel_right = vel_right + (1 - (distances[4] / no_detection_distance)) * -0.4

        return vel_left, vel_right
