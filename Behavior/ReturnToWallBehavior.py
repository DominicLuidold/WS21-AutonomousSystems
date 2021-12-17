from Behavior.BaseBehavior import BaseBehavior


class ReturnToWallBehavior(BaseBehavior):

    def is_applicable(self, distances: list, no_detection_distance: int) -> bool:
        return distances[2] >= 0.2

    def calculate_motor_value(self, distances: list, no_detection_distance: int) -> (float, float):
        vel_left = 1
        vel_right = 1

        print("Returning to Wall - Sensor 2", distances[2])

        vel_right = vel_right + 0.3

        return vel_left, vel_right
