from PIL import Image

from Behavior.BaseBehavior import BaseBehavior


class ReturnToWallBehavior(BaseBehavior):

    def is_applicable(self, distances: list, no_detection_distance: int, camera_image: Image) -> bool:
        return distances[2] >= 0.05

    def calculate_motor_value(self, distances: list, no_detection_distance: int, camera_image: Image) -> (float, float):
        # Initial values
        vel_left = 1
        vel_right = 1

        # Calculate velocity proportionally
        print("Returning to Wall")
        vel_left = vel_left + (1 - (distances[2] / no_detection_distance)) * 0.7
        vel_right = vel_right + (1 - (distances[2] / no_detection_distance)) * 1.3

        return vel_left, vel_right
