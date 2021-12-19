from PIL import Image

from Behavior.BaseBehavior import BaseBehavior


class NavigateToBlobBehavior(BaseBehavior):
    resolution_x, resolution_y = 64, 64

    def is_applicable(self, distances: list, no_detection_distance: int, camera_image: Image) -> bool:
        blob_detected, center_of_blob = self.__detect_box(camera_image)

        return not (blob_detected and (distances[3] < 0.05 and distances[4] < 0.05))

    def calculate_motor_value(self, distances: list, no_detection_distance: int, camera_image: Image) -> (float, float):
        # Initial values
        max_vel = 120 * 3.1415 / 180
        vel_left, vel_right = max_vel, max_vel
        tolerance = 4
        blob_detected, center_of_blob = self.__detect_box(camera_image)

        print("Navigating to Blob, detected=" + str(blob_detected) + " with center=" + str(center_of_blob))
        if not ((self.resolution_x / 2 + tolerance) > center_of_blob > (self.resolution_x / 2 - tolerance)):
            # Blob is currently not visible - turn puck
            if center_of_blob > self.resolution_x / 2:
                vel_left = max_vel / 5
                vel_right = -max_vel / 10
            else:
                vel_left = -max_vel / 10
                vel_right = max_vel / 5

        return vel_left, vel_right

    def __detect_box(self, image: Image) -> (bool, float):
        """
            Looks in current image for a black blob on a red background, from left to right
            :param
                    image: PIL.Image
                        a rgb image with black blobs on red background

            :return:
                true and xCenter, if black blob found
                false and None otherwise
            """
        minBlobWidth = 5
        xStart = -1
        xCenter = [-1]
        for y in range(self.resolution_y):
            blob_width = 0
            for x in range(self.resolution_x):
                pixel = image.getpixel((x, y))
                if pixel == (0, 0, 0):  # black pixel: a box!
                    blob_width += 1
                    if blob_width == 1:
                        xStart = x
                else:
                    if blob_width >= minBlobWidth:
                        xCenter[0] = xStart + blob_width / 2
                        # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                        return True, xCenter[0]
                    elif blob_width > 0:
                        blob_width = 0
            if blob_width >= minBlobWidth:
                xCenter[0] = xStart + blob_width / 2
                # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                return True, xCenter[0]

        return False, -1
