from PIL import Image

from Behavior.BaseBehavior import BaseBehavior


class NavigateToBlobBehavior(BaseBehavior):
    resolX, resolY = 64, 64

    def is_applicable(self, distances: list, no_detection_distance: int, camera_image: Image) -> bool:
        return not self.detectBox(camera_image)

    def calculate_motor_value(self, distances: list, no_detection_distance: int, center_of_blob: float) -> (float, float):
        tolerance = 2
        maxVel = 120 * 3.1415 / 180

        if (self.resolX / 2 + tolerance) > center_of_blob > (self.resolX / 2 - tolerance):
            velLeft = maxVel
            velRight = maxVel
        else:  # Blob is currently not visible - turn puck
            if center_of_blob > self.resolX / 2:
                velLeft = maxVel
                velRight = -maxVel / 8
            else:
                velLeft = -maxVel / 8
                velRight = maxVel

        return velLeft, velRight

    def detectBox(self, image: Image):
        """
            looks in current image for a black blob on a red background, from left to right
            :param
                    resolX, resolY: int
                        image resolution in pixel
                    image: PIL.Image
                        a rgb image with black blobs on red background
                    xCenter: [int]
                        the center of the image: result of the function

            :return: true,  if black blob found
            """
        minBlobWidth = 5
        xStart = -1
        xCenter = [-1]
        for y in range(self.resolY):
            blobwidth = 0
            for x in range(self.resolX):
                pixel = image.getpixel((x, y))
                if pixel == (0, 0, 0):  # black pixel: a box!
                    blobwidth += 1
                    if blobwidth == 1:
                        xStart = x
                else:
                    if blobwidth >= minBlobWidth:
                        xCenter[0] = xStart + blobwidth / 2
                        # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                        return True
                    elif blobwidth > 0:
                        blobwidth = 0
            if blobwidth >= minBlobWidth:
                xCenter[0] = xStart + blobwidth / 2
                print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                return True

        return False
