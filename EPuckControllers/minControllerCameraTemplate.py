# -*- coding: utf-8 -*-
"""
minimal template for BasicEPuck.ePuckVRep
for usage with ePuckS5V12.ttm

@author: hoch ralph, hans vollbrecht
"""
import time
from BasicEPuck.ePuckVRep import EPuckVRep
from PIL import Image as imagef

resolX, resolY = 64, 64


def detectBox(image: imagef):
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
    for y in range(resolY):
        blobwidth = 0
        for x in range(resolX):
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
            # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
            return True

    return False


def calculateMotorValues():
    """
    TODO: include parameters
    :return: (float,float)
        left and right motor velocity
    """
    # maximum velocity = ~2 Rad
    maxVel = 120 * 3.1415 / 180
    # TODO: calculate left and right motor velocity
    velRight = 0
    velLeft = 0

    return velLeft, velRight


def main():
    image = I.new("RGB", (resolX, resolY), "white")
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableCamera()
    robot.enableAllSensors()
    robot.setSensesAllTogether(False)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    noDetectionDistance = 0.05 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense

    # main sense-act cycle
    while robot.isConnected():
        # print( 'proximity: ', robot.getProximitySensorValues())
        # print( 'ground: ', robot.getGroundSensorValues())
        # print( 'acceleration: ', robot.getAccelerometerValues())
        # print( 'wheel encoding: ', robot.getWheelEncoderValues())

        robot.fastSensingOverSignal()

        # sense
        distVector = robot.getProximitySensorValues()
        groundValues = robot.getGroundSensorValues()

        image = robot.getCameraImage()
        print(detectBox(image))

        # plan
        leftMotor, rightMotor = calculateMotorValues()

        # act
        robot.setMotorSpeeds(leftMotor, rightMotor)

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()
