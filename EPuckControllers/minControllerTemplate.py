# -*- coding: utf-8 -*-
"""
minimal template for BasicEPuck.ePuckVRep
for usage with ePuckS5V12.ttm

@author: hoch ralph
"""
import time
from BasicEPuck.ePuckVRep import EPuckVRep

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
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

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

        # plan
        leftMotor, rightMotor = calculateMotorValues()

        # act
        robot.setMotorSpeeds(leftMotor, rightMotor)

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()
