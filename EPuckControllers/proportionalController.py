# -*- coding: utf-8 -*-
"""
minimal template for BasicEPuck.ePuckVRep
for usage with ePuckS5V12.ttm

@author: hoch ralph
"""
import time
from BasicEPuck.ePuckVRep import EPuckVRep
from Behavior.FollowWallBeavhior import FollowWallBehavior
from Behavior.NavigateToBlobBehavior import NavigateToBlobBehavior
from Behavior.ReturnToWallBehavior import ReturnToWallBehavior


def main():
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableCamera()
    robot.enableAllSensors()
    robot.setSensesAllTogether(False)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    noDetectionDistance = 0.5 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense

    behaviors = [NavigateToBlobBehavior(), ReturnToWallBehavior(), FollowWallBehavior()]

    # main sense-act cycle
    while robot.isConnected():
        # print( 'proximity: ', robot.getProximitySensorValues())
        # print( 'ground: ', robot.getGroundSensorValues())
        # print( 'acceleration: ', robot.getAccelerometerValues())
        # print( 'wheel encoding: ', robot.getWheelEncoderValues())

        robot.fastSensingOverSignal()

        # sense
        image = robot.getCameraImage()
        distVector = robot.getProximitySensorValues()

        # plan
        leftMotor = 0
        rightMotor = 0

        for behavior in behaviors:
            applicable = behavior.is_applicable(distVector, noDetectionDistance, image)
            if applicable:
                leftMotor, rightMotor = behavior.calculate_motor_value(distVector, noDetectionDistance, image)
                break

        # act
        robot.setMotorSpeeds(leftMotor, rightMotor)

        time.sleep(0.05)

    robot.disconnect()


if __name__ == '__main__':
    main()
