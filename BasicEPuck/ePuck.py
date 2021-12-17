# -*- coding: utf-8 -*-
"""
Created on Wed Jul 19 16:40:22 2017
the class "EPuck" encapsulates the ePuck robot for a controller:  it represents a high-level proxy of the ePuck for the controller.
Controllers may communicate with the robot either by direct call of a method of ePuck, or by registering
as observers to sensor value changes.
It has proximity sensors, light sensors, a camera, and an acceleration sensor.


@author: hans vollbrecht
"""
import time
from threading import Thread, Lock
import logging
import numpy as np
from PIL import Image as I
from .differentialWheels import DifferentialWheels


class EPuck(DifferentialWheels):

    def __init__(self, name):
        """
        constructor for a proxy object representing an ePuck robot with
        :param
            name: string
                must be the name of a robot node in the Vrep scene
        """
        self._image = I.new('RGB', (64, 64))
        self._resolX = self._image.size[0]
        self._resolY = self._image.size[1]
        self._senseCycleTime = 0.05  # in seconds
        self._cameraCycleTime = 0.5  # in seconds
        self._hasOwnSensingThread = False
        self._hasOwnCameraThread = False
        self._resultLock = Lock()
        self._imageLock = Lock()
        self._sensesAllTogether = False

        super(EPuck, self).__init__(name)


    def _connect(self):
        pass


    def _initRobotModel(self):
        # get wheel diameters from simulator, and set maximum velocity to the simulator

        self._wheelDiameter = 0.0425  # in meters
        self._wheelDistance = 0.0541  # in meters
        self._numProximitySensors = 8
        self._numLightSensors = 8



    def _getProximitySensorValues(self):
        """
        :return: numpy float array
            current values of enabled proximity sensors
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    def _getLightSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled light sensors.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    def _getGroundSensorValues(self):
        """
        :return: numpy 3x1 float array
            current (latest) values of enabled ground sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    def _getAccelerometerValues(self):
        """
        :return: numpy float array ([x, y, z])
            current (latest) values of enabled acceleration sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    def _getWheelEncodingValues(self):
        """
        :return: numpy float array ([left,right])
            current (latest) values of enabled wheel encoding sensor, in radians.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    def _getPose(self):
        """
        :return: float triple ( (x, y, theta) )
            current (latest) values of estimated robot pose, or measured by sensors, in global reference frame
            Encapsulates whether this is calculated by odometry, or by a request for sensing, or just returning values
            calculated before by some other method.
        """
        pass


    def _getCameraImage(self):
        """
        :return: a PIL Image
            current (latest) camera image
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    def setSensesAllTogether(self, sensesAllTogether = True):
        """
        see the following functions for effect
        :param
            sensesAllTogether: Boolean
                if True, sensing works only by calls to function "fastSensing.."
        :return:
        """
        self._sensesAllTogether = sensesAllTogether


    def getProximitySensorValues(self):
        """
        :return: numpy float array
            current values of enabled proximity sensors
            Encapsulates whether this implies a request for sensing or not.
        """
        if self._hasOwnSensingThread:
            self._resultLock.acquire()
            values = self._proximitySensorValues
            self._resultLock.release()
            return values
        elif self._sensesAllTogether:
            return self._proximitySensorValues
        else:
            return self._getProximitySensorValues()


    def getLightSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled light sensors.
            Encapsulates whether this implies a request for sensing or not.
        """
        if self._hasOwnSensingThread:
            self._resultLock.acquire()
            values = self._lightSensorValues
            self._resultLock.release()
            return values
        elif self._sensesAllTogether:
            return self._lightSensorValues
        else:
            return self._getLightSensorValues()



    def getGroundSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled ground sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        if self._hasOwnSensingThread:
            self._resultLock.acquire()
            values = self._groundSensorValues
            self._resultLock.release()
            return values
        elif self._sensesAllTogether:
            return self._groundSensorValues
        else:
            return self._getGroundSensorValues()


    def getAccelerometerValues(self):
        """
        :return: numpy float array ([x, y, z])
            current (latest) values of enabled acceleration sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        if self._hasOwnSensingThread:
            self._resultLock.acquire()
            values = self._accelerometerValues
            self._resultLock.release()
            return values
        elif self._sensesAllTogether:
            return self._accelerometerValues
        else:
            return self._getAccelerometerValues()


    def getWheelEncoderValues(self):
        """
        :return: numpy float array ([left,right])
            current (latest) values of enabled wheel encoding sensor, in radians.
            Encapsulates whether this implies a request for sensing or not
        """
        if self._hasOwnSensingThread:
            self._resultLock.acquire()
            values = self._wheelEncoderValues
            self._resultLock.release()
            return values
        elif self._sensesAllTogether:
            return self._wheelEncoderValues
        else:
            return self._getWheelEncodingValues()


    def getPose(self):
        """
        :return: float triple ( (x, y, theta) )
            current (latest) values of estimated robot pose, or measured by sensors, in global reference frame
            Encapsulates whether this is calculated by odometry, or by a request for sensing, or just returning values
            calculated before by some other method.
        """
        if self._hasOwnSensingThread:
            self._resultLock.acquire()
            values = self._pose
            self._resultLock.release()
            return values
        else:
            return self._getPose()


    def getCameraImage(self):
        """
        :return: a PIL Image
            current (latest) camera image
            Encapsulates whether this implies a request for sensing or not.
        """
        if self._hasOwnCameraThread:
            self._imageLock.acquire()
            im = self._image
            self._imageLock.release()
            return im
        else:
            return self._getCameraImage()


    def _setMotorSpeeds(self, leftSpeed, rightSpeed):
        pass


    def senseAllTogether(self):
        pass


    class SensingThread(Thread):
        """
        this class runs a thread in which we cyclically read the sensor values (except camera images which have
        a separate thread, see below) from the robot, in the cycle time as passed by the parameter. This cycletime may be
        adapted from the outer class

        :return:

        todo:  make it an Observable
        """

        def __init__(self, outer_instance, sense_cycletime):
            """

            :param
            outer_instance:  EPuck
                reference to outer EPuck object for calling sensing methods and setting result of sensing in attributes
            :param
            sense_cycletime:  Float
                cycle time for periodically sensing, in seconds
            """
            Thread.__init__(self)
            self._outer_instance = outer_instance
            self._sense_cycletime = sense_cycletime


        def run(self):
            while self._outer_instance.isConnected():
                changed = []

                if self._outer_instance._sensesAllTogether:
                    self._outer_instance._resultLock.acquire()
                    self._outer_instance.senseAllTogether()
                    self._outer_instance._resultLock.release()
                else:
                    if  self._outer_instance._proximitySensorsEnabled:
                        proxValues = self._outer_instance._getProximitySensorValues()
                        self._outer_instance._resultLock.acquire()
                        self._outer_instance._proximitySensorValues = proxValues
                        self._outer_instance._resultLock.release()
                        changed.append('proximity')
                    if  self._outer_instance._lightSensorsEnabled:
                        lightValues = self._outer_instance._getLightSensorValues()
                        self._outer_instance._resultLock.acquire()
                        self._outer_instance._lightSensorValues = lightValues
                        self._outer_instance._resultLock.release()
                        changed.append('light')
                    if  self._outer_instance._groundSensorEnabled:
                        groundValues = self._outer_instance._getGroundSensorValues()
                        self._outer_instance._resultLock.acquire()
                        self._outer_instance._groundSensorValues = groundValues
                        self._outer_instance._resultLock.release()
                        changed.append('ground')
                    if  self._outer_instance._accelerometerEnabled:
                        accelValues = self._outer_instance._getAccelerometerValues()
                        self._outer_instance._resultLock.acquire()
                        self._outer_instance._accelerometerValues = accelValues
                        self._outer_instance._resultLock.release()
                        changed.append('acceleration')
                    if self._outer_instance._wheelEncodingSensorEnabled:
                        wheelValues = self._outer_instance._getAccelerometerValues()
                        self._outer_instance._resultLock.acquire()
                        self._outer_instance._wheelEncoderValues = wheelValues
                        self._outer_instance._resultLock.release()
                        changed.append('wheelEncoding')
                    if self._outer_instance._poseEnabled:
                        pose = self._outer_instance._getPose()
                        self._outer_instance._resultLock.acquire()
                        self._outer_instance._pose = pose
                        self._outer_instance._resultLock.release()
                        changed.append('pose')

                if len(changed) > 0:
                    self._outer_instance.setChanged()
                    self._outer_instance.notifyObservers(changed)

                time.sleep(self._sense_cycletime)

            logging.info("SensingThread finished")



    class ImageThread(Thread):
        """
        this class runs a thread in which cyclically we read a camera image from VRep, in the cycle time as passed by
        the parameter. This cycletime may be adapted from the outer class

        """

        def __init__(self, outer_instance, camera_cycletime):
            """
            :param
            outer_instance:  EPuck
                reference to outer EPuck object for calling sensing methods and setting result of sensing in attributes
            :param
            sense_cycletime:  Float
                cycle time for periodically sensing, in seconds
            """
            Thread.__init__(self)
            self._outer_instance = outer_instance
            self._camera_cycletime = camera_cycletime


        def run(self):
            while self._outer_instance.isConnected():
                if  self._outer_instance._cameraEnabled:
                    im = self._outer_instance._getCameraImage()
                    self._outer_instance._imageLock.acquire()
                    self._outer_instance._image = im
                    self._outer_instance._imageLock.release()
                    self._outer_instance.setImageChanged()
                    self._outer_instance.notifyImageObservers(im)

                time.sleep(self._camera_cycletime)

            logging.info("IMageThread finished")



    def createSensingThread(self, senseCycleTime=0.05):
        if not (self._proximitySensorsEnabled or self._lightSensorsEnabled or self._groundSensorsEnabled or
                self._accelerometerEnabled):
            logging.error('cannot read any sensor if no one is enabled')
            return

        self._senseCycleTime = senseCycleTime
        self._sensingThreadObject = EPuck.SensingThread(self, senseCycleTime)
        self._sensingThreadObject.start()
        self._hasOwnSensingThread = True


    def createImageThread(self, cameraCycleTime=0.5):
        if not self._cameraEnabled:
            logging.error('cannot read camera image if not enabled')
            return
        self._cameraCycleTime = cameraCycleTime
        self._imageThreadObject = EPuck.ImageThread(self, cameraCycleTime)
        self._imageThreadObject.start()
        self._hasOwnCameraThread = True


    def enableCamera(self):
        super(EPuck, self).enableCamera()
        #todo: get camera resolution from robot, and get first camera image (comment seems to be obsolete. HV, 29.9.2017)


