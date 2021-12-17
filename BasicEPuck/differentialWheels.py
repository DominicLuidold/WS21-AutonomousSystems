# -*- coding: utf-8 -*-
"""
Created on Wed Jul 19 16:40:22 2017
the abstract base class "DifferentialWheels" encapsulates a robot with a differential wheel locomotion
(2 fixed, active wheels) for use by a controller: it represents a high-level proxy of the robot for the controller.
Controllers may communicate with the robot either by direct call of a method of DifferentialWheels, or by registering
as observers to sensor value changes.
The robot may have a distance sensor (ultrasonic or 2D-laser) or  proximity sensors (infrared), light sensors, a ground sensor,
an acceleration sensor, a wheel encoding sensor, and a camera.
Design makes no effort to further abstract the sensors, which would require a major redesign.



@author: hans vollbrecht
"""

import logging
import numpy as np

from abc import ABCMeta, abstractmethod


class DifferentialWheels:
    __metaclass__ = ABCMeta

    def __init__(self, name):
        """
        constructor for a proxy object representing a robot with
        :param
        name: string
            the unique name of a robot in a real or simulated scene

        """
        self._name = name
        self._pose = (0.0, 0.0, 0.0)   #estimated or sensor-measured pose of the robot: (x,y,theta) in global frame
        self._maxVel = 1   #in rad/sec
        self._velLeft = 0  #left wheel velocity in rad/sec
        self._velRight = 0
        self._wheelDiameter = 0.0  # in meters
        self._wheelDistance = 0.0  # in meters
        self._distanceSensorEnabled = False
        self._proximitySensorsEnabled = False
        self._numProximitySensors = 0
        self._enabledProximitySensors = np.array([], dtype=np.int)
        self._lightSensorsEnabled = False
        self._numLightSensors = 0
        self._enabledLightSensors = np.array([], dtype=np.int)
        self._groundSensorEnabled = False
        self._accelerometerEnabled = False
        self._wheelEncodingSensorEnabled = False
        self._poseEnabled = False
        self._cameraEnabled = False
        self._proximitySensorValues = np.array([], dtype=np.float)
        self._lightSensorValues = np.array([], dtype=np.float)
        self._groundSensorValues = np.array([], dtype=np.float)
        self._accelerometerValues = np.array([], dtype=np.float)
        self._wheelEncoderValues = np.array([], dtype=np.float)
        self._connectionStatus = False
        self._obs = []  #observers for sensors except cameras (usually "passive" controllers)
        self._changed = 0
        self._obsImage = []  #observers for image changes (usually "passive" controllers)
        self._changedImage = 0


        # connect to real robot or simulator
        self._connect()

        self._initRobotModel()



    @abstractmethod
    def _connect(self):     # connect to a real robot or simulated robot in a simulator
        pass


    @abstractmethod
    def _initRobotModel(self):
        """
        in DifferentialWheels, we require at least the wheel diameters and the distance between the wheels.
        if _enabledProximitySensors: we require at least number and the angular positions of the proximity sensors
        if _enabledCamera: we require at least the image type (RGB, ...) and its resolution

        """
        pass


    @abstractmethod
    def _getProximitySensorValues(self):
        """
        :return: numpy float array
            current values of enabled proximity sensors
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    @abstractmethod
    def _getLightSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled light sensors.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    @abstractmethod
    def _getGroundSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled ground sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    @abstractmethod
    def _getAccelerometerValues(self):
        """
        :return: numpy float array ([x, y, z])
            current (latest) values of enabled acceleration sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    @abstractmethod
    def _getWheelEncodingValues(self):
        """
        :return: numpy float array ([left,right])
            current (latest) values of enabled wheel encoding sensor, in radians.
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    @abstractmethod
    def _getPose(self):
        """
        :return: float triple ( (x, y, theta) )
            current (latest) values of estimated robot pose, or measured by sensors, in global reference frame
            Encapsulates whether this is calculated by odometry, or by a request for sensing, or just returning values
            calculated before by some other method.
        """
        pass


    @abstractmethod
    def _getCameraImage(self):
        """
        :return: a PIL Image
            current (latest) camera image
            Encapsulates whether this implies a request for sensing or not.
        """
        pass


    @abstractmethod
    def _setMotorSpeeds(self, leftSpeed, rightSpeed):
        """
        set the desired speeds of the left and right wheel of the robot
        :param
        leftSpeed: float
            speed in rad/sec of left motor
        :param
        rightSpeed: float
            speed in rad/sec of right motor

        """
        pass



    """
    all the following getSomeSensorValue methods are default implementations and are usually overwritten in subclasses -
    note that these implementations may lead to inefficiencies because it may cause too frequent a robot communication 
    """
    def getProximitySensorValues(self):
        """
        :return: numpy float array
            current values of enabled proximity sensors
            Encapsulates whether this implies a request for sensing or not.
        """
        return self._getProximitySensorValues()


    def getLightSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled light sensors.
            Encapsulates whether this implies a request for sensing or not.
        """
        return self._getLightSensorValues()


    def getGroundSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled ground sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        return self._getGroundSensorValues()


    def getAccelerometerValues(self):
        """
        :return: numpy float array ([x, y, z])
            current (latest) values of enabled acceleration sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        return self._getAccelerometerValues()

    def getWheelEncodingValues(self):
        """
        :return: numpy float array ([left, right])
            current (latest) values of enabled wheel encoding sensor.
            Encapsulates whether this implies a request for sensing or not.
        """
        return self._getWheelEncodingValues()

    def getPose(self):
        """
        :return: float triple ( (x, y, theta) )
            current (latest) values of estimated robot pose, or measured by sensors, in global reference frame
            Encapsulates whether this is calculated by odometry, or by a request for sensing, or just returning values
            calculated before by some other method.
        """
        return self._getPose()


    def getCameraImage(self):
        """
        :return: a PIL Image
            current (latest) camera image
            Encapsulates whether this implies a request for sensing or not.
        """
        return self._getCameraImage()

    def sensing(self):
        """
        this method activates for enabled sensors the sensing methods _get...
        that means, it activates on the robot a true sensing cycle (except for camera images).
        We expect these methods to set the corresponding sensor value attribute.
        The method notifies eventually oberservers of this object.
        :return: nothing
        """
        changed = []
        if self._proximitySensorsEnabled:
            self._getProximitySensorValues()
            changed.append('proximity')
        if self._lightSensorsEnabled:
            self._getLightSensorValues()
            changed.append('light')
        if self._groundSensorEnabled:
            self._getGroundSensorValues()
            changed.append('ground')
        if self._accelerometerEnabled:
            self._getAccelerometerValues()
            changed.append('acceleration')
        if self._wheelEncodingSensorEnabled:
            self._getWheelEncodingValues()
            changed.append('wheelEncoding')
        if self._poseEnabled:
            self._getPose()
            changed.append('pose')

        if len(changed) > 0:
            self.setChanged()
            self.notifyObservers(changed)
        else:
            logging.error('cannot read sensor values if nothing is enabled')


    def sensingCamera(self):
        """
        this methods activates a camera reading on the robot
        we expect the method _getCameraImage to set an image attribute
        :return:
        """
        if self._cameraEnabled:
            self._getCameraImage()
            self.setImageChanged()
            self.notifyImageObservers()
        else:
            logging.error('cannot read image if camera is not enabled')


    def enableAllSensors(self):
        """
        all but camera and pose which must be enabled explicitely!

        :return:
        """
        self.enableProximitySensors()
        self.enableGroundSensor()
        self.enableAccelerometer()
        self.enableWheelEncoding()
        # light sensors not yet implemented
        # self.enableLightSensors()


    def enableDistanceSensor(self):
        self._distanceSensorEnabled = True


    def enableProximitySensors(self, sensorIDs=[]):
        """
        we assume the robot has _numProximitySensors > 0 proximity sensors some of which get enabled:
        :param
        sensorIDs : numpy integer array
            sensorIDs contains the indices of the proximity sensors to enable; if empty: enable all sensors
        """

        if self._numProximitySensors > 0:
            self._proximitySensorsEnabled = True
            if len(sensorIDs) == 0:
                self._enabledProximitySensors = np.array(range(self._numProximitySensors))
            else:
                if sensorIDs.max() < self._numProximitySensors:
                    raise Exception('proximity sensor id out of range')
                else:
                    self._enabledProximitySensors = np.array(range(self._numProximitySensors))[sensorIDs]
        else:
            raise Exception('cannot enable _numProximitySensors=0 sensors')


    def enableLightSensors(self, sensorIDs=[]):
        """
        we assume the robot has _numLightSensors > 0 light sensors some of which get enabled:
        :param
        sensorIDs : numpy integer array
            sensorIDs contains the indices of the light sensors to enable
        """
        if self._numLightSensors > 0:
            self._lightSensorsEnabled = True
            if len(sensorIDs) == 0:
                self._enabledLightSensors = np.array(range(self._numLightSensors))
            else:
                if sensorIDs.max() < self._numLightSensors:
                    raise Exception('light sensor id out of range')
                else:
                    self._enabledLightSensors = np.array(range(self._numLightSensors))[sensorIDs]
        else:
            raise Exception('cannot enable _numLightSensors=0 sensors')


    def enableGroundSensor(self):
        self._groundSensorEnabled = True


    def enableAccelerometer(self):
        self._accelerometerEnabled = True


    def enableWheelEncoding(self):
        self._wheelEncodingSensorEnabled = True

    def enablePose(self):
        self._poseEnabled = True

    def enableCamera(self):
        self._cameraEnabled = True


    def isConnected(self):
        return self._connectionStatus


    def setMotorSpeeds(self, leftSpeed, rightSpeed):
        """
        set the desired speeds of the left and right wheel of the robot
        :param
        leftSpeed: float
            speed in rad/sec of left motor
        :param
        rightSpeed: float
            speed in rad/sec of right motor

        """
        #set robot velocity attributes eventually clamping them to upper or lower bound
        self._velLeft = max(min(leftSpeed, self._maxVel), -self._maxVel)
        self._velRight = max(min(rightSpeed, self._maxVel), -self._maxVel)

        self._setMotorSpeeds(self._velLeft, self._velRight)


    def getMotorSpeeds(self):
        return self._velLeft, self._velRight   #in rad/sec


    """
    when sensing runs inside of threads (sensors and/or camera thread), we assume there will be somer passive
    controller which will "observe" the robot for new sensor/image values.
    This is offered by the subsequent methods.
    Notifying occurs either when one of the two sensing methods (see above) gets called (usually by an active controller),
    or when sensor/image values get updated by independent robot threads reading cyclically values from the robot 
    """

    def addObserver(self,  observer):
        if observer not in self.obs:
            self._obs.append(observer)

    def deleteObserver(self, observer):
        self._obs.remove(observer)

    def deleteObservers(self):
        self._obs = []

    def notifyObservers(self, arg=None):
        """
        If '_changed' indicates that this object
        has changed, notify all its observers, then
        call clearChanged(). Each observer has its
        update() called with two arguments: this
        observable object and the generic 'arg'.
        """

        if not self._changed:
            return

        localArray = self._obs[:]

        self.clearChanged()

        # Updating is not required to be synchronized:
        for observer in localArray:
            observer.update(self, arg)


    def setChanged(self):
        self._changed = 1

    def clearChanged(self):
        self._changed = 0

    def hasChanged(self):
        return self._changed

    def addImageObserver(self,  observer):
        if observer not in self._obsImage:
            self._obsImage.append(observer)

    def deleteImageObserver(self, observer):
        self._obsImage.remove(observer)

    def deleteImageObservers(self):
        self._obsImage = []

    def notifyImageObservers(self, arg=None):
        """
        If '_changedImage' indicates that this object
        has changed, notify all its observers, then
        call clearImageChanged(). Each observer has its
        update() called with two arguments: this
        observable object and the generic 'arg'.
        """

        if not self._changedImage:
            return

        localArray = self._obsImage[:]

        self.clearImageChanged()

        # Updating is not required to be synchronized:
        for observer in localArray:
            observer.update(self, arg)


    def setImageChanged(self):
        self._changedImage = 1

    def clearImageChanged(self):
        self._changedImage = 0

    def hasImagehanged(self):
        return self._changedImage


