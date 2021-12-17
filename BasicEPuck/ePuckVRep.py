# -*- coding: utf-8 -*-
"""
Created on Wed Jul 19 16:40:22 2017
the class "EPuckVRep" encapsulates the ePuck robot in a scene of the VRep simulator for a controller: it represents
a proxy of the ePuck for the controller.
It has proximity sensors, a ground sensor, a camera, and an acceleration sensor.
ToDo: wheels encoding sensor, light sensors



@author: hans vollbrecht
"""

import logging
import numpy as np
from PIL import Image as I
from .ePuck import EPuck

try:
    import vrep
    import vrepConst
except:
    print ('--------------------------------------------------------------')
    print ('"vrep.py" could not be imported. This means very probably that')
    print ('either "vrep.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "vrep.py"')
    print ('--------------------------------------------------------------')
    print ('')


class EPuckVRep(EPuck):

    def __init__(self, name, host_url='127.0.0.1', port=20000, synchronous=False):
        """
        constructor for a robot that connects to the VRep Simulator on port
        :param
            name: string
                must be the name of a robot node in the Vrep scene
            host: string
                the host url of the VRep simulator
            port: integer
                the port for communication with the robot node in Vrep
            synchronous: boolean
                the simulator is to run strictly synchronously with the controller
        """
        self.__port = port
        self.__host_url = host_url
        self.__synchronous = synchronous
        self.__clientID = -1
        self.__signalName = 'epuck' + str(port)
        self.__empty_buff = bytearray()

        super(EPuckVRep, self).__init__(name)


    def _connect(self):
        self.__clientID = vrep.simxStart(self.__host_url, self.__port, True, True, 5000, 5)  # Connect to V-REP
        if self.__clientID != -1:
            print ('Connected to remote API server of Vrep with clientID: ', self.__clientID)
            self._connectionStatus = True
            vrep.simxGetStringSignal(self.__clientID, self.__signalName+'_allSens', vrep. simx_opmode_streaming)   #first time call
            vrep.simxGetStringSignal(self.__clientID, self.__signalName+'_camera', vrep. simx_opmode_streaming)   #first time call
            vrep.simxSetStringSignal(self.__clientID, self.__signalName+'_velocities', vrep.simxPackFloats([0.0, 0.0]), vrep.simx_opmode_streaming) #first time call

#            if self.__synchronous:
#                self.startsim()
        else:
            logging.error('Failed connecting to remote API server of Vrep')

    def disconnect(self):
        vrep.simxFinish(-1)  # close all opened connections
        print ('disconnected to remote API server of Vrep')

    def startsim(self):
        """
        to be called from a controller when we initialized self with synchronous=True
        Start the simulation in synchronous mode to achieve exact simulation
        independent of the framerate.
        The following comment seems to be obsolete (HV: 29.9.2017)
        To be able to do this, you have to
        connecz to the port 19997 of vrep, edit the file
        `remoteApiConnections.txt` to contain something like
            portIndex1_port           = 19997
            portIndex1_debug          = true
            portIndex1_syncSimTrigger = false
        """
#        self.set_led(8, 0)
        if not self.__synchronous:
            logging.error('startsim requires the synchronous mode to have been set in the init method')
            return

        vrep.simxStartSimulation(self.__clientID, vrep.simx_opmode_oneshot)
        vrep.simxSynchronous(self.__clientID, True)
        self.stepsim(1)


    def stepsim(self, steps):
        """
        to be called from a controller when we initialized self with synchronous=True
        Do n-steps of simulation.
        :param steps: Number of steps you want to simulate
        :type steps: int
        """
        if not self.__synchronous:
            logging.error('stepsim requires the synchronous mode to have been set in the init method')
            return
        if self._hasOwnSensingThread or self._hasOwnCameraThread:
            logging.error('stepsim assumes the synchronous mode and is incompatible with sensing or camera threads')
            return

        for i in range(steps):
            vrep.simxSynchronousTrigger(self.__clientID)


    def _initRobotModel(self):
        # get wheel diameters from simulator, and set maximum velocity to the simulator

        self._numProximitySensors = 8
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                     vrep.sim_scripttype_childscript,
                                                                                     'getWheelDiameterForRemote', [],
                                                                                     [], [], self.__empty_buff,
                                                                                     vrep.simx_opmode_blocking)

        if res == vrep.simx_return_ok:
            self._wheelDiameter = ret_floats[0]
        else:
            logging.error('Remote function call "getWheelDiameterForRemote" failed')
            self._wheelDiameter = 0.0425  # in meters

        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                     vrep.sim_scripttype_childscript,
                                                                                     'getWheelDistanceForRemote', [],
                                                                                     [], [], self.__empty_buff,
                                                                                     vrep.simx_opmode_blocking)

        if res == vrep.simx_return_ok:
            self._wheelDistance = ret_floats[0]
        else:
            logging.error('Remote function call "getWheelDistanceForRemote" failed')
            self._wheelDistance = 0.0623  # in meters


        self._maxVel = 120 * np.pi / 180    # to be verified on real ePuck

        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                      vrep.sim_scripttype_childscript,
                                                                                      'setMaxVelocityForRemote', [],
                                                                                      [self._maxVel], [], self.__empty_buff,
                                                                                      vrep.simx_opmode_blocking);
        if res != vrep.simx_return_ok:
            logging.error('Remote function call "setMaxVelocity" failed')

        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                     vrep.sim_scripttype_childscript,
                                                                                     'getObjectSizeFactorForRemote', [],
                                                                                     [], [], self.__empty_buff,
                                                                                     vrep.simx_opmode_blocking)
        # self._s : ObjectSizeFactor, see Vrep model of ePuck
        if res == vrep.simx_return_ok:
            self._s = ret_floats[0]
        else:
            logging.error('Remote function call getObjectSizeFactorForRemote failed')
            self._s = 1

        self._noDetectionDistance = 0.05 * self._s


    def getS(self):
        return self._s


    def _getProximitySensorValues(self):
        """
        :return: numpy float array
            current values of enabled proximity sensors
            This always implies a request to the robot for sensing proximities .
        """

        if not self._proximitySensorsEnabled:
            logging.error('cannot read proximity sensors if not enabled')
            return self._proximitySensorValues

        # read distance sensor values from VRep ePuck simulator
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                      vrep.sim_scripttype_childscript,
                                                                                      'getProxSensorsForRemote', [],
                                                                                      [], [], self.__empty_buff,
                                                                                      vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok:
            self._proximitySensorValues = np.array(ret_floats)[self._enabledProximitySensors]
            logging.debug('dist values: ', self._proximitySensorValues)
        else:
            logging.warning('Remote function call getProxSensorsForRemote failed: last proximity values used')
            # self._proximitySensorValues = np.full((8, 1), self._noDetectionDistance)

        return self._proximitySensorValues


    def _getLightSensorValues(self):
        """
        :return: numpy float array
            current (latest) values of enabled light sensors.
            Encapsulates whether this implies a request for sensing or not.
        """
        logging.error('light sensors for ePuck in VRep not implemented yet')



    def _getGroundSensorValues(self):
        """
        :return: numpy 3x1 float array
            current (latest) values of enabled ground sensor.
            This always implies a request to the robot for sensing the ground.
        """
        if not self._groundSensorEnabled:
            logging.error('cannot read ground sensor if not enabled')
            return self._groundSensorValues

        # read distance sensor values from VRep ePuck simulator
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                      vrep.sim_scripttype_childscript,
                                                                                      'getGroundSensorForRemote', [],
                                                                                      [], [], self.__empty_buff,
                                                                                      vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok:
            self._groundSensorValues = np.array(ret_floats)
            logging.debug('ground values: ',self._groundSensorValues)
        else:
            logging.warning('Remote function call getGroundSensorForRemote failed: last ground values used')
            # self._proximitySensorValues = np.full((8, 1), self._noDetectionDistance)

        return self._groundSensorValues





    def _getAccelerometerValues(self):
        """
        :return: numpy float array ([x, y, z])
            current (latest) values of enabled acceleration sensor.
            This always implies a request to the robot for sensing acceleration.
        """
        if not self._accelerometerEnabled:
            logging.error('cannot read accelerometer if not enabled')
            return self._accelerometerValues

        # read distance sensor values from VRep ePuck simulator
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                      vrep.sim_scripttype_childscript,
                                                                                      'getAccelerometerForRemote', [],
                                                                                      [], [], self.__empty_buff,
                                                                                      vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok:
            self._accelerometerValues = np.array(ret_floats)
            logging.debug('accelerometer values: ',self._accelerometerValues)
        else:
            logging.warning('Remote function call getAcclerometerForRemote failed: last accelerometer values used')

        return self._accelerometerValues




    def _getWheelEncodingValues(self):
        """
        :return: numpy float array ([left,right])
            current (latest) values of enabled wheel encoding sensor, in radians.
            This always implies a request to the robot for sensing wheel encodings.
        """
        if not self._wheelEncodingSensorEnabled:
            logging.error('cannot read wheel encoding if not enabled')
            return self._wheelEncoderValues

        # read wheel encoding sensor values from VRep ePuck simulator
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                      vrep.sim_scripttype_childscript,
                                                                                      'getWheelEncodingSensorForRemote', [],
                                                                                      [], [], self.__empty_buff,
                                                                                      vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok:
            self._wheelEncoderValues = np.array(ret_floats)
            logging.debug('wheel encoder values: ',self._wheelEncoderValues)
        else:
            logging.warning('Remote function call getWheelEncodingSensorForRemote failed: last wheel encoder values used')

        return self._wheelEncoderValues


    def _getPose(self):
        """

        :return: x, y, theta  (int)
            the robot position in x and y coordinates and its orientation, in the global reference frame
        """

        if not self._poseEnabled:
            logging.error('cannot read robot pose if not enabled')
            return self._pose

        # read wheel encoding sensor values from VRep ePuck simulator
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                      vrep.sim_scripttype_childscript,
                                                                                      'getPoseForRemote', [],
                                                                                      [], [], self.__empty_buff,
                                                                                      vrep.simx_opmode_blocking);
        if res == vrep.simx_return_ok:
            self._pose = tuple(ret_floats)
            logging.debug('pose: ',self._pose)
        else:
            logging.warning('Remote function call gePoseForRemote failed: last pose used')

        return self._pose



    def _getCameraImage(self):
        """
        :return: a PIL Image
            current (latest) camera image
            This always implies a request to the robot for sensing with vision sensor (camera).
        """
        if not self._cameraEnabled:
            logging.error('cannot read camera image if not enabled')
            return self._image

        """
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                        vrep.sim_scripttype_childscript,
                                                                                        'getCameraSensorsForRemote', [],
                                                                                        [], [], self.__empty_buff,
                                                                                        vrep.simx_opmode_blocking)
        """
        res, values = vrep.simxGetStringSignal(self.__clientID, self.__signalName+'_camera', vrep.simx_opmode_buffer)
        if res == vrep.simx_return_ok:
            imageValues = vrep.simxUnpackFloats(values)
            logging.debug(' camera values length: ', len(imageValues))
            vrep.simxClearStringSignal(self.__clientID, self.__signalName+'_camera', vrep.simx_opmode_buffer)

            cameraVector = np.array(imageValues)
            # map pixel values (float in [0,1]) to integers [0-255] as RGB values
            imageVrep = list(map(lambda x: np.uint8(x * 255), cameraVector))

            imageData = np.flipud(np.asarray(imageVrep).reshape((self._resolY, self._resolX, 3)))
            self._image = I.fromarray(imageData, 'RGB')


            # self._image.save('my.png')
            # self._image.show()
            return self._image

        else:
            logging.error('Remote function call getCameraSensorsForRemote failed')


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
        """
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                  vrep.sim_scripttype_childscript,
                                                                                  'setVelocitiesForRemote', [],
                                                                                  [leftSpeed, rightSpeed], [],
                                                                                  self.__empty_buff, vrep.simx_opmode_blocking)        
        """
        velocities = vrep.simxPackFloats([leftSpeed, rightSpeed])
        res = vrep.simxSetStringSignal(self.__clientID, self.__signalName+'_velocities', velocities, vrep.simx_opmode_oneshot)
        # res = vrep.simx_return_ok
        # res = vrep.simxSetFloatSignal(self.__clientID, self.__signalName+'_velLeft', 0.0, vrep.simx_opmode_oneshot)
        # res = vrep.simxSetFloatSignal(self.__clientID, self.__signalName+'_velRight', 0.0, vrep.simx_opmode_oneshot)
        if res == vrep.simx_return_ok:
            logging.debug(' motor values', leftSpeed, rightSpeed)
        else:
            logging.error('Remote function call simxSetStringSignal for signal EPUCK_velLeft/-Right failed')



    def setImageCycle(self, imageCycle):
        """
        sets the VRep ePuck parameter imageCycle
        :param
            imageCycle: int
                indicates that each "imageCycle" simulation steps, the VRep simulator provides via signal a new camera image
        :return:
        """
        res, ret_ints, ret_floats, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                vrep.sim_scripttype_childscript,
                                                                                'setImageCycleForRemote', [imageCycle],
                                                                                [], [], self.__empty_buff,
                                                                                vrep.simx_opmode_blocking)

        if res == vrep.simx_return_ok:
            logging.debug(' changed Vrep ePuck parameter imageCycle', imageCycle)
        else:
            logging.error('Remote function call setImageCycleForRemote failed')


    def fastSensingOverSignal(self):
        """
        read all non-camera sensors in one signal call, except for pose
        :return:
        """
        res, values = vrep.simxGetStringSignal(self.__clientID, self.__signalName+'_allSens', vrep.simx_opmode_buffer)
        if res == vrep.simx_return_ok:
            sensValues = vrep.simxUnpackFloats(values)
            self._proximitySensorValues = np.array(sensValues[:8], dtype=np.float)
            self._lightSensorValues = np.array(sensValues[8:16], dtype=np.float)
            self._groundSensorValues = np.array(sensValues[16:19], dtype=np.float)
            self._accelerometerValues = np.array(sensValues[19:22], dtype=np.float)
            self._wheelEncoderValues = np.array(sensValues[22:], dtype=np.float)
            logging.debug('Remote signal reception for signal EPUCK_allSens ok')
        else:
            logging.error('Remote signal reception for signal EPUCK_allSens failed')


    def senseAllTogether(self):
        """
        convenience method used by super().SensingThread
        :return: nothing
        """
        self.fastSensingOverSignal()


    def fastSensing(self):
        """
        read all non-camera sensors in one script call
        inefficient - use fastSensingOverSignal
        :return:
        """
        res, ret_ints, sensValues, ret_strings, ret_buffer = vrep.simxCallScriptFunction(self.__clientID, self._name,
                                                                                  vrep.sim_scripttype_childscript,
                                                                                  'getAllSensorsForRemote', [], [], [],
                                                                                  self.__empty_buff, vrep.simx_opmode_blocking)
        if res == vrep.simx_return_ok:
            self._proximitySensorValues = np.array(sensValues[:8], dtype=np.float)
            self._lightSensorValues = np.array(sensValues[8:16], dtype=np.float)
            self._groundSensorValues = np.array(sensValues[16:19], dtype=np.float)
            self._accelerometerValues = np.array(sensValues[19:22], dtype=np.float)
            self._wheelEncoderValues = np.array(sensValues[22:], dtype=np.float)
            logging.debug('Remote function call getAllSensorsForRemote ok' )
        else:
            logging.error('Remote function call getAllSensorsForRemote failed')

