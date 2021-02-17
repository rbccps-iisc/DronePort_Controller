import serial
import sys
import glob
import math
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

class DronePortController():
    def __init__(self, settings=dict()):
        # Define default UAV position values
        self._uavPos = [-1, -1, -1]

        # Define boolean values used for flagging errors
        self._updatedPosition = False

        # Define default UAV offset angle value
        self._uavOffsetAngle = 0  # in radians

        # Define UAV velocity in meters per second
        self._uavVelocity = 0.2

        # Define starting hover height for UAV in meters
        self._hoverHeight = 0.5

        # Define the minimum hover height for UAV in meters
        self._minHoverHeight = self._hoverHeight - self._hoverHeight * 0.5

        # Define the maximum hover height for the UAV in meters
        self._maxHoverHeight = 3

        # Define the landing position world coordinates in meters
        self._landingPos = [0, 0, 0]

        # Define the final landing position offset coordinates in meters
        self._landingOffset = self._landingPos

        # Define a number of points the camera will sample to determine UAV position
        self._cameraAccuracy = 15

        # Define a number of points camera will use to determine if UAV is in frame
        self._cameraInFrameAccuracy = 5

        # Define a percentage of points at which the UAV is considered 'in frame'
        self._cameraInFrameThreshold = 0.5

        # Define a value used to determine if a new coordinate transform is necessary
        self._coordTolerance = 0.01

        # Define an integer value that determines the factor of the logarithmic function that determines if the UAV
        # is on target
        self._onTargetFactor = 10

        # Define a floating point value that determines the height offset
        self._onTargetOffset = 2.0

        # Begin definitions of values to allow for pixel to world coordinate conversion
        # Define the focal length of lens in meters, per datasheet
        self._focalLength = 0.0028

        # Define the sensor x-size and y-size in meters, per datasheet
        self._xImage = 0.003984
        self._yImage = 0.002952

        # Define the sensor x-size and y-size in pixels, per datasheet
        self._xSensor = 656
        self._ySensor = 488

        # Define the dimension of active sensors in the x and y direction in pixels, per datasheet
        self._xActive = 640
        self._yActive = 480

        # Define the frame size in the x and y dimension in pixels, per selected camera mode
        self._xRange = 240
        self._yRange = 240

        # Define the offset value in the x dimension
        self._xOff = self._xRange / 2
        self._yOff = self._yRange / 2

        # Begin definitions of values to manage/enable serial connection to the camera
        # Define the initial string values expected from the camera
        self._cameraInitValue = '{904$904}\r\n'

        # Define the string values expected from the camera when the UAV is not in frame
        self._cameraOutOfFrameValue = '{900$900}\r\n'

        # Define limiters used to parse the serial data for coordinate values
        self._serialLimiters = ['{', '$', '}']

        # Create camera serial connection
        self._camera = None
        while (self._getCameraSerialConnection(self._cameraInitValue) == None): {"""Do Nothing"""}
        self._camera = serial.Serial(port=self._getCameraSerialConnection(self._cameraInitValue))

        # Send start string to camera value to begin operations
        self._camera.write(self._cameraStartString.encode())

    def _getUAVPosition(self):
        """
        Function:    _getUAVPosition
        Purpose:     Get the most up-to-date UAV position in rectangular coordinates
        Inputs:      None
        Outputs:     _uavPos - a list floasts that represent the x, y, z position of the UAV
        Description: This function attempts to grab a number of data points, determined by cameraAccuracy value, and convert them to world coordinates.
                     These coordinates are then averaged to reduce positional errors. Once averaged, they are placed into the uavPos data member which
                     is then reported to the calling function. 
        """
        if self._camera == None:
            return

            # Find the default value for the camera, this indicates non-detection
        xValDummy = int(self._cameraOutOfFrameValue[self._cameraOutOfFrameValue.rfind(
            self._serialLimiters[0]) + 1:self._cameraOutOfFrameValue.rfind(self._serialLimiters[1])])
        yValDummy = int(self._cameraOutOfFrameValue[self._cameraOutOfFrameValue.rfind(
            self._serialLimiters[1]) + 1:self._cameraOutOfFrameValue.rfind(self._serialLimiters[2])])

        # Get update height
        self._uavGetHeight()
        xPoints = [0]
        yPoints = [0]

        # Flush serial buffer to insure that most recent data points are grabbed
        self._camera.reset_input_buffer()

        for i in range(0, int(self._cameraAccuracy)):
            # Workaround currently
            # Until the last value in the initial value is found, read characters
            while (self._camera.read(1).decode('ascii') != self._cameraInitValue[
                -1]): {}  # Need to fix this to a limiter

            # Read serial data for largest possible string length from camera
            posString = self._camera.read(len(self._cameraInitValue)).decode('ascii')
            # Throw away any characters after the final limiter value
            posString = posString[posString.find(self._serialLimiters[0]):posString.find(self._serialLimiters[2]) + 1]

            if len(posString) > 0:
                if posString[0] == self._serialLimiters[0]:
                    # Find the limiter positions to properly split string into x & y components
                    initIndex = posString.rfind(self._serialLimiters[0])
                    splitIndex = posString.rfind(self._serialLimiters[1])
                    lastIndex = posString.rfind(self._serialLimiters[2])

                    # Convert both positions to integer values
                    xPos = int(posString[(initIndex + 1):splitIndex])
                    yPos = int(posString[(splitIndex + 1):lastIndex])

                    # If values are less than dummy values, append to array
                    if (xPos <= xValDummy) and (yPos <= yValDummy):
                        # Convert from pixels to world coordinates with conversion function
                        xPos, yPos = self._pixelConversion(xPos, yPos, self._uavPos[2])
                        # Append to list for potential averaging
                        xPoints.append(xPos)
                        yPoints.append(yPos)
            else:
                # If a proper value was not found, flush the input buffer again
                self._camera.reset_input_buffer()

        # Update the UAV x,y positions with the averages from camera                
        if (len(xPoints) > 0) and (len(yPoints) > 0):
            self._uavPos[0] = math.fsum(xPoints) / len(xPoints)
            self._uavPos[1] = math.fsum(yPoints) / len(yPoints)
            self._updatedPosition = True
        else:
            self._updatedPosition = False

        return self._uavPos

    def _pixelConversion(self, x_pixel, y_pixel, distance):
        """
        Function:    _pixelConversion Purpose:     Convert pixel coordinates into world coordinates Inputs:
        x_pixel - an integer value denoting the x-coordinate of the UAV y_pixel - an integer value denoting the
        y-coordiante of the UAV distance - a floating point value denoting the distance in meters of the UAV from the
        camera Outputs:     a tuple of x,y x - a floating point value denoting the x-coordinate of the UAV in the
        world frame y - a floating point value denoting the y-coordinate of the UAV in the world frame Description:
        This function uses the distance from the camera, focal length, pixel size, and lengths of the sensors to
        convert pixel coordinates to world coordinates.
        """
        k = distance / self._focalLength
        xPixSize = 2 * (self._xImage / self._xSensor)
        yPixSize = 2 * (self._yImage / self._ySensor)  # Temporary fix corresponding to QVGA vs VGA pixel
        x = k * xPixSize * (x_pixel - self._xOff)
        y = k * yPixSize * (y_pixel - self._yOff)
        return x, y

    def _getCameraSerialConnection(self, _cameraInitValue):
        # Create a blank camera port
        cameraPort = None

        # Get all available serial ports
        availablePorts = self._getAllSerialPorts()

        # For all ports returned, create a test connection and look for expected values
        for port in availablePorts:

            test = serial.Serial(port, timeout=0.01)
            timeoutCount = 0
            while test.read(1).decode('ascii') != _cameraInitValue[-1] and timeoutCount <= len(_cameraInitValue) * 4:
                timeoutCount += 1

            if test.read(len(_cameraInitValue)).decode('ascii') == _cameraInitValue:
                # If expected values are found, assign the string value of the port
                cameraPort = port

            test.close()

        # Return the last found serial connection as a string value
        return cameraPort

    def _getAllSerialPorts(self):
        """
        Function: _getAllSerialPorts
        Purpose:  Find all available serial ports on the current machine regardless of operating system
        Inputs:   none
        Outputs:  array of all found ports represented as strings
        Credit:   Thomas, https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python
        Edits:    Joseph Haun
        """
        # Create blank arrays for eventual contents
        availablePorts = []
        possiblePorts = []
        # Depending on Operating System:
        # Dynamically create an array of values of known possible serial port connection names
        if (sys.platform.startswith("win") == True):
            possiblePorts = ['COM%s' % (i + 1) for i in range(256)]
        elif ((sys.platform.startswith("linux") == True) or (sys.platform.startswith("cygwin") == True)):
            possiblePorts = glob.glob('/dev/tty[A-Za-z]*')
        elif (sys.platform.startswith("darwin") == True):
            possiblePorts = glob.glob('/dev/tty.*')
        else:
            # If a known Operating System is not found, raise an exception
            raise EnvironmentError("_getAllSerialPorts: Operating System not supported")

        for port in possiblePorts:
            try:
                # For all possible ports, attempt to create a test connection
                test = serial.Serial(port)
                test.close()
                # If successfully created, append to list
                availablePorts.append(port)
            except(OSError, serial.SerialException):
                # If not successfully created, catch exception and ignore it
                pass

        return availablePorts

    def callbackHeight(self,data):
        self._uavPos[2]=data.pose.pose.position.z


    def _uavGetHeight(self):
        rospy.Subscribe("/mavros/global_position/local",PoseWithCovarianceStamped,self.callbackHeight)
