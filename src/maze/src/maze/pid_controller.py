import rospy
import time as time

class PID:

    def __init__(self, kp, ki, kd, outMin, outMax, iMin, iMax):
        """PID controller class.

        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            outMin: Minimum output value.
            outMax: Maximum output value.
            iMin: Minimum value for the integral term.
            iMax: Maximum value for the integral term.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.outMin = outMin
        self.outMax = outMax
        self.iMin = iMin
        self.iMax = iMax
        self.lastError = 0
        self.sumError = 0
        self.current_time = None
        self.last_time = None

    def resetValues(self):
        """Reset the PID values to 0.
        """
        self.lastError = 0
        self.sumError = 0
        self.current_time = None
        self.last_time = None

    def pidExecute(self, desired, actValue):
        """Execute the PID controller.

        Args:
            desired: Desired value.
            actValue: Actual value.

        Returns:
            output: Angular velocity.
        """
        if self.last_time == None:
            self.last_time = time.time()
            self.del_time = 0.

        else:
            self.current_time = time.time()
            self.del_time = self.current_time - self.last_time
            self.last_time = self.current_time

        try:
            error = desired - actValue
            newErrorSum = self.sumError + (error * self.del_time)

            if((newErrorSum >= self.iMin) and (newErrorSum <= self.iMax)):
                self.sumError = newErrorSum
            if self.del_time == 0.:
                dError = 0.
            else:
                dError = (error - self.lastError) / self.del_time
            output = (self.kp * error) + (self.ki * self.sumError) + (self.kd * dError)

            self.last_error = error

            if(output > self.outMax):
                output = self.outMax
            if(output < self.outMin):
                output = self.outMin

            return output
        except Exception as e:
            rospy.logerr_throttle(10, "PID Error")


