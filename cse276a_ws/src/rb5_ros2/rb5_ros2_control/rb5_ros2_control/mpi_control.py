"""
This file include code that control the robot motors
+,+,+,+ left
-,+,+,- straight
-,+,-,+ anticlockwise rotation
"""


from megapi import MegaPi

# You need to tune these numbers, if needed, to find the correct port for each wheel
# The range should be integers from 0 to 14
MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.verbose = verbose
        if verbose:
            self.printConfiguration()
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left   

    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:" + repr(self.port))
        print("Motor ports: MFR: " + repr(MFR) +
              " MBL: " + repr(MBL) + 
              " MBR: " + repr(MBR) + 
              " MFL: " + repr(MFL))


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl: " + repr(int(round(vfl,0))) + 
                  " vfr: " + repr(int(round(vfr,0))) +
                  " vbl: " + repr(int(round(vbl,0))) +
                  " vbr: " + repr(int(round(vbr,0))))
        print('Inside sfm')
        self.bot.motorRun(self.mfl,-vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,-vbl)
        self.bot.motorRun(self.mbr,vbr)

    # The actual motor signal need to be tuned as well.
    # The motor signal can be larger than 50, but you may not want to go too large (e.g. 100 or -100)
    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, speed, -speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )
    
    def close(self):
        self.bot.close()
        self.bot.exit()


if __name__ == "__main__":
    import time
    mpi_ctrl = MegaPiController(port='/dev/ttyUSB0', verbose=True)  
    time.sleep(1)
    # mpi_ctrl.carStraight(100)
    # mpi_ctrl.setFourMotors(-100, 100, 100, -100)
    # time.sleep(2)
    # mpi_ctrl.carStop()
    rspeed = 40
    mpi_ctrl.setFourMotors(-rspeed, rspeed, -rspeed, rspeed) # for rotation
    time.sleep(6.5)
    # mpi_ctrl.carSlide(30)
    # time.sleep(1)
    # mpi_ctrl.carRotate(30)
    # time.sleep(1)
    mpi_ctrl.carStop()
    mpi_ctrl.close()
