#!/usr/bin/env python
"""
Delta Robot Kinematics code, liberally borrowed from tutorial:
http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
License: "You can freely use this code in your applications."

Note that the above tutorial defines the lengths f and e differently to me.  The
tutorial uses the side length of the equilateral triangle which has either the
servo output (for f) or the parallel link anchor (for e) in the middle of the side.

I use the convention that f or e is the displacement from the servo output/parallel
link anchor to the centre of the triangle, which is easier to measure.
"""

import math as maths
'''from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt'''
import numpy as np

class DeltaPositionError(Exception):
    pass

class SimulatedDeltaBot(object):

    def __init__(self, servo_link_length, parallel_link_length, servo_displacement, effector_displacement):
        self.e = effector_displacement
        self.f = servo_displacement
        self.re = parallel_link_length
        self.rf = servo_link_length

    def forward(self, theta1, theta2, theta3):
        """ 
        Takes three servo angles in degrees.  Zero is horizontal.
        return (x,y,z) if point valid, None if not 
        """
        t = self.f-self.e

        theta1, theta2, theta3 = maths.radians(theta1), maths.radians(theta2), maths.radians(theta3)

        # Calculate position of leg1's joint.  x1 is implicitly zero - along the axis
        y1 = -(t + self.rf*maths.cos(theta1))
        z1 = -self.rf*maths.sin(theta1)

        # Calculate leg2's joint position
        y2 = (t + self.rf*maths.cos(theta2))*maths.sin(maths.pi/6)
        x2 = y2*maths.tan(maths.pi/3)
        z2 = -self.rf*maths.sin(theta2)

        # Calculate leg3's joint position
        y3 = (t + self.rf*maths.cos(theta3))*maths.sin(maths.pi/6)
        x3 = -y3*maths.tan(maths.pi/3)
        z3 = -self.rf*maths.sin(theta3)

        # From the three positions in space, determine if there is a valid
        # location for the effector
        dnm = (y2-y1)*x3-(y3-y1)*x2
    
        w1 = y1*y1 + z1*z1
        w2 = x2*x2 + y2*y2 + z2*z2
        w3 = x3*x3 + y3*y3 + z3*z3

        # x = (a1*z + b1)/dnm
        a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
        b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0

        # y = (a2*z + b2)/dnm;
        a2 = -(z2-z1)*x3+(z3-z1)*x2
        b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0

        # a*z^2 + b*z + c = 0
        a = a1*a1 + a2*a2 + dnm*dnm
        b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
        c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - self.re*self.re)
 
        # discriminant
        d = b*b - 4.0*a*c
        print(d)
        if d < 0:
            return None # non-existing point

        z0 = -0.5*(b+maths.sqrt(d))/a
        x0 = (a1*z0 + b1)/dnm
        y0 = (a2*z0 + b2)/dnm
        return (x0,y0,z0)


    def _calcAngleYZ(self, x0, y0, z0):
        y1 = -self.f
        y0 -= self.e
        a = (x0*x0 + y0*y0 + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1)/(2*z0)
        b = (y1-y0)/z0
        d = -(a + b*y1)*(a + b*y1) + self.rf*(b*b*self.rf + self.rf)
        print(y1,y0,b,d)
        if d < 0:
            raise DeltaPositionError()
        yj = (y1 - a*b - maths.sqrt(d))/(b*b + 1)
        zj = a + b*yj
        theta = 180.0*maths.atan(-zj/(y1-yj))/maths.pi
        if yj>y1:
            theta += 180.0
        return theta


    def reverse(self, x0, y0, z0):
        """
        Takes position and returns three servo angles, or 0,0,0 if not possible
        return (x,y,z) if point valid, None if not
        """
        cos120 = maths.cos(2.0*maths.pi/3.0)
        sin120 = maths.sin(2.0*maths.pi/3.0)

        try:
            
            theta1 = self._calcAngleYZ(x0, y0, z0)
            theta2 = self._calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120 - x0*sin120, z0) # rotate +120 deg
            theta3 = self._calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120 + x0*sin120, z0) # rotate -120 deg

            return theta1, theta2, theta3
        except DeltaPositionError:
            return 0,0,0


'''if __name__ == '__main__':

    bot = SimulatedDeltaBot(servo_link_length = 85.0, parallel_link_length = 210.0,
                            servo_displacement = 72, effector_displacement = 20)

    step=10
    minServo = -10
    maxServo = 50

    points = []
    for t1 in range(minServo, maxServo, step):
        for t2 in range(minServo, maxServo, step):
            for t3 in range(minServo, maxServo, step):
                servos = (t1, t2, t3)
                points.append(bot.forward(*servos))
                there_and_back = bot.reverse(*bot.forward(*servos))
                err = map(lambda a,b: abs(a-b), servos, there_and_back)
                if max(err) > 0.0000000000001:
                    print (servos, there_and_back, err)

    fig = plt.figure()
    ax = fig.add_subplot(1,1,1, projection='3d')
    surf = ax.scatter(xs=[x for x,y,z in points] ,ys=[y for x,y,z in points],zs=[z for x,y,z in points])
    plt.show()'''



import RPi.GPIO as GPIO
import sys, time
class A4988Nema(object):
    
    """ Class to control a Nema bi-polar stepper motor with a A4988 also tested with DRV8825"""
    def __init__(self, direction_pin, step_pin, mode_pins, motor_type="A4988"):
        """ class init method 3 inputs
        (1) direction type=int , help=GPIO pin connected to DIR pin of IC
        (2) step_pin type=int , help=GPIO pin connected to STEP of IC
        (3) mode_pins type=tuple of 3 ints, help=GPIO pins connected to
        Microstep Resolution pins MS1-MS3 of IC, can be set to (-1,-1,-1) to turn off
        GPIO resolution.
        (4) motor_type type=string, help=Type of motor two options: A4988 or DRV8825
        """
        self.motor_type = motor_type
        self.direction_pin = direction_pin
        self.step_pin = step_pin

        if mode_pins[0] != -1:
            self.mode_pins = mode_pins
        else:
            self.mode_pins = False

        self.stop_motor = False
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

    def motor_stop(self):
        """ Stop the motor """
        self.stop_motor = True

    def resolution_set(self, steptype):
        """ method to calculate step resolution
        based on motor type and steptype"""
        if self.motor_type == "A4988":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (1, 1, 1)}
        elif self.motor_type == "DRV8825":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (0, 0, 1),
                          '1/32': (1, 0, 1)}
        elif self.motor_type == "LV8729":
            resolution = {'Full': (0, 0, 0),
                          'Half': (1, 0, 0),
                          '1/4': (0, 1, 0),
                          '1/8': (1, 1, 0),
                          '1/16': (0, 0, 1),
                          '1/32': (1, 0, 1),
                          '1/64': (0, 1, 1),
                          '1/128': (1, 1, 1)}
        else:
            print("Error invalid motor_type: {}".format(self.motor_type))
            quit()

        # error check stepmode
        if steptype in resolution:
            pass
        else:
            print("Error invalid steptype: {}".format(steptype))
            quit()

        if self.mode_pins != False:
            GPIO.output(self.mode_pins, resolution[steptype])

    def motor_go(self, clockwise=False, steptype="Full",
                 steps=200, stepdelay=.005, verbose=False, initdelay=.05):
        """ motor_go,  moves stepper motor based on 6 inputs
         (1) clockwise, type=bool default=False
         help="Turn stepper counterclockwise"
         (2) steptype, type=string , default=Full help= type of drive to
         step motor 5 options
            (Full, Half, 1/4, 1/8, 1/16) 1/32 for DRV8825 only
         (3) steps, type=int, default=200, help=Number of steps sequence's
         to execute. Default is one revolution , 200 in Full mode.
         (4) stepdelay, type=float, default=0.05, help=Time to wait
         (in seconds) between steps.
         (5) verbose, type=bool  type=bool default=False
         help="Write pin actions",
         (6) initdelay, type=float, default=1mS, help= Intial delay after
         GPIO pins initialized but before motor is moved.
        """
        self.stop_motor = False
        # setup GPIO
        GPIO.setup(self.direction_pin, GPIO.OUT)
        GPIO.setup(self.step_pin, GPIO.OUT)
        GPIO.output(self.direction_pin, clockwise)
        if self.mode_pins != False:
            GPIO.setup(self.mode_pins, GPIO.OUT)

        try:
            # dict resolution
            self.resolution_set(steptype)
            time.sleep(initdelay)

            
            if self.stop_motor:
                raise StopMotorInterrupt
            else:
                GPIO.output(self.step_pin, True)
                time.sleep(stepdelay)
                GPIO.output(self.step_pin, False)
                time.sleep(stepdelay)
                if verbose:
                        print("Steps count {}".format(i+1), end="\r", flush=True)

        except KeyboardInterrupt:
            print("User Keyboard Interrupt : RpiMotorLib:")
            exit() 
        except Exception as motor_error:
            print(sys.exc_info()[0])
            print(motor_error)
            print("RpiMotorLib  : Unexpected error:")
        else:
            # print report status
            if verbose:
                print("\nRpiMotorLib, Motor Run finished, Details:.\n")
                print("Motor type = {}".format(self.motor_type))
                print("Clockwise = {}".format(clockwise))
                print("Step Type = {}".format(steptype))
                print("Number of steps = {}".format(steps))
                print("Step Delay = {}".format(stepdelay))
                print("Intial delay = {}".format(initdelay))
                print("Size of turn in degrees = {}"
                      .format(degree_calc(steps, steptype)))
        finally:
            # cleanup
            GPIO.output(self.step_pin, False)
            GPIO.output(self.direction_pin, False)
            if self.mode_pins != False:
                for pin in self.mode_pins:
                    GPIO.output(pin, False)

def Home():
    print('rt')
    speed=0.01
    while (GPIO.input(L1)!=False or GPIO.input(L2)!=False or GPIO.input(L3)!=False):
        #print(GPIO.input(L1),GPIO.input(L2),GPIO.input(L3))
        if GPIO.input(L3)!=False:
            mot1.motor_go(False, steptype="Full",
                     steps=1, stepdelay=speed, verbose=False, initdelay=0)
        if GPIO.input(L2)!=False:
            mot2.motor_go(True, steptype="Full",
                     steps=1, stepdelay=speed, verbose=False, initdelay=0)
        if GPIO.input(L1)!=False:
            mot3.motor_go(False, steptype="Full",
                     steps=1, stepdelay=speed, verbose=False, initdelay=0)

mot1 = A4988Nema(3, 5, (-1,-1,-1), motor_type="A4988")
mot3=A4988Nema(11, 13, (-1,-1,-1), motor_type="A4988")
mot2=A4988Nema(19, 21 , (-1,-1,-1), motor_type="A4988")

L2=26
L1=36
L3=24
GPIO.setup(L1,GPIO.IN)
GPIO.setup(L2,GPIO.IN)
GPIO.setup(L3,GPIO.IN)
#Home()
cur1,cur2,cur3=360,360,360
kine  = SimulatedDeltaBot(10,30,6,4.25)
import math
angl=1
while True:
    #rev= kine.forward(-45,-45,-45)
    #print(rev)
    x,y,z = 15*math.cos(math.radians(angl)),100,15*math.sin(math.radians(angl))
    angl+=0.5
    #x,y,z=eval(input('Enter the point: '))
    re1,re2,re3  = kine.reverse(x,y,z)
    #rev= kine.forward(re1,re2,re3)

    #print(re1,re2,re3)
    #print(cur1,cur2,cur3)

    step=1
    #cur1,cur2,cur3=(eval(input('CurAng ')))
    #re1,re2,re3=(eval(input('ReAng ')))
    f=2
    steps1=f*int((re1-cur1)/step)
    steps2=f*int((re2-cur2)/step)
    steps3=f*int((re3-cur3)/step)
    #print(steps1,steps2,steps3)
    speed=0.01
    count=0
    dir1,dir2,dir3,gdir=False,True,False,0   #UP
    #dir1,dir2,dir3,gdir=True,False,True,1     #Down
    if steps1<0:
        dir1=True
    if steps2<0:
        dir2=False
    if steps3<0:
        dir3=True
    for i in range(max(abs(steps1),abs(steps2),abs(steps3))):
        #print(GPIO.input(L3))
        #exit()
        if int(cur1)!=int(re1):
            mot1.motor_go(dir1, steptype="Full",
                     steps=cur1, stepdelay=speed, verbose=False, initdelay=0)
            if count%f==0:        
                if dir1!=True and GPIO.input(L3)!=False:
                    print('mo1')
                    cur1=cur1+step
                else:
                    cur1=cur1-step
                
        if int(cur2)!=int(re2):
            mot2.motor_go(dir2, steptype="Full",
                     steps=cur2, stepdelay=speed, verbose=False, initdelay=0)
            if count%f==0: 
                if dir2!=False and GPIO.input(L2)!=False:
                    print('mo2')
                    cur2=cur2+step
                else:
                    cur2=cur2-step
                
        if int(cur3)!=int(re3):
            mot3.motor_go(dir3, steptype="Full",
                     steps=cur3, stepdelay=speed, verbose=False, initdelay=0)
            if count%f==0: 
                if dir3!=False and GPIO.input(L1)!=False:
                    print('mo3')
                    cur3=cur3-step
                else:
                    cur3=cur3+step
        if int(cur3)==int(re3) and  int(cur2)==int(re2) and int(cur1)==int(re1):
            break
        count+=1      
        #print('cur1',int(cur1),int(re1))
        #print('cur2',int(cur2),int(re2))
        #print('cur3',int(cur3),int(re3))
        #time.sleep(0.01)
    


    
    
    
 

