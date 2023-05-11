"""project controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Motor, DistanceSensor, PositionSensor
import ikpy
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import tempfile

# create the Robot instance.
pickup  = Robot()

# get the time step of the current world.
timestep = int(pickup.getBasicTimeStep())

with tempfile.NamedTemporaryFile(suffix = '.urdf',  delete= False) as file:
     filename = file.name
     file.write(pickup.getUrdf().encode('utf-8'))
pickupchain = Chain.from_urdf_file(filename, active_links_mask = [False, True, True, True])

joint_names = ['ArmLowerL', 'ArmLowerR', 'ArmUpperL',  'ArmUpperR', 'ShoulderL', 'ShoulderR']

initPos = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

pickup_joints = []
vel = 0.5

for i in range(0,8):
    temp = pickup.getDevice(joint_names[i])
    pickup_joints.append(temp)
    pickup_joints[i].setPosition(initPos[i])
    pickup_joints[i].setVelocity(vel)
    
    
    
# x= 0.45
# y = 0.29
# z = 0.6

# x = 0.3
# y = 0.13
# z = 0.3

x = -0.15 + 0.010
y = -0.16 + 0.010
z = -0.3 + 0.010

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

def action():
    iksolution = pickupchain.inverse_kinematics([x,y,z])
    print(iksolution)
    pickup_joints[6].setPosition(0.01)
    pickup_joints[7].setPosition(0.01)
    pickup_joints[5].setPosition(0.00)
    for i in range(0,5):
        pickup_joints[i].setPosition(iksolution[i+1])

def actionpick():
    pickup_joints[6].setPosition(0.0015)
    pickup_joints[7].setPosition(0.0015)
    
    
def destination(x,y,z, grip):
    iksolution = pickupchain.inverse_kinematics([x,y,z])
    print(iksolution)
    pickup_joints[6].setPosition(grip)
    pickup_joints[7].setPosition(grip)
    pickup_joints[5].setPosition(0.00)
    pickup_joints[4].setPosition(0.00)
    pickup_joints[3].setPosition(0.00)
    for i in range(0,4):
        pickup_joints[i].setPosition(iksolution[i+1])
    
def finaldestination():
    pickup_joints[0].setPosition(-1.57)

def release():
    pickup_joints[6].setPosition(0.0015)
    pickup_joints[7].setPosition(0.0015)
    
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while pickup.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
       t = pickup.getTime()
       if t > 5 and t < 7.5:
          action()
       elif t >= 7.5 and t < 10:
          actionpick() 
       elif t > 10 and t < 12.5:
            destination(0, 0.32, 0.1, 0.0020)
       elif t > 12.5 and t < 15:
            finaldestination()
       elif t > 15 and t < 17.5:
            release()
       
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
pass

# Enter here exit cleanup code.
