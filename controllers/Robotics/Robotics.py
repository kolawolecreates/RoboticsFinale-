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
pickupchain = Chain.from_urdf_file(filename, active_links_mask = [False, True, True, True, True, True, True, True, False])

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_base_to_jaw_1', 'joint_base_to_jaw_2']

initPos = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

pickup_joints = []
vel = 0.5

for i in range(0,8):
    temp = pickup.getDevice(joint_names[i])
    pickup_joints.append(temp)
    pickup_joints[i].setPosition(initPos[i])
    pickup_joints[i].setVelocity(vel)
    
    
    
# x= -1.41
# y = 0.28
# z = 0

# x = -1.57
# y = 1.12
# z = 0.13

x = -0.16
y = 0.84
z = 0.13

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while pickup.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
       iksolution = pickupchain.inverse_kinematics([x,y,z])
       print(iksolution)
       for i in range(0,1):
           pickup_joints[i].setPosition(iksolution[i+1])
       
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
pass

# Enter here exit cleanup code.
