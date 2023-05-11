from controller import Robot, Motor, DistanceSensor, PositionSensor #import the robots, motor and sensors to the file
import ikpy #import the IKPy library
from ikpy.chain import Chain #import the chain from the IKPy library
from ikpy.link import OriginLink, URDFLink #import the links from the IKPy library
import tempfile #import the temporary file


pickup  = Robot() #define the robot function

# get the time step of the current world.
timestep = int(pickup.getBasicTimeStep())

# make code to ensure the right urdf file is written to the temporary file
with tempfile.NamedTemporaryFile(suffix = '.urdf',  delete= False) as file:
     filename = file.name
     file.write(pickup.getUrdf().encode('utf-8'))
     

pickupchain = Chain.from_urdf_file(filename, active_links_mask = [False, True, True, True, True, True, False, False,  False])

joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_base_to_jaw_1', 'joint_base_to_jaw_2']

initPos = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

pickup_joints = []
vel = 0.5

for i in range(0,8):
    temp = pickup.getDevice(joint_names[i])
    pickup_joints.append(temp)
    pickup_joints[i].setPosition(initPos[i])
    pickup_joints[i].setVelocity(vel)
    
    
    
# x= 1.98
# y = 0.78
# z = 0.615

# x = 2
# y = 0.53
# z = 0.62

x = 0.02
y = 0.25 + 0.015
z = -0.005 + 0.010



def action():
    iksolution = pickupchain.inverse_kinematics([x,y,z])
    print(iksolution)
    pickup_joints[6].setPosition(0.01)
    pickup_joints[7].setPosition(0.01)
    pickup_joints[5].setPosition(0.00)
    for i in range(0,5):
        pickup_joints[i].setPosition(iksolution[i+1])

def actionpick():
    pickup_joints[6].setPosition(0.003)
    pickup_joints[7].setPosition(0.003)
    
    
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


    

while pickup.step(timestep) != -1:
    
    
    
       t = pickup.getTime()
       if t > 10 and t < 12.5:
          action()
       elif t >= 12.5 and t < 15:
          actionpick() 
       elif t > 15 and t < 17.5:
            destination(0, 0.335, 0.1, 0.003)
       elif t > 17.5 and t < 22.5:
            finaldestination()
       elif t > 22.5 and t < 25:
            release()
       

pass


