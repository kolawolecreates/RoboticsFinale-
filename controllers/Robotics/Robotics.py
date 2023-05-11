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
     
#create  the links chain
pickupchain = Chain.from_urdf_file(filename, active_links_mask = [False, True, True, True, True, True, False, False,  False])
#define the joints
joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_base_to_jaw_1', 'joint_base_to_jaw_2']
#specify the initial position of joints
initPos = [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

pickup_joints = []
vel = 0.5
#set the position and velocity for the 8 joints
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

# the cordinate difference between the arm and the cardboard box
x = 0.02
y = 0.25 + 0.015
z = -0.005 + 0.010


#define the move to get the arm to the position of the box and print the inverse kinematics
def action():
    iksolution = pickupchain.inverse_kinematics([x,y,z])
    print(iksolution)
    pickup_joints[6].setPosition(0.01)
    pickup_joints[7].setPosition(0.01)
    pickup_joints[5].setPosition(0.00)
    for i in range(0,5):
        pickup_joints[i].setPosition(iksolution[i+1])

#define the motion to tigethern the arm
def actionpick():
    pickup_joints[6].setPosition(0.003)
    pickup_joints[7].setPosition(0.003)
    
  
  #  define the motion to pick up the box
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
    #define the motion to move the arm above the metal storage box
def finaldestination():
    pickup_joints[0].setPosition(-1.57)
#define the function to release the cardboard box from the arm
def release():
    pickup_joints[6].setPosition(0.0015)
    pickup_joints[7].setPosition(0.0015)
    
def return1():
    pickup_joints[0].setPosition(0)
    
    
def return2():
    pickup_joints[2].setPosition(0)
    pickup_joints[6].setPosition(0)
    pickup_joints[7].setPosition(0)
    pickup_joints[5].setPosition(0)
    pickup_joints[4].setPosition(0)
    pickup_joints[3].setPosition(0)
    pickup_joints[1].setPosition(0)
    

    
#the  main loop
while pickup.step(timestep) != -1:
    
    
    
       t = pickup.getTime()# creates a variable for the time the simulation took
       if t > 8 and t < 11:
          action()  #go for the box after 10s less than 12.5s
       elif t >= 11 and t < 14:
          actionpick() #lock the arm with the box in it within 2.5s
       elif t > 14 and t < 18:
            destination(0, 0.335, 0.1, 0.003) #lift up the arm withn 2.5s
       elif t > 18 and t < 22:
            finaldestination() #turn the arm to the storage box in 5s
       elif t > 22 and t < 25:
            release()   #release the box
       elif t > 25 and t < 27.5:
            return1()  #first part to return back the arm
       elif t > 28 and t < 32:
            return2()   #second part to return back the arm
            
       

pass


