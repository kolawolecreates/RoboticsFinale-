

from controller import Robot #import the robot functions



roller = Robot() #define the robots



timestep = int(roller.getBasicTimeStep())


slide = roller.getDevice('belt_motor') #create a function for the motor



slide.setVelocity(0.1) #set velocity for the conveyor  belt
slide.setPosition(0.5) #set position on the conveyor belt
    


while roller.step(timestep) != -1:
 
 
    pass


