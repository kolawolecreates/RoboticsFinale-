

from controller import Robot



roller = Robot()



timestep = int(roller.getBasicTimeStep())


slide = roller.getDevice('belt_motor')



slide.setVelocity(0.1)
slide.setPosition(0.5)
    


while roller.step(timestep) != -1:
 
 
    pass


