from controller import Robot

robot = Robot()
timestep = 100

m = robot.getMotor("motor")
m.setPosition(float('inf'))
m.setVelocity(0.0)

pSensor = robot.getPositionSensor("ps")
pSensor.enable(timestep)

speed =3
k = 0
t=0
while (robot.step(timestep) != -1):
    m.setVelocity(speed)
    k = pSensor.getValue()
    t += timestep
    print (t)
    if (k > 22) & (speed ==3) & (t>0):
        speed =0
    if (k > 22) & (speed ==0) & (t >10000) :
        speed = -3
    if (k < 21.5) & (speed == -3) & (t >10000):
        speed = 0
    if (k < 21.5) & (speed == -0) & (t >20000):
        speed = 3
        t=0
   
     
            
    pass