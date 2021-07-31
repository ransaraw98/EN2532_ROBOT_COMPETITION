from controller import Robot

robot = Robot()
timestep = 16

m = robot.getDevice("motor2")
m.setPosition(float('inf'))
m.setVelocity(0.0)

pSensor = robot.getPositionSensor("ps2")
pSensor.enable(timestep)

speed =0
k = 0
t=0
while (robot.step(timestep) != -1):
    m.setVelocity(speed)
    k = pSensor.getValue()
    t += timestep
    if (k < 21.1) & (speed ==-0) & (t>3000):
        speed =3
    if (k > 23) & (speed ==3) & (t >3000) :
        speed = 0
    if (k > 23) & (speed == 0) & (t >13000):
        speed = -3
    if (k < 21.1) & (speed ==-3) & (t >13000):
        speed = 0
        t=-7000 
    pass
