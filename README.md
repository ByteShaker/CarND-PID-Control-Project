[video1]: ./PID_Sim.mov

# PID Controller
## Controlling a Simulator car with steering PID Controller and Throttle PID Controller

### 1. I implemented a Standard PID Controller in PID.cpp and PID.h
Therefore I followed what was taught in the lessons.

### 2. I builded up the TWIDDLE Algorithm in the main.cpp
To dont interupt the connection to the Simulator I created a workflow inwith the on.message function.

### 3. I started two PID controller:
One for the Steering and one for the Throttle Signal.

For a starting point I twiddled values only for steering and then I started a new Twiddle Session for both PIDs switched on.

I ended up with following values, having the described effects:

p = {0.3479, 0.000730566, 6.63597, 0.001509, -0.000199, 16.07897};

Steering:

    P:  0.3479          Proportional Steering to the middle of the Road
                        
    I:  0.000730566     For the Simulator this value corrects a constant turnrate 
                        as long as the vehicle is in a turn.
                        
    D:  6.63597         The differntial Part has two effects:
                        First it attenuates the oscillation of the car
                        Second it recognizes suddenly starting turns
                        
    
Throttle:

    P:  0.001509        Slows down the car proportional to the distance to the middle of the road 
                        
    I:  -0.000199       This negative value starts speeding up the car, after a constance turn. 
                        It does the correct acceleration even before the end of the turn.
                        
    D:  16.07897        Breaks or slows down the car in the beginning of a turn.
    

Video of two laps on the simulator:
Here's a [link to my video result](./PID_Sim.mov)

    
    
