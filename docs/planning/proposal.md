# Final project proposal

- [x] I have reviewed the project guidelines.
- [x] I will be working alone on this project.
- [x] No significant portion of this project will be (or has been) used in other course work.

## Embedded System Description

The project I propose essentially is a launchpad for controls concepts: the hardware has all components necessary for a complete testbed for control algorithms, and the software provides the necessary functionality for collecting data and interfacing with the system as well as implementing control algorithms.

## Hardware Setup

The hardware consits of a master MSP430FR2355, a slave MSP430FR2310, a brushed DC motor, an encoder, a 4x4 keyad, POTs, an LCD, and heat-beat LEDs.

![sys_arch.jpg](image)

## Software overview

### Master
This will run a state machine used to select modes, send data, and communicated with the slave MSP430.

### Slave
This will run the routines to calculate position and velocity, control states, and control outputs, drive the motor, and communicate with the master MSP430.

Discuss, at a high level, a concept of how your code will work. Include a *high-level* flowchart. This is a high-level concept that should concisely communicate the project's concept.

## Testing Procedure

A necesssarily functional system will provide the framework for testing control algorithms with the hardware, data collection, and interface portions complete. An ideally functional system with also implement a model-reference adaptive system (MRAS) controller that allows the motor parameters to also be calculated.

## Prescaler

Desired Prescaler level: 

- [x] 100%
- [ ] 95% 
- [ ] 90% 
- [ ] 85% 
- [ ] 80% 
- [ ] 75% 

### Prescalar requirements 

**Outline how you meet the requirements for your desired prescalar level**

**The inputs to the system will be:**
1.  Keypad
2.  POT
3.  Encoder

**The outputs of the system will be:**
1.  Motor control
2.  Heart beat LEDs
3.  LCD display  
4.  UART to a laptop
5.  Motor parameters *a* and *b*

**The project objective is**

Design a test platform for control algorithms and attempt to implement MRAS adaptive control.

**The new hardware or software modules are:**
1. Send logged data over UART
2. Encoder
3. Motor control
4. MRAS (state space with 8 vars!!)


The Master will be responsible for: menu navigation with keypad | data transfer to computer | adjust controller values | LCD

The Slave will be responsible for: MRAS algorithm | communicating with motor | reading encoder


### Argument for Desired Prescaler

The software implemented would be notably more complex. The use of periferals would be non-trival (transmitting logged data over UART). The overall system (if functional) would be awesome.
