# Researching torque control algorithm in Py before porting to C and integrating with rest of VCU systems

'''
How to accelerate:
APPS (Accel Pedal Position Sensor) sends CAN message to VCU 
VCU calculates desired torque based on APPS value
VCU sends CAN message to inverter to set torque, inverter sends 3 phase AC current to motor
Inverter sends CAN status message back to VCU
Motor spins
Motor resolver (which is basically a really fast analog rotational speed sensor) sends back the angular velocity of the motor to the Inverter
Inverter sends resolver data to VCU via CAN message
Need Resolver-to-Digital Converter to decode analog resolver signals into digital signals (this is already done in the Inverter)
'''

'''
From the  persepective of the torque controller algorithm running on the VCU, the torque controller is a closed loop control system.
'''

'''
Goal: Develop a torque control algorithm that limits motor torque based on current vehicle dynamics, such as:
- Wheel speed
- Vehicle dynamics (IMU)
- Throttle change
- Torque smoothing techniques (i.e. make the slope less steep)
'''
import numpy as np
import matplotlib.pyplot as plot
import dataclasses as dc
 
@dc.dataclass
class APPS:
    position: float = 0.0
    max_position: float = 1.0  # Max position of the accelerator pedal
    min_position: float = 0.0  # Min position of the accelerator pedal
    
@dc.dataclass
class TorqueController:
    desired_torque: float = 0.0
    max_torque: float = 100.0
