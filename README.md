# Self Balancing Robot
This is a personal project of building a self balancing robot like an inverted pendelum.   

##Hardware
-Microcontroller: Arudino UNO
-Motor Controller: L298N H-bridge module
-Motor: DC-motor with endcoder (Encoder not used for this project)
-Power: 3-cell LIPO



##Software
Arduino code is used to program a proportional derivative (PD) controller such that the balancing robot stands upright. It uses an inertial measurement unit (IMU) to sense its orientation and feeds the error value to the PD-controller. The PD-controller corrects the error by sending signals to the DC-motors.   
