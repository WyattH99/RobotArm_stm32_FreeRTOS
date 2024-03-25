#RobotArm_stm32_FreeRTOS

This is a V2 of my RobotArm_stm32 project where I made a 3D Printed Robotic Arm with a STM32 micro-controller, integrating a smaller robot as the primary controller using a Super Loop Control methodology. Using STM32 HAL, I mapped the readings of the Potentiometers using the ADC Peripheral to control
the servos through a Servo Driver with I2C communication.

This version will be functionaly the same but include the following upgrades:
- Emergency Stop Button
- LCD Screen to Display the Output

The Emergency Stop Button has a hard limit on the response time. To achieve this I will be using FreeRTOS tasks with varying priorities.
