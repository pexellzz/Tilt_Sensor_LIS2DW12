# Tilt_Sensor_LIS2DW12

This is an STM32 based project containing the necessary code for a Tilt Sensor.

List of used materials : 

- LIS2DW12 sensor (STEVAL-MKI179V1)
- X-NUCLEO-IKS01A2 
- B-L072Z-LRWAN1 
- STM32CubeIDE / STM32CubeMX
- TeraTerm

The output is printed via UART over a baud rate of 115200 hence the use of Teraterm. The used accelerometer does not feature an e-compass thus only 2 angles can be measured (roll and pitch).

