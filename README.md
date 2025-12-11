# echo_locate
A sound-based trilateratation system using a distributed array of microphones on STM32F401RE. The trilateration method used is Time Difference of Arrival (TDoA), measuring the delays between identical peaks in microphone samples using GCC-PHAT, then calculating position using nonlinear least squares (NLLS).
This version using 3 distributed microphones at the points (0, 0), (0, 1), and (1, 1) in the cartesian plane.

## /firmware

### firmware/echo_locate
- Contains the STM32CubeIDE project source code and included libraries
### firmware/scripts
- Python scripts for printing and graphing received event locations
- MATLAB function to compute NLLS to verify embedded algorithm

## /hardware
- Contains active bandpass-filter schematics and notes on hardware selection
## /misc
- Contains block diagram of core signal processing algorithm (draw.io)

