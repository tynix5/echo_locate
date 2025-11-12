# echo_locate
A sound-based trilateratation system using a distributed array of microphones. The trilateration method used is Time Difference of Arrival (TDoA), measuring the delays between identical peaks in microphone samples, then calculating position using nonlinear least squares.
This version using 3 distributed microphones at the points (0, 0), (1, 0), and (1, 1) in the cartesian plane.

## /firmware

### firmware/echo_locate
- Contains the STM32CubeIDE project source code and included libraries
### firmware/scripts
- Python scripts for printing and graphing received event locations. Run using command *python plotter.py*

## /hardware
- Contains active bandpass-filter schematics and (future) PCB designs
## /misc
- Contains block diagram of core signal processing algorithm

