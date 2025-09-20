# echo_locate
A sound-based trilateratation system using a distributed array of microphones. The trilateration method used is Time Distance of Arrival (TDoA), measuring the delays between identical peaks in microphone samples, then calculating position using nonlinear least squares.

## /firmware

### firmware/echo_locate
    #### Core
        - Contains the source (*Core/Src/main.c*) and include folders/files
    #### Debug
        - Contains binaries and linker files
    #### Drivers
        - Contains relevant CMSIS_DSP signal processing libraries

### firmware/scripts
    - Python scripts for printing and graphing received event locations. Run using command *python plotter.py*
## /hardware
    - Contains active bandpass-filter schematics and (future) PCB designs
## /misc
    - Contains block diagram of core signal processing algorithm

