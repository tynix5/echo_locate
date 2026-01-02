# Firmware
- MCU: STM32F401RE

## Overview (Revision 2.0)
In Revision 1.1, the triangulator works ok with some events, but is unreliable and gives wrong estimates other times. This is likely due to a number of reasons, including:
1. Faulty peak detection/logic
2. Noisy cross correlation
3. Sampling using non-simultaneous ADCs

The last point is probably the most important; in previous versions, there was a delay (~2 us) between ADC samples per channel (ex. mic 0 sample --> delay --> mic 1 sample --> delay --> ...). This small skew leads to huge angular errors impacting cross correlation and TDoA. To prevent this, an FPGA with three digital ADC chips (MCP3202) will be used to simultaneously sample the ADCs, transmitting the buffers over SPI to the STM32F401RE. There were a couple of options including buying an STM32 with 3 different ADC units and triggering all using the same timer or buying a dedicated simultaneous sampling ADC (expensive), but I went with this because I have these parts on hand and its good exercise.

GCC-PHAT was selected instead of cross-correlation for its robustness and increased SNR characteristics, and envelope detection was added to reduce spurious events.

### Overview
#### FPGA
1. Simultaneously sample N samples using iCE40HX8K FPGA at fs=40kHz
2. Transfer streams to STM32F401RE over SPI
#### STM32
1. Wait for DMA buffer to fill
2. Bandpass filter [1kHz, 5kHz] microphone stream(FPGA)
3. Envelope detection
4. Detect peaks and extract sample windows
5. Update noise thresholds
6. GCC-PHAT
7. Solve using NLLS/linear variant to estimate sound origin

![alt text](image.png)