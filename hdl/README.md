# Overview
- FPGA: Alchitry-Cu (iCE40HX8K)
- ADC: MCP3202 (12-bit ADC)
- MCU: STM32F401RE

## Hardware
Using three 12-bit MCP3202 digital ADCs capable of up to 100ksps, interfacing with SPI. All MCP3202 SCLK and MOSI lines are connected to ensure identical sample times. The ADC SPI controller will clock out 16 bits on shared SCLK and MOSI lines, then receive MISO data on 3 unique channels. After simultaneous samples, the FPGA will send 48 bits (16 bits/sample * 1 sample/ADC * 3 ADCs) over SPI to STM32. The STM32 acts as a slave controller in this case. In order to ensure timing constraints are met, the approximate maximum SPI frequency (1MHz) for MCP3202 at 3.3V is used, and the STM32 SPI controller sends data out at 5MHz. The STM32 SPI controller starts transmitting data after first exchange, and must be finished before the end of the next exchange.

## Pinouts