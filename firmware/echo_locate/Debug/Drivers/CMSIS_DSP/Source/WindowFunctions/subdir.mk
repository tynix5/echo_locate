################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.c \
../Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.c 

OBJS += \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.o \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.o 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.d \
./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/WindowFunctions/%.o Drivers/CMSIS_DSP/Source/WindowFunctions/%.su Drivers/CMSIS_DSP/Source/WindowFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/WindowFunctions/%.c Drivers/CMSIS_DSP/Source/WindowFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -D__DSP_PRESENT=1 -D__FPU_PRESENT=1 -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tynix/Development/stm32/lib/uart2/Inc" -I../Drivers/CMSIS_DSP/Include -I../Drivers/CMSIS_DSP/PrivateInclude -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-WindowFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-WindowFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_bartlett_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_blackman_harris_92db_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hamming_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hanning_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft116d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft144d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft169d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft196d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft223d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft248d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft90d_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.d
	-$(RM) ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_hft95_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3a_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall3b_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4a_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4b_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_nuttall4c_f64.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f32.su ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.cyclo ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.d ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.o ./Drivers/CMSIS_DSP/Source/WindowFunctions/arm_welch_f64.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-WindowFunctions

