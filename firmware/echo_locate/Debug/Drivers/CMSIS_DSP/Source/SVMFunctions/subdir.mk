################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.c \
../Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.c 

OBJS += \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.o \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.o 

C_DEPS += \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.d \
./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS_DSP/Source/SVMFunctions/%.o Drivers/CMSIS_DSP/Source/SVMFunctions/%.su Drivers/CMSIS_DSP/Source/SVMFunctions/%.cyclo: ../Drivers/CMSIS_DSP/Source/SVMFunctions/%.c Drivers/CMSIS_DSP/Source/SVMFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -D__DSP_PRESENT=1 -D__FPU_PRESENT=1 -DARM_MATH_CM4 -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/tynix/Development/stm32/lib/uart2/Inc" -I../Drivers/CMSIS_DSP/Include -I../Drivers/CMSIS_DSP/PrivateInclude -Ofast -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-SVMFunctions

clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-SVMFunctions:
	-$(RM) ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_init_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_linear_predict_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_init_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_polynomial_predict_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_init_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_rbf_predict_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_init_f32.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f16.su ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.cyclo ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.d ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.o ./Drivers/CMSIS_DSP/Source/SVMFunctions/arm_svm_sigmoid_predict_f32.su

.PHONY: clean-Drivers-2f-CMSIS_DSP-2f-Source-2f-SVMFunctions

