################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Control_eyeBug/eBugAPI.cpp 

C_SRCS += \
../Control_eyeBug/serial_config.c 

OBJS += \
./Control_eyeBug/eBugAPI.o \
./Control_eyeBug/serial_config.o 

C_DEPS += \
./Control_eyeBug/serial_config.d 

CPP_DEPS += \
./Control_eyeBug/eBugAPI.d 


# Each subdirectory must supply rules for building sources it contributes
Control_eyeBug/%.o: ../Control_eyeBug/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -D__GXX_EXPERIMENTAL_CXX0X__ -I/usr/local/include/opencv -I"/home/al/Als-stuff/fyp/eyebug__Max-start-Als-Mod/include" -I/usr/local/include/cvd -I/usr/local/include/libfreenect -I/usr/include -O3 -std=c++0x -mcpu=cortex-a8  -mfloat-abi=hard -mfpu=neon -funsafe-math-optimizations -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Control_eyeBug/%.o: ../Control_eyeBug/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	arm-linux-gnueabihf-gcc -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


