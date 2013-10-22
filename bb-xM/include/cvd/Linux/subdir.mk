################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CXX_SRCS += \
../include/cvd/Linux/capture_logic.cxx 

OBJS += \
./include/cvd/Linux/capture_logic.o 

CXX_DEPS += \
./include/cvd/Linux/capture_logic.d 


# Each subdirectory must supply rules for building sources it contributes
include/cvd/Linux/%.o: ../include/cvd/Linux/%.cxx
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -D__GXX_EXPERIMENTAL_CXX0X__ -I/usr/local/include/opencv -I"/home/al/Als-stuff/fyp/eyebug__Max-start-Als-Mod/include" -I/usr/local/include/cvd -I/usr/local/include/libfreenect -I/usr/include -O3 -std=c++0x -mcpu=cortex-a8  -mfloat-abi=hard -mfpu=neon -funsafe-math-optimizations -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


