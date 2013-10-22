################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../LED_FILTER_2.cpp \
../LED_Filter.cpp \
../Yuv_LED_filter.cpp \
../als_livefreenect.cpp \
../cycles.cpp \
../eyeBug_movement.cpp \
../libfreenect_cv.cpp \
../maxs_func.cpp \
../pitcher.cpp \
../sender.cpp \
../serial_port_access.cpp \
../swarm.cpp 

OBJS += \
./LED_FILTER_2.o \
./LED_Filter.o \
./Yuv_LED_filter.o \
./als_livefreenect.o \
./cycles.o \
./eyeBug_movement.o \
./libfreenect_cv.o \
./maxs_func.o \
./pitcher.o \
./sender.o \
./serial_port_access.o \
./swarm.o 

CPP_DEPS += \
./LED_FILTER_2.d \
./LED_Filter.d \
./Yuv_LED_filter.d \
./als_livefreenect.d \
./cycles.d \
./eyeBug_movement.d \
./libfreenect_cv.d \
./maxs_func.d \
./pitcher.d \
./sender.d \
./serial_port_access.d \
./swarm.d 


# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	arm-linux-gnueabihf-g++ -D__GXX_EXPERIMENTAL_CXX0X__ -I/usr/local/include/opencv -I"/home/al/Als-stuff/fyp/eyebug__Max-start-Als-Mod/include" -I/usr/local/include/cvd -I/usr/local/include/libfreenect -I/usr/include -O3 -std=c++0x -mcpu=cortex-a8  -mfloat-abi=hard -mfpu=neon -funsafe-math-optimizations -Wall -c -fmessage-length=0 -fPIC -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


