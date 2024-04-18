################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32g474retx.s 

OBJS += \
./Core/Startup/startup_stm32g474retx.o 

S_DEPS += \
./Core/Startup/startup_stm32g474retx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/%.o: ../Core/Startup/%.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m4 -g3 -DDEBUG -c -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/BasicMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/BayesFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/CommonTables" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/ComplexMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/ControllerFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/DistanceFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/FastMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/FilteringFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/InterpolationFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/MatrixFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/QuaternionMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/StatisticsFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/SupportFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/SVMFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/TransformFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/WindowFunctions" -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-Core-2f-Startup

clean-Core-2f-Startup:
	-$(RM) ./Core/Startup/startup_stm32g474retx.d ./Core/Startup/startup_stm32g474retx.o

.PHONY: clean-Core-2f-Startup

