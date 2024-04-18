################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.c \
../Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.c 

OBJS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.o \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.o 

C_DEPS += \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.d \
./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.o Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.su Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.cyclo: ../Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/%.c Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/ARM_CMSIS/CMSIS/Core/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/PrivateInclude/ -I../Middlewares/Third_Party/ARM_CMSIS/Include/ -I../Middlewares/Third_Party/ARM_CMSIS/Include -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/BasicMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/BayesFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/CommonTables" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/ComplexMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/ControllerFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/DistanceFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/FastMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/FilteringFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/InterpolationFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/MatrixFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/QuaternionMathFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/StatisticsFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/SupportFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/SVMFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/TransformFunctions" -I"C:/Users/porpo/Downloads/Lab2_Microcon-main/Lab2_Microcon-main/Lab2_Microcon/Source/WindowFunctions" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-StatisticsFunctions

clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-StatisticsFunctions:
	-$(RM) ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.d ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.o ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctions.su ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.cyclo ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.d ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.o ./Middlewares/Third_Party/ARM_CMSIS/Source/StatisticsFunctions/StatisticsFunctionsF16.su

.PHONY: clean-Middlewares-2f-Third_Party-2f-ARM_CMSIS-2f-Source-2f-StatisticsFunctions

