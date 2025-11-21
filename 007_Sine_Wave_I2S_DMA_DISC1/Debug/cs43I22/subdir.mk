################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../cs43I22/cs43l22.c 

OBJS += \
./cs43I22/cs43l22.o 

C_DEPS += \
./cs43I22/cs43l22.d 


# Each subdirectory must supply rules for building sources it contributes
cs43I22/%.o cs43I22/%.su cs43I22/%.cyclo: ../cs43I22/%.c cs43I22/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../USB_HOST/App -I../USB_HOST/Target -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-cs43I22

clean-cs43I22:
	-$(RM) ./cs43I22/cs43l22.cyclo ./cs43I22/cs43l22.d ./cs43I22/cs43l22.o ./cs43I22/cs43l22.su

.PHONY: clean-cs43I22

