################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
LSS += \
rfm12-uart.lss \

FLASH_IMAGE += \
rfm12-uart.hex \

EEPROM_IMAGE += \
rfm12-uart.eep \

SIZEDUMMY += \
sizedummy \

AVRDUDEDUMMY += \
avrdudedummy \


# All Target
all: rfm12-uart.elf secondary-outputs

# Tool invocations
rfm12-uart.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: AVR C++ Linker'
	avr-g++ -Wl,-Map,rfm12-uart.map,--cref -mmcu=attiny2313 -o "rfm12-uart.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

rfm12-uart.lss: rfm12-uart.elf
	@echo 'Invoking: AVR Create Extended Listing'
	-avr-objdump -h -S rfm12-uart.elf  >"rfm12-uart.lss"
	@echo 'Finished building: $@'
	@echo ' '

rfm12-uart.hex: rfm12-uart.elf
	@echo 'Create Flash image (ihex format)'
	-avr-objcopy -R .eeprom -O ihex rfm12-uart.elf  "rfm12-uart.hex"
	@echo 'Finished building: $@'
	@echo ' '

rfm12-uart.eep: rfm12-uart.elf
	@echo 'Create eeprom image (ihex format)'
	-avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex rfm12-uart.elf  "rfm12-uart.eep"
	@echo 'Finished building: $@'
	@echo ' '

sizedummy: rfm12-uart.elf
	@echo 'Invoking: Print Size'
	-avr-size --format=avr --mcu=attiny2313 rfm12-uart.elf
	@echo 'Finished building: $@'
	@echo ' '

avrdudedummy: rfm12-uart.elf
	@echo 'Invoking: AVRDude'
	C:\WinAVR-20100110\bin\avrdude -pt2313 -cusbasp -u -Uflash:w:rfm12-uart.hex:a -Ulfuse:w:0xde:m -Uhfuse:w:0xdf:m -Uefuse:w:0xff:m
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C_DEPS)$(EEPROM_IMAGE)$(ELFS)$(FLASH_IMAGE)$(LSS)$(S_DEPS)$(CXX_DEPS)$(S_UPPER_DEPS)$(C++_DEPS)$(ASM_DEPS)$(CC_DEPS)$(AVRDUDEDUMMY)$(CPP_DEPS)$(C_UPPER_DEPS)$(SIZEDUMMY) rfm12-uart.elf
	-@echo ' '

secondary-outputs: $(LSS) $(FLASH_IMAGE) $(EEPROM_IMAGE) $(SIZEDUMMY) $(AVRDUDEDUMMY)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
