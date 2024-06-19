################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F1xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := FRTOS-1-LED-Cola-Switch
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX := 
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME).$(BUILD_ARTIFACT_EXTENSION)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
FRTOS-1-LED-Cola-Switch.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
FRTOS-1-LED-Cola-Switch.list \

OBJCOPY_BIN += \
FRTOS-1-LED-Cola-Switch.bin \


# All Target
all: main-build

# Main-build Target
main-build: FRTOS-1-LED-Cola-Switch.elf secondary-outputs

# Tool invocations
FRTOS-1-LED-Cola-Switch.elf: $(OBJS) $(USER_OBJS) C:\Users\Marcos\Documents\STM32\ -\ WorkSpace\FRTOS-1-LED-Cola-Switch\STM32F103C8TX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "FRTOS-1-LED-Cola-Switch.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"C:\Users\Marcos\Documents\STM32 - WorkSpace\FRTOS-1-LED-Cola-Switch\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="FRTOS-1-LED-Cola-Switch.map" -Wl,--gc-sections -static -Wl,--start-group -lfreertos -Wl,--end-group -L"C:\Users\Marcos\Documents\STM32 - WorkSpace\freertos\Debug" --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

FRTOS-1-LED-Cola-Switch.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "FRTOS-1-LED-Cola-Switch.list"
	@echo 'Finished building: $@'
	@echo ' '

FRTOS-1-LED-Cola-Switch.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "FRTOS-1-LED-Cola-Switch.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents fail-specified-linker-script-missing warn-no-linker-script-specified
.SECONDARY:

-include ../makefile.targets
