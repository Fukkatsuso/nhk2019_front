################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include mbed-src/targets/hal/TARGET_NXP/TARGET_LPC176X/subdir.mk
-include mbed-src/targets/cmsis/TARGET_NXP/TARGET_LPC176X/TOOLCHAIN_GCC_CR/subdir.mk
-include mbed-src/targets/cmsis/TARGET_NXP/TARGET_LPC176X/subdir.mk
-include mbed-src/common/subdir.mk
-include Walk/Protected/subdir.mk
-include Walk/CANs/subdir.mk
-include Walk/subdir.mk
-include QEI/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: FrontLegs_2.0.axf

# Tool invocations
FrontLegs_2.0.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU C++ Linker'
	arm-none-eabi-c++ -nostdlib -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\hal" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\api" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\hal" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\hal\TARGET_NXP" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\hal\TARGET_NXP\TARGET_LPC176X" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\cmsis" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\cmsis\TARGET_NXP" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\cmsis\TARGET_NXP\TARGET_LPC176X" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\cmsis\TARGET_NXP\TARGET_LPC176X\TOOLCHAIN_GCC_CR" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\common" -L"C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\QEI" -Xlinker -Map="FrontLegs_2.0.map" -Xlinker --gc-sections -mcpu=cortex-m3 -mthumb -T "C:\Users\mutsuro\Documents\LPCXpresso_8.2.2_650\workspace\nhk_2019_walk_2v\FrontLegs_2.0\mbed-src\targets\cmsis\TARGET_NXP\TARGET_LPC176X\TOOLCHAIN_GCC_CR\LPC1768.ld" -o "FrontLegs_2.0.axf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(CC_DEPS)$(C++_DEPS)$(EXECUTABLES)$(C_UPPER_DEPS)$(CXX_DEPS)$(OBJS)$(CPP_DEPS)$(C_DEPS) FrontLegs_2.0.axf
	-@echo ' '

post-build:
	-@echo 'Performing post-build steps'
	-arm-none-eabi-size "FrontLegs_2.0.axf"; arm-none-eabi-objcopy -O binary "FrontLegs_2.0.axf" "FrontLegs_2.0.bin" ; # checksum -p LPC1768 -d "FrontLegs_2.0.bin";
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
