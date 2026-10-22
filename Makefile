######################################
# Target
######################################
TARGET = stm32_foc_test

######################################
# Toolchain
######################################
PREFIX ?= /opt/arm-toolchains/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
CC  = $(PREFIX)gcc
AS  = $(PREFIX)gcc -x assembler-with-cpp
CP  = $(PREFIX)objcopy
SZ  = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

######################################
# Build directory
######################################
BUILD_DIR = build

######################################
# Sources
######################################
C_SOURCES =  \
Core/Src/main.c \
Core/Src/gpio.c \
Core/Src/adc.c \
Core/Src/dma.c \
Core/Src/spi.c \
Core/Src/tim.c \
Core/Src/usart.c \
Core/Src/stm32f1xx_it.c \
Core/Src/stm32f1xx_hal_msp.c \
Core/Src/system_stm32f1xx.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_exti.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_spi.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c \
Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c \
Drivers/motor/foc.c \
Drivers/motor/motor_runtime_param.c \
algorithm/filter.c \
Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_init_f32.c \
Drivers/CMSIS/DSP/Source/ControllerFunctions/arm_pid_reset_f32.c \
Drivers/CMSIS/DSP/Source/CommonTables/arm_common_tables.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_f32.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_f32.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_sin_q15.c \
Drivers/CMSIS/DSP/Source/FastMathFunctions/arm_cos_q15.c

ASM_SOURCES = \
startup_stm32f103xb.s

######################################
# CFLAGS
######################################
CPU = -mcpu=cortex-m3
FPU =
FLOAT-ABI =
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32F103xB \
-DALIGN4=

C_INCLUDES = \
-ICore/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc \
-IDrivers/STM32F1xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
-IDrivers/CMSIS/Include \
-IMiddlewares/ST/ARM/DSP/Inc -IDrivers/CMSIS/DSP/Include \
-IDrivers \
-Ialgorithm \
-I.

OPT = -O2

CFLAGS  = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT)
CFLAGS += -Wall -Wextra -Wno-unused-parameter
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -g -gdwarf-2

ASFLAGS = $(MCU) -Wall -fdata-sections -ffunction-sections

######################################
# LDFLAGS
######################################
LDSCRIPT = STM32F103XB_FLASH.ld

LIBS = -lc -lm -lnosys
LIBDIR =
LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS)
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections

######################################
# Rules
######################################
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:.o=.d)" -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf | $(BUILD_DIR)
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

######################################
# Flash via OpenOCD + CMSIS-DAP (Keil)
######################################
flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/cmsis-dap.cfg -c "cmsis_dap_vid_pid 0xc251 0xf001" \
	  -f target/stm32f1x.cfg -c "adapter speed 100" \
	  -c "program $< verify reset exit"

######################################
# Clean
######################################
clean:
	-rm -rf $(BUILD_DIR)

-include $(wildcard $(BUILD_DIR)/*.d)
