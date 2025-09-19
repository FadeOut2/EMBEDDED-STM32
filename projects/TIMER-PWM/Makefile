# Use '>' instead of TAB for recipes
.RECIPEPREFIX := >

# ================= Project =================
PROJECT := TIMER-PWM
BUILD   := build
TARGET  := $(BUILD)/$(PROJECT)

# ================= Toolchain =================
CC      := arm-none-eabi-gcc
AS      := arm-none-eabi-gcc
LD      := arm-none-eabi-gcc
OBJCOPY := arm-none-eabi-objcopy
OBJDUMP := arm-none-eabi-objdump
SIZE    := arm-none-eabi-size
OPENOCD := openocd

# ================= MCU / Flags =================
# STM32F767 has single-precision FPU (fpv5-sp-d16)
CPUFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard
DEFS     := -DSTM32F767xx
CSTD     := -std=c17
OPT      := -O2 -g3 -ffunction-sections -fdata-sections
WARN     := -Wall -Wextra -Wundef -Wshadow -Wdouble-promotion -Wformat -Wformat-security

INCLUDES := \
  -ICore/Inc \
  -IDrivers/CMSIS/Include \
  -IDrivers/CMSIS/Device/ST/STM32F7xx/Include

# Dependency generation
DEPFLAGS := -MMD -MP

# Linker script (must exist at project root)
LDSCRIPT := STM32F767XX_FLASH.ld
LDFLAGS  := $(CPUFLAGS) -nostartfiles -Wl,--gc-sections -Wl,-Map,$(TARGET).map -T $(LDSCRIPT) --specs=nosys.specs

# ================= Sources =================
SRCS := \
  Core/Src/main.c \
  Core/Src/system_stm32f7xx.c \
  startup_stm32f767xx.s

# Mirror sources under build/
OBJS := $(addprefix $(BUILD)/,$(SRCS:=.o))
DEPS := $(OBJS:.o=.d)

# ================= Default =================
.PHONY: all
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin size

# ================= Compile Rules =================
$(BUILD)/%.c.o: %.c
> mkdir -p $(dir $@)
> $(CC) $(CPUFLAGS) $(OPT) $(WARN) $(CSTD) $(DEFS) $(INCLUDES) $(DEPFLAGS) -c $< -o $@

$(BUILD)/%.s.o: %.s
> mkdir -p $(dir $@)
> $(AS) $(CPUFLAGS) $(DEFS) $(INCLUDES) -x assembler-with-cpp -c $< -o $@

$(BUILD)/%.S.o: %.S
> mkdir -p $(dir $@)
> $(AS) $(CPUFLAGS) $(DEFS) $(INCLUDES) -x assembler-with-cpp -c $< -o $@

# ================= Link =================
$(TARGET).elf: $(OBJS)
> $(LD) $(LDFLAGS) $(OBJS) -o $@

# ================= Artifacts =================
$(TARGET).hex: $(TARGET).elf
> $(OBJCOPY) -O ihex $< $@

$(TARGET).bin: $(TARGET).elf
> $(OBJCOPY) -O binary $< $@

# Optional: disassembly
.PHONY: dump
dump: $(TARGET).elf
> $(OBJDUMP) -d -C -S $< > $(TARGET).lst

# ================= Utils =================
.PHONY: size
size: $(TARGET).elf
> $(SIZE) --format=berkeley $<

.PHONY: clean
clean:
> rm -rf $(BUILD)

# ================= Flash (OpenOCD) =================
.PHONY: flash
flash: $(TARGET).elf
> $(OPENOCD) -f interface/stlink.cfg -f target/stm32f7x.cfg -c "init" -c "halt" -c "program $< verify reset exit"

# Include auto-generated dependencies
-include $(DEPS)
