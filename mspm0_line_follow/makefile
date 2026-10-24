SHELL := cmd.exe

# ---- Toolchain locations (edit if your install path differs) ----
SDK_DIR := C:/ti/mspm0_sdk_2_11_00_07
COMPILER_DIR := C:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS
SYSCONFIG_DIR := C:/ti/sysconfig_1.26.2

PROJECT := line_follow_mspm0g3507
SYSCFG := $(PROJECT).syscfg
BUILD_DIR := Debug/syscfg_build
OUTPUT := Debug/$(PROJECT).out
TI_TXT := Debug/$(PROJECT).txt
# SysConfig 生成（genLinker = true），不需要自定义链接脚本
LINKER_CMD := $(BUILD_DIR)/device_linker.cmd

# cmd.exe 的内置命令（if exist / del / rmdir）只认反斜杠路径，
# 编译规则里的正斜杠路径没问题，但 clean 必须转换，否则报「系统找不到指定的路径」
OUTPUT_WIN    := $(subst /,\,$(OUTPUT))
TI_TXT_WIN    := $(subst /,\,$(TI_TXT))
BUILD_DIR_WIN := $(subst /,\,$(BUILD_DIR))

CC := "$(COMPILER_DIR)/bin/tiarmclang.exe"
HEX := "$(COMPILER_DIR)/bin/tiarmhex.exe"
SYSCONFIG_CLI := "$(SYSCONFIG_DIR)/sysconfig_cli.bat"

# 循迹芯片：驱动（灰度/电机/编码器）+ 循迹控制 + 板级 + 启动文件
SOURCES := \
    main.c \
    startup_mspm0g350x_ticlang.c \
    app/line_task.c \
    bsp/board.c \
    bsp/hal_system_mspm0.c \
    control/line_controller.c \
    control/line_estimator.c \
    drivers/motor_tb6612.c \
    drivers/encoder_speed.c \
    drivers/line_sensor_mux_adc.c \
    services/line_processing.c

HEADERS := $(wildcard app/*.h bsp/*.h config/*.h control/*.h drivers/*.h hal/*.h services/*.h)

OBJECTS := $(foreach source,$(SOURCES),$(BUILD_DIR)/$(notdir $(source:.c=.obj)))
GENERATED_OBJECT := $(BUILD_DIR)/ti_msp_dl_config.obj

CFLAGS := \
    @"$(BUILD_DIR)/device.opt" \
    -march=thumbv6m \
    -mcpu=cortex-m0plus \
    -mfloat-abi=soft \
    -mlittle-endian \
    -mthumb \
    -O2 \
    -g \
    -Wall \
    -Wextra \
    -I. \
    -I"$(BUILD_DIR)" \
    -I"$(SDK_DIR)/source" \
    -I"$(SDK_DIR)/source/third_party/CMSIS/Core/Include"

.PHONY: all clean syscfg

all: $(OUTPUT) $(TI_TXT)

$(BUILD_DIR):
	@if not exist "$(BUILD_DIR)" mkdir "$(BUILD_DIR)"

syscfg: $(BUILD_DIR)/ti_msp_dl_config.c

$(BUILD_DIR)/ti_msp_dl_config.c: $(SYSCFG) | $(BUILD_DIR)
	@echo Generating SysConfig sources
	$(SYSCONFIG_CLI) -s "$(SDK_DIR)/.metadata/product.json" --script "$<" -o "$(BUILD_DIR)" --compiler ticlang

define COMPILE_SOURCE
$(BUILD_DIR)/$(notdir $(1:.c=.obj)): $(1) $(HEADERS) $(BUILD_DIR)/ti_msp_dl_config.c
	@echo Building $(1)
	$(CC) $$(CFLAGS) -c "$(1)" -o "$$@"
endef

$(foreach source,$(SOURCES),$(eval $(call COMPILE_SOURCE,$(source))))

$(GENERATED_OBJECT): $(BUILD_DIR)/ti_msp_dl_config.c
	@echo Building generated SysConfig source
	$(CC) $(CFLAGS) -c "$<" -o "$@"

$(OUTPUT): $(OBJECTS) $(GENERATED_OBJECT) $(LINKER_CMD)
	@echo Linking $(OUTPUT)
	$(CC) @"$(BUILD_DIR)/device.opt" \
	    -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft \
	    -mlittle-endian -mthumb -O2 -g -Wall \
	    "-Wl,-m$(BUILD_DIR)/$(PROJECT).map" \
	    "-Wl,-i$(SDK_DIR)/source" \
	    "-Wl,-i$(BUILD_DIR)" \
	    "-Wl,-i$(COMPILER_DIR)/lib" \
	    "-Wl,--diag_wrap=off" \
	    "-Wl,--display_error_number" \
	    "-Wl,--warn_sections" \
	    "-Wl,--rom_model" \
	    "-Wl,-u,_c_int00" \
	    "-Wl,--stack_size=2048" \
	    -o "$(OUTPUT)" $(OBJECTS) $(GENERATED_OBJECT) \
	    "-Wl,-l$(LINKER_CMD)" \
	    "-Wl,-l$(BUILD_DIR)/device.cmd.genlibs" \
	    "-Wl,-llibc.a"

$(TI_TXT): $(OUTPUT)
	@echo Generating TI-TXT image $(TI_TXT)
	$(HEX) --ti_txt --outfile="$(TI_TXT)" "$(OUTPUT)"

clean:
	@if exist "$(OUTPUT_WIN)" del /q "$(OUTPUT_WIN)"
	@if exist "$(TI_TXT_WIN)" del /q "$(TI_TXT_WIN)"
	@if exist "$(BUILD_DIR_WIN)" rmdir /s /q "$(BUILD_DIR_WIN)"
	@echo clean done
