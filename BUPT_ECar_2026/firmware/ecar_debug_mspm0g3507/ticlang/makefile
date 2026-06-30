MSPM0_SDK_INSTALL_DIR ?= D:/BUPT_ECar_2026/mspm0-sdk

ifneq ("$(wildcard $(MSPM0_SDK_INSTALL_DIR)/imports.mak)","")
include $(MSPM0_SDK_INSTALL_DIR)/imports.mak
else ifneq ("$(wildcard $(MSPM0_SDK_INSTALL_DIR)/imports.mak.windows)","")
include $(MSPM0_SDK_INSTALL_DIR)/imports.mak.windows
else
$(error Cannot find imports.mak or imports.mak.windows under $(MSPM0_SDK_INSTALL_DIR))
endif

CC = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"
LNK = "$(TICLANG_ARMCOMPILER)/bin/tiarmclang"

SYSCONFIG_GUI_TOOL = $(dir $(SYSCONFIG_TOOL))sysconfig_gui$(suffix $(SYSCONFIG_TOOL))
SYSCFG_CMD_STUB = $(SYSCONFIG_TOOL) --compiler ticlang --product $(MSPM0_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_GUI_CMD_STUB = $(SYSCONFIG_GUI_TOOL) --compiler ticlang --product $(MSPM0_SDK_INSTALL_DIR)/.metadata/product.json
SYSCFG_OUTPUT_DIR = syscfg
SYSCFG_FILES := $(shell $(SYSCFG_CMD_STUB) --listGeneratedFiles --listReferencedFiles --output $(SYSCFG_OUTPUT_DIR) ../ecar_debug_mspm0g3507.syscfg)

SYSCFG_C_FILES = $(filter %.c,$(SYSCFG_FILES))
SYSCFG_H_FILES = $(filter %.h,$(SYSCFG_FILES))
SYSCFG_OPT_FILES = $(filter %.opt,$(SYSCFG_FILES))

V := @
ifeq ($(VERBOSE), 1)
  V :=
endif

SRC_FILES = \
    ../src/encoder.c \
    ../src/gimbal.c \
    ../src/gimbal_route.c \
    ../src/grayscale.c \
    ../src/imu.c \
    ../src/k230_uart.c \
    ../src/laser.c \
    ../src/line_follow.c \
    ../src/main.c \
    ../src/motor.c \
    ../src/pca9685.c \
    ../src/sound_light.c \
    ../src/timebase.c \
    ../src/ui.c

SRC_HEADERS = $(wildcard ../src/*.h)

OBJECTS = $(patsubst ../src/%.c,%.obj,$(SRC_FILES)) $(patsubst %.c,%.obj,$(notdir $(SYSCFG_C_FILES)))
NAME = ecar_debug_mspm0g3507

CFLAGS += -I.. \
    -I../src \
    -I. \
    -I$(SYSCFG_OUTPUT_DIR) \
    $(addprefix @,$(SYSCFG_OPT_FILES)) \
    -O2 \
    "-I$(MSPM0_SDK_INSTALL_DIR)/source/third_party/CMSIS/Core/Include" \
    "-I$(MSPM0_SDK_INSTALL_DIR)/source" \
    -gdwarf-3 \
    -mcpu=cortex-m0plus \
    -march=thumbv6m \
    -mfloat-abi=soft \
    -mthumb \
    -Wall

LFLAGS += -ldevice.cmd.genlibs \
    "-L$(MSPM0_SDK_INSTALL_DIR)/source" \
    -L.. \
    -L$(SYSCFG_OUTPUT_DIR) \
    $(SYSCFG_OUTPUT_DIR)/device_linker.cmd \
    "-Wl,-m,$(NAME).map" \
    -Wl,--rom_model \
    -Wl,--warn_sections \
    "-L$(TICLANG_ARMCOMPILER)/lib" \
    -llibc.a

all: $(NAME).out

.PHONY: syscfg-gen syscfg-gui clean
$(SYSCFG_FILES): syscfg-gen
	@cmd /C echo generation complete

syscfg-gen: ../ecar_debug_mspm0g3507.syscfg
	@cmd /C echo Generating configuration files...
	$(V) $(SYSCFG_CMD_STUB) --output $(SYSCFG_OUTPUT_DIR) $<

$(SYSCONFIG_GUI_TOOL):
	$(error $(dir $(SYSCONFIG_TOOL)) does not contain the GUI framework \
        necessary to launch the SysConfig GUI. Please set SYSCONFIG_TOOL \
        to a standalone SysConfig installation.)

syscfg-gui: ../ecar_debug_mspm0g3507.syscfg $(SYSCONFIG_GUI_TOOL)
	@cmd /C echo Opening SysConfig GUI
	$(V) $(SYSCFG_GUI_CMD_STUB) $<

define C_RULE
$(basename $(notdir $(1))).obj: $(1) $(SRC_HEADERS) $(SYSCFG_H_FILES)
	@cmd /C echo Building $$@
	$(V) $(CC) $(CFLAGS) -c $$< -o $$@
endef

$(foreach c_file,$(SYSCFG_C_FILES),$(eval $(call C_RULE,$(c_file))))
$(foreach src_file,$(SRC_FILES),$(eval $(call C_RULE,$(src_file))))

$(NAME).out: $(OBJECTS)
	@cmd /C echo linking $@
	$(V) $(LNK) -Wl,-u,_c_int00 $(OBJECTS) $(LFLAGS) -o $(NAME).out

clean:
	@cmd /C echo Cleaning...
	-$(V) $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	-$(V) $(RM) $(NAME).out > $(DEVNULL) 2>&1
	-$(V) $(RM) $(NAME).map > $(DEVNULL) 2>&1
	-$(V) cmd /C "if exist $(SYSCFG_OUTPUT_DIR) rmdir /S /Q $(SYSCFG_OUTPUT_DIR)" > $(DEVNULL) 2>&1
