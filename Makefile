#/* ===Kisarazu RBKN Library===
# *
# * autor          : Oishi
# * version        : v0.10
# * last update    : 20160612
# *
# * **overview***
# * this is makefile of gcc-arm.
# */
#
#  This makefile is made for stm32f103 using HAL driver based autogenerated at mbed online compiler.
#  based platform is [NUCLEO stm32f103]. 
#  if you rebuild all, you should add '-j' option, it makes more faster.(to enable Parallel compilation)
# 

GCC_BIN = 
PROJECT = Test

#use debugger?
DEBUG = 1
APPDIR = ./App
DRIVERSDIR = ./Drivers
HALDIR = $(DRIVERSDIR)/STM32F1xx_HAL_Driver
CMSISDIR = $(DRIVERSDIR)/CMSIS
CMSIS_DEVICEDIR = $(CMSISDIR)/Device/ST/STM32F1xx
MIDDLE_DRIVERDIR = $(DRIVERSDIR)/middleLayers
DEV_DRIVERDIR = $(DRIVERSDIR)/DevDriver
SYSTEM_TASKS_MANAGERDIR = $(DRIVERSDIR)/SystemTasksManager

OBJDIR=./obj

USR_SRCDIR = $(APPDIR)/Src
HAL_SRCDIR = $(HALDIR)/Src
MIDDLE_SRCDIRS = $(wildcard $(MIDDLE_DRIVERDIR)/*/Src)
DEV_SRCDIRS =  $(wildcard $(DEV_DRIVERDIR)/*/Src)
SYSTEM_TASKS_SRCDIR = $(SYSTEM_TASKS_MANAGERDIR)/Src
STARTUP_SRC = $(DRIVERSDIR)/startup_stm32f103xb.s
STARTUP_OBJ = startup.o

BINDIR = ./bin

SRCS   = 	\
		$(wildcard $(USR_SRCDIR)/*.c)	\
		$(wildcard $(HAL_SRCDIR)/*.c)	\
		$(foreach dir,$(wildcard $(MIDDLE_SRCDIRS)),$(wildcard $(dir)/*.c))	\
		$(foreach dir,$(wildcard $(DEV_SRCDIRS)),$(wildcard $(dir)/*.c))	\
		$(wildcard $(SYSTEM_TASKS_SRCDIR)/*.c)


SYS_OBJECTS = 	$(addprefix $(OBJDIR)/, $(notdir $(SRCS:.c=.o)))\
		$(OBJDIR)/$(STARTUP_OBJ)

INCLUDEDIRS =   -I $(shell cd $(APPDIR)/Inc &&  pwd)							\
	 	-I $(shell cd  $(HALDIR)/Inc&& pwd)						\
		-I $(shell cd  $(CMSISDIR)/Inc&& pwd)						\
		-I $(shell cd  $(CMSIS_DEVICEDIR)/Inc&& pwd)					\
		-I $(shell cd  $(SYSTEM_TASKS_MANAGERDIR)/Inc&& pwd)				\
		-I $(shell cd  $(MIDDLE_DRIVERDIR)/Inc && pwd)				\
	$(addprefix -I, \
		$(foreach dir,$(wildcard $(MIDDLE_DRIVERDIR)/*/Inc),$(shell cd $(dir) && pwd))\
	)\
	$(addprefix -I, \
		$(foreach dir,$(wildcard $(DEV_DRIVERDIR)/*/Inc),$(shell cd $(dir) && pwd))\
	)

LINKER_SCRIPT = $(DRIVERSDIR)/STM32F103RBTx_FLASH.ld

CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE    = $(GCC_BIN)arm-none-eabi-size 

CPU = -mcpu=cortex-m3 -mthumb 

CC_FLAGS = $(CPU) -c -fno-common -fmessage-length=0 -Wall -Wextra -fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer -MMD -MP

CC_SYMBOLS = -DTARGET_FF_ARDUINO -DTARGET_NUCLEO_F103RB -DTOOLCHAIN_GCC -DTARGET_FF_MORPHO -DTARGET_LIKE_CORTEX_M3 -DTARGET_CORTEX_M -DTARGET_LIKE_MBED -DTARGET_STM32F1 -D__MBED__=1 -DARM_MATH_CM3 -DMBED_BUILD_TIMESTAMP=1463831794.11 -DTARGET_STM -DTOOLCHAIN_GCC_ARM -D__CORTEX_M3 -DTARGET_M3 -DTARGET_STM32F103RB -D STM32F103xB -std=c99

LD_FLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs -Wl,-Map=$(BINDIR)/$(PROJECT).map,-cref
LD_SYS_LIBS = -lgcc -lm
#-lstdc++ -lsupc++ -lm -lc -lgcc -lnosys

ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -Og -g3
else
  CC_FLAGS += -DNDEBUG -Os -g0
endif

.PHONY: all clean lst size disp openocd include

all: $(BINDIR)/$(PROJECT).bin $(BINDIR)/$(PROJECT).hex size

clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).lst $(OBJDIR) $(BINDIR) -rf

$(OBJDIR)/$(STARTUP_OBJ): $(STARTUP_SRC)
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<

$(OBJDIR)/%.o: $(USR_SRCDIR)/%.c 
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(HAL_SRCDIR)/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(SYSTEM_TASKS_SRCDIR)/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 1,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 2,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 3,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 4,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 5,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 6,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 7,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 8,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 9,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 10,$(MIDDLE_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 1,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 2,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 3,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 4,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 5,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 6,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 7,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 8,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 9,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(OBJDIR)/%.o: $(word 10,$(DEV_SRCDIRS))/%.c
	-mkdir -p $(OBJDIR)
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDEDIRS) -o $@ $<

$(BINDIR)/$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
	-mkdir -p $(BINDIR)
	$(LD) $(LD_FLAGS) -T $(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ -Wl,--start-group $(LIBRARIES) $(LD_SYS_LIBS) -Wl,--end-group

$(BINDIR)/$(PROJECT).bin: $(BINDIR)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(BINDIR)/$(PROJECT).hex: $(BINDIR)/$(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(BINDIR)/$(PROJECT).lst: $(BINDIR)/$(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size: $(BINDIR)/$(PROJECT).elf
	$(SIZE) $(BINDIR)/$(PROJECT).elf

include:
	echo $(INCLUDEDIRS)

OPENOCDPATH=/home/evaota/app/openocd0.10.0-201601101000-dev/bin
OPENOCDCONFIGDIR=./Drivers/openocd_configulation

openocd: $(BINDIR)/$(PROJECT).elf
	$(OPENOCDPATH)/openocd -f $(OPENOCDCONFIGDIR)/stlink-v2-1.cfg -f $(OPENOCDCONFIGDIR)/stm32f1x_flash.cfg


DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)

export