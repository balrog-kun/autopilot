CROSS=arm-linux-gnueabi-
CPU=LPC1343
CC=$(CROSS)gcc
LD=$(CROSS)ld
OBJCOPY=$(CROSS)objcopy
OBJDUMP=$(CROSS)objdump
AR=$(CROSS)ar
LDSCRIPT=linker/$(CPU).ld

TARGET=tri

OPTIM=-O3

FIRMWARE:=/media/CRP\ DISABLD/firmware.bin
ifeq ("$(wildcard $(FIRMWARE))","")
	FIRMWARE:="/var/run/media/$(USER)/CRP DISABLD/firmware.bin"
endif

# determine program version
PROGRAM_VERSION:=$(shell git describe --tags --abbrev=4 --dirty 2>/dev/null | sed s/^v//)
ifeq ("$(PROGRAM_VERSION)","")
    PROGRAM_VERSION:='unknown'
endif

CFLAGS= \
-D F_CPU=12000000 \
-D __$(CPU)__ \
-D __$(ARCH)xx__ \
-D PROGRAM_VERSION=\"$(PROGRAM_VERSION)\" \
-D PROGRAM_NAME=\"$(TARGET)\" \
-I. \
-I/usr/include/arm-linux-gnueabihf \
-Wall \
-Wextra  \
-Wno-multichar \
-Wstrict-prototypes  \
-Wno-strict-aliasing  \
-D CORTEXM3_GCC  \
-msoft-float \
-mthumb \
-T$(LDSCRIPT) \
$(DEBUG)  \
$(OPTIM) \
-fshort-enums \
-mfix-cortex-m3-ldrd \
-mcpu=cortex-m3 \
-fno-common \
-fomit-frame-pointer \
-foptimize-sibling-calls \
-fno-stack-protector \
-Wdouble-promotion \
$(APP_CFLAGS)

#-ffunction-sections \
#-fdata-sections \

LINKER_FLAGS=$(APP_LDFLAGS) -Xlinker --gc-sections -Xlinker -o$(TARGET).elf -Xlinker -M -Xlinker -Map=$(TARGET).map -msoft-float -mcpu=cortex-m3 -nostartfiles -nodefaultlibs -nostdlib -static -fshort-enums thumb2/libm.a thumb2/libgcc.a thumb2/libc.a

ARM_SRC= \
  tri.c \
  ahrs-ekf-float.c \
  startup/$(CPU).c \
  uart.c \
  timer1.c \
  rx-rmilec.c \
  actuators.c \
  twi.c

#ARM_SRC= rx-test.c startup/$(CPU).c uart.c timer1.c

#  soft-fp/addsf3.c \
#  soft-fp/mulsf3.c \
#  soft-fp/subsf3.c \
#  soft-fp/fixsfsi.c \
#  soft-fp/floatsisf.c \
#  soft-fp/lesf2.c \

#
# Define all object files.
#
ARM_OBJ = $(ARM_SRC:.c=.o)
APP_ADDITIONAL?=

$(TARGET).bin : $(TARGET).elf
	$(OBJCOPY) $(TARGET).elf -O binary $(TARGET).bin

$(TARGET).hex : $(TARGET).elf
	$(OBJCOPY) $(TARGET).elf -O ihex $(TARGET).hex

$(TARGET).elf : $(ARM_OBJ) $(CRT0) $(BOOTLOADER) $(APP_ADDITIONAL) Makefile
	$(CC) $(CFLAGS) $(ARM_OBJ) $(BOOTLOADER) $(APP_ADDITIONAL) -nostartfiles $(CRT0) $(LINKER_FLAGS)
	$(OBJDUMP) -d $(TARGET).elf > $(TARGET).asm

$(ARM_OBJ) : %.o : %.c $(LDSCRIPT) Makefile
	$(CC) -c $(CFLAGS) $< -o $@

tri.c: uart.h timer1.h rx.h actuators-hwpwm.h actuators-serial.h actuators-i2c.h twi.h l3g4200d.h adxl345.h hmc5883l.h bmp085.h ahrs.h config.h config-tri.h config-quad.h adc.h
uart.c: uart.h timer1.h
timer1.c: timer1.h
ahrs-ekf-float.c: uart.h timer1.h twi.h cmps09.h wmp.h l3g4200d.h adxl345.h hmc5883l.h ahrs.h
rx.c: timer1.h
rx-rmilec.c: timer1.h
rx-test.c: uart.h timer1.h

version:
	@echo "$(TARGET) version $(PROGRAM_VERSION)"

upload: $(TARGET).bin
	##mount /dev/sda
	#lpc-flash $(TARGET).bin $(FIRMWARE)
	#lpc-flash $(TARGET).bin /mnt/sda/firmware.bin
	##umount /dev/sda
	##sync
	#umount /media/CRP\ DISABLD
	./lpc-flash $(TARGET).bin firmware.bin
	scp balrog@10.0.2.3:/Volumes/CRP\\\ DISABLD/firmware.bin .oldflash.bin
	# Copy the flash config area (last 512 bytes) in case bootloader
	# erases the entire flash, which is one of the possible settings
	dd if=.oldflash.bin of=firmware.bin bs=512 skip=63 seek=63 count=1
	scp firmware.bin balrog@10.0.2.3:/Volumes/CRP\\\ DISABLD/firmware.bin

clean : app_clean
	find . -name '*.o' -exec rm \{\} \;
	rm -f $(TARGET).bin $(TARGET).elf $(TARGET).map $(TARGET).asm $(TARGET)-$(CPU).bin
