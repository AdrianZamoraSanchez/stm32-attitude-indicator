# Debug Cheat Sheet (STM32 + OpenOCD + GDB)

## OpenOCD (Windows)

```bash
./openocd.exe -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "gdb port 3333" -c "bindto 0.0.0.0"
```

## GDB (WSL2)

```bash
gdb-multiarch ./projects/XXXXX/XXXXX.elf
set architecture arm
target remote 192.168.1.xxx:3333
```

---

## Basic Debug Commands

### Reset MCU
```bash
monitor reset halt
load
```

### Continue execution
```bash
continue
```

### Step (next line)
```bash
next
```

---

## Breakpoints

```bash
break function_name
break file.cpp:line_number
info breakpoints
delete breakpoints
delete breakpoint X
```

---

## Code navigation

```bash
list
where
```

---

## Memory inspection

```bash
x/1wx 0xXXXXXX
display/x *(volatile unsigned int*)0x40005814
x/1wx 0x40020410
```

---

## Build (ARM toolchain)

```bash
rm -rf build
cmake -B build -S . \
  -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

---

## Notes

- Ensure correct toolchain is used (`arm-none-eabi-gcc/g++`)
- Avoid mixing WSL and Windows compilers (MinGW issues)
- clangd needs to be installed
- CMake needs a arm-none-eabi kit



# USB-UART config for WSL
## Windows
`usbipd list`
-->  3-4    10c4:ea60  Silicon Labs CP210x USB to UART Bridge (COM4)

`usbipd bind --busid 3-4`

`usbipd attach --wsl --busid 3-4`

## WSL
`lsusb`
--> Bus 10c4:ea60  Silicon Labs CP210x USB to UART Bridge (COM4)