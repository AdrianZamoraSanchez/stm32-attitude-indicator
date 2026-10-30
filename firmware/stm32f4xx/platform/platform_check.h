// Check for the native word size
static_assert(sizeof(void *) == 4, "Expected 32-bit architecture");

// Check for the correct architecture
#if !defined(__arm__)
static_assert(false, "Not compiling for ARM architecture");
#endif

// Check endianness
static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__,
              "Expected little-endian architecture");

// Check FPU present and active
#ifndef __VFP_FP__
static_assert(false, "FPU not supported");
#endif
