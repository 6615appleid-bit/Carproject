/* Drop-in definitions of the two CMSIS V1.30 core functions this firmware calls.
 *
 * Libraries/CMSIS/core_cm3.c cannot be compiled with a recent GNU toolchain:
 * its GNU branch emits inline assembly that the assembler rejects, for example
 *   Error: registers may not be the same -- `strexb r0,r0,[r1]'
 * Only __get_PRIMASK / __set_PRIMASK are used by the firmware (Hardware/track.c
 * and Hardware/Ultrasound.c save and restore the interrupt mask), and
 * Libraries/CMSIS/core_cm3.h declares exactly those as ordinary functions for
 * GCC, so the equivalents are provided here.
 *
 * This file exists for the PlatformIO target only; the Keil MDK project still
 * compiles the original core_cm3.c and does not reference this file.
 */
#include <stdint.h>

uint32_t __get_PRIMASK(void)
{
    uint32_t result;
    __asm volatile ("mrs %0, primask" : "=r" (result));
    return result;
}

void __set_PRIMASK(uint32_t priMask)
{
    __asm volatile ("msr primask, %0" : : "r" (priMask) : "memory");
}
