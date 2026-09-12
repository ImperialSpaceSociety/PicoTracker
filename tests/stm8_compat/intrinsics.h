/* Compile-only compatibility shim for IAR intrinsic names. */
#ifndef INTRINSICS_STUB_H
#define INTRINSICS_STUB_H
#define __interrupt
#define __disable_interrupt() ((void)0)
#define __enable_interrupt() ((void)0)
#define __halt() ((void)0)
#endif
