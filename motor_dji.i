# 0 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
# 0 "<built-in>"
# 0 "<command-line>"
# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/autoconf.h" 1
# 0 "<command-line>" 2
# 1 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"





# 1 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 1
# 10 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stdbool.h" 1 3 4
# 11 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 2
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 1 3 4
# 12 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 1 3 4







# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/features.h" 1 3 4
# 28 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/features.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/picolibc.h" 1 3 4





       
# 29 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/features.h" 2 3 4
# 9 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 2 3 4
# 41 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4

# 41 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef signed char __int8_t;

typedef unsigned char __uint8_t;
# 55 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef short int __int16_t;

typedef short unsigned int __uint16_t;
# 77 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef long int __int32_t;

typedef long unsigned int __uint32_t;
# 103 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef long long int __int64_t;

typedef long long unsigned int __uint64_t;
# 134 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef signed char __int_least8_t;

typedef unsigned char __uint_least8_t;
# 160 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef short int __int_least16_t;

typedef short unsigned int __uint_least16_t;
# 182 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef long int __int_least32_t;

typedef long unsigned int __uint_least32_t;
# 200 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef long long int __int_least64_t;

typedef long long unsigned int __uint_least64_t;
# 214 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_default_types.h" 3 4
typedef long long int __intmax_t;







typedef long long unsigned int __uintmax_t;







typedef int __intptr_t;

typedef unsigned int __uintptr_t;
# 13 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 2 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_intsup.h" 1 3 4
# 35 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_intsup.h" 3 4
       
       
       
       
       
       
       
       
# 190 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_intsup.h" 3 4
       
       
       
       
       
       
       
       
# 14 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 2 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_stdint.h" 1 3 4
# 20 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_stdint.h" 3 4
typedef __int8_t int8_t ;



typedef __uint8_t uint8_t ;







typedef __int16_t int16_t ;



typedef __uint16_t uint16_t ;







typedef __int32_t int32_t ;



typedef __uint32_t uint32_t ;







typedef __int64_t int64_t ;



typedef __uint64_t uint64_t ;






typedef __intmax_t intmax_t;




typedef __uintmax_t uintmax_t;




typedef __intptr_t intptr_t;




typedef __uintptr_t uintptr_t;
# 15 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 2 3 4






typedef __int_least8_t int_least8_t;
typedef __uint_least8_t uint_least8_t;




typedef __int_least16_t int_least16_t;
typedef __uint_least16_t uint_least16_t;




typedef __int_least32_t int_least32_t;
typedef __uint_least32_t uint_least32_t;




typedef __int_least64_t int_least64_t;
typedef __uint_least64_t uint_least64_t;
# 51 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 3 4
  typedef int int_fast8_t;
  typedef unsigned int uint_fast8_t;
# 61 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 3 4
  typedef int int_fast16_t;
  typedef unsigned int uint_fast16_t;
# 71 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 3 4
  typedef int int_fast32_t;
  typedef unsigned int uint_fast32_t;
# 81 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdint.h" 3 4
  typedef long long int int_fast64_t;
  typedef long long unsigned int uint_fast64_t;
# 12 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 2
# 1 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_ratios.h" 1
# 38 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_ratios.h"

# 38 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_ratios.h"
const float convert[3][6] = {{0.000022703f, 44047.0422411f, 1.0f,
                              1.0f, 0.043945312f, 22.75277778f},

                             {0.00010986f, 9102.22222f, 1.0f,
                              1.0f, 0.043950678f, 22.75277778f},

                             {0.0000089606f, 111600.0f, 1.0f,
                              1.0f, 0.043950678f, 22.75277778f}};
# 13 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 2
# 1 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_macros.h" 1





# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 1
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h"
# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/devicetree_generated.h" 1
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_multilevel.h" 1
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_multilevel.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/__assert.h" 1
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/__assert.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain.h" 1
# 50 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain/gcc.h" 1
# 98 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain/gcc.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain/common.h" 1
# 99 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain/gcc.h" 2
# 51 "/home/ttwards/zephyrproject/zephyr/include/zephyr/toolchain.h" 2
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/__assert.h" 2
# 35 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/__assert.h"
void __attribute__((format (printf, 1, 2))) assert_print(const char *fmt, ...);
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_multilevel.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_macro.h" 1
# 34 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_macro.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_loops.h" 1
# 1083 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_loops.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_listify.h" 1
# 1084 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_loops.h" 2
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h" 2
# 162 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal_is_eq.h" 1
# 163 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h" 2
# 193 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal_util_inc.h" 1
# 194 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal_util_dec.h" 1
# 197 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal_util_x2.h" 1
# 200 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_internal.h" 2
# 35 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util_macro.h" 2
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_multilevel.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/types.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/types.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 145 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4

# 145 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4
typedef int ptrdiff_t;
# 214 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4
typedef unsigned int size_t;
# 329 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4
typedef unsigned int wchar_t;
# 424 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4
typedef struct {
  long long __max_align_ll __attribute__((__aligned__(__alignof__(long long))));
  long double __max_align_ld __attribute__((__aligned__(__alignof__(long double))));
# 435 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4
} max_align_t;
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/types.h" 2
# 22 "/home/ttwards/zephyrproject/zephyr/include/zephyr/types.h"

# 22 "/home/ttwards/zephyrproject/zephyr/include/zephyr/types.h"
typedef union {
 long long thelonglong;
 long double thelongdouble;
 uintmax_t theuintmax_t;
 size_t thesize_t;
 uintptr_t theuintptr_t;
 void *thepvoid;
 void (*thepfunc)(void);
} z_max_align_t;
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_multilevel.h" 2
# 21 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2





# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 1
# 29 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 2
# 429 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline 
# 429 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 3 4
             _Bool 
# 429 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
                  is_power_of_two(unsigned int x)
{
 return (((x) != 0U) && (((x) & ((x) - 1U)) == 0U));
}
# 453 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline __attribute__((always_inline)) 
# 453 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 3 4
                    _Bool 
# 453 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
                         is_null_no_warn(void *p)
{
 return p == 
# 455 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 3 4
            ((void *)0)
# 455 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
                ;
}
# 465 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline int64_t arithmetic_shift_right(int64_t value, uint8_t shift)
{
 int64_t sign_ext;

 if (shift == 0U) {
  return value;
 }


 sign_ext = (value >> 63) & 1;


 sign_ext = -sign_ext;


 return (value >> shift) | (sign_ext << (64 - shift));
}
# 492 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline void bytecpy(void *dst, const void *src, size_t size)
{
 size_t i;

 for (i = 0; i < size; ++i) {
  ((volatile uint8_t *)dst)[i] = ((volatile const uint8_t *)src)[i];
 }
}
# 511 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline void byteswp(void *a, void *b, size_t size)
{
 uint8_t t;
 uint8_t *aa = (uint8_t *)a;
 uint8_t *bb = (uint8_t *)b;

 for (; size > 0; --size) {
  t = *aa;
  *aa++ = *bb;
  *bb++ = t;
 }
}
# 532 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
int char2hex(char c, uint8_t *x);
# 542 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
int hex2char(uint8_t x, char *c);
# 554 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
size_t bin2hex(const uint8_t *buf, size_t buflen, char *hex, size_t hexlen);
# 566 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
size_t hex2bin(const char *hex, size_t hexlen, uint8_t *buf, size_t buflen);
# 575 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline uint8_t bcd2bin(uint8_t bcd)
{
 return ((10 * (bcd >> 4)) + (bcd & 0x0F));
}
# 587 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline uint8_t bin2bcd(uint8_t bin)
{
 return (((bin / 10) << 4) | (bin % 10));
}
# 605 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
uint8_t u8_to_dec(char *buf, uint8_t buflen, uint8_t value);







static inline int32_t sign_extend(uint32_t value, uint8_t index)
{
 { };

 uint8_t shift = 31 - index;

 return (int32_t)(value << shift) >> shift;
}







static inline int64_t sign_extend_64(uint64_t value, uint8_t index)
{
 { };

 uint8_t shift = 63 - index;

 return (int64_t)(value << shift) >> shift;
}
# 661 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
char *utf8_trunc(char *utf8_str);
# 677 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
char *utf8_lcpy(char *dst, const char *src, size_t n);
# 745 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline void mem_xor_n(uint8_t *dst, const uint8_t *src1, const uint8_t *src2, size_t len)
{
 while (len--) {
  *dst++ = *src1++ ^ *src2++;
 }
}
# 759 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline void mem_xor_32(uint8_t dst[4], const uint8_t src1[4], const uint8_t src2[4])
{
 mem_xor_n(dst, src1, src2, 4U);
}
# 771 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
static inline void mem_xor_128(uint8_t dst[16], const uint8_t src1[16], const uint8_t src2[16])
{
 mem_xor_n(dst, src1, src2, 16);
}
# 783 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/time_units.h" 1
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/time_units.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/time_units.h" 2
# 784 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/util.h" 2
# 27 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 5151 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/io-channels.h" 1
# 5152 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/clocks.h" 1
# 5153 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/gpio.h" 1
# 5154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/spi.h" 1
# 5155 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/dma.h" 1
# 5156 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/pwms.h" 1
# 5157 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/fixed-partitions.h" 1
# 5158 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/ordinals.h" 1
# 5159 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/pinctrl.h" 1
# 5160 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/can.h" 1
# 5161 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/reset.h" 1
# 5162 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree/mbox.h" 1
# 5163 "/home/ttwards/zephyrproject/zephyr/include/zephyr/devicetree.h" 2
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h" 1
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h" 2
# 50 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h"
struct device;
# 59 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h"
union init_function {






 int (*sys)(void);
# 75 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h"
 int (*dev)(const struct device *dev);
# 87 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h"
};
# 103 "/home/ttwards/zephyrproject/zephyr/include/zephyr/init.h"
struct init_entry {

 union init_function init_fn;




 union {
  const struct device *dev;



 };
};
# 14 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/sections.h" 1
# 158 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/sections.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/section_tags.h" 1
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/section_tags.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/sections.h" 1
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/section_tags.h" 2
# 159 "/home/ttwards/zephyrproject/zephyr/include/zephyr/linker/sections.h" 2
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h" 1
# 27 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
enum pm_state {







 PM_STATE_ACTIVE,
# 46 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 PM_STATE_RUNTIME_IDLE,
# 58 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 PM_STATE_SUSPEND_TO_IDLE,
# 70 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 PM_STATE_STANDBY,
# 82 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 PM_STATE_SUSPEND_TO_RAM,
# 95 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 PM_STATE_SUSPEND_TO_DISK,
# 106 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 PM_STATE_SOFT_OFF,

 PM_STATE_COUNT,
};




struct pm_state_info {
 enum pm_state state;
# 142 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 uint8_t substate_id;
# 151 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
 
# 151 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h" 3 4
_Bool 
# 151 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
     pm_device_disabled;







 uint32_t min_residency_us;






 uint32_t exit_latency_us;
};




struct pm_state_constraint {





 enum pm_state state;





 uint8_t substate_id;
};
# 379 "/home/ttwards/zephyrproject/zephyr/include/zephyr/pm/state.h"
static inline uint8_t pm_state_cpu_get_all(uint8_t cpu, const struct pm_state_info **states)
{
 (void)(cpu);
 (void)(states);

 return 0;
}
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/device_mmio.h" 1
# 47 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/device_mmio.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 48 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/device_mmio.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h" 1
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h" 1
# 81 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 82 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h" 2
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 1 3 4
# 16 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/newlib.h" 1 3 4
# 17 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 2 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/config.h" 1 3 4
# 32 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/config.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/ieeefp.h" 1 3 4
# 33 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/config.h" 2 3 4

# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/float.h" 1 3 4
# 35 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/config.h" 2 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/newlib.h" 1 3 4
# 36 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/config.h" 2 3 4
# 18 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 2 3 4

# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/_ansi.h" 1 3 4
# 43 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/_ansi.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/newlib.h" 1 3 4
# 44 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/_ansi.h" 2 3 4
# 20 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 2 3 4


# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 23 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 2 3 4
# 312 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 3 4

# 312 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/inttypes.h" 3 4
typedef struct {
  intmax_t quot;
  intmax_t rem;
} imaxdiv_t;





extern intmax_t imaxabs(intmax_t);
extern imaxdiv_t imaxdiv(intmax_t __numer, intmax_t __denomer);
extern intmax_t strtoimax(const char *__restrict, char **__restrict, int);
extern uintmax_t strtoumax(const char *__restrict, char **__restrict, int);
extern intmax_t wcstoimax(const wchar_t *__restrict, wchar_t **__restrict, int);
extern uintmax_t wcstoumax(const wchar_t *__restrict, wchar_t **__restrict, int);
# 83 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_manage.h" 1
# 38 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_manage.h"
_Bool 
# 38 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_manage.h"
    sys_mm_is_phys_addr_in_range(uintptr_t phys);
# 57 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_manage.h"

# 57 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_manage.h" 3 4
_Bool 
# 57 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_manage.h"
    sys_mm_is_virt_addr_in_range(void *virt);
# 85 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h" 2
# 97 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
static inline uintptr_t k_mem_phys_addr(void *virt)
{
 uintptr_t addr = (uintptr_t)virt;
# 118 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
 { }
# 129 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
                       ;






 return ((addr) - 0);
}
# 150 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
static inline void *k_mem_virt_addr(uintptr_t phys)
{




 { }
# 166 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
                                                            ;






 return (void *)((phys) + 0);
}
# 224 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
void k_mem_map_phys_bare(uint8_t **virt_ptr, uintptr_t phys, size_t size,
    uint32_t flags);
# 254 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
void k_mem_unmap_phys_bare(uint8_t *virt, size_t size);
# 304 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
void *k_mem_map_phys_guard(uintptr_t phys, size_t size, uint32_t flags, 
# 304 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h" 3 4
                                                                       _Bool 
# 304 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
                                                                            is_anon);
# 324 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
void k_mem_unmap_phys_guard(void *addr, size_t size, 
# 324 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h" 3 4
                                                    _Bool 
# 324 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/mm.h"
                                                         is_anon);
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h" 2
# 84 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 85 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h" 2
# 149 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
size_t k_mem_free_get(void);
# 190 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
static inline void *k_mem_map(size_t size, uint32_t flags)
{
 return k_mem_map_phys_guard((uintptr_t)
# 192 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h" 3 4
                                       ((void *)0)
# 192 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
                                           , size, flags, 
# 192 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h" 3 4
                                                          1
# 192 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
                                                              );
}
# 256 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
static inline void k_mem_unmap(void *addr, size_t size)
{
 k_mem_unmap_phys_guard(addr, size, 
# 258 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h" 3 4
                                   1
# 258 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
                                       );
}
# 276 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
int k_mem_update_flags(void *addr, size_t size, uint32_t flags);
# 291 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/mm.h"
size_t k_mem_region_align(uintptr_t *aligned_addr, size_t *aligned_size,
     uintptr_t addr, size_t size, size_t align);
# 49 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/device_mmio.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_io.h" 1
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_io.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 14 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_io.h" 2





typedef uint32_t io_port_t;
typedef uintptr_t mm_reg_t;
typedef uintptr_t mem_addr_t;
# 50 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/device_mmio.h" 2
# 120 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/device_mmio.h"
struct z_device_mmio_rom {

 mm_reg_t addr;
};
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/iterable_sections.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 72 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
typedef int16_t device_handle_t;
# 380 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
struct device_state {







 uint8_t init_res;




 
# 393 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 3 4
_Bool 
# 393 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
     initialized : 1;
};

struct pm_device_base;
struct pm_device;
struct pm_device_isr;
# 412 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
struct device {

 const char *name;

 const void *config;

 const void *api;

 struct device_state *state;

 void *data;
# 448 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
};
# 458 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
static inline device_handle_t device_handle_get(const struct device *dev)
{
 device_handle_t ret = 0;
 extern struct device _device_list_start[];




 if (dev != 
# 466 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 3 4
           ((void *)0)
# 466 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
               ) {
  ret = 1 + (device_handle_t)(dev - _device_list_start);
 }

 return ret;
}
# 481 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
static inline const struct device *
device_from_handle(device_handle_t dev_handle)
{
 extern struct device _device_list_start[];
 const struct device *dev = 
# 485 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 3 4
                           ((void *)0)
# 485 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
                               ;
 size_t numdev;

 do { extern struct device _device_list_start[]; extern struct device _device_list_end[]; *(&numdev) = ((uintptr_t)_device_list_end - (uintptr_t)_device_list_start) / sizeof(struct device); } while (0);;

 if ((dev_handle > 0) && ((size_t)dev_handle <= numdev)) {
  dev = &_device_list_start[dev_handle - 1];
 }

 return dev;
}
# 736 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
static inline const struct device *device_get_binding(const char *name);
# 746 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
size_t z_device_get_all_static(const struct device **devices);
# 764 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
static inline 
# 764 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 3 4
         _Bool 
# 764 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
              device_is_ready(const struct device *dev);
# 780 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
static inline int device_init(const struct device *dev);
# 1196 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h"
extern const struct device __device_dts_ord_0; extern const struct device __device_dts_ord_2; extern const struct device __device_dts_ord_1; extern const struct device __device_dts_ord_3; extern const struct device __device_dts_ord_4; extern const struct device __device_dts_ord_78; extern const struct device __device_dts_ord_87; extern const struct device __device_dts_ord_88; extern const struct device __device_dts_ord_8; extern const struct device __device_dts_ord_60; extern const struct device __device_dts_ord_57; extern const struct device __device_dts_ord_14; extern const struct device __device_dts_ord_32; extern const struct device __device_dts_ord_97; extern const struct device __device_dts_ord_102; extern const struct device __device_dts_ord_89; extern const struct device __device_dts_ord_90; extern const struct device __device_dts_ord_91; extern const struct device __device_dts_ord_92; extern const struct device __device_dts_ord_93; extern const struct device __device_dts_ord_94; extern const struct device __device_dts_ord_15; extern const struct device __device_dts_ord_35; extern const struct device __device_dts_ord_16; extern const struct device __device_dts_ord_36; extern const struct device __device_dts_ord_98; extern const struct device __device_dts_ord_99; extern const struct device __device_dts_ord_100; extern const struct device __device_dts_ord_68; extern const struct device __device_dts_ord_63; extern const struct device __device_dts_ord_72; extern const struct device __device_dts_ord_69; extern const struct device __device_dts_ord_64; extern const struct device __device_dts_ord_73; extern const struct device __device_dts_ord_82; extern const struct device __device_dts_ord_83; extern const struct device __device_dts_ord_86; extern const struct device __device_dts_ord_70; extern const struct device __device_dts_ord_74; extern const struct device __device_dts_ord_101; extern const struct device __device_dts_ord_103; extern const struct device __device_dts_ord_104; extern const struct device __device_dts_ord_84; extern const struct device __device_dts_ord_95; extern const struct device __device_dts_ord_54; extern const struct device __device_dts_ord_71; extern const struct device __device_dts_ord_65; extern const struct device __device_dts_ord_17; extern const struct device __device_dts_ord_37; extern const struct device __device_dts_ord_59; extern const struct device __device_dts_ord_43; extern const struct device __device_dts_ord_46; extern const struct device __device_dts_ord_44; extern const struct device __device_dts_ord_45; extern const struct device __device_dts_ord_12; extern const struct device __device_dts_ord_5; extern const struct device __device_dts_ord_6; extern const struct device __device_dts_ord_41; extern const struct device __device_dts_ord_7; extern const struct device __device_dts_ord_47; extern const struct device __device_dts_ord_81; extern const struct device __device_dts_ord_79; extern const struct device __device_dts_ord_11; extern const struct device __device_dts_ord_33; extern const struct device __device_dts_ord_34; extern const struct device __device_dts_ord_13; extern const struct device __device_dts_ord_18; extern const struct device __device_dts_ord_38; extern const struct device __device_dts_ord_22; extern const struct device __device_dts_ord_19; extern const struct device __device_dts_ord_20; extern const struct device __device_dts_ord_21; extern const struct device __device_dts_ord_23;







# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 1






# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing_syscall.h" 1
# 8 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 2



# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stdarg.h" 1 3 4
# 40 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stdarg.h" 3 4

# 40 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stdarg.h" 3 4
typedef __builtin_va_list __gnuc_va_list;
# 99 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stdarg.h" 3 4
typedef __gnuc_va_list va_list;
# 12 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 2

# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscall_list.h" 1
# 14 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/syscall.h" 1
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/syscall.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/syscall.h" 1
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/syscall.h" 2
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 2
# 86 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"

# 86 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
typedef uintptr_t (*_k_syscall_handler_t)(uintptr_t arg1, uintptr_t arg2,
       uintptr_t arg3, uintptr_t arg4,
       uintptr_t arg5, uintptr_t arg6,
       void *ssf);




static inline __attribute__((always_inline)) 
# 94 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 3 4
                    _Bool 
# 94 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
                         z_syscall_trap(void)
{
 
# 96 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 3 4
_Bool 
# 96 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
     ret = 
# 96 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 3 4
           0
# 96 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
                ;
# 106 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
 return ret;
}







static inline 
# 115 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 3 4
             _Bool 
# 115 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
                  k_is_user_context(void)
{



 return 
# 120 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h" 3 4
       0
# 120 "/home/ttwards/zephyrproject/zephyr/include/zephyr/syscall.h"
            ;

}
# 15 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 2
# 23 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
extern const struct device * z_impl_device_get_binding(const char * name);


static inline const struct device * device_get_binding(const char * name)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 34 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 3 4
0
# 34 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
);
 return z_impl_device_get_binding(name);
}
# 46 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
extern 
# 46 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 3 4
      _Bool 
# 46 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
           z_impl_device_is_ready(const struct device * dev);


static inline 
# 49 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 3 4
             _Bool 
# 49 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
                  device_is_ready(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 57 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 3 4
0
# 57 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
);
 return z_impl_device_is_ready(dev);
}
# 69 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
extern int z_impl_device_init(const struct device * dev);


static inline int device_init(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 80 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 3 4
0
# 80 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
);
 return z_impl_device_init(dev);
}
# 92 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
extern const struct device * z_impl_device_get_by_dt_nodelabel(const char * nodelabel);


static inline const struct device * device_get_by_dt_nodelabel(const char * nodelabel)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 103 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h" 3 4
0
# 103 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/device.h"
);
 return z_impl_device_get_by_dt_nodelabel(nodelabel);
}
# 1205 "/home/ttwards/zephyrproject/zephyr/include/zephyr/device.h" 2
# 7 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_macros.h" 2

# 1 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 1
# 24 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
# 1 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 1
# 9 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 1 3 4
# 38 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/cdefs.h" 1 3 4
# 47 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/cdefs.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 48 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/cdefs.h" 2 3 4
# 39 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 2 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/ieeefp.h" 1 3 4
# 39 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/ieeefp.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/ieeefp.h" 1 3 4
# 40 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/ieeefp.h" 2 3 4




# 126 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/ieeefp.h" 3 4

# 126 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/ieeefp.h" 3 4
typedef int fp_rnd;





fp_rnd fpgetround (void);
fp_rnd fpsetround (fp_rnd);



typedef int fp_except;






fp_except fpgetmask (void);
fp_except fpsetmask (fp_except);
fp_except fpgetsticky (void);
fp_except fpsetsticky (fp_except);



typedef int fp_rdi;



fp_rdi fpgetroundtoi (void);
fp_rdi fpsetroundtoi (fp_rdi);
# 207 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/ieeefp.h" 3 4

# 40 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 2 3 4



# 116 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
extern double atan (double);
extern double cos (double);
extern double sin (double);
extern double tan (double);
extern double tanh (double);
extern double frexp (double, int *);
extern double modf (double, double *);
extern double ceil (double);
extern double fabs (double);
extern double floor (double);

extern double acos (double);
extern double asin (double);
extern double atan2 (double, double);
extern double cosh (double);
extern double sinh (double);
extern double exp (double);
extern double ldexp (double, int);
extern double log (double);
extern double log10 (double);
extern double pow (double, double);
extern double sqrt (double);
extern double fmod (double, double);
# 183 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
    typedef float float_t;
    typedef double double_t;
# 256 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
extern int __isinff (float);
extern int __isinfd (double);
extern int __isnanf (float);
extern int __isnand (double);
extern int __fpclassifyf (float);
extern int __fpclassifyd (double);
extern int __signbitf (float);
extern int __signbitd (double);
extern int __finite (double);
extern int __finitef (float);

extern int __fpclassifyl (long double);
extern int __finitel (long double);
# 326 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
int __iseqsigd(double x, double y);
int __iseqsigf(float x, float y);


int __iseqsigl(long double x, long double y);
# 349 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
int __issignalingf(float f);
int __issignaling(double d);


int __issignalingl(long double d);
# 417 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
extern double infinity (void);
extern double nan (const char *);
extern double copysign (double, double);
extern double logb (double);
extern int ilogb (double);

extern double asinh (double);
extern double cbrt (double);
extern double nextafter (double, double);
extern double rint (double);
extern double scalbn (double, int);
extern double scalb (double, double);
extern double getpayload(const double *x);
extern double significand (double);

extern double exp2 (double);
extern double scalbln (double, long int);
extern double tgamma (double);
extern double nearbyint (double);
extern long int lrint (double);
extern long long int llrint (double);
extern double round (double);
extern long int lround (double);
extern long long int llround (double);
extern double trunc (double);
extern double remquo (double, double, int *);
extern double fdim (double, double);
extern double fmax (double, double);
extern double fmin (double, double);
extern double fma (double, double, double);

extern double log1p (double);
extern double expm1 (double);

extern double acosh (double);
extern double atanh (double);
extern double remainder (double, double);
extern double gamma (double);
extern double lgamma (double);
extern double erf (double);
extern double erfc (double);
extern double log2 (double);




extern double hypot (double, double);



extern float atanf (float);
extern float cosf (float);
extern float sinf (float);
extern float tanf (float);
extern float tanhf (float);
extern float frexpf (float, int *);
extern float modff (float, float *);
extern float ceilf (float);
extern float fabsf (float);
extern float floorf (float);

extern float acosf (float);
extern float asinf (float);
extern float atan2f (float, float);
extern float coshf (float);
extern float sinhf (float);
extern float expf (float);
extern float ldexpf (float, int);
extern float logf (float);
extern float log10f (float);
extern float powf (float, float);
extern float sqrtf (float);
extern float fmodf (float, float);



extern float exp2f (float);
extern float scalblnf (float, long int);
extern float tgammaf (float);
extern float nearbyintf (float);
extern long int lrintf (float);
extern long long int llrintf (float);
extern float roundf (float);
extern long int lroundf (float);
extern long long int llroundf (float);
extern float truncf (float);
extern float remquof (float, float, int *);
extern float fdimf (float, float);
extern float fmaxf (float, float);
extern float fminf (float, float);
extern float fmaf (float, float, float);

extern float infinityf (void);
extern float nanf (const char *);
extern float copysignf (float, float);
extern float logbf (float);
extern int ilogbf (float);

extern float asinhf (float);
extern float cbrtf (float);
extern float nextafterf (float, float);
extern float rintf (float);
extern float scalbnf (float, int);
extern float scalbf (float, float);
extern float log1pf (float);
extern float expm1f (float);
extern float getpayloadf(const float *x);
extern float significandf (float);

extern float acoshf (float);
extern float atanhf (float);
extern float remainderf (float, float);
extern float gammaf (float);
extern float lgammaf (float);
extern float erff (float);
extern float erfcf (float);
extern float log2f (float);
extern float hypotf (float, float);





extern long double hypotl (long double, long double);
extern long double sqrtl (long double);
extern long double frexpl (long double, int *);
extern long double scalbnl (long double, int);
extern long double scalblnl (long double, long);
extern long double rintl (long double);
extern long int lrintl (long double);
extern long long int llrintl (long double);
extern int ilogbl (long double);
extern long double logbl (long double);
extern long double ldexpl (long double, int);
extern long double nearbyintl (long double);
extern long double ceill (long double);
extern long double fmaxl (long double, long double);
extern long double fminl (long double, long double);
extern long double roundl (long double);
extern long lroundl (long double);
extern long long int llroundl (long double);
extern long double truncl (long double);
extern long double nanl (const char *);
extern long double floorl (long double);

extern long double fabsl (long double);
extern long double copysignl (long double, long double);


extern long double atanl (long double);
extern long double cosl (long double);
extern long double sinl (long double);
extern long double tanl (long double);
extern long double tanhl (long double);
extern long double modfl (long double, long double *);
extern long double log1pl (long double);
extern long double expm1l (long double);
extern long double acosl (long double);
extern long double asinl (long double);
extern long double atan2l (long double, long double);
extern long double coshl (long double);
extern long double sinhl (long double);
extern long double expl (long double);
extern long double logl (long double);
extern long double log10l (long double);
extern long double powl (long double, long double);
extern long double fmodl (long double, long double);
extern long double asinhl (long double);
extern long double cbrtl (long double);
extern long double nextafterl (long double, long double);
extern float nexttowardf (float, long double);
extern double nexttoward (double, long double);
extern long double nexttowardl (long double, long double);
extern long double log2l (long double);
extern long double exp2l (long double);
extern long double scalbl (long double, long double);
extern long double tgammal (long double);
extern long double remquol (long double, long double, int *);
extern long double fdiml (long double, long double);
extern long double fmal (long double, long double, long double);
extern long double acoshl (long double);
extern long double atanhl (long double);
extern long double remainderl (long double, long double);
extern long double lgammal (long double);
extern long double gammal (long double);
extern long double erfl (long double);
extern long double erfcl (long double);
extern long double j0l(long double);
extern long double y0l(long double);
extern long double j1l(long double);
extern long double y1l(long double);
extern long double jnl(int, long double);
extern long double ynl(int, long double);

extern long double getpayloadl(const long double *x);
extern long double significandl(long double);
# 719 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/math.h" 1 3 4
# 720 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/math.h" 2 3 4


# 10 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 2


# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 1 3 4
# 49 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 50 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 2 3 4
# 59 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 3 4


void * memchr (const void *, int, size_t);
int memcmp (const void *, const void *, size_t);
void * memcpy (void *restrict, const void *restrict, size_t);
void * memmove (void *, const void *, size_t);
void * memset (void *, int, size_t);
char *strcat (char *restrict, const char *restrict);
char *strchr (const char *, int);
int strcmp (const char *, const char *);
int strcoll (const char *, const char *);
char *strcpy (char *restrict, const char *restrict);
size_t strcspn (const char *, const char *);
char *strerror (int);
size_t strlen (const char *);
char *strncat (char *restrict, const char *restrict, size_t);
int strncmp (const char *, const char *, size_t);
char *strncpy (char *restrict, const char *restrict, size_t);
char *strpbrk (const char *, const char *);
char *strrchr (const char *, int);
size_t strspn (const char *, const char *);
char *strstr (const char *, const char *);

char *strtok (char *restrict, const char *restrict);

size_t strxfrm (char *restrict, const char *restrict, size_t);







char *strtok_r (char *restrict, const char *restrict, char **restrict);
# 143 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 3 4
char * _strerror_r (int, int, int *);






size_t strnlen (const char *, size_t);
# 210 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/string.h" 1 3 4
# 211 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/string.h" 2 3 4


# 13 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 2
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 1 3 4
# 48 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/_ansi.h" 1 3 4
# 49 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 2 3 4

# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/_types.h" 1 3 4
# 51 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 2 3 4





typedef __uint8_t u_int8_t;


typedef __uint16_t u_int16_t;


typedef __uint32_t u_int32_t;


typedef __uint64_t u_int64_t;

typedef __intptr_t register_t;





# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 1 3 4
# 52 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 359 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 3 4
typedef unsigned int wint_t;
# 53 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 2 3 4
# 61 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/newlib.h" 1 3 4
# 62 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 2 3 4




typedef long __blkcnt_t;
typedef __int64_t __blkcnt64_t;



typedef long __blksize_t;



typedef __uint64_t __fsblkcnt_t;



typedef __uint32_t __fsfilcnt_t;



typedef long _off_t;





typedef int __pid_t;






typedef short __dev_t;







typedef unsigned short __uid_t;






typedef unsigned short __gid_t;




typedef __uint32_t __id_t;







typedef unsigned short __ino_t;

typedef __uint64_t __ino64_t;
# 140 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 3 4
typedef __uint32_t __mode_t;





__extension__ typedef long long _off64_t;





typedef _off_t __off_t;
typedef __uint64_t __off64_t;


typedef _off64_t __loff_t;


typedef long __key_t;







typedef long _fpos_t;
# 182 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 3 4
typedef unsigned int __size_t;
# 198 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 3 4
typedef signed int _ssize_t;
# 209 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/_types.h" 3 4
typedef _ssize_t __ssize_t;



typedef struct
{
  int __count;
  union
  {
    wint_t __wch;
    unsigned char __wchb[4];
  } __value;
} _mbstate_t;




typedef void *_iconv_t;






typedef unsigned long __clock_t;






typedef __int_least64_t __time_t;





typedef unsigned long __clockid_t;


typedef long __daddr_t;



typedef unsigned long __timer_t;


typedef __uint8_t __sa_family_t;



typedef __uint32_t __socklen_t;


typedef __int32_t __nl_item;
typedef unsigned short __nlink_t;
typedef long __suseconds_t;
typedef unsigned long __useconds_t;
# 74 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 2 3 4
# 124 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 3 4
typedef __blkcnt_t blkcnt_t;




typedef __blksize_t blksize_t;




typedef unsigned long clock_t;





typedef __int_least64_t time_t;




typedef __daddr_t daddr_t;


typedef char * caddr_t;




typedef __fsblkcnt_t fsblkcnt_t;
typedef __fsfilcnt_t fsfilcnt_t;




typedef __id_t id_t;




typedef __ino_t ino_t;
# 182 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 3 4
typedef __off_t off_t;



typedef __dev_t dev_t;



typedef __uid_t uid_t;



typedef __gid_t gid_t;




typedef __pid_t pid_t;




typedef __key_t key_t;




typedef _ssize_t ssize_t;




typedef __mode_t mode_t;




typedef __nlink_t nlink_t;




typedef __clockid_t clockid_t;





typedef __timer_t timer_t;





typedef __useconds_t useconds_t;




typedef __suseconds_t suseconds_t;



typedef __int64_t sbintime_t;


# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/machine/types.h" 1 3 4
# 249 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/types.h" 2 3 4
# 14 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 2
# 37 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"

# 37 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
struct pid_config {
 float k_p;
 float k_i;
 float k_d;
 
# 41 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
_Bool 
# 41 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
     mit;
};

struct pid_data {
 float *ref;
 float *curr;

 float *detri_ref;
 float *detri_curr;

 float err_integral;
 float err_derivate;
 float ratio;
 struct device *pid_dev;
 int32_t *curr_time;
 int32_t *prev_time;
 float *output;
};

static 
# 60 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
      _Bool 
# 60 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
           float_equal(float a, float b)
{
 return fabsf(a - b) < 0.0001f;
}

static void pid_calc(struct pid_data *data)
{
 const struct device *dev = data->pid_dev;
 if (dev == 
# 68 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
           ((void *)0)
# 68 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
               ) {
  return;
 }
 const struct pid_config *pid_para = dev->config;
 if (!pid_para->mit) {
  if (data->curr == 
# 73 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                   ((void *)0)
# 73 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                       ) {
   return;
  }
  float kp = pid_para->k_p;
  float ki = pid_para->k_i;
  float kd = pid_para->k_d;
  float err = *(data->ref) - *(data->curr);
  float deltaT = (((!0)) ? ( ((1000000) == (168000000)) ? (uint32_t) (*(data->curr_time) - *(data->prev_time)) : ((168000000) > (1000000) && (168000000) % (1000000) == 0U) ? ((uint64_t) (*(data->curr_time) - *(data->prev_time)) <= 0xffffffffU - ((
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) / 2 : (
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) - 1 : 0) ? ((uint32_t)((*(data->curr_time) - *(data->prev_time)) + ((
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) / 2 : (
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) - 1 : 0)) / ((168000000)/(1000000) ? (168000000)/(1000000) : 01u)) : (uint32_t) (((uint64_t) (*(data->curr_time) - *(data->prev_time)) + ((
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) / 2 : (
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) - 1 : 0)) / ((168000000)/(1000000) ? (168000000)/(1000000) : 01u)) ) : ((1000000) > (168000000) && (1000000) % (168000000) == 0U) ? (uint32_t) (*(data->curr_time) - *(data->prev_time))*((1000000) / (168000000)) : ((uint32_t) (((uint64_t) (*(data->curr_time) - *(data->prev_time))*(1000000) + ((
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) / 2 : (
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) - 1 : 0)) / (168000000))) ) : ((uint32_t) (((uint64_t) (*(data->curr_time) - *(data->prev_time))*(1000000) + ((
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) / 2 : (
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 80 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) - 1 : 0)) / (168000000))) );
  if (!float_equal(ki, 0)) {
   data->err_integral += (err * deltaT) / (1000000 * ki);
  }
  if (!float_equal(kd, 0)) {
   data->err_derivate = kd * err / deltaT;
  }


  *(data->output) = kp * (err + data->err_integral + data->err_derivate);

 } else {
  if (data->curr == 
# 92 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                   ((void *)0)
# 92 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                       ) {
   return;
  }
  float kp = pid_para->k_p;
  float ki = pid_para->k_i;
  float kd = pid_para->k_d;
  float err = *(data->ref) - *(data->curr);
  float deltaT = (((!0)) ? ( ((1000000) == (168000000)) ? (uint32_t) (*(data->curr_time) - *(data->prev_time)) : ((168000000) > (1000000) && (168000000) % (1000000) == 0U) ? ((uint64_t) (*(data->curr_time) - *(data->prev_time)) <= 0xffffffffU - ((
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) / 2 : (
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) - 1 : 0) ? ((uint32_t)((*(data->curr_time) - *(data->prev_time)) + ((
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) / 2 : (
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) - 1 : 0)) / ((168000000)/(1000000) ? (168000000)/(1000000) : 01u)) : (uint32_t) (((uint64_t) (*(data->curr_time) - *(data->prev_time)) + ((
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) / 2 : (
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? ((168000000) / (1000000)) - 1 : 0)) / ((168000000)/(1000000) ? (168000000)/(1000000) : 01u)) ) : ((1000000) > (168000000) && (1000000) % (168000000) == 0U) ? (uint32_t) (*(data->curr_time) - *(data->prev_time))*((1000000) / (168000000)) : ((uint32_t) (((uint64_t) (*(data->curr_time) - *(data->prev_time))*(1000000) + ((
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) / 2 : (
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) - 1 : 0)) / (168000000))) ) : ((uint32_t) (((uint64_t) (*(data->curr_time) - *(data->prev_time))*(1000000) + ((
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                1
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) / 2 : (
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
                0
# 99 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) ? (168000000) - 1 : 0)) / (168000000))) );
  if (!float_equal(ki, 0)) {
   data->err_integral += (err * deltaT) / (1000000 * ki);
  }
  if (!float_equal(kd, 0)) {
   data->err_derivate =
    kd * (*(data->detri_ref) - *(data->detri_curr)) / deltaT;
  }

  *(data->output) = kp * (err + data->err_integral + data->err_derivate);
  return;
 }
}

static void pid_reg_input(struct pid_data *data, float *curr, float *ref)
{
 if (data == 
# 115 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
            ((void *)0)
# 115 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) {
  return;
 }
 data->curr = curr;
 data->ref = ref;
}

static void pid_reg_output(struct pid_data *data, float *output)
{
 if (data == 
# 124 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
            ((void *)0)
# 124 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) {
  return;
 }
 data->output = output;
}

static void pid_reg_time(struct pid_data *data, uint32_t *curr_cyc, uint32_t *prev_cyc)
{
 if (data == 
# 132 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
            ((void *)0)
# 132 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) {
  return;
 }
 data->curr_time = curr_cyc;
 data->prev_time = prev_cyc;
}

static void mit_reg_detri_input(struct pid_data *data, float *detri_curr, float *detri_ref)
{
 if (data == 
# 141 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
            ((void *)0)
# 141 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
                ) {
  return;
 }
 data->detri_curr = detri_curr;
 data->detri_ref = detri_ref;
}

static const struct pid_config *pid_get_params(struct pid_data *data)
{
 const struct device *dev = data->pid_dev;
 if (dev == 
# 151 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
           ((void *)0)
# 151 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
               ) {
  return 
# 152 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h" 3 4
        ((void *)0)
# 152 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/pid.h"
            ;
 }
 return dev->config;
}
# 25 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 1
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 1
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 21 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2

# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/limits.h" 1 3 4




# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/newlib.h" 1 3 4
# 6 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/limits.h" 2 3 4

# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/syslimits.h" 1 3 4
# 8 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/limits.h" 2 3 4
# 138 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/limits.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include-fixed/limits.h" 1 3 4
# 139 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/limits.h" 2 3 4
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 1
# 14 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_types.h" 1
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_types.h"
typedef long atomic_t;
typedef atomic_t atomic_val_t;
typedef void *atomic_ptr_t;
typedef atomic_ptr_t atomic_ptr_val_t;
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 2
# 40 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h" 1
# 14 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h" 2
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h"
static inline 
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h" 3 4
             _Bool 
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h"
                  atomic_cas(atomic_t *target, atomic_val_t old_value,
     atomic_val_t new_value)
{
 return __atomic_compare_exchange_n(target, &old_value, new_value,
        0, 5,
        5);
}

static inline 
# 31 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h" 3 4
             _Bool 
# 31 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h"
                  atomic_ptr_cas(atomic_ptr_t *target, atomic_ptr_val_t old_value,
      atomic_ptr_val_t new_value)
{
 return __atomic_compare_exchange_n(target, &old_value, new_value,
        0, 5,
        5);
}

static inline atomic_val_t atomic_add(atomic_t *target, atomic_val_t value)
{
 return __atomic_fetch_add(target, value, 5);
}

static inline atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value)
{
 return __atomic_fetch_sub(target, value, 5);
}

static inline atomic_val_t atomic_inc(atomic_t *target)
{
 return atomic_add(target, 1);
}

static inline atomic_val_t atomic_dec(atomic_t *target)
{
 return atomic_sub(target, 1);
}

static inline atomic_val_t atomic_get(const atomic_t *target)
{
 return __atomic_load_n(target, 5);
}

static inline atomic_ptr_val_t atomic_ptr_get(const atomic_ptr_t *target)
{
 return __atomic_load_n(target, 5);
}

static inline atomic_val_t atomic_set(atomic_t *target, atomic_val_t value)
{




 return __atomic_exchange_n(target, value, 5);
}

static inline atomic_ptr_val_t atomic_ptr_set(atomic_ptr_t *target, atomic_ptr_val_t value)
{
 return __atomic_exchange_n(target, value, 5);
}

static inline atomic_val_t atomic_clear(atomic_t *target)
{
 return atomic_set(target, 0);
}

static inline atomic_ptr_val_t atomic_ptr_clear(atomic_ptr_t *target)
{
 return atomic_ptr_set(target, 
# 90 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h" 3 4
                              ((void *)0)
# 90 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic_builtin.h"
                                  );
}

static inline atomic_val_t atomic_or(atomic_t *target, atomic_val_t value)
{
 return __atomic_fetch_or(target, value, 5);
}

static inline atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value)
{
 return __atomic_fetch_xor(target, value, 5);
}

static inline atomic_val_t atomic_and(atomic_t *target, atomic_val_t value)
{
 return __atomic_fetch_and(target, value, 5);
}

static inline atomic_val_t atomic_nand(atomic_t *target, atomic_val_t value)
{
 return __atomic_fetch_nand(target, value, 5);
}
# 41 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 2
# 127 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
static inline 
# 127 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 3 4
             _Bool 
# 127 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
                  atomic_test_bit(const atomic_t *target, int bit)
{
 atomic_val_t val = atomic_get(((target) + ((bit) / (sizeof(atomic_val_t) * 8))));

 return (1 & (val >> (bit & ((sizeof(atomic_val_t) * 8) - 1)))) != 0;
}
# 147 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
static inline 
# 147 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 3 4
             _Bool 
# 147 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
                  atomic_test_and_clear_bit(atomic_t *target, int bit)
{
 atomic_val_t mask = (1UL << ((unsigned long)(bit) & ((sizeof(atomic_val_t) * 8) - 1U)));
 atomic_val_t old;

 old = atomic_and(((target) + ((bit) / (sizeof(atomic_val_t) * 8))), ~mask);

 return (old & mask) != 0;
}
# 170 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
static inline 
# 170 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 3 4
             _Bool 
# 170 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
                  atomic_test_and_set_bit(atomic_t *target, int bit)
{
 atomic_val_t mask = (1UL << ((unsigned long)(bit) & ((sizeof(atomic_val_t) * 8) - 1U)));
 atomic_val_t old;

 old = atomic_or(((target) + ((bit) / (sizeof(atomic_val_t) * 8))), mask);

 return (old & mask) != 0;
}
# 191 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
static inline void atomic_clear_bit(atomic_t *target, int bit)
{
 atomic_val_t mask = (1UL << ((unsigned long)(bit) & ((sizeof(atomic_val_t) * 8) - 1U)));

 (void)atomic_and(((target) + ((bit) / (sizeof(atomic_val_t) * 8))), ~mask);
}
# 209 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
static inline void atomic_set_bit(atomic_t *target, int bit)
{
 atomic_val_t mask = (1UL << ((unsigned long)(bit) & ((sizeof(atomic_val_t) * 8) - 1U)));

 (void)atomic_or(((target) + ((bit) / (sizeof(atomic_val_t) * 8))), mask);
}
# 228 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
static inline void atomic_set_bit_to(atomic_t *target, int bit, 
# 228 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 3 4
                                                               _Bool 
# 228 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
                                                                    val)
{
 atomic_val_t mask = (1UL << ((unsigned long)(bit) & ((sizeof(atomic_val_t) * 8) - 1U)));

 if (val) {
  (void)atomic_or(((target) + ((bit) / (sizeof(atomic_val_t) * 8))), mask);
 } else {
  (void)atomic_and(((target) + ((bit) / (sizeof(atomic_val_t) * 8))), ~mask);
 }
}
# 254 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"

# 254 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 3 4
_Bool 
# 254 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
    atomic_cas(atomic_t *target, atomic_val_t old_value, atomic_val_t new_value);
# 271 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"

# 271 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h" 3 4
_Bool 
# 271 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
    atomic_ptr_cas(atomic_ptr_t *target, atomic_ptr_val_t old_value,
      atomic_ptr_val_t new_value);
# 286 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_add(atomic_t *target, atomic_val_t value);
# 300 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_sub(atomic_t *target, atomic_val_t value);
# 313 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_inc(atomic_t *target);
# 326 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_dec(atomic_t *target);
# 339 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_get(const atomic_t *target);
# 352 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_ptr_val_t atomic_ptr_get(const atomic_ptr_t *target);
# 367 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_set(atomic_t *target, atomic_val_t value);
# 382 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_ptr_val_t atomic_ptr_set(atomic_ptr_t *target, atomic_ptr_val_t value);
# 396 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_clear(atomic_t *target);
# 410 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_ptr_val_t atomic_ptr_clear(atomic_ptr_t *target);
# 425 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_or(atomic_t *target, atomic_val_t value);
# 440 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_xor(atomic_t *target, atomic_val_t value);
# 455 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_and(atomic_t *target, atomic_val_t value);
# 470 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/atomic.h"
atomic_val_t atomic_nand(atomic_t *target, atomic_val_t value);
# 26 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 1
# 28 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 29 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 2







struct _dnode {
 union {
  struct _dnode *head;
  struct _dnode *next;
 };
 union {
  struct _dnode *tail;
  struct _dnode *prev;
 };
};




typedef struct _dnode sys_dlist_t;



typedef struct _dnode sys_dnode_t;
# 202 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dlist_init(sys_dlist_t *list)
{
 list->head = (sys_dnode_t *)list;
 list->tail = (sys_dnode_t *)list;
}
# 219 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dnode_init(sys_dnode_t *node)
{
 node->next = 
# 221 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             ((void *)0)
# 221 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                 ;
 node->prev = 
# 222 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             ((void *)0)
# 222 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                 ;
}
# 233 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline 
# 233 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             _Bool 
# 233 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                  sys_dnode_is_linked(const sys_dnode_t *node)
{
 return node->next != 
# 235 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                     ((void *)0)
# 235 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                         ;
}
# 247 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline 
# 247 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             _Bool 
# 247 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                  sys_dlist_is_head(sys_dlist_t *list, sys_dnode_t *node)
{
 return list->head == node;
}
# 261 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline 
# 261 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             _Bool 
# 261 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                  sys_dlist_is_tail(sys_dlist_t *list, sys_dnode_t *node)
{
 return list->tail == node;
}
# 274 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline 
# 274 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             _Bool 
# 274 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                  sys_dlist_is_empty(sys_dlist_t *list)
{
 return list->head == list;
}
# 289 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline 
# 289 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
             _Bool 
# 289 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                  sys_dlist_has_multiple_nodes(sys_dlist_t *list)
{
 return list->head != list->tail;
}
# 302 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_head(sys_dlist_t *list)
{
 return sys_dlist_is_empty(list) ? 
# 304 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                                  ((void *)0) 
# 304 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                                       : list->head;
}
# 317 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_head_not_empty(sys_dlist_t *list)
{
 return list->head;
}
# 333 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_next_no_check(sys_dlist_t *list,
       sys_dnode_t *node)
{
 return (node == list->tail) ? 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                              ((void *)0) 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                                   : node->next;
}
# 349 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_next(sys_dlist_t *list,
            sys_dnode_t *node)
{
 return (node != 
# 352 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                ((void *)0)
# 352 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                    ) ? sys_dlist_peek_next_no_check(list, node) : 
# 352 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                                                                   ((void *)0)
# 352 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                                                                       ;
}
# 367 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_prev_no_check(sys_dlist_t *list,
       sys_dnode_t *node)
{
 return (node == list->head) ? 
# 370 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                              ((void *)0) 
# 370 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                                   : node->prev;
}
# 384 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_prev(sys_dlist_t *list,
            sys_dnode_t *node)
{
 return (node != 
# 387 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                ((void *)0)
# 387 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                    ) ? sys_dlist_peek_prev_no_check(list, node) : 
# 387 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                                                                   ((void *)0)
# 387 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                                                                       ;
}
# 398 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_peek_tail(sys_dlist_t *list)
{
 return sys_dlist_is_empty(list) ? 
# 400 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                                  ((void *)0) 
# 400 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                                       : list->tail;
}
# 412 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dlist_append(sys_dlist_t *list, sys_dnode_t *node)
{
 sys_dnode_t *const tail = list->tail;

 node->next = list;
 node->prev = tail;

 tail->next = node;
 list->tail = node;
}
# 432 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dlist_prepend(sys_dlist_t *list, sys_dnode_t *node)
{
 sys_dnode_t *const head = list->head;

 node->next = head;
 node->prev = list;

 head->prev = node;
 list->head = node;
}
# 451 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dlist_insert(sys_dnode_t *successor, sys_dnode_t *node)
{
 sys_dnode_t *const prev = successor->prev;

 node->prev = prev;
 node->next = successor;
 prev->next = node;
 successor->prev = node;
}
# 476 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dlist_insert_at(sys_dlist_t *list, sys_dnode_t *node,
 int (*cond)(sys_dnode_t *node, void *data), void *data)
{
 if (sys_dlist_is_empty(list)) {
  sys_dlist_append(list, node);
 } else {
  sys_dnode_t *pos = sys_dlist_peek_head(list);

  while ((pos != 
# 484 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                ((void *)0)
# 484 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                    ) && (cond(pos, data) == 0)) {
   pos = sys_dlist_peek_next(list, pos);
  }
  if (pos != 
# 487 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
            ((void *)0)
# 487 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                ) {
   sys_dlist_insert(pos, node);
  } else {
   sys_dlist_append(list, node);
  }
 }
}
# 504 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline void sys_dlist_remove(sys_dnode_t *node)
{
 sys_dnode_t *const prev = node->prev;
 sys_dnode_t *const next = node->next;

 prev->next = next;
 next->prev = prev;
 sys_dnode_init(node);
}
# 524 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline sys_dnode_t *sys_dlist_get(sys_dlist_t *list)
{
 sys_dnode_t *node = 
# 526 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                    ((void *)0)
# 526 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                        ;

 if (!sys_dlist_is_empty(list)) {
  node = list->head;
  sys_dlist_remove(node);
 }

 return node;
}
# 543 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
static inline size_t sys_dlist_len(sys_dlist_t *list)
{
 size_t len = 0;
 sys_dnode_t *node = 
# 546 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
                    ((void *)0)
# 546 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
                        ;

 for (node = sys_dlist_peek_head(list); node != 
# 548 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h" 3 4
((void *)0)
# 548 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/dlist.h"
; node = sys_dlist_peek_next(list, node)) {
  len++;
 }
 return len;
}
# 28 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 1
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 24 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/list_gen.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/list_gen.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/list_gen.h" 2
# 26 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 2







struct _snode {
 struct _snode *next;
};



typedef struct _snode sys_snode_t;


struct _slist {
 sys_snode_t *head;
 sys_snode_t *tail;
};



typedef struct _slist sys_slist_t;
# 199 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_init(sys_slist_t *list)
{
 list->head = 
# 201 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
             ((void *)0)
# 201 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
                 ;
 list->tail = 
# 202 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
             ((void *)0)
# 202 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
                 ;
}







static inline sys_snode_t *z_snode_next_peek(sys_snode_t *node)
{
 return node->next;
}

static inline void z_snode_next_set(sys_snode_t *parent, sys_snode_t *child)
{
 parent->next = child;
}

static inline void z_slist_head_set(sys_slist_t *list, sys_snode_t *node)
{
 list->head = node;
}

static inline void z_slist_tail_set(sys_slist_t *list, sys_snode_t *node)
{
 list->tail = node;
}
# 238 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline sys_snode_t *sys_slist_peek_head(sys_slist_t *list)
{
 return list->head;
}
# 250 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline sys_snode_t *sys_slist_peek_tail(sys_slist_t *list)
{
 return list->tail;
}
# 266 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline 
# 266 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
             _Bool 
# 266 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
                  sys_slist_is_empty(sys_slist_t *list);

static inline 
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
_Bool 
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
sys_slist_is_empty(sys_slist_t *list) { return (sys_slist_peek_head(list) == 
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
); }
# 279 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline sys_snode_t *sys_slist_peek_next_no_check(sys_snode_t *node);

static inline sys_snode_t * sys_slist_peek_next_no_check(sys_snode_t *node) { return z_snode_next_peek(node); }
# 290 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline sys_snode_t *sys_slist_peek_next(sys_snode_t *node);

static inline sys_snode_t * sys_slist_peek_next(sys_snode_t *node) { return (node != 
# 292 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 292 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) ? sys_slist_peek_next_no_check(node) : 
# 292 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 292 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; }
# 302 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_prepend(sys_slist_t *list,
         sys_snode_t *node);

static inline void sys_slist_prepend(sys_slist_t *list, sys_snode_t *node) { z_snode_next_set(node, sys_slist_peek_head(list)); z_slist_head_set(list, node); if (sys_slist_peek_tail(list) == 
# 305 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 305 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { z_slist_tail_set(list, sys_slist_peek_head(list)); } }
# 315 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_append(sys_slist_t *list,
        sys_snode_t *node);

static inline void sys_slist_append(sys_slist_t *list, sys_snode_t *node) { z_snode_next_set(node, 
# 318 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 318 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
); if (sys_slist_peek_tail(list) == 
# 318 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 318 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { z_slist_tail_set(list, node); z_slist_head_set(list, node); } else { z_snode_next_set( sys_slist_peek_tail(list), node); z_slist_tail_set(list, node); } }
# 333 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_append_list(sys_slist_t *list,
      void *head, void *tail);

static inline void sys_slist_append_list(sys_slist_t *list, void *head, void *tail) { if (head != 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0) 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
&& tail != 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { if (sys_slist_peek_tail(list) == 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { z_slist_head_set(list, (sys_snode_t *)head); } else { z_snode_next_set( sys_slist_peek_tail(list), (sys_snode_t *)head); } z_slist_tail_set(list, (sys_snode_t *)tail); } }
# 347 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_merge_slist(sys_slist_t *list,
      sys_slist_t *list_to_append);

static inline void sys_slist_merge_slist ( sys_slist_t *list, sys_slist_t *list_to_append) { sys_snode_t *head, *tail; head = sys_slist_peek_head(list_to_append); tail = sys_slist_peek_tail(list_to_append); sys_slist_append_list(list, head, tail); sys_slist_init(list_to_append); }
# 361 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_insert(sys_slist_t *list,
        sys_snode_t *prev,
        sys_snode_t *node);

static inline void sys_slist_insert(sys_slist_t *list, sys_snode_t *prev, sys_snode_t *node) { if (prev == 
# 365 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 365 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { sys_slist_prepend(list, node); } else if (z_snode_next_peek(prev) == 
# 365 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 365 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { sys_slist_append(list, node); } else { z_snode_next_set(node, z_snode_next_peek(prev)); z_snode_next_set(prev, node); } }
# 377 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline sys_snode_t *sys_slist_get_not_empty(sys_slist_t *list);

static inline sys_snode_t * sys_slist_get_not_empty(sys_slist_t *list) { sys_snode_t *node = sys_slist_peek_head(list); z_slist_head_set(list, z_snode_next_peek(node)); if (sys_slist_peek_tail(list) == node) { z_slist_tail_set(list, sys_slist_peek_head(list)); } return node; }
# 390 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline sys_snode_t *sys_slist_get(sys_slist_t *list);

static inline sys_snode_t * sys_slist_get(sys_slist_t *list) { return sys_slist_is_empty(list) ? 
# 392 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0) 
# 392 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
: sys_slist_get_not_empty(list); }
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline void sys_slist_remove(sys_slist_t *list,
        sys_snode_t *prev_node,
        sys_snode_t *node);

static inline void sys_slist_remove(sys_slist_t *list, sys_snode_t *prev_node, sys_snode_t *node) { if (prev_node == 
# 408 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 408 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { z_slist_head_set(list, z_snode_next_peek(node)); if (sys_slist_peek_tail(list) == node) { z_slist_tail_set(list, sys_slist_peek_head(list)); } } else { z_snode_next_set(prev_node, z_snode_next_peek(node)); if (sys_slist_peek_tail(list) == node) { z_slist_tail_set(list, prev_node); } } z_snode_next_set(node, 
# 408 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 408 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
); }
# 420 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline 
# 420 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
             _Bool 
# 420 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
                  sys_slist_find_and_remove(sys_slist_t *list,
          sys_snode_t *node);
# 434 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline 
# 434 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
             _Bool 
# 434 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
                  sys_slist_find(sys_slist_t *list, sys_snode_t *node,
         sys_snode_t **prev);
static inline 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
_Bool 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
sys_slist_find( sys_slist_t *list, sys_snode_t *node, sys_snode_t **prev) { sys_snode_t *current = 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; sys_snode_t *previous = 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; for ((current) = sys_slist_peek_head(list); (current) != 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; (current) = sys_slist_peek_next(current)) { if (current == node) { if (prev != 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { *prev = previous; } return 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
1
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; } previous = current; } if (prev != 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
) { *prev = previous; } return 
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
0
# 436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; }
# 445 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
static inline size_t sys_slist_len(sys_slist_t *list);

static inline size_t sys_slist_len(sys_slist_t * list) { size_t len = 0; static sys_snode_t * node; for ((node) = sys_slist_peek_head(list); (node) != 
# 447 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 447 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; (node) = sys_slist_peek_next(node)) { len++; } return len; }


static inline 
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
_Bool 
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
sys_slist_find_and_remove(sys_slist_t *list, sys_snode_t *node) { sys_snode_t *prev = 
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; sys_snode_t *test; for ((test) = sys_slist_peek_head(list); (test) != 
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
((void *)0)
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; (test) = sys_slist_peek_next(test)) { if (test == node) { sys_slist_remove(list, prev, node); return 
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
1
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; } prev = test; } return 
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h" 3 4
0
# 450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/slist.h"
; }
# 29 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 1
# 38 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
struct _sfnode {
 uintptr_t next_and_flags;
};



typedef struct _sfnode sys_sfnode_t;


struct _sflist {
 sys_sfnode_t *head;
 sys_sfnode_t *tail;
};



typedef struct _sflist sys_sflist_t;
# 204 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_init(sys_sflist_t *list)
{
 list->head = 
# 206 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
             ((void *)0)
# 206 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
                 ;
 list->tail = 
# 207 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
             ((void *)0)
# 207 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
                 ;
}
# 219 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
_Static_assert((((uintptr_t)(__alignof__(sys_sfnode_t) - 1)) >= 0x3), "" );

static inline sys_sfnode_t *z_sfnode_next_peek(sys_sfnode_t *node)
{
 return (sys_sfnode_t *)(node->next_and_flags & ~((uintptr_t)(__alignof__(sys_sfnode_t) - 1)));
}

static inline uint8_t sys_sfnode_flags_get(sys_sfnode_t *node);

static inline void z_sfnode_next_set(sys_sfnode_t *parent,
           sys_sfnode_t *child)
{
 uint8_t cur_flags = sys_sfnode_flags_get(parent);

 parent->next_and_flags = cur_flags | (uintptr_t)child;
}

static inline void z_sflist_head_set(sys_sflist_t *list, sys_sfnode_t *node)
{
 list->head = node;
}

static inline void z_sflist_tail_set(sys_sflist_t *list, sys_sfnode_t *node)
{
 list->tail = node;
}
# 253 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline sys_sfnode_t *sys_sflist_peek_head(sys_sflist_t *list)
{
 return list->head;
}
# 265 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline sys_sfnode_t *sys_sflist_peek_tail(sys_sflist_t *list)
{
 return list->tail;
}
# 281 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline uint8_t sys_sfnode_flags_get(sys_sfnode_t *node)
{
 return node->next_and_flags & ((uintptr_t)(__alignof__(sys_sfnode_t) - 1));
}
# 300 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sfnode_init(sys_sfnode_t *node, uint8_t flags)
{
 { };
 node->next_and_flags = flags;
}
# 317 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sfnode_flags_set(sys_sfnode_t *node, uint8_t flags)
{
 { };
 node->next_and_flags = (uintptr_t)(z_sfnode_next_peek(node)) | flags;
}
# 334 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline 
# 334 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
             _Bool 
# 334 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
                  sys_sflist_is_empty(sys_sflist_t *list);

static inline 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
_Bool 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
sys_sflist_is_empty(sys_sflist_t *list) { return (sys_sflist_peek_head(list) == 
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
); }
# 347 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline sys_sfnode_t *sys_sflist_peek_next_no_check(sys_sfnode_t *node);

static inline sys_sfnode_t * sys_sflist_peek_next_no_check(sys_sfnode_t *node) { return z_sfnode_next_peek(node); }
# 358 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline sys_sfnode_t *sys_sflist_peek_next(sys_sfnode_t *node);

static inline sys_sfnode_t * sys_sflist_peek_next(sys_sfnode_t *node) { return (node != 
# 360 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 360 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) ? sys_sflist_peek_next_no_check(node) : 
# 360 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 360 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
; }
# 370 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_prepend(sys_sflist_t *list,
          sys_sfnode_t *node);

static inline void sys_sflist_prepend(sys_sflist_t *list, sys_sfnode_t *node) { z_sfnode_next_set(node, sys_sflist_peek_head(list)); z_sflist_head_set(list, node); if (sys_sflist_peek_tail(list) == 
# 373 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 373 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { z_sflist_tail_set(list, sys_sflist_peek_head(list)); } }
# 383 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_append(sys_sflist_t *list,
         sys_sfnode_t *node);

static inline void sys_sflist_append(sys_sflist_t *list, sys_sfnode_t *node) { z_sfnode_next_set(node, 
# 386 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 386 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
); if (sys_sflist_peek_tail(list) == 
# 386 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 386 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { z_sflist_tail_set(list, node); z_sflist_head_set(list, node); } else { z_sfnode_next_set( sys_sflist_peek_tail(list), node); z_sflist_tail_set(list, node); } }
# 401 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_append_list(sys_sflist_t *list,
       void *head, void *tail);

static inline void sys_sflist_append_list(sys_sflist_t *list, void *head, void *tail) { if (head != 
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0) 
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
&& tail != 
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { if (sys_sflist_peek_tail(list) == 
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 404 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { z_sflist_head_set(list, (sys_sfnode_t *)head); } else { z_sfnode_next_set( sys_sflist_peek_tail(list), (sys_sfnode_t *)head); } z_sflist_tail_set(list, (sys_sfnode_t *)tail); } }
# 415 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_merge_sflist(sys_sflist_t *list,
        sys_sflist_t *list_to_append);

static inline void sys_sflist_merge_sflist ( sys_sflist_t *list, sys_sflist_t *list_to_append) { sys_sfnode_t *head, *tail; head = sys_sflist_peek_head(list_to_append); tail = sys_sflist_peek_tail(list_to_append); sys_sflist_append_list(list, head, tail); sys_sflist_init(list_to_append); }
# 429 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_insert(sys_sflist_t *list,
         sys_sfnode_t *prev,
         sys_sfnode_t *node);

static inline void sys_sflist_insert(sys_sflist_t *list, sys_sfnode_t *prev, sys_sfnode_t *node) { if (prev == 
# 433 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 433 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { sys_sflist_prepend(list, node); } else if (z_sfnode_next_peek(prev) == 
# 433 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 433 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { sys_sflist_append(list, node); } else { z_sfnode_next_set(node, z_sfnode_next_peek(prev)); z_sfnode_next_set(prev, node); } }
# 445 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline sys_sfnode_t *sys_sflist_get_not_empty(sys_sflist_t *list);

static inline sys_sfnode_t * sys_sflist_get_not_empty(sys_sflist_t *list) { sys_sfnode_t *node = sys_sflist_peek_head(list); z_sflist_head_set(list, z_sfnode_next_peek(node)); if (sys_sflist_peek_tail(list) == node) { z_sflist_tail_set(list, sys_sflist_peek_head(list)); } return node; }
# 458 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline sys_sfnode_t *sys_sflist_get(sys_sflist_t *list);

static inline sys_sfnode_t * sys_sflist_get(sys_sflist_t *list) { return sys_sflist_is_empty(list) ? 
# 460 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0) 
# 460 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
: sys_sflist_get_not_empty(list); }
# 472 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline void sys_sflist_remove(sys_sflist_t *list,
         sys_sfnode_t *prev_node,
         sys_sfnode_t *node);

static inline void sys_sflist_remove(sys_sflist_t *list, sys_sfnode_t *prev_node, sys_sfnode_t *node) { if (prev_node == 
# 476 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 476 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
) { z_sflist_head_set(list, z_sfnode_next_peek(node)); if (sys_sflist_peek_tail(list) == node) { z_sflist_tail_set(list, sys_sflist_peek_head(list)); } } else { z_sfnode_next_set(prev_node, z_sfnode_next_peek(node)); if (sys_sflist_peek_tail(list) == node) { z_sflist_tail_set(list, prev_node); } } z_sfnode_next_set(node, 
# 476 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 476 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
); }
# 488 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline 
# 488 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
             _Bool 
# 488 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
                  sys_sflist_find_and_remove(sys_sflist_t *list,
           sys_sfnode_t *node);

static inline 
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
_Bool 
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
sys_sflist_find_and_remove(sys_sflist_t *list, sys_sfnode_t *node) { sys_sfnode_t *prev = 
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
; sys_sfnode_t *test; for ((test) = sys_sflist_peek_head(list); (test) != 
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
; (test) = sys_sflist_peek_next(test)) { if (test == node) { sys_sflist_remove(list, prev, node); return 
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
1
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
; } prev = test; } return 
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
0
# 491 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
; }
# 500 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
static inline size_t sys_sflist_len(sys_sflist_t *list);

static inline size_t sys_sflist_len(sys_sflist_t * list) { size_t len = 0; static sys_sfnode_t * node; for ((node) = sys_sflist_peek_head(list); (node) != 
# 502 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h" 3 4
((void *)0)
# 502 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sflist.h"
; (node) = sys_sflist_peek_next(node)) { len++; } return len; }
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h" 1
# 63 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
struct k_obj_type;
struct k_obj_core;
# 89 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
extern sys_slist_t z_obj_type_list;


struct k_obj_core_stats_desc {
 size_t raw_size;
 size_t query_size;


 int (*raw)(struct k_obj_core *obj_core, void *stats);

 int (*query)(struct k_obj_core *obj_core, void *stats);

 int (*reset)(struct k_obj_core *obj_core);

 int (*disable)(struct k_obj_core *obj_core);

 int (*enable)(struct k_obj_core *obj_core);
};


struct k_obj_type {
 sys_snode_t node;
 sys_slist_t list;
 uint32_t id;
 size_t obj_core_offset;




};


struct k_obj_core {
 sys_snode_t node;
 struct k_obj_type *type;



};
# 141 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
struct k_obj_type *z_obj_type_init(struct k_obj_type *type,
       uint32_t id, size_t off);
# 155 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
struct k_obj_type *k_obj_type_find(uint32_t type_id);
# 174 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_type_walk_locked(struct k_obj_type *type,
      int (*func)(struct k_obj_core *, void *),
      void *data);
# 196 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_type_walk_unlocked(struct k_obj_type *type,
        int (*func)(struct k_obj_core *, void *),
        void *data);
# 209 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
void k_obj_core_init(struct k_obj_core *obj_core, struct k_obj_type *type);
# 221 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
void k_obj_core_link(struct k_obj_core *obj_core);
# 232 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
void k_obj_core_init_and_link(struct k_obj_core *obj_core,
         struct k_obj_type *type);
# 244 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
void k_obj_core_unlink(struct k_obj_core *obj_core);
# 299 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_register(struct k_obj_core *obj_core, void *stats,
         size_t stats_len);
# 314 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_deregister(struct k_obj_core *obj_core);
# 331 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_raw(struct k_obj_core *obj_core, void *stats,
    size_t stats_len);
# 350 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_query(struct k_obj_core *obj_core, void *stats,
      size_t stats_len);
# 364 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_reset(struct k_obj_core *obj_core);
# 378 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_disable(struct k_obj_core *obj_core);
# 391 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/obj_core.h"
int k_obj_core_stats_enable(struct k_obj_core *obj_core);
# 32 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h" 1
# 28 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h" 1
# 9 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_stats.h" 1
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_stats.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 21 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mem_stats.h" 2



struct sys_memory_stats {
 size_t free_bytes;
 size_t allocated_bytes;
 size_t max_allocated_bytes;
};
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h" 2
# 57 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
struct sys_heap {
 struct z_heap *heap;
 void *init_mem;
 size_t init_bytes;
};

struct z_heap_stress_result {
 uint32_t total_allocs;
 uint32_t successful_allocs;
 uint32_t total_frees;
 uint64_t accumulated_in_use_bytes;
};
# 109 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void sys_heap_init(struct sys_heap *heap, void *mem, size_t bytes);
# 128 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void *sys_heap_alloc(struct sys_heap *heap, size_t bytes);
# 143 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void *sys_heap_aligned_alloc(struct sys_heap *heap, size_t align, size_t bytes);
# 158 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void sys_heap_free(struct sys_heap *heap, void *mem);
# 178 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void *sys_heap_aligned_realloc(struct sys_heap *heap, void *ptr,
          size_t align, size_t bytes);
# 198 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
size_t sys_heap_usable_size(struct sys_heap *heap, void *mem);
# 216 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
static inline 
# 216 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h" 3 4
             _Bool 
# 216 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
                  sys_heap_validate(struct sys_heap *heap)
{
 (void)(heap);
 return 
# 219 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h" 3 4
       1
# 219 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
           ;
}
# 252 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void sys_heap_stress(void *(*alloc_fn)(void *arg, size_t bytes),
       void (*free_fn)(void *arg, void *p),
       void *arg, size_t total_bytes,
       uint32_t op_count,
       void *scratch_mem, size_t scratch_bytes,
       int target_percent,
       struct z_heap_stress_result *result);
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
void sys_heap_print_info(struct sys_heap *heap, 
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h" 3 4
                                               _Bool 
# 268 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/sys_heap.h"
                                                    dump_chunks);
# 29 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/structs.h" 1
# 31 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/structs.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/structs.h" 1
# 22 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/structs.h"
struct _cpu_arch {






};
# 32 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/structs.h" 2
# 49 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/structs.h"
typedef struct _cpu_arch _cpu_arch_t;
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/stats.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/stats.h"
struct k_cycle_stats {
 uint64_t total;
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/stats.h"
 
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/stats.h" 3 4
_Bool 
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/stats.h"
          track_usage;
};
# 31 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h" 1
# 58 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"
struct rbnode {

 struct rbnode *children[2];

};
# 86 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"
typedef 
# 86 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h" 3 4
       _Bool 
# 86 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"
            (*rb_lessthan_t)(struct rbnode *a, struct rbnode *b);




struct rbtree {

 struct rbnode *root;

 rb_lessthan_t lessthan_fn;

 int max_depth;





};






typedef void (*rb_visit_t)(struct rbnode *node, void *cookie);

struct rbnode *z_rb_child(struct rbnode *node, uint8_t side);
int z_rb_is_black(struct rbnode *node);

void z_rb_walk(struct rbnode *node, rb_visit_t visit_fn, void *cookie);

struct rbnode *z_rb_get_minmax(struct rbtree *tree, uint8_t side);




void rb_insert(struct rbtree *tree, struct rbnode *node);




void rb_remove(struct rbtree *tree, struct rbnode *node);




static inline struct rbnode *rb_get_min(struct rbtree *tree)
{
 return z_rb_get_minmax(tree, 0U);
}




static inline struct rbnode *rb_get_max(struct rbtree *tree)
{
 return z_rb_get_minmax(tree, 1U);
}
# 154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"

# 154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h" 3 4
_Bool 
# 154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"
    rb_contains(struct rbtree *tree, struct rbnode *node);
# 165 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"
static inline void rb_walk(struct rbtree *tree, rb_visit_t visit_fn,
      void *cookie)
{
 z_rb_walk(tree->root, visit_fn, cookie);
}


struct _rb_foreach {
 struct rbnode **stack;
 uint8_t *is_left;
 int32_t top;
};
# 193 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/rb.h"
struct rbnode *z_rb_foreach_next(struct rbtree *tree, struct _rb_foreach *f);
# 33 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h" 2
# 109 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
struct _priq_rb {
 struct rbtree tree;
 int next_order_key;
};
# 122 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
struct _priq_mq {
 sys_dlist_t queues[(15 + 16 + 1)];
 unsigned long bitmask[(((((15 + 16 + 1)) + ((8 * 4)) - 1) / ((8 * 4))))];
};

struct _ready_q {


 struct k_thread *cache;



 sys_dlist_t runq;





};

typedef struct _ready_q _ready_q_t;

struct _cpu {

 uint32_t nested;


 char *irq_stack;


 struct k_thread *current;


 struct k_thread *idle_thread;
# 167 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
 uint8_t id;


 void *fp_ctx;
# 185 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
 uint32_t usage0;


 struct k_cycle_stats *usage;
# 197 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
 struct _cpu_arch arch;
};

typedef struct _cpu _cpu_t;

struct z_kernel {
 struct _cpu cpus[1];
# 214 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
 struct _ready_q ready_q;
# 228 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
 struct k_thread *current_fp;



 struct k_thread *threads;


 struct k_cycle_stats usage[1];
# 246 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
};

typedef struct z_kernel _kernel_t;

extern struct z_kernel _kernel;

extern atomic_t _cpus_active;
# 284 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_structs.h"
typedef struct {
 sys_dlist_t waitq;
} _wait_q_t;






struct _timeout;
typedef void (*_timeout_func_t)(struct _timeout *t);

struct _timeout {
 sys_dnode_t node;
 _timeout_func_t fn;


 int64_t dticks;



};

typedef void (*k_thread_timeslice_fn_t)(struct k_thread *thread, void *data);
# 33 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_version.h" 1
# 47 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_version.h"
uint32_t sys_kernel_version_get(void);
# 34 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/printk.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/printk.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/printk.h" 2
# 47 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/printk.h"
__attribute__((format (printf, 1, 2))) void printk(const char *fmt, ...);
__attribute__((format (printf, 1, 0))) void vprintk(const char *fmt, va_list ap);
# 65 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/printk.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 1 3 4
# 48 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 49 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 2 3 4
# 81 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4

# 81 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
typedef uint16_t __ungetc_t;


struct __file {
 __ungetc_t unget;
 uint8_t flags;
# 95 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
 int (*put)(char, struct __file *);
 int (*get)(struct __file *);
 int (*flush)(struct __file *);
};





struct __file_close {
 struct __file file;
 int (*close)(struct __file *);
};







struct __file_ext {
        struct __file_close cfile;
        __off_t (*seek)(struct __file *, __off_t offset, int whence);
        int (*setvbuf)(struct __file *, char *buf, int mode, size_t size);
};
# 133 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
typedef struct __file __FILE;

typedef __FILE FILE;
# 146 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
extern FILE *const stdin;
extern FILE *const stdout;
extern FILE *const stderr;
# 200 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
FILE *fdevopen(int (*__put)(char, FILE*), int (*__get)(FILE*), int(*__flush)(FILE *));
int fclose(FILE *__stream);
int fflush(FILE *stream);
# 220 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
int fputc(int __c, FILE *__stream);
int putc(int __c, FILE *__stream);
int putchar(int __c);



int printf(const char *__fmt, ...) __attribute__((__format__ (printf, 1, 2)));
int fprintf(FILE *__stream, const char *__fmt, ...) __attribute__((__format__ (printf, 2, 3)));
int vprintf(const char *__fmt, va_list __ap) __attribute__((__format__ (printf, 1, 0)));
int vfprintf(FILE *__stream, const char *__fmt, va_list __ap) __attribute__((__format__ (printf, 2, 0)));
int sprintf(char *__s, const char *__fmt, ...) __attribute__((__format__ (printf, 2, 3)));
int snprintf(char *__s, size_t __n, const char *__fmt, ...) __attribute__((__format__ (printf, 3, 4)));
int vsprintf(char *__s, const char *__fmt, va_list ap) __attribute__((__format__ (printf, 2, 0)));
int vsnprintf(char *__s, size_t __n, const char *__fmt, va_list ap) __attribute__((__format__ (printf, 3, 0)));
int asprintf(char **strp, const char *fmt, ...) __attribute__((__format__ (printf, 2, 3)));
int vasprintf(char **strp, const char *fmt, va_list ap) __attribute__((__format__ (printf, 2, 0)));

int fputs(const char *__str, FILE *__stream);
int puts(const char *__str);
size_t fwrite(const void *__ptr, size_t __size, size_t __nmemb,
         FILE *__stream);

int fgetc(FILE *__stream);
int getc(FILE *__stream);
int getchar(void);


int ungetc(int __c, FILE *__stream);

int scanf(const char *__fmt, ...) __attribute__((__format__ (scanf, 1, 2)));
int fscanf(FILE *__stream, const char *__fmt, ...) __attribute__((__format__ (scanf, 2, 3)));
int vscanf(const char *__fmt, va_list __ap) __attribute__((__format__ (scanf, 1, 0)));
int vfscanf(FILE *__stream, const char *__fmt, va_list __ap) __attribute__((__format__ (scanf, 2, 0)));
int sscanf(const char *__buf, const char *__fmt, ...) __attribute__((__format__ (scanf, 2, 3)));
int vsscanf(const char *__buf, const char *__fmt, va_list ap) __attribute__((__format__ (scanf, 2, 0)));

char *fgets(char *__str, int __size, FILE *__stream);
char *gets(char *__str);
size_t fread(void *__ptr, size_t __size, size_t __nmemb,
        FILE *__stream);

void clearerr(FILE *__stream);




int feof(FILE *__stream);




int ferror(FILE *__stream);
# 302 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
__extension__ typedef _fpos_t fpos_t;
int fgetpos(FILE *stream, fpos_t *pos);
FILE *fopen(const char *path, const char *mode) __attribute__((__malloc__, __malloc__(fclose,1)));
FILE *freopen(const char *path, const char *mode, FILE *stream);
FILE *fdopen(int, const char *) __attribute__((__malloc__, __malloc__(fclose,1)));
FILE *fmemopen(void *buf, size_t size, const char *mode) __attribute__((__malloc__, __malloc__(fclose,1)));
int fseek(FILE *stream, long offset, int whence);
int fseeko(FILE *stream, __off_t offset, int whence);
int fsetpos(FILE *stream, fpos_t *pos);
long ftell(FILE *stream);
__off_t ftello(FILE *stream);
int fileno(FILE *);
void perror(const char *s);
int remove(const char *pathname);
int rename(const char *oldpath, const char *newpath);
void rewind(FILE *stream);
void setbuf(FILE *stream, char *buf);
void setbuffer(FILE *stream, char *buf, size_t size);
void setlinebuf(FILE *stream);
int setvbuf(FILE *stream, char *buf, int mode, size_t size);
FILE *tmpfile(void);
char *tmpnam (char *s);
ssize_t getline(char **restrict lineptr, size_t *restrict n, FILE *restrict stream);
ssize_t getdelim(char **restrict lineptr, size_t *restrict n, int delim, FILE *restrict stream);
# 349 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/stdio.h" 3 4
static __inline uint32_t
__printf_float(float f)
{
 union {
  float f;
  uint32_t u;
 } u = { .f = f };
 return u.u;
}
# 66 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/printk.h" 2
# 36 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/cpu.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/cpu.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 1
# 32 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 33 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/cpu.h" 1
# 35 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_offload.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_offload.h"

# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_offload.h"
typedef void (*irq_offload_routine_t)(const void *parameter);
# 39 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq_offload.h"
void irq_offload(irq_offload_routine_t routine, const void *parameter);
# 36 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 2






struct arch_esf;
struct k_thread;
struct k_mem_domain;

typedef struct z_thread_stack_element k_thread_stack_t;

typedef void (*k_thread_entry_t)(void *p1, void *p2, void *p3);

__attribute__((deprecated)) typedef struct arch_esf z_arch_esf_t;
# 72 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline uint32_t arch_k_cycle_get_32(void);
# 86 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline uint64_t arch_k_cycle_get_64(void);
# 187 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_cpu_idle(void);
# 207 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_cpu_atomic_idle(unsigned int key);
# 222 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
typedef void (*arch_cpustart_t)(void *data);
# 244 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_cpu_start(int cpu_num, k_thread_stack_t *stack, int sz,
      arch_cpustart_t fn, void *arg);







# 252 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 3 4
_Bool 
# 252 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
    arch_cpu_active(int cpu_num);
# 267 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline unsigned int arch_irq_lock(void);






static inline void arch_irq_unlock(unsigned int key);
# 283 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline 
# 283 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 3 4
             _Bool 
# 283 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
                  arch_irq_unlocked(unsigned int key);
# 299 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_irq_disable(unsigned int irq);






void arch_irq_enable(unsigned int irq);






int arch_irq_is_enabled(unsigned int irq);
# 326 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
int arch_irq_connect_dynamic(unsigned int irq, unsigned int priority,
        void (*routine)(const void *parameter),
        const void *parameter, uint32_t flags);
# 344 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
int arch_irq_disconnect_dynamic(unsigned int irq, unsigned int priority,
    void (*routine)(const void *parameter),
    const void *parameter, uint32_t flags);
# 402 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
unsigned int arch_irq_allocate(void);
# 412 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_irq_set_used(unsigned int irq);
# 421 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"

# 421 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 3 4
_Bool 
# 421 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
    arch_irq_is_used(unsigned int irq);
# 529 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline unsigned int arch_num_cpus(void);
# 890 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline 
# 890 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 3 4
             _Bool 
# 890 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
                  arch_mem_coherent(void *ptr)
{
 (void)(ptr);
 return 
# 893 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 3 4
       1
# 893 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
           ;
}
# 938 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
static inline void arch_cohere_stacks(struct k_thread *old_thread,
          void *old_switch_handle,
          struct k_thread *new_thread)
{
 (void)(old_thread);
 (void)(old_switch_handle);
 (void)(new_thread);
}
# 1259 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_spin_relax(void);
# 1279 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
typedef 
# 1279 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 3 4
       _Bool 
# 1279 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
            (*stack_trace_callback_fn)(void *cookie, unsigned long addr);
# 1296 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
void arch_stack_walk(stack_trace_callback_fn callback_fn, void *cookie,
       const struct k_thread *thread, const struct arch_esf *esf);
# 1308 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_inlines.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_inlines.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch_inlines.h" 1
# 22 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch_inlines.h"
static inline __attribute__((always_inline)) _cpu_t *arch_curr_cpu(void)
{

 return &_kernel.cpus[0];
}



static inline __attribute__((always_inline)) uint32_t arch_proc_id(void)
{




 return arch_curr_cpu()->id;
}

static inline __attribute__((always_inline)) unsigned int arch_num_cpus(void)
{
 return 1;
}
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_inlines.h" 2
# 1309 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arch_interface.h" 2
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/cpu.h" 2






# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 1
# 22 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/thread.h" 1
# 25 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/thread.h"
struct _callee_saved {
 uint32_t v1;
 uint32_t v2;
 uint32_t v3;
 uint32_t v4;
 uint32_t v5;
 uint32_t v6;
 uint32_t v7;
 uint32_t v8;
 uint32_t psp;



};

typedef struct _callee_saved _callee_saved_t;


struct _preempt_float {
 float s16;
 float s17;
 float s18;
 float s19;
 float s20;
 float s21;
 float s22;
 float s23;
 float s24;
 float s25;
 float s26;
 float s27;
 float s28;
 float s29;
 float s30;
 float s31;
};


struct _thread_arch {


 uint32_t basepri;


 uint32_t swap_return_value;







 struct _preempt_float preempt_float;
# 117 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/thread.h"
 union {
  uint32_t mode;


  struct {
   uint8_t mode_bits;
   uint8_t mode_exc_return;
   uint16_t mode_reserved2;
  };

 };
# 137 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/thread.h"
};




typedef struct _thread_arch _thread_arch_t;
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/exception.h" 1
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/exception.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/exception.h" 1
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/exception.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/nvic.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/exception.h" 2
# 77 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/exception.h"
struct __fpu_sf {
 uint32_t s[16];



 uint32_t fpscr;
 uint32_t undefined;
};
# 104 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/exception.h"
struct arch_esf {
 struct __basic_sf {
  union { uint32_t a1, r0; };
  union { uint32_t a2, r1; };
  union { uint32_t a3, r2; };
  union { uint32_t a4, r3; };
  union { uint32_t ip, r12; };
  union { uint32_t lr, r14; };
  union { uint32_t pc, r15; };
  uint32_t xpsr;
 } basic;

 struct __fpu_sf fpu;




};

extern uint32_t z_arm_coredump_fault_sp;

extern void z_arm_exc_exit(void);
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/exception.h" 2
# 24 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/irq.h" 1
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/irq.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sw_isr_table.h" 1
# 29 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sw_isr_table.h"
void _isr_wrapper(void);


void z_irq_spurious(const void *unused);






struct _isr_table_entry {
 const void *arg;
 void (*isr)(const void *);
};




extern struct _isr_table_entry _sw_isr_table[];

struct _irq_parent_entry {
 const struct device *dev;
 unsigned int level;
 unsigned int irq;
 unsigned int offset;
};
# 139 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sw_isr_table.h"
struct _isr_list {

 int32_t irq;

 int32_t flags;

 void *func;

 const void *param;
};
# 159 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sw_isr_table.h"
struct _isr_list_sname {

 int32_t irq;

 int32_t flags;

 const char sname[];
};
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/irq.h" 2
# 39 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/irq.h"
extern void arch_irq_enable(unsigned int irq);
extern void arch_irq_disable(unsigned int irq);
extern int arch_irq_is_enabled(unsigned int irq);


extern void z_arm_irq_priority_set(unsigned int irq, unsigned int prio,
       uint32_t flags);
# 74 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/irq.h"
extern void z_arm_int_exit(void);

extern void z_arm_interrupt_init(void);
# 145 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/irq.h"
extern void z_arm_int_exit(void);






static inline void arch_isr_direct_header(void)
{



}

static inline void arch_isr_direct_footer(int maybe_swap)
{



 if (maybe_swap != 0) {
  z_arm_int_exit();
 }
}
# 25 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/error.h" 1
# 26 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/misc.h" 1
# 22 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/misc.h"
extern uint32_t sys_clock_cycle_get_32(void);

static inline uint32_t arch_k_cycle_get_32(void)
{
 return sys_clock_cycle_get_32();
}

extern uint64_t sys_clock_cycle_get_64(void);

static inline uint64_t arch_k_cycle_get_64(void)
{
 return sys_clock_cycle_get_64();
}

static inline __attribute__((always_inline)) void arch_nop(void)
{
 __asm__ volatile("nop");
}
# 27 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/addr_types.h" 1
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/addr_types.h"
typedef uintptr_t paddr_t;
typedef void *vaddr_t;
# 28 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/ffs.h" 1
# 31 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/ffs.h"
static inline __attribute__((always_inline)) unsigned int find_msb_set(uint32_t op)
{
 if (op == 0) {
  return 0;
 }

 return 32 - __builtin_clz(op);
}
# 53 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/ffs.h"
static inline __attribute__((always_inline)) unsigned int find_lsb_set(uint32_t op)
{

 return __builtin_ffs(op);
# 68 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/ffs.h"
}
# 29 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/nmi.h" 1
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/nmi.h"
extern void z_arm_nmi_set_handler(void (*pHandler)(void));
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline.h" 1
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h" 1
# 24 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h"
# 1 "/home/ttwards/zephyrproject/zephyr/modules/cmsis/./cmsis_core.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/modules/cmsis/./cmsis_core.h"
# 1 "/home/ttwards/zephyrproject/zephyr/modules/cmsis/./cmsis_core_m.h" 1
# 24 "/home/ttwards/zephyrproject/zephyr/modules/cmsis/./cmsis_core_m.h"
# 1 "/home/ttwards/zephyrproject/zephyr/soc/st/stm32/stm32f4x/./soc.h" 1
# 23 "/home/ttwards/zephyrproject/zephyr/soc/st/stm32/stm32f4x/./soc.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h" 1
# 132 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f407xx.h" 1
# 65 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f407xx.h"
typedef enum
{

  NonMaskableInt_IRQn = -14,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn = -11,
  UsageFault_IRQn = -10,
  SVCall_IRQn = -5,
  DebugMonitor_IRQn = -4,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,

  WWDG_IRQn = 0,
  PVD_IRQn = 1,
  TAMP_STAMP_IRQn = 2,
  RTC_WKUP_IRQn = 3,
  FLASH_IRQn = 4,
  RCC_IRQn = 5,
  EXTI0_IRQn = 6,
  EXTI1_IRQn = 7,
  EXTI2_IRQn = 8,
  EXTI3_IRQn = 9,
  EXTI4_IRQn = 10,
  DMA1_Stream0_IRQn = 11,
  DMA1_Stream1_IRQn = 12,
  DMA1_Stream2_IRQn = 13,
  DMA1_Stream3_IRQn = 14,
  DMA1_Stream4_IRQn = 15,
  DMA1_Stream5_IRQn = 16,
  DMA1_Stream6_IRQn = 17,
  ADC_IRQn = 18,
  CAN1_TX_IRQn = 19,
  CAN1_RX0_IRQn = 20,
  CAN1_RX1_IRQn = 21,
  CAN1_SCE_IRQn = 22,
  EXTI9_5_IRQn = 23,
  TIM1_BRK_TIM9_IRQn = 24,
  TIM1_UP_TIM10_IRQn = 25,
  TIM1_TRG_COM_TIM11_IRQn = 26,
  TIM1_CC_IRQn = 27,
  TIM2_IRQn = 28,
  TIM3_IRQn = 29,
  TIM4_IRQn = 30,
  I2C1_EV_IRQn = 31,
  I2C1_ER_IRQn = 32,
  I2C2_EV_IRQn = 33,
  I2C2_ER_IRQn = 34,
  SPI1_IRQn = 35,
  SPI2_IRQn = 36,
  USART1_IRQn = 37,
  USART2_IRQn = 38,
  USART3_IRQn = 39,
  EXTI15_10_IRQn = 40,
  RTC_Alarm_IRQn = 41,
  OTG_FS_WKUP_IRQn = 42,
  TIM8_BRK_TIM12_IRQn = 43,
  TIM8_UP_TIM13_IRQn = 44,
  TIM8_TRG_COM_TIM14_IRQn = 45,
  TIM8_CC_IRQn = 46,
  DMA1_Stream7_IRQn = 47,
  FSMC_IRQn = 48,
  SDIO_IRQn = 49,
  TIM5_IRQn = 50,
  SPI3_IRQn = 51,
  UART4_IRQn = 52,
  UART5_IRQn = 53,
  TIM6_DAC_IRQn = 54,
  TIM7_IRQn = 55,
  DMA2_Stream0_IRQn = 56,
  DMA2_Stream1_IRQn = 57,
  DMA2_Stream2_IRQn = 58,
  DMA2_Stream3_IRQn = 59,
  DMA2_Stream4_IRQn = 60,
  ETH_IRQn = 61,
  ETH_WKUP_IRQn = 62,
  CAN2_TX_IRQn = 63,
  CAN2_RX0_IRQn = 64,
  CAN2_RX1_IRQn = 65,
  CAN2_SCE_IRQn = 66,
  OTG_FS_IRQn = 67,
  DMA2_Stream5_IRQn = 68,
  DMA2_Stream6_IRQn = 69,
  DMA2_Stream7_IRQn = 70,
  USART6_IRQn = 71,
  I2C3_EV_IRQn = 72,
  I2C3_ER_IRQn = 73,
  OTG_HS_EP1_OUT_IRQn = 74,
  OTG_HS_EP1_IN_IRQn = 75,
  OTG_HS_WKUP_IRQn = 76,
  OTG_HS_IRQn = 77,
  DCMI_IRQn = 78,
  RNG_IRQn = 80,
  FPU_IRQn = 81
} IRQn_Type;







# 1 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h" 1
# 63 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_version.h" 1
# 64 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h" 2
# 162 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_compiler.h" 1
# 54 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_compiler.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h" 1
# 29 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
# 71 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed)) T_UINT32 { uint32_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT16_WRITE { uint16_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT16_READ { uint16_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT32_WRITE { uint32_t v; };
#pragma GCC diagnostic pop



#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpacked"
#pragma GCC diagnostic ignored "-Wattributes"
  struct __attribute__((packed, aligned(1))) T_UINT32_READ { uint32_t v; };
#pragma GCC diagnostic pop
# 258 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __ISB(void)
{
  __asm volatile ("isb 0xF":::"memory");
}







__attribute__((always_inline)) static inline void __DSB(void)
{
  __asm volatile ("dsb 0xF":::"memory");
}







__attribute__((always_inline)) static inline void __DMB(void)
{
  __asm volatile ("dmb 0xF":::"memory");
}
# 292 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __REV(uint32_t value)
{

  return __builtin_bswap32(value);






}
# 311 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return result;
}
# 326 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline int16_t __REVSH(int16_t value)
{

  return (int16_t)__builtin_bswap16(value);






}
# 346 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  op2 %= 32U;
  if (op2 == 0U)
  {
    return op1;
  }
  return (op1 >> op2) | (op1 << (32U - op2));
}
# 373 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;






  uint32_t s = (4U * 8U) - 1U;

  result = value;
  for (value >>= 1U; value != 0U; value >>= 1U)
  {
    result <<= 1U;
    result |= value & 1U;
    s--;
  }
  result <<= s;

  return result;
}
# 403 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint8_t __CLZ(uint32_t value)
{
# 414 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
  if (value == 0U)
  {
    return 32U;
  }
  return __builtin_clz(value);
}
# 707 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline int32_t __SSAT(int32_t val, uint32_t sat)
{
  if ((sat >= 1U) && (sat <= 32U))
  {
    const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
    const int32_t min = -1 - max ;
    if (val > max)
    {
      return max;
    }
    else if (val < min)
    {
      return min;
    }
  }
  return val;
}
# 732 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __USAT(int32_t val, uint32_t sat)
{
  if (sat <= 31U)
  {
    const uint32_t max = ((1U << sat) - 1U);
    if (val > (int32_t)max)
    {
      return max;
    }
    else if (val < 0)
    {
      return 0U;
    }
  }
  return (uint32_t)val;
}
# 949 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__((always_inline)) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}







__attribute__((always_inline)) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 1001 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
  __ISB();
}
# 1027 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}







__attribute__((always_inline)) static inline uint32_t __get_PSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, psp" : "=r" (result) );
  return(result);
}
# 1099 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0" : : "r" (topOfProcStack) : );
}
# 1123 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_MSP(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, msp" : "=r" (result) );
  return(result);
}
# 1153 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0" : : "r" (topOfMainStack) : );
}
# 1204 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 1234 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 1588 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
__attribute__((always_inline)) static inline uint32_t __get_FPSCR(void)
{
# 1604 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
  return(0U);

}







__attribute__((always_inline)) static inline void __set_FPSCR(uint32_t fpscr)
{
# 1627 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
  (void)fpscr;

}
# 2209 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_gcc.h"
#pragma GCC diagnostic pop
# 55 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/cmsis_compiler.h" 2
# 163 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h" 2
# 264 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t _reserved0:16;
    uint32_t GE:4;
    uint32_t _reserved1:7;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;
# 303 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;
# 321 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:1;
    uint32_t ICI_IT_1:6;
    uint32_t GE:4;
    uint32_t _reserved1:4;
    uint32_t T:1;
    uint32_t ICI_IT_2:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;
# 376 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 411 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile uint32_t ISER[8U];
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];
        uint32_t RESERVED1[24U];
  volatile uint32_t ISPR[8U];
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];
        uint32_t RESERVED4[56U];
  volatile uint8_t IP[240U];
        uint32_t RESERVED5[644U];
  volatile uint32_t STIR;
} NVIC_Type;
# 445 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t SHP[12U];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile const uint32_t PFR[2U];
  volatile const uint32_t DFR;
  volatile const uint32_t ADR;
  volatile const uint32_t MMFR[4U];
  volatile const uint32_t ISAR[5U];
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;
} SCB_Type;
# 724 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const uint32_t ICTR;
  volatile uint32_t ACTLR;
} SCnSCB_Type;
# 764 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 816 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile union
  {
    volatile uint8_t u8;
    volatile uint16_t u16;
    volatile uint32_t u32;
  } PORT [32U];
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;
        uint32_t RESERVED3[32U];
        uint32_t RESERVED4[43U];
  volatile uint32_t LAR;
  volatile const uint32_t LSR;
        uint32_t RESERVED5[6U];
  volatile const uint32_t PID4;
  volatile const uint32_t PID5;
  volatile const uint32_t PID6;
  volatile const uint32_t PID7;
  volatile const uint32_t PID0;
  volatile const uint32_t PID1;
  volatile const uint32_t PID2;
  volatile const uint32_t PID3;
  volatile const uint32_t CID0;
  volatile const uint32_t CID1;
  volatile const uint32_t CID2;
  volatile const uint32_t CID3;
} ITM_Type;
# 904 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
  volatile uint32_t MASK0;
  volatile uint32_t FUNCTION0;
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;
  volatile uint32_t MASK1;
  volatile uint32_t FUNCTION1;
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;
  volatile uint32_t MASK2;
  volatile uint32_t FUNCTION2;
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;
  volatile uint32_t MASK3;
  volatile uint32_t FUNCTION3;
} DWT_Type;
# 1051 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile const uint32_t SSPSR;
  volatile uint32_t CSPSR;
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;
        uint32_t RESERVED2[131U];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile const uint32_t FSCR;
        uint32_t RESERVED3[759U];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t FIFO0;
  volatile const uint32_t ITATBCTR2;
        uint32_t RESERVED4[1U];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t FIFO1;
  volatile uint32_t ITCTRL;
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
        uint32_t RESERVED7[8U];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPI_Type;
# 1213 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RASR;
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
} MPU_Type;
# 1309 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;
  volatile uint32_t FPCAR;
  volatile uint32_t FPDSCR;
  volatile const uint32_t MVFR0;
  volatile const uint32_t MVFR1;
  volatile const uint32_t MVFR2;
} FPU_Type;
# 1421 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
} CoreDebug_Type;
# 1653 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);

  reg_value = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));
  reg_value = (reg_value |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U) );
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = reg_value;
}







static inline uint32_t __NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}
# 1684 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __asm volatile("":::"memory");
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __asm volatile("":::"memory");
  }
}
# 1703 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t __NVIC_GetEnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1722 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_DisableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __DSB();
    __ISB();
  }
}
# 1741 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t __NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1760 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1775 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}
# 1792 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t __NVIC_GetActive(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
  }
  else
  {
    return(0U);
  }
}
# 1814 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) >= 0)
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}
# 1836 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t __NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) >= 0)
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)IRQn)] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
}
# 1861 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority & (uint32_t)((1UL << (SubPriorityBits )) - 1UL)))
         );
}
# 1888 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority = (Priority ) & (uint32_t)((1UL << (SubPriorityBits )) - 1UL);
}
# 1911 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline void __NVIC_SetVector(IRQn_Type IRQn, uint32_t vector)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  vectors[(int32_t)IRQn + 16] = vector;

}
# 1927 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t __NVIC_GetVector(IRQn_Type IRQn)
{
  uint32_t *vectors = (uint32_t *)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->VTOR;
  return vectors[(int32_t)IRQn + 16];
}






__attribute__((__noreturn__)) static inline void __NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = (uint32_t)((0x5FAUL << 16U) |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U) );
  __DSB();

  for(;;)
  {
    __asm volatile ("nop");
  }
}
# 1960 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/mpu_armv7.h" 1
# 183 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/mpu_armv7.h"
typedef struct {
  uint32_t RBAR;
  uint32_t RASR;
} ARM_MPU_Region_t;




static inline void ARM_MPU_Enable(uint32_t MPU_Control)
{
  __DMB();
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL = MPU_Control | (1UL );

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);

  __DSB();
  __ISB();
}



static inline void ARM_MPU_Disable(void)
{
  __DMB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);

  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL &= ~(1UL );
  __DSB();
  __ISB();
}




static inline void ARM_MPU_ClrRegion(uint32_t rnr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = 0U;
}





static inline void ARM_MPU_SetRegion(uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}






static inline void ARM_MPU_SetRegionEx(uint32_t rnr, uint32_t rbar, uint32_t rasr)
{
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR = rnr;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR = rbar;
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR = rasr;
}






static inline void ARM_MPU_OrderedMemcpy(volatile uint32_t* dst, const uint32_t* 
# 251 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/mpu_armv7.h" 3 4
                                                                                  restrict 
# 251 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/mpu_armv7.h"
                                                                                             src, uint32_t len)
{
  uint32_t i;
  for (i = 0U; i < len; ++i)
  {
    dst[i] = src[i];
  }
}





static inline void ARM_MPU_Load(ARM_MPU_Region_t const* table, uint32_t cnt)
{
  const uint32_t rowWordSize = sizeof(ARM_MPU_Region_t)/4U;
  while (cnt > 4U) {
    ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), 4U*rowWordSize);
    table += 4U;
    cnt -= 4U;
  }
  ARM_MPU_OrderedMemcpy(&(((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR), &(table->RBAR), cnt*rowWordSize);
}
# 1961 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h" 2
# 1981 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((FPU_Type *) ((0xE000E000UL) + 0x0F30UL) )->MVFR0;
  if ((mvfr0 & ((0xFUL << 4U) | (0xFUL << 8U))) == 0x020U)
  {
    return 1U;
  }
  else
  {
    return 0U;
  }
}
# 2022 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (uint32_t)(ticks - 1UL);
  __NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0UL;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2U) |
                   (1UL << 1U) |
                   (1UL );
  return (0UL);
}
# 2052 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
extern volatile int32_t ITM_RxBuffer;
# 2064 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL ) != 0UL) )
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __asm volatile ("nop");
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}
# 2085 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;

  if (ITM_RxBuffer != ((int32_t)0x5AA55AA5U))
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = ((int32_t)0x5AA55AA5U);
  }

  return (ch);
}
# 2105 "/home/ttwards/zephyrproject/modules/hal/cmsis/CMSIS/Core/Include/core_cm4.h"
static inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == ((int32_t)0x5AA55AA5U))
  {
    return (0);
  }
  else
  {
    return (1);
  }
}
# 167 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f407xx.h" 2
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/system_stm32f4xx.h" 1
# 57 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/system_stm32f4xx.h"
extern uint32_t SystemCoreClock;

extern const uint8_t AHBPrescTable[16];
extern const uint8_t APBPrescTable[8];
# 86 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/system_stm32f4xx.h"
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
# 168 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f407xx.h" 2
# 178 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f407xx.h"
typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;
  volatile uint32_t CCR;
  volatile uint32_t CDR;

} ADC_Common_TypeDef;






typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;





typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;





typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;





typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t RESERVED2;
  volatile uint32_t FS1R;
  uint32_t RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t RESERVED4;
  volatile uint32_t FA1R;
  uint32_t RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[28];
} CAN_TypeDef;





typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t IDR;
  uint8_t RESERVED0;
  uint16_t RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;
  volatile uint32_t SR;
} DAC_TypeDef;





typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;
  volatile uint32_t APB1FZ;
  volatile uint32_t APB2FZ;
} DBGMCU_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t RISR;
  volatile uint32_t IER;
  volatile uint32_t MISR;
  volatile uint32_t ICR;
  volatile uint32_t ESCR;
  volatile uint32_t ESUR;
  volatile uint32_t CWSTRTR;
  volatile uint32_t CWSIZER;
  volatile uint32_t DR;
} DCMI_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t NDTR;
  volatile uint32_t PAR;
  volatile uint32_t M0AR;
  volatile uint32_t M1AR;
  volatile uint32_t FCR;
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;
  volatile uint32_t HISR;
  volatile uint32_t LIFCR;
  volatile uint32_t HIFCR;
} DMA_TypeDef;





typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;
  uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;
  volatile uint32_t MACPMTCSR;
  uint32_t RESERVED1;
  volatile uint32_t MACDBGR;
  volatile uint32_t MACSR;
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;
  uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;
  uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;
  volatile uint32_t MMCTGFMSCCR;
  uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;





typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;





typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t OPTCR;
  volatile uint32_t OPTCR1;
} FLASH_TypeDef;







typedef struct
{
  volatile uint32_t BTCR[8];
} FSMC_Bank1_TypeDef;





typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;





typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t RESERVED0;
  volatile uint32_t ECCR2;
  uint32_t RESERVED1;
  uint32_t RESERVED2;
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t RESERVED3;
  volatile uint32_t ECCR3;
} FSMC_Bank2_3_TypeDef;





typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4;
} FSMC_Bank4_TypeDef;





typedef struct
{
  volatile uint32_t MODER;
  volatile uint32_t OTYPER;
  volatile uint32_t OSPEEDR;
  volatile uint32_t PUPDR;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t LCKR;
  volatile uint32_t AFR[2];
} GPIO_TypeDef;





typedef struct
{
  volatile uint32_t MEMRMP;
  volatile uint32_t PMC;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED[2];
  volatile uint32_t CMPCR;
} SYSCFG_TypeDef;





typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t OAR1;
  volatile uint32_t OAR2;
  volatile uint32_t DR;
  volatile uint32_t SR1;
  volatile uint32_t SR2;
  volatile uint32_t CCR;
  volatile uint32_t TRISE;
} I2C_TypeDef;





typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;






typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t PLLCFGR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t AHB1RSTR;
  volatile uint32_t AHB2RSTR;
  volatile uint32_t AHB3RSTR;
  uint32_t RESERVED0;
  volatile uint32_t APB1RSTR;
  volatile uint32_t APB2RSTR;
  uint32_t RESERVED1[2];
  volatile uint32_t AHB1ENR;
  volatile uint32_t AHB2ENR;
  volatile uint32_t AHB3ENR;
  uint32_t RESERVED2;
  volatile uint32_t APB1ENR;
  volatile uint32_t APB2ENR;
  uint32_t RESERVED3[2];
  volatile uint32_t AHB1LPENR;
  volatile uint32_t AHB2LPENR;
  volatile uint32_t AHB3LPENR;
  uint32_t RESERVED4;
  volatile uint32_t APB1LPENR;
  volatile uint32_t APB2LPENR;
  uint32_t RESERVED5[2];
  volatile uint32_t BDCR;
  volatile uint32_t CSR;
  uint32_t RESERVED6[2];
  volatile uint32_t SSCGR;
  volatile uint32_t PLLI2SCFGR;
} RCC_TypeDef;





typedef struct
{
  volatile uint32_t TR;
  volatile uint32_t DR;
  volatile uint32_t CR;
  volatile uint32_t ISR;
  volatile uint32_t PRER;
  volatile uint32_t WUTR;
  volatile uint32_t CALIBR;
  volatile uint32_t ALRMAR;
  volatile uint32_t ALRMBR;
  volatile uint32_t WPR;
  volatile uint32_t SSR;
  volatile uint32_t SHIFTR;
  volatile uint32_t TSTR;
  volatile uint32_t TSDR;
  volatile uint32_t TSSSR;
  volatile uint32_t CALR;
  volatile uint32_t TAFCR;
  volatile uint32_t ALRMASSR;
  volatile uint32_t ALRMBSSR;
  uint32_t RESERVED7;
  volatile uint32_t BKP0R;
  volatile uint32_t BKP1R;
  volatile uint32_t BKP2R;
  volatile uint32_t BKP3R;
  volatile uint32_t BKP4R;
  volatile uint32_t BKP5R;
  volatile uint32_t BKP6R;
  volatile uint32_t BKP7R;
  volatile uint32_t BKP8R;
  volatile uint32_t BKP9R;
  volatile uint32_t BKP10R;
  volatile uint32_t BKP11R;
  volatile uint32_t BKP12R;
  volatile uint32_t BKP13R;
  volatile uint32_t BKP14R;
  volatile uint32_t BKP15R;
  volatile uint32_t BKP16R;
  volatile uint32_t BKP17R;
  volatile uint32_t BKP18R;
  volatile uint32_t BKP19R;
} RTC_TypeDef;





typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;





typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t CRCPR;
  volatile uint32_t RXCRCR;
  volatile uint32_t TXCRCR;
  volatile uint32_t I2SCFGR;
  volatile uint32_t I2SPR;
} SPI_TypeDef;






typedef struct
{
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMCR;
  volatile uint32_t DIER;
  volatile uint32_t SR;
  volatile uint32_t EGR;
  volatile uint32_t CCMR1;
  volatile uint32_t CCMR2;
  volatile uint32_t CCER;
  volatile uint32_t CNT;
  volatile uint32_t PSC;
  volatile uint32_t ARR;
  volatile uint32_t RCR;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t BDTR;
  volatile uint32_t DCR;
  volatile uint32_t DMAR;
  volatile uint32_t OR;
} TIM_TypeDef;





typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t DR;
  volatile uint32_t BRR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t CR3;
  volatile uint32_t GTPR;
} USART_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;





typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SR;
  volatile uint32_t DR;
} RNG_TypeDef;




typedef struct
{
  volatile uint32_t GOTGCTL;
  volatile uint32_t GOTGINT;
  volatile uint32_t GAHBCFG;
  volatile uint32_t GUSBCFG;
  volatile uint32_t GRSTCTL;
  volatile uint32_t GINTSTS;
  volatile uint32_t GINTMSK;
  volatile uint32_t GRXSTSR;
  volatile uint32_t GRXSTSP;
  volatile uint32_t GRXFSIZ;
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;
  volatile uint32_t HNPTXSTS;
  uint32_t Reserved30[2];
  volatile uint32_t GCCFG;
  volatile uint32_t CID;
  uint32_t Reserved40[48];
  volatile uint32_t HPTXFSIZ;
  volatile uint32_t DIEPTXF[0x0F];
} USB_OTG_GlobalTypeDef;




typedef struct
{
  volatile uint32_t DCFG;
  volatile uint32_t DCTL;
  volatile uint32_t DSTS;
  uint32_t Reserved0C;
  volatile uint32_t DIEPMSK;
  volatile uint32_t DOEPMSK;
  volatile uint32_t DAINT;
  volatile uint32_t DAINTMSK;
  uint32_t Reserved20;
  uint32_t Reserved9;
  volatile uint32_t DVBUSDIS;
  volatile uint32_t DVBUSPULSE;
  volatile uint32_t DTHRCTL;
  volatile uint32_t DIEPEMPMSK;
  volatile uint32_t DEACHINT;
  volatile uint32_t DEACHMSK;
  uint32_t Reserved40;
  volatile uint32_t DINEP1MSK;
  uint32_t Reserved44[15];
  volatile uint32_t DOUTEP1MSK;
} USB_OTG_DeviceTypeDef;




typedef struct
{
  volatile uint32_t DIEPCTL;
  uint32_t Reserved04;
  volatile uint32_t DIEPINT;
  uint32_t Reserved0C;
  volatile uint32_t DIEPTSIZ;
  volatile uint32_t DIEPDMA;
  volatile uint32_t DTXFSTS;
  uint32_t Reserved18;
} USB_OTG_INEndpointTypeDef;




typedef struct
{
  volatile uint32_t DOEPCTL;
  uint32_t Reserved04;
  volatile uint32_t DOEPINT;
  uint32_t Reserved0C;
  volatile uint32_t DOEPTSIZ;
  volatile uint32_t DOEPDMA;
  uint32_t Reserved18[2];
} USB_OTG_OUTEndpointTypeDef;




typedef struct
{
  volatile uint32_t HCFG;
  volatile uint32_t HFIR;
  volatile uint32_t HFNUM;
  uint32_t Reserved40C;
  volatile uint32_t HPTXSTS;
  volatile uint32_t HAINT;
  volatile uint32_t HAINTMSK;
} USB_OTG_HostTypeDef;




typedef struct
{
  volatile uint32_t HCCHAR;
  volatile uint32_t HCSPLT;
  volatile uint32_t HCINT;
  volatile uint32_t HCINTMSK;
  volatile uint32_t HCTSIZ;
  volatile uint32_t HCDMA;
  uint32_t Reserved[2];
} USB_OTG_HostChannelTypeDef;
# 133 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h" 2
# 184 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h"
typedef enum
{
  RESET = 0U,
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum
{
  DISABLE = 0U,
  ENABLE = !DISABLE
} FunctionalState;


typedef enum
{
  SUCCESS = 0U,
  ERROR = !SUCCESS
} ErrorStatus;
# 287 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h" 1
# 29 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 1
# 280 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h" 1
# 27 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_def.h" 1
# 29 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_def.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h" 1
# 30 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_def.h" 2
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/Legacy/stm32_hal_legacy.h" 1
# 31 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_def.h" 2
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 32 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_def.h" 2






typedef enum
{
  HAL_OK = 0x00U,
  HAL_ERROR = 0x01U,
  HAL_BUSY = 0x02U,
  HAL_TIMEOUT = 0x03U
} HAL_StatusTypeDef;




typedef enum
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED = 0x01U
} HAL_LockTypeDef;
# 28 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc_ex.h" 1
# 45 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc_ex.h"
typedef struct
{
  uint32_t PLLState;


  uint32_t PLLSource;


  uint32_t PLLM;


  uint32_t PLLN;



  uint32_t PLLP;


  uint32_t PLLQ;
# 73 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc_ex.h"
}RCC_PLLInitTypeDef;
# 382 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc_ex.h"
typedef struct
{





  uint32_t PLLI2SN;




  uint32_t PLLI2SR;



}RCC_PLLI2SInitTypeDef;




typedef struct
{
  uint32_t PeriphClockSelection;


  RCC_PLLI2SInitTypeDef PLLI2S;


  uint32_t RTCClockSelection;





}RCC_PeriphCLKInitTypeDef;
# 6806 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc_ex.h"
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit);

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);
# 6818 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc_ex.h"
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef *PLLI2SInit);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void);
# 32 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h" 2
# 49 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h"
typedef struct
{
  uint32_t OscillatorType;


  uint32_t HSEState;


  uint32_t LSEState;


  uint32_t HSIState;


  uint32_t HSICalibrationValue;


  uint32_t LSIState;


  RCC_PLLInitTypeDef PLL;
}RCC_OscInitTypeDef;




typedef struct
{
  uint32_t ClockType;


  uint32_t SYSCLKSource;


  uint32_t AHBCLKDivider;


  uint32_t APB1CLKDivider;


  uint32_t APB2CLKDivider;


}RCC_ClkInitTypeDef;
# 1242 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h"
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);
# 1253 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rcc.h"
void HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void HAL_RCC_EnableCSS(void);
void HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);


void HAL_RCC_NMI_IRQHandler(void);


void HAL_RCC_CSSCallback(void);
# 281 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio.h"
typedef struct
{
  uint32_t Pin;


  uint32_t Mode;


  uint32_t Pull;


  uint32_t Speed;


  uint32_t Alternate;

}GPIO_InitTypeDef;




typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;
# 213 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio_ex.h" 1
# 214 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio.h" 2
# 224 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio.h"
void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint32_t GPIO_Pin);
# 234 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_gpio.h"
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
# 285 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_exti.h" 1
# 44 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_exti.h"
typedef enum
{
  HAL_EXTI_COMMON_CB_ID = 0x00U
} EXTI_CallbackIDTypeDef;




typedef struct
{
  uint32_t Line;
  void (* PendingCallback)(void);
} EXTI_HandleTypeDef;




typedef struct
{
  uint32_t Line;

  uint32_t Mode;

  uint32_t Trigger;

  uint32_t GPIOSel;


} EXTI_ConfigTypeDef;
# 326 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_exti.h"
HAL_StatusTypeDef HAL_EXTI_SetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_GetConfigLine(EXTI_HandleTypeDef *hexti, EXTI_ConfigTypeDef *pExtiConfig);
HAL_StatusTypeDef HAL_EXTI_ClearConfigLine(EXTI_HandleTypeDef *hexti);
HAL_StatusTypeDef HAL_EXTI_RegisterCallback(EXTI_HandleTypeDef *hexti, EXTI_CallbackIDTypeDef CallbackID, void (*pPendingCbfn)(void));
HAL_StatusTypeDef HAL_EXTI_GetHandle(EXTI_HandleTypeDef *hexti, uint32_t ExtiLine);
# 340 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_exti.h"
void HAL_EXTI_IRQHandler(EXTI_HandleTypeDef *hexti);
uint32_t HAL_EXTI_GetPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_ClearPending(EXTI_HandleTypeDef *hexti, uint32_t Edge);
void HAL_EXTI_GenerateSWI(EXTI_HandleTypeDef *hexti);
# 289 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h" 1
# 48 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h"
typedef struct
{
  uint32_t Channel;


  uint32_t Direction;



  uint32_t PeriphInc;


  uint32_t MemInc;


  uint32_t PeriphDataAlignment;


  uint32_t MemDataAlignment;


  uint32_t Mode;




  uint32_t Priority;


  uint32_t FIFOMode;




  uint32_t FIFOThreshold;


  uint32_t MemBurst;





  uint32_t PeriphBurst;




}DMA_InitTypeDef;





typedef enum
{
  HAL_DMA_STATE_RESET = 0x00U,
  HAL_DMA_STATE_READY = 0x01U,
  HAL_DMA_STATE_BUSY = 0x02U,
  HAL_DMA_STATE_TIMEOUT = 0x03U,
  HAL_DMA_STATE_ERROR = 0x04U,
  HAL_DMA_STATE_ABORT = 0x05U,
}HAL_DMA_StateTypeDef;




typedef enum
{
  HAL_DMA_FULL_TRANSFER = 0x00U,
  HAL_DMA_HALF_TRANSFER = 0x01U
}HAL_DMA_LevelCompleteTypeDef;




typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID = 0x00U,
  HAL_DMA_XFER_HALFCPLT_CB_ID = 0x01U,
  HAL_DMA_XFER_M1CPLT_CB_ID = 0x02U,
  HAL_DMA_XFER_M1HALFCPLT_CB_ID = 0x03U,
  HAL_DMA_XFER_ERROR_CB_ID = 0x04U,
  HAL_DMA_XFER_ABORT_CB_ID = 0x05U,
  HAL_DMA_XFER_ALL_CB_ID = 0x06U
}HAL_DMA_CallbackIDTypeDef;




typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef *Instance;

  DMA_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_DMA_StateTypeDef State;

  void *Parent;

  void (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);

  void (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);

  volatile uint32_t ErrorCode;

  uint32_t StreamBaseAddress;

  uint32_t StreamIndex;

}DMA_HandleTypeDef;
# 639 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma_ex.h" 1
# 47 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma_ex.h"
typedef enum
{
  MEMORY0 = 0x00U,
  MEMORY1 = 0x01U
}HAL_DMA_MemoryTypeDef;
# 69 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma_ex.h"
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);
# 640 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h" 2
# 652 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h"
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);
# 662 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h"
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_CleanCallbacks(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);
# 680 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma.h"
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t HAL_DMA_GetError(DMA_HandleTypeDef *hdma);
# 293 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_cortex.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_cortex.h"
typedef struct
{
  uint8_t Enable;

  uint8_t Number;

  uint32_t BaseAddress;
  uint8_t Size;

  uint8_t SubRegionDisable;

  uint8_t TypeExtField;

  uint8_t AccessPermission;

  uint8_t DisableExec;

  uint8_t IsShareable;

  uint8_t IsCacheable;

  uint8_t IsBufferable;

}MPU_Region_InitTypeDef;
# 260 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_cortex.h"
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);
# 274 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_cortex.h"
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);

void HAL_CORTEX_ClearEvent(void);
# 297 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h" 1
# 31 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h" 1
# 320 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
typedef struct
{
  uint32_t CommonClock;





  uint32_t Multimode;




  uint32_t MultiDMATransfer;




  uint32_t MultiTwoSamplingDelay;





} LL_ADC_CommonInitTypeDef;
# 366 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
typedef struct
{
  uint32_t Resolution;




  uint32_t DataAlignment;




  uint32_t SequencersScanMode;




} LL_ADC_InitTypeDef;
# 404 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
typedef struct
{
  uint32_t TriggerSource;






  uint32_t SequencerLength;





  uint32_t SequencerDiscont;






  uint32_t ContinuousMode;





  uint32_t DMATransfer;




} LL_ADC_REG_InitTypeDef;
# 458 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
typedef struct
{
  uint32_t TriggerSource;






  uint32_t SequencerLength;





  uint32_t SequencerDiscont;






  uint32_t TrigAuto;





} LL_ADC_INJ_InitTypeDef;
# 1862 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_DMA_GetRegAddr(ADC_TypeDef *ADCx, uint32_t Register)
{
  uint32_t data_reg_addr = 0UL;

  if (Register == 0x00000000UL)
  {

    data_reg_addr = (uint32_t) & (ADCx->DR);
  }
  else
  {

    data_reg_addr = (uint32_t) & (((((ADC_Common_TypeDef *) ((0x40000000UL + 0x00010000UL) + 0x2300UL))))->CDR);
  }

  return data_reg_addr;
}
# 1910 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t CommonClock)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (16U))))) | (CommonClock))));
}
# 1926 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetCommonClock(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (16U)))));
}
# 1959 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t PathInternal)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1UL << (23U)) | (0x1UL << (22U))))) | (PathInternal))));
}
# 1980 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetCommonPathInternalCh(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1UL << (23U)) | (0x1UL << (22U)))));
}
# 2006 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetResolution(ADC_TypeDef *ADCx, uint32_t Resolution)
{
  (((ADCx->CR1)) = ((((((ADCx->CR1))) & (~((0x3UL << (24U))))) | (Resolution))));
}
# 2023 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetResolution(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR1) & ((0x3UL << (24U)))));
}
# 2039 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetDataAlignment(ADC_TypeDef *ADCx, uint32_t DataAlignment)
{
  (((ADCx->CR2)) = ((((((ADCx->CR2))) & (~((0x1UL << (11U))))) | (DataAlignment))));
}
# 2054 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetDataAlignment(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR2) & ((0x1UL << (11U)))));
}
# 2080 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetSequencersScanMode(ADC_TypeDef *ADCx, uint32_t ScanMode)
{
  (((ADCx->CR1)) = ((((((ADCx->CR1))) & (~((0x1UL << (8U))))) | (ScanMode))));
}
# 2105 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetSequencersScanMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR1) & ((0x1UL << (8U)))));
}
# 2149 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{





  (((ADCx->CR2)) = ((((((ADCx->CR2))) & (~((0xFUL << (24U))))) | ((TriggerSource & (0xFUL << (24U)))))));
}
# 2193 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetTriggerSource(ADC_TypeDef *ADCx)
{
  uint32_t TriggerSource = ((ADCx->CR2) & ((0xFUL << (24U)) | (0x3UL << (28U))));



  uint32_t ShiftExten = ((TriggerSource & (0x3UL << (28U))) >> ((28UL) - 2UL));



  return ((TriggerSource
           & ((((0x00000000UL & (0xFUL << (24U))) >> (4UL * 0UL)) | (((0xFUL << (24U))) >> (4UL * 1UL)) | (((0xFUL << (24U))) >> (4UL * 2UL)) | (((0xFUL << (24U))) >> (4UL * 3UL))) << ShiftExten) & (0xFUL << (24U)))
          | (((((0x00000000UL & (0x3UL << (28U))) >> (4UL * 0UL)) | ((((0x1UL << (28U)))) >> (4UL * 1UL)) | ((((0x1UL << (28U)))) >> (4UL * 2UL)) | ((((0x1UL << (28U)))) >> (4UL * 3UL))) << ShiftExten) & (0x3UL << (28U)))
         );
}
# 2220 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR2) & ((0x3UL << (28U)))) == (0x00000000UL & (0x3UL << (28U))));
}
# 2237 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR2) & ((0x3UL << (28U)))));
}
# 2299 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->SQR1)) = ((((((ADCx->SQR1))) & (~((0xFUL << (20U))))) | (SequencerNbRanks))));
}
# 2359 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->SQR1) & ((0xFUL << (20U)))));
}
# 2387 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CR1)) = ((((((ADCx->CR1))) & (~((0x1UL << (11U)) | (0x7UL << (13U))))) | (SeqDiscont))));
}
# 2410 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR1) & ((0x1UL << (11U)) | (0x7UL << (13U)))));
}
# 2493 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{




  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~(((0x1FUL << (0U))) << (Rank & (0x0000001FU))))) | ((Channel & ((0x1FUL << (0U)))) << (Rank & (0x0000001FU))))))

                                                                                          ;
}
# 2590 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SQR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  return (uint32_t)(((*preg) & (((0x1FUL << (0U))) << (Rank & (0x0000001FU))))

                    >> (Rank & (0x0000001FU))
                   );
}
# 2615 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetContinuousMode(ADC_TypeDef *ADCx, uint32_t Continuous)
{
  (((ADCx->CR2)) = ((((((ADCx->CR2))) & (~((0x1UL << (1U))))) | (Continuous))));
}
# 2632 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetContinuousMode(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR2) & ((0x1UL << (1U)))));
}
# 2668 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetDMATransfer(ADC_TypeDef *ADCx, uint32_t DMATransfer)
{
  (((ADCx->CR2)) = ((((((ADCx->CR2))) & (~((0x1UL << (8U)) | (0x1UL << (9U))))) | (DMATransfer))));
}
# 2703 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetDMATransfer(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR2) & ((0x1UL << (8U)) | (0x1UL << (9U)))));
}
# 2725 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_SetFlagEndOfConversion(ADC_TypeDef *ADCx, uint32_t EocSelection)
{
  (((ADCx->CR2)) = ((((((ADCx->CR2))) & (~((0x1UL << (10U))))) | (EocSelection))));
}
# 2740 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_GetFlagEndOfConversion(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR2) & ((0x1UL << (10U)))));
}
# 2784 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_SetTriggerSource(ADC_TypeDef *ADCx, uint32_t TriggerSource)
{





  (((ADCx->CR2)) = ((((((ADCx->CR2))) & (~((0xFUL << (16U))))) | ((TriggerSource & (0xFUL << (16U)))))));
}
# 2828 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetTriggerSource(ADC_TypeDef *ADCx)
{
  uint32_t TriggerSource = ((ADCx->CR2) & ((0xFUL << (16U)) | (0x3UL << (20U))));



  uint32_t ShiftExten = ((TriggerSource & (0x3UL << (20U))) >> ((20UL) - 2UL));



  return ((TriggerSource
           & ((((0x00000000UL & (0xFUL << (16U))) >> (4UL * 0UL)) | (((0xFUL << (16U))) >> (4UL * 1UL)) | (((0xFUL << (16U))) >> (4UL * 2UL)) | (((0xFUL << (16U))) >> (4UL * 3UL))) << ShiftExten) & (0xFUL << (16U)))
          | (((((0x00000000UL & (0x3UL << (20U))) >> (4UL * 0UL)) | ((((0x1UL << (20U)))) >> (4UL * 1UL)) | ((((0x1UL << (20U)))) >> (4UL * 2UL)) | ((((0x1UL << (20U)))) >> (4UL * 3UL))) << ShiftExten) & (0x3UL << (20U)))
         );
}
# 2855 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_IsTriggerSourceSWStart(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR2) & ((0x3UL << (20U)))) == (0x00000000UL & (0x3UL << (20U))));
}
# 2870 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetTriggerEdge(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR2) & ((0x3UL << (20U)))));
}
# 2898 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_SetSequencerLength(ADC_TypeDef *ADCx, uint32_t SequencerNbRanks)
{
  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~((0x3UL << (20U))))) | (SequencerNbRanks))));
}
# 2925 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetSequencerLength(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->JSQR) & ((0x3UL << (20U)))));
}
# 2943 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_SetSequencerDiscont(ADC_TypeDef *ADCx, uint32_t SeqDiscont)
{
  (((ADCx->CR1)) = ((((((ADCx->CR1))) & (~((0x1UL << (12U))))) | (SeqDiscont))));
}
# 2958 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetSequencerDiscont(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR1) & ((0x1UL << (12U)))));
}
# 3010 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_SetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t Channel)
{




  uint32_t tmpreg1 = (((ADCx->JSQR) & ((0x3UL << (20U)))) >> (20U)) + 1UL;

  (((ADCx->JSQR)) = ((((((ADCx->JSQR))) & (~(((0x1FUL << (0U))) << (5UL * (uint8_t)(((Rank) + 3UL) - (tmpreg1)))))) | ((Channel & ((0x1FUL << (0U)))) << (5UL * (uint8_t)(((Rank) + 3UL) - (tmpreg1)))))))

                                                                                                     ;
}
# 3079 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetSequencerRanks(ADC_TypeDef *ADCx, uint32_t Rank)
{
  uint32_t tmpreg1 = (((ADCx->JSQR) & ((0x3UL << (20U)))) >> (20U)) + 1UL;

  return (uint32_t)(((ADCx->JSQR) & (((0x1FUL << (0U))) << (5UL * (uint8_t)(((Rank) + 3UL) - (tmpreg1)))))

                    >> (5UL * (uint8_t)(((Rank) + 3UL) - (tmpreg1)))
                   );
}
# 3115 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_SetTrigAuto(ADC_TypeDef *ADCx, uint32_t TrigAuto)
{
  (((ADCx->CR1)) = ((((((ADCx->CR1))) & (~((0x1UL << (10U))))) | (TrigAuto))));
}
# 3129 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetTrigAuto(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR1) & ((0x1UL << (10U)))));
}
# 3159 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_SetOffset(ADC_TypeDef *ADCx, uint32_t Rank, uint32_t OffsetLevel)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JOFR1)) + (((((Rank) & ((0x00000000UL | 0x00001000UL | 0x00002000UL | 0x00003000UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00001000UL | 0x00002000UL | 0x00003000UL))))))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0xFFFUL << (0U))))) | (OffsetLevel))))

                         ;
}
# 3186 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_GetOffset(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JOFR1)) + (((((Rank) & ((0x00000000UL | 0x00001000UL | 0x00002000UL | 0x00003000UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00001000UL | 0x00002000UL | 0x00003000UL))))))) << 2UL))));

  return (uint32_t)(((*preg) & ((0xFFFUL << (0U))))

                   );
}
# 3279 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel, uint32_t SamplingTime)
{




  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + (((((Channel) & ((0x00000000UL | 0x02000000UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x02000000UL))))))) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0x7UL << (0U)) << (((Channel) & (0x01F00000UL)) >> (__CLZ(__RBIT((0x01F00000UL)))))))) | (SamplingTime << (((Channel) & (0x01F00000UL)) >> (__CLZ(__RBIT((0x01F00000UL)))))))))

                                                                                          ;
}
# 3356 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetChannelSamplingTime(ADC_TypeDef *ADCx, uint32_t Channel)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->SMPR1)) + (((((Channel) & ((0x00000000UL | 0x02000000UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x02000000UL))))))) << 2UL))));

  return (uint32_t)(((*preg) & ((0x7UL << (0U)) << (((Channel) & (0x01F00000UL)) >> (__CLZ(__RBIT((0x01F00000UL)))))))

                    >> (((Channel) & (0x01F00000UL)) >> (__CLZ(__RBIT((0x01F00000UL)))))
                   );
}
# 3470 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetAnalogWDMonitChannels(ADC_TypeDef *ADCx, uint32_t AWDChannelGroup)
{
  (((ADCx->CR1)) = ((((((ADCx->CR1))) & (~(((0x1UL << (23U)) | (0x1UL << (22U)) | (0x1UL << (9U)) | (0x1FUL << (0U)))))) | (AWDChannelGroup))))

                             ;
}
# 3566 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetAnalogWDMonitChannels(ADC_TypeDef *ADCx)
{
  return (uint32_t)(((ADCx->CR1) & (((0x1UL << (23U)) | (0x1UL << (22U)) | (0x1UL << (9U)) | (0x1FUL << (0U))))));
}
# 3593 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDThresholdsHighLow, uint32_t AWDThresholdValue)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->HTR)) + ((AWDThresholdsHighLow) << 2UL))));

  (((*preg)) = ((((((*preg))) & (~((0xFFFUL << (0U))))) | (AWDThresholdValue))))

                               ;
}
# 3616 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetAnalogWDThresholds(ADC_TypeDef *ADCx, uint32_t AWDThresholdsHighLow)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->HTR)) + ((AWDThresholdsHighLow) << 2UL))));

  return (uint32_t)(((*preg) & ((0xFFFUL << (0U)))));
}
# 3658 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetMultimode(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t Multimode)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x1FUL << (0U))))) | (Multimode))));
}
# 3688 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetMultimode(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x1FUL << (0U)))));
}
# 3737 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiDMATransfer)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0x3UL << (14U)) | (0x1UL << (13U))))) | (MultiDMATransfer))));
}
# 3785 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetMultiDMATransfer(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0x3UL << (14U)) | (0x1UL << (13U)))));
}
# 3819 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_SetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t MultiTwoSamplingDelay)
{
  (((ADCxy_COMMON->CCR)) = ((((((ADCxy_COMMON->CCR))) & (~((0xFUL << (8U))))) | (MultiTwoSamplingDelay))));
}
# 3847 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_GetMultiTwoSamplingDelay(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (uint32_t)(((ADCxy_COMMON->CCR) & ((0xFUL << (8U)))));
}
# 3870 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_Enable(ADC_TypeDef *ADCx)
{
  ((ADCx->CR2) |= ((0x1UL << (0U))));
}







static inline void LL_ADC_Disable(ADC_TypeDef *ADCx)
{
  ((ADCx->CR2) &= ~((0x1UL << (0U))));
}







static inline uint32_t LL_ADC_IsEnabled(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR2) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}
# 3921 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_StartConversionSWStart(ADC_TypeDef *ADCx)
{
  ((ADCx->CR2) |= ((0x1UL << (30U))));
}
# 3942 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_StartConversionExtTrig(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  ((ADCx->CR2) |= (ExternalTriggerEdge));
}
# 3960 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_REG_StopConversionExtTrig(ADC_TypeDef *ADCx)
{
  ((ADCx->CR2) &= ~((0x3UL << (28U))));
}
# 3974 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_ReadConversionData32(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 3989 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint16_t LL_ADC_REG_ReadConversionData12(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 4004 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint16_t LL_ADC_REG_ReadConversionData10(ADC_TypeDef *ADCx)
{
  return (uint16_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 4019 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint8_t LL_ADC_REG_ReadConversionData8(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 4034 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint8_t LL_ADC_REG_ReadConversionData6(ADC_TypeDef *ADCx)
{
  return (uint8_t)(((ADCx->DR) & ((0xFFFFUL << (0U)))));
}
# 4061 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_REG_ReadMultiConversionData32(ADC_Common_TypeDef *ADCxy_COMMON, uint32_t ConversionData)
{
  return (uint32_t)(((ADCxy_COMMON->CDR) & ((0xFFFFUL << (16U))))

                    >> (__CLZ(__RBIT(ConversionData)))
                   );
}
# 4094 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_StartConversionSWStart(ADC_TypeDef *ADCx)
{
  ((ADCx->CR2) |= ((0x1UL << (22U))));
}
# 4115 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_StartConversionExtTrig(ADC_TypeDef *ADCx, uint32_t ExternalTriggerEdge)
{
  ((ADCx->CR2) |= (ExternalTriggerEdge));
}
# 4133 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_INJ_StopConversionExtTrig(ADC_TypeDef *ADCx)
{
  ((ADCx->CR2) &= ~((0x3UL << (20U))));
}
# 4155 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_INJ_ReadConversionData32(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  return (uint32_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}
# 4182 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint16_t LL_ADC_INJ_ReadConversionData12(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}
# 4209 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint16_t LL_ADC_INJ_ReadConversionData10(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  return (uint16_t)(((*preg) & ((0xFFFFUL << (0U))))

                   );
}
# 4236 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint8_t LL_ADC_INJ_ReadConversionData8(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}
# 4263 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint8_t LL_ADC_INJ_ReadConversionData6(ADC_TypeDef *ADCx, uint32_t Rank)
{
  volatile uint32_t *preg = ((volatile uint32_t *)((uint32_t) ((uint32_t)(&(ADCx->JDR1)) + (((((Rank) & ((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))) >> (__CLZ(__RBIT(((0x00000000UL | 0x00000100UL | 0x00000200UL | 0x00000300UL))))))) << 2UL))));

  return (uint8_t)(((*preg) & ((0xFFFFUL << (0U))))

                  );
}
# 4290 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_EOCS(ADC_TypeDef *ADCx)
{
  return (((ADCx->SR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_OVR(ADC_TypeDef *ADCx)
{
  return (((ADCx->SR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 4313 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_JEOS(ADC_TypeDef *ADCx)
{




  return (((ADCx->SR) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_AWD1(ADC_TypeDef *ADCx)
{
  return (((ADCx->SR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}
# 4343 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_ClearFlag_EOCS(ADC_TypeDef *ADCx)
{
  ((ADCx->SR) = (~(0x1UL << (1U))));
}







static inline void LL_ADC_ClearFlag_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->SR) = (~(0x1UL << (5U))));
}
# 4366 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_ClearFlag_JEOS(ADC_TypeDef *ADCx)
{




  ((ADCx->SR) = (~(0x1UL << (2U))));
}







static inline void LL_ADC_ClearFlag_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->SR) = (~(0x1UL << (0U))));
}
# 4398 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_EOCS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (1U)))) == ((0x1UL << (1U))));
}
# 4414 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV1_EOCS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (9U)))) == ((0x1UL << (9U))));
}
# 4430 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV2_EOCS(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (17U)))) == ((0x1UL << (17U))));
}







static inline uint32_t LL_ADC_IsActiveFlag_MST_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 4453 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV1_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (13U)))) == ((0x1UL << (13U))));
}
# 4465 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV2_OVR(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (21U)))) == ((0x1UL << (21U))));
}
# 4478 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{




  return (((ADCxy_COMMON->CSR) & ((0x1UL << (2U)))) == ((0x1UL << (2U))));
}
# 4494 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV1_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{




  return (((ADCxy_COMMON->CSR) & ((0x1UL << (10U)))) == ((0x1UL << (10U))));
}
# 4510 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV2_JEOS(ADC_Common_TypeDef *ADCxy_COMMON)
{




  return (((ADCxy_COMMON->CSR) & ((0x1UL << (18U)))) == ((0x1UL << (18U))));
}
# 4526 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_MST_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (0U)))) == ((0x1UL << (0U))));
}
# 4538 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV1_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (8U)))) == ((0x1UL << (8U))));
}
# 4550 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsActiveFlag_SLV2_AWD1(ADC_Common_TypeDef *ADCxy_COMMON)
{
  return (((ADCxy_COMMON->CSR) & ((0x1UL << (16U)))) == ((0x1UL << (16U))));
}
# 4575 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_EnableIT_EOCS(ADC_TypeDef *ADCx)
{
  ((ADCx->CR1) |= ((0x1UL << (5U))));
}







static inline void LL_ADC_EnableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->CR1) |= ((0x1UL << (26U))));
}
# 4598 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_EnableIT_JEOS(ADC_TypeDef *ADCx)
{




  ((ADCx->CR1) |= ((0x1UL << (7U))));
}







static inline void LL_ADC_EnableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->CR1) |= ((0x1UL << (6U))));
}
# 4628 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_DisableIT_EOCS(ADC_TypeDef *ADCx)
{
  ((ADCx->CR1) &= ~((0x1UL << (5U))));
}







static inline void LL_ADC_DisableIT_OVR(ADC_TypeDef *ADCx)
{
  ((ADCx->CR1) &= ~((0x1UL << (26U))));
}
# 4651 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline void LL_ADC_DisableIT_JEOS(ADC_TypeDef *ADCx)
{




  ((ADCx->CR1) &= ~((0x1UL << (7U))));
}







static inline void LL_ADC_DisableIT_AWD1(ADC_TypeDef *ADCx)
{
  ((ADCx->CR1) &= ~((0x1UL << (6U))));
}
# 4682 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_EOCS(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR1) & ((0x1UL << (5U)))) == ((0x1UL << (5U))));
}
# 4694 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_OVR(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR1) & ((0x1UL << (26U)))) == ((0x1UL << (26U))));
}
# 4707 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_JEOS(ADC_TypeDef *ADCx)
{




  return (((ADCx->CR1) & ((0x1UL << (7U)))) == ((0x1UL << (7U))));
}
# 4723 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
static inline uint32_t LL_ADC_IsEnabledIT_AWD1(ADC_TypeDef *ADCx)
{
  return (((ADCx->CR1) & ((0x1UL << (6U)))) == ((0x1UL << (6U))));
}
# 4738 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_adc.h"
ErrorStatus LL_ADC_CommonDeInit(ADC_Common_TypeDef *ADCxy_COMMON);
ErrorStatus LL_ADC_CommonInit(ADC_Common_TypeDef *ADCxy_COMMON, LL_ADC_CommonInitTypeDef *ADC_CommonInitStruct);
void LL_ADC_CommonStructInit(LL_ADC_CommonInitTypeDef *ADC_CommonInitStruct);



ErrorStatus LL_ADC_DeInit(ADC_TypeDef *ADCx);


ErrorStatus LL_ADC_Init(ADC_TypeDef *ADCx, LL_ADC_InitTypeDef *ADC_InitStruct);
void LL_ADC_StructInit(LL_ADC_InitTypeDef *ADC_InitStruct);


ErrorStatus LL_ADC_REG_Init(ADC_TypeDef *ADCx, LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct);
void LL_ADC_REG_StructInit(LL_ADC_REG_InitTypeDef *ADC_REG_InitStruct);


ErrorStatus LL_ADC_INJ_Init(ADC_TypeDef *ADCx, LL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct);
void LL_ADC_INJ_StructInit(LL_ADC_INJ_InitTypeDef *ADC_INJ_InitStruct);
# 32 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h" 2
# 59 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
typedef struct
{
  uint32_t ClockPrescaler;


  uint32_t Resolution;

  uint32_t DataAlign;


  uint32_t ScanConvMode;






  uint32_t EOCSelection;






  FunctionalState ContinuousConvMode;


  uint32_t NbrOfConversion;


  FunctionalState DiscontinuousConvMode;



  uint32_t NbrOfDiscConversion;


  uint32_t ExternalTrigConv;



  uint32_t ExternalTrigConvEdge;


  FunctionalState DMAContinuousRequests;




} ADC_InitTypeDef;
# 117 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
typedef struct
{
  uint32_t Channel;

  uint32_t Rank;

  uint32_t SamplingTime;
# 132 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
  uint32_t Offset;
} ADC_ChannelConfTypeDef;




typedef struct
{
  uint32_t WatchdogMode;

  uint32_t HighThreshold;

  uint32_t LowThreshold;

  uint32_t Channel;


  FunctionalState ITMode;


  uint32_t WatchdogNumber;
} ADC_AnalogWDGConfTypeDef;
# 195 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
typedef struct

{
  ADC_TypeDef *Instance;

  ADC_InitTypeDef Init;

  volatile uint32_t NbrOfCurrentConversionRank;

  DMA_HandleTypeDef *DMA_Handle;

  HAL_LockTypeDef Lock;

  volatile uint32_t State;

  volatile uint32_t ErrorCode;
# 220 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
} ADC_HandleTypeDef;
# 554 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc_ex.h" 1
# 55 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc_ex.h"
typedef struct
{
  uint32_t InjectedChannel;


  uint32_t InjectedRank;


  uint32_t InjectedSamplingTime;
# 72 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc_ex.h"
  uint32_t InjectedOffset;



  uint32_t InjectedNbrOfConversion;




  FunctionalState InjectedDiscontinuousConvMode;






  FunctionalState AutoInjectedConv;







  uint32_t ExternalTrigInjecConv;







  uint32_t ExternalTrigInjecConvEdge;




} ADC_InjectionConfTypeDef;




typedef struct
{
  uint32_t Mode;

  uint32_t DMAAccessMode;

  uint32_t TwoSamplingDelay;

} ADC_MultiModeTypeDef;
# 267 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc_ex.h"
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc);
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *hadc, uint32_t InjectedRank);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef *hadc);
uint32_t HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef *hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);


HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *hadc, ADC_InjectionConfTypeDef *sConfigInjected);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *hadc, ADC_MultiModeTypeDef *multimode);
# 555 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h" 2
# 565 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc);
# 583 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef *hadc, uint32_t EventType, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef *hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef *hadc);

void HAL_ADC_IRQHandler(ADC_HandleTypeDef *hadc);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef *hadc, uint32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef *hadc);

uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *hadc);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
# 611 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, ADC_ChannelConfTypeDef *sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef *hadc, ADC_AnalogWDGConfTypeDef *AnalogWDGConfig);
# 621 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_adc.h"
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef *hadc);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc);
# 301 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
typedef enum
{
  HAL_CAN_STATE_RESET = 0x00U,
  HAL_CAN_STATE_READY = 0x01U,
  HAL_CAN_STATE_LISTENING = 0x02U,
  HAL_CAN_STATE_SLEEP_PENDING = 0x03U,
  HAL_CAN_STATE_SLEEP_ACTIVE = 0x04U,
  HAL_CAN_STATE_ERROR = 0x05U

} HAL_CAN_StateTypeDef;




typedef struct
{
  uint32_t Prescaler;


  uint32_t Mode;


  uint32_t SyncJumpWidth;



  uint32_t TimeSeg1;


  uint32_t TimeSeg2;


  FunctionalState TimeTriggeredMode;


  FunctionalState AutoBusOff;


  FunctionalState AutoWakeUp;


  FunctionalState AutoRetransmission;


  FunctionalState ReceiveFifoLocked;


  FunctionalState TransmitFifoPriority;


} CAN_InitTypeDef;




typedef struct
{
  uint32_t FilterIdHigh;




  uint32_t FilterIdLow;




  uint32_t FilterMaskIdHigh;





  uint32_t FilterMaskIdLow;





  uint32_t FilterFIFOAssignment;


  uint32_t FilterBank;





  uint32_t FilterMode;


  uint32_t FilterScale;


  uint32_t FilterActivation;


  uint32_t SlaveStartFilterBank;






} CAN_FilterTypeDef;




typedef struct
{
  uint32_t StdId;


  uint32_t ExtId;


  uint32_t IDE;


  uint32_t RTR;


  uint32_t DLC;


  FunctionalState TransmitGlobalTime;





} CAN_TxHeaderTypeDef;




typedef struct
{
  uint32_t StdId;


  uint32_t ExtId;


  uint32_t IDE;


  uint32_t RTR;


  uint32_t DLC;


  uint32_t Timestamp;



  uint32_t FilterMatchIndex;


} CAN_RxHeaderTypeDef;







typedef struct

{
  CAN_TypeDef *Instance;

  CAN_InitTypeDef Init;

  volatile HAL_CAN_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 246 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
} CAN_HandleTypeDef;
# 649 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef *hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef *hcan);
# 671 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *hcan, const CAN_FilterTypeDef *sFilterConfig);
# 683 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Stop(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_RequestSleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsSleepActive(const CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, const CAN_TxHeaderTypeDef *pHeader,
                                       const uint8_t aData[], uint32_t *pTxMailbox);
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_IsTxMessagePending(const CAN_HandleTypeDef *hcan, uint32_t TxMailboxes);
uint32_t HAL_CAN_GetTxTimestamp(const CAN_HandleTypeDef *hcan, uint32_t TxMailbox);
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *hcan, uint32_t RxFifo,
                                       CAN_RxHeaderTypeDef *pHeader, uint8_t aData[]);
uint32_t HAL_CAN_GetRxFifoFillLevel(const CAN_HandleTypeDef *hcan, uint32_t RxFifo);
# 707 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *hcan, uint32_t ActiveITs);
HAL_StatusTypeDef HAL_CAN_DeactivateNotification(CAN_HandleTypeDef *hcan, uint32_t InactiveITs);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *hcan);
# 721 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_SleepCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_WakeUpFromRxMsgCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
# 744 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_can.h"
HAL_CAN_StateTypeDef HAL_CAN_GetState(const CAN_HandleTypeDef *hcan);
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *hcan);
# 305 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2







# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_crc.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_crc.h"
typedef enum
{
  HAL_CRC_STATE_RESET = 0x00U,
  HAL_CRC_STATE_READY = 0x01U,
  HAL_CRC_STATE_BUSY = 0x02U,
  HAL_CRC_STATE_TIMEOUT = 0x03U,
  HAL_CRC_STATE_ERROR = 0x04U
} HAL_CRC_StateTypeDef;





typedef struct
{
  CRC_TypeDef *Instance;

  HAL_LockTypeDef Lock;

  volatile HAL_CRC_StateTypeDef State;

} CRC_HandleTypeDef;
# 138 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_crc.h"
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);
# 150 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_crc.h"
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
# 160 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_crc.h"
HAL_CRC_StateTypeDef HAL_CRC_GetState(const CRC_HandleTypeDef *hcrc);
# 313 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_cryp.h" 1
# 317 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dma2d.h" 1
# 321 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h" 1
# 49 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
typedef enum
{
  HAL_DAC_STATE_RESET = 0x00U,
  HAL_DAC_STATE_READY = 0x01U,
  HAL_DAC_STATE_BUSY = 0x02U,
  HAL_DAC_STATE_TIMEOUT = 0x03U,
  HAL_DAC_STATE_ERROR = 0x04U

} HAL_DAC_StateTypeDef;







typedef struct

{
  DAC_TypeDef *Instance;

  volatile HAL_DAC_StateTypeDef State;

  HAL_LockTypeDef Lock;

  DMA_HandleTypeDef *DMA_Handle1;

  DMA_HandleTypeDef *DMA_Handle2;

  volatile uint32_t ErrorCode;
# 96 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
} DAC_HandleTypeDef;




typedef struct
{
  uint32_t DAC_Trigger;


  uint32_t DAC_OutputBuffer;


} DAC_ChannelConfTypeDef;
# 380 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac_ex.h" 1
# 147 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac_ex.h"
HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Amplitude);


HAL_StatusTypeDef HAL_DACEx_DualStart(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DACEx_DualStop(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef *hdac, uint32_t Alignment, uint32_t Data1, uint32_t Data2);
uint32_t HAL_DACEx_DualGetValue(const DAC_HandleTypeDef *hdac);



void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac);
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac);
void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef *hdac);
void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef *hdac);
# 182 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac_ex.h"
void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *hdma);
# 381 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h" 2
# 392 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef *hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef *hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef *hdac);
# 405 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel, const uint32_t *pData, uint32_t Length,
                                    uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef *hdac, uint32_t Channel);
void HAL_DAC_IRQHandler(DAC_HandleTypeDef *hdac);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef *hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);
# 433 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
uint32_t HAL_DAC_GetValue(const DAC_HandleTypeDef *hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef *hdac,
                                        const DAC_ChannelConfTypeDef *sConfig, uint32_t Channel);
# 444 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
HAL_DAC_StateTypeDef HAL_DAC_GetState(const DAC_HandleTypeDef *hdac);
uint32_t HAL_DAC_GetError(const DAC_HandleTypeDef *hdac);
# 458 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dac.h"
void DAC_DMAConvCpltCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh1(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh1(DMA_HandleTypeDef *hdma);
# 325 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h" 1
# 34 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi_ex.h" 1
# 50 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi_ex.h"
typedef struct
{
  uint8_t FrameStartCode;
  uint8_t LineStartCode;
  uint8_t LineEndCode;
  uint8_t FrameEndCode;
}DCMI_CodesInitTypeDef;




typedef struct
{
  uint32_t SynchroMode;


  uint32_t PCKPolarity;


  uint32_t VSPolarity;


  uint32_t HSPolarity;


  uint32_t CaptureRate;


  uint32_t ExtendedDataMode;


  DCMI_CodesInitTypeDef SyncroCode;

  uint32_t JPEGMode;
# 99 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi_ex.h"
}DCMI_InitTypeDef;
# 35 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h" 2
# 52 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
typedef struct
{
  uint8_t FrameStartUnmask;
  uint8_t LineStartUnmask;
  uint8_t LineEndUnmask;
  uint8_t FrameEndUnmask;
}DCMI_SyncUnmaskTypeDef;



typedef enum
{
  HAL_DCMI_STATE_RESET = 0x00U,
  HAL_DCMI_STATE_READY = 0x01U,
  HAL_DCMI_STATE_BUSY = 0x02U,
  HAL_DCMI_STATE_TIMEOUT = 0x03U,
  HAL_DCMI_STATE_ERROR = 0x04U,
  HAL_DCMI_STATE_SUSPENDED = 0x05U
}HAL_DCMI_StateTypeDef;




typedef struct __DCMI_HandleTypeDef
{
  DCMI_TypeDef *Instance;

  DCMI_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_DCMI_StateTypeDef State;

  volatile uint32_t XferCount;

  volatile uint32_t XferSize;

  uint32_t XferTransferNumber;

  uint32_t pBuffPtr;

  DMA_HandleTypeDef *DMA_Handle;

  volatile uint32_t ErrorCode;
# 104 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
}DCMI_HandleTypeDef;
# 429 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
HAL_StatusTypeDef HAL_DCMI_Init(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef HAL_DCMI_DeInit(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi);
void HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi);
# 447 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
HAL_StatusTypeDef HAL_DCMI_Start_DMA(DCMI_HandleTypeDef* hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length);
HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef* hdcmi);
HAL_StatusTypeDef HAL_DCMI_Suspend(DCMI_HandleTypeDef* hdcmi);
HAL_StatusTypeDef HAL_DCMI_Resume(DCMI_HandleTypeDef* hdcmi);
void HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_VsyncCallback(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_HsyncCallback(DCMI_HandleTypeDef *hdcmi);
void HAL_DCMI_IRQHandler(DCMI_HandleTypeDef *hdcmi);
# 466 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
HAL_StatusTypeDef HAL_DCMI_ConfigCrop(DCMI_HandleTypeDef *hdcmi, uint32_t X0, uint32_t Y0, uint32_t XSize, uint32_t YSize);
HAL_StatusTypeDef HAL_DCMI_EnableCrop(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef HAL_DCMI_DisableCrop(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef HAL_DCMI_ConfigSyncUnmask(DCMI_HandleTypeDef *hdcmi, DCMI_SyncUnmaskTypeDef *SyncUnmask);
# 478 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dcmi.h"
HAL_DCMI_StateTypeDef HAL_DCMI_GetState(DCMI_HandleTypeDef *hdcmi);
uint32_t HAL_DCMI_GetError(DCMI_HandleTypeDef *hdcmi);
# 329 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h" 1
# 59 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
typedef struct
{
  volatile uint32_t DESC0;
  volatile uint32_t DESC1;
  volatile uint32_t DESC2;
  volatile uint32_t DESC3;
  volatile uint32_t DESC4;
  volatile uint32_t DESC5;
  volatile uint32_t DESC6;
  volatile uint32_t DESC7;
  uint32_t BackupAddr0;
  uint32_t BackupAddr1;
} ETH_DMADescTypeDef;







typedef struct __ETH_BufferTypeDef
{
  uint8_t *buffer;

  uint32_t len;

  struct __ETH_BufferTypeDef *next;
} ETH_BufferTypeDef;







typedef struct
{
  uint32_t TxDesc[4U];

  uint32_t CurTxDesc;

  uint32_t *PacketAddress[4U];

  uint32_t *CurrentPacketAddress;

  uint32_t BuffersInUse;

  uint32_t releaseIndex;
} ETH_TxDescListTypeDef;







typedef struct
{
  uint32_t Attributes;


  uint32_t Length;

  ETH_BufferTypeDef *TxBuffer;

  uint32_t SrcAddrCtrl;


  uint32_t CRCPadCtrl;


  uint32_t ChecksumCtrl;


  uint32_t MaxSegmentSize;


  uint32_t PayloadLen;


  uint32_t TCPHeaderLen;


  uint32_t VlanTag;


  uint32_t VlanCtrl;


  uint32_t InnerVlanTag;


  uint32_t InnerVlanCtrl;


  void *pData;

} ETH_TxPacketConfig_t;







typedef struct
{
  uint32_t TimeStampLow;
  uint32_t TimeStampHigh;

} ETH_TimeStampTypeDef;
# 191 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
typedef struct
{
  uint32_t RxDesc[4U];

  uint32_t ItMode;


  uint32_t RxDescIdx;

  uint32_t RxDescCnt;

  uint32_t RxDataLength;

  uint32_t RxBuildDescIdx;

  uint32_t RxBuildDescCnt;

  uint32_t pRxLastRxDesc;

  ETH_TimeStampTypeDef TimeStamp;

  void *pRxStart;

  void *pRxEnd;

} ETH_RxDescListTypeDef;







typedef struct
{
  uint32_t
  SourceAddrControl;


  FunctionalState
  ChecksumOffload;

  uint32_t InterPacketGapVal;


  FunctionalState GiantPacketSizeLimitControl;

  FunctionalState Support2KPacket;

  FunctionalState CRCStripTypePacket;

  FunctionalState AutomaticPadCRCStrip;

  FunctionalState Watchdog;

  FunctionalState Jabber;

  FunctionalState JumboPacket;



  uint32_t Speed;


  uint32_t DuplexMode;


  FunctionalState LoopbackMode;

  FunctionalState
  CarrierSenseBeforeTransmit;

  FunctionalState ReceiveOwn;

  FunctionalState
  CarrierSenseDuringTransmit;

  FunctionalState
  RetryTransmission;

  uint32_t BackOffLimit;


  FunctionalState
  DeferralCheck;

  uint32_t
  PreambleLength;


  FunctionalState SlowProtocolDetect;

  FunctionalState CRCCheckingRxPackets;

  uint32_t
  GiantPacketSizeLimit;




  FunctionalState ExtendedInterPacketGap;

  uint32_t ExtendedInterPacketGapVal;


  FunctionalState ProgrammableWatchdog;

  uint32_t WatchdogTimeout;


  uint32_t
  PauseTime;



  FunctionalState
  ZeroQuantaPause;

  uint32_t
  PauseLowThreshold;


  FunctionalState
  TransmitFlowControl;


  FunctionalState
  UnicastPausePacketDetect;

  FunctionalState ReceiveFlowControl;


  uint32_t TransmitQueueMode;


  uint32_t ReceiveQueueMode;


  FunctionalState DropTCPIPChecksumErrorPacket;

  FunctionalState ForwardRxErrorPacket;

  FunctionalState ForwardRxUndersizedGoodPacket;
} ETH_MACConfigTypeDef;







typedef struct
{
  uint32_t DMAArbitration;


  FunctionalState AddressAlignedBeats;


  uint32_t BurstMode;

  FunctionalState DropTCPIPChecksumErrorFrame;

  FunctionalState ReceiveStoreForward;

  FunctionalState TransmitStoreForward;


  uint32_t
  TxDMABurstLength;


  uint32_t TransmitThresholdControl;



  uint32_t
  RxDMABurstLength;


  FunctionalState ForwardErrorFrames;
  FunctionalState FlushRxPacket;

  FunctionalState
  ForwardUndersizedGoodFrames;



  uint32_t ReceiveThresholdControl;



  FunctionalState
  SecondFrameOperate;



  FunctionalState EnhancedDescriptorFormat;

  uint32_t
  DescriptorSkipLength;


} ETH_DMAConfigTypeDef;







typedef enum
{
  HAL_ETH_MII_MODE = 0x00U,
  HAL_ETH_RMII_MODE = (0x1UL << (23U))
} ETH_MediaInterfaceTypeDef;
# 428 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
typedef struct
{
  uint8_t
  *MACAddr;

  ETH_MediaInterfaceTypeDef MediaInterface;

  ETH_DMADescTypeDef
  *TxDesc;

  ETH_DMADescTypeDef
  *RxDesc;

  uint32_t RxBuffLen;

} ETH_InitTypeDef;
# 481 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
typedef uint32_t HAL_ETH_StateTypeDef;







typedef void (*pETH_rxAllocateCallbackTypeDef)(uint8_t **buffer);







typedef void (*pETH_rxLinkCallbackTypeDef)(void **pStart, void **pEnd, uint8_t *buff,
                                            uint16_t Length);







typedef void (*pETH_txFreeCallbackTypeDef)(uint32_t *buffer);







typedef void (*pETH_txPtpCallbackTypeDef)(uint32_t *buffer,
                                           ETH_TimeStampTypeDef *timestamp);
# 526 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
typedef struct

{
  ETH_TypeDef *Instance;

  ETH_InitTypeDef Init;

  ETH_TxDescListTypeDef TxDescList;


  ETH_RxDescListTypeDef RxDescList;






  volatile HAL_ETH_StateTypeDef gState;



  volatile uint32_t ErrorCode;


  volatile uint32_t
  DMAErrorCode;



  volatile uint32_t
  MACErrorCode;



  volatile uint32_t MACWakeUpEvent;



  volatile uint32_t MACLPIEvent;


  volatile uint32_t IsPtpConfigured;
# 584 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
  pETH_rxAllocateCallbackTypeDef rxAllocateCallback;
  pETH_rxLinkCallbackTypeDef rxLinkCallback;
  pETH_txFreeCallbackTypeDef txFreeCallback;
  pETH_txPtpCallbackTypeDef txPtpCallback;

} ETH_HandleTypeDef;
# 620 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
typedef struct
{
  FunctionalState PromiscuousMode;

  FunctionalState ReceiveAllMode;

  FunctionalState HachOrPerfectFilter;

  FunctionalState HashUnicast;

  FunctionalState HashMulticast;

  FunctionalState PassAllMulticast;

  FunctionalState SrcAddrFiltering;

  FunctionalState SrcAddrInverseFiltering;

  FunctionalState DestAddrInverseFiltering;

  FunctionalState BroadcastFilter;

  uint32_t ControlPacketsFilter;

} ETH_MACFilterConfigTypeDef;







typedef struct
{
  FunctionalState WakeUpPacket;

  FunctionalState MagicPacket;

  FunctionalState GlobalUnicast;

  FunctionalState WakeUpForward;

} ETH_PowerDownConfigTypeDef;
# 2013 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_DeInit(ETH_HandleTypeDef *heth);
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth);
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth);
# 2033 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
HAL_StatusTypeDef HAL_ETH_Start(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_Start_IT(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_Stop_IT(ETH_HandleTypeDef *heth);

HAL_StatusTypeDef HAL_ETH_ReadData(ETH_HandleTypeDef *heth, void **pAppBuff);
HAL_StatusTypeDef HAL_ETH_RegisterRxAllocateCallback(ETH_HandleTypeDef *heth,
                                                     pETH_rxAllocateCallbackTypeDef rxAllocateCallback);
HAL_StatusTypeDef HAL_ETH_UnRegisterRxAllocateCallback(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_RegisterRxLinkCallback(ETH_HandleTypeDef *heth, pETH_rxLinkCallbackTypeDef rxLinkCallback);
HAL_StatusTypeDef HAL_ETH_UnRegisterRxLinkCallback(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_GetRxDataErrorCode(ETH_HandleTypeDef *heth, uint32_t *pErrorCode);
HAL_StatusTypeDef HAL_ETH_RegisterTxFreeCallback(ETH_HandleTypeDef *heth, pETH_txFreeCallbackTypeDef txFreeCallback);
HAL_StatusTypeDef HAL_ETH_UnRegisterTxFreeCallback(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_ReleaseTxPacket(ETH_HandleTypeDef *heth);
# 2063 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
HAL_StatusTypeDef HAL_ETH_Transmit(ETH_HandleTypeDef *heth, ETH_TxPacketConfig_t *pTxConfig, uint32_t Timeout);
HAL_StatusTypeDef HAL_ETH_Transmit_IT(ETH_HandleTypeDef *heth, ETH_TxPacketConfig_t *pTxConfig);

HAL_StatusTypeDef HAL_ETH_WritePHYRegister(const ETH_HandleTypeDef *heth, uint32_t PHYAddr, uint32_t PHYReg,
                                           uint32_t RegValue);
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint32_t PHYAddr, uint32_t PHYReg,
                                          uint32_t *pRegValue);

void HAL_ETH_IRQHandler(ETH_HandleTypeDef *heth);
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_PMTCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_WakeUpCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_RxAllocateCallback(uint8_t **buff);
void HAL_ETH_RxLinkCallback(void **pStart, void **pEnd, uint8_t *buff, uint16_t Length);
void HAL_ETH_TxFreeCallback(uint32_t *buff);
void HAL_ETH_TxPtpCallback(uint32_t *buff, ETH_TimeStampTypeDef *timestamp);
# 2090 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
HAL_StatusTypeDef HAL_ETH_GetMACConfig(ETH_HandleTypeDef *heth, ETH_MACConfigTypeDef *macconf);
HAL_StatusTypeDef HAL_ETH_GetDMAConfig(ETH_HandleTypeDef *heth, ETH_DMAConfigTypeDef *dmaconf);
HAL_StatusTypeDef HAL_ETH_SetMACConfig(ETH_HandleTypeDef *heth, ETH_MACConfigTypeDef *macconf);
HAL_StatusTypeDef HAL_ETH_SetDMAConfig(ETH_HandleTypeDef *heth, ETH_DMAConfigTypeDef *dmaconf);
void HAL_ETH_SetMDIOClockRange(ETH_HandleTypeDef *heth);


void HAL_ETH_SetRxVLANIdentifier(ETH_HandleTypeDef *heth, uint32_t ComparisonBits,
                                              uint32_t VLANIdentifier);


HAL_StatusTypeDef HAL_ETH_GetMACFilterConfig(ETH_HandleTypeDef *heth, ETH_MACFilterConfigTypeDef *pFilterConfig);
HAL_StatusTypeDef HAL_ETH_SetMACFilterConfig(ETH_HandleTypeDef *heth, const ETH_MACFilterConfigTypeDef *pFilterConfig);
HAL_StatusTypeDef HAL_ETH_SetHashTable(ETH_HandleTypeDef *heth, uint32_t *pHashTable);
HAL_StatusTypeDef HAL_ETH_SetSourceMACAddrMatch(const ETH_HandleTypeDef *heth, uint32_t AddrNbr,
                                                const uint8_t *pMACAddr);


void HAL_ETH_EnterPowerDownMode(ETH_HandleTypeDef *heth,
                                             const ETH_PowerDownConfigTypeDef *pPowerDownConfig);
void HAL_ETH_ExitPowerDownMode(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_SetWakeUpFilter(ETH_HandleTypeDef *heth, uint32_t *pFilter, uint32_t Count);
# 2121 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_eth.h"
HAL_ETH_StateTypeDef HAL_ETH_GetState(const ETH_HandleTypeDef *heth);
uint32_t HAL_ETH_GetError(const ETH_HandleTypeDef *heth);
uint32_t HAL_ETH_GetDMAError(const ETH_HandleTypeDef *heth);
uint32_t HAL_ETH_GetMACError(const ETH_HandleTypeDef *heth);
uint32_t HAL_ETH_GetMACWakeUpSource(const ETH_HandleTypeDef *heth);
# 333 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2







# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h" 1
# 45 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h"
typedef enum
{
  FLASH_PROC_NONE = 0U,
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;




typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;

  volatile uint32_t NbSectorsToErase;

  volatile uint8_t VoltageForErase;

  volatile uint32_t Sector;

  volatile uint32_t Bank;

  volatile uint32_t Address;

  HAL_LockTypeDef Lock;

  volatile uint32_t ErrorCode;

}FLASH_ProcessTypeDef;
# 295 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash_ex.h" 1
# 45 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash_ex.h"
typedef struct
{
  uint32_t TypeErase;


  uint32_t Banks;


  uint32_t Sector;


  uint32_t NbSectors;


  uint32_t VoltageRange;


} FLASH_EraseInitTypeDef;




typedef struct
{
  uint32_t OptionType;


  uint32_t WRPState;


  uint32_t WRPSector;


  uint32_t Banks;


  uint32_t RDPLevel;


  uint32_t BORLevel;


  uint8_t USERConfig;

} FLASH_OBProgramInitTypeDef;
# 725 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash_ex.h"
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);
# 1044 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash_ex.h"
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);
# 296 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h" 2
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash_ramfunc.h" 1
# 297 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h" 2
# 306 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h"
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);

void HAL_FLASH_IRQHandler(void);

void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);
# 321 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h"
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);

HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);
# 335 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_flash.h"
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);
# 341 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h" 1
# 31 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_fsmc.h" 1
# 166 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_fsmc.h"
typedef struct
{
  uint32_t NSBank;


  uint32_t DataAddressMux;



  uint32_t MemoryType;



  uint32_t MemoryDataWidth;


  uint32_t BurstAccessMode;



  uint32_t WaitSignalPolarity;



  uint32_t WrapMode;




  uint32_t WaitSignalActive;




  uint32_t WriteOperation;


  uint32_t WaitSignal;



  uint32_t ExtendedMode;


  uint32_t AsynchronousWait;



  uint32_t WriteBurst;


  uint32_t ContinuousClock;





  uint32_t WriteFifo;





  uint32_t PageSize;

} FSMC_NORSRAM_InitTypeDef;




typedef struct
{
  uint32_t AddressSetupTime;




  uint32_t AddressHoldTime;




  uint32_t DataSetupTime;





  uint32_t BusTurnAroundDuration;




  uint32_t CLKDivision;





  uint32_t DataLatency;







  uint32_t AccessMode;

} FSMC_NORSRAM_TimingTypeDef;






typedef struct
{
  uint32_t NandBank;


  uint32_t Waitfeature;


  uint32_t MemoryDataWidth;


  uint32_t EccComputation;


  uint32_t ECCPageSize;


  uint32_t TCLRSetupTime;



  uint32_t TARSetupTime;


} FSMC_NAND_InitTypeDef;






typedef struct
{
  uint32_t SetupTime;





  uint32_t WaitSetupTime;





  uint32_t HoldSetupTime;






  uint32_t HiZSetupTime;




} FSMC_NAND_PCC_TimingTypeDef;






typedef struct
{
  uint32_t Waitfeature;


  uint32_t TCLRSetupTime;



  uint32_t TARSetupTime;


}FSMC_PCCARD_InitTypeDef;
# 988 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_fsmc.h"
HAL_StatusTypeDef FSMC_NORSRAM_Init(FSMC_Bank1_TypeDef *Device,
                                    FSMC_NORSRAM_InitTypeDef *Init);
HAL_StatusTypeDef FSMC_NORSRAM_Timing_Init(FSMC_Bank1_TypeDef *Device,
                                           FSMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef FSMC_NORSRAM_Extended_Timing_Init(FSMC_Bank1E_TypeDef *Device,
                                                    FSMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank,
                                                    uint32_t ExtendedMode);
HAL_StatusTypeDef FSMC_NORSRAM_DeInit(FSMC_Bank1_TypeDef *Device,
                                      FSMC_Bank1E_TypeDef *ExDevice, uint32_t Bank);







HAL_StatusTypeDef FSMC_NORSRAM_WriteOperation_Enable(FSMC_Bank1_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef FSMC_NORSRAM_WriteOperation_Disable(FSMC_Bank1_TypeDef *Device, uint32_t Bank);
# 1021 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_fsmc.h"
HAL_StatusTypeDef FSMC_NAND_Init(FSMC_Bank2_3_TypeDef *Device, FSMC_NAND_InitTypeDef *Init);
HAL_StatusTypeDef FSMC_NAND_CommonSpace_Timing_Init(FSMC_Bank2_3_TypeDef *Device,
                                                    FSMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef FSMC_NAND_AttributeSpace_Timing_Init(FSMC_Bank2_3_TypeDef *Device,
                                                       FSMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef FSMC_NAND_DeInit(FSMC_Bank2_3_TypeDef *Device, uint32_t Bank);







HAL_StatusTypeDef FSMC_NAND_ECC_Enable(FSMC_Bank2_3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef FSMC_NAND_ECC_Disable(FSMC_Bank2_3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef FSMC_NAND_GetECC(FSMC_Bank2_3_TypeDef *Device, uint32_t *ECCval, uint32_t Bank,
                                   uint32_t Timeout);
# 1053 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_fsmc.h"
HAL_StatusTypeDef FSMC_PCCARD_Init(FSMC_Bank4_TypeDef *Device, FSMC_PCCARD_InitTypeDef *Init);
HAL_StatusTypeDef FSMC_PCCARD_CommonSpace_Timing_Init(FSMC_Bank4_TypeDef *Device,
                                                               FSMC_NAND_PCC_TimingTypeDef *Timing);
HAL_StatusTypeDef FSMC_PCCARD_AttributeSpace_Timing_Init(FSMC_Bank4_TypeDef *Device,
                                                                  FSMC_NAND_PCC_TimingTypeDef *Timing);
HAL_StatusTypeDef FSMC_PCCARD_IOSpace_Timing_Init(FSMC_Bank4_TypeDef *Device,
                                                           FSMC_NAND_PCC_TimingTypeDef *Timing);
HAL_StatusTypeDef FSMC_PCCARD_DeInit(FSMC_Bank4_TypeDef *Device);
# 32 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h" 2
# 51 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h"
typedef enum
{
  HAL_SRAM_STATE_RESET = 0x00U,
  HAL_SRAM_STATE_READY = 0x01U,
  HAL_SRAM_STATE_BUSY = 0x02U,
  HAL_SRAM_STATE_ERROR = 0x03U,
  HAL_SRAM_STATE_PROTECTED = 0x04U

} HAL_SRAM_StateTypeDef;







typedef struct

{
  FSMC_Bank1_TypeDef *Instance;

  FSMC_Bank1E_TypeDef *Extended;

  FSMC_NORSRAM_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_SRAM_StateTypeDef State;

  DMA_HandleTypeDef *hdma;







} SRAM_HandleTypeDef;
# 147 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h"
HAL_StatusTypeDef HAL_SRAM_Init(SRAM_HandleTypeDef *hsram, FSMC_NORSRAM_TimingTypeDef *Timing,
                                FSMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_SRAM_DeInit(SRAM_HandleTypeDef *hsram);
void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram);
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram);
# 162 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h"
HAL_StatusTypeDef HAL_SRAM_Read_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pDstBuffer,
                                   uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pSrcBuffer,
                                    uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pDstBuffer,
                                    uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pSrcBuffer,
                                     uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                    uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                     uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer,
                                    uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer,
                                     uint32_t BufferSize);

void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);
# 200 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h"
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Enable(SRAM_HandleTypeDef *hsram);
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Disable(SRAM_HandleTypeDef *hsram);
# 212 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sram.h"
HAL_SRAM_StateTypeDef HAL_SRAM_GetState(const SRAM_HandleTypeDef *hsram);
# 345 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nor.h" 1
# 52 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nor.h"
typedef enum
{
  HAL_NOR_STATE_RESET = 0x00U,
  HAL_NOR_STATE_READY = 0x01U,
  HAL_NOR_STATE_BUSY = 0x02U,
  HAL_NOR_STATE_ERROR = 0x03U,
  HAL_NOR_STATE_PROTECTED = 0x04U
} HAL_NOR_StateTypeDef;




typedef enum
{
  HAL_NOR_STATUS_SUCCESS = 0U,
  HAL_NOR_STATUS_ONGOING,
  HAL_NOR_STATUS_ERROR,
  HAL_NOR_STATUS_TIMEOUT
} HAL_NOR_StatusTypeDef;




typedef struct
{
  uint16_t Manufacturer_Code;

  uint16_t Device_Code1;

  uint16_t Device_Code2;

  uint16_t Device_Code3;



} NOR_IDTypeDef;




typedef struct
{




  uint16_t CFI_1;

  uint16_t CFI_2;

  uint16_t CFI_3;

  uint16_t CFI_4;
} NOR_CFITypeDef;







typedef struct


{
  FSMC_Bank1_TypeDef *Instance;

  FSMC_Bank1E_TypeDef *Extended;

  FSMC_NORSRAM_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_NOR_StateTypeDef State;

  uint32_t CommandSet;





} NOR_HandleTypeDef;
# 186 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nor.h"
HAL_StatusTypeDef HAL_NOR_Init(NOR_HandleTypeDef *hnor, FSMC_NORSRAM_TimingTypeDef *Timing,
                               FSMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_NOR_DeInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspDeInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspWait(NOR_HandleTypeDef *hnor, uint32_t Timeout);
# 201 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nor.h"
HAL_StatusTypeDef HAL_NOR_Read_ID(NOR_HandleTypeDef *hnor, NOR_IDTypeDef *pNOR_ID);
HAL_StatusTypeDef HAL_NOR_ReturnToReadMode(NOR_HandleTypeDef *hnor);
HAL_StatusTypeDef HAL_NOR_Read(NOR_HandleTypeDef *hnor, uint32_t *pAddress, uint16_t *pData);
HAL_StatusTypeDef HAL_NOR_Program(NOR_HandleTypeDef *hnor, uint32_t *pAddress, uint16_t *pData);

HAL_StatusTypeDef HAL_NOR_ReadBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData,
                                     uint32_t uwBufferSize);
HAL_StatusTypeDef HAL_NOR_ProgramBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData,
                                        uint32_t uwBufferSize);

HAL_StatusTypeDef HAL_NOR_Erase_Block(NOR_HandleTypeDef *hnor, uint32_t BlockAddress, uint32_t Address);
HAL_StatusTypeDef HAL_NOR_Erase_Chip(NOR_HandleTypeDef *hnor, uint32_t Address);
HAL_StatusTypeDef HAL_NOR_Read_CFI(NOR_HandleTypeDef *hnor, NOR_CFITypeDef *pNOR_CFI);
# 230 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nor.h"
HAL_StatusTypeDef HAL_NOR_WriteOperation_Enable(NOR_HandleTypeDef *hnor);
HAL_StatusTypeDef HAL_NOR_WriteOperation_Disable(NOR_HandleTypeDef *hnor);
# 241 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nor.h"
HAL_NOR_StateTypeDef HAL_NOR_GetState(const NOR_HandleTypeDef *hnor);
HAL_NOR_StatusTypeDef HAL_NOR_GetStatus(NOR_HandleTypeDef *hnor, uint32_t Address, uint32_t Timeout);
# 349 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nand.h" 1
# 53 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nand.h"
typedef enum
{
  HAL_NAND_STATE_RESET = 0x00U,
  HAL_NAND_STATE_READY = 0x01U,
  HAL_NAND_STATE_BUSY = 0x02U,
  HAL_NAND_STATE_ERROR = 0x03U
} HAL_NAND_StateTypeDef;




typedef struct
{


  uint8_t Maker_Id;

  uint8_t Device_Id;

  uint8_t Third_Id;

  uint8_t Fourth_Id;
} NAND_IDTypeDef;




typedef struct
{
  uint16_t Page;

  uint16_t Plane;

  uint16_t Block;

} NAND_AddressTypeDef;




typedef struct
{
  uint32_t PageSize;


  uint32_t SpareAreaSize;


  uint32_t BlockSize;

  uint32_t BlockNbr;

  uint32_t PlaneNbr;

  uint32_t PlaneSize;

  FunctionalState ExtraCommandEnable;




} NAND_DeviceConfigTypeDef;







typedef struct

{
  FSMC_Bank2_3_TypeDef *Instance;

  FSMC_NAND_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_NAND_StateTypeDef State;

  NAND_DeviceConfigTypeDef Config;






} NAND_HandleTypeDef;
# 197 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nand.h"
HAL_StatusTypeDef HAL_NAND_Init(NAND_HandleTypeDef *hnand, FSMC_NAND_PCC_TimingTypeDef *ComSpace_Timing,
                                 FSMC_NAND_PCC_TimingTypeDef *AttSpace_Timing);
HAL_StatusTypeDef HAL_NAND_DeInit(NAND_HandleTypeDef *hnand);

HAL_StatusTypeDef HAL_NAND_ConfigDevice(NAND_HandleTypeDef *hnand, NAND_DeviceConfigTypeDef *pDeviceConfig);

HAL_StatusTypeDef HAL_NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDTypeDef *pNAND_ID);

void HAL_NAND_MspInit(NAND_HandleTypeDef *hnand);
void HAL_NAND_MspDeInit(NAND_HandleTypeDef *hnand);
void HAL_NAND_IRQHandler(NAND_HandleTypeDef *hnand);
void HAL_NAND_ITCallback(NAND_HandleTypeDef *hnand);
# 219 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nand.h"
HAL_StatusTypeDef HAL_NAND_Reset(NAND_HandleTypeDef *hnand);

HAL_StatusTypeDef HAL_NAND_Read_Page_8b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                         uint8_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef HAL_NAND_Write_Page_8b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                          const uint8_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef HAL_NAND_Read_SpareArea_8b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                              uint8_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef HAL_NAND_Write_SpareArea_8b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                               const uint8_t *pBuffer, uint32_t NumSpareAreaTowrite);

HAL_StatusTypeDef HAL_NAND_Read_Page_16b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                          uint16_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef HAL_NAND_Write_Page_16b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                           const uint16_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef HAL_NAND_Read_SpareArea_16b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                               uint16_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef HAL_NAND_Write_SpareArea_16b(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress,
                                                const uint16_t *pBuffer, uint32_t NumSpareAreaTowrite);

HAL_StatusTypeDef HAL_NAND_Erase_Block(NAND_HandleTypeDef *hnand, const NAND_AddressTypeDef *pAddress);

uint32_t HAL_NAND_Address_Inc(const NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);
# 259 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nand.h"
HAL_StatusTypeDef HAL_NAND_ECC_Enable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef HAL_NAND_ECC_Disable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef HAL_NAND_GetECC(NAND_HandleTypeDef *hnand, uint32_t *ECCval, uint32_t Timeout);
# 271 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_nand.h"
HAL_NAND_StateTypeDef HAL_NAND_GetState(const NAND_HandleTypeDef *hnand);
uint32_t HAL_NAND_Read_Status(const NAND_HandleTypeDef *hnand);
# 353 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pccard.h" 1
# 52 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pccard.h"
typedef enum
{
  HAL_PCCARD_STATE_RESET = 0x00U,
  HAL_PCCARD_STATE_READY = 0x01U,
  HAL_PCCARD_STATE_BUSY = 0x02U,
  HAL_PCCARD_STATE_ERROR = 0x04U
} HAL_PCCARD_StateTypeDef;

typedef enum
{
  HAL_PCCARD_STATUS_SUCCESS = 0U,
  HAL_PCCARD_STATUS_ONGOING,
  HAL_PCCARD_STATUS_ERROR,
  HAL_PCCARD_STATUS_TIMEOUT
} HAL_PCCARD_StatusTypeDef;







typedef struct

{
  FSMC_Bank4_TypeDef *Instance;

  FSMC_PCCARD_InitTypeDef Init;

  volatile HAL_PCCARD_StateTypeDef State;

  HAL_LockTypeDef Lock;






} PCCARD_HandleTypeDef;
# 143 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pccard.h"
HAL_StatusTypeDef HAL_PCCARD_Init(PCCARD_HandleTypeDef *hpccard, FSMC_NAND_PCC_TimingTypeDef *ComSpaceTiming,
                                   FSMC_NAND_PCC_TimingTypeDef *AttSpaceTiming, FSMC_NAND_PCC_TimingTypeDef *IOSpaceTiming);
HAL_StatusTypeDef HAL_PCCARD_DeInit(PCCARD_HandleTypeDef *hpccard);
void HAL_PCCARD_MspInit(PCCARD_HandleTypeDef *hpccard);
void HAL_PCCARD_MspDeInit(PCCARD_HandleTypeDef *hpccard);
# 156 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pccard.h"
HAL_StatusTypeDef HAL_PCCARD_Read_ID(PCCARD_HandleTypeDef *hpccard, uint8_t CompactFlash_ID[], uint8_t *pStatus);
HAL_StatusTypeDef HAL_PCCARD_Write_Sector(PCCARD_HandleTypeDef *hpccard, uint16_t *pBuffer, uint16_t SectorAddress,
                                           uint8_t *pStatus);
HAL_StatusTypeDef HAL_PCCARD_Read_Sector(PCCARD_HandleTypeDef *hpccard, uint16_t *pBuffer, uint16_t SectorAddress,
                                          uint8_t *pStatus);
HAL_StatusTypeDef HAL_PCCARD_Erase_Sector(PCCARD_HandleTypeDef *hpccard, uint16_t SectorAddress, uint8_t *pStatus);
HAL_StatusTypeDef HAL_PCCARD_Reset(PCCARD_HandleTypeDef *hpccard);
void HAL_PCCARD_IRQHandler(PCCARD_HandleTypeDef *hpccard);
void HAL_PCCARD_ITCallback(PCCARD_HandleTypeDef *hpccard);
# 181 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pccard.h"
HAL_PCCARD_StateTypeDef HAL_PCCARD_GetState(PCCARD_HandleTypeDef *hpccard);
HAL_PCCARD_StatusTypeDef HAL_PCCARD_GetStatus(PCCARD_HandleTypeDef *hpccard);
HAL_PCCARD_StatusTypeDef HAL_PCCARD_ReadStatus(PCCARD_HandleTypeDef *hpccard);
# 357 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sdram.h" 1
# 361 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hash.h" 1
# 365 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2







# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
typedef struct
{
  uint32_t ClockSpeed;


  uint32_t AnalogFilter;


  uint32_t OwnAddress1;


  uint32_t AddressingMode;


  uint32_t DualAddressMode;


  uint32_t OwnAddress2;


  uint32_t GeneralCallMode;


  uint32_t NoStretchMode;


  uint32_t PacketErrorCheckMode;


  uint32_t PeripheralMode;


} SMBUS_InitTypeDef;
# 106 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
typedef enum
{

  HAL_SMBUS_STATE_RESET = 0x00U,
  HAL_SMBUS_STATE_READY = 0x20U,
  HAL_SMBUS_STATE_BUSY = 0x24U,
  HAL_SMBUS_STATE_BUSY_TX = 0x21U,
  HAL_SMBUS_STATE_BUSY_RX = 0x22U,
  HAL_SMBUS_STATE_LISTEN = 0x28U,
  HAL_SMBUS_STATE_BUSY_TX_LISTEN = 0x29U,

  HAL_SMBUS_STATE_BUSY_RX_LISTEN = 0x2AU,

  HAL_SMBUS_STATE_ABORT = 0x60U,
  HAL_SMBUS_STATE_TIMEOUT = 0xA0U,
  HAL_SMBUS_STATE_ERROR = 0xE0U
} HAL_SMBUS_StateTypeDef;
# 140 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
typedef enum
{
  HAL_SMBUS_MODE_NONE = 0x00U,
  HAL_SMBUS_MODE_MASTER = 0x10U,
  HAL_SMBUS_MODE_SLAVE = 0x20U,

} HAL_SMBUS_ModeTypeDef;




typedef struct __SMBUS_HandleTypeDef
{
  I2C_TypeDef *Instance;

  SMBUS_InitTypeDef Init;

  uint8_t *pBuffPtr;

  uint16_t XferSize;

  volatile uint16_t XferCount;

  volatile uint32_t XferOptions;


  volatile uint32_t PreviousState;


  HAL_LockTypeDef Lock;

  volatile HAL_SMBUS_StateTypeDef State;

  volatile HAL_SMBUS_ModeTypeDef Mode;

  volatile uint32_t ErrorCode;

  volatile uint32_t Devaddress;

  volatile uint32_t EventCount;

  uint8_t XferPEC;
# 198 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
} SMBUS_HandleTypeDef;
# 536 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
HAL_StatusTypeDef HAL_SMBUS_Init(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_DeInit(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MspInit(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MspDeInit(SMBUS_HandleTypeDef *hsmbus);
# 563 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
HAL_StatusTypeDef HAL_SMBUS_IsDeviceReady(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);
# 572 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
HAL_StatusTypeDef HAL_SMBUS_Master_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_SMBUS_Master_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_SMBUS_Master_Abort_IT(SMBUS_HandleTypeDef *hsmbus, uint16_t DevAddress);
HAL_StatusTypeDef HAL_SMBUS_Slave_Transmit_IT(SMBUS_HandleTypeDef *hsmbus, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_SMBUS_Slave_Receive_IT(SMBUS_HandleTypeDef *hsmbus, uint8_t *pData, uint16_t Size, uint32_t XferOptions);

HAL_StatusTypeDef HAL_SMBUS_EnableAlert_IT(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_DisableAlert_IT(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_EnableListen_IT(SMBUS_HandleTypeDef *hsmbus);
HAL_StatusTypeDef HAL_SMBUS_DisableListen_IT(SMBUS_HandleTypeDef *hsmbus);
# 596 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
void HAL_SMBUS_EV_IRQHandler(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_ER_IRQHandler(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MasterTxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_MasterRxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_SlaveTxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_SlaveRxCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_AddrCallback(SMBUS_HandleTypeDef *hsmbus, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_SMBUS_ListenCpltCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_ErrorCallback(SMBUS_HandleTypeDef *hsmbus);
void HAL_SMBUS_AbortCpltCallback(SMBUS_HandleTypeDef *hsmbus);
# 620 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smbus.h"
HAL_SMBUS_StateTypeDef HAL_SMBUS_GetState(SMBUS_HandleTypeDef *hsmbus);
HAL_SMBUS_ModeTypeDef HAL_SMBUS_GetMode(SMBUS_HandleTypeDef *hsmbus);
uint32_t HAL_SMBUS_GetError(SMBUS_HandleTypeDef *hsmbus);
# 373 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h"
typedef struct
{
  uint32_t Mode;


  uint32_t Standard;


  uint32_t DataFormat;


  uint32_t MCLKOutput;


  uint32_t AudioFreq;


  uint32_t CPOL;


  uint32_t ClockSource;

  uint32_t FullDuplexMode;

} I2S_InitTypeDef;




typedef enum
{
  HAL_I2S_STATE_RESET = 0x00U,
  HAL_I2S_STATE_READY = 0x01U,
  HAL_I2S_STATE_BUSY = 0x02U,
  HAL_I2S_STATE_BUSY_TX = 0x03U,
  HAL_I2S_STATE_BUSY_RX = 0x04U,
  HAL_I2S_STATE_BUSY_TX_RX = 0x05U,
  HAL_I2S_STATE_TIMEOUT = 0x06U,
  HAL_I2S_STATE_ERROR = 0x07U
} HAL_I2S_StateTypeDef;




typedef struct __I2S_HandleTypeDef
{
  SPI_TypeDef *Instance;

  I2S_InitTypeDef Init;

  uint16_t *pTxBuffPtr;

  volatile uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint16_t *pRxBuffPtr;

  volatile uint16_t RxXferSize;

  volatile uint16_t RxXferCount;





  void (*IrqHandlerISR)(struct __I2S_HandleTypeDef *hi2s);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  volatile HAL_LockTypeDef Lock;

  volatile HAL_I2S_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 137 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h"
} I2S_HandleTypeDef;
# 438 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s_ex.h" 1
# 138 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s_ex.h"
HAL_StatusTypeDef HAL_I2SEx_TransmitReceive(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_I2SEx_TransmitReceive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                               uint16_t Size);

HAL_StatusTypeDef HAL_I2SEx_TransmitReceive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pTxData, uint16_t *pRxData,
                                                uint16_t Size);

void HAL_I2SEx_FullDuplex_IRQHandler(I2S_HandleTypeDef *hi2s);
void HAL_I2SEx_TxRxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2SEx_TxRxCpltCallback(I2S_HandleTypeDef *hi2s);
# 439 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h" 2
# 449 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h"
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);
# 469 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h"
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);


HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);


HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);


void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);
# 499 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_i2s.h"
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);
# 377 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_iwdg.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_iwdg.h"
typedef struct
{
  uint32_t Prescaler;


  uint32_t Reload;


} IWDG_InitTypeDef;




typedef struct
{
  IWDG_TypeDef *Instance;

  IWDG_InitTypeDef Init;
} IWDG_HandleTypeDef;
# 127 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_iwdg.h"
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);
# 136 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_iwdg.h"
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);
# 381 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_ltdc.h" 1
# 385 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr.h"
typedef struct
{
  uint32_t PVDLevel;


  uint32_t Mode;

}PWR_PVDTypeDef;
# 275 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr_ex.h" 1
# 203 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr_ex.h"
void HAL_PWREx_EnableFlashPowerDown(void);
void HAL_PWREx_DisableFlashPowerDown(void);
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void);
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);
# 276 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr.h" 2
# 286 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr.h"
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);
# 298 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pwr.h"
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);


void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);


void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);


void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);


void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);
# 389 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h" 1
# 58 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h"
typedef enum
{
  HAL_RNG_STATE_RESET = 0x00U,
  HAL_RNG_STATE_READY = 0x01U,
  HAL_RNG_STATE_BUSY = 0x02U,
  HAL_RNG_STATE_TIMEOUT = 0x03U,
  HAL_RNG_STATE_ERROR = 0x04U

} HAL_RNG_StateTypeDef;
# 78 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h"
typedef struct

{
  RNG_TypeDef *Instance;

  HAL_LockTypeDef Lock;

  volatile HAL_RNG_StateTypeDef State;

  volatile uint32_t ErrorCode;

  uint32_t RandomNumber;
# 99 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h"
} RNG_HandleTypeDef;
# 279 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h"
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng);
HAL_StatusTypeDef HAL_RNG_DeInit(RNG_HandleTypeDef *hrng);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);
# 301 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h"
uint32_t HAL_RNG_GetRandomNumber(RNG_HandleTypeDef
                                 *hrng);
uint32_t HAL_RNG_GetRandomNumber_IT(RNG_HandleTypeDef
                                    *hrng);
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber_IT(RNG_HandleTypeDef *hrng);
uint32_t HAL_RNG_ReadLastRandomNumber(const RNG_HandleTypeDef *hrng);

void HAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng);
void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng);
void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef *hrng, uint32_t random32bit);
# 320 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rng.h"
HAL_RNG_StateTypeDef HAL_RNG_GetState(const RNG_HandleTypeDef *hrng);
uint32_t HAL_RNG_GetError(const RNG_HandleTypeDef *hrng);
# 393 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h" 1
# 48 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
typedef enum
{
  HAL_RTC_STATE_RESET = 0x00U,
  HAL_RTC_STATE_READY = 0x01U,
  HAL_RTC_STATE_BUSY = 0x02U,
  HAL_RTC_STATE_TIMEOUT = 0x03U,
  HAL_RTC_STATE_ERROR = 0x04U
} HAL_RTCStateTypeDef;




typedef struct
{
  uint32_t HourFormat;


  uint32_t AsynchPrediv;


  uint32_t SynchPrediv;


  uint32_t OutPut;


  uint32_t OutPutPolarity;


  uint32_t OutPutType;

} RTC_InitTypeDef;




typedef struct
{
  uint8_t Hours;



  uint8_t Minutes;


  uint8_t Seconds;


  uint8_t TimeFormat;


  uint32_t SubSeconds;



  uint32_t SecondFraction;





  uint32_t DayLightSaving;


  uint32_t StoreOperation;

} RTC_TimeTypeDef;




typedef struct
{
  uint8_t WeekDay;


  uint8_t Month;


  uint8_t Date;


  uint8_t Year;


} RTC_DateTypeDef;




typedef struct
{
  RTC_TimeTypeDef AlarmTime;

  uint32_t AlarmMask;


  uint32_t AlarmSubSecondMask;


  uint32_t AlarmDateWeekDaySel;


  uint8_t AlarmDateWeekDay;



  uint32_t Alarm;

} RTC_AlarmTypeDef;







typedef struct

{
  RTC_TypeDef *Instance;

  RTC_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  volatile HAL_RTCStateTypeDef State;
# 197 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
} RTC_HandleTypeDef;
# 683 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc_ex.h" 1
# 48 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc_ex.h"
typedef struct
{
  uint32_t Tamper;


  uint32_t PinSelection;


  uint32_t Trigger;


  uint32_t Filter;


  uint32_t SamplingFrequency;


  uint32_t PrechargeDuration;


  uint32_t TamperPullUp;


  uint32_t TimeStampOnTamperDetection;

} RTC_TamperTypeDef;
# 843 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc_ex.h"
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t RTC_TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t RTC_TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);

HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef *sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);
void HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);

void HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);

void HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);

void HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);

HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
# 871 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc_ex.h"
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
HAL_StatusTypeDef HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
# 886 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc_ex.h"
void HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);

HAL_StatusTypeDef HAL_RTCEx_SetCoarseCalib(RTC_HandleTypeDef *hrtc, uint32_t CalibSign, uint32_t Value);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCoarseCalib(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmoothCalibMinusPulsesValue);
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);
# 907 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc_ex.h"
void HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
# 684 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h" 2
# 695 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);
# 713 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
# 725 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);
# 740 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
HAL_StatusTypeDef HAL_RTC_WaitForSynchro(RTC_HandleTypeDef *hrtc);


void HAL_RTC_DST_Add1Hour(RTC_HandleTypeDef *hrtc);
void HAL_RTC_DST_Sub1Hour(RTC_HandleTypeDef *hrtc);
void HAL_RTC_DST_SetStoreOperation(RTC_HandleTypeDef *hrtc);
void HAL_RTC_DST_ClearStoreOperation(RTC_HandleTypeDef *hrtc);
uint32_t HAL_RTC_DST_ReadStoreOperation(RTC_HandleTypeDef *hrtc);
# 756 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);
# 902 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_rtc.h"
HAL_StatusTypeDef RTC_EnterInitMode(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef RTC_ExitInitMode(RTC_HandleTypeDef *hrtc);
uint8_t RTC_ByteToBcd2(uint8_t number);
uint8_t RTC_Bcd2ToByte(uint8_t number);
# 397 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sai.h" 1
# 401 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h" 1
# 30 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h" 1
# 48 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h"
typedef struct
{
  uint32_t ClockEdge;


  uint32_t ClockBypass;



  uint32_t ClockPowerSave;



  uint32_t BusWide;


  uint32_t HardwareFlowControl;


  uint32_t ClockDiv;


}SDIO_InitTypeDef;





typedef struct
{
  uint32_t Argument;




  uint32_t CmdIndex;


  uint32_t Response;


  uint32_t WaitForInterrupt;



  uint32_t CPSM;


}SDIO_CmdInitTypeDef;





typedef struct
{
  uint32_t DataTimeOut;

  uint32_t DataLength;

  uint32_t DataBlockSize;


  uint32_t TransferDir;



  uint32_t TransferMode;


  uint32_t DPSM;


}SDIO_DataInitTypeDef;
# 1040 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h"
HAL_StatusTypeDef SDIO_Init(SDIO_TypeDef *SDIOx, SDIO_InitTypeDef Init);
# 1049 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h"
uint32_t SDIO_ReadFIFO(SDIO_TypeDef *SDIOx);
HAL_StatusTypeDef SDIO_WriteFIFO(SDIO_TypeDef *SDIOx, uint32_t *pWriteData);
# 1059 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h"
HAL_StatusTypeDef SDIO_PowerState_ON(SDIO_TypeDef *SDIOx);
HAL_StatusTypeDef SDIO_PowerState_OFF(SDIO_TypeDef *SDIOx);
uint32_t SDIO_GetPowerState(SDIO_TypeDef *SDIOx);


HAL_StatusTypeDef SDIO_SendCommand(SDIO_TypeDef *SDIOx, SDIO_CmdInitTypeDef *Command);
uint8_t SDIO_GetCommandResponse(SDIO_TypeDef *SDIOx);
uint32_t SDIO_GetResponse(SDIO_TypeDef *SDIOx, uint32_t Response);


HAL_StatusTypeDef SDIO_ConfigData(SDIO_TypeDef *SDIOx, SDIO_DataInitTypeDef* Data);
uint32_t SDIO_GetDataCounter(SDIO_TypeDef *SDIOx);
uint32_t SDIO_GetFIFOCount(SDIO_TypeDef *SDIOx);


HAL_StatusTypeDef SDIO_SetSDMMCReadWaitMode(SDIO_TypeDef *SDIOx, uint32_t SDIO_ReadWaitMode);
# 1083 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h"
uint32_t SDMMC_CmdBlockLength(SDIO_TypeDef *SDIOx, uint32_t BlockSize);
uint32_t SDMMC_CmdReadSingleBlock(SDIO_TypeDef *SDIOx, uint32_t ReadAdd);
uint32_t SDMMC_CmdReadMultiBlock(SDIO_TypeDef *SDIOx, uint32_t ReadAdd);
uint32_t SDMMC_CmdWriteSingleBlock(SDIO_TypeDef *SDIOx, uint32_t WriteAdd);
uint32_t SDMMC_CmdWriteMultiBlock(SDIO_TypeDef *SDIOx, uint32_t WriteAdd);
uint32_t SDMMC_CmdEraseStartAdd(SDIO_TypeDef *SDIOx, uint32_t StartAdd);
uint32_t SDMMC_CmdSDEraseStartAdd(SDIO_TypeDef *SDIOx, uint32_t StartAdd);
uint32_t SDMMC_CmdEraseEndAdd(SDIO_TypeDef *SDIOx, uint32_t EndAdd);
uint32_t SDMMC_CmdSDEraseEndAdd(SDIO_TypeDef *SDIOx, uint32_t EndAdd);
uint32_t SDMMC_CmdErase(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdStopTransfer(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdSelDesel(SDIO_TypeDef *SDIOx, uint64_t Addr);
uint32_t SDMMC_CmdGoIdleState(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdOperCond(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdAppCommand(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdAppOperCommand(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdBusWidth(SDIO_TypeDef *SDIOx, uint32_t BusWidth);
uint32_t SDMMC_CmdSendSCR(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdSendCID(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdSendCSD(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdSetRelAdd(SDIO_TypeDef *SDIOx, uint16_t *pRCA);
uint32_t SDMMC_CmdSetRelAddMmc(SDIO_TypeDef *SDIOx, uint16_t RCA);
uint32_t SDMMC_CmdSendStatus(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdStatusRegister(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_CmdOpCondition(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdSwitch(SDIO_TypeDef *SDIOx, uint32_t Argument);
uint32_t SDMMC_CmdSendEXTCSD(SDIO_TypeDef *SDIOx, uint32_t Argument);
# 1118 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_sdmmc.h"
uint32_t SDMMC_GetCmdResp1(SDIO_TypeDef *SDIOx, uint8_t SD_CMD, uint32_t Timeout);
uint32_t SDMMC_GetCmdResp2(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_GetCmdResp3(SDIO_TypeDef *SDIOx);
uint32_t SDMMC_GetCmdResp6(SDIO_TypeDef *SDIOx, uint8_t SD_CMD, uint16_t *pRCA);
uint32_t SDMMC_GetCmdResp7(SDIO_TypeDef *SDIOx);
# 31 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h" 2
# 49 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
typedef enum
{
  HAL_SD_STATE_RESET = 0x00000000U,
  HAL_SD_STATE_READY = 0x00000001U,
  HAL_SD_STATE_TIMEOUT = 0x00000002U,
  HAL_SD_STATE_BUSY = 0x00000003U,
  HAL_SD_STATE_PROGRAMMING = 0x00000004U,
  HAL_SD_STATE_RECEIVING = 0x00000005U,
  HAL_SD_STATE_TRANSFER = 0x00000006U,
  HAL_SD_STATE_ERROR = 0x0000000FU
}HAL_SD_StateTypeDef;







typedef uint32_t HAL_SD_CardStateTypeDef;
# 91 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
typedef struct
{
  uint32_t CardType;

  uint32_t CardVersion;

  uint32_t Class;

  uint32_t RelCardAdd;

  uint32_t BlockNbr;

  uint32_t BlockSize;

  uint32_t LogBlockNbr;

  uint32_t LogBlockSize;

}HAL_SD_CardInfoTypeDef;







typedef struct

{
  SDIO_TypeDef *Instance;

  SDIO_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  uint8_t *pTxBuffPtr;

  uint32_t TxXferSize;

  uint8_t *pRxBuffPtr;

  uint32_t RxXferSize;

  volatile uint32_t Context;

  volatile HAL_SD_StateTypeDef State;

  volatile uint32_t ErrorCode;

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_SD_CardInfoTypeDef SdCard;

  uint32_t CSD[4];

  uint32_t CID[4];
# 159 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
}SD_HandleTypeDef;
# 168 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
typedef struct
{
  volatile uint8_t CSDStruct;
  volatile uint8_t SysSpecVersion;
  volatile uint8_t Reserved1;
  volatile uint8_t TAAC;
  volatile uint8_t NSAC;
  volatile uint8_t MaxBusClkFrec;
  volatile uint16_t CardComdClasses;
  volatile uint8_t RdBlockLen;
  volatile uint8_t PartBlockRead;
  volatile uint8_t WrBlockMisalign;
  volatile uint8_t RdBlockMisalign;
  volatile uint8_t DSRImpl;
  volatile uint8_t Reserved2;
  volatile uint32_t DeviceSize;
  volatile uint8_t MaxRdCurrentVDDMin;
  volatile uint8_t MaxRdCurrentVDDMax;
  volatile uint8_t MaxWrCurrentVDDMin;
  volatile uint8_t MaxWrCurrentVDDMax;
  volatile uint8_t DeviceSizeMul;
  volatile uint8_t EraseGrSize;
  volatile uint8_t EraseGrMul;
  volatile uint8_t WrProtectGrSize;
  volatile uint8_t WrProtectGrEnable;
  volatile uint8_t ManDeflECC;
  volatile uint8_t WrSpeedFact;
  volatile uint8_t MaxWrBlockLen;
  volatile uint8_t WriteBlockPaPartial;
  volatile uint8_t Reserved3;
  volatile uint8_t ContentProtectAppli;
  volatile uint8_t FileFormatGroup;
  volatile uint8_t CopyFlag;
  volatile uint8_t PermWrProtect;
  volatile uint8_t TempWrProtect;
  volatile uint8_t FileFormat;
  volatile uint8_t ECC;
  volatile uint8_t CSD_CRC;
  volatile uint8_t Reserved4;
}HAL_SD_CardCSDTypeDef;







typedef struct
{
  volatile uint8_t ManufacturerID;
  volatile uint16_t OEM_AppliID;
  volatile uint32_t ProdName1;
  volatile uint8_t ProdName2;
  volatile uint8_t ProdRev;
  volatile uint32_t ProdSN;
  volatile uint8_t Reserved1;
  volatile uint16_t ManufactDate;
  volatile uint8_t CID_CRC;
  volatile uint8_t Reserved2;

}HAL_SD_CardCIDTypeDef;







typedef struct
{
  volatile uint8_t DataBusWidth;
  volatile uint8_t SecuredMode;
  volatile uint16_t CardType;
  volatile uint32_t ProtectedAreaSize;
  volatile uint8_t SpeedClass;
  volatile uint8_t PerformanceMove;
  volatile uint8_t AllocationUnitSize;
  volatile uint16_t EraseSize;
  volatile uint8_t EraseTimeout;
  volatile uint8_t EraseOffset;

}HAL_SD_CardStatusTypeDef;
# 595 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
HAL_StatusTypeDef HAL_SD_Init(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_InitCard(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_DeInit (SD_HandleTypeDef *hsd);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);
# 608 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
HAL_StatusTypeDef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_SD_Erase(SD_HandleTypeDef *hsd, uint32_t BlockStartAdd, uint32_t BlockEndAdd);

HAL_StatusTypeDef HAL_SD_ReadBlocks_IT(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_SD_WriteBlocks_IT(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

HAL_StatusTypeDef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd);


void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd);
void HAL_SD_AbortCallback(SD_HandleTypeDef *hsd);
# 639 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_sd.h"
HAL_StatusTypeDef HAL_SD_ConfigWideBusOperation(SD_HandleTypeDef *hsd, uint32_t WideMode);







HAL_StatusTypeDef HAL_SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
HAL_SD_CardStateTypeDef HAL_SD_GetCardState(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_GetCardCID(SD_HandleTypeDef *hsd, HAL_SD_CardCIDTypeDef *pCID);
HAL_StatusTypeDef HAL_SD_GetCardCSD(SD_HandleTypeDef *hsd, HAL_SD_CardCSDTypeDef *pCSD);
HAL_StatusTypeDef HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypeDef *pStatus);
HAL_StatusTypeDef HAL_SD_GetCardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypeDef *pCardInfo);







HAL_SD_StateTypeDef HAL_SD_GetState(SD_HandleTypeDef *hsd);
uint32_t HAL_SD_GetError(SD_HandleTypeDef *hsd);







HAL_StatusTypeDef HAL_SD_Abort(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef HAL_SD_Abort_IT(SD_HandleTypeDef *hsd);
# 405 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spi.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spi.h"
typedef struct
{
  uint32_t Mode;


  uint32_t Direction;


  uint32_t DataSize;


  uint32_t CLKPolarity;


  uint32_t CLKPhase;


  uint32_t NSS;



  uint32_t BaudRatePrescaler;





  uint32_t FirstBit;


  uint32_t TIMode;


  uint32_t CRCCalculation;


  uint32_t CRCPolynomial;

} SPI_InitTypeDef;




typedef enum
{
  HAL_SPI_STATE_RESET = 0x00U,
  HAL_SPI_STATE_READY = 0x01U,
  HAL_SPI_STATE_BUSY = 0x02U,
  HAL_SPI_STATE_BUSY_TX = 0x03U,
  HAL_SPI_STATE_BUSY_RX = 0x04U,
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,
  HAL_SPI_STATE_ERROR = 0x06U,
  HAL_SPI_STATE_ABORT = 0x07U
} HAL_SPI_StateTypeDef;




typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef *Instance;

  SPI_InitTypeDef Init;

  uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_SPI_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 149 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spi.h"
} SPI_HandleTypeDef;
# 651 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spi.h"
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);
# 670 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spi.h"
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);
# 706 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spi.h"
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t HAL_SPI_GetError(SPI_HandleTypeDef *hspi);
# 409 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
typedef struct
{
  uint32_t Prescaler;


  uint32_t CounterMode;


  uint32_t Period;



  uint32_t ClockDivision;


  uint32_t RepetitionCounter;
# 72 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
  uint32_t AutoReloadPreload;

} TIM_Base_InitTypeDef;




typedef struct
{
  uint32_t OCMode;


  uint32_t Pulse;


  uint32_t OCPolarity;


  uint32_t OCNPolarity;



  uint32_t OCFastMode;




  uint32_t OCIdleState;



  uint32_t OCNIdleState;


} TIM_OC_InitTypeDef;




typedef struct
{
  uint32_t OCMode;


  uint32_t Pulse;


  uint32_t OCPolarity;


  uint32_t OCNPolarity;



  uint32_t OCIdleState;



  uint32_t OCNIdleState;



  uint32_t ICPolarity;


  uint32_t ICSelection;


  uint32_t ICFilter;

} TIM_OnePulse_InitTypeDef;




typedef struct
{
  uint32_t ICPolarity;


  uint32_t ICSelection;


  uint32_t ICPrescaler;


  uint32_t ICFilter;

} TIM_IC_InitTypeDef;




typedef struct
{
  uint32_t EncoderMode;


  uint32_t IC1Polarity;


  uint32_t IC1Selection;


  uint32_t IC1Prescaler;


  uint32_t IC1Filter;


  uint32_t IC2Polarity;


  uint32_t IC2Selection;


  uint32_t IC2Prescaler;


  uint32_t IC2Filter;

} TIM_Encoder_InitTypeDef;




typedef struct
{
  uint32_t ClockSource;

  uint32_t ClockPolarity;

  uint32_t ClockPrescaler;

  uint32_t ClockFilter;

} TIM_ClockConfigTypeDef;




typedef struct
{
  uint32_t ClearInputState;

  uint32_t ClearInputSource;

  uint32_t ClearInputPolarity;

  uint32_t ClearInputPrescaler;


  uint32_t ClearInputFilter;

} TIM_ClearInputConfigTypeDef;




typedef struct
{
  uint32_t MasterOutputTrigger;

  uint32_t MasterSlaveMode;






} TIM_MasterConfigTypeDef;




typedef struct
{
  uint32_t SlaveMode;

  uint32_t InputTrigger;

  uint32_t TriggerPolarity;

  uint32_t TriggerPrescaler;

  uint32_t TriggerFilter;


} TIM_SlaveConfigTypeDef;






typedef struct
{
  uint32_t OffStateRunMode;

  uint32_t OffStateIDLEMode;

  uint32_t LockLevel;

  uint32_t DeadTime;

  uint32_t BreakState;

  uint32_t BreakPolarity;

  uint32_t BreakFilter;

  uint32_t AutomaticOutput;

} TIM_BreakDeadTimeConfigTypeDef;




typedef enum
{
  HAL_TIM_STATE_RESET = 0x00U,
  HAL_TIM_STATE_READY = 0x01U,
  HAL_TIM_STATE_BUSY = 0x02U,
  HAL_TIM_STATE_TIMEOUT = 0x03U,
  HAL_TIM_STATE_ERROR = 0x04U
} HAL_TIM_StateTypeDef;




typedef enum
{
  HAL_TIM_CHANNEL_STATE_RESET = 0x00U,
  HAL_TIM_CHANNEL_STATE_READY = 0x01U,
  HAL_TIM_CHANNEL_STATE_BUSY = 0x02U,
} HAL_TIM_ChannelStateTypeDef;




typedef enum
{
  HAL_DMA_BURST_STATE_RESET = 0x00U,
  HAL_DMA_BURST_STATE_READY = 0x01U,
  HAL_DMA_BURST_STATE_BUSY = 0x02U,
} HAL_TIM_DMABurstStateTypeDef;




typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1 = 0x01U,
  HAL_TIM_ACTIVE_CHANNEL_2 = 0x02U,
  HAL_TIM_ACTIVE_CHANNEL_3 = 0x04U,
  HAL_TIM_ACTIVE_CHANNEL_4 = 0x08U,
  HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0x00U
} HAL_TIM_ActiveChannel;







typedef struct

{
  TIM_TypeDef *Instance;
  TIM_Base_InitTypeDef Init;
  HAL_TIM_ActiveChannel Channel;
  DMA_HandleTypeDef *hdma[7];

  HAL_LockTypeDef Lock;
  volatile HAL_TIM_StateTypeDef State;
  volatile HAL_TIM_ChannelStateTypeDef ChannelState[4];
  volatile HAL_TIM_ChannelStateTypeDef ChannelNState[4];
  volatile HAL_TIM_DMABurstStateTypeDef DMABurstState;
# 380 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
} TIM_HandleTypeDef;
# 1878 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h" 1
# 47 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
typedef struct
{
  uint32_t IC1Polarity;


  uint32_t IC1Prescaler;


  uint32_t IC1Filter;


  uint32_t Commutation_Delay;

} TIM_HallSensor_InitTypeDef;
# 207 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef *htim, const TIM_HallSensor_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef *htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef *htim);


HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef *htim);
# 232 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);


HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);


HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                          uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 253 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);


HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                           uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 273 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);


HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
# 288 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent(TIM_HandleTypeDef *htim, uint32_t InputTrigger,
                                              uint32_t CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_IT(TIM_HandleTypeDef *htim, uint32_t InputTrigger,
                                                 uint32_t CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutEvent_DMA(TIM_HandleTypeDef *htim, uint32_t InputTrigger,
                                                  uint32_t CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *htim,
                                                        const TIM_MasterConfigTypeDef *sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef *htim,
                                                const TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef *htim, uint32_t Remap);
# 308 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_CommutHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim);
# 320 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(const TIM_HandleTypeDef *htim);
HAL_TIM_ChannelStateTypeDef HAL_TIMEx_GetChannelNState(const TIM_HandleTypeDef *htim, uint32_t ChannelN);
# 335 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim_ex.h"
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);
void TIMEx_DMACommutationHalfCplt(DMA_HandleTypeDef *hdma);
# 1879 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h" 2
# 1890 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, const uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);
# 1912 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                       uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 1935 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, const uint32_t *pData,
                                        uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 1958 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 1980 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
# 1999 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim, const TIM_Encoder_InitTypeDef *sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);

HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);

HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1,
                                            uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);
# 2022 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);
# 2032 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, const TIM_OC_InitTypeDef *sConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, const TIM_OC_InitTypeDef *sConfig,
                                            uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, const TIM_IC_InitTypeDef *sConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef *sConfig,
                                                 uint32_t OutputChannel, uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim,
                                           const TIM_ClearInputConfigTypeDef *sClearInputConfig,
                                           uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, const TIM_ClockConfigTypeDef *sClockSourceConfig);
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro(TIM_HandleTypeDef *htim, const TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchro_IT(TIM_HandleTypeDef *htim, const TIM_SlaveConfigTypeDef *sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                              uint32_t BurstRequestSrc, const uint32_t *BurstBuffer,
                                              uint32_t BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                   uint32_t BurstRequestSrc, const uint32_t *BurstBuffer,
                                                   uint32_t BurstLength, uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                             uint32_t BurstRequestSrc, uint32_t *BurstBuffer, uint32_t BurstLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_MultiReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress,
                                                  uint32_t BurstRequestSrc, uint32_t *BurstBuffer,
                                                  uint32_t BurstLength, uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(const TIM_HandleTypeDef *htim, uint32_t Channel);
# 2071 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerHalfCpltCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);
# 2098 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(const TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(const TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(const TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(const TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(const TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(const TIM_HandleTypeDef *htim);


HAL_TIM_ActiveChannel HAL_TIM_GetActiveChannel(const TIM_HandleTypeDef *htim);
HAL_TIM_ChannelStateTypeDef HAL_TIM_GetChannelState(const TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_TIM_DMABurstStateTypeDef HAL_TIM_DMABurstState(const TIM_HandleTypeDef *htim);
# 2122 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_tim.h"
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, const TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, const TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ExtTRGPrescaler,
                       uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState);
# 413 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
typedef struct
{
  uint32_t BaudRate;





  uint32_t WordLength;


  uint32_t StopBits;


  uint32_t Parity;






  uint32_t Mode;


  uint32_t HwFlowCtl;


  uint32_t OverSampling;

} UART_InitTypeDef;
# 116 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
typedef enum
{
  HAL_UART_STATE_RESET = 0x00U,

  HAL_UART_STATE_READY = 0x20U,

  HAL_UART_STATE_BUSY = 0x24U,

  HAL_UART_STATE_BUSY_TX = 0x21U,

  HAL_UART_STATE_BUSY_RX = 0x22U,

  HAL_UART_STATE_BUSY_TX_RX = 0x23U,


  HAL_UART_STATE_TIMEOUT = 0xA0U,

  HAL_UART_STATE_ERROR = 0xE0U

} HAL_UART_StateTypeDef;
# 144 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
typedef uint32_t HAL_UART_RxTypeTypeDef;
# 155 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
typedef uint32_t HAL_UART_RxEventTypeTypeDef;




typedef struct __UART_HandleTypeDef
{
  USART_TypeDef *Instance;

  UART_InitTypeDef Init;

  const uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  volatile HAL_UART_RxTypeTypeDef ReceptionType;

  volatile HAL_UART_RxEventTypeTypeDef RxEventType;

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_UART_StateTypeDef gState;



  volatile HAL_UART_StateTypeDef RxState;


  volatile uint32_t ErrorCode;
# 213 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
} UART_HandleTypeDef;
# 718 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit(UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);
# 745 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint16_t *RxLen,
                                           uint32_t Timeout);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef *huart);


HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
# 790 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);
# 803 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
HAL_UART_StateTypeDef HAL_UART_GetState(const UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(const UART_HandleTypeDef *huart);
# 889 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_uart.h"
HAL_StatusTypeDef UART_Start_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef UART_Start_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
# 417 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_usart.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_usart.h"
typedef struct
{
  uint32_t BaudRate;




  uint32_t WordLength;


  uint32_t StopBits;


  uint32_t Parity;






  uint32_t Mode;


  uint32_t CLKPolarity;


  uint32_t CLKPhase;


  uint32_t CLKLastBit;


} USART_InitTypeDef;




typedef enum
{
  HAL_USART_STATE_RESET = 0x00U,
  HAL_USART_STATE_READY = 0x01U,
  HAL_USART_STATE_BUSY = 0x02U,
  HAL_USART_STATE_BUSY_TX = 0x12U,
  HAL_USART_STATE_BUSY_RX = 0x22U,
  HAL_USART_STATE_BUSY_TX_RX = 0x32U,
  HAL_USART_STATE_TIMEOUT = 0x03U,
  HAL_USART_STATE_ERROR = 0x04U
} HAL_USART_StateTypeDef;




typedef struct __USART_HandleTypeDef
{
  USART_TypeDef *Instance;

  USART_InitTypeDef Init;

  const uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_USART_StateTypeDef State;

  volatile uint32_t ErrorCode;
# 139 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_usart.h"
} USART_HandleTypeDef;
# 492 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_usart.h"
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);
# 512 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_usart.h"
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_TransmitReceive(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                            uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                               uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, const uint8_t *pTxData, uint8_t *pRxData,
                                                uint16_t Size);
HAL_StatusTypeDef HAL_USART_DMAPause(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAResume(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAStop(USART_HandleTypeDef *husart);

HAL_StatusTypeDef HAL_USART_Abort(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_Abort_IT(USART_HandleTypeDef *husart);

void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);
void HAL_USART_AbortCpltCallback(USART_HandleTypeDef *husart);
# 547 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_usart.h"
HAL_USART_StateTypeDef HAL_USART_GetState(const USART_HandleTypeDef *husart);
uint32_t HAL_USART_GetError(const USART_HandleTypeDef *husart);
# 421 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h" 1
# 45 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h"
typedef struct
{
  uint32_t BaudRate;




  uint32_t WordLength;


  uint32_t Parity;






  uint32_t Mode;


  uint8_t Prescaler;




  uint32_t IrDAMode;

} IRDA_InitTypeDef;
# 113 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h"
typedef enum
{
  HAL_IRDA_STATE_RESET = 0x00U,

  HAL_IRDA_STATE_READY = 0x20U,

  HAL_IRDA_STATE_BUSY = 0x24U,

  HAL_IRDA_STATE_BUSY_TX = 0x21U,

  HAL_IRDA_STATE_BUSY_RX = 0x22U,

  HAL_IRDA_STATE_BUSY_TX_RX = 0x23U,


  HAL_IRDA_STATE_TIMEOUT = 0xA0U,

  HAL_IRDA_STATE_ERROR = 0xE0U

} HAL_IRDA_StateTypeDef;







typedef struct

{
  USART_TypeDef *Instance;

  IRDA_InitTypeDef Init;

  const uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_IRDA_StateTypeDef gState;



  volatile HAL_IRDA_StateTypeDef RxState;


  volatile uint32_t ErrorCode;
# 197 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h"
} IRDA_HandleTypeDef;
# 543 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h"
HAL_StatusTypeDef HAL_IRDA_Init(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DeInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspDeInit(IRDA_HandleTypeDef *hirda);
# 562 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h"
HAL_StatusTypeDef HAL_IRDA_Transmit(IRDA_HandleTypeDef *hirda, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Receive(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Transmit_DMA(IRDA_HandleTypeDef *hirda, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_DMAPause(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAResume(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAStop(IRDA_HandleTypeDef *hirda);

HAL_StatusTypeDef HAL_IRDA_Abort(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortTransmit(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortReceive(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_Abort_IT(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortTransmit_IT(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_AbortReceive_IT(IRDA_HandleTypeDef *hirda);

void HAL_IRDA_IRQHandler(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_AbortCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_AbortTransmitCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_AbortReceiveCpltCallback(IRDA_HandleTypeDef *hirda);
# 596 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_irda.h"
HAL_IRDA_StateTypeDef HAL_IRDA_GetState(const IRDA_HandleTypeDef *hirda);
uint32_t HAL_IRDA_GetError(const IRDA_HandleTypeDef *hirda);
# 425 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h" 1
# 46 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h"
typedef struct
{
  uint32_t BaudRate;




  uint32_t WordLength;


  uint32_t StopBits;


  uint32_t Parity;






  uint32_t Mode;


  uint32_t CLKPolarity;


  uint32_t CLKPhase;


  uint32_t CLKLastBit;



  uint32_t Prescaler;




  uint32_t GuardTime;

  uint32_t NACKState;

}SMARTCARD_InitTypeDef;
# 129 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h"
typedef enum
{
  HAL_SMARTCARD_STATE_RESET = 0x00U,

  HAL_SMARTCARD_STATE_READY = 0x20U,

  HAL_SMARTCARD_STATE_BUSY = 0x24U,

  HAL_SMARTCARD_STATE_BUSY_TX = 0x21U,

  HAL_SMARTCARD_STATE_BUSY_RX = 0x22U,

  HAL_SMARTCARD_STATE_BUSY_TX_RX = 0x23U,


  HAL_SMARTCARD_STATE_TIMEOUT = 0xA0U,

  HAL_SMARTCARD_STATE_ERROR = 0xE0U

}HAL_SMARTCARD_StateTypeDef;




typedef struct __SMARTCARD_HandleTypeDef
{
  USART_TypeDef *Instance;

  SMARTCARD_InitTypeDef Init;

  const uint8_t *pTxBuffPtr;

  uint16_t TxXferSize;

  volatile uint16_t TxXferCount;

  uint8_t *pRxBuffPtr;

  uint16_t RxXferSize;

  volatile uint16_t RxXferCount;

  DMA_HandleTypeDef *hdmatx;

  DMA_HandleTypeDef *hdmarx;

  HAL_LockTypeDef Lock;

  volatile HAL_SMARTCARD_StateTypeDef gState;



  volatile HAL_SMARTCARD_StateTypeDef RxState;


  volatile uint32_t ErrorCode;
# 204 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h"
} SMARTCARD_HandleTypeDef;
# 627 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h"
HAL_StatusTypeDef HAL_SMARTCARD_Init(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_ReInit(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_DeInit(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_MspInit(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_MspDeInit(SMARTCARD_HandleTypeDef *hsc);
# 645 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h"
HAL_StatusTypeDef HAL_SMARTCARD_Transmit(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Receive(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_DMA(SMARTCARD_HandleTypeDef *hsc, const uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_SMARTCARD_Abort(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_AbortTransmit(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_AbortReceive(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_Abort_IT(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_AbortTransmit_IT(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_AbortReceive_IT(SMARTCARD_HandleTypeDef *hsc);

void HAL_SMARTCARD_IRQHandler(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_AbortCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_AbortTransmitCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_AbortReceiveCpltCallback(SMARTCARD_HandleTypeDef *hsc);
# 674 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_smartcard.h"
HAL_SMARTCARD_StateTypeDef HAL_SMARTCARD_GetState(const SMARTCARD_HandleTypeDef *hsc);
uint32_t HAL_SMARTCARD_GetError(const SMARTCARD_HandleTypeDef *hsc);
# 429 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_wwdg.h" 1
# 47 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_wwdg.h"
typedef struct
{
  uint32_t Prescaler;


  uint32_t Window;


  uint32_t Counter;


  uint32_t EWIMode ;


} WWDG_InitTypeDef;







typedef struct

{
  WWDG_TypeDef *Instance;

  WWDG_InitTypeDef Init;






} WWDG_HandleTypeDef;
# 258 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_wwdg.h"
HAL_StatusTypeDef HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);
void HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);
# 275 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_wwdg.h"
HAL_StatusTypeDef HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);
void HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);
void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg);
# 433 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h" 1
# 28 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_usb.h" 1
# 52 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_usb.h"
typedef enum
{
  USB_DEVICE_MODE = 0,
  USB_HOST_MODE = 1,
  USB_DRD_MODE = 2
} USB_ModeTypeDef;




typedef enum
{
  URB_IDLE = 0,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
} USB_URBStateTypeDef;




typedef enum
{
  HC_IDLE = 0,
  HC_XFRC,
  HC_HALTED,
  HC_ACK,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,
  HC_BBLERR,
  HC_DATATGLERR
} USB_HCStateTypeDef;





typedef struct
{
  uint8_t dev_endpoints;



  uint8_t Host_channels;



  uint8_t dma_enable;


  uint8_t speed;



  uint8_t ep0_mps;

  uint8_t phy_itface;


  uint8_t Sof_enable;

  uint8_t low_power_enable;

  uint8_t lpm_enable;

  uint8_t battery_charging_enable;

  uint8_t vbus_sensing_enable;

  uint8_t use_dedicated_ep1;

  uint8_t use_external_vbus;

} USB_CfgTypeDef;

typedef struct
{
  uint8_t num;


  uint8_t is_in;


  uint8_t is_stall;


  uint8_t is_iso_incomplete;


  uint8_t type;


  uint8_t data_pid_start;


  uint32_t maxpacket;


  uint8_t *xfer_buff;

  uint32_t xfer_len;

  uint32_t xfer_count;

  uint8_t even_odd_frame;


  uint16_t tx_fifo_num;


  uint32_t dma_addr;

  uint32_t xfer_size;
} USB_EPTypeDef;

typedef struct
{
  uint8_t dev_addr;


  uint8_t ch_num;


  uint8_t ep_num;


  uint8_t ep_is_in;


  uint8_t speed;



  uint8_t do_ping;
  uint8_t do_ssplit;
  uint8_t do_csplit;
  uint8_t ep_ss_schedule;
  uint32_t iso_splt_xactPos;

  uint8_t hub_port_nbr;
  uint8_t hub_addr;

  uint8_t ep_type;


  uint16_t max_packet;


  uint8_t data_pid;


  uint8_t *xfer_buff;

  uint32_t XferSize;

  uint32_t xfer_len;

  uint32_t xfer_count;

  uint8_t toggle_in;


  uint8_t toggle_out;


  uint32_t dma_addr;

  uint32_t ErrCnt;
  uint32_t NyetErrCnt;

  USB_URBStateTypeDef urb_state;


  USB_HCStateTypeDef state;

} USB_HCTypeDef;

typedef USB_ModeTypeDef USB_OTG_ModeTypeDef;
typedef USB_CfgTypeDef USB_OTG_CfgTypeDef;
typedef USB_EPTypeDef USB_OTG_EPTypeDef;
typedef USB_URBStateTypeDef USB_OTG_URBStateTypeDef;
typedef USB_HCStateTypeDef USB_OTG_HCStateTypeDef;
typedef USB_HCTypeDef USB_OTG_HCTypeDef;
# 499 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_ll_usb.h"
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_SetTurnaroundTime(USB_OTG_GlobalTypeDef *USBx, uint32_t hclk, uint8_t speed);
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx, USB_OTG_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(const USB_OTG_GlobalTypeDef *USBx, uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo(USB_OTG_GlobalTypeDef *USBx, uint32_t num);
HAL_StatusTypeDef USB_ActivateEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_WritePacket(const USB_OTG_GlobalTypeDef *USBx, uint8_t *src,
                                  uint8_t ch_ep_num, uint16_t len, uint8_t dma);

void *USB_ReadPacket(const USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(const USB_OTG_GlobalTypeDef *USBx, const USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStopXfer(const USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress(const USB_OTG_GlobalTypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect(const USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect(const USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateSetup(const USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(const USB_OTG_GlobalTypeDef *USBx, uint8_t dma, const uint8_t *psetup);
uint8_t USB_GetDevSpeed(const USB_OTG_GlobalTypeDef *USBx);
uint32_t USB_GetMode(const USB_OTG_GlobalTypeDef *USBx);
uint32_t USB_ReadInterrupts(USB_OTG_GlobalTypeDef const *USBx);
uint32_t USB_ReadChInterrupts(const USB_OTG_GlobalTypeDef *USBx, uint8_t chnum);
uint32_t USB_ReadDevAllOutEpInterrupt(const USB_OTG_GlobalTypeDef *USBx);
uint32_t USB_ReadDevOutEPInterrupt(const USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);
uint32_t USB_ReadDevAllInEpInterrupt(const USB_OTG_GlobalTypeDef *USBx);
uint32_t USB_ReadDevInEPInterrupt(const USB_OTG_GlobalTypeDef *USBx, uint8_t epnum);
void USB_ClearInterrupts(USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_HostInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_InitFSLSPClkSel(const USB_OTG_GlobalTypeDef *USBx, uint8_t freq);
HAL_StatusTypeDef USB_ResetPort(const USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DriveVbus(const USB_OTG_GlobalTypeDef *USBx, uint8_t state);
uint32_t USB_GetHostSpeed(USB_OTG_GlobalTypeDef const *USBx);
uint32_t USB_GetCurrentFrame(USB_OTG_GlobalTypeDef const *USBx);
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num,
                              uint8_t epnum, uint8_t dev_address, uint8_t speed,
                              uint8_t ep_type, uint16_t mps);
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx,
                                   USB_OTG_HCTypeDef *hc, uint8_t dma);

uint32_t USB_HC_ReadInterrupt(const USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Halt(const USB_OTG_GlobalTypeDef *USBx, uint8_t hc_num);
HAL_StatusTypeDef USB_DoPing(const USB_OTG_GlobalTypeDef *USBx, uint8_t ch_num);
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateRemoteWakeup(const USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DeActivateRemoteWakeup(const USB_OTG_GlobalTypeDef *USBx);
# 29 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h" 2
# 48 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
typedef enum
{
  HAL_PCD_STATE_RESET = 0x00,
  HAL_PCD_STATE_READY = 0x01,
  HAL_PCD_STATE_ERROR = 0x02,
  HAL_PCD_STATE_BUSY = 0x03,
  HAL_PCD_STATE_TIMEOUT = 0x04
} PCD_StateTypeDef;


typedef enum
{
  LPM_L0 = 0x00,
  LPM_L1 = 0x01,
  LPM_L2 = 0x02,
  LPM_L3 = 0x03,
} PCD_LPM_StateTypeDef;

typedef enum
{
  PCD_LPM_L0_ACTIVE = 0x00,
  PCD_LPM_L1_ACTIVE = 0x01,
} PCD_LPM_MsgTypeDef;

typedef enum
{
  PCD_BCD_ERROR = 0xFF,
  PCD_BCD_CONTACT_DETECTION = 0xFE,
  PCD_BCD_STD_DOWNSTREAM_PORT = 0xFD,
  PCD_BCD_CHARGING_DOWNSTREAM_PORT = 0xFC,
  PCD_BCD_DEDICATED_CHARGING_PORT = 0xFB,
  PCD_BCD_DISCOVERY_COMPLETED = 0x00,

} PCD_BCD_MsgTypeDef;


typedef USB_OTG_GlobalTypeDef PCD_TypeDef;
typedef USB_OTG_CfgTypeDef PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef PCD_EPTypeDef;
# 95 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
typedef struct

{
  PCD_TypeDef *Instance;
  PCD_InitTypeDef Init;
  volatile uint8_t USB_Address;
  PCD_EPTypeDef IN_ep[16];
  PCD_EPTypeDef OUT_ep[16];
  HAL_LockTypeDef Lock;
  volatile PCD_StateTypeDef State;
  volatile uint32_t ErrorCode;
  uint32_t Setup[12];
  PCD_LPM_StateTypeDef LPM_State;
  uint32_t BESL;
  uint32_t FrameNumber;


  uint32_t lpm_active;


  uint32_t battery_charging_active;

  void *pData;
# 138 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
} PCD_HandleTypeDef;






# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd_ex.h" 1
# 50 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd_ex.h"
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size);
# 69 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd_ex.h"
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);
void HAL_PCDEx_BCD_Callback(PCD_HandleTypeDef *hpcd, PCD_BCD_MsgTypeDef msg);
# 146 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h" 2
# 248 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);
# 333 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);
void HAL_PCD_WKUP_IRQHandler(PCD_HandleTypeDef *hpcd);

void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
# 358 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Abort(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);

HAL_StatusTypeDef HAL_PCD_SetTestMode(const PCD_HandleTypeDef *hpcd, uint8_t testmode);


uint32_t HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef const *hpcd, uint8_t ep_addr);
# 384 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_pcd.h"
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef const *hpcd);
# 437 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h" 1
# 47 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
typedef enum
{
  HAL_HCD_STATE_RESET = 0x00,
  HAL_HCD_STATE_READY = 0x01,
  HAL_HCD_STATE_ERROR = 0x02,
  HAL_HCD_STATE_BUSY = 0x03,
  HAL_HCD_STATE_TIMEOUT = 0x04
} HCD_StateTypeDef;

typedef USB_OTG_GlobalTypeDef HCD_TypeDef;
typedef USB_OTG_CfgTypeDef HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef HCD_HCTypeDef;
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef;
typedef USB_OTG_HCStateTypeDef HCD_HCStateTypeDef;
# 71 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
typedef struct

{
  HCD_TypeDef *Instance;
  HCD_InitTypeDef Init;
  HCD_HCTypeDef hc[16];
  HAL_LockTypeDef Lock;
  volatile HCD_StateTypeDef State;
  volatile uint32_t ErrorCode;
  void *pData;
# 93 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
} HCD_HandleTypeDef;
# 189 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
HAL_StatusTypeDef HAL_HCD_Init(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_DeInit(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                  uint8_t epnum, uint8_t dev_address,
                                  uint8_t speed, uint8_t ep_type, uint16_t mps);

HAL_StatusTypeDef HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);
void HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);
# 253 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
HAL_StatusTypeDef HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                           uint8_t direction, uint8_t ep_type,
                                           uint8_t token, uint8_t *pbuff,
                                           uint16_t length, uint8_t do_ping);

HAL_StatusTypeDef HAL_HCD_HC_SetHubInfo(HCD_HandleTypeDef *hhcd, uint8_t ch_num,
                                        uint8_t addr, uint8_t PortNbr);

HAL_StatusTypeDef HAL_HCD_HC_ClearHubInfo(HCD_HandleTypeDef *hhcd, uint8_t ch_num);


void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void HAL_HCD_WKUP_IRQHandler(HCD_HandleTypeDef *hhcd);
void HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_PortEnabled_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_PortDisabled_Callback(HCD_HandleTypeDef *hhcd);
void HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, uint8_t chnum,
                                         HCD_URBStateTypeDef urb_state);
# 281 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
HAL_StatusTypeDef HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_Start(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef HAL_HCD_Stop(HCD_HandleTypeDef *hhcd);
# 292 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_hcd.h"
HCD_StateTypeDef HAL_HCD_GetState(HCD_HandleTypeDef const *hhcd);
HCD_URBStateTypeDef HAL_HCD_HC_GetURBState(HCD_HandleTypeDef const *hhcd, uint8_t chnum);
HCD_HCStateTypeDef HAL_HCD_HC_GetState(HCD_HandleTypeDef const *hhcd, uint8_t chnum);
uint32_t HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef const *hhcd, uint8_t chnum);
uint32_t HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);
# 441 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dsi.h" 1
# 445 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_qspi.h" 1
# 449 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_cec.h" 1
# 453 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_fmpi2c.h" 1
# 457 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_fmpsmbus.h" 1
# 461 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_spdifrx.h" 1
# 465 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_dfsdm.h" 1
# 469 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_lptim.h" 1
# 473 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2



# 1 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h" 1
# 48 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
typedef enum
{
  HAL_MMC_STATE_RESET = 0x00000000U,
  HAL_MMC_STATE_READY = 0x00000001U,
  HAL_MMC_STATE_TIMEOUT = 0x00000002U,
  HAL_MMC_STATE_BUSY = 0x00000003U,
  HAL_MMC_STATE_PROGRAMMING = 0x00000004U,
  HAL_MMC_STATE_RECEIVING = 0x00000005U,
  HAL_MMC_STATE_TRANSFER = 0x00000006U,
  HAL_MMC_STATE_ERROR = 0x0000000FU
}HAL_MMC_StateTypeDef;







typedef uint32_t HAL_MMC_CardStateTypeDef;
# 90 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
typedef struct
{
  uint32_t CardType;

  uint32_t Class;

  uint32_t RelCardAdd;

  uint32_t BlockNbr;

  uint32_t BlockSize;

  uint32_t LogBlockNbr;

  uint32_t LogBlockSize;

}HAL_MMC_CardInfoTypeDef;







typedef struct

{
  SDIO_TypeDef *Instance;

  SDIO_InitTypeDef Init;

  HAL_LockTypeDef Lock;

  uint8_t *pTxBuffPtr;

  uint32_t TxXferSize;

  uint8_t *pRxBuffPtr;

  uint32_t RxXferSize;

  volatile uint32_t Context;

  volatile HAL_MMC_StateTypeDef State;

  volatile uint32_t ErrorCode;

  DMA_HandleTypeDef *hdmarx;

  DMA_HandleTypeDef *hdmatx;

  HAL_MMC_CardInfoTypeDef MmcCard;

  uint32_t CSD[4U];

  uint32_t CID[4U];

  uint32_t Ext_CSD[128];
# 158 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
}MMC_HandleTypeDef;
# 167 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
typedef struct
{
  volatile uint8_t CSDStruct;
  volatile uint8_t SysSpecVersion;
  volatile uint8_t Reserved1;
  volatile uint8_t TAAC;
  volatile uint8_t NSAC;
  volatile uint8_t MaxBusClkFrec;
  volatile uint16_t CardComdClasses;
  volatile uint8_t RdBlockLen;
  volatile uint8_t PartBlockRead;
  volatile uint8_t WrBlockMisalign;
  volatile uint8_t RdBlockMisalign;
  volatile uint8_t DSRImpl;
  volatile uint8_t Reserved2;
  volatile uint32_t DeviceSize;
  volatile uint8_t MaxRdCurrentVDDMin;
  volatile uint8_t MaxRdCurrentVDDMax;
  volatile uint8_t MaxWrCurrentVDDMin;
  volatile uint8_t MaxWrCurrentVDDMax;
  volatile uint8_t DeviceSizeMul;
  volatile uint8_t EraseGrSize;
  volatile uint8_t EraseGrMul;
  volatile uint8_t WrProtectGrSize;
  volatile uint8_t WrProtectGrEnable;
  volatile uint8_t ManDeflECC;
  volatile uint8_t WrSpeedFact;
  volatile uint8_t MaxWrBlockLen;
  volatile uint8_t WriteBlockPaPartial;
  volatile uint8_t Reserved3;
  volatile uint8_t ContentProtectAppli;
  volatile uint8_t FileFormatGroup;
  volatile uint8_t CopyFlag;
  volatile uint8_t PermWrProtect;
  volatile uint8_t TempWrProtect;
  volatile uint8_t FileFormat;
  volatile uint8_t ECC;
  volatile uint8_t CSD_CRC;
  volatile uint8_t Reserved4;

}HAL_MMC_CardCSDTypeDef;







typedef struct
{
  volatile uint8_t ManufacturerID;
  volatile uint16_t OEM_AppliID;
  volatile uint32_t ProdName1;
  volatile uint8_t ProdName2;
  volatile uint8_t ProdRev;
  volatile uint32_t ProdSN;
  volatile uint8_t Reserved1;
  volatile uint16_t ManufactDate;
  volatile uint8_t CID_CRC;
  volatile uint8_t Reserved2;

}HAL_MMC_CardCIDTypeDef;
# 586 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
HAL_StatusTypeDef HAL_MMC_Init(MMC_HandleTypeDef *hmmc);
HAL_StatusTypeDef HAL_MMC_InitCard(MMC_HandleTypeDef *hmmc);
HAL_StatusTypeDef HAL_MMC_DeInit (MMC_HandleTypeDef *hmmc);
void HAL_MMC_MspInit(MMC_HandleTypeDef *hmmc);
void HAL_MMC_MspDeInit(MMC_HandleTypeDef *hmmc);
# 600 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
HAL_StatusTypeDef HAL_MMC_ReadBlocks(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_MMC_WriteBlocks(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks, uint32_t Timeout);
HAL_StatusTypeDef HAL_MMC_Erase(MMC_HandleTypeDef *hmmc, uint32_t BlockStartAdd, uint32_t BlockEndAdd);

HAL_StatusTypeDef HAL_MMC_ReadBlocks_IT(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_MMC_WriteBlocks_IT(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

HAL_StatusTypeDef HAL_MMC_ReadBlocks_DMA(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);
HAL_StatusTypeDef HAL_MMC_WriteBlocks_DMA(MMC_HandleTypeDef *hmmc, uint8_t *pData, uint32_t BlockAdd, uint32_t NumberOfBlocks);

void HAL_MMC_IRQHandler(MMC_HandleTypeDef *hmmc);


void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hmmc);
void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hmmc);
void HAL_MMC_ErrorCallback(MMC_HandleTypeDef *hmmc);
void HAL_MMC_AbortCallback(MMC_HandleTypeDef *hmmc);
# 630 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_mmc.h"
HAL_StatusTypeDef HAL_MMC_ConfigWideBusOperation(MMC_HandleTypeDef *hmmc, uint32_t WideMode);







HAL_MMC_CardStateTypeDef HAL_MMC_GetCardState(MMC_HandleTypeDef *hmmc);
HAL_StatusTypeDef HAL_MMC_GetCardCID(MMC_HandleTypeDef *hmmc, HAL_MMC_CardCIDTypeDef *pCID);
HAL_StatusTypeDef HAL_MMC_GetCardCSD(MMC_HandleTypeDef *hmmc, HAL_MMC_CardCSDTypeDef *pCSD);
HAL_StatusTypeDef HAL_MMC_GetCardInfo(MMC_HandleTypeDef *hmmc, HAL_MMC_CardInfoTypeDef *pCardInfo);
HAL_StatusTypeDef HAL_MMC_GetCardExtCSD(MMC_HandleTypeDef *hmmc, uint32_t *pExtCSD, uint32_t Timeout);







HAL_MMC_StateTypeDef HAL_MMC_GetState(MMC_HandleTypeDef *hmmc);
uint32_t HAL_MMC_GetError(MMC_HandleTypeDef *hmmc);







HAL_StatusTypeDef HAL_MMC_Abort(MMC_HandleTypeDef *hmmc);
HAL_StatusTypeDef HAL_MMC_Abort_IT(MMC_HandleTypeDef *hmmc);
# 477 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal_conf.h" 2
# 30 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h" 2
# 49 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h"
typedef enum
{
  HAL_TICK_FREQ_10HZ = 100U,
  HAL_TICK_FREQ_100HZ = 10U,
  HAL_TICK_FREQ_1KHZ = 1U,
  HAL_TICK_FREQ_DEFAULT = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;
# 204 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h"
extern volatile uint32_t uwTick;
extern uint32_t uwTickPrio;
extern HAL_TickFreqTypeDef uwTickFreq;
# 219 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h"
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);
# 232 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/drivers/include/stm32f4xx_hal.h"
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);
uint32_t HAL_GetUIDw0(void);
uint32_t HAL_GetUIDw1(void);
uint32_t HAL_GetUIDw2(void);
# 288 "/home/ttwards/zephyrproject/modules/hal/stm32/stm32cube/stm32f4xx/soc/stm32f4xx.h" 2
# 24 "/home/ttwards/zephyrproject/zephyr/soc/st/stm32/stm32f4x/./soc.h" 2
# 25 "/home/ttwards/zephyrproject/zephyr/modules/cmsis/./cmsis_core_m.h" 2
# 11 "/home/ttwards/zephyrproject/zephyr/modules/cmsis/./cmsis_core.h" 2
# 25 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h" 2
# 44 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h"
static inline __attribute__((always_inline)) unsigned int arch_irq_lock(void)
{
 unsigned int key;
# 56 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h"
 key = __get_BASEPRI();
 __set_BASEPRI_MAX(((((1 + 0)) << (8 - 4)) & 0xff));
 __ISB();
# 72 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h"
 return key;
}






static inline __attribute__((always_inline)) void arch_irq_unlock(unsigned int key)
{







 __set_BASEPRI(key);
 __ISB();
# 100 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h"
}

static inline __attribute__((always_inline)) 
# 102 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h" 3 4
                    _Bool 
# 102 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline_gcc.h"
                         arch_irq_unlocked(unsigned int key)
{

 return key == 0U;
}
# 19 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/asm_inline.h" 2
# 31 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/sys_bitops.h" 1
# 24 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/sys_bitops.h"
static inline __attribute__((always_inline)) void sys_set_bit(mem_addr_t addr, unsigned int bit)
{
 uint32_t temp = *(volatile uint32_t *)addr;

 *(volatile uint32_t *)addr = temp | (1 << bit);
}

static inline __attribute__((always_inline)) void sys_clear_bit(mem_addr_t addr, unsigned int bit)
{
 uint32_t temp = *(volatile uint32_t *)addr;

 *(volatile uint32_t *)addr = temp & ~(1 << bit);
}

static inline __attribute__((always_inline)) int sys_test_bit(mem_addr_t addr, unsigned int bit)
{
 uint32_t temp = *(volatile uint32_t *)addr;

 return temp & (1 << bit);
}

static inline __attribute__((always_inline)) void sys_set_bits(mem_addr_t addr, unsigned int mask)
{
 uint32_t temp = *(volatile uint32_t *)addr;

 *(volatile uint32_t *)addr = temp | mask;
}

static inline __attribute__((always_inline)) void sys_clear_bits(mem_addr_t addr, unsigned int mask)
{
 uint32_t temp = *(volatile uint32_t *)addr;

 *(volatile uint32_t *)addr = temp & ~mask;
}

static inline __attribute__((always_inline))
 void sys_bitfield_set_bit(mem_addr_t addr, unsigned int bit)
{



 sys_set_bit(addr + ((bit >> 5) << 2), bit & 0x1F);
}

static inline __attribute__((always_inline))
 void sys_bitfield_clear_bit(mem_addr_t addr, unsigned int bit)
{
 sys_clear_bit(addr + ((bit >> 5) << 2), bit & 0x1F);
}

static inline __attribute__((always_inline))
 int sys_bitfield_test_bit(mem_addr_t addr, unsigned int bit)
{
 return sys_test_bit(addr + ((bit >> 5) << 2), bit & 0x1F);
}

static inline __attribute__((always_inline))
 int sys_test_and_set_bit(mem_addr_t addr, unsigned int bit)
{
 int ret;

 ret = sys_test_bit(addr, bit);
 sys_set_bit(addr, bit);

 return ret;
}

static inline __attribute__((always_inline))
 int sys_test_and_clear_bit(mem_addr_t addr, unsigned int bit)
{
 int ret;

 ret = sys_test_bit(addr, bit);
 sys_clear_bit(addr, bit);

 return ret;
}

static inline __attribute__((always_inline))
 int sys_bitfield_test_and_set_bit(mem_addr_t addr, unsigned int bit)
{
 int ret;

 ret = sys_bitfield_test_bit(addr, bit);
 sys_bitfield_set_bit(addr, bit);

 return ret;
}

static inline __attribute__((always_inline))
 int sys_bitfield_test_and_clear_bit(mem_addr_t addr, unsigned int bit)
{
 int ret;

 ret = sys_bitfield_test_bit(addr, bit);
 sys_bitfield_clear_bit(addr, bit);

 return ret;
}
# 32 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2





# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/cpu.h" 1
# 38 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/cortex_m/memory_map.h" 1
# 39 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/sys_io.h" 1
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/common/sys_io.h"
static inline __attribute__((always_inline)) uint8_t sys_read8(mem_addr_t addr)
{
 return *(volatile uint8_t *)addr;
}

static inline __attribute__((always_inline)) void sys_write8(uint8_t data, mem_addr_t addr)
{
 *(volatile uint8_t *)addr = data;
}

static inline __attribute__((always_inline)) uint16_t sys_read16(mem_addr_t addr)
{
 return *(volatile uint16_t *)addr;
}

static inline __attribute__((always_inline)) void sys_write16(uint16_t data, mem_addr_t addr)
{
 *(volatile uint16_t *)addr = data;
}

static inline __attribute__((always_inline)) uint32_t sys_read32(mem_addr_t addr)
{
 return *(volatile uint32_t *)addr;
}

static inline __attribute__((always_inline)) void sys_write32(uint32_t data, mem_addr_t addr)
{
 *(volatile uint32_t *)addr = data;
}

static inline __attribute__((always_inline)) uint64_t sys_read64(mem_addr_t addr)
{
 return *(volatile uint64_t *)addr;
}

static inline __attribute__((always_inline)) void sys_write64(uint64_t data, mem_addr_t addr)
{
 *(volatile uint64_t *)addr = data;
}
# 40 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2
# 57 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal_types.h" 1
# 24 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal_types.h"
enum k_fatal_error_reason {

 K_ERR_CPU_EXCEPTION,


 K_ERR_SPURIOUS_IRQ,


 K_ERR_STACK_CHK_FAIL,


 K_ERR_KERNEL_OOPS,


 K_ERR_KERNEL_PANIC,


 K_ERR_ARCH_START = 16
};
# 58 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/arm/arch.h" 2

enum k_fatal_error_reason_arch {

 K_ERR_ARM_MEM_GENERIC = K_ERR_ARCH_START,
 K_ERR_ARM_MEM_STACKING,
 K_ERR_ARM_MEM_UNSTACKING,
 K_ERR_ARM_MEM_DATA_ACCESS,
 K_ERR_ARM_MEM_INSTRUCTION_ACCESS,
 K_ERR_ARM_MEM_FP_LAZY_STATE_PRESERVATION,


 K_ERR_ARM_BUS_GENERIC,
 K_ERR_ARM_BUS_STACKING,
 K_ERR_ARM_BUS_UNSTACKING,
 K_ERR_ARM_BUS_PRECISE_DATA_BUS,
 K_ERR_ARM_BUS_IMPRECISE_DATA_BUS,
 K_ERR_ARM_BUS_INSTRUCTION_BUS,
 K_ERR_ARM_BUS_FP_LAZY_STATE_PRESERVATION,


 K_ERR_ARM_USAGE_GENERIC,
 K_ERR_ARM_USAGE_DIV_0,
 K_ERR_ARM_USAGE_UNALIGNED_ACCESS,
 K_ERR_ARM_USAGE_STACK_OVERFLOW,
 K_ERR_ARM_USAGE_NO_COPROCESSOR,
 K_ERR_ARM_USAGE_ILLEGAL_EXC_RETURN,
 K_ERR_ARM_USAGE_ILLEGAL_EPSR,
 K_ERR_ARM_USAGE_UNDEFINED_INSTRUCTION,


 K_ERR_ARM_SECURE_GENERIC,
 K_ERR_ARM_SECURE_ENTRY_POINT,
 K_ERR_ARM_SECURE_INTEGRITY_SIGNATURE,
 K_ERR_ARM_SECURE_EXCEPTION_RETURN,
 K_ERR_ARM_SECURE_ATTRIBUTION_UNIT,
 K_ERR_ARM_SECURE_TRANSITION,
 K_ERR_ARM_SECURE_LAZY_STATE_PRESERVATION,
 K_ERR_ARM_SECURE_LAZY_STATE_ERROR,


 K_ERR_ARM_UNDEFINED_INSTRUCTION,
 K_ERR_ARM_ALIGNMENT_FAULT,
 K_ERR_ARM_BACKGROUND_FAULT,
 K_ERR_ARM_PERMISSION_FAULT,
 K_ERR_ARM_SYNC_EXTERNAL_ABORT,
 K_ERR_ARM_ASYNC_EXTERNAL_ABORT,
 K_ERR_ARM_SYNC_PARITY_ERROR,
 K_ERR_ARM_ASYNC_PARITY_ERROR,
 K_ERR_ARM_DEBUG_EVENT,
 K_ERR_ARM_TRANSLATION_FAULT,
 K_ERR_ARM_UNSUPPORTED_EXCLUSIVE_ACCESS_FAULT
};
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/cpu.h" 2
# 37 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h" 1
# 46 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
typedef int64_t k_ticks_t;
# 65 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
typedef struct {
 k_ticks_t ticks;
} k_timeout_t;
# 191 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
uint32_t sys_clock_tick_get_32(void);
# 200 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
int64_t sys_clock_tick_get(void);
# 219 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
typedef struct { uint64_t tick; } k_timepoint_t;
# 237 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
k_timepoint_t sys_timepoint_calc(k_timeout_t timeout);
# 252 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
k_timeout_t sys_timepoint_timeout(k_timepoint_t timepoint);
# 264 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
static inline int sys_timepoint_cmp(k_timepoint_t a, k_timepoint_t b)
{
 if (a.tick == b.tick) {
  return 0;
 }
 return a.tick < b.tick ? -1 : 1;
}
# 312 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
static inline 
# 312 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h" 3 4
             _Bool 
# 312 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys_clock.h"
                  sys_timepoint_expired(k_timepoint_t timepoint)
{
 return ((sys_timepoint_timeout(timepoint)).ticks == (((k_timeout_t) {0})).ticks);
}
# 39 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h" 1
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/errno.h" 1 3 4
# 38 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/errno.h" 3 4

# 38 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/errno.h" 3 4
typedef int error_t;



# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/errno.h" 1 3 4
# 43 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/errno.h" 3 4
extern const char * const _sys_errlist[];
extern int _sys_nerr;
# 62 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/sys/errno.h" 3 4
extern __thread int errno;
# 43 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/errno.h" 2 3 4
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h" 2
# 34 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"

# 34 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
struct z_spinlock_key {
 int key;
};
# 45 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
struct k_spinlock {
# 101 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
};
# 130 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
typedef struct z_spinlock_key k_spinlock_key_t;

static inline __attribute__((always_inline)) void z_spinlock_validate_pre(struct k_spinlock *l)
{
 (void)(l);






}

static inline __attribute__((always_inline)) void z_spinlock_validate_post(struct k_spinlock *l)
{
 (void)(l);






}
# 182 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
static inline __attribute__((always_inline)) k_spinlock_key_t k_spin_lock(struct k_spinlock *l)
{
 (void)(l);
 k_spinlock_key_t k;





 k.key = arch_irq_lock();

 z_spinlock_validate_pre(l);
# 211 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
 z_spinlock_validate_post(l);

 return k;
}
# 230 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
static inline __attribute__((always_inline)) int k_spin_trylock(struct k_spinlock *l, k_spinlock_key_t *k)
{
 int key = arch_irq_lock();

 z_spinlock_validate_pre(l);
# 266 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
 z_spinlock_validate_post(l);

 k->key = key;

 return 0;






}
# 300 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
static inline __attribute__((always_inline)) void k_spin_unlock(struct k_spinlock *l,
     k_spinlock_key_t key)
{
 (void)(l);
# 331 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
 arch_irq_unlock(key.key);
}
# 359 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
static inline __attribute__((always_inline)) void k_spin_release(struct k_spinlock *l)
{
 (void)(l);
# 372 "/home/ttwards/zephyrproject/zephyr/include/zephyr/spinlock.h"
}
# 40 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal.h" 1
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/arch/exception.h" 1
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal.h" 2
# 37 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal.h"
__attribute__((__noreturn__)) void k_fatal_halt(unsigned int reason);
# 68 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal.h"
void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf);
# 84 "/home/ttwards/zephyrproject/zephyr/include/zephyr/fatal.h"
void z_fatal_error(unsigned int reason, const struct arch_esf *esf);
# 41 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq.h" 1
# 64 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq.h"
static inline int
irq_connect_dynamic(unsigned int irq, unsigned int priority,
      void (*routine)(const void *parameter),
      const void *parameter, uint32_t flags)
{
 return arch_irq_connect_dynamic(irq, priority, routine, parameter,
     flags);
}
# 89 "/home/ttwards/zephyrproject/zephyr/include/zephyr/irq.h"
static inline int
irq_disconnect_dynamic(unsigned int irq, unsigned int priority,
         void (*routine)(const void *parameter),
         const void *parameter, uint32_t flags)
{
 return arch_irq_disconnect_dynamic(irq, priority, routine,
        parameter, flags);
}
# 42 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread_stack.h" 1
# 47 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread_stack.h"
struct 
# 47 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread_stack.h" 3 4
      __attribute__((__packed__)) 
# 47 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread_stack.h"
               z_thread_stack_element {
 char data;
};
# 69 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread_stack.h"
static inline char *z_stack_ptr_align(char *ptr)
{
 return (char *)(((unsigned long)(ptr) / (unsigned long)(8)) * (unsigned long)(8));
}
# 286 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread_stack.h"
static inline char *K_KERNEL_STACK_BUFFER(k_thread_stack_t *sym)
{
 return (char *)sym + ((size_t)0);
}
# 43 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h" 1
# 35 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
struct __thread_entry {
 k_thread_entry_t pEntry;
 void *parameter1;
 void *parameter2;
 void *parameter3;
};


struct k_thread;






struct _pipe_desc {
 sys_dnode_t node;
 unsigned char *buffer;
 size_t bytes_to_xfer;
 struct k_thread *thread;
};


struct _thread_base {


 union {
  sys_dnode_t qnode_dlist;
  struct rbnode qnode_rb;
 };




 _wait_q_t *pended_on;


 uint8_t user_options;


 uint8_t thread_state;
# 91 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 union {
  struct {




   int8_t prio;
   uint8_t sched_locked;

  };
  uint16_t preempt;
 };





 uint32_t order_key;
# 132 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 void *swap_data;



 struct _timeout timeout;
# 146 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 struct k_cycle_stats usage;

};

typedef struct _thread_base _thread_base_t;



struct _thread_stack_info {



 uintptr_t start;
# 167 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 size_t size;





 size_t delta;
# 184 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
};

typedef struct _thread_stack_info _thread_stack_info_t;
# 207 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
typedef struct k_thread_runtime_stats {





 uint64_t execution_cycles;
 uint64_t total_cycles;
# 237 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 uint64_t idle_cycles;
# 248 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
} k_thread_runtime_stats_t;

struct z_poller {
 
# 251 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h" 3 4
_Bool 
# 251 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
     is_polling;
 uint8_t mode;
};





struct k_thread {

 struct _thread_base base;


 struct _callee_saved callee_saved;


 void *init_data;


 _wait_q_t join_queue;






 struct k_thread *next_event_link;

 uint32_t events;
 uint32_t event_options;


 
# 283 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h" 3 4
_Bool 
# 283 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
     no_wake_on_timeout;




 struct __thread_entry entry;


 struct k_thread *next_thread;




 char name[32];
# 317 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 struct _thread_stack_info stack_info;
# 349 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 struct k_heap *resource_pool;



 uintptr_t tls;
# 376 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/thread.h"
 struct _thread_arch arch;
};

typedef struct k_thread _thread_t;
typedef struct k_thread *k_tid_t;
# 16 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h" 2
# 105 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h"
struct k_mem_domain;
struct k_mem_partition;
# 129 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h"
int k_mem_domain_init(struct k_mem_domain *domain, uint8_t num_parts,
        struct k_mem_partition *parts[]);
# 159 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h"
int k_mem_domain_add_partition(struct k_mem_domain *domain,
          struct k_mem_partition *part);
# 174 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h"
int k_mem_domain_remove_partition(struct k_mem_domain *domain,
      struct k_mem_partition *part);
# 188 "/home/ttwards/zephyrproject/zephyr/include/zephyr/app_memory/mem_domain.h"
int k_mem_domain_add_thread(struct k_mem_domain *domain,
       k_tid_t thread);
# 44 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h" 1
# 92 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h"
static inline void k_object_init(const void *obj)
{
 (void)(obj);
}
# 155 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h"
static inline struct k_object *k_object_create_dynamic_aligned(size_t align,
              size_t size)
{
 (void)(align);
 (void)(size);

 return 
# 161 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h" 3 4
       ((void *)0)
# 161 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h"
           ;
}

static inline struct k_object *k_object_create_dynamic(size_t size)
{
 (void)(size);

 return 
# 168 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h" 3 4
       ((void *)0)
# 168 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/internal/kobject_internal.h"
           ;
}
# 14 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 2





struct k_thread;
struct k_mutex;
struct z_futex_data;
# 30 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
enum k_objects {
 K_OBJ_ANY,







# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/kobj-types-enum.h" 1

K_OBJ_MEM_SLAB,
K_OBJ_MSGQ,
K_OBJ_MUTEX,
K_OBJ_PIPE,
K_OBJ_QUEUE,
K_OBJ_POLL_SIGNAL,
K_OBJ_SEM,
K_OBJ_STACK,
K_OBJ_THREAD,
K_OBJ_TIMER,
K_OBJ_THREAD_STACK_ELEMENT,
K_OBJ_NET_SOCKET,
K_OBJ_NET_IF,
K_OBJ_SYS_MUTEX,
K_OBJ_FUTEX,
K_OBJ_CONDVAR,

K_OBJ_EVENT,
# 43 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/kobj-types-enum.h"
K_OBJ_DRIVER_CAN,
K_OBJ_DRIVER_DMA,
K_OBJ_DRIVER_GPIO,
K_OBJ_DRIVER_RESET,
K_OBJ_DRIVER_SENSOR,
K_OBJ_DRIVER_UART,
K_OBJ_DRIVER_SPI,
K_OBJ_DRIVER_CHASSIS,
K_OBJ_DRIVER_MOTOR,
K_OBJ_DRIVER_SBUS,
K_OBJ_DRIVER_SHARED_IRQ,
K_OBJ_DRIVER_CRYPTO,
K_OBJ_DRIVER_ADC,
K_OBJ_DRIVER_AUXDISPLAY,
K_OBJ_DRIVER_BBRAM,
K_OBJ_DRIVER_BT_HCI,
K_OBJ_DRIVER_CELLULAR,
K_OBJ_DRIVER_CHARGER,
K_OBJ_DRIVER_CLOCK_CONTROL,
K_OBJ_DRIVER_COMPARATOR,
K_OBJ_DRIVER_COREDUMP,
K_OBJ_DRIVER_COUNTER,
K_OBJ_DRIVER_DAC,
K_OBJ_DRIVER_DAI,
K_OBJ_DRIVER_DISPLAY,
K_OBJ_DRIVER_EDAC,
K_OBJ_DRIVER_EEPROM,
K_OBJ_DRIVER_EMUL_BBRAM,
K_OBJ_DRIVER_FUEL_GAUGE_EMUL,
K_OBJ_DRIVER_EMUL_SENSOR,
K_OBJ_DRIVER_ENTROPY,
K_OBJ_DRIVER_ESPI,
K_OBJ_DRIVER_ESPI_SAF,
K_OBJ_DRIVER_FLASH,
K_OBJ_DRIVER_FPGA,
K_OBJ_DRIVER_FUEL_GAUGE,
K_OBJ_DRIVER_GNSS,
K_OBJ_DRIVER_HAPTICS,
K_OBJ_DRIVER_HWSPINLOCK,
K_OBJ_DRIVER_I2C,
K_OBJ_DRIVER_I2C_TARGET,
K_OBJ_DRIVER_I2S,
K_OBJ_DRIVER_I3C,
K_OBJ_DRIVER_IPM,
K_OBJ_DRIVER_KSCAN,
K_OBJ_DRIVER_LED,
K_OBJ_DRIVER_LED_STRIP,
K_OBJ_DRIVER_LORA,
K_OBJ_DRIVER_MBOX,
K_OBJ_DRIVER_MDIO,
K_OBJ_DRIVER_MIPI_DBI,
K_OBJ_DRIVER_MIPI_DSI,
K_OBJ_DRIVER_MSPI,
K_OBJ_DRIVER_PECI,
K_OBJ_DRIVER_PS2,
K_OBJ_DRIVER_PTP_CLOCK,
K_OBJ_DRIVER_PWM,
K_OBJ_DRIVER_REGULATOR_PARENT,
K_OBJ_DRIVER_REGULATOR,
K_OBJ_DRIVER_RETAINED_MEM,
K_OBJ_DRIVER_RTC,
K_OBJ_DRIVER_SDHC,
K_OBJ_DRIVER_SMBUS,
K_OBJ_DRIVER_STEPPER,
K_OBJ_DRIVER_SYSCON,
K_OBJ_DRIVER_TEE,
K_OBJ_DRIVER_VIDEO,
K_OBJ_DRIVER_W1,
K_OBJ_DRIVER_WDT,
K_OBJ_DRIVER_CAN_TRANSCEIVER,
K_OBJ_DRIVER_NRF_CLOCK_CONTROL,
K_OBJ_DRIVER_I3C_TARGET,
K_OBJ_DRIVER_ITS,
K_OBJ_DRIVER_VTD,
K_OBJ_DRIVER_TGPIO,
K_OBJ_DRIVER_PCIE_CTRL,
K_OBJ_DRIVER_PCIE_EP,
K_OBJ_DRIVER_SVC,
K_OBJ_DRIVER_BC12_EMUL,
K_OBJ_DRIVER_BC12,
K_OBJ_DRIVER_USBC_PPC,
K_OBJ_DRIVER_TCPC,
K_OBJ_DRIVER_USBC_VBUS,
K_OBJ_DRIVER_IVSHMEM,
K_OBJ_DRIVER_ETHPHY,
# 40 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 2



 K_OBJ_LAST
};
# 157 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
static inline void z_impl_k_object_access_grant(const void *object,
      struct k_thread *thread)
{
 (void)(object);
 (void)(thread);
}




static inline void k_object_access_revoke(const void *object,
       struct k_thread *thread)
{
 (void)(object);
 (void)(thread);
}




static inline void z_impl_k_object_release(const void *object)
{
 (void)(object);
}

static inline void k_object_access_all_grant(const void *object)
{
 (void)(object);
}

static inline 
# 187 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 3 4
             _Bool 
# 187 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
                  k_object_is_valid(const void *obj, enum k_objects otype)
{
 (void)(obj);
 (void)(otype);

 return 
# 192 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 3 4
       1
# 192 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
           ;
}
# 256 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
static inline void *z_impl_k_object_alloc(enum k_objects otype)
{
 (void)(otype);

 return 
# 260 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 3 4
       ((void *)0)
# 260 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
           ;
}

static inline void *z_impl_k_object_alloc_size(enum k_objects otype,
     size_t size)
{
 (void)(otype);
 (void)(size);

 return 
# 269 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 3 4
       ((void *)0)
# 269 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h"
           ;
}






static inline void k_object_free(void *obj)
{
 (void)(obj);
}





# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h" 1
# 23 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
extern void z_impl_k_object_access_grant(const void * object, struct k_thread * thread);


static inline void k_object_access_grant(const void * object, struct k_thread * thread)
{
# 36 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 36 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h" 3 4
0
# 36 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
);
 z_impl_k_object_access_grant(object, thread);
}
# 48 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
extern void z_impl_k_object_release(const void * object);


static inline void k_object_release(const void * object)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 60 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h" 3 4
0
# 60 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
);
 z_impl_k_object_release(object);
}
# 72 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
extern void * z_impl_k_object_alloc(enum k_objects otype);


static inline void * k_object_alloc(enum k_objects otype)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 83 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h" 3 4
0
# 83 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
);
 return z_impl_k_object_alloc(otype);
}
# 95 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
extern void * z_impl_k_object_alloc_size(enum k_objects otype, size_t size);


static inline void * k_object_alloc_size(enum k_objects otype, size_t size)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 107 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h" 3 4
0
# 107 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kobject.h"
);
 return z_impl_k_object_alloc_size(otype, size);
}
# 287 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/kobject.h" 2
# 45 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/smp.h" 1
# 9 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel/internal/smp.h"
void z_sched_ipi(void);
# 48 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel_includes.h" 2
# 18 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 2

# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/picolibc/include/limits.h" 1 3 4
# 20 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 2


# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing_macros.h" 1
# 23 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 2
# 34 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
_Static_assert((sizeof(int32_t) == sizeof(int)), "" );
_Static_assert((sizeof(int64_t) == sizeof(long long)), "" );
_Static_assert((sizeof(intptr_t) == sizeof(long)), "" );
# 71 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_thread;
struct k_mutex;
struct k_sem;
struct k_msgq;
struct k_mbox;
struct k_pipe;
struct k_queue;
struct k_fifo;
struct k_lifo;
struct k_stack;
struct k_mem_slab;
struct k_timer;
struct k_poll_event;
struct k_poll_signal;
struct k_mem_domain;
struct k_mem_partition;
struct k_futex;
struct k_event;

enum execution_context_types {
 K_ISR = 0,
 K_COOP_THREAD,
 K_PREEMPT_THREAD,
};


struct k_work_poll;
typedef int (*_poller_cb_t)(struct k_poll_event *event, uint32_t state);






typedef void (*k_thread_user_cb_t)(const struct k_thread *thread,
       void *user_data);
# 123 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_thread_foreach(k_thread_user_cb_t user_cb, void *user_data);
# 147 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline
void k_thread_foreach_filter_by_cpu(unsigned int cpu,
        k_thread_user_cb_t user_cb, void *user_data)
{
 { };
 (void)(cpu);
 k_thread_foreach(user_cb, user_data);
}
# 184 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_thread_foreach_unlocked(
 k_thread_user_cb_t user_cb, void *user_data);
# 222 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline
void k_thread_foreach_unlocked_filter_by_cpu(unsigned int cpu,
          k_thread_user_cb_t user_cb, void *user_data)
{
 { };
 (void)(cpu);
 k_thread_foreach_unlocked(user_cb, user_data);
}
# 345 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_thread_stack_t *k_thread_stack_alloc(size_t size, int flags);
# 359 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_thread_stack_free(k_thread_stack_t *stack);
# 409 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_tid_t k_thread_create(struct k_thread *new_thread,
      k_thread_stack_t *stack,
      size_t stack_size,
      k_thread_entry_t entry,
      void *p1, void *p2, void *p3,
      int prio, uint32_t options, k_timeout_t delay);
# 437 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
__attribute__((__noreturn__)) void k_thread_user_mode_enter(k_thread_entry_t entry,
         void *p1, void *p2,
         void *p3);
# 471 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_heap_assign(struct k_thread *thread,
     struct k_heap *heap)
{
 thread->resource_pool = heap;
}
# 498 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_thread_stack_space_get(const struct k_thread *thread,
           size_t *unused_ptr);
# 537 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_thread_join(struct k_thread *thread, k_timeout_t timeout);
# 553 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int32_t k_sleep(k_timeout_t timeout);
# 566 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int32_t k_msleep(int32_t ms)
{
 return k_sleep(((k_timeout_t) { .ticks = ((k_ticks_t)((
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               1
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? ( ((10000) == (1000)) ? (uint64_t) ((((ms) > (0)) ? (ms) : (0))) : ((1000) > (10000) && (1000) % (10000) == 0U) ? (((uint64_t) ((((ms) > (0)) ? (ms) : (0))) + ((
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               0
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? ((1000) / (10000)) / 2 : (
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               1
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? ((1000) / (10000)) - 1 : 0)) / ((1000)/(10000) ? (1000)/(10000) : 01u)) : ((10000) > (1000) && (10000) % (1000) == 0U) ? (uint64_t) ((((ms) > (0)) ? (ms) : (0)))*((10000) / (1000)) : ((((((365 * 24ULL * 3600ULL * 1000) + (
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               (0xffffffffUL)
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) - 1) / (
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               (0xffffffffUL)
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               )) * 10000) <= 
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               (0xffffffffUL)
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (((uint64_t) ((((ms) > (0)) ? (ms) : (0)))*(10000) + ((
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               0
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (1000) / 2 : (
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               1
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (1000) - 1 : 0)) / (1000)) : (((uint64_t) ((((ms) > (0)) ? (ms) : (0))) / (1000))*(10000) + (((uint64_t) ((((ms) > (0)) ? (ms) : (0))) % (1000))*(10000) + ((
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               0
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (1000) / 2 : (
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               1
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (1000) - 1 : 0)) / (1000))) ) : (((uint64_t) ((((ms) > (0)) ? (ms) : (0))) / (1000))*(10000) + (((uint64_t) ((((ms) > (0)) ? (ms) : (0))) % (1000))*(10000) + ((
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               0
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (1000) / 2 : (
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
               1
# 568 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
               ) ? (1000) - 1 : 0)) / (1000)) )) }));
}
# 587 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int32_t k_usleep(int32_t us);
# 605 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_busy_wait(uint32_t usec_to_wait);
# 618 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 618 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 618 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_can_yield(void);
# 627 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_yield(void);
# 638 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_wakeup(k_tid_t thread);
# 653 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
__attribute__((__const__))
static inline k_tid_t k_sched_current_thread_query(void);







__attribute__((__const__))
static inline k_tid_t k_current_get(void)
{



 extern _Thread_local k_tid_t z_tls_current;

 return z_tls_current;



}
# 695 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_abort(k_tid_t thread);
# 707 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_start(k_tid_t thread);

k_ticks_t z_timeout_expires(const struct _timeout *timeout);
k_ticks_t z_timeout_remaining(const struct _timeout *timeout);
# 721 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_ticks_t k_thread_timeout_expires_ticks(const struct k_thread *thread);

static inline k_ticks_t z_impl_k_thread_timeout_expires_ticks(
      const struct k_thread *thread)
{
 return z_timeout_expires(&thread->base.timeout);
}
# 736 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_ticks_t k_thread_timeout_remaining_ticks(const struct k_thread *thread);

static inline k_ticks_t z_impl_k_thread_timeout_remaining_ticks(
      const struct k_thread *thread)
{
 return z_timeout_remaining(&thread->base.timeout);
}







struct _static_thread_data {
 struct k_thread *init_thread;
 k_thread_stack_t *init_stack;
 unsigned int init_stack_size;
 k_thread_entry_t init_entry;
 void *init_p1;
 void *init_p2;
 void *init_p3;
 int init_prio;
 uint32_t init_options;
 const char *init_name;



 k_timeout_t init_delay;

};
# 897 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_thread_priority_get(k_tid_t thread);
# 924 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_priority_set(k_tid_t thread, int prio);
# 1053 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_suspend(k_tid_t thread);
# 1065 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_resume(k_tid_t thread);
# 1093 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_sched_time_slice_set(int32_t slice, int prio);
# 1133 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_thread_time_slice_set(struct k_thread *th, int32_t slice_ticks,
        k_thread_timeslice_fn_t expired, void *data);
# 1154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 1154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 1154 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_is_in_isr(void);
# 1172 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_is_preempt_thread(void);
# 1185 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline 
# 1185 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
             _Bool 
# 1185 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                  k_is_pre_kernel(void)
{
 extern 
# 1187 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       _Bool 
# 1187 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
            z_sys_post_kernel;

 return !z_sys_post_kernel;
}
# 1226 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_sched_lock(void);
# 1235 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_sched_unlock(void);
# 1249 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_thread_custom_data_set(void *value);
# 1258 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void *k_thread_custom_data_get(void);
# 1273 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_thread_name_set(k_tid_t thread, const char *str);
# 1283 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
const char *k_thread_name_get(k_tid_t thread);
# 1296 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_thread_name_copy(k_tid_t thread, char *buf,
     size_t size);
# 1311 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
const char *k_thread_state_str(k_tid_t thread_id, char *buf, size_t buf_size);
# 1523 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_timer {





 struct _timeout timeout;


 _wait_q_t wait_q;


 void (*expiry_fn)(struct k_timer *timer);


 void (*stop_fn)(struct k_timer *timer);


 k_timeout_t period;


 uint32_t status;


 void *user_data;






};
# 1590 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
typedef void (*k_timer_expiry_t)(struct k_timer *timer);
# 1606 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
typedef void (*k_timer_stop_t)(struct k_timer *timer);
# 1632 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_timer_init(struct k_timer *timer,
    k_timer_expiry_t expiry_fn,
    k_timer_stop_t stop_fn);
# 1650 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_timer_start(struct k_timer *timer,
        k_timeout_t duration, k_timeout_t period);
# 1669 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_timer_stop(struct k_timer *timer);
# 1683 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_timer_status_get(struct k_timer *timer);
# 1702 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_timer_status_sync(struct k_timer *timer);
# 1716 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_ticks_t k_timer_expires_ticks(const struct k_timer *timer);

static inline k_ticks_t z_impl_k_timer_expires_ticks(
           const struct k_timer *timer)
{
 return z_timeout_expires(&timer->timeout);
}
# 1734 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_ticks_t k_timer_remaining_ticks(const struct k_timer *timer);

static inline k_ticks_t z_impl_k_timer_remaining_ticks(
           const struct k_timer *timer)
{
 return z_timeout_remaining(&timer->timeout);
}
# 1752 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_timer_remaining_get(struct k_timer *timer)
{
 return ((
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       1
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ( ((1000) == (10000)) ? (uint32_t) (k_timer_remaining_ticks(timer)) : ((10000) > (1000) && (10000) % (1000) == 0U) ? ((uint64_t) (k_timer_remaining_ticks(timer)) <= 0xffffffffU - ((
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) / 2 : (
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) - 1 : 0) ? ((uint32_t)((k_timer_remaining_ticks(timer)) + ((
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) / 2 : (
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) - 1 : 0)) / ((10000)/(1000) ? (10000)/(1000) : 01u)) : (uint32_t) (((uint64_t) (k_timer_remaining_ticks(timer)) + ((
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) / 2 : (
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) - 1 : 0)) / ((10000)/(1000) ? (10000)/(1000) : 01u)) ) : ((1000) > (10000) && (1000) % (10000) == 0U) ? (uint32_t) (k_timer_remaining_ticks(timer))*((1000) / (10000)) : ((uint32_t) (((uint64_t) (k_timer_remaining_ticks(timer))*(1000) + ((
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000))) ) : ((uint32_t) (((uint64_t) (k_timer_remaining_ticks(timer))*(1000) + ((
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000))) );
}
# 1771 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_timer_user_data_set(struct k_timer *timer, void *user_data);




static inline void z_impl_k_timer_user_data_set(struct k_timer *timer,
            void *user_data)
{
 timer->user_data = user_data;
}
# 1789 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void *k_timer_user_data_get(const struct k_timer *timer);

static inline void *z_impl_k_timer_user_data_get(const struct k_timer *timer)
{
 return timer->user_data;
}
# 1813 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int64_t k_uptime_ticks(void);
# 1828 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int64_t k_uptime_get(void)
{
 return ((
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       1
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ( ((1000) == (10000)) ? (uint64_t) (k_uptime_ticks()) : ((10000) > (1000) && (10000) % (1000) == 0U) ? (((uint64_t) (k_uptime_ticks()) + ((
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) / 2 : (
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1000)) - 1 : 0)) / ((10000)/(1000) ? (10000)/(1000) : 01u)) : ((1000) > (10000) && (1000) % (10000) == 0U) ? (uint64_t) (k_uptime_ticks())*((1000) / (10000)) : ((((((365 * 24ULL * 3600ULL * 10000) + (
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       (0xffffffffUL)
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) - 1) / (
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       (0xffffffffUL)
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       )) * 1000) <= 
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       (0xffffffffUL)
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (((uint64_t) (k_uptime_ticks())*(1000) + ((
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000)) : (((uint64_t) (k_uptime_ticks()) / (10000))*(1000) + (((uint64_t) (k_uptime_ticks()) % (10000))*(1000) + ((
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000))) ) : (((uint64_t) (k_uptime_ticks()) / (10000))*(1000) + (((uint64_t) (k_uptime_ticks()) % (10000))*(1000) + ((
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000)) );
}
# 1852 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_uptime_get_32(void)
{
 return (uint32_t)k_uptime_get();
}
# 1865 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_uptime_seconds(void)
{
 return ((
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       1
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ( ((1) == (10000)) ? (uint32_t) (k_uptime_ticks()) : ((10000) > (1) && (10000) % (1) == 0U) ? ((uint64_t) (k_uptime_ticks()) <= 0xffffffffU - ((
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1)) / 2 : (
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1)) - 1 : 0) ? ((uint32_t)((k_uptime_ticks()) + ((
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1)) / 2 : (
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1)) - 1 : 0)) / ((10000)/(1) ? (10000)/(1) : 01u)) : (uint32_t) (((uint64_t) (k_uptime_ticks()) + ((
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1)) / 2 : (
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? ((10000) / (1)) - 1 : 0)) / ((10000)/(1) ? (10000)/(1) : 01u)) ) : ((1) > (10000) && (1) % (10000) == 0U) ? (uint32_t) (k_uptime_ticks())*((1) / (10000)) : ((uint32_t) (((uint64_t) (k_uptime_ticks())*(1) + ((
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000))) ) : ((uint32_t) (((uint64_t) (k_uptime_ticks())*(1) + ((
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) / 2 : (
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       0
# 1867 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       ) ? (10000) - 1 : 0)) / (10000))) );
}
# 1881 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int64_t k_uptime_delta(int64_t *reftime)
{
 int64_t uptime, delta;

 uptime = k_uptime_get();
 delta = uptime - *reftime;
 *reftime = uptime;

 return delta;
}
# 1900 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_cycle_get_32(void)
{
 return arch_k_cycle_get_32();
}
# 1915 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint64_t k_cycle_get_64(void)
{
 if (!1) {
  { }
                                                  ;
  return 0;
 }

 return arch_k_cycle_get_64();
}





struct k_queue {
 sys_sflist_t data_q;
 struct k_spinlock lock;
 _wait_q_t wait_q;




};
# 1969 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_queue_init(struct k_queue *queue);
# 1984 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_queue_cancel_wait(struct k_queue *queue);
# 1998 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_queue_append(struct k_queue *queue, void *data);
# 2016 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int32_t k_queue_alloc_append(struct k_queue *queue, void *data);
# 2030 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_queue_prepend(struct k_queue *queue, void *data);
# 2048 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int32_t k_queue_alloc_prepend(struct k_queue *queue, void *data);
# 2063 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_queue_insert(struct k_queue *queue, void *prev, void *data);
# 2083 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_queue_append_list(struct k_queue *queue, void *head, void *tail);
# 2100 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_queue_merge_slist(struct k_queue *queue, sys_slist_t *list);
# 2119 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void *k_queue_get(struct k_queue *queue, k_timeout_t timeout);
# 2137 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 2137 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 2137 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_queue_remove(struct k_queue *queue, void *data);
# 2153 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 2153 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 2153 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_queue_unique_append(struct k_queue *queue, void *data);
# 2168 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_queue_is_empty(struct k_queue *queue);

static inline int z_impl_k_queue_is_empty(struct k_queue *queue)
{
 return sys_sflist_is_empty(&queue->data_q) ? 1 : 0;
}
# 2184 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void *k_queue_peek_head(struct k_queue *queue);
# 2195 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void *k_queue_peek_tail(struct k_queue *queue);
# 2301 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_event {
 _wait_q_t wait_q;
 uint32_t events;
 struct k_spinlock lock;







};
# 2327 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_event_init(struct k_event *event);
# 2344 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_post(struct k_event *event, uint32_t events);
# 2361 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_set(struct k_event *event, uint32_t events);
# 2377 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_set_masked(struct k_event *event, uint32_t events,
      uint32_t events_mask);
# 2390 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_clear(struct k_event *event, uint32_t events);
# 2413 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_wait(struct k_event *event, uint32_t events,
    
# 2414 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
   _Bool 
# 2414 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
        reset, k_timeout_t timeout);
# 2437 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_wait_all(struct k_event *event, uint32_t events,
        
# 2438 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       _Bool 
# 2438 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
            reset, k_timeout_t timeout);
# 2448 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_event_test(struct k_event *event, uint32_t events_mask)
{
 return k_event_wait(event, events_mask, 
# 2450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
                                        0
# 2450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                                             , ((k_timeout_t) {0}));
}
# 2468 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_fifo {
 struct k_queue _queue;



};
# 2707 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_lifo {
 struct k_queue _queue;



};
# 2837 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
typedef uintptr_t stack_data_t;

struct k_stack {
 _wait_q_t wait_q;
 struct k_spinlock lock;
 stack_data_t *base, *next, *top;

 uint8_t flags;






};
# 2880 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_stack_init(struct k_stack *stack,
    stack_data_t *buffer, uint32_t num_entries);
# 2898 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int32_t k_stack_alloc_init(struct k_stack *stack,
       uint32_t num_entries);
# 2912 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_stack_cleanup(struct k_stack *stack);
# 2927 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_stack_push(struct k_stack *stack, stack_data_t data);
# 2949 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_stack_pop(struct k_stack *stack, stack_data_t *data,
     k_timeout_t timeout);
# 2975 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_work;
struct k_work_q;
struct k_work_queue_config;
extern struct k_work_q k_sys_work_q;
# 2994 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_mutex {

 _wait_q_t wait_q;

 struct k_thread *owner;


 uint32_t lock_count;


 int owner_orig_prio;






};
# 3053 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_mutex_init(struct k_mutex *mutex);
# 3077 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_mutex_lock(struct k_mutex *mutex, k_timeout_t timeout);
# 3099 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_mutex_unlock(struct k_mutex *mutex);






struct k_condvar {
 _wait_q_t wait_q;




};
# 3131 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_condvar_init(struct k_condvar *condvar);







static inline int k_condvar_signal(struct k_condvar *condvar);
# 3148 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_condvar_broadcast(struct k_condvar *condvar);
# 3167 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_condvar_wait(struct k_condvar *condvar, struct k_mutex *mutex,
        k_timeout_t timeout);
# 3191 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_sem {
 _wait_q_t wait_q;
 unsigned int count;
 unsigned int limit;








};
# 3248 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_sem_init(struct k_sem *sem, unsigned int initial_count,
     unsigned int limit);
# 3269 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_sem_take(struct k_sem *sem, k_timeout_t timeout);
# 3281 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_sem_give(struct k_sem *sem);
# 3292 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_sem_reset(struct k_sem *sem);
# 3303 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline unsigned int k_sem_count_get(struct k_sem *sem);




static inline unsigned int z_impl_k_sem_count_get(struct k_sem *sem)
{
 return sem->count;
}
# 3337 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_work_delayable;
struct k_work_sync;
# 3356 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
typedef void (*k_work_handler_t)(struct k_work *work);
# 3371 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_work_init(struct k_work *work,
    k_work_handler_t handler);
# 3388 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_busy_get(const struct k_work *work);
# 3403 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline 
# 3403 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
             _Bool 
# 3403 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                  k_work_is_pending(const struct k_work *work);
# 3425 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_submit_to_queue(struct k_work_q *queue,
      struct k_work *work);
# 3436 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_submit(struct k_work *work);
# 3462 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 3462 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 3462 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_work_flush(struct k_work *work,
    struct k_work_sync *sync);
# 3484 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_cancel(struct k_work *work);
# 3516 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 3516 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 3516 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_work_cancel_sync(struct k_work *work, struct k_work_sync *sync);
# 3527 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_work_queue_init(struct k_work_q *queue);
# 3548 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_work_queue_start(struct k_work_q *queue,
   k_thread_stack_t *stack, size_t stack_size,
   int prio, const struct k_work_queue_config *cfg);
# 3561 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_tid_t k_work_queue_thread_get(struct k_work_q *queue);
# 3586 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_queue_drain(struct k_work_q *queue, 
# 3586 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
                                              _Bool 
# 3586 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                                                   plug);
# 3601 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_queue_unplug(struct k_work_q *queue);
# 3616 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_work_init_delayable(struct k_work_delayable *dwork,
      k_work_handler_t handler);
# 3630 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline struct k_work_delayable *
k_work_delayable_from_work(struct k_work *work);
# 3646 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_delayable_busy_get(const struct k_work_delayable *dwork);
# 3662 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline 
# 3662 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
             _Bool 
# 3662 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                  k_work_delayable_is_pending(
 const struct k_work_delayable *dwork);
# 3678 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_ticks_t k_work_delayable_expires_get(
 const struct k_work_delayable *dwork);
# 3694 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_ticks_t k_work_delayable_remaining_get(
 const struct k_work_delayable *dwork);
# 3724 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_schedule_for_queue(struct k_work_q *queue,
          struct k_work_delayable *dwork,
          k_timeout_t delay);
# 3741 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_schedule(struct k_work_delayable *dwork,
       k_timeout_t delay);
# 3779 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_reschedule_for_queue(struct k_work_q *queue,
     struct k_work_delayable *dwork,
     k_timeout_t delay);
# 3795 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_reschedule(struct k_work_delayable *dwork,
         k_timeout_t delay);
# 3822 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 3822 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 3822 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_work_flush_delayable(struct k_work_delayable *dwork,
       struct k_work_sync *sync);
# 3845 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_cancel_delayable(struct k_work_delayable *dwork);
# 3875 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"

# 3875 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 3875 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
    k_work_cancel_delayable_sync(struct k_work_delayable *dwork,
      struct k_work_sync *sync);

enum {
# 3890 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
 K_WORK_RUNNING_BIT = 0,
 K_WORK_CANCELING_BIT = 1,
 K_WORK_QUEUED_BIT = 2,
 K_WORK_DELAYED_BIT = 3,
 K_WORK_FLUSHING_BIT = 4,

 K_WORK_MASK = (1UL << (K_WORK_DELAYED_BIT)) | (1UL << (K_WORK_QUEUED_BIT))
  | (1UL << (K_WORK_RUNNING_BIT)) | (1UL << (K_WORK_CANCELING_BIT)) | (1UL << (K_WORK_FLUSHING_BIT)),


 K_WORK_DELAYABLE_BIT = 8,
 K_WORK_DELAYABLE = (1UL << (K_WORK_DELAYABLE_BIT)),


 K_WORK_QUEUE_STARTED_BIT = 0,
 K_WORK_QUEUE_STARTED = (1UL << (K_WORK_QUEUE_STARTED_BIT)),
 K_WORK_QUEUE_BUSY_BIT = 1,
 K_WORK_QUEUE_BUSY = (1UL << (K_WORK_QUEUE_BUSY_BIT)),
 K_WORK_QUEUE_DRAIN_BIT = 2,
 K_WORK_QUEUE_DRAIN = (1UL << (K_WORK_QUEUE_DRAIN_BIT)),
 K_WORK_QUEUE_PLUGGED_BIT = 3,
 K_WORK_QUEUE_PLUGGED = (1UL << (K_WORK_QUEUE_PLUGGED_BIT)),


 K_WORK_QUEUE_NO_YIELD_BIT = 8,
 K_WORK_QUEUE_NO_YIELD = (1UL << (K_WORK_QUEUE_NO_YIELD_BIT)),
# 3927 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
 K_WORK_RUNNING = (1UL << (K_WORK_RUNNING_BIT)),





 K_WORK_CANCELING = (1UL << (K_WORK_CANCELING_BIT)),






 K_WORK_QUEUED = (1UL << (K_WORK_QUEUED_BIT)),






 K_WORK_DELAYED = (1UL << (K_WORK_DELAYED_BIT)),





 K_WORK_FLUSHING = (1UL << (K_WORK_FLUSHING_BIT)),
};


struct k_work {





 sys_snode_t node;


 k_work_handler_t handler;


 struct k_work_q *queue;







 uint32_t flags;
};






struct k_work_delayable {

 struct k_work work;


 struct _timeout timeout;


 struct k_work_q *queue;
};
# 4034 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct z_work_flusher {
 struct k_work work;
 struct k_sem sem;
};







struct z_work_canceller {
 sys_snode_t node;
 struct k_work *work;
 struct k_sem sem;
};
# 4068 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_work_sync {
 union {
  struct z_work_flusher flusher;
  struct z_work_canceller canceller;
 };
};







struct k_work_queue_config {




 const char *name;
# 4100 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
 
# 4100 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 4100 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
     no_yield;




 
# 4105 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
_Bool 
# 4105 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
     essential;
};


struct k_work_q {

 struct k_thread thread;






 sys_slist_t pending;


 _wait_q_t notifyq;


 _wait_q_t drainq;


 uint32_t flags;
};



static inline 
# 4132 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
             _Bool 
# 4132 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                  k_work_is_pending(const struct k_work *work)
{
 return k_work_busy_get(work) != 0;
}

static inline struct k_work_delayable *
k_work_delayable_from_work(struct k_work *work)
{
 return ({ _Static_assert((__builtin_types_compatible_p(__typeof__(*(work)), __typeof__(((struct k_work_delayable *)0)->work)) || __builtin_types_compatible_p(__typeof__(*(work)), __typeof__(void))), "" "pointer type mismatch in CONTAINER_OF"); ((struct k_work_delayable *)(((char *)(work)) - 
# 4140 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       __builtin_offsetof (
# 4140 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       struct k_work_delayable
# 4140 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       , 
# 4140 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       work
# 4140 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
       )
# 4140 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
       )); });
}

static inline 
# 4143 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
             _Bool 
# 4143 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                  k_work_delayable_is_pending(
 const struct k_work_delayable *dwork)
{
 return k_work_delayable_busy_get(dwork) != 0;
}

static inline k_ticks_t k_work_delayable_expires_get(
 const struct k_work_delayable *dwork)
{
 return z_timeout_expires(&dwork->timeout);
}

static inline k_ticks_t k_work_delayable_remaining_get(
 const struct k_work_delayable *dwork)
{
 return z_timeout_remaining(&dwork->timeout);
}

static inline k_tid_t k_work_queue_thread_get(struct k_work_q *queue)
{
 return &queue->thread;
}



struct k_work_user;
# 4184 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
typedef void (*k_work_user_handler_t)(struct k_work_user *work);





struct k_work_user_q {
 struct k_queue queue;
 struct k_thread thread;
};

enum {
 K_WORK_USER_STATE_PENDING,
};

struct k_work_user {
 void *_reserved;
 k_work_user_handler_t handler;
 atomic_t flags;
};
# 4243 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_work_user_init(struct k_work_user *work,
        k_work_user_handler_t handler)
{
 *work = (struct k_work_user){ ._reserved = 
# 4246 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
                            ((void *)0)
# 4246 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                            , .handler = (handler), .flags = 0 };
}
# 4265 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline 
# 4265 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
             _Bool 
# 4265 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                  k_work_user_is_pending(struct k_work_user *work)
{
 return atomic_test_bit(&work->flags, K_WORK_USER_STATE_PENDING);
}
# 4288 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_work_user_submit_to_queue(struct k_work_user_q *work_q,
           struct k_work_user *work)
{
 int ret = -
# 4291 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 3 4
           16
# 4291 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
                ;

 if (!atomic_test_and_set_bit(&work->flags,
         K_WORK_USER_STATE_PENDING)) {
  ret = k_queue_alloc_append(&work_q->queue, work);




  if (ret != 0) {
   atomic_clear_bit(&work->flags,
      K_WORK_USER_STATE_PENDING);
  }
 }

 return ret;
}
# 4328 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_work_user_queue_start(struct k_work_user_q *work_q,
        k_thread_stack_t *stack,
        size_t stack_size, int prio,
        const char *name);
# 4343 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline k_tid_t k_work_user_queue_thread_get(struct k_work_user_q *work_q)
{
 return &work_q->thread;
}







struct k_work_poll {
 struct k_work work;
 struct k_work_q *workq;
 struct z_poller poller;
 struct k_poll_event *events;
 int num_events;
 k_work_handler_t real_handler;
 struct _timeout timeout;
 int poll_result;
};
# 4397 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_work_poll_init(struct k_work_poll *work,
        k_work_handler_t handler);
# 4434 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_poll_submit_to_queue(struct k_work_q *work_q,
           struct k_work_poll *work,
           struct k_poll_event *events,
           int num_events,
           k_timeout_t timeout);
# 4471 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_poll_submit(struct k_work_poll *work,
         struct k_poll_event *events,
         int num_events,
         k_timeout_t timeout);
# 4490 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_work_poll_cancel(struct k_work_poll *work);
# 4503 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_msgq {

 _wait_q_t wait_q;

 struct k_spinlock lock;

 size_t msg_size;

 uint32_t max_msgs;

 char *buffer_start;

 char *buffer_end;

 char *read_ptr;

 char *write_ptr;

 uint32_t used_msgs;




 uint8_t flags;






};
# 4562 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_msgq_attrs {

 size_t msg_size;

 uint32_t max_msgs;

 uint32_t used_msgs;
};
# 4611 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_msgq_init(struct k_msgq *msgq, char *buffer, size_t msg_size,
   uint32_t max_msgs);
# 4633 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_msgq_alloc_init(struct k_msgq *msgq, size_t msg_size,
    uint32_t max_msgs);
# 4646 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_msgq_cleanup(struct k_msgq *msgq);
# 4668 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_msgq_put(struct k_msgq *msgq, const void *data, k_timeout_t timeout);
# 4690 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_msgq_get(struct k_msgq *msgq, void *data, k_timeout_t timeout);
# 4706 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_msgq_peek(struct k_msgq *msgq, void *data);
# 4724 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_msgq_peek_at(struct k_msgq *msgq, void *data, uint32_t idx);
# 4735 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_msgq_purge(struct k_msgq *msgq);
# 4747 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_msgq_num_free_get(struct k_msgq *msgq);
# 4757 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_msgq_get_attrs(struct k_msgq *msgq,
     struct k_msgq_attrs *attrs);


static inline uint32_t z_impl_k_msgq_num_free_get(struct k_msgq *msgq)
{
 return msgq->max_msgs - msgq->used_msgs;
}
# 4775 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_msgq_num_used_get(struct k_msgq *msgq);

static inline uint32_t z_impl_k_msgq_num_used_get(struct k_msgq *msgq)
{
 return msgq->used_msgs;
}
# 4794 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_mbox_msg {

 size_t size;

 uint32_t info;

 void *tx_data;

 k_tid_t rx_source_thread;

 k_tid_t tx_target_thread;

 k_tid_t _syncing_thread;


 struct k_sem *_async_sem;

};




struct k_mbox {

 _wait_q_t tx_msg_queue;

 _wait_q_t rx_msg_queue;
 struct k_spinlock lock;






};
# 4863 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_mbox_init(struct k_mbox *mbox);
# 4884 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_mbox_put(struct k_mbox *mbox, struct k_mbox_msg *tx_msg,
        k_timeout_t timeout);
# 4900 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_mbox_async_put(struct k_mbox *mbox, struct k_mbox_msg *tx_msg,
        struct k_sem *sem);
# 4920 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_mbox_get(struct k_mbox *mbox, struct k_mbox_msg *rx_msg,
        void *buffer, k_timeout_t timeout);
# 4936 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_mbox_data_get(struct k_mbox_msg *rx_msg, void *buffer);
# 4947 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_pipe {
 unsigned char *buffer;
 size_t size;
 size_t bytes_used;
 size_t read_index;
 size_t write_index;
 struct k_spinlock lock;

 struct {
  _wait_q_t readers;
  _wait_q_t writers;
 } wait_q;



 uint8_t flags;






};
# 5026 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_pipe_init(struct k_pipe *pipe, unsigned char *buffer, size_t size);
# 5039 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_pipe_cleanup(struct k_pipe *pipe);
# 5056 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_pipe_alloc_init(struct k_pipe *pipe, size_t size);
# 5076 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_pipe_put(struct k_pipe *pipe, const void *data,
    size_t bytes_to_write, size_t *bytes_written,
    size_t min_xfer, k_timeout_t timeout);
# 5099 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_pipe_get(struct k_pipe *pipe, void *data,
    size_t bytes_to_read, size_t *bytes_read,
    size_t min_xfer, k_timeout_t timeout);
# 5111 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline size_t k_pipe_read_avail(struct k_pipe *pipe);
# 5121 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline size_t k_pipe_write_avail(struct k_pipe *pipe);
# 5133 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_pipe_flush(struct k_pipe *pipe);
# 5146 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_pipe_buffer_flush(struct k_pipe *pipe);







struct k_mem_slab_info {
 uint32_t num_blocks;
 size_t block_size;
 uint32_t num_used;



};

struct k_mem_slab {
 _wait_q_t wait_q;
 struct k_spinlock lock;
 char *buffer;
 char *free_list;
 struct k_mem_slab_info info;






};
# 5272 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_mem_slab_init(struct k_mem_slab *slab, void *buffer,
      size_t block_size, uint32_t num_blocks);
# 5297 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_mem_slab_alloc(struct k_mem_slab *slab, void **mem,
       k_timeout_t timeout);
# 5309 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_mem_slab_free(struct k_mem_slab *slab, void *mem);
# 5321 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_mem_slab_num_used_get(struct k_mem_slab *slab)
{
 return slab->info.num_used;
}
# 5336 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_mem_slab_max_used_get(struct k_mem_slab *slab)
{



 (void)(slab);
 return 0;

}
# 5356 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline uint32_t k_mem_slab_num_free_get(struct k_mem_slab *slab)
{
 return slab->info.num_blocks - slab->info.num_used;
}
# 5373 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_mem_slab_runtime_stats_get(struct k_mem_slab *slab, struct sys_memory_stats *stats);
# 5386 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_mem_slab_runtime_stats_reset_max(struct k_mem_slab *slab);
# 5397 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_heap {
 struct sys_heap heap;
 _wait_q_t wait_q;
 struct k_spinlock lock;
};
# 5416 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_heap_init(struct k_heap *h, void *mem,
  size_t bytes) __attribute__((nonnull(1)));
# 5439 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_heap_aligned_alloc(struct k_heap *h, size_t align, size_t bytes,
   k_timeout_t timeout) __attribute__((nonnull(1)));
# 5463 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_heap_alloc(struct k_heap *h, size_t bytes,
  k_timeout_t timeout) __attribute__((nonnull(1)));
# 5489 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_heap_realloc(struct k_heap *h, void *ptr, size_t bytes, k_timeout_t timeout)
 __attribute__((nonnull(1)));
# 5502 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_heap_free(struct k_heap *h, void *mem) __attribute__((nonnull(1)));
# 5599 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_aligned_alloc(size_t align, size_t size);
# 5612 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_malloc(size_t size);
# 5624 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_free(void *ptr);
# 5637 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_calloc(size_t nmemb, size_t size);
# 5656 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void *k_realloc(void *ptr, size_t size);
# 5669 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
enum _poll_types_bits {

 _POLL_TYPE_IGNORE,


 _POLL_TYPE_SIGNAL,


 _POLL_TYPE_SEM_AVAILABLE,


 _POLL_TYPE_DATA_AVAILABLE,


 _POLL_TYPE_MSGQ_DATA_AVAILABLE,


 _POLL_TYPE_PIPE_DATA_AVAILABLE,

 _POLL_NUM_TYPES
};




enum _poll_states_bits {

 _POLL_STATE_NOT_READY,


 _POLL_STATE_SIGNALED,


 _POLL_STATE_SEM_AVAILABLE,


 _POLL_STATE_DATA_AVAILABLE,


 _POLL_STATE_CANCELLED,


 _POLL_STATE_MSGQ_DATA_AVAILABLE,


 _POLL_STATE_PIPE_DATA_AVAILABLE,

 _POLL_NUM_STATES
};
# 5750 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
enum k_poll_modes {

 K_POLL_MODE_NOTIFY_ONLY = 0,

 K_POLL_NUM_MODES
};
# 5768 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_poll_signal {

 sys_dlist_t poll_events;





 unsigned int signaled;


 int result;
};
# 5792 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
struct k_poll_event {

 sys_dnode_t _node;


 struct z_poller *poller;


 uint32_t tag:8;


 uint32_t type:_POLL_NUM_TYPES;


 uint32_t state:_POLL_NUM_STATES;


 uint32_t mode:1;


 uint32_t unused:(32 - (0 + 8 + _POLL_NUM_TYPES + _POLL_NUM_STATES + 1 ));


 union {



  void *obj, *typed_K_POLL_TYPE_IGNORE;
  struct k_poll_signal *signal, *typed_K_POLL_TYPE_SIGNAL;
  struct k_sem *sem, *typed_K_POLL_TYPE_SEM_AVAILABLE;
  struct k_fifo *fifo, *typed_K_POLL_TYPE_FIFO_DATA_AVAILABLE;
  struct k_queue *queue, *typed_K_POLL_TYPE_DATA_AVAILABLE;
  struct k_msgq *msgq, *typed_K_POLL_TYPE_MSGQ_DATA_AVAILABLE;



 };
};
# 5871 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_poll_event_init(struct k_poll_event *event, uint32_t type,
         int mode, void *obj);
# 5917 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_poll(struct k_poll_event *events, int num_events,
       k_timeout_t timeout);
# 5928 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_poll_signal_init(struct k_poll_signal *sig);






static inline void k_poll_signal_reset(struct k_poll_signal *sig);
# 5947 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_poll_signal_check(struct k_poll_signal *sig,
       unsigned int *signaled, int *result);
# 5974 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_poll_signal_raise(struct k_poll_signal *sig, int result);
# 5996 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_cpu_idle(void)
{
 arch_cpu_idle();
}
# 6015 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_cpu_atomic_idle(unsigned int key)
{
 arch_cpu_atomic_idle(key);
}
# 6090 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void z_timer_expiration_handler(struct _timeout *timeout);
# 6103 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline void k_str_out(char *c, size_t n);
# 6132 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_float_disable(struct k_thread *thread);
# 6172 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
static inline int k_float_enable(struct k_thread *thread, unsigned int options);
# 6185 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_thread_runtime_stats_get(k_tid_t thread,
          k_thread_runtime_stats_t *stats);







int k_thread_runtime_stats_all_get(k_thread_runtime_stats_t *stats);
# 6203 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_thread_runtime_stats_cpu_get(int cpu, k_thread_runtime_stats_t *stats);
# 6214 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_thread_runtime_stats_enable(k_tid_t thread);
# 6225 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
int k_thread_runtime_stats_disable(k_tid_t thread);
# 6234 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_sys_runtime_stats_enable(void);
# 6243 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h"
void k_sys_runtime_stats_disable(void);





# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing.h" 1
# 9 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing.h" 2

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracking.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing.h" 2
# 2385 "/home/ttwards/zephyrproject/zephyr/include/zephyr/tracing/tracing.h"
void sys_trace_isr_enter(void);




void sys_trace_isr_exit(void);




void sys_trace_isr_exit_to_scheduler(void);




void sys_trace_idle(void);
# 6250 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 2
# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 1
# 23 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
extern k_thread_stack_t * z_impl_k_thread_stack_alloc(size_t size, int flags);


static inline k_thread_stack_t * k_thread_stack_alloc(size_t size, int flags)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 35 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 35 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_stack_alloc(size, flags);
}


extern int z_impl_k_thread_stack_free(k_thread_stack_t * stack);


static inline int k_thread_stack_free(k_thread_stack_t * stack)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 51 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 51 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_stack_free(stack);
}


extern k_tid_t z_impl_k_thread_create(struct k_thread * new_thread, k_thread_stack_t * stack, size_t stack_size, k_thread_entry_t entry, void * p1, void * p2, void * p3, int prio, uint32_t options, k_timeout_t delay);


static inline k_tid_t k_thread_create(struct k_thread * new_thread, k_thread_stack_t * stack, size_t stack_size, k_thread_entry_t entry, void * p1, void * p2, void * p3, int prio, uint32_t options, k_timeout_t delay)
{
# 84 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 84 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 84 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_create(new_thread, stack, stack_size, entry, p1, p2, p3, prio, options, delay);
}


extern int z_impl_k_thread_stack_space_get(const struct k_thread * thread, size_t * unused_ptr);


static inline int k_thread_stack_space_get(const struct k_thread * thread, size_t * unused_ptr)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 101 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 101 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_stack_space_get(thread, unused_ptr);
}


extern int z_impl_k_thread_join(struct k_thread * thread, k_timeout_t timeout);


static inline int k_thread_join(struct k_thread * thread, k_timeout_t timeout)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 118 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 118 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_join(thread, timeout);
}


extern int32_t z_impl_k_sleep(k_timeout_t timeout);


static inline int32_t k_sleep(k_timeout_t timeout)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 134 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 134 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_sleep(timeout);
}


extern int32_t z_impl_k_usleep(int32_t us);


static inline int32_t k_usleep(int32_t us)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 150 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 150 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_usleep(us);
}


extern void z_impl_k_busy_wait(uint32_t usec_to_wait);


static inline void k_busy_wait(uint32_t usec_to_wait)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 167 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 167 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_busy_wait(usec_to_wait);
}


extern void z_impl_k_yield(void);


static inline void k_yield(void)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 183 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 183 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_yield();
}


extern void z_impl_k_wakeup(k_tid_t thread);


static inline void k_wakeup(k_tid_t thread)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 200 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 200 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_wakeup(thread);
}


extern k_tid_t z_impl_k_sched_current_thread_query(void);


static inline k_tid_t k_sched_current_thread_query(void)
{





 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 215 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 215 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_sched_current_thread_query();
}


extern void z_impl_k_thread_abort(k_tid_t thread);


static inline void k_thread_abort(k_tid_t thread)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 232 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 232 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_abort(thread);
}


extern void z_impl_k_thread_start(k_tid_t thread);


static inline void k_thread_start(k_tid_t thread)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 249 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 249 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_start(thread);
}


extern k_ticks_t z_impl_k_thread_timeout_expires_ticks(const struct k_thread * thread);


static inline k_ticks_t k_thread_timeout_expires_ticks(const struct k_thread * thread)
{
# 267 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 267 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 267 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_timeout_expires_ticks(thread);
}


extern k_ticks_t z_impl_k_thread_timeout_remaining_ticks(const struct k_thread * thread);


static inline k_ticks_t k_thread_timeout_remaining_ticks(const struct k_thread * thread)
{
# 285 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 285 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 285 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_timeout_remaining_ticks(thread);
}


extern int z_impl_k_thread_priority_get(k_tid_t thread);


static inline int k_thread_priority_get(k_tid_t thread)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 301 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 301 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_priority_get(thread);
}


extern void z_impl_k_thread_priority_set(k_tid_t thread, int prio);


static inline void k_thread_priority_set(k_tid_t thread, int prio)
{
# 319 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 319 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 319 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_priority_set(thread, prio);
}


extern void z_impl_k_thread_deadline_set(k_tid_t thread, int deadline);


static inline void k_thread_deadline_set(k_tid_t thread, int deadline)
{
# 337 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 337 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 337 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_deadline_set(thread, deadline);
}


extern void z_impl_k_thread_suspend(k_tid_t thread);


static inline void k_thread_suspend(k_tid_t thread)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 354 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 354 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_suspend(thread);
}


extern void z_impl_k_thread_resume(k_tid_t thread);


static inline void k_thread_resume(k_tid_t thread)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 371 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 371 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_resume(thread);
}


extern int z_impl_k_is_preempt_thread(void);


static inline int k_is_preempt_thread(void)
{





 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 386 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 386 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_is_preempt_thread();
}


extern void z_impl_k_thread_custom_data_set(void * value);


static inline void k_thread_custom_data_set(void * value)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 403 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 403 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_thread_custom_data_set(value);
}


extern void * z_impl_k_thread_custom_data_get(void);


static inline void * k_thread_custom_data_get(void)
{





 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 418 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 418 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_custom_data_get();
}


extern int z_impl_k_thread_name_set(k_tid_t thread, const char * str);


static inline int k_thread_name_set(k_tid_t thread, const char * str)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 435 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 435 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_name_set(thread, str);
}


extern int z_impl_k_thread_name_copy(k_tid_t thread, char * buf, size_t size);


static inline int k_thread_name_copy(k_tid_t thread, char * buf, size_t size)
{
# 453 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 453 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 453 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_thread_name_copy(thread, buf, size);
}


extern void z_impl_k_timer_start(struct k_timer * timer, k_timeout_t duration, k_timeout_t period);


static inline void k_timer_start(struct k_timer * timer, k_timeout_t duration, k_timeout_t period)
{
# 472 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 472 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 472 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_timer_start(timer, duration, period);
}


extern void z_impl_k_timer_stop(struct k_timer * timer);


static inline void k_timer_stop(struct k_timer * timer)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 489 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 489 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_timer_stop(timer);
}


extern uint32_t z_impl_k_timer_status_get(struct k_timer * timer);


static inline uint32_t k_timer_status_get(struct k_timer * timer)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 505 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 505 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_timer_status_get(timer);
}


extern uint32_t z_impl_k_timer_status_sync(struct k_timer * timer);


static inline uint32_t k_timer_status_sync(struct k_timer * timer)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 521 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 521 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_timer_status_sync(timer);
}


extern k_ticks_t z_impl_k_timer_expires_ticks(const struct k_timer * timer);


static inline k_ticks_t k_timer_expires_ticks(const struct k_timer * timer)
{
# 539 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 539 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 539 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_timer_expires_ticks(timer);
}


extern k_ticks_t z_impl_k_timer_remaining_ticks(const struct k_timer * timer);


static inline k_ticks_t k_timer_remaining_ticks(const struct k_timer * timer)
{
# 557 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 557 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 557 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_timer_remaining_ticks(timer);
}


extern void z_impl_k_timer_user_data_set(struct k_timer * timer, void * user_data);


static inline void k_timer_user_data_set(struct k_timer * timer, void * user_data)
{
# 575 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 575 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 575 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_timer_user_data_set(timer, user_data);
}


extern void * z_impl_k_timer_user_data_get(const struct k_timer * timer);


static inline void * k_timer_user_data_get(const struct k_timer * timer)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 591 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 591 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_timer_user_data_get(timer);
}


extern int64_t z_impl_k_uptime_ticks(void);


static inline int64_t k_uptime_ticks(void)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 608 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 608 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_uptime_ticks();
}


extern void z_impl_k_queue_init(struct k_queue * queue);


static inline void k_queue_init(struct k_queue * queue)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 625 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 625 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_queue_init(queue);
}


extern void z_impl_k_queue_cancel_wait(struct k_queue * queue);


static inline void k_queue_cancel_wait(struct k_queue * queue)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 642 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 642 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_queue_cancel_wait(queue);
}


extern int32_t z_impl_k_queue_alloc_append(struct k_queue * queue, void * data);


static inline int32_t k_queue_alloc_append(struct k_queue * queue, void * data)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 659 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 659 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_queue_alloc_append(queue, data);
}


extern int32_t z_impl_k_queue_alloc_prepend(struct k_queue * queue, void * data);


static inline int32_t k_queue_alloc_prepend(struct k_queue * queue, void * data)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 676 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 676 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_queue_alloc_prepend(queue, data);
}


extern void * z_impl_k_queue_get(struct k_queue * queue, k_timeout_t timeout);


static inline void * k_queue_get(struct k_queue * queue, k_timeout_t timeout)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 693 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 693 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_queue_get(queue, timeout);
}


extern int z_impl_k_queue_is_empty(struct k_queue * queue);


static inline int k_queue_is_empty(struct k_queue * queue)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 709 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 709 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_queue_is_empty(queue);
}


extern void * z_impl_k_queue_peek_head(struct k_queue * queue);


static inline void * k_queue_peek_head(struct k_queue * queue)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 725 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 725 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_queue_peek_head(queue);
}


extern void * z_impl_k_queue_peek_tail(struct k_queue * queue);


static inline void * k_queue_peek_tail(struct k_queue * queue)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 741 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 741 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_queue_peek_tail(queue);
}


extern int z_impl_k_futex_wait(struct k_futex * futex, int expected, k_timeout_t timeout);


static inline int k_futex_wait(struct k_futex * futex, int expected, k_timeout_t timeout)
{
# 759 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 759 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 759 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_futex_wait(futex, expected, timeout);
}


extern int z_impl_k_futex_wake(struct k_futex * futex, 
# 764 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
                                                      _Bool 
# 764 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
                                                           wake_all);


static inline int k_futex_wake(struct k_futex * futex, 
# 767 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
                                                      _Bool 
# 767 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
                                                           wake_all)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 776 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 776 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_futex_wake(futex, wake_all);
}


extern void z_impl_k_event_init(struct k_event * event);


static inline void k_event_init(struct k_event * event)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 793 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 793 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_event_init(event);
}


extern uint32_t z_impl_k_event_post(struct k_event * event, uint32_t events);


static inline uint32_t k_event_post(struct k_event * event, uint32_t events)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 810 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 810 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_event_post(event, events);
}


extern uint32_t z_impl_k_event_set(struct k_event * event, uint32_t events);


static inline uint32_t k_event_set(struct k_event * event, uint32_t events)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 827 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 827 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_event_set(event, events);
}


extern uint32_t z_impl_k_event_set_masked(struct k_event * event, uint32_t events, uint32_t events_mask);


static inline uint32_t k_event_set_masked(struct k_event * event, uint32_t events, uint32_t events_mask)
{
# 845 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 845 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 845 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_event_set_masked(event, events, events_mask);
}


extern uint32_t z_impl_k_event_clear(struct k_event * event, uint32_t events);


static inline uint32_t k_event_clear(struct k_event * event, uint32_t events)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 862 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 862 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_event_clear(event, events);
}


extern uint32_t z_impl_k_event_wait(struct k_event * event, uint32_t events, 
# 867 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
                                                                            _Bool 
# 867 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
                                                                                 reset, k_timeout_t timeout);


static inline uint32_t k_event_wait(struct k_event * event, uint32_t events, 
# 870 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
                                                                            _Bool 
# 870 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
                                                                                 reset, k_timeout_t timeout)
{
# 881 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 881 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 881 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_event_wait(event, events, reset, timeout);
}


extern uint32_t z_impl_k_event_wait_all(struct k_event * event, uint32_t events, 
# 886 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
                                                                                _Bool 
# 886 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
                                                                                     reset, k_timeout_t timeout);


static inline uint32_t k_event_wait_all(struct k_event * event, uint32_t events, 
# 889 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
                                                                                _Bool 
# 889 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
                                                                                     reset, k_timeout_t timeout)
{
# 900 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 900 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 900 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_event_wait_all(event, events, reset, timeout);
}


extern int32_t z_impl_k_stack_alloc_init(struct k_stack * stack, uint32_t num_entries);


static inline int32_t k_stack_alloc_init(struct k_stack * stack, uint32_t num_entries)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 917 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 917 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_stack_alloc_init(stack, num_entries);
}


extern int z_impl_k_stack_push(struct k_stack * stack, stack_data_t data);


static inline int k_stack_push(struct k_stack * stack, stack_data_t data)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 934 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 934 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_stack_push(stack, data);
}


extern int z_impl_k_stack_pop(struct k_stack * stack, stack_data_t * data, k_timeout_t timeout);


static inline int k_stack_pop(struct k_stack * stack, stack_data_t * data, k_timeout_t timeout)
{
# 952 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 952 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 952 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_stack_pop(stack, data, timeout);
}


extern int z_impl_k_mutex_init(struct k_mutex * mutex);


static inline int k_mutex_init(struct k_mutex * mutex)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 968 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 968 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_mutex_init(mutex);
}


extern int z_impl_k_mutex_lock(struct k_mutex * mutex, k_timeout_t timeout);


static inline int k_mutex_lock(struct k_mutex * mutex, k_timeout_t timeout)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 985 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 985 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_mutex_lock(mutex, timeout);
}


extern int z_impl_k_mutex_unlock(struct k_mutex * mutex);


static inline int k_mutex_unlock(struct k_mutex * mutex)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1001 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1001 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_mutex_unlock(mutex);
}


extern int z_impl_k_condvar_init(struct k_condvar * condvar);


static inline int k_condvar_init(struct k_condvar * condvar)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1017 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1017 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_condvar_init(condvar);
}


extern int z_impl_k_condvar_signal(struct k_condvar * condvar);


static inline int k_condvar_signal(struct k_condvar * condvar)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1033 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1033 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_condvar_signal(condvar);
}


extern int z_impl_k_condvar_broadcast(struct k_condvar * condvar);


static inline int k_condvar_broadcast(struct k_condvar * condvar)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1049 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1049 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_condvar_broadcast(condvar);
}


extern int z_impl_k_condvar_wait(struct k_condvar * condvar, struct k_mutex * mutex, k_timeout_t timeout);


static inline int k_condvar_wait(struct k_condvar * condvar, struct k_mutex * mutex, k_timeout_t timeout)
{
# 1067 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1067 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1067 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_condvar_wait(condvar, mutex, timeout);
}


extern int z_impl_k_sem_init(struct k_sem * sem, unsigned int initial_count, unsigned int limit);


static inline int k_sem_init(struct k_sem * sem, unsigned int initial_count, unsigned int limit)
{
# 1085 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1085 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1085 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_sem_init(sem, initial_count, limit);
}


extern int z_impl_k_sem_take(struct k_sem * sem, k_timeout_t timeout);


static inline int k_sem_take(struct k_sem * sem, k_timeout_t timeout)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1102 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1102 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_sem_take(sem, timeout);
}


extern void z_impl_k_sem_give(struct k_sem * sem);


static inline void k_sem_give(struct k_sem * sem)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1119 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1119 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_sem_give(sem);
}


extern void z_impl_k_sem_reset(struct k_sem * sem);


static inline void k_sem_reset(struct k_sem * sem)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1136 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1136 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_sem_reset(sem);
}


extern unsigned int z_impl_k_sem_count_get(struct k_sem * sem);


static inline unsigned int k_sem_count_get(struct k_sem * sem)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1152 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1152 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_sem_count_get(sem);
}


extern int z_impl_k_msgq_alloc_init(struct k_msgq * msgq, size_t msg_size, uint32_t max_msgs);


static inline int k_msgq_alloc_init(struct k_msgq * msgq, size_t msg_size, uint32_t max_msgs)
{
# 1170 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1170 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1170 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_alloc_init(msgq, msg_size, max_msgs);
}


extern int z_impl_k_msgq_put(struct k_msgq * msgq, const void * data, k_timeout_t timeout);


static inline int k_msgq_put(struct k_msgq * msgq, const void * data, k_timeout_t timeout)
{
# 1188 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1188 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1188 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_put(msgq, data, timeout);
}


extern int z_impl_k_msgq_get(struct k_msgq * msgq, void * data, k_timeout_t timeout);


static inline int k_msgq_get(struct k_msgq * msgq, void * data, k_timeout_t timeout)
{
# 1206 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1206 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1206 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_get(msgq, data, timeout);
}


extern int z_impl_k_msgq_peek(struct k_msgq * msgq, void * data);


static inline int k_msgq_peek(struct k_msgq * msgq, void * data)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1223 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1223 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_peek(msgq, data);
}


extern int z_impl_k_msgq_peek_at(struct k_msgq * msgq, void * data, uint32_t idx);


static inline int k_msgq_peek_at(struct k_msgq * msgq, void * data, uint32_t idx)
{
# 1241 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1241 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1241 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_peek_at(msgq, data, idx);
}


extern void z_impl_k_msgq_purge(struct k_msgq * msgq);


static inline void k_msgq_purge(struct k_msgq * msgq)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1258 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1258 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_msgq_purge(msgq);
}


extern uint32_t z_impl_k_msgq_num_free_get(struct k_msgq * msgq);


static inline uint32_t k_msgq_num_free_get(struct k_msgq * msgq)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1274 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1274 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_num_free_get(msgq);
}


extern void z_impl_k_msgq_get_attrs(struct k_msgq * msgq, struct k_msgq_attrs * attrs);


static inline void k_msgq_get_attrs(struct k_msgq * msgq, struct k_msgq_attrs * attrs)
{
# 1292 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1292 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1292 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_msgq_get_attrs(msgq, attrs);
}


extern uint32_t z_impl_k_msgq_num_used_get(struct k_msgq * msgq);


static inline uint32_t k_msgq_num_used_get(struct k_msgq * msgq)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1308 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1308 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_msgq_num_used_get(msgq);
}


extern int z_impl_k_pipe_alloc_init(struct k_pipe * pipe, size_t size);


static inline int k_pipe_alloc_init(struct k_pipe * pipe, size_t size)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1325 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1325 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_pipe_alloc_init(pipe, size);
}


extern int z_impl_k_pipe_put(struct k_pipe * pipe, const void * data, size_t bytes_to_write, size_t * bytes_written, size_t min_xfer, k_timeout_t timeout);


static inline int k_pipe_put(struct k_pipe * pipe, const void * data, size_t bytes_to_write, size_t * bytes_written, size_t min_xfer, k_timeout_t timeout)
{
# 1350 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1350 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1350 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_pipe_put(pipe, data, bytes_to_write, bytes_written, min_xfer, timeout);
}


extern int z_impl_k_pipe_get(struct k_pipe * pipe, void * data, size_t bytes_to_read, size_t * bytes_read, size_t min_xfer, k_timeout_t timeout);


static inline int k_pipe_get(struct k_pipe * pipe, void * data, size_t bytes_to_read, size_t * bytes_read, size_t min_xfer, k_timeout_t timeout)
{
# 1375 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1375 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1375 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_pipe_get(pipe, data, bytes_to_read, bytes_read, min_xfer, timeout);
}


extern size_t z_impl_k_pipe_read_avail(struct k_pipe * pipe);


static inline size_t k_pipe_read_avail(struct k_pipe * pipe)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1391 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1391 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_pipe_read_avail(pipe);
}


extern size_t z_impl_k_pipe_write_avail(struct k_pipe * pipe);


static inline size_t k_pipe_write_avail(struct k_pipe * pipe)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1407 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1407 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_pipe_write_avail(pipe);
}


extern void z_impl_k_pipe_flush(struct k_pipe * pipe);


static inline void k_pipe_flush(struct k_pipe * pipe)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1424 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1424 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_pipe_flush(pipe);
}


extern void z_impl_k_pipe_buffer_flush(struct k_pipe * pipe);


static inline void k_pipe_buffer_flush(struct k_pipe * pipe)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1441 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1441 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_pipe_buffer_flush(pipe);
}


extern int z_impl_k_poll(struct k_poll_event * events, int num_events, k_timeout_t timeout);


static inline int k_poll(struct k_poll_event * events, int num_events, k_timeout_t timeout)
{
# 1459 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1459 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1459 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_poll(events, num_events, timeout);
}


extern void z_impl_k_poll_signal_init(struct k_poll_signal * sig);


static inline void k_poll_signal_init(struct k_poll_signal * sig)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1476 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1476 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_poll_signal_init(sig);
}


extern void z_impl_k_poll_signal_reset(struct k_poll_signal * sig);


static inline void k_poll_signal_reset(struct k_poll_signal * sig)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1493 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1493 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_poll_signal_reset(sig);
}


extern void z_impl_k_poll_signal_check(struct k_poll_signal * sig, unsigned int * signaled, int * result);


static inline void k_poll_signal_check(struct k_poll_signal * sig, unsigned int * signaled, int * result)
{
# 1512 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1512 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1512 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_poll_signal_check(sig, signaled, result);
}


extern int z_impl_k_poll_signal_raise(struct k_poll_signal * sig, int result);


static inline int k_poll_signal_raise(struct k_poll_signal * sig, int result)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1529 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1529 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_poll_signal_raise(sig, result);
}


extern void z_impl_k_str_out(char * c, size_t n);


static inline void k_str_out(char * c, size_t n)
{
# 1547 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1547 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1547 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 z_impl_k_str_out(c, n);
}


extern int z_impl_k_float_disable(struct k_thread * thread);


static inline int k_float_disable(struct k_thread * thread)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1563 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1563 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_float_disable(thread);
}


extern int z_impl_k_float_enable(struct k_thread * thread, unsigned int options);


static inline int k_float_enable(struct k_thread * thread, unsigned int options)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 1580 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h" 3 4
0
# 1580 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/kernel.h"
);
 return z_impl_k_float_enable(thread, options);
}
# 6251 "/home/ttwards/zephyrproject/zephyr/include/zephyr/kernel.h" 2
# 28 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 2







# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_instance.h" 1
# 17 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_instance.h"
struct log_source_const_data {
 const char *name;
 uint8_t level;







};


struct log_source_dynamic_data {
 uint32_t filters;
# 43 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_instance.h"
};
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h" 1
# 9 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 1
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mpsc_packet.h" 1
# 37 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/mpsc_packet.h"
struct mpsc_pbuf_hdr {
 uint32_t valid: 1; uint32_t busy: 1;
 uint32_t data: 32 - 2;
};


struct mpsc_pbuf_skip {
 uint32_t valid: 1; uint32_t busy: 1;
 uint32_t len: 32 - 2;
};


union mpsc_pbuf_generic {
 struct mpsc_pbuf_hdr hdr;
 struct mpsc_pbuf_skip skip;
 uint32_t raw;
};
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 2
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 1
# 11 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 2
# 45 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
struct cbprintf_package_desc {

 uint8_t len;


 uint8_t str_cnt;


 uint8_t ro_str_cnt;


 uint8_t rw_str_cnt;
# 72 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
} 
# 72 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 3 4
 __attribute__((__packed__))
# 72 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
         ;





union cbprintf_package_hdr {

 struct cbprintf_package_desc desc;

 void *raw;





} 
# 88 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 3 4
 __attribute__((__packed__))
# 88 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
         ;







struct cbprintf_package_hdr_ext {

 union cbprintf_package_hdr hdr;


 char *fmt;





} 
# 107 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 3 4
 __attribute__((__packed__))
# 107 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
         ;
# 124 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_internal.h" 1
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_internal.h"
# 1 "/home/ttwards/zephyr-sdk-0.16.8/arm-zephyr-eabi/lib/gcc/arm-zephyr-eabi/12.2.0/include/stddef.h" 1 3 4
# 13 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_internal.h" 2
# 55 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_internal.h"
static inline void z_cbprintf_wcpy(int *dst, int *src, size_t len)
{
 for (size_t i = 0; i < len; i++) {
  dst[i] = src[i];
 }
}

# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_cxx.h" 1
# 63 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_internal.h" 2
# 125 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 2
# 145 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
_Static_assert((((((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1))))) != 0) && ((((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1))))) & (((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1)))))-1)) == 0))), "" );
# 274 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_enums.h" 1
# 15 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf_enums.h"
enum cbprintf_package_arg_type {

 CBPRINTF_PACKAGE_ARG_TYPE_END = 0,

 CBPRINTF_PACKAGE_ARG_TYPE_CHAR,
 CBPRINTF_PACKAGE_ARG_TYPE_UNSIGNED_CHAR,

 CBPRINTF_PACKAGE_ARG_TYPE_SHORT,
 CBPRINTF_PACKAGE_ARG_TYPE_UNSIGNED_SHORT,

 CBPRINTF_PACKAGE_ARG_TYPE_INT,
 CBPRINTF_PACKAGE_ARG_TYPE_UNSIGNED_INT,

 CBPRINTF_PACKAGE_ARG_TYPE_LONG,
 CBPRINTF_PACKAGE_ARG_TYPE_UNSIGNED_LONG,

 CBPRINTF_PACKAGE_ARG_TYPE_LONG_LONG,
 CBPRINTF_PACKAGE_ARG_TYPE_UNSIGNED_LONG_LONG,

 CBPRINTF_PACKAGE_ARG_TYPE_FLOAT,
 CBPRINTF_PACKAGE_ARG_TYPE_DOUBLE,
 CBPRINTF_PACKAGE_ARG_TYPE_LONG_DOUBLE,

 CBPRINTF_PACKAGE_ARG_TYPE_PTR_CHAR,

 CBPRINTF_PACKAGE_ARG_TYPE_PTR_VOID,

 CBPRINTF_PACKAGE_ARG_TYPE_MAX,

 CBPRINTF_PACKAGE_ARG_TYPE_COUNT = CBPRINTF_PACKAGE_ARG_TYPE_MAX
};
# 275 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 2
# 296 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
typedef int (*cbprintf_cb)( );






typedef int (*cbprintf_cb_local)(int c, void *ctx);
# 313 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
typedef int (*cbprintf_convert_cb)(const void *buf, size_t len, void *ctx);
# 333 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
typedef int (*cbvprintf_external_formatter_func)(cbprintf_cb out, void *ctx,
       const char *fmt, va_list ap);
# 430 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
__attribute__((format (printf, 4, 5)))
int cbprintf_package(void *packaged,
       size_t len,
       uint32_t flags,
       const char *format,
       ...);
# 471 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
int cbvprintf_package(void *packaged,
        size_t len,
        uint32_t flags,
        const char *format,
        va_list ap);
# 512 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
int cbprintf_package_convert(void *in_packaged,
        size_t in_len,
        cbprintf_convert_cb cb,
        void *ctx,
        uint32_t flags,
        uint16_t *strl,
        size_t strl_len);


struct z_cbprintf_buf_desc {
 void *buf;
 size_t size;
 size_t off;
};


static inline int z_cbprintf_cpy(const void *buf, size_t len, void *ctx)
{
 struct z_cbprintf_buf_desc *desc = (struct z_cbprintf_buf_desc *)ctx;

 if ((desc->size - desc->off) < len) {
  return -
# 533 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 3 4
         28
# 533 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
               ;
 }

 memcpy(&((uint8_t *)desc->buf)[desc->off], buf, len);
 desc->off += len;

 return len;
}
# 572 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
static inline int cbprintf_package_copy(void *in_packaged,
     size_t in_len,
     void *packaged,
     size_t len,
     uint32_t flags,
     uint16_t *strl,
     size_t strl_len)
{
 struct z_cbprintf_buf_desc buf_desc = {
  .buf = packaged,
  .size = len,
  .off = 0,
 };

 return cbprintf_package_convert(in_packaged, in_len,
     packaged ? z_cbprintf_cpy : 
# 587 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 3 4
                                ((void *)0)
# 587 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
                                    , &buf_desc,
     flags, strl, strl_len);
}
# 620 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
static inline int cbprintf_fsc_package(void *in_packaged,
           size_t in_len,
           void *packaged,
           size_t len)
{
 return cbprintf_package_copy(in_packaged, in_len, packaged, len,
         (1UL << (0)) |
         (1UL << (1)), 
# 627 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h" 3 4
                                         ((void *)0)
# 627 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
                                             , 0);
}
# 650 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
int cbpprintf_external(cbprintf_cb out,
         cbvprintf_external_formatter_func formatter,
         void *ctx,
         void *packaged);
# 681 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
__attribute__((format (printf, 3, 4)))
int cbprintf(cbprintf_cb out, void *ctx, const char *format, ...);
# 712 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
int z_cbvprintf_impl(cbprintf_cb out, void *ctx, const char *format,
       va_list ap, uint32_t flags);
# 741 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
int cbvprintf(cbprintf_cb out, void *ctx, const char *format, va_list ap);
# 777 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
static inline
int cbvprintf_tagged_args(cbprintf_cb out, void *ctx,
     const char *format, va_list ap)
{
 return z_cbvprintf_impl(out, ctx, format, ap,
    (1UL << (0)));
}
# 802 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
static inline
int cbpprintf(cbprintf_cb out, void *ctx, void *packaged)
{
# 816 "/home/ttwards/zephyrproject/zephyr/include/zephyr/sys/cbprintf.h"
 return cbpprintf_external(out, cbvprintf, ctx, packaged);
}
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 2
# 36 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
typedef uint32_t log_timestamp_t;
# 56 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
struct log_msg_desc {
 uint32_t valid: 1; uint32_t busy: 1; uint32_t type:1;
 uint32_t domain:3;
 uint32_t level:3;
 uint32_t package_len:11;
 uint32_t data_len:12;
};

union log_msg_source {
 const struct log_source_const_data *fixed;
 struct log_source_dynamic_data *dynamic;
 void *raw;
};

struct log_msg_hdr {
 struct log_msg_desc desc;







 const void *source;
 log_timestamp_t timestamp;




};
# 94 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
struct log_msg {
 struct log_msg_hdr hdr;



 uint8_t padding[((sizeof(struct log_msg_hdr) % (((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1))))) > 0 ? ((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1)))) - (sizeof(struct log_msg_hdr) % (((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1)))))) : 0)];
 uint8_t data[];
};




_Static_assert((sizeof(struct log_msg) % (((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1)))) == 0), "" "Log msg size must aligned")
                                  ;





struct log_msg_generic_hdr {
 uint32_t valid: 1; uint32_t busy: 1; uint32_t type:1;
};

union log_msg_generic {
 union mpsc_pbuf_generic buf;
 struct log_msg_generic_hdr generic;
 struct log_msg log;
};





enum z_log_msg_mode {



 Z_LOG_MSG_MODE_RUNTIME,




 Z_LOG_MSG_MODE_FROM_STACK,





 Z_LOG_MSG_MODE_ZERO_COPY,


 Z_LOG_MSG_MODE_SIMPLE,
};
# 595 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
struct log_msg *z_log_msg_alloc(uint32_t wlen);
# 610 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
void z_log_msg_finalize(struct log_msg *msg, const void *source,
    const struct log_msg_desc desc, const void *data);







static inline void z_log_msg_simple_create_0(const void *source, uint32_t level,
      const char *fmt);
# 629 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
static inline void z_log_msg_simple_create_1(const void *source, uint32_t level,
      const char *fmt, uint32_t arg);
# 640 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
static inline void z_log_msg_simple_create_2(const void *source, uint32_t level,
      const char *fmt, uint32_t arg0, uint32_t arg1);
# 653 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
static inline void z_log_msg_static_create(const void *source,
     const struct log_msg_desc desc,
     uint8_t *package, const void *data);
# 678 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
void z_log_msg_runtime_vcreate(uint8_t domain_id, const void *source,
    uint8_t level, const void *data,
    size_t dlen, uint32_t package_flags,
    const char *fmt,
    va_list ap);
# 705 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
static inline void z_log_msg_runtime_create(uint8_t domain_id,
          const void *source,
          uint8_t level, const void *data,
          size_t dlen, uint32_t package_flags,
          const char *fmt, ...)
{
 va_list ap;

 
# 713 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
__builtin_va_start(
# 713 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
ap
# 713 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
,
# 713 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
fmt
# 713 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
)
# 713 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
                 ;
 z_log_msg_runtime_vcreate(domain_id, source, level,
       data, dlen, package_flags, fmt, ap);
 
# 716 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
__builtin_va_end(
# 716 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
ap
# 716 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
)
# 716 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
          ;
}

static inline 
# 719 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
             _Bool 
# 719 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
                  z_log_item_is_msg(const union log_msg_generic *msg)
{
 return msg->generic.type == 0;
}







static inline uint32_t log_msg_get_total_wlen(const struct log_msg_desc desc)
{
 return (((((((unsigned long)((
# 732 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
       __builtin_offsetof (
# 732 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
       struct log_msg
# 732 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
       , 
# 732 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
       data
# 732 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
       ) 
# 732 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
       + (desc.package_len) + (desc.data_len))) + ((unsigned long)((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1))))) - 1)) / (unsigned long)((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1)))))) * (unsigned long)((((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) <= 2UL ? ((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) : (1UL << (8 * sizeof(long) - __builtin_clzl(((((sizeof(double)) > (sizeof(long long))) ? (sizeof(double)) : (sizeof(long long)))) - 1))))))) + (sizeof(uint32_t)) - 1) / (sizeof(uint32_t)));
}







static inline uint32_t log_msg_generic_get_wlen(const union mpsc_pbuf_generic *item)
{
 const union log_msg_generic *generic_msg = (const union log_msg_generic *)item;

 if (z_log_item_is_msg(generic_msg)) {
  const struct log_msg *msg = (const struct log_msg *)generic_msg;

  return log_msg_get_total_wlen(msg->hdr.desc);
 }

 return 0;
}







static inline uint8_t log_msg_get_domain(struct log_msg *msg)
{
 return msg->hdr.desc.domain;
}







static inline uint8_t log_msg_get_level(struct log_msg *msg)
{
 return msg->hdr.desc.level;
}







static inline const void *log_msg_get_source(struct log_msg *msg)
{
 return msg->hdr.source;
}







int16_t log_msg_get_source_id(struct log_msg *msg);







static inline log_timestamp_t log_msg_get_timestamp(struct log_msg *msg)
{
 return msg->hdr.timestamp;
}







static inline void *log_msg_get_tid(struct log_msg *msg)
{



 (void)(msg);
 return 
# 818 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 3 4
       ((void *)0)
# 818 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
           ;

}
# 830 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
static inline uint8_t *log_msg_get_data(struct log_msg *msg, size_t *len)
{
 *len = msg->hdr.desc.data_len;

 return msg->data + msg->hdr.desc.package_len;
}
# 845 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h"
static inline uint8_t *log_msg_get_package(struct log_msg *msg, size_t *len)
{
 *len = msg->hdr.desc.package_len;

 return msg->data;
}





# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h" 1
# 23 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
extern void z_impl_z_log_msg_simple_create_0(const void * source, uint32_t level, const char * fmt);


static inline void z_log_msg_simple_create_0(const void * source, uint32_t level, const char * fmt)
{
# 37 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 37 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h" 3 4
0
# 37 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
);
 z_impl_z_log_msg_simple_create_0(source, level, fmt);
}
# 49 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
extern void z_impl_z_log_msg_simple_create_1(const void * source, uint32_t level, const char * fmt, uint32_t arg);


static inline void z_log_msg_simple_create_1(const void * source, uint32_t level, const char * fmt, uint32_t arg)
{
# 64 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 64 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h" 3 4
0
# 64 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
);
 z_impl_z_log_msg_simple_create_1(source, level, fmt, arg);
}
# 76 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
extern void z_impl_z_log_msg_simple_create_2(const void * source, uint32_t level, const char * fmt, uint32_t arg0, uint32_t arg1);


static inline void z_log_msg_simple_create_2(const void * source, uint32_t level, const char * fmt, uint32_t arg0, uint32_t arg1)
{
# 92 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 92 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h" 3 4
0
# 92 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
);
 z_impl_z_log_msg_simple_create_2(source, level, fmt, arg0, arg1);
}
# 104 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
extern void z_impl_z_log_msg_static_create(const void * source, const struct log_msg_desc desc, uint8_t * package, const void * data);


static inline void z_log_msg_static_create(const void * source, const struct log_msg_desc desc, uint8_t * package, const void * data)
{
# 119 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 119 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h" 3 4
0
# 119 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/log_msg.h"
);
 z_impl_z_log_msg_static_create(source, desc, package, data);
}
# 857 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_msg.h" 2
# 10 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h" 2
# 216 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
void z_log_minimal_hexdump_print(int level, const void *data, size_t size);
void z_log_minimal_vprintk(const char *fmt, va_list ap);
void z_log_minimal_printk(const char *fmt, ...);
# 232 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
static inline char z_log_minimal_level_to_char(int level)
{
 switch (level) {
 case 1:
  return 'E';
 case 2:
  return 'W';
 case 3:
  return 'I';
 case 4:
  return 'D';
 default:
  return '?';
 }
}
# 430 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
extern struct log_source_const_data _log_const_list_start[];
extern struct log_source_const_data _log_const_list_end[];
# 467 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
static inline uint32_t log_const_source_id(
    const struct log_source_const_data *data)
{
 return ((const uint8_t *)data - (uint8_t *)_log_const_list_start)/
   sizeof(struct log_source_const_data);
}

extern struct log_source_dynamic_data _log_dynamic_list_start[];
extern struct log_source_dynamic_data _log_dynamic_list_end[];
# 493 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
static inline uint32_t log_dynamic_source_id(struct log_source_dynamic_data *data)
{
 return ((uint8_t *)data - (uint8_t *)_log_dynamic_list_start)/
   sizeof(struct log_source_dynamic_data);
}
# 506 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
static inline uint32_t log_source_id(const void *source)
{
 return 0 ?
  log_dynamic_source_id((struct log_source_dynamic_data *)source) :
  log_const_source_id((const struct log_source_const_data *)source);
}


static inline __attribute__((format (printf, 1, 2)))
void z_log_printf_arg_checker(const char *fmt, ...)
{
 (void)(fmt);
}
# 529 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
static inline void log_generic(uint8_t level, const char *fmt, va_list ap)
{
 z_log_msg_runtime_vcreate(0, 
# 531 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h" 3 4
                                                 ((void *)0)
# 531 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
                                                     , level,
       
# 532 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h" 3 4
      ((void *)0)
# 532 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log_core.h"
          , 0, 0, fmt, ap);
}
# 12 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log.h" 2
# 299 "/home/ttwards/zephyrproject/zephyr/include/zephyr/logging/log.h"
void z_log_vprintk(const char *fmt, va_list ap);
# 36 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 2
# 50 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
enum motor_mode {
 MIT,
 PV,
 VO,
 MULTILOOP
};




enum motor_cmd {
 ENABLE_MOTOR,
 DISABLE_MOTOR,
 SET_ZERO_OFFSET,
 CLEAR_PID,
 CLEAR_ERROR
};

struct motor_driver_config {

 const struct device *phy;

 uint8_t id;

 int tx_id;

 int rx_id;

 char capabilities[4][12];
 struct pid_data *pid_datas[4];
};

struct motor_driver_data {
 float angle;
 float rpm;
 float torque;
 float temperature;
 int round_cnt;

 float speed_limit[2];
 float torque_limit[2];

 enum motor_mode mode;
};







typedef float (*motor_api_stat_speed_t)(const struct device *dev);
# 110 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef float (*motor_api_stat_torque_t)(const struct device *dev);
# 119 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef float (*motor_api_stat_angle_t)(const struct device *dev);
# 129 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef int (*motor_api_speed_t)(const struct device *dev, float speed_rpm);
# 139 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef int (*motor_api_torque_t)(const struct device *dev, float torque);
# 149 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef int (*motor_api_angle_t)(const struct device *dev, float angle);
# 159 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef void (*motor_api_ctrl_t)(const struct device *dev, enum motor_cmd cmd);
# 169 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
typedef int (*motor_api_mode_t)(const struct device *dev, enum motor_mode mode);

typedef void (*motor_limit_speed_t)(const struct device *dev, float max_speed, float min_speed);

typedef void (*motor_limit_torque_t)(const struct device *dev, float max_torque, float min_torque);




 struct motor_driver_api {
 motor_api_stat_speed_t motor_get_speed;
 motor_api_stat_torque_t motor_get_torque;
 motor_api_stat_torque_t motor_get_angle;
 motor_api_speed_t motor_set_speed;
 motor_api_torque_t motor_set_torque;
 motor_api_angle_t motor_set_angle;
 motor_api_mode_t motor_set_mode;
 motor_api_ctrl_t motor_control;
 motor_limit_speed_t motor_limit_speed;
 motor_limit_torque_t motor_limit_torque;
};




enum MotorError {
 MOTOR_SUCCESS = 0,
 MOTOR_ERROR_INIT = -1,
 MOTOR_ERROR_COMM = -2,
 MOTOR_ERROR_UNKNOWN = -3
};







static inline float motor_get_torque(const struct device *dev);

static inline float z_impl_motor_get_torque(const struct device *dev)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_get_torque == 
# 212 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                             ((void *)0)
# 212 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                 ) {
  return -
# 213 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 213 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_get_torque(dev);
}







static inline float motor_get_speed(const struct device *dev);

static inline float z_impl_motor_get_speed(const struct device *dev)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_get_speed == 
# 229 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                            ((void *)0)
# 229 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                ) {
  return -
# 230 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 230 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_get_speed(dev);
}







static inline float motor_get_angle(const struct device *dev);

static inline float z_impl_motor_get_angle(const struct device *dev)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_get_angle == 
# 246 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                            ((void *)0)
# 246 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                ) {
  return -
# 247 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 247 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_get_angle(dev);
}
# 259 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
static inline int motor_set_speed(const struct device *dev, float speed_rpm);

static inline int z_impl_motor_set_speed(const struct device *dev, float speed_rpm)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_set_speed == 
# 264 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                            ((void *)0)
# 264 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                ) {
  return -
# 265 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 265 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_set_speed(dev, speed_rpm);
}
# 277 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
static inline int motor_set_angle(const struct device *dev, float angle);

static inline int z_impl_motor_set_angle(const struct device *dev, float angle)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_set_angle == 
# 282 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                            ((void *)0)
# 282 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                ) {
  return -
# 283 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 283 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_set_angle(dev, angle);
}
# 295 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
static inline int motor_set_torque(const struct device *dev, float torque);

static inline int z_impl_motor_set_torque(const struct device *dev, float torque)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_set_torque == 
# 300 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                             ((void *)0)
# 300 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                 ) {
  return -
# 301 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 301 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_set_torque(dev, torque);
}
# 313 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
static inline void motor_control(const struct device *dev, enum motor_cmd cmd);

static inline void z_impl_motor_control(const struct device *dev, enum motor_cmd cmd)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_control == 
# 318 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                          ((void *)0)
# 318 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                              ) {
  return;
 }
 api->motor_control(dev, cmd);
 return;
}
# 332 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
static inline int motor_set_mode(const struct device *dev, enum motor_mode mode);

static inline int z_impl_motor_set_mode(const struct device *dev, enum motor_mode mode)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_set_mode == 
# 337 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                           ((void *)0)
# 337 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                               ) {
  return -
# 338 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
         88
# 338 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
               ;
 }
 return api->motor_set_mode(dev, mode);
}

static inline void motor_limit_speed(const struct device *dev, float max_speed, float min_speed);

static inline void z_impl_motor_limit_speed(const struct device *dev, float max_speed,
         float min_speed)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_limit_speed == 
# 349 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                              ((void *)0)
# 349 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                  ) {
  return;
 }
 api->motor_limit_speed(dev, max_speed, min_speed);
}

static inline void motor_limit_torque(const struct device *dev, float max_torque, float min_torque);

static inline void z_impl_motor_limit_torque(const struct device *dev, float max_torque,
          float min_torque)
{
 const struct motor_driver_api *api = (const struct motor_driver_api *)dev->api;
 if (api->motor_limit_torque == 
# 361 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 3 4
                               ((void *)0)
# 361 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
                                   ) {
  return;
 }
 api->motor_limit_torque(dev, max_torque, min_torque);
}
# 411 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h"
# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 1
# 23 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern float z_impl_motor_get_torque(const struct device * dev);


static inline float motor_get_torque(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 34 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 34 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_get_torque(dev);
}
# 46 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern float z_impl_motor_get_speed(const struct device * dev);


static inline float motor_get_speed(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 57 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 57 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_get_speed(dev);
}
# 69 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern float z_impl_motor_get_angle(const struct device * dev);


static inline float motor_get_angle(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 80 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 80 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_get_angle(dev);
}
# 92 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern int z_impl_motor_set_speed(const struct device * dev, float speed_rpm);


static inline int motor_set_speed(const struct device * dev, float speed_rpm)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 104 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 104 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_set_speed(dev, speed_rpm);
}
# 116 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern int z_impl_motor_set_angle(const struct device * dev, float angle);


static inline int motor_set_angle(const struct device * dev, float angle)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 128 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 128 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_set_angle(dev, angle);
}
# 140 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern int z_impl_motor_set_torque(const struct device * dev, float torque);


static inline int motor_set_torque(const struct device * dev, float torque)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 152 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 152 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_set_torque(dev, torque);
}
# 164 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern void z_impl_motor_control(const struct device * dev, enum motor_cmd cmd);


static inline void motor_control(const struct device * dev, enum motor_cmd cmd)
{
# 177 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 177 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 177 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 z_impl_motor_control(dev, cmd);
}
# 189 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern int z_impl_motor_set_mode(const struct device * dev, enum motor_mode mode);


static inline int motor_set_mode(const struct device * dev, enum motor_mode mode)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 201 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 201 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 return z_impl_motor_set_mode(dev, mode);
}
# 213 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern void z_impl_motor_limit_speed(const struct device * dev, float max_speed, float min_speed);


static inline void motor_limit_speed(const struct device * dev, float max_speed, float min_speed)
{
# 227 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 227 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 227 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 z_impl_motor_limit_speed(dev, max_speed, min_speed);
}
# 239 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
extern void z_impl_motor_limit_torque(const struct device * dev, float max_torque, float min_torque);


static inline void motor_limit_torque(const struct device * dev, float max_torque, float min_torque)
{
# 253 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 253 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h" 3 4
0
# 253 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/motor.h"
);
 z_impl_motor_limit_torque(dev, max_torque, min_torque);
}
# 412 "/home/ttwards/zephyrproject/motor/include/zephyr/drivers/motor.h" 2
# 9 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_macros.h" 2
# 95 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/dji_macros.h"
typedef uint8_t motor_mode_t;
# 14 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 2



# 1 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 1
# 125 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
typedef uint32_t can_mode_t;




enum can_state {

 CAN_STATE_ERROR_ACTIVE,

 CAN_STATE_ERROR_WARNING,

 CAN_STATE_ERROR_PASSIVE,

 CAN_STATE_BUS_OFF,

 CAN_STATE_STOPPED,
};
# 172 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
struct can_frame {

 uint32_t id;

 uint8_t dlc;

 uint8_t flags;
# 191 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
 uint16_t reserved;



 union {

  uint8_t data[8U];

  uint32_t data_32[(((8U) + (sizeof(uint32_t)) - 1) / (sizeof(uint32_t)))];
 };
};
# 218 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
struct can_filter {

 uint32_t id;



 uint32_t mask;

 uint8_t flags;
};




struct can_bus_err_cnt {

 uint8_t tx_err_cnt;

 uint8_t rx_err_cnt;
};
# 271 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
struct can_timing {

 uint16_t sjw;

 uint16_t prop_seg;

 uint16_t phase_seg1;

 uint16_t phase_seg2;

 uint16_t prescaler;
};
# 292 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
typedef void (*can_tx_callback_t)(const struct device *dev, int error, void *user_data);
# 301 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
typedef void (*can_rx_callback_t)(const struct device *dev, struct can_frame *frame,
      void *user_data);
# 312 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
typedef void (*can_state_change_callback_t)(const struct device *dev,
         enum can_state state,
         struct can_bus_err_cnt err_cnt,
         void *user_data);
# 346 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
struct can_driver_config {

 const struct device *phy;

 uint32_t min_bitrate;

 uint32_t max_bitrate;

 uint32_t bitrate;

 uint16_t sample_point;






};
# 403 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
struct can_driver_data {

 can_mode_t mode;

 
# 407 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
_Bool 
# 407 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
     started;

 can_state_change_callback_t state_change_cb;

 void *state_change_cb_user_data;
};





typedef int (*can_set_timing_t)(const struct device *dev,
    const struct can_timing *timing);





typedef int (*can_set_timing_data_t)(const struct device *dev,
         const struct can_timing *timing_data);





typedef int (*can_get_capabilities_t)(const struct device *dev, can_mode_t *cap);





typedef int (*can_start_t)(const struct device *dev);





typedef int (*can_stop_t)(const struct device *dev);





typedef int (*can_set_mode_t)(const struct device *dev, can_mode_t mode);
# 459 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
typedef int (*can_send_t)(const struct device *dev,
     const struct can_frame *frame,
     k_timeout_t timeout, can_tx_callback_t callback,
     void *user_data);





typedef int (*can_add_rx_filter_t)(const struct device *dev,
       can_rx_callback_t callback,
       void *user_data,
       const struct can_filter *filter);





typedef void (*can_remove_rx_filter_t)(const struct device *dev, int filter_id);





typedef int (*can_recover_t)(const struct device *dev, k_timeout_t timeout);





typedef int (*can_get_state_t)(const struct device *dev, enum can_state *state,
          struct can_bus_err_cnt *err_cnt);





typedef void(*can_set_state_change_callback_t)(const struct device *dev,
            can_state_change_callback_t callback,
            void *user_data);





typedef int (*can_get_core_clock_t)(const struct device *dev, uint32_t *rate);





typedef int (*can_get_max_filters_t)(const struct device *dev, 
# 510 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
                                                              _Bool 
# 510 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
                                                                   ide);

 struct can_driver_api {
 can_get_capabilities_t get_capabilities;
 can_start_t start;
 can_stop_t stop;
 can_set_mode_t set_mode;
 can_set_timing_t set_timing;
 can_send_t send;
 can_add_rx_filter_t add_rx_filter;
 can_remove_rx_filter_t remove_rx_filter;



 can_get_state_t get_state;
 can_set_state_change_callback_t set_state_change_callback;
 can_get_core_clock_t get_core_clock;
 can_get_max_filters_t get_max_filters;

 struct can_timing timing_min;

 struct can_timing timing_max;







};
# 824 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_get_core_clock(const struct device *dev, uint32_t *rate);

static inline int z_impl_can_get_core_clock(const struct device *dev, uint32_t *rate)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return api->get_core_clock(dev, rate);
}
# 841 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_get_bitrate_min(const struct device *dev);

static inline uint32_t z_impl_can_get_bitrate_min(const struct device *dev)
{
 const struct can_driver_config *common = (const struct can_driver_config *)dev->config;

 return common->min_bitrate;
}
# 863 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
__attribute__((deprecated)) static inline int can_get_min_bitrate(const struct device *dev, uint32_t *min_bitrate)
{
 *min_bitrate = can_get_bitrate_min(dev);

 return 0;
}
# 878 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_get_bitrate_max(const struct device *dev);

static inline uint32_t z_impl_can_get_bitrate_max(const struct device *dev)
{
 const struct can_driver_config *common = (const struct can_driver_config *)dev->config;

 return common->max_bitrate;
}
# 901 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
__attribute__((deprecated)) static inline int can_get_max_bitrate(const struct device *dev, uint32_t *max_bitrate)
{
 *max_bitrate = can_get_bitrate_max(dev);

 return 0;
}
# 915 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline const struct can_timing *can_get_timing_min(const struct device *dev);

static inline const struct can_timing *z_impl_can_get_timing_min(const struct device *dev)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return &api->timing_min;
}
# 931 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline const struct can_timing *can_get_timing_max(const struct device *dev);

static inline const struct can_timing *z_impl_can_get_timing_max(const struct device *dev)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return &api->timing_max;
}
# 966 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_calc_timing(const struct device *dev, struct can_timing *res,
         uint32_t bitrate, uint16_t sample_pnt);
# 982 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline const struct can_timing *can_get_timing_data_min(const struct device *dev);
# 1006 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline const struct can_timing *can_get_timing_data_max(const struct device *dev);
# 1037 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_calc_timing_data(const struct device *dev, struct can_timing *res,
       uint32_t bitrate, uint16_t sample_pnt);
# 1057 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_set_timing_data(const struct device *dev,
      const struct can_timing *timing_data);
# 1088 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_set_bitrate_data(const struct device *dev, uint32_t bitrate_data);
# 1110 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
__attribute__((deprecated)) int can_calc_prescaler(const struct device *dev, struct can_timing *timing,
        uint32_t bitrate);
# 1126 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_set_timing(const struct device *dev,
        const struct can_timing *timing);
# 1142 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_get_capabilities(const struct device *dev, can_mode_t *cap);

static inline int z_impl_can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return api->get_capabilities(dev, cap);
}
# 1160 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline const struct device *can_get_transceiver(const struct device *dev);

static const struct device *z_impl_can_get_transceiver(const struct device *dev)
{
 const struct can_driver_config *common = (const struct can_driver_config *)dev->config;

 return common->phy;
}
# 1186 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_start(const struct device *dev);

static inline int z_impl_can_start(const struct device *dev)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return api->start(dev);
}
# 1210 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_stop(const struct device *dev);

static inline int z_impl_can_stop(const struct device *dev)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return api->stop(dev);
}
# 1229 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_set_mode(const struct device *dev, can_mode_t mode);

static inline int z_impl_can_set_mode(const struct device *dev, can_mode_t mode)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return api->set_mode(dev, mode);
}
# 1245 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline can_mode_t can_get_mode(const struct device *dev);

static inline can_mode_t z_impl_can_get_mode(const struct device *dev)
{
 const struct can_driver_data *common = (const struct can_driver_data *)dev->data;

 return common->mode;
}
# 1279 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_set_bitrate(const struct device *dev, uint32_t bitrate);
# 1333 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_send(const struct device *dev, const struct can_frame *frame,
         k_timeout_t timeout, can_tx_callback_t callback,
         void *user_data);
# 1368 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
int can_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
        void *user_data, const struct can_filter *filter);
# 1410 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_add_rx_filter_msgq(const struct device *dev, struct k_msgq *msgq,
         const struct can_filter *filter);
# 1422 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline void can_remove_rx_filter(const struct device *dev, int filter_id);

static inline void z_impl_can_remove_rx_filter(const struct device *dev, int filter_id)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 api->remove_rx_filter(dev, filter_id);
}
# 1444 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_get_max_filters(const struct device *dev, 
# 1444 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
                                                           _Bool 
# 1444 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
                                                                ide);

static inline int z_impl_can_get_max_filters(const struct device *dev, 
# 1446 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
                                                                      _Bool 
# 1446 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
                                                                           ide)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 if (api->get_max_filters == 
# 1450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
                            ((void *)0)
# 1450 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
                                ) {
  return -
# 1451 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
         88
# 1451 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
               ;
 }

 return api->get_max_filters(dev, ide);
}
# 1478 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_get_state(const struct device *dev, enum can_state *state,
       struct can_bus_err_cnt *err_cnt);

static inline int z_impl_can_get_state(const struct device *dev, enum can_state *state,
           struct can_bus_err_cnt *err_cnt)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 return api->get_state(dev, state, err_cnt);
}
# 1506 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline int can_recover(const struct device *dev, k_timeout_t timeout);
# 1534 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline void can_set_state_change_callback(const struct device *dev,
       can_state_change_callback_t callback,
       void *user_data)
{
 const struct can_driver_api *api = (const struct can_driver_api *)dev->api;

 api->set_state_change_callback(dev, callback, user_data);
}
# 1563 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_bit_errors(const struct device *dev);
# 1586 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_bit0_errors(const struct device *dev);
# 1609 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_bit1_errors(const struct device *dev);
# 1630 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_stuff_errors(const struct device *dev);
# 1651 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_crc_errors(const struct device *dev);
# 1672 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_form_errors(const struct device *dev);
# 1693 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_ack_errors(const struct device *dev);
# 1715 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint32_t can_stats_get_rx_overruns(const struct device *dev);
# 1739 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint8_t can_dlc_to_bytes(uint8_t dlc)
{
 static const uint8_t dlc_table[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12,
         16, 20, 24, 32, 48, 64};

 return dlc_table[(((dlc) < (((size_t) (((int) sizeof(char[1 - 2 * !(!__builtin_types_compatible_p(__typeof__(dlc_table), __typeof__(&(dlc_table)[0])))]) - 1) + (sizeof(dlc_table) / sizeof((dlc_table)[0])))) - 1)) ? (dlc) : (((size_t) (((int) sizeof(char[1 - 2 * !(!__builtin_types_compatible_p(__typeof__(dlc_table), __typeof__(&(dlc_table)[0])))]) - 1) + (sizeof(dlc_table) / sizeof((dlc_table)[0])))) - 1))];
}
# 1754 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline uint8_t can_bytes_to_dlc(uint8_t num_bytes)
{
 return num_bytes <= 8 ? num_bytes :
        num_bytes <= 12 ? 9 :
        num_bytes <= 16 ? 10 :
        num_bytes <= 20 ? 11 :
        num_bytes <= 24 ? 12 :
        num_bytes <= 32 ? 13 :
        num_bytes <= 48 ? 14 :
        15;
}
# 1773 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
static inline 
# 1773 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
             _Bool 
# 1773 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
                  can_frame_matches_filter(const struct can_frame *frame,
         const struct can_filter *filter)
{
 if ((frame->flags & (1UL << (0))) != 0 && (filter->flags & (1UL << (0))) == 0) {

  return 
# 1778 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
        0
# 1778 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
             ;
 }

 if ((frame->flags & (1UL << (0))) == 0 && (filter->flags & (1UL << (0))) != 0) {

  return 
# 1783 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
        0
# 1783 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
             ;
 }

 if ((frame->id ^ filter->id) & filter->mask) {

  return 
# 1788 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
        0
# 1788 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
             ;
 }

 return 
# 1791 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 3 4
       1
# 1791 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
           ;
}
# 1804 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h"
# 1 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 1
# 23 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_get_core_clock(const struct device * dev, uint32_t * rate);


static inline int can_get_core_clock(const struct device * dev, uint32_t * rate)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 35 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 35 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_core_clock(dev, rate);
}
# 47 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_get_bitrate_min(const struct device * dev);


static inline uint32_t can_get_bitrate_min(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 58 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 58 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_bitrate_min(dev);
}
# 70 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_get_bitrate_max(const struct device * dev);


static inline uint32_t can_get_bitrate_max(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 81 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 81 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_bitrate_max(dev);
}
# 93 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern const struct can_timing * z_impl_can_get_timing_min(const struct device * dev);


static inline const struct can_timing * can_get_timing_min(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 104 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 104 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_timing_min(dev);
}
# 116 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern const struct can_timing * z_impl_can_get_timing_max(const struct device * dev);


static inline const struct can_timing * can_get_timing_max(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 127 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 127 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_timing_max(dev);
}
# 139 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_calc_timing(const struct device * dev, struct can_timing * res, uint32_t bitrate, uint16_t sample_pnt);


static inline int can_calc_timing(const struct device * dev, struct can_timing * res, uint32_t bitrate, uint16_t sample_pnt)
{
# 153 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 153 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 153 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_calc_timing(dev, res, bitrate, sample_pnt);
}
# 165 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern const struct can_timing * z_impl_can_get_timing_data_min(const struct device * dev);


static inline const struct can_timing * can_get_timing_data_min(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 176 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 176 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_timing_data_min(dev);
}
# 188 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern const struct can_timing * z_impl_can_get_timing_data_max(const struct device * dev);


static inline const struct can_timing * can_get_timing_data_max(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 199 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 199 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_timing_data_max(dev);
}
# 211 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_calc_timing_data(const struct device * dev, struct can_timing * res, uint32_t bitrate, uint16_t sample_pnt);


static inline int can_calc_timing_data(const struct device * dev, struct can_timing * res, uint32_t bitrate, uint16_t sample_pnt)
{
# 225 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 225 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 225 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_calc_timing_data(dev, res, bitrate, sample_pnt);
}
# 237 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_set_timing_data(const struct device * dev, const struct can_timing * timing_data);


static inline int can_set_timing_data(const struct device * dev, const struct can_timing * timing_data)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 249 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 249 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_set_timing_data(dev, timing_data);
}
# 261 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_set_bitrate_data(const struct device * dev, uint32_t bitrate_data);


static inline int can_set_bitrate_data(const struct device * dev, uint32_t bitrate_data)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 273 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 273 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_set_bitrate_data(dev, bitrate_data);
}
# 285 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_set_timing(const struct device * dev, const struct can_timing * timing);


static inline int can_set_timing(const struct device * dev, const struct can_timing * timing)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 297 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 297 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_set_timing(dev, timing);
}
# 309 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_get_capabilities(const struct device * dev, can_mode_t * cap);


static inline int can_get_capabilities(const struct device * dev, can_mode_t * cap)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 321 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 321 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_capabilities(dev, cap);
}
# 333 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern const struct device * z_impl_can_get_transceiver(const struct device * dev);


static inline const struct device * can_get_transceiver(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 344 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 344 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_transceiver(dev);
}
# 356 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_start(const struct device * dev);


static inline int can_start(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 367 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 367 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_start(dev);
}
# 379 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_stop(const struct device * dev);


static inline int can_stop(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 390 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 390 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stop(dev);
}
# 402 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_set_mode(const struct device * dev, can_mode_t mode);


static inline int can_set_mode(const struct device * dev, can_mode_t mode)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 414 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 414 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_set_mode(dev, mode);
}
# 426 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern can_mode_t z_impl_can_get_mode(const struct device * dev);


static inline can_mode_t can_get_mode(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 437 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 437 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_mode(dev);
}
# 449 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_set_bitrate(const struct device * dev, uint32_t bitrate);


static inline int can_set_bitrate(const struct device * dev, uint32_t bitrate)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 461 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 461 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_set_bitrate(dev, bitrate);
}
# 473 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_send(const struct device * dev, const struct can_frame * frame, k_timeout_t timeout, can_tx_callback_t callback, void * user_data);


static inline int can_send(const struct device * dev, const struct can_frame * frame, k_timeout_t timeout, can_tx_callback_t callback, void * user_data)
{
# 488 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 488 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 488 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_send(dev, frame, timeout, callback, user_data);
}
# 500 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_add_rx_filter_msgq(const struct device * dev, struct k_msgq * msgq, const struct can_filter * filter);


static inline int can_add_rx_filter_msgq(const struct device * dev, struct k_msgq * msgq, const struct can_filter * filter)
{
# 513 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 513 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 513 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_add_rx_filter_msgq(dev, msgq, filter);
}
# 525 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern void z_impl_can_remove_rx_filter(const struct device * dev, int filter_id);


static inline void can_remove_rx_filter(const struct device * dev, int filter_id)
{
# 538 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 538 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 538 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 z_impl_can_remove_rx_filter(dev, filter_id);
}
# 550 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_get_max_filters(const struct device * dev, 
# 550 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
                                                                _Bool 
# 550 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
                                                                     ide);


static inline int can_get_max_filters(const struct device * dev, 
# 553 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
                                                                _Bool 
# 553 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
                                                                     ide)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 562 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 562 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_max_filters(dev, ide);
}
# 574 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_get_state(const struct device * dev, enum can_state * state, struct can_bus_err_cnt * err_cnt);


static inline int can_get_state(const struct device * dev, enum can_state * state, struct can_bus_err_cnt * err_cnt)
{
# 587 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 587 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 587 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_get_state(dev, state, err_cnt);
}
# 599 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern int z_impl_can_recover(const struct device * dev, k_timeout_t timeout);


static inline int can_recover(const struct device * dev, k_timeout_t timeout)
{







 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 611 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 611 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_recover(dev, timeout);
}
# 623 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_bit_errors(const struct device * dev);


static inline uint32_t can_stats_get_bit_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 634 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 634 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_bit_errors(dev);
}
# 646 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_bit0_errors(const struct device * dev);


static inline uint32_t can_stats_get_bit0_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 657 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 657 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_bit0_errors(dev);
}
# 669 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_bit1_errors(const struct device * dev);


static inline uint32_t can_stats_get_bit1_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 680 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 680 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_bit1_errors(dev);
}
# 692 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_stuff_errors(const struct device * dev);


static inline uint32_t can_stats_get_stuff_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 703 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 703 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_stuff_errors(dev);
}
# 715 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_crc_errors(const struct device * dev);


static inline uint32_t can_stats_get_crc_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 726 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 726 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_crc_errors(dev);
}
# 738 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_form_errors(const struct device * dev);


static inline uint32_t can_stats_get_form_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 749 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 749 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_form_errors(dev);
}
# 761 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_ack_errors(const struct device * dev);


static inline uint32_t can_stats_get_ack_errors(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 772 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 772 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_ack_errors(dev);
}
# 784 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
extern uint32_t z_impl_can_stats_get_rx_overruns(const struct device * dev);


static inline uint32_t can_stats_get_rx_overruns(const struct device * dev)
{






 do { __asm__ __volatile__ ("" ::: "memory"); } while (
# 795 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h" 3 4
0
# 795 "/home/ttwards/zephyrproject/motor/build/zephyr/include/generated/zephyr/syscalls/can.h"
);
 return z_impl_can_stats_get_rx_overruns(dev);
}
# 1805 "/home/ttwards/zephyrproject/zephyr/include/zephyr/drivers/can.h" 2
# 18 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 2





typedef uint8_t canbus_id_t;




typedef uint16_t allmotor_id_t;



typedef uint16_t motor_id_t;

static struct k_sem dji_thread_sem;
struct k_work_q dji_work_queue;

struct motor_controller {
 const struct device *can_dev;
# 47 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
 int rx_ids[8];
 
# 48 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
_Bool 
# 48 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
     full[5];
 uint8_t mapping[5][4];
 uint8_t flags;
 uint8_t mask[5];
 struct device *motor_devs[8];

 struct k_work full_handle;

 struct k_sem tx_queue_sem;
};

struct dji_motor_data {
 struct motor_driver_data common;
 canbus_id_t canbus_id;
 struct motor_controller *ctrl_struct;


 
# 65 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
_Bool 
# 65 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
     online;
 uint8_t convert_num;
 int8_t current_mode_index;


 uint16_t RAWangle;
 uint16_t RAWprev_angle;
 int32_t RAWcurrent;
 int16_t RAWrpm;
 int8_t RAWtemp;
 float angle_add;

 uint32_t curr_time;
 uint32_t prev_time;
 int8_t missed_times;

 float angle_offset;

 float pid_angle_input;
 float pid_ref_input;

 struct k_spinlock data_input_lock;

 
# 88 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
_Bool 
# 88 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
     minorArc;


 float target_angle;
 float target_rpm;
 float target_torque;
 float target_current;
};

struct dji_motor_config {
 struct motor_driver_config common;

 float gear_ratio;
 
# 101 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
_Bool 
# 101 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
     is_gm6020;
 
# 102 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
_Bool 
# 102 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
     is_m3508;
 
# 103 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
_Bool 
# 103 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
     is_m2006;
};


extern struct motor_controller ctrl_structs[];


void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data);

void dji_speed_limit(const struct device *dev, float max_speed, float min_speed);
void dji_torque_limit(const struct device *dev, float max_torque, float min_torque);
int dji_set_speed(const struct device *dev, float speed_rpm);
int dji_set_angle(const struct device *dev, float angle);
int dji_set_torque(const struct device *dev, float torque);
float dji_set_zero(const struct device *dev);
float dji_get_angle(const struct device *dev);
float dji_get_speed(const struct device *dev);
float dji_get_torque(const struct device *dev);
int dji_init(const struct device *dev);
void dji_control(const struct device *dev, enum motor_cmd cmd);

void dji_tx_handler(struct k_work *work);
void dji_miss_handler(struct k_work *work);

void dji_miss_isr_handler(struct k_timer *dummy);

void dji_init_handler(struct k_work *work);

struct z_thread_stack_element __attribute__((section("." "noinit" "." "\"/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h\"" "." "0"))) 
# 131 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
__attribute__((__aligned__(
# 131 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
8
# 131 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
))) 
# 131 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
dm_work_queue_stack[((((unsigned long)((((((unsigned long)(2048) + ((unsigned long)(8) - 1)) / (unsigned long)(8)) * (unsigned long)(8)) + ((size_t)0))) + ((unsigned long)(8) - 1)) / (unsigned long)(8)) * (unsigned long)(8))];

struct k_work dji_miss_handle = { .handler = (dji_miss_handler), };
struct k_work dji_init_handle = { .handler = (dji_init_handler), };


# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
__attribute__((__aligned__(
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
__alignof(struct k_timer)
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
))) 
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
struct k_timer dji_miss_handle_timer __attribute__((section("." "_k_timer" "." "static" "." "dji_miss_handle_timer_"))) 
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
__attribute__((__used__)) 
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
= { .timeout = { .node = {}, .fn = z_timer_expiration_handler, .dticks = 0, }, .wait_q = { { {(&(&dji_miss_handle_timer.wait_q)->waitq)}, {(&(&dji_miss_handle_timer.wait_q)->waitq)} } }, .expiry_fn = dji_miss_isr_handler, .stop_fn = 
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h" 3 4
((void *)0)
# 136 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.h"
, .status = 0, .user_data = 0, };

static const struct motor_driver_api motor_api_funcs = {
 .motor_get_speed = dji_get_speed,
 .motor_get_torque = dji_get_torque,
 .motor_get_angle = dji_get_angle,
 .motor_set_speed = dji_set_speed,
 .motor_set_torque = dji_set_torque,
 .motor_set_angle = dji_set_angle,
 .motor_control = dji_control,
 .motor_limit_speed = dji_speed_limit,
 .motor_limit_torque = dji_torque_limit,
};

extern const struct device *can_devices[];
extern const struct device *motor_devices[];
# 7 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 2
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
const 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__aligned__(
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
__alignof(struct log_source_const_data)
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
))) 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
struct log_source_const_data log_const_motor_dji __attribute__((section("." "_log_const" "." "static" "." "log_const_motor_dji_"))) 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__used__)) 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
= { .name = "motor_dji", .level = (4) }; extern const struct log_source_const_data log_const_motor_dji; extern struct log_source_dynamic_data log_dynamic_motor_dji; static const struct log_source_const_data * __log_current_const_data 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__unused__)) 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
= 1 ? &log_const_motor_dji : 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
((void *)0)
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
; static struct log_source_dynamic_data * __log_current_dynamic_data 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__unused__)) 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
= (1 && 0) ? &log_dynamic_motor_dji : 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
((void *)0)
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
; static const uint32_t __log_level 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__unused__)) 
# 25 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
= 4;

const struct device *motor_devices[] = {(&__device_dts_ord_22),};

const struct device *can_devices[] = {
 (&__device_dts_ord_17) , (&__device_dts_ord_37)};







static int frames_id(int tx_id)
{
 if (tx_id == 0x200) {
  return 0;
 } else if (tx_id == 0x1FF) {
  return 1;
 } else if (tx_id == 0x1FE) {
  return 2;
 } else if (tx_id == 0x2FE) {
  return 3;
 } else if (tx_id == 0x2FF) {
  return 4;
 }
 return -1;
}

int get_can_id(const struct device *dev)
{
 const struct dji_motor_config *cfg = dev->config;
 for (int i = 0; i < 2; i++) {
  if (can_devices[i] == cfg->common.phy) {
   return i;
  }
 }
 return -1;
}

static int txframe_id(int frames_id)
{
 if (frames_id == 0) {
  return 0x200;
 } else if (frames_id == 1) {
  return 0x1FF;
 } else if (frames_id == 2) {
  return 0x1FE;
 } else if (frames_id == 3) {
  return 0x2FE;
 } else if (frames_id == 4) {
  return 0x2FF;
 }
 return -1;
}







static int16_t to16t(float value)
{

 if (value > 
# 90 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
            (0x7fff)
# 90 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                     ) {
  return 
# 91 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
        (0x7fff)
# 91 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                 ;
 } else if (value < 
# 92 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                   (-0x7fff - 1)
# 92 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                            ) {
  return 
# 93 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
        (-0x7fff - 1)
# 93 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                 ;
 } else {
  return (int16_t)value;
 }
}

struct motor_controller ctrl_structs[2] = {
 { .can_dev = (&__device_dts_ord_17), .flags = 0, .full = {{
# 100 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
0
# 100 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
}}, .mask = 0, } , { .can_dev = (&__device_dts_ord_37), .flags = 0, .full = {{
# 100 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
0
# 100 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
}}, .mask = 0, }};

static inline motor_id_t canbus_id(const struct device *dev)
{
 for (int i = 0; i < sizeof(motor_devices) / sizeof(motor_devices[0]); i++) {
  if (motor_devices[i] == dev) {
   return i;
  }
 }
 return 0;
}

static inline motor_id_t motor_id(const struct device *dev)
{
 const struct dji_motor_config *cfg = dev->config;
 return cfg->common.id - 1;
}

void dji_speed_limit(const struct device *dev, float max_speed, float min_speed)
{
 struct dji_motor_data *data = dev->data;
 data->common.speed_limit[0] = min_speed;
 data->common.speed_limit[1] = max_speed;
}

void dji_torque_limit(const struct device *dev, float max_torque, float min_torque)
{
 struct dji_motor_data *data = dev->data;
 data->common.torque_limit[0] = min_torque;
 data->common.torque_limit[1] = max_torque;
}

int dji_set_speed(const struct device *dev, float speed_rpm)
{
 struct dji_motor_data *data = dev->data;
 const struct dji_motor_config *cfg = dev->config;

 if (speed_rpm > data->common.speed_limit[1]) {
  speed_rpm = data->common.speed_limit[1];
 } else if (speed_rpm < data->common.speed_limit[0]) {
  speed_rpm = data->common.speed_limit[0];
 }

 data->target_rpm = speed_rpm;
 for (int i = 0; i < sizeof(cfg->common.pid_datas) / sizeof(cfg->common.pid_datas[0]); i++) {
  if (cfg->common.pid_datas[i]->pid_dev == 
# 145 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                          ((void *)0)
# 145 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                              ) {
   break;
  }
  if (strcmp(cfg->common.capabilities[i], "speed") == 0) {
   pid_calc(cfg->common.pid_datas[i]);
   data->current_mode_index = i;
  }
 }

 return 0;
}

int dji_set_angle(const struct device *dev, float angle)
{
 struct dji_motor_data *data = dev->data;
 const struct dji_motor_config *cfg = dev->config;

 if (angle > 360) {
  angle = fmodf(angle, 360);
 } else if (angle < 0) {
  while (angle < 0) {
   angle += 360;
  }
 }

 data->target_angle = angle;

 for (int i = 0; i < (sizeof(cfg->common.pid_datas) / sizeof(cfg->common.pid_datas[0])); i++) {
  if (cfg->common.pid_datas[i]->pid_dev == 
# 173 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                          ((void *)0)
# 173 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                              ) {
   break;
  }
  if (strcmp(cfg->common.capabilities[i], "angle") == 0) {
   pid_calc(cfg->common.pid_datas[i]);
   data->current_mode_index = i;
  }
 }

 return 0;
}

int dji_set_torque(const struct device *dev, float torque)
{
 struct dji_motor_data *data = dev->data;
 const struct dji_motor_config *cfg = dev->config;

 if (torque > data->common.torque_limit[1]) {
  torque = data->common.torque_limit[1];
 } else if (torque < data->common.torque_limit[0]) {
  torque = data->common.torque_limit[0];
 }

 data->target_torque = torque;
 for (int i = 0; i < (sizeof(cfg->common.pid_datas) / sizeof(cfg->common.pid_datas[0])); i++) {
  if (cfg->common.pid_datas[i]->pid_dev == 
# 198 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                          ((void *)0)
# 198 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                              ) {
   data->current_mode_index = i + 1;
   break;
  }
  if (strcmp(cfg->common.capabilities[i], "torque") == 0) {
   pid_calc(cfg->common.pid_datas[i]);
   data->current_mode_index = i;
  }
 }

 return 0;
}

void dji_control(const struct device *dev, enum motor_cmd cmd)
{
 struct dji_motor_data *data = dev->data;
 const struct dji_motor_config *cfg = dev->config;

 struct can_frame frame;
 frame.id = cfg->common.tx_id;

 switch (cmd) {
 case ENABLE_MOTOR:
  data->online = 
# 221 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                1
# 221 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                    ;
  break;
 case DISABLE_MOTOR:
  data->online = 
# 224 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                0
# 224 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                     ;
  break;
 case SET_ZERO_OFFSET:
  data->angle_add = 0;
  data->common.angle = 0;
  break;
 case CLEAR_PID:
  break;
 case CLEAR_ERROR:
  data->missed_times = 0;
  data->online = 
# 234 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                1
# 234 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                    ;
  break;
 }
}

float dji_get_angle(const struct device *dev)
{
 struct dji_motor_data *data = dev->data;
 return fmodf(data->common.angle, 360.0f);
}

float dji_get_speed(const struct device *dev)
{
 struct dji_motor_data *data = dev->data;
 return data->common.rpm;
}

float dji_get_torque(const struct device *dev)
{
 struct dji_motor_data *data = dev->data;
 return data->common.torque;
}

int dji_init(const struct device *dev)
{





 if (dev) {
  const struct dji_motor_config *cfg = dev->config;
  struct dji_motor_data *data = dev->data;
  uint8_t frame_id = frames_id(cfg->common.tx_id);
  uint8_t id = motor_id(dev);
  data->ctrl_struct->mask[frame_id] |= id >= 4 ? 0xF0 : 0x0F;
  data->ctrl_struct->mask[frame_id] ^= 1 << id;
  data->ctrl_struct->rx_ids[id] = cfg->common.rx_id;

  data->ctrl_struct->full_handle.handler = dji_tx_handler;

  data->online = 
# 275 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                1
# 275 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                    ;
  for (int i = 0;
       i < sizeof(cfg->common.pid_datas) / sizeof(cfg->common.pid_datas[0]); i++) {
   if (cfg->common.pid_datas[i]->pid_dev == 
# 278 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                           ((void *)0)
# 278 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                               ) {
    if (i > 0) {
     pid_reg_output(cfg->common.pid_datas[i - 1],
             &data->target_torque);
    }
    break;
   }
   if (strcmp(cfg->common.capabilities[i], "speed") == 0) {
    pid_reg_input(cfg->common.pid_datas[i], &data->common.rpm,
           &data->target_rpm);
    if (i > 0) {
     pid_reg_output(cfg->common.pid_datas[i - 1],
             &data->target_rpm);
    }
   } else if (strcmp(cfg->common.capabilities[i], "angle") == 0) {
    pid_reg_input(cfg->common.pid_datas[i], &data->pid_angle_input,
           &data->pid_ref_input);
   } else if (strcmp(cfg->common.capabilities[i], "torque") == 0) {
    pid_reg_input(cfg->common.pid_datas[i], &data->common.torque,
           &data->target_torque);
    if (i > 0) {
     pid_reg_output(cfg->common.pid_datas[i - 1],
             &data->target_torque);
    }
   } else {
    do { if (!((1 && (((1) <= 0) || ((0 == 
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   0
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Unsupported motor mode" "\n", z_log_minimal_level_to_char(1)); } while (
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   0
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   ); break; } int _mode; 
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   _Bool 
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   string_ok; string_ok = 
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   1
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   ((void *)0)
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Unsupported motor mode"); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   0
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   ); (void)_mode; if (
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   0
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   ) { z_log_printf_arg_checker("Unsupported motor mode"); } } while (
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
   0
# 303 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
   );
    return -1;
   }
   pid_reg_time(cfg->common.pid_datas[i], &(data->curr_time),
         &(data->prev_time));
  }
  data->current_mode_index = 0;
  data->ctrl_struct->motor_devs[id] = (struct device *)dev;
  data->prev_time = 0;
  data->ctrl_struct->flags = 0;
  data->ctrl_struct->mapping[frame_id][id % 4] = id;
  if (cfg->is_gm6020) {
   data->convert_num = 1;
  } else if (cfg->is_m3508) {
   data->convert_num = 0;
  } else if (cfg->is_m2006) {
   data->convert_num = 2;
  } else {
   do { if (!((1 && (((1) <= 0) || ((0 == 
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Unsupported motor type" "\n", z_log_minimal_level_to_char(1)); } while (
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ); break; } int _mode; 
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  _Bool 
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  string_ok; string_ok = 
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  1
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  ((void *)0)
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Unsupported motor type"); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ); (void)_mode; if (
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ) { z_log_printf_arg_checker("Unsupported motor type"); } } while (
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 321 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  );
  }

  if (!device_is_ready(cfg->common.phy)) {
   return -1;
  }
  if (dji_miss_handle_timer.expiry_fn == 
# 327 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                        ((void *)0)
# 327 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                            ) {
   dji_miss_handle_timer.expiry_fn = dji_miss_isr_handler;
   k_timer_init(&dji_miss_handle_timer, dji_miss_isr_handler, 
# 329 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                             ((void *)0)
# 329 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                                 );
   k_timer_start(&dji_miss_handle_timer, ((k_timeout_t) {0}), ((k_timeout_t) { .ticks = ((k_ticks_t)((
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   1
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? ( ((10000) == (1000)) ? (uint64_t) ((((2) > (0)) ? (2) : (0))) : ((1000) > (10000) && (1000) % (10000) == 0U) ? (((uint64_t) ((((2) > (0)) ? (2) : (0))) + ((
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   0
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? ((1000) / (10000)) / 2 : (
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   1
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? ((1000) / (10000)) - 1 : 0)) / ((1000)/(10000) ? (1000)/(10000) : 01u)) : ((10000) > (1000) && (10000) % (1000) == 0U) ? (uint64_t) ((((2) > (0)) ? (2) : (0)))*((10000) / (1000)) : ((((((365 * 24ULL * 3600ULL * 1000) + (
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   (0xffffffffUL)
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) - 1) / (
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   (0xffffffffUL)
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   )) * 10000) <= 
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   (0xffffffffUL)
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (((uint64_t) ((((2) > (0)) ? (2) : (0)))*(10000) + ((
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   0
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (1000) / 2 : (
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   1
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (1000) - 1 : 0)) / (1000)) : (((uint64_t) ((((2) > (0)) ? (2) : (0))) / (1000))*(10000) + (((uint64_t) ((((2) > (0)) ? (2) : (0))) % (1000))*(10000) + ((
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   0
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (1000) / 2 : (
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   1
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (1000) - 1 : 0)) / (1000))) ) : (((uint64_t) ((((2) > (0)) ? (2) : (0))) / (1000))*(10000) + (((uint64_t) ((((2) > (0)) ? (2) : (0))) % (1000))*(10000) + ((
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   0
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (1000) / 2 : (
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                   1
# 330 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                   ) ? (1000) - 1 : 0)) / (1000)) )) }));
  }
 }
 return 0;
}

void can_rx_callback(const struct device *can_dev, struct can_frame *frame, void *user_data)
{
 uint32_t curr_time = k_cycle_get_32();
 struct motor_controller *ctrl_struct = (struct motor_controller *)user_data;
 struct can_frame rx_frame = *frame;

 uint8_t id = (rx_frame.id & 0xF) - 1;

 if (ctrl_struct->rx_ids[id] != rx_frame.id && id >= 4) {
  id -= 4;
 }

 if (ctrl_struct->rx_ids[id] != rx_frame.id) {
  do { if (!((1 && (((1) <= 0) || ((0 == 
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Unknown motor ID: %d, database: %d, received: %d" "\n", z_log_minimal_level_to_char(1), id, ctrl_struct->rx_ids[id], rx_frame.id); } while (
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ); break; } int _mode; 
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 _Bool 
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 string_ok; string_ok = 
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 1
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 ((void *)0)
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Unknown motor ID: %d, database: %d, received: %d", id, ctrl_struct->rx_ids[id], rx_frame.id); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ); (void)_mode; if (
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ) { z_log_printf_arg_checker("Unknown motor ID: %d, database: %d, received: %d", id, ctrl_struct->rx_ids[id], rx_frame.id); } } while (
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 349 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 )
                                        ;
  return;
 }
 int8_t bus_id = canbus_id(can_dev);
 if (!ctrl_struct) {
  return;
 }
 struct dji_motor_data *motor_data = ctrl_struct->motor_devs[id]->data;
 uint32_t prev_time = motor_data->curr_time;

 k_spinlock_key_t key;
 if (k_spin_trylock(&motor_data->data_input_lock, &key) != 0) {
  return;
 }

 if (!motor_data) {
  return;
 }
 if (motor_data->missed_times > 3) {
  do { if (!((1 && (((1) <= 0) || ((0 == 
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Motor %d is responding again, resuming..." "\n", z_log_minimal_level_to_char(1), id); } while (
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ); break; } int _mode; 
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 _Bool 
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 string_ok; string_ok = 
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 1
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 ((void *)0)
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Motor %d is responding again, resuming...", id); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ); (void)_mode; if (
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 ) { z_log_printf_arg_checker("Motor %d is responding again, resuming...", id); } } while (
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 0
# 369 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 );
  motor_data->missed_times = 0;
  motor_data->online = 
# 371 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                      1
# 371 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                          ;
  const struct dji_motor_config *motor_cfg =
   (const struct dji_motor_config *)ctrl_struct->motor_devs[id]->config;
  ctrl_struct[bus_id].mask[frames_id(motor_cfg->common.tx_id)] ^= 1 << id;
 } else if (motor_data->missed_times > 0) {
  motor_data->missed_times--;
 }


 motor_data->RAWprev_angle = motor_data->RAWangle;
 motor_data->RAWangle = ((rx_frame.data[0] << 8) + rx_frame.data[1]);
 motor_data->RAWrpm = ((rx_frame.data[2] << 8) + rx_frame.data[3]);
 motor_data->RAWcurrent = ((rx_frame.data[4] << 8) + rx_frame.data[5]);
 motor_data->RAWtemp = rx_frame.data[6];
 ctrl_struct[bus_id].flags |= 1 << id;
 motor_data->curr_time = curr_time;
 motor_data->prev_time = prev_time;
 
# 388 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
_Bool 
# 388 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
     full = 
# 388 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
            0
# 388 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                 ;
 for (int i = 0; i < 5; i++) {
  uint8_t combined = ctrl_struct[bus_id].mask[i] | ctrl_struct[bus_id].flags;
  if ((((combined | 0xF0)) == 0xF0) || ((combined | 0x0F) == 0x0F)) {

   ctrl_struct[bus_id].full[i] = 
# 393 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                1
# 393 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                    ;
   full = 
# 394 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
         1
# 394 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
             ;
  }
 }

 if (full) {
  k_work_submit_to_queue(&dji_work_queue, &ctrl_struct[bus_id].full_handle);
 }

 k_spin_unlock(&motor_data->data_input_lock, key);
 return;
}

static const struct can_filter filter20x = {.id = 0x200, .mask = 0x3F0, .flags = 0};

static void can_tx_callback(const struct device *can_dev, int error, void *user_data)
{
 struct k_sem *queue_sem = user_data;
 k_sem_give(queue_sem);
}

static void proceed_delta_degree(const struct device *dev)
{
 struct dji_motor_data *data_temp = dev->data;
 const struct dji_motor_config *config_temp = dev->config;
 int delta = data_temp->RAWangle - data_temp->RAWprev_angle;
 if (data_temp->RAWangle < 2048 && data_temp->RAWprev_angle > 6144) {
  delta += 8192;
  data_temp->common.round_cnt++;
 } else if (data_temp->RAWangle > 6144 && data_temp->RAWprev_angle < 2048) {
  delta -= 8192;
  data_temp->common.round_cnt--;
 }

 if (fabsf(config_temp->gear_ratio - 1) > 0.001f) {

  data_temp->angle_add +=
   (float)(delta)*convert[data_temp->convert_num][4] /
   (config_temp->gear_ratio);

  data_temp->common.angle = fmodf(data_temp->angle_add, 360.0f);
  while (data_temp->common.angle < 0) {
   data_temp->common.angle += 360.0f;
  }
 } else {
  data_temp->common.angle = (float)(data_temp->RAWangle) *
       convert[data_temp->convert_num][4];
 }

 float delta_angle = data_temp->common.angle - data_temp->target_angle;

 if (delta_angle > 0) {
  if (delta_angle < 180) {
   data_temp->pid_angle_input = delta_angle;
  } else {
   data_temp->pid_angle_input = delta_angle - 360.0f;
  }
 } else if (delta_angle < 0) {
  if (delta_angle > -180) {
   data_temp->pid_angle_input = delta_angle;
  } else {
   data_temp->pid_angle_input = delta_angle + 360.0f;
  }
 }
}

static void can_pack_add(uint8_t *data, struct device *motor_dev, uint8_t num)
{
 struct dji_motor_data *data_temp = motor_dev->data;

 int16_t value = to16t(data_temp->target_current);

 data[num * 2] = ((value) >> 8);
 data[num * 2 + 1] = ((value) & 0xFF);
}

static void dji_timeout_handle(const struct device *dev, uint32_t curr_time,
          struct motor_controller *ctrl_struct)
{
 struct dji_motor_data *motor_data = (struct dji_motor_data *)dev->data;
 const struct dji_motor_config *motor_cfg = (const struct dji_motor_config *)dev->config;

 if (motor_data->online == 
# 475 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                          0
# 475 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                               ) {
  return;
 }
 uint32_t prev_time = motor_data->curr_time;
 if ((((!0)) ? ( ((1000000) == (168000000)) ? (uint32_t) (curr_time - prev_time) : ((168000000) > (1000000) && (168000000) % (1000000) == 0U) ? ((uint64_t) (curr_time - prev_time) <= 0xffffffffU - ((
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    1
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? ((168000000) / (1000000)) / 2 : (
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? ((168000000) / (1000000)) - 1 : 0) ? ((uint32_t)((curr_time - prev_time) + ((
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    1
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? ((168000000) / (1000000)) / 2 : (
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? ((168000000) / (1000000)) - 1 : 0)) / ((168000000)/(1000000) ? (168000000)/(1000000) : 01u)) : (uint32_t) (((uint64_t) (curr_time - prev_time) + ((
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    1
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? ((168000000) / (1000000)) / 2 : (
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? ((168000000) / (1000000)) - 1 : 0)) / ((168000000)/(1000000) ? (168000000)/(1000000) : 01u)) ) : ((1000000) > (168000000) && (1000000) % (168000000) == 0U) ? (uint32_t) (curr_time - prev_time)*((1000000) / (168000000)) : ((uint32_t) (((uint64_t) (curr_time - prev_time)*(1000000) + ((
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    1
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? (168000000) / 2 : (
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? (168000000) - 1 : 0)) / (168000000))) ) : ((uint32_t) (((uint64_t) (curr_time - prev_time)*(1000000) + ((
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    1
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? (168000000) / 2 : (
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 479 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) ? (168000000) - 1 : 0)) / (168000000))) ) > 2000) {
  motor_data->missed_times++;
  if (motor_data->missed_times > 3) {
   do { if (!((1 && (((1) <= 0) || ((0 == 
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Motor %d is not responding" "\n", z_log_minimal_level_to_char(1), motor_cfg->common.id); } while (
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ); break; } int _mode; 
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  _Bool 
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  string_ok; string_ok = 
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  1
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  ((void *)0)
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Motor %d is not responding", motor_cfg->common.id); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ); (void)_mode; if (
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ) { z_log_printf_arg_checker("Motor %d is not responding", motor_cfg->common.id); } } while (
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 482 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  );
   ctrl_struct[motor_data->canbus_id]
    .mask[frames_id(motor_cfg->common.tx_id)] ^=
    1 << (motor_cfg->common.id - 1);
   motor_data->online = 
# 486 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                       0
# 486 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                            ;
  }
 }
}

static void motor_calc(const struct device *dev)
{
 const struct device *dev_temp = dev;
 struct dji_motor_data *data_temp = dev_temp->data;
 k_spinlock_key_t key;
 if (k_spin_trylock(&data_temp->data_input_lock, &key) != 0) {
  return;
 }
 const struct dji_motor_config *config_temp = dev_temp->config;


 proceed_delta_degree(dev_temp);

 data_temp->common.rpm = data_temp->RAWrpm * convert[data_temp->convert_num][2] /
    config_temp->gear_ratio;
 data_temp->common.torque = data_temp->RAWcurrent *
       convert[data_temp->convert_num][0] *
       config_temp->gear_ratio;



 
# 512 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
_Bool 
# 512 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
     torque_proceeded = 
# 512 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                        0
# 512 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                             ;
 for (int i = data_temp->current_mode_index;
      i < (sizeof(config_temp->common.capabilities) / sizeof(config_temp->common.capabilities[0])); i++) {
  if (config_temp->common.pid_datas[i]->pid_dev == 
# 515 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                                  ((void *)0)
# 515 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                      ) {
   if (torque_proceeded) {
    break;
   }
   if (data_temp->target_torque > data_temp->common.torque_limit[1]) {
    data_temp->target_torque = data_temp->common.torque_limit[1];
   } else if (data_temp->target_torque < data_temp->common.torque_limit[0]) {
    data_temp->target_torque = data_temp->common.torque_limit[0];
   }
   data_temp->target_current = data_temp->target_torque /
          config_temp->gear_ratio *
          convert[data_temp->convert_num][1];
   break;
  }

  pid_calc(config_temp->common.pid_datas[i]);

  if (strcmp(config_temp->common.capabilities[i], "angle") == 0) {
   if (data_temp->target_rpm > data_temp->common.speed_limit[1]) {
    data_temp->target_rpm = data_temp->common.speed_limit[1];
   } else if (data_temp->target_rpm < data_temp->common.speed_limit[0]) {
    data_temp->target_rpm = data_temp->common.speed_limit[0];
   }
  }

  if (strcmp(config_temp->common.capabilities[i], "torque") == 0) {
   torque_proceeded = 
# 541 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                     1
# 541 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                         ;
  } else if (strcmp(config_temp->common.capabilities[i], "mit") == 0) {
   break;
  }
 }
 k_spin_unlock(&data_temp->data_input_lock, key);
}

struct can_frame txframe;

static struct k_sem dji_thread_sem;

void dji_miss_isr_handler(struct k_timer *dummy)
{
 (void)(dummy);
 k_work_submit_to_queue(&dji_work_queue, &dji_miss_handle);
}

void dji_miss_handler(struct k_work *work)
{
 (void)(work);
 int curr_time = k_cycle_get_32();
 for (int i = 0; i < 2; i++) {
  for (int j = 0; j < 8; j++) {
   if (ctrl_structs[i].motor_devs[j]) {
    dji_timeout_handle(ctrl_structs[i].motor_devs[i], curr_time,
         &ctrl_structs[i]);
   }
  }
 }
}

void dji_init_handler(struct k_work *work)
{
 (void)(work);
 struct device *can_dev = 
# 576 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                         ((void *)0)
# 576 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                             ;
 for (int i = 0; i < 2; i++) {
  k_sem_init(&ctrl_structs[i].tx_queue_sem, 3, 3);

  can_dev = (struct device *)ctrl_structs[i].can_dev;
  can_start(can_dev);

  int err = can_add_rx_filter(can_dev, can_rx_callback, &ctrl_structs[i], &filter20x);

  if (err < 0) {
   do { if (!((1 && (((1) <= 0) || ((0 == 
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Error adding CAN filter (err %d)" "\n", z_log_minimal_level_to_char(1), err); } while (
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ); break; } int _mode; 
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  _Bool 
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  string_ok; string_ok = 
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  1
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  ((void *)0)
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Error adding CAN filter (err %d)", err); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ); (void)_mode; if (
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  ) { z_log_printf_arg_checker("Error adding CAN filter (err %d)", err); } } while (
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  0
# 586 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
  );
  }



 }
 dji_miss_handle_timer.expiry_fn = dji_miss_isr_handler;
}

void dji_tx_handler(struct k_work *work)
{
 struct motor_controller *ctrl_struct =
  ({ _Static_assert((__builtin_types_compatible_p(__typeof__(*(work)), __typeof__(((struct motor_controller *)0)->full_handle)) || __builtin_types_compatible_p(__typeof__(*(work)), __typeof__(void))), "" "pointer type mismatch in CONTAINER_OF"); ((struct motor_controller *)(((char *)(work)) - 
# 598 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 __builtin_offsetof (
# 598 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 struct motor_controller
# 598 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 , 
# 598 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 full_handle
# 598 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
 )
# 598 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
 )); });

 for (int i = 0; i < 5; i++) {
  if (ctrl_struct->full[i]) {
   uint8_t id_temp = ctrl_struct->mapping[0][0];
   uint8_t data[8] = {0};
   
# 604 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
  _Bool 
# 604 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
       packed = 
# 604 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                0
# 604 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                     ;
   for (int j = 0; j < 4; j++) {
    id_temp = ctrl_struct->mapping[i][j];
    if (id_temp < 0) {
     continue;
    }
    const struct device *dev = ctrl_struct->motor_devs[id_temp];
    struct dji_motor_data *data_temp = dev->data;

    if (id_temp < 8 &&
        ((ctrl_struct->flags & (1 << id_temp)) || !data_temp->online)) {
     motor_calc(ctrl_struct->motor_devs[id_temp]);
     can_pack_add(data, ctrl_struct->motor_devs[id_temp], j);
     ctrl_struct->flags ^= 1 << id_temp;
     packed = 
# 618 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
             1
# 618 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                 ;
    }
   }
   if (packed) {
    txframe.id = txframe_id(i);
    txframe.dlc = 8;
    txframe.flags = 0;
    memcpy(txframe.data, data, sizeof(data));
    const struct device *can_dev = ctrl_struct->can_dev;
    struct k_sem tx_queue_sem = ctrl_struct->tx_queue_sem;
    int err = k_sem_take(&tx_queue_sem, ((k_timeout_t) {0}));
    if (err == 0) {
     err = can_send(can_dev, &txframe, ((k_timeout_t) {0}),
             can_tx_callback, &tx_queue_sem);
    }
    if (err != 0 && err != -
# 633 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                           11 
# 633 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                  && err != -
# 633 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
                                             16
# 633 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
                                                  ) {
     do { if (!((1 && (((1) <= 0) || ((0 == 
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) && ((1) <= __log_level) && ((1) <= 4) ) )) && (0 || !0 || (1 <= ((const struct log_source_const_data *)__log_current_const_data)->level)) && (!0 || k_is_user_context() || ((1) <= ((*(&(((struct log_source_dynamic_data *)__log_current_const_data)->filters)) >> (3U * (0))) & ((1UL << (3U)) - 1U)))))) { break; } if (0) { do { z_log_minimal_printk("%c: " "Error sending CAN frame (err %d)" "\n", z_log_minimal_level_to_char(1), err); } while (
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ); break; } int _mode; 
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    _Bool 
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    string_ok; string_ok = 
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    1
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ; if (!string_ok) { ; break; } do { z_log_msg_runtime_create((0), (void *)(__log_current_const_data), (1), (uint8_t *)(
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    ((void *)0)
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ), (0), ((0 << 3) | (0 ? ((1UL << (1)) | (1UL << (0))) : 0)) | (0 ? (1UL << (6)) : 0), "Error sending CAN frame (err %d)", err); (_mode) = Z_LOG_MSG_MODE_RUNTIME; } while (
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ); (void)_mode; if (
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    ) { z_log_printf_arg_checker("Error sending CAN frame (err %d)", err); } } while (
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
    0
# 634 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
    );
    }
   }
  }
 }
}

struct pid_data PID_INS_NAME(DT_NODE_FULL_NAME_UNQUOTED, motor0) = { .pid_dev = (&__device_dts_ord_20), }; struct pid_data PID_INS_NAME(DT_NODE_FULL_NAME_UNQUOTED, motor0) = { .pid_dev = (&__device_dts_ord_21), }; static const struct dji_motor_config dji_motor_cfg_0 = { .common = { .phy = (const struct device *)(&__device_dts_ord_17), .id = 1, .tx_id = 512, .rx_id = 513, .capabilities = {"angle", "speed"}, .pid_datas = { &PID_INS_NAME(DT_NODE_FULL_NAME_UNQUOTED, motor0) , &PID_INS_NAME(DT_NODE_FULL_NAME_UNQUOTED, motor0)}, }, .gear_ratio = (0 ? 1 : 4) * (float)19.20, .is_gm6020 = 0, .is_m3508 = 1, .is_m2006 = 0, }; static struct dji_motor_data dji_motor_data_0 = { .common = { .angle = 0, .rpm = 0, .torque = 0, .temperature = 0, .round_cnt = 0, .speed_limit = {-99999, 99999}, .torque_limit = {-99999, 99999}, .mode = MIT, }, .canbus_id = 0, .ctrl_struct = &ctrl_structs[0], .pid_angle_input = 0, .pid_ref_input = 0, }; static 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__aligned__(
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
__alignof(struct device_state)
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
))) 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
struct device_state __devstate_dts_ord_22 __attribute__((__section__(".z_devstate"))); _Static_assert((sizeof("\"motor0\"") <= 48U), "" "\"motor0\"" " too long"); const 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__aligned__(
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
__alignof(struct device)
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
))) 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
struct device __device_dts_ord_22 __attribute__((section("." "_device" "." "static" "." "3_90_"))) 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__used__)) 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
= { .name = "motor0", .config = (&dji_motor_cfg_0), .api = (&motor_api_funcs), .state = (&__devstate_dts_ord_22), .data = (&dji_motor_data_0), }; static const 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__aligned__(
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
__alignof(struct init_entry)
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
))) 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
struct init_entry 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c" 3 4
__attribute__((__used__)) 
# 641 "/home/ttwards/zephyrproject/motor/drivers/motor/dji/motor_dji.c"
__attribute__((__section__( ".z_init_" "POST_KERNEL" "90""_" "00022""_"))) __init___device_dts_ord_22 = { .init_fn = {.dev = (dji_init)}, .dev = &__device_dts_ord_22, }; ;
