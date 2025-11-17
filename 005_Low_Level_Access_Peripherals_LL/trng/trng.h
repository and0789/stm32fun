/*
* trng.h
 *
 *  True Random Number Generator driver (STM32F4xx)
 *  Raw register access - clean, zero warning, production ready
 */

#ifndef TRNG_H_
#define TRNG_H_

#include "stm32f4xx_hal.h"

/* Debug print (uncomment kalau lagi debugging) */
// #define TRNG_DEBUG_ENABLE
#ifdef TRNG_DEBUG_ENABLE
    #define TRNG_DBG(...)     printf(__VA_ARGS__)
#else
    #define TRNG_DBG(...)
#endif

/* RNG peripheral base - langsung pakai CMSIS instance (paling aman) */
#define RNG                 ((RNG_TypeDef *) RNG_BASE)

/* Bit masks - langsung pakai yang sudah ada di CMSIS (stm32f407xx.h)
   JANGAN REDEFINE lagi → itu penyebab warning/error */
 /*
    RNG_CR_RNGEN
    RNG_SR_DRDY
    RNG_SR_CEIS
    RNG_SR_SEIS
    dll sudah ada dan benar
 */

void     trng_init(void);
uint32_t trng_get_random_number(void);

#endif /* TRNG_H_ */