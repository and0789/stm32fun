/*
 * trng.h
 *
 *  Created on: Nov 17, 2025
 *      Author: mc
 */

#ifndef TRNG_H_
#define TRNG_H_

#ifdef DEBUG
#include <stdio.h>
#define TRNG_DBG(...) printf(__VA_ARGS__)
#endif



// Register offset from reference manual
#define TRNG_ADDR       0x50060800UL

#define TRNG_CR_ADDR    (TRNG_ADDR + 0x00)
#define TRNG_SR_ADDR    (TRNG_ADDR + 0x04)   // BUKAN +1 !!
#define TRNG_DR_ADDR    (TRNG_ADDR + 0x08)   // BUKAN +2 !!

// Control register bits - from reference manual
#define TRNG_CR_IE (1 << 3)
#define TRNG_CR_RNGEN (1 << 2)

// status register bits - from reference manual
#define TRNG_SR_DRDY (1 << 0)
#define TRNG_SR_SEIS (1 << 6)
#define TRNG_SR_CEIS (1 << 5)
#define TRNG_SR_SECS (1 << 2)
#define TRNG_SR_CECS (1 << 1)
#include <stdint.h>

void trng_init(void);
uint32_t trng_get_random_number(void);

#endif /* TRNG_H_ */
