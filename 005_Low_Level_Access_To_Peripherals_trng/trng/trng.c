/*
 * trng.c
 *
 *  Created on: Nov 17, 2025
 *      Author: mc
 */
#include "main.h"
#include "trng.h"
//
uint32_t *trng_cr = (uint32_t *) TRNG_CR_ADDR;
uint32_t *trng_sr = (uint32_t *) TRNG_SR_ADDR;
uint32_t *trng_dr = (uint32_t *) TRNG_DR_ADDR;

uint32_t last_rng = 0;

uint32_t trng_get_random_number(void)
{
    // Tunggu sampai data ready
    while (!(*trng_sr & TRNG_SR_DRDY)) {
        // Jika ada error, clear dan lanjut (RNG akan restart otomatis)
        if (*trng_sr & (TRNG_SR_CEIS | TRNG_SR_SEIS)) {
            *trng_sr = (TRNG_SR_CEIS | TRNG_SR_SEIS);  // clear error flags
        }
    }

    // Baca data (otomatis clear DRDY)
    uint32_t rnd = *trng_dr;

    // Optional: jika mau hindari nilai berulang (jarang terjadi tapi boleh)
    static uint32_t last = 0;
    if (rnd == last) {
        // Baca sekali lagi (pasti beda karena RNG sudah generate baru)
        rnd = *trng_dr;
    }
    last = rnd;

    return rnd;
}

void trng_init() {
    TRNG_DBG("TRNG Init\n");

    __HAL_RCC_RNG_CLK_ENABLE();

    TRNG_DBG("CR = 0x%08lx SR = 0x%08lx DR = 0x%08lx\n", (uint32_t)trng_cr, (uint32_t)trng_sr, (uint32_t)trng_dr);

    TRNG_DBG("TRNG Initial Values\n");
    TRNG_DBG("CR = 0x%08lx\n", *trng_cr);
    TRNG_DBG("SR = 0x%08lx\n", *trng_sr);

    // Fire up the RNG
    *trng_cr |= TRNG_CR_RNGEN;

    trng_get_random_number();
}
