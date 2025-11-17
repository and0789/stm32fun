/*
* trng.c
 *
 *  True Random Number Generator driver
 *  Raw register access - zero overhead, fully robust
 */

#include "trng.h"

void trng_init(void)
{
    TRNG_DBG("[TRNG] Initializing...\n");

    /* Enable RNG clock (AHB2) */
    __HAL_RCC_RNG_CLK_ENABLE();

    /* Enable RNG */
    RNG->CR |= RNG_CR_RNGEN;

    TRNG_DBG("[TRNG] Enabled, CR = 0x%08lX\n", RNG->CR);
}

/**
 * @brief  Get 32-bit true random number
 *         Robust with timeout + error clearing (sesuai RM0090 section 23.3.6)
 * @retval Random number atau 0xDEADDEAD kalau timeout (jarang banget terjadi)
 */
uint32_t trng_get_random_number(void)
{
    uint32_t timeout = 200000U;     // lebih dari cukup bahkan di board paling lambat sekalipun

    while (!(RNG->SR & RNG_SR_DRDY))
    {
        /* Clear error flags (cara benar: write 0 ke bit CEIS & SEIS) */
        if (RNG->SR & (RNG_SR_CEIS | RNG_SR_SEIS))
        {
            RNG->SR &= ~(RNG_SR_CEIS | RNG_SR_SEIS);
            TRNG_DBG("[TRNG] Error flags cleared\n");
        }

        if (--timeout == 0)
        {
            TRNG_DBG("[TRNG] Timeout waiting for random data!\n");
            return 0xDEADDEADUL;   // nilai error yang jelas
        }
    }

    return RNG->DR;
}