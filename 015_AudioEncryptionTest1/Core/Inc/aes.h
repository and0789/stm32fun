//
// Created by mc on 02/12/25.
//

#ifndef AUDIOENCRYPTIONTEST1_AES_H
#define AUDIOENCRYPTIONTEST1_AES_H

#include <stdint.h>
#include <stddef.h>

// Konfigurasi AES-128
#define AES_BLOCKLEN 16 // Block size selalu 16 byte
#define AES_KEYLEN 16   // Key size 16 byte (128 bit)

typedef struct {
    uint8_t RoundKey[176];
    uint8_t Iv[AES_BLOCKLEN];
} AES_ctx;

// Fungsi Inisialisasi
void AES_init_ctx_iv(AES_ctx* ctx, const uint8_t* key, const uint8_t* iv);

// Fungsi Utama: Enkripsi & Dekripsi
void AES_CTR_xcrypt_buffer(AES_ctx* ctx, uint8_t* buf, uint32_t length);

#endif //AUDIOENCRYPTIONTEST1_AES_H