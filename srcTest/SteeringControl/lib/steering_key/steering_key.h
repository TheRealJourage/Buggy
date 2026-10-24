/*******************************************************************************
 * STEERING KEY - challenge-response between steering wheel and master
 *
 * The wheel proves it knows the shared secret (key_secret.h) by answering an
 * 8-byte random challenge with the first 8 bytes of HMAC-SHA256(secret, challenge).
 * Used by the RP2040 slave and the master (Uno now, STM32 hub later).
 ******************************************************************************/
#pragma once
#include <stdint.h>
#include <stddef.h>

#define STEERING_KEY_LEN 8

void steeringKeyResponse(const uint8_t challenge[STEERING_KEY_LEN], uint8_t response[STEERING_KEY_LEN]);

void hmacSha256(const uint8_t* key, uint8_t keyLen, const uint8_t* msg, size_t msgLen, uint8_t out[32]);

// RFC 4231 test case 2, true if this build computes HMAC-SHA256 correctly
bool steeringKeySelfTest();
