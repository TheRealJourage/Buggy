#include "steering_key.h"
#include <string.h>

#if __has_include("key_secret.h")
#include "key_secret.h"
#else
#error "lib/steering_key/key_secret.h missing: copy key_secret.example.h and fill in the shared secret"
#endif

#ifdef __AVR__
#include <avr/pgmspace.h>
#define K_READ(i) pgm_read_dword(&K[i])
#else
#define PROGMEM
#define K_READ(i) K[i]
#endif

// On the Uno the round constants stay in flash, RAM is only 2 KB
static const uint32_t K[64] PROGMEM = {
  0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
  0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
  0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
  0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
  0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
  0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
  0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
  0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

#define ROTR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

struct Sha256 {
  uint32_t h[8];
  uint8_t buf[64];
  uint8_t len;
  uint32_t total;
};

static void shaInit(Sha256* s) {
  static const uint32_t H0[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  memcpy(s->h, H0, sizeof H0);
  s->len = 0;
  s->total = 0;
}

// Message schedule kept as a rolling 16-word window instead of 64 words
static void shaCompress(Sha256* s) {
  uint32_t w[16];
  for (uint8_t i = 0; i < 16; i++) {
    w[i] = (uint32_t)s->buf[4 * i] << 24 | (uint32_t)s->buf[4 * i + 1] << 16 |
           (uint32_t)s->buf[4 * i + 2] << 8 | s->buf[4 * i + 3];
  }
  uint32_t a = s->h[0], b = s->h[1], c = s->h[2], d = s->h[3];
  uint32_t e = s->h[4], f = s->h[5], g = s->h[6], h = s->h[7];
  for (uint8_t i = 0; i < 64; i++) {
    if (i >= 16) {
      uint32_t w15 = w[(i + 1) & 15], w2 = w[(i + 14) & 15];
      w[i & 15] += (ROTR(w2, 17) ^ ROTR(w2, 19) ^ (w2 >> 10)) + w[(i + 9) & 15] +
                   (ROTR(w15, 7) ^ ROTR(w15, 18) ^ (w15 >> 3));
    }
    uint32_t t1 = h + (ROTR(e, 6) ^ ROTR(e, 11) ^ ROTR(e, 25)) + ((e & f) ^ (~e & g)) + K_READ(i) + w[i & 15];
    uint32_t t2 = (ROTR(a, 2) ^ ROTR(a, 13) ^ ROTR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
    h = g; g = f; f = e; e = d + t1;
    d = c; c = b; b = a; a = t1 + t2;
  }
  s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d;
  s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}

static void shaUpdate(Sha256* s, const uint8_t* data, size_t n) {
  while (n--) {
    s->buf[s->len++] = *data++;
    if (s->len == 64) {
      shaCompress(s);
      s->len = 0;
    }
    s->total++;
  }
}

static void shaFinal(Sha256* s, uint8_t out[32]) {
  uint32_t bits = s->total * 8;
  s->buf[s->len++] = 0x80;
  if (s->len > 56) {
    while (s->len < 64) s->buf[s->len++] = 0;
    shaCompress(s);
    s->len = 0;
  }
  while (s->len < 60) s->buf[s->len++] = 0;
  s->buf[60] = bits >> 24;
  s->buf[61] = bits >> 16;
  s->buf[62] = bits >> 8;
  s->buf[63] = bits;
  shaCompress(s);
  for (uint8_t i = 0; i < 8; i++) {
    out[4 * i] = s->h[i] >> 24;
    out[4 * i + 1] = s->h[i] >> 16;
    out[4 * i + 2] = s->h[i] >> 8;
    out[4 * i + 3] = s->h[i];
  }
}

// keyLen <= 64
void hmacSha256(const uint8_t* key, uint8_t keyLen, const uint8_t* msg, size_t msgLen, uint8_t out[32]) {
  uint8_t pad[64];
  Sha256 s;

  for (uint8_t i = 0; i < 64; i++) pad[i] = (i < keyLen ? key[i] : 0) ^ 0x36;
  shaInit(&s);
  shaUpdate(&s, pad, 64);
  shaUpdate(&s, msg, msgLen);
  shaFinal(&s, out);

  for (uint8_t i = 0; i < 64; i++) pad[i] = (i < keyLen ? key[i] : 0) ^ 0x5c;
  shaInit(&s);
  shaUpdate(&s, pad, 64);
  shaUpdate(&s, out, 32);
  shaFinal(&s, out);
}

void steeringKeyResponse(const uint8_t challenge[STEERING_KEY_LEN], uint8_t response[STEERING_KEY_LEN]) {
  uint8_t mac[32];
  hmacSha256(STEERING_KEY_SECRET, sizeof STEERING_KEY_SECRET, challenge, STEERING_KEY_LEN, mac);
  memcpy(response, mac, STEERING_KEY_LEN);
}

bool steeringKeySelfTest() {
  static const uint8_t expected[8] = {0x5b, 0xdc, 0xc1, 0x46, 0xbf, 0x60, 0x75, 0x4e};
  uint8_t mac[32];
  hmacSha256((const uint8_t*)"Jefe", 4, (const uint8_t*)"what do ya want for nothing?", 28, mac);
  return memcmp(mac, expected, sizeof expected) == 0;
}
