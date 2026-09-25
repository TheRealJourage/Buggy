// Copy to key_secret.h (git-ignored) and fill in 32 random bytes.
// The same file must be built into the steering wheel AND the master.
// Once the wheel is potted this secret can no longer be changed: keep a backup.
#pragma once
static const uint8_t STEERING_KEY_SECRET[32] = {0};
