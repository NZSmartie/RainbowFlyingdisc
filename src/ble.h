#ifndef _RAINBOW_FLYING_DISC_BLE_H_
#define _RAINBOW_FLYING_DISC_BLE_H_

#include <bluetooth/uuid.h>

// d9a204a1-9ca4-41ec-9dd6-526d6c53c7fe
#define UUID_RAINBOW_FLYING_DISC_SERVICE \
    0xfe, 0xc7, 0x53, 0x6c, 0x6d, 0x52, 0xd6, 0x9d, \
    0xec, 0x41, 0xa4, 0x9c, 0xa1, 0x04, 0xa2, 0xd9

// d9a204a2-9ca4-41ec-9dd6-526d6c53c7fe
#define UUID_RAINBOW_FLYING_DISC_CHARACTERISTIC_MESSAGE \
    0xfe, 0xc7, 0x53, 0x6c, 0x6d, 0x52, 0xd6, 0x9d, \
    0xec, 0x41, 0xa4, 0x9c, 0xa2, 0x04, 0xa2,0xd9

#ifdef __cplusplus
extern "C" {
#endif

void rainbow_flying_disc_init(void);
void rainbow_flying_disc_discover(void);
void rainbow_flying_disc_cancel_connect(void);

// mbox?
// semaphors?
// callback?

#ifdef __cplusplus
}
#endif

#endif // _RAINBOW_FLYING_DISC_BLE_H_
