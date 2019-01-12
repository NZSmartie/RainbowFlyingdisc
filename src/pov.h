#ifndef _RAINBOW_FLYING_DISC_POV_H_
#define _RAINBOW_FLYING_DISC_POV_H_

#include <zephyr/types.h>

void pov_init(void);
void pov_set_message(const char *message, ssize_t length);
void pov_set_rpm(u32_t rpm);

#endif // _RAINBOW_FLYING_DISC_POV_H_
