#ifndef _RAINBOW_FLYING_DISC_EVENTS_H_
#define _RAINBOW_FLYING_DISC_EVENTS_H_

#define MAX_MESSAGE_SIZE 80

typedef enum event_type {
    kEventNone,
    kEventMessage,
} event_type_t;

typedef struct {
    event_type_t event_type;
    ssize_t size;
    union {
        char message[MAX_MESSAGE_SIZE];
    };
} event_t;

static int inline event_message(struct k_msgq *msgq, const char* message, ssize_t length, s32_t timeout)
{
    event_t event = {
        .event_type = kEventMessage,
        .size = length
    };

    memcpy(event.message, message, length);

    return k_msgq_put(msgq, &event, timeout);
}

#endif // _RAINBOW_FLYING_DISC_EVENTS_H_
