#ifndef __BASE_EVENTS_H__
#define __BASE_EVENTS_H__

enum class BaseEvents_e
{
    TIMEOUT,               // Timeout or sequence over
    MESSAGE_RECEIVED,      // Received a message
    MESSAGE_TRANSMITTED    // Transmitted a message
};

#endif // __BASE_EVENTS_H__
