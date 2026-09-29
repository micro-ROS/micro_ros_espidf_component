#include <errno.h>
#include <time.h>
#include <unistd.h>

// ESP-IDF defines _POSIX_TIMERS but does not implement nanosleep()
// Declared weak so that a native ESP-IDF implementation takes precedence
__attribute__((weak)) int nanosleep(const struct timespec * req, struct timespec * rem)
{
    (void) rem; // Never interrupted: FreeRTOS has no signals

    if (req == NULL ||
        req->tv_sec < 0 ||
        req->tv_nsec < 0 ||
        req->tv_nsec >= 1000000000L) {
        errno = EINVAL;
        return -1;
    }

    // Sleep the whole seconds without overflowing useconds_t
    for (time_t s = 0; s < req->tv_sec; s++) {
        usleep(1000000);
    }

    // Round the remaining nanoseconds up to microseconds
    useconds_t us = (req->tv_nsec + 999) / 1000;
    if (us > 0) {
        usleep(us);
    }

    return 0;
}
