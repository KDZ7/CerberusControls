#ifndef __IIO_HPP__
#define __IIO_HPP__
#include <stddef.h>
#include <iostream>
/*
 * This is the interface class for Input/Output operations.
 */
#if defined(__DEBUG__)
#define LOG_INFO(x) std::cout << "[API INFO]:" << x << std::endl
#define LOG_ERROR(x) std::cerr << "[API ERROR]:" << x << std::endl
#define LOG_WARN(x) std::cerr << "[API WARN]:" << x << std::endl

#else
#define LOG_INFO(x)
#define LOG_ERROR(x)
#define LOG_WARN(x)
#endif

namespace iio
{
    enum STATUS
    {
        ERROR = -1
    };
    class Bus
    {
    public:
        virtual ~Bus() = default;
        virtual int write(void *buf, size_t len) = 0;
        virtual int read(void *buf, size_t len) = 0;
        virtual int readTimeout(void *buf, size_t len, int timeout) = 0;
    };
} // namespace IIO

#endif // __IIO_HPP__