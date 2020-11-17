#ifndef LOG_H_
#define LOG_H_

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_INFO,
    LOG_LEVEL_ERROR,
} log_level_t;

// TODO: THese macro need to be implemented
#define LOG_DEBUG(MESSAGE)

#define LOG_WARNING(MESSAGE)

#define LOG_INFO(MESSAGE)

#define LOG_ERROR(MESSAGE)

// TODO: Add trottle macro

#ifdef __cplusplus
}
#endif

#endif /* LOG_H_ */
