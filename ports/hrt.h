/**
 * @file hrt.h
 * @author Pumpkin Rice
 * @brief 时钟定义
 * @version 0.1
 * @date 2026-07-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef _PICA_DRIVE_HRT_H_
#define _PICA_DRIVE_HRT_H_

#include "hrt.h"

#include <stdint.h>
#include <time.h>

/**
 * @brief 高精度时钟数据类型，us
 * 
 */
typedef uint64_t    hrt_abstime;
#define HRT_ABS_TIME_MAX    (UINT64_MAX)

/**
 * @brief 高精度时钟数据类型，ns
 * 
 */
typedef uint64_t hrt_absnano;
#define HRT_ABS_NANO_MAX    (UINT64_MAX)

/**
 * @brief hrt 定时器回调函数
 * @note 此类函数在定时器中断上下文中运行，不能存在阻塞操作
 */
typedef void (*hrt_callback)(void *arg);

struct hrt_call {
    hrt_abstime     deadline;
    hrt_abstime     period;
    hrt_callback    callback;

    void            *arg;
};
typedef struct hrt_call hrt_call_t;

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief 初始化 hrt_init 相关底层接口
 * 
 */
void hrt_init(void);

/**
 * @brief 获取 hrt_abstime
 * 
 * @return hrt_abstime us
 */
hrt_abstime hrt_absolute_time();

/**
 * @brief 获取纳秒时间戳
 * 
 * @return hrt_absnano ns
 */
hrt_absnano hrt_absolute_nano();

#define absnano_to_abstime(_nano) ((_nano)/1000)
#define abstime_to_absnano(_us)   ((_us)*1000)

/**
 * @brief 计算时间戳与当前时间差
 * 
 * @param then 
 * @return hrt_abstime 
 */
static inline hrt_abstime hrt_elapsed_time(const hrt_abstime *then)
{
    hrt_abstime now = hrt_absolute_time();

    // Cannot allow a negative elapsed time as this would appear
    // to be a huge positive elapsed time when represented as an
    // unsigned value!
    if (*then > now) {
        return 0;
    }

    return now - *then;
}

static inline hrt_absnano hrt_elapsed_nano(const hrt_absnano *then)
{
    hrt_absnano now = hrt_absolute_nano();

    if (*then > now) {
        return 0;
    }

    return now - *then;
}

/**
 * @brief 计算时间戳差值
 * 
 * @param[in] end 结束时刻
 * @param[in] start 起始时刻
 * @return hrt_absnano 时间差值，ns
 * @retval 0 时刻相同，或时间有误
 */
static inline hrt_absnano hrt_diff_nano(const hrt_absnano *end, const hrt_absnano *start)
{
    if (*start > *end) {
        // 启动时为0,最大计数值为 500+年，无需考虑回绕情况
        return 0;
    }

    return *end - *start;
}

inline hrt_abstime hrt_absolute_tick();

static inline float hrt_tick_to_second(hrt_abstime tick)
{
    return 0.1f;
}

/**
 * @brief timespec(ns) to abstime(us)
 * 
 * @param[in] ts 
 * @return hrt_abstime 
 */
static inline hrt_abstime ts_to_abstime(const struct timespec *ts) {
    hrt_abstime ret;

    ret = (hrt_abstime)(ts->tv_sec) * 1000000;
    ret += (hrt_abstime)(ts->tv_nsec / 1000);

    return ret;
}

/**
 * @brief convert abstime to timespec
 * 
 * @param[in] abstime 
 * @param[in] ts 
 */
static inline void asbtime_to_ts(hrt_abstime abstime, struct timespec *ts)
{
    ts->tv_sec = (typeof(ts->tv_sec))(abstime / 1000000);
    abstime -= (hrt_abstime)(ts->tv_sec) * 1000000;
    ts->tv_nsec = (typeof(ts->tv_nsec))(abstime * 1000);
}

/**
 * @brief Convert hrt_abstime(us) to timeval(us)
 * 
 * @param[in] abstime us
 * @param[in] tv us
 */
static inline void abstime_to_timeval(hrt_abstime abstime, struct timeval *tv)
{
    tv->tv_sec = abstime / 1000000;
    tv->tv_usec = abstime - (tv->tv_sec * 1000000);
}

static inline void timeval_to_abstime(const struct timeval *tv, hrt_abstime abstime)
{
    abstime = tv->tv_sec * 1000000 + tv->tv_usec;
}

#ifdef __cplusplus
 }
#endif

#ifdef	__cplusplus

namespace time_literals
{

// User-defined integer literals for different time units.
// The base unit is hrt_abstime in microseconds

constexpr hrt_abstime operator ""_s(unsigned long long seconds)
{
    return hrt_abstime(seconds * 1000000ULL);
}

constexpr hrt_abstime operator ""_ms(unsigned long long milliseconds)
{
    return hrt_abstime(milliseconds * 1000ULL);
}

constexpr hrt_abstime operator ""_us(unsigned long long microseconds)
{
    return hrt_abstime(microseconds);
}

} /* namespace time_literals */

#endif /* __cplusplus */

#endif /* !_PICA_DRIVE_HRT_ */
