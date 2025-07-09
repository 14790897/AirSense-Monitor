/**
 * @file onenet_token.h
 * @brief OneNET Token generation for Arduino/ESP32
 * Based on official OneNET SDK
 */

#ifndef __ONENET_TOKEN_H__
#define __ONENET_TOKEN_H__

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

// 签名方法枚举
typedef enum {
    SIG_METHOD_MD5 = 0,
    SIG_METHOD_SHA1,
    SIG_METHOD_SHA256,
    SIG_METHOD_SM2
} sig_method_e;

// Token生成函数声明
int32_t onenet_token_generate(
    char* token, 
    sig_method_e method, 
    uint32_t exp_time, 
    const char* product_id, 
    const char* dev_name, 
    const char* access_key
);

#ifdef __cplusplus
}
#endif

#endif /* __ONENET_TOKEN_H__ */
