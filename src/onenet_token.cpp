/**
 * @file onenet_token.cpp
 * @brief OneNET Token generation for Arduino/ESP32
 * Based on official OneNET SDK
 */

#include "onenet_token.h"
#include <mbedtls/md.h>
#include <mbedtls/base64.h>

#define DEV_TOKEN_LEN 256
#define DEV_TOKEN_VERSION_STR "2018-10-31"

#define DEV_TOKEN_SIG_METHOD_MD5 "md5"
#define DEV_TOKEN_SIG_METHOD_SHA1 "sha1"
#define DEV_TOKEN_SIG_METHOD_SHA256 "sha256"
#define DEV_TOKEN_SIG_METHOD_SM2 "sm2"

// URL编码函数
static void url_encode_char(char** dest, char c) {
    switch (c) {
        case '+':
            strcpy(*dest, "%2B");
            *dest += 3;
            break;
        case ' ':
            strcpy(*dest, "%20");
            *dest += 3;
            break;
        case '/':
            strcpy(*dest, "%2F");
            *dest += 3;
            break;
        case '?':
            strcpy(*dest, "%3F");
            *dest += 3;
            break;
        case '%':
            strcpy(*dest, "%25");
            *dest += 3;
            break;
        case '#':
            strcpy(*dest, "%23");
            *dest += 3;
            break;
        case '&':
            strcpy(*dest, "%26");
            *dest += 3;
            break;
        case '=':
            strcpy(*dest, "%3D");
            *dest += 3;
            break;
        default:
            **dest = c;
            (*dest)++;
            break;
    }
}

int32_t onenet_token_generate(
    char* token, 
    sig_method_e method, 
    uint32_t exp_time, 
    const char* product_id, 
    const char* dev_name, 
    const char* access_key
) {
    if (!token || !product_id || !access_key) {
        return -1;
    }

    char base64_data[128] = {0};
    char str_for_sig[256] = {0};
    unsigned char sign_buf[64] = {0};
    size_t base64_data_len = sizeof(base64_data);
    const char* sig_method_str = NULL;
    uint32_t sign_len = 0;
    uint32_t i = 0;
    char* tmp = NULL;

    // 构建Token开始部分
    sprintf(token, "version=%s", DEV_TOKEN_VERSION_STR);

    // 添加资源路径
    if (dev_name) {
        sprintf(token + strlen(token), "&res=products%%2F%s%%2Fdevices%%2F%s", product_id, dev_name);
    } else {
        sprintf(token + strlen(token), "&res=products%%2F%s", product_id);
    }

    // 添加过期时间
    sprintf(token + strlen(token), "&et=%u", exp_time);

    // Base64解码access_key
    size_t key_len = strlen(access_key);
    size_t decoded_len = 0;
    int ret = mbedtls_base64_decode(NULL, 0, &decoded_len, (const unsigned char*)access_key, key_len);
    if (ret != MBEDTLS_ERR_BASE64_BUFFER_TOO_SMALL) {
        Serial.println("Base64 decode length calculation failed");
        return -1;
    }

    unsigned char* decoded_key = (unsigned char*)malloc(decoded_len);
    if (!decoded_key) {
        Serial.println("Memory allocation failed");
        return -1;
    }

    ret = mbedtls_base64_decode(decoded_key, decoded_len, &decoded_len, (const unsigned char*)access_key, key_len);
    if (ret != 0) {
        Serial.println("Base64 decode failed");
        free(decoded_key);
        return -1;
    }

    // 设置签名方法和长度
    mbedtls_md_type_t md_type;
    switch (method) {
        case SIG_METHOD_MD5:
            md_type = MBEDTLS_MD_MD5;
            sig_method_str = DEV_TOKEN_SIG_METHOD_MD5;
            sign_len = 16;
            break;
        case SIG_METHOD_SHA1:
            md_type = MBEDTLS_MD_SHA1;
            sig_method_str = DEV_TOKEN_SIG_METHOD_SHA1;
            sign_len = 20;
            break;
        case SIG_METHOD_SHA256:
            md_type = MBEDTLS_MD_SHA256;
            sig_method_str = DEV_TOKEN_SIG_METHOD_SHA256;
            sign_len = 32;
            break;
        default:
            Serial.println("Unsupported signature method");
            free(decoded_key);
            return -1;
    }

    // 添加签名方法到token
    sprintf(token + strlen(token), "&method=%s", sig_method_str);

    // 构建用于签名的字符串
    if (dev_name) {
        sprintf(str_for_sig, "%u\n%s\nproducts/%s/devices/%s\n%s", 
                exp_time, sig_method_str, product_id, dev_name, DEV_TOKEN_VERSION_STR);
    } else {
        sprintf(str_for_sig, "%u\n%s\nproducts/%s\n%s", 
                exp_time, sig_method_str, product_id, DEV_TOKEN_VERSION_STR);
    }

    Serial.println("String to sign: " + String(str_for_sig));

    // 计算HMAC签名
    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    
    ret = mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    if (ret != 0) {
        Serial.println("MD setup failed");
        mbedtls_md_free(&ctx);
        free(decoded_key);
        return -1;
    }

    ret = mbedtls_md_hmac_starts(&ctx, decoded_key, decoded_len);
    if (ret != 0) {
        Serial.println("HMAC start failed");
        mbedtls_md_free(&ctx);
        free(decoded_key);
        return -1;
    }

    ret = mbedtls_md_hmac_update(&ctx, (const unsigned char*)str_for_sig, strlen(str_for_sig));
    if (ret != 0) {
        Serial.println("HMAC update failed");
        mbedtls_md_free(&ctx);
        free(decoded_key);
        return -1;
    }

    ret = mbedtls_md_hmac_finish(&ctx, sign_buf);
    if (ret != 0) {
        Serial.println("HMAC finish failed");
        mbedtls_md_free(&ctx);
        free(decoded_key);
        return -1;
    }

    mbedtls_md_free(&ctx);
    free(decoded_key);

    // 将签名结果编码为Base64
    memset(base64_data, 0, sizeof(base64_data));
    base64_data_len = sizeof(base64_data);
    
    ret = mbedtls_base64_encode((unsigned char*)base64_data, base64_data_len, &base64_data_len, sign_buf, sign_len);
    if (ret != 0) {
        Serial.println("Base64 encode failed");
        return -1;
    }

    // 添加签名到token，并进行URL编码
    strcat(token, "&sign=");
    tmp = token + strlen(token);

    for (i = 0; i < base64_data_len; i++) {
        url_encode_char(&tmp, base64_data[i]);
    }

    return 0;
}
