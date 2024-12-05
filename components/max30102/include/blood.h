/*
 * @Author: mojionghao
 * @Date: 2024-08-12 16:47:53
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-08-13 12:03:23
 * @FilePath: \max30102_test\components\max30102\include\blood.h
 * @Description: 
 */
#pragma once

#include "max30102.h"
#include "algorithm.h"
#include "math.h"

typedef enum
{
    BLD_NORMAL, // 正常
    BLD_ERROR,  // 侦测错误

} BloodState; // 血液状态

typedef struct
{
    int heart;  // 心率数据
    float SpO2; // 血氧数据
} BloodData;



void blood_Loop(max30102_handle_t sensor, float *heart, float *spo2);