/*
 * @Author: mojionghao
 * @Date: 2024-08-12 16:47:41
 * @LastEditors: mojionghao
 * @LastEditTime: 2024-10-13 21:33:38
 * @FilePath: \max30102_test\components\max30102\src\blood.c
 * @Description:
 */
#include "blood.h"
#include "esp_log.h"

// uint16_t g_fft_index = 0;
struct compx s1[FFT_N + 16];
struct compx s2[FFT_N + 16];
float Data_heart, Data_spo2;
uint16_t g_fft_index = 0;
struct
{
    float Hp;
    float HPO2;
} g_BloodWave;

BloodData g_blooddata = {0}; // 血液数据存储

#define CORRECTED_VALUE 47 // 标定血液氧气含量

/*funcation start ------------------------------------------------------------*/
// 血液检测信息更新
void blood_data_update(max30102_handle_t sensor)
{

    uint16_t fifo_red = 0, fifo_ir = 0;
    max30102_dev_t *sens = (max30102_dev_t *)sensor;
    while (g_fft_index < FFT_N)
    {
        while (gpio_get_level(sens->int_pin) == 0)
        {
            max30102_read_fifo(sensor, &fifo_red, &fifo_ir);
            if (g_fft_index < FFT_N)
            {
                s1[g_fft_index].real = fifo_red;
                s1[g_fft_index].imag = 0;

                s2[g_fft_index].real = fifo_ir;
                s2[g_fft_index].imag = 0;

                g_fft_index++;
            }
        }
    }
}

void blood_data_translate(void)
{
    float n_denom;
    uint16_t i;

    // 直流滤波
    float dc_red = 0;
    float dc_ir = 0;
    float ac_red = 0;
    float ac_ir = 0;

    for (i = 0; i < FFT_N; i++)
    {
        dc_red += s1[i].real;
        dc_ir += s2[i].real;
    }
    dc_red = dc_red / FFT_N;
    dc_ir = dc_ir / FFT_N;
    for (i = 0; i < FFT_N; i++)
    {
        s1[i].real = s1[i].real - dc_red;
        s2[i].real = s2[i].real - dc_ir;
    }

    // 移动平均滤波

    for (i = 1; i < FFT_N - 1; i++)
    {
        n_denom = (s1[i - 1].real + 2 * s1[i].real + s1[i + 1].real);
        s1[i].real = n_denom / 4.00;

        n_denom = (s2[i - 1].real + 2 * s2[i].real + s2[i + 1].real);
        s2[i].real = n_denom / 4.00;
    }
    for (i = 0; i < FFT_N - 8; i++)
    {
        n_denom = (s1[i].real + s1[i + 1].real + s1[i + 2].real + s1[i + 3].real + s1[i + 4].real + s1[i + 5].real + s1[i + 6].real + s1[i + 7].real);
        s1[i].real = n_denom / 8.00;

        n_denom = (s2[i].real + s2[i + 1].real + s2[i + 2].real + s2[i + 3].real + s2[i + 4].real + s2[i + 5].real + s2[i + 6].real + s2[i + 7].real);
        s2[i].real = n_denom / 8.00;
    }

    g_fft_index = 0;
    FFT(s1);
    FFT(s2);
    for (i = 0; i < FFT_N; i++)
    {
        s1[i].real = sqrtf(s1[i].real * s1[i].real + s1[i].imag * s1[i].imag);
        s1[i].real = sqrtf(s2[i].real * s2[i].real + s2[i].imag * s2[i].imag);
    }
    for (i = 1; i < FFT_N; i++)
    {
        ac_red += s1[i].real;
        ac_ir += s2[i].real;
    }
    for (i = 0; i < 50; i++)
    {
        if (s1[i].real <= 10)
            break;
    }
    // 读取峰值点的横坐标  结果的物理意义为
    int s1_max_index = find_max_num_index(s1, 30);
    int s2_max_index = find_max_num_index(s2, 30);
    //	UsartPrintf(USART_DEBUG,"%d\r\n",s1_max_index);
    //	UsartPrintf(USART_DEBUG,"%d\r\n",s2_max_index);
    float Heart_Rate = 60.00 * ((100.0 * s1_max_index) / 512.00) + 20;

    g_blooddata.heart = Heart_Rate;

    float R = (ac_ir * dc_red) / (ac_red * dc_ir);
    float sp02_num = -45.060 * R * R + 30.354 * R + 94.845;
    g_blooddata.SpO2 = sp02_num;
}

void blood_Loop(max30102_handle_t sensor, float *heart, float *spo2)
{
    blood_data_update(sensor);
    blood_data_translate();
    g_blooddata.SpO2 = (g_blooddata.SpO2 > 99.99) ? 99.99 : g_blooddata.SpO2;
    if (isnan(g_blooddata.SpO2) || g_blooddata.heart == 66)
    {
        g_blooddata.SpO2 = 0;
        ESP_LOGE("blood", "No human body detected!");
    }
    *heart = g_blooddata.heart;
    *spo2 = g_blooddata.SpO2;
}

