#include "soil.h"
#include "adc.h" // 包含STM32Cubemx生成的ADC驱动代码
#include "app_main.h"

// 定义阈值（适配电压计算场景，3.3V量程下，1e-4f对应约0.33mV，足够精确）
#define EPSILON (1e-4f) // 极小阈值，可根据场景调整

// 变量声明
typedef struct
{
    uint32_t res; // 土壤湿度对应的阻值
    uint8_t humi; // 土壤湿度等级
} Soil_TypeDef;

// 土壤传感器阻值与土壤湿度等级对应关系表
const Soil_TypeDef Soil[] =
    {
        {100000, 1}, // 1. 干燥
        {50000, 2},  // 2. 微湿
        {20000, 3},  // 3. 湿润
        {10000, 4},  // 4. 水分饱和
};

// 获取土壤传感器电压
static float get_soil_voltage(void)
{
    float voltage = 0;
//    static uint8_t first = 1;
//    if (first)
//    {
//        HAL_ADC_Start(&hadc2); // 启动ADC
//        first = 0;
//    }
//    uint16_t adc_value = HAL_ADC_GetValue(&hadc2); // 获取ADC值
    uint16_t adc_value = adc1_values[1];
		voltage = (float)adc_value * 3.3 / 4096;       // 将ADC值转换为电压值
    return voltage;
}

// 获取土壤湿度等级
uint8_t GetSoilHumidity(void)
{
    int i;
    uint8_t humi = 0;
    float voltage = get_soil_voltage();
    printf("soil voltage: %.1f\n", voltage);

    // 电压转换为电阻值 v = 3.3 * r / (r + 10k), r = 10k * v / (3.3 - v)
    if (((3.3 - voltage) >= 0.0f && (3.3 - voltage) < EPSILON) ||
        ((3.3 - voltage) < 0.0f && -(3.3 - voltage) < EPSILON)) // 防止除零错误
        return 1;                                               // 如果电压接近3.3V，则阻值非常大，土壤非常干燥，返回1

    uint32_t resistance = (uint32_t)(10.0f * voltage * 1000) / (3.3 - voltage);
    printf("soil resistance: %d\n", resistance);
    // 查表法，根据电阻值得出土壤湿度
    for (i = 0; i < sizeof(Soil) / sizeof(Soil[0]); i++)
    {
        humi = Soil[i].humi;
        if (resistance >= Soil[i].res)
        {
            break;
        }
    }
    return humi;
}
