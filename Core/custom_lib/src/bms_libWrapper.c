/*
 * bms_libWrapper.cpp
 *
 *  创建日期: 2024年11月24日
 *      作者: amrlxyz
 */

/*
 * 兼容命令映射表
 * ADBMS2950 对应 ADBMS6830
 *
 * RDCFGA
 * RDCFGB
 * ADI1     = ADCV      (2950启动电流/电压ADC 对应 6830启动主ADC)
 * ADI2     = ADSV      (2950启动辅助ADC 对应 6830启动冗余ADC)
 * RDI      = RDFCA or RDCVA (读取电流 对应 读取滤波/瞬时电压)
 * RDVB     = RDFCB or RDCVB
 * RDIVB1   = RDFCC or RDCVC
 * RDIACC   = RDACA     (读取电流累加 对应 读取平均电压)
 * RDVBACC  = RDACB
 * RDIVB1ACC= RDACC
 *
 */

/*
 * 命令说明
 *
 * -- 6830 -- (从控芯片，测电芯电压和温度)
 * ADCV : 启动主ADC转换
 * ADSV : 启动冗余ADC转换
 * RDCVA: 读取A组电芯电压
 * RDFCA: 读取滤波后的A组电芯电压
 * RDACA: 读取平均后的A组电芯电压
 *
 * -- 2950 -- (主控芯片，测总压和电流)
 * ADIx: 启动IxADC和VBxADC
 * RDI : 读取I1ADC和I2ADC结果
 *
 */


#include "bms_libWrapper.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_cmdlist.h"

#include <string.h>
#include <stdio.h>

#include "main.h"
#include "uartDMA.h"
#include "bms_can.h"


// 2950芯片数据结构体(主控，测总压/电流)
typedef struct
{
    ad29_cfa_t cfa_Tx; // 配置寄存器A 发送缓存
    ad29_cfa_t cfa_Rx; // 配置寄存器A 接收缓存
    ad29_cfb_t cfb_Tx; // 配置寄存器B 发送缓存
    ad29_cfb_t cfb_Rx; // 配置寄存器B 接收缓存

    float current1;    // 电流传感器1读数
    float current2;    // 电流传感器2读数
    float vb1;         // 总压测量1
    float vb2;         // 总压测量2

} Ic_ad29;


// 6830芯片数据结构体(从控，测电芯电压/温度)
typedef struct
{
    // 来自/用于 配置寄存器
    ad68_cfa_t cfa_Tx   [TOTAL_AD68]; // 配置A发送缓存
    ad68_cfa_t cfa_Rx   [TOTAL_AD68]; // 配置A接收缓存
    ad68_cfb_t cfb_Tx   [TOTAL_AD68]; // 配置B发送缓存
    ad68_cfb_t cfb_Rx   [TOTAL_AD68]; // 配置B接收缓存

    ad68_pwma_t pwma    [TOTAL_AD68]; // PWM寄存器A缓存(控制1-12号电芯均衡)
    ad68_pwmb_t pwmb    [TOTAL_AD68]; // PWM寄存器B缓存(控制13-16号电芯均衡)

    // 来自读取寄存器
    float v_cell        [TOTAL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL]; // 各种类型的电芯电压(瞬时/平均/滤波等)
    // 计算值
    float v_cell_diff   [TOTAL_VOLTAGE_TYPES][TOTAL_AD68][TOTAL_CELL]; // 电芯与最低电压的压差
    float v_cell_sum    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];             // 单个芯片所有电芯电压总和
    float v_cell_avg    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];             // 单个芯片电芯平均电压
    float v_cell_min    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];             // 单个芯片最低电压
    float v_cell_max    [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];             // 单个芯片最高电压
    float v_cell_delta  [TOTAL_VOLTAGE_TYPES][TOTAL_AD68];             // 单个芯片最高最低压差

    // 来自辅助ADC测量(温度等)
    float v_segment     [TOTAL_AD68];             // 模块总电压
    float temp_cell     [TOTAL_AD68][TOTAL_CELL]; // 电芯温度
    float temp_ic       [TOTAL_AD68];             // 芯片内部温度

    // 以位存储的标志
    uint16_t isDischarging          [TOTAL_AD68]; // 正在均衡放电标志位
    uint16_t isCellFaultDetected    [TOTAL_AD68]; // 电芯故障检测标志位

} Ic_ad68;


// 整个电池包的公共数据结构
typedef struct
{
    bool isFaultDetected    [TOTAL_IC]; // 各芯片故障标志
    bool isCommsError       [TOTAL_IC]; // 各芯片通讯错误标志

    // 电池包状态
    float v_pack_total; // 电池包总压
    float v_pack_min;   // 包内单体最低电压
    float v_pack_max;   // 包内单体最高电压
} Ic_common;


Ic_common   ic_common; // 公共状态全局变量
Ic_ad29     ic_ad29;   // 2950数据全局变量
Ic_ad68     ic_ad68;   // 6830数据全局变量

uint8_t  txData[TOTAL_IC][DATA_LEN]; // SPI发送数据缓冲区
uint8_t  rxData[TOTAL_IC][DATA_LEN]; // SPI接收数据缓冲区
uint16_t rxPec[TOTAL_IC];           // 接收到的PEC校验码
uint8_t  rxCc[TOTAL_IC];            // 接收到的命令计数器

CanTxMsg canTxBuffer[CAN_BUFFER_LEN] = {0}; // CAN发送缓冲区

VoltageTypes dischargeVoltageType = VOLTAGE_S; // 用于均衡判断的电压类型(此处选为S冗余电压)

uint32_t BMS_StatusFlags = BMS_ERR_COMMS; // BMS状态标志位(按位存储错误)，初始设为通讯错误

ChargerConfiguration chargerConfig = { // 充电机配置
        .max_current = 1,           // 最大电流
        .target_voltage = 450,      // 目标电压
        .disable_charging = 1,      // 禁止充电标志(1=禁止)
};

static const float balancingThreshold = 0.030; // 均衡开启压差阈值(伏特)，30mV

static const bool DEBUG_SERIAL_VOLTAGE_ENABLED = false; // 串口打印电压调试开关
static const bool DEBUG_SERIAL_AUX_ENABLED = false;     // 串口打印辅助测量调试开关
static const bool DEBUG_SERIAL_MASTER_MEASUREMENTS = false; // 串口打印主控测量调试开关

volatile bool enableBalancing = true; // 均衡使能标志

/**
 * @brief 重置BMS系统中所有AFE芯片的配置寄存器为默认值
 * 
 * @details 该函数主要用于系统上电初始化阶段，确保所有2950和6830芯片的配置寄存器
 *          处于一个已知的、干净的初始状态，防止芯片内部残留的随机状态或
 *          上次运行的配置影响当前的电压采集和充放电控制。
 * 
 *          具体操作：
 *          1. 定义2950和6830芯片的配置寄存器A和B的默认值。
 *             注意：这里的默认值是经过字节序翻转的，因为MCU的小端模式与
 *             AFE芯片SPI通信要求的大端序不一致，提前翻转是为了后续直接内存拷贝。
 *          2. 如果系统中存在2950芯片，将其默认值拷贝到2950的接收缓存结构体中暂存。
 *          3. 遍历系统中所有的6830芯片，将其默认值拷贝到6830的发送缓存结构体中，
 *             为后续调用 bms_writeRegister 正式下发配置做准备。
 */
void bms_resetConfig(void)
{
    // 芯片复位后的默认寄存器值(从RDCFG读取获得)
    // 由于小端模式，字节序被翻转了
    uint64_t const ad29_cfaDefault = 0x113F3F000000;
    uint64_t const ad29_cfbDefault = 0xF00100000000;
    uint64_t const ad68_cfaDefault = 0x0003FF000001;
    uint64_t const ad68_cfbDefault = 0x0000007FF800;

    // 将默认值拷贝到发送缓冲区(实际上是借用Rx结构体暂存，后续会拷贝到Tx)
    if (TOTAL_AD29)
    {
        memcpy(&ic_ad29.cfa_Rx, &ad29_cfaDefault, DATA_LEN);
        memcpy(&ic_ad29.cfb_Rx, &ad29_cfbDefault, DATA_LEN);
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        memcpy(&ic_ad68.cfa_Tx[ic], &ad68_cfaDefault, DATA_LEN);
        memcpy(&ic_ad68.cfb_Tx[ic], &ad68_cfbDefault, DATA_LEN);
    }
}

/**
 * @brief 向底层AFE芯片写入指定类型的寄存器
 * 
 * @param regType 要写入的寄存器类型，为 RegisterTypes 枚举变量
 *                (REG_CONFIG_A: 配置寄存器A,
 *                 REG_CONFIG_B: 配置寄存器B,
 *                 REG_PWM_A: PWM寄存器A,
 *                 REG_PWM_B: PWM寄存器B)
 * 
 * @details 该函数是BMS向下发配置的核心封装。由于系统采用菊花链通信，
 *          发送数据时需要将所有IC的数据按顺序拼接到一起。
 * 
 *          执行流程：
 *          1. 根据 regType 进入对应的 switch 分支。
 *          2. 将内存中准备好的数据（如 ic_ad68.cfa_Tx）按顺序拷贝到
 *             SPI底层发送缓冲区 txData 中。
 *             - 如果系统包含2950芯片，其数据固定放在 txData[0]。
 *             - 6830芯片的数据紧随其后，放在 txData[ic + TOTAL_AD29]。
 *             - 对于PWM寄存器，2950不支持该功能，用 0x00 填充占位。
 *          3. 根据寄存器类型，匹配对应的SPI操作命令码（如 WRCFGA, WRPWMA 等）。
 *          4. 调用底层通信接口 bms_transmitData，将命令码和拼接好的 txData 
 *             打包下发到菊花链，完成寄存器写入。
 */
void bms_writeRegister(RegisterTypes regType)
{
    uint8_t* command;

    // 根据寄存器类型处理逻辑
    switch (regType)
    {
        case REG_CONFIG_A:
            // 准备写入配置寄存器A的数据
            if (TOTAL_AD29) {
                memcpy(txData[0], &ic_ad29.cfa_Tx, DATA_LEN);
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.cfa_Tx[ic], DATA_LEN);
            }
            command = WRCFGA; // 写配置A命令
            break;

        case REG_CONFIG_B:
            // 准备写入配置寄存器B的数据
            if (TOTAL_AD29) {
                memcpy(txData[0], &ic_ad29.cfb_Tx, DATA_LEN);
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.cfb_Tx[ic], DATA_LEN);
            }
            command = WRCFGB; // 写配置B命令
            break;

        case REG_PWM_A:
            // 准备写入PWM寄存器组A的数据
            if (TOTAL_AD29) {
                memset(txData[0], 0x00, DATA_LEN); // 2950无此功能，填0填充
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.pwma[ic], DATA_LEN);
            }
            command = WRPWMA; // 写PWM A命令
            break;

        case REG_PWM_B:
            // 准备写入PWM寄存器组B的数据
            if (TOTAL_AD29) {
                memset(txData[0], 0x00, DATA_LEN); // 2950无此功能，填0填充
            }
            for (int ic = 0; ic < TOTAL_AD68; ic++) {
                memcpy(txData[ic + TOTAL_AD29], &ic_ad68.pwmb[ic], DATA_LEN);
            }
            command = WRPWMB; // 写PWM B命令
            break;

        default:
            // 无效的寄存器类型
            Error_Handler();
            return;
    }

    // 准备好缓冲区后，使用选定的命令发送数据(所有情况共用)
    bms_transmitData(command, txData);
}

/**
 * @brief BMS系统初始化函数
 * 
 * @return BMS_OK 初始化成功
 * @return BMS_ERR_COMMS 菊花链通讯失败（唤醒或读取SID校验未通过）
 * 
 * @details 该函数是BMS采集板软件的入口初始化流程，负责将AFE芯片配置到正常工作状态。
 *          主要执行以下步骤：
 *          1. 恢复默认配置：调用 bms_resetConfig 将2950和6830的配置寄存器设为已知默认值。
 *          2. 配置2950 GPIO：将2950的GPIO1和GPIO2配置为推挽输出，用于控制外部电路（如继电器）。
 *          3. 配置6830基准与滤波：开启6830内部基准电压（保持常开以加快ADC转换），
 *             设置内部滤波器截止频率为110Hz。
 *          4. 唤醒与通讯测试：唤醒菊花链上的芯片（休眠超过4ms需重新唤醒），
 *             并通过读取SID寄存器验证SPI通讯是否正常。
 *          5. 下发配置并启动ADC：将配置写入芯片，启动连续主ADC转换，
 *             并主动延时12ms等待ADC首次转换完成及平均寄存器填满数据。
 */
BMS_StatusTypeDef bms_init(void)
{
    bms_resetConfig();

    // 针对2950 - 使能电压测量(参考原理图)
    if (TOTAL_AD29)
    {
        ic_ad29.cfa_Tx.gpo1c  = 1;      // GPIO1状态控制
        ic_ad29.cfa_Tx.gpo1od = 0;      // 1=开漏, 0=推挽
        ic_ad29.cfa_Tx.gpo2c  = 1;      // GPIO2状态控制
        ic_ad29.cfa_Tx.gpo2od = 0;      // 1=开漏, 0=推挽
    }

    // 针对6830的配置
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        ic_ad68.cfa_Tx[ic].refon = 0b1;  // 开启内部基准电压(保持常开以加快ADC转换)
        ic_ad68.cfa_Tx[ic].fc = 0b001;   // 滤波器截止频率设为110Hz
    }

    printfDma("\n --- BMS初始化 --- \n");

    bms_wakeupChain();                                  // 唤醒菊花链(超过4ms不活动会休眠)
    if (bms_readRegister(REG_SID) == BMS_ERR_COMMS)     // 读取SID确保通讯正常
    {
        return BMS_ERR_COMMS;
    }

    bms_writeRegister(REG_CONFIG_A);     // 写入配置A
    bms_startAdcvCont(false);            // 启动连续ADC，需要等待8ms让平均寄存器填满
    bms_delayMsActive(12);               // 主动延时12ms

    return BMS_OK;
}

/**
 * @brief 设置6830芯片的GPIO4和GPIO5引脚状态，用于控制外部多路复用器(MUX)
 * 
 * @param twoBitIndex 2位索引值，用于控制MUX通道切换。
 *                    该值的低2位将决定GPIO4和GPIO5的输出状态。
 * 
 * @details 6830芯片的GPIO4和GPIO5被用来控制外部模拟多路复用器（如温度传感器切换）。
 *          - GPIO输出逻辑：1 = 无下拉（默认高阻态），0 = 下拉（低电平）。
 * 
 *          该函数的核心逻辑是位操作：
 *          1. 将传入的 twoBitIndex 左移3位，使其对齐到GPIO4和GPIO5在寄存器中的位置。
 *          2. 使用按位或操作，将这个值与屏蔽码 (0xFF ^ (0x3 << 3)) 进行组合。
 *             屏蔽码的作用是保持其他GPIO引脚（GPIO1-3, GPIO6-8）的状态不变（强制为1），
 *             只允许修改GPIO4和GPIO5对应的位。
 *          3. 修改完内存中的配置结构体后，调用 bms_writeRegister 将最新的配置
 *             写入6830芯片的配置寄存器A，使GPIO状态立即生效。
 */
void bms68_setGpo45(uint8_t twoBitIndex)
{
    // GPIO输出: 1=无下拉(默认), 0=下拉
    // 仅针对引脚4和5(用于控制外部多路复用器)
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        // 将2位索引移到第3和第4位，其余位保持1(不影响其他GPIO)
        ic_ad68.cfa_Tx[ic].gpo1to8 = ((twoBitIndex) << 3) | (0xFF ^ (0x3 << 3));
    }

    bms_writeRegister(REG_CONFIG_A); // 写入配置更新GPIO状态
}

/**
 * @brief 通过串口DMA打印所有IC接收到的原始寄存器数据和命令计数器(主要用于调试)
 * 
 * @param data 接收到的原始字节数组，二维数组结构 [IC索引][数据字节]
 * @param cc 命令计数器数组，记录每个IC接收到的有效命令数量 [IC索引]
 * 
 * @details 该函数用于底层通信调试。它会遍历菊花链上的所有IC，
 *          将每次SPI通信接收到的6个字节原始数据以十六进制格式打印出来，
 *          同时打印该IC对应的命令计数器(CC)值。
 *          命令计数器可以用来判断芯片是否正常接收并执行了通信命令（每次成功通信CC应递增）。
 */
void bms_printRawData(uint8_t data[TOTAL_IC][DATA_LEN], uint8_t cc[TOTAL_IC])
{
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        printfDma("IC%d: ", ic+1);
        for (int j = 0; j < 6; j++)             // 遍历接收到的每一个字节(6字节)
        {
            printfDma("0x%02X, ", data[ic][j]);    // 打印字节
        }
        printfDma("CC: %d |   ", cc[ic]); // 打印命令计数器
    }
    printfDma("\n\n");
}

/**
 * @brief 检查SPI通讯接收数据的PEC(包错误校验)是否正确，并标记通讯故障
 * 
 * @param data 接收到的原始字节数组 [IC索引][数据字节]
 * @param pec 接收到的PEC校验码数组 [IC索引]
 * @param cc 命令计数器数组 [IC索引]
 * 
 * @return true  检测到通讯故障（PEC校验失败）
 * @return false 无通讯故障（PEC校验通过）
 * 
 * @details 该函数是BMS通信安全的重要保障。在菊花链SPI通信中，数据容易受电磁干扰导致错位或翻转。
 *          1. 调用底层 bms_checkRxPec 函数，根据接收到的原始数据重新计算PEC校验和，
 *             并与接收到的PEC进行比对。
 *          2. 如果校验失败，说明数据在传输过程中出错。
 *          3. 函数会通过串口打印警告信息，列出发生通讯错误的IC编号。
 *          4. 同时，底层校验函数会将错误状态写入全局公共结构体 ic_common.isCommsError 数组中，
 *             供上层逻辑（如CAN发送、均衡控制）判断该IC数据是否有效。
 *             
 * @todo 待办: 添加命令计数器(CC)故障检查（判断CC是否按预期递增）
 * @todo 待办: 添加PEC故障的专门处理程序（如多次失败后的重传或断路保护）
 */
bool bms_checkRxFault(uint8_t data[TOTAL_IC][DATA_LEN], uint16_t pec[TOTAL_IC], uint8_t cc[TOTAL_IC])
{
    bool faultDetected = false;
    bool* errorIndex = ic_common.isCommsError;

    if (bms_checkRxPec(data, pec, cc, errorIndex)) // 检查PEC校验
    {
        printfDma("警告! PEC错误 - IC:");
        for(int ic = 0; ic < TOTAL_IC; ic++)
        {
            if (errorIndex[ic])
            {
                printfDma(" %d,", ic+1);
            }
        }
        printfDma("\n");
        faultDetected = true;
    }

    return faultDetected;

    // 待办: 添加命令计数器(CC)故障检查
    // 待办: 添加PEC故障的处理程序
}


// 主要用于调试
BMS_StatusTypeDef bms_readRegister(RegisterTypes regType)
{
    char* title;
    uint8_t* cmd;
    switch (regType)
    {
    case REG_CONFIG_A:
        title = "配置A寄存器";
        cmd = RDCFGA;
        break;
    case REG_CONFIG_B:
        title = "配置B寄存器";
        cmd = RDCFGB;
        break;
    case REG_PWM_A:
        title = "PWM A寄存器";
        cmd = RDPWMA;
        break;
    case REG_PWM_B:
        title = "PWM B寄存器";
        cmd = RDPWMB;
        break;
    case REG_SID:
        title = "SID寄存器";
        cmd = RDSID;
        break;
    default:
        Error_Handler(); // 无效选择
        break;
    }

    bms_receiveData(cmd, rxData, rxPec, rxCc);
    if (bms_checkRxFault(rxData, rxPec, rxCc))
    {
        return BMS_ERR_COMMS;
    }

    printfDma("%s: \n", title);
    bms_printRawData(rxData, rxCc);

    return BMS_OK;
}

/**
 * @brief 启动6830芯片的连续主ADC转换，用于持续测量电芯电压
 * 
 * @param enableRedundant 是否使能冗余测量
 *                        - true: 开启冗余ADC测量（RD=1），用于安全校验
 *                        - false: 仅使用主ADC测量（RD=0），速度更快
 * 
 * @details 该函数通过配置全局命令结构体 ADCV，并发送给6830芯片来启动ADC转换。
 *          ADCV是6830专用的主ADC控制命令（对于2950芯片，此命令等效为ADI1）。
 * 
 *          配置参数说明：
 *          - CONT = 1: 设置为连续转换模式，芯片会自动循环进行ADC转换，无需反复发送命令
 *          - DCP  = 0: 允许在ADC转换期间进行放电（Discharge Permitted），不影响均衡
 *          - RSTF = 1: 复位内部数字滤波器，确保首次转换结果干净
 *          - OW   = 00: 禁止开路检测（Open Wire），加快转换速度
 * 
 *          关于冗余测量(RD)与放电(PWM)的互斥行为（当DCP=0时）：
 *          - RD=0 且 CONT=1：PWM放电不受影响（推荐模式）
 *          - RD=1 且 CONT=0：PWM放电会被临时中断，直到冗余转换完成（约8ms）
 *          - RD=1 且 CONT=1：PWM放电会被持续中断
 */
void bms_startAdcvCont(bool enableRedundant)
{
    // 6830行为说明
    // 当 DCP = 0 (放电允许位)时:
    // 如果 RD=0 且 CONT=1，PWM放电不受影响
    // 如果 RD=1 且 CONT=0，PWM放电临时中断直到冗余转换完成(典型8ms)
    // 如果 RD=1 且 CONT=1，PWM放电被中断

    ADCV.CONT = 1;      // 连续转换模式
    ADCV.DCP  = 0;      // 允许在ADC转换时放电
    ADCV.RSTF = 1;      // 复位滤波器
    ADCV.OW   = 0b00;   // 禁止C-ADC和S-ADC的开路检测

    ADCV.RD   = enableRedundant;      // 冗余测量使能

    // 2950行为 (ADI1命令)
    //

    bms_transmitCmd((uint8_t *)&ADCV); // 发送ADCV命令
}

/**
 * @brief 解析从6830芯片读取的电芯电压原始数据，转换为实际浮点电压值
 * 
 * @param rawData       SPI接收到的原始字节数组 [IC索引][数据字节]
 * @param vArr          存放解析后浮点电压的数组 [IC索引][电芯索引]
 * @param register_index 当前读取的寄存器组索引 (0~5)
 *                      6830将16节电芯的电压分6个寄存器组返回：
 *                      - 索引 0~4：每组包含3节电芯的数据
 *                      - 索引 5：最后一组只包含1节电芯的数据
 * 
 * @details 该函数将底层SPI通信返回的原始字节还原为工程值。
 *          - 6830每帧返回6字节数据，包含3个16位有符号的ADC结果。
 *          - 电压转换公式：实际电压 = 原始ADC值 * 0.00015V(LSB) + 1.5V(固定偏移)
 *          - 注意：该函数仅处理6830芯片的数据， rawData 中的 2950 数据部分被跳过。
 *          - 当 register_index == 5 时，由于最后一组只有1节电芯有效数据，
 *            解析完第一个后直接 break 跳出，防止越界读取或写入脏数据。
 */
void bms_parseVoltage(uint8_t rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_IC][TOTAL_CELL], uint8_t register_index)
{
    // 不处理2950
    // 待办: 也读取主控寄存器

    uint8_t cell_index = (register_index * 3); // 每个寄存器组包含3节电芯的数据

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = cell_index; c < (cell_index + 3); c++)
        {
            // 将2字节原始数据强转为int16_t，乘以LSB(0.00015V)加偏移(1.5V)
            vArr[ic][c] = *((int16_t *)(rawData[ic + TOTAL_AD29] + (c - cell_index)*2)) * 0.00015 + 1.5;

            if (register_index == 5) // 跳过最后一个寄存器组的后两个数据，因为最后一组只存1节电芯
            {
                break;
            }
        }
    }
}

/**
 * @brief 解析从6830芯片读取的辅助ADC数据（包括NTC温度传感器电压、芯片内部温度、模块总压）
 * 
 * @param rawData     SPI接收到的原始字节数组 [IC索引][数据字节]
 * @param vArr        存放解析后浮点电压的数组 [IC索引][电芯索引]，此处复用存放NTC传感器的电压
 * @param cell_index  当前读取的辅助寄存器组索引 (0~4)
 *                    6830的辅助寄存器分为5组(RDAUXA~D, RDSTATA)，每组包含3个通道的结果
 * @param muxIndex    当前外部多路复用器(MUX)的通道索引 (0 或 1)
 *                    因为电芯数量多于GPIO引脚，硬件上使用MUX扩展了温度传感器，
 *                    同一个物理引脚在不同MUX通道下对应不同的温度传感器
 * 
 * @details 该函数处理6830辅助ADC返回的复杂数据结构，特殊逻辑如下：
 *          1. cell_index == 4：读取的是状态寄存器A(RDSTATA)，包含芯片内部温度传感器。
 *             通过特定公式 (电压 / 0.0075 - 273) 将其转换为摄氏温度存入 temp_ic。
 *          2. cell_index == 3：读取的是辅助寄存器D(RDAUXD)，其中包含了模块总压(v_segment)。
 *             提取对应字节并乘以分压系数25转换为实际电压。
 *          3. 跳过数字引脚：GPIO3和GPIO4被配置为数字输出，不具备ADC功能，
 *             因此在解析时必须跳过索引3和4，并将后续索引减2进行补偿。
 *          4. NTC电压存储：解析出的热敏电阻电压，按照 (内部补偿后的索引*2 + muxIndex) 
 *             的规则存入 vArr 数组，等待后续 convertCellTemp 函数转换为温度值。
 */
void bms_parseAuxVoltage(uint8_t const rawData[TOTAL_IC][DATA_LEN], float vArr[TOTAL_AD68][TOTAL_CELL], uint8_t cell_index, uint8_t muxIndex)
{
    // 不处理2950

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        if (cell_index == 4) // 第4组是状态寄存器，包含芯片内部温度
        {
            // 读取内部温度传感器电压，转换为温度值: (电压 / 0.0075) - 273
            ic_ad68.temp_ic[ic] = (*((int16_t *)(rawData[ic + TOTAL_AD29] + 2)) * 0.00015 + 1.5) / 0.0075 - 273;
            continue;
        }

        uint8_t cellArrIndex = cell_index*3;

        for (int c = cellArrIndex; c < (cellArrIndex + 3); c++)
        {
            if (c == 3 || c == 4) continue; // 跳过数字输出引脚(GPIO3, GPIO4)
            int ci = c;                     // 补偿跳过的数字引脚导致的索引偏移
            if (c > 4)
            {
                ci -= 2;
            }

            // 将解析出的电压存入温度数组(此时存的是NTC热敏电阻的电压)
            vArr[ic][ci*2 + muxIndex] = *((int16_t *)(rawData[ic + TOTAL_AD29] + (c-cellArrIndex)*2)) * 0.00015 + 1.5;

            if (cell_index == 3) // 第3组包含模块总压
            {
                // 读取第4、5字节，转换为模块总压(乘以分压系数25)
                ic_ad68.v_segment[ic] = (*((int16_t *)(rawData[ic + TOTAL_AD29] + 4)) * 0.00015 + 1.5) * 25;
                break;
            }
        }
    }
}

/**
 * @brief 计算指定电压类型的统计信息（最小值、最大值、总和、平均值、压差等）
 * 
 * @param voltageType 当前计算的电压类型枚举（如：瞬时电压、平均电压、滤波电压、冗余电压、温度传感器电压）
 * 
 * @details 该函数对采集到的电压数据进行二次加工，提取出对BMS控制策略至关重要的统计特征。
 *          计算分为两个层级：
 * 
 *          1. 【单芯片层级】：遍历单个6830芯片下的所有电芯（默认16节）
 *             - 找出该芯片管理的单体最低电压(min)和最高电压(max)
 *             - 计算单体电压总和、单体平均电压(sum / 16.0)
 *             - 计算该芯片内部的极差(delta = max - min)
 *             - 初步计算每节电芯与当前芯片最低电压的差值(v_cell_diff)
 * 
 *          2. 【电池包层级】：在遍历所有芯片的过程中同步进行
 *             - 找出整个电池包的绝对最低电压(pack_min)和绝对最高电压(pack_max)
 *             - 累加所有电芯电压得到电池包总压(total_voltage)
 *             - 如果当前计算的是用于均衡判断的电压类型(dischargeVoltageType)，
 *               则将包总压、包最高/最低值更新到全局公共结构体 ic_common 中
 * 
 *          3. 【全局压差修正】：
 *             均衡控制需要站在全局视角，因此最后再次遍历所有电芯，
 *             将 v_cell_diff 重新计算为每节电芯相对于【整个电池包最低电压】的差值，
 *             以确保跨芯片的均衡判断基准一致。
 */
void bms_calculateStats(VoltageTypes voltageType)
{
    float total_voltage = 0;
    float pack_min =  999.0;
    float pack_max = -999.0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        float min =  999.0;
        float max = -999.0;
        float sum = 0;

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            float voltage = ic_ad68.v_cell[voltageType][ic][c];
            sum += voltage;
            if (voltage > max)
            {
                max = voltage;
            }
            if (voltage < min)
            {
                min = voltage;
            }
            if (voltage > pack_max)
            {
                pack_max = voltage;
            }
            if (voltage < pack_min)
            {
                pack_min = voltage;
            }
        }

        total_voltage += sum;
        ic_ad68.v_cell_min  [voltageType][ic] = min;
        ic_ad68.v_cell_max  [voltageType][ic] = max;
        ic_ad68.v_cell_sum  [voltageType][ic] = sum;
        ic_ad68.v_cell_avg  [voltageType][ic] = sum / 16.0; // 16节电芯平均
        ic_ad68.v_cell_delta[voltageType][ic] = max - min;   // 单模块压差

        // 计算与当前IC最低电压的差值
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            ic_ad68.v_cell_diff[voltageType][ic][c] = ic_ad68.v_cell[voltageType][ic][c] - min;
        }
    }

    // 如果是用于均衡判断的电压类型，则更新整个包的统计信息
    if (voltageType == dischargeVoltageType)
    {
        ic_common.v_pack_total = total_voltage;
        ic_common.v_pack_min = pack_min;
        ic_common.v_pack_max = pack_max;
    }

    // 计算相对于整个电池包最低电压的压差(用于全局均衡判断)
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            ic_ad68.v_cell_diff[voltageType][ic][c] = ic_ad68.v_cell[voltageType][ic][c] - pack_min;
        }
    }
}

/**
 * @brief 通过串口DMA打印指定类型的电芯电压表格（主要用于调试）
 * 
 * @param voltageType 要打印的电压类型枚举
 *                    (VOLTAGE_C: 瞬时电压, VOLTAGE_C_AVG: 平均电压, 
 *                     VOLTAGE_C_FIL: 滤波电压, VOLTAGE_S: 冗余电压, 
 *                     VOLTAGE_TEMP: 温度传感器电压)
 */
void bms_printVoltage(VoltageTypes voltageType)
{
    float (*vArr)[TOTAL_CELL] = ic_ad68.v_cell[voltageType];
    float (*vDev)[TOTAL_CELL] = ic_ad68.v_cell_diff[voltageType];

    char* title;
    switch (voltageType)
    {
    case VOLTAGE_C:     title = "C 瞬时电压"; break;
    case VOLTAGE_C_AVG: title = "C 平均电压"; break;
    case VOLTAGE_C_FIL: title = "C 滤波电压"; break;
    case VOLTAGE_S:     title = "S 冗余电压"; break;
    case VOLTAGE_TEMP:  title = "温度传感器电压"; break;
    default: Error_Handler(); break;
    }
    printfDma("%s: \n", title);

    printfDma("| IC |");
    for (int i = 0; i < TOTAL_CELL; i++)
    {
        printfDma("   %2d   |", i+1);
    }
    printfDma("  Sum   |  Delta |\n");

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        uint8_t paddingOffset;
        paddingOffset = printfDma("| %2d |", ic+1);
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printfDma("%8.5f|", vArr[ic][c]);
        }

        printfDma("%8.5f|", ic_ad68.v_cell_sum  [voltageType][ic]);
        printfDma("%8.5f|", ic_ad68.v_cell_delta[voltageType][ic]);
        printfDma("\n");

        // 打印压差行
        printfDma("%*s", paddingOffset, "");
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printfDma("%8.5f|", vDev[ic][c]);
        }
        printfDma("\n");
    }
}

/**
 * @brief 通过串口打印所有电芯温度及芯片内部温度/模块电压的表格（调试用）
 * 
 * @details 遍历所有6830芯片，以表格形式打印每个IC下所有电芯的温度(℃)，
 *          并在行尾附带打印该芯片的内部温度和模块总压。
 */
void bms_printTemps(void)
{
    float (*tArr)[TOTAL_CELL] = ic_ad68.temp_cell;

    printfDma("| IC |");
    for (int i = 0; i < TOTAL_CELL; i++)
    {
        printfDma("  %2d   |", i+1);
    }
    printfDma("  芯片温度 / 模块电压");
    printfDma("\n");

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        printfDma("| %2d |", ic);
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            printfDma("%6.1f |", tArr[ic][c]);
        }
        printfDma("  %.2f C  /  %.2f V \n", ic_ad68.temp_ic[ic], ic_ad68.v_segment[ic]);
    }
}


// 各类型电压读取命令列表
uint8_t* readCellVoltageCmdList[TOTAL_VOLTAGE_TYPES][6] = {
        {RDCVA, RDCVB, RDCVC, RDCVD, RDCVE, RDCVF}, // 瞬时电压
        {RDACA, RDACB, RDACC, RDACD, RDACE, RDACF}, // 平均电压
        {RDFCA, RDFCB, RDFCC, RDFCD, RDFCE, RDFCF}, // 滤波电压
        {RDSVA, RDSVB, RDSVC, RDSVD, RDSVE, RDSVF}, // 冗余电压
        {} // 温度传感器电压(不在此处读取)
};

/**
 * @brief 读取所有6830芯片的指定类型电芯电压
 * 
 * @param voltageType 要读取的电压类型(瞬时/平均/滤波/冗余)
 * @return BMS_OK 读取成功，BMS_ERR_COMMS 通讯失败
 * 
 * @note 6830有16节电芯，分6次读取(每组3节，最后一组1节)。
 *       读取完成后自动计算统计值，若调试开关开启则打印数据。
 */
BMS_StatusTypeDef bms_readCellVoltage(VoltageTypes voltageType)
{
    uint8_t** cmdList = readCellVoltageCmdList[voltageType];

    for (int i = 0; i < 6; i++) // 分6次读取16节电芯
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }
        bms_parseVoltage(rxData, ic_ad68.v_cell[voltageType], i);
    }

    bms_calculateStats(voltageType); // 计算统计值

    if (DEBUG_SERIAL_VOLTAGE_ENABLED) bms_printVoltage(voltageType);

    return BMS_OK;
}

/**
 * @brief 读取6830的辅助ADC电压(温度传感器、模块电压等)
 * 
 * @param muxIndex 当前外部多路复用器通道索引(0或1)
 * @return 0 读取成功，-1 通讯失败
 * 
 * @note 分5次读取RDAUXA~D和RDSTATA寄存器，并调用解析函数提取温度和电压。
 */
uint8_t bms_getAuxVoltage(uint8_t muxIndex)
{
    uint8_t* cmdList[] = {RDAUXA, RDAUXB, RDAUXC, RDAUXD, RDSTATA};

    for (int i = 0; i < 5; i++) // 分5次读取辅助寄存器
    {
        bms_receiveData(cmdList[i], rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return -1;
        }
        bms_parseAuxVoltage(rxData, ic_ad68.v_cell[VOLTAGE_TEMP], i, muxIndex);
    }
    return 0;
}

/**
 * @brief 将NTC热敏电阻电压转换为摄氏温度（线性插值法）
 * 
 * @param cellVoltage 传感器电压值
 * @return 转换后的温度(℃)，若超量程返回888.0
 * 
 * @note 根据数据手册提供的温度-电压查找表，找到电压所在区间后进行线性插值计算。
 */
float static convertCellTemp(float cellVoltage)
{
    // 来自数据手册的NTC查找表
    static const float tempValues[]    = { -40,  -35,  -30,  -25,  -20,  -15,  -10,   -5,    0,    5,   10,   15,   20,   25,   30,   35,   40,   45,   50,   55,   60,   65,   70,   75,   80,   85,   90,   95,  100,  105,  110,  115,  120};
    static const float voltageValues[] = {2.44, 2.42, 2.40, 2.38, 2.35, 2.32, 2.27, 2.23, 2.17, 2.11, 2.05, 1.99, 1.92, 1.86, 1.80, 1.74, 1.68, 1.63, 1.59, 1.55, 1.51, 1.48, 1.45, 1.43, 1.40, 1.38, 1.37, 1.35, 1.34, 1.33, 1.32, 1.31, 1.30};
    static const int   numDataPoints   = sizeof(tempValues) / sizeof(tempValues[0]);

    // 检查是否在量程内
    if (cellVoltage > 2.44 || cellVoltage < 1.30)
    {
        // 电压超出范围，返回异常温度值
        return 888.0;
    }

    int idx;
    for (idx = 0; idx < numDataPoints - 1; idx++)
    {
        if (cellVoltage > voltageValues[idx + 1]) break; // 找到电压所在的区间
    }

    // 线性插值计算温度
    float x1 = voltageValues[idx];
    float x2 = voltageValues[idx + 1];
    float y1 = tempValues[idx];
    float y2 = tempValues[idx + 1];

    return y1 + (cellVoltage - x1) * (y2 - y1) / (x2 - x1);
}

/**
 * @brief 将辅助ADC测得的电压值转换为温度值
 * 
 * @note 遍历所有6830芯片和电芯，调用 convertCellTemp 将 VOLTAGE_TEMP 中的电压转换为摄氏度。
 */
void bms_parseTemps(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            // 将电压转换为温度存入温度数组
            ic_ad68.temp_cell[ic][c] = convertCellTemp(ic_ad68.v_cell[VOLTAGE_TEMP][ic][c]);
        }
    }
}

/**
 * @brief 获取辅助测量值(电芯温度、芯片内部温度、模块总压)
 * 
 * @return BMS_OK 测量成功，BMS_ERR_COMMS 通讯失败
 * 
 * @note 流程：
 *       1. 发送ADAX命令启动辅助ADC，轮询等待转换完成，读取MUX通道1的电压。
 *       2. 切换GPIO4/5改变外部MUX通道，延时1ms稳定。
 *       3. 再次启动ADC并读取MUX通道0的电压。
 *       4. 恢复GPIO默认状态，解析温度并计算统计值。
 */
BMS_StatusTypeDef bms_getAuxMeasurement(void)
{
    /*
     *  GPO4 = 多路复用器开关
     *  GPO5 = 3V3转换器使能
     *
     *  基于 TEC 2-4810WI 数据手册，启动时间为20ms
     */

    ADAX.OW   = 0b0;   // 禁止开路检测
    ADAX.CH   = 0b0000;// 测量所有通道
    ADAX.CH4  = 0b0;
    ADAX.PUP  = 0b0;   // 禁止内部上拉

    bms_transmitCmd((uint8_t *)&ADAX); // 发送辅助ADC转换命令
    bms_transmitPoll(PLAUX1);          // 轮询等待转换完成
    if (bms_getAuxVoltage(1))           // 必须先读MUX通道1，因为硬件切换逻辑
    {
        return BMS_ERR_COMMS;
    }
    bms68_setGpo45(0b00);           // 切换到另一个MUX通道
    bms_delayMsActive(1);           // 切换微小延时等待稳定

    bms_transmitCmd((uint8_t *)&ADAX); // 再次发送辅助ADC转换命令
    bms_transmitPoll(PLAUX1);
    if (bms_getAuxVoltage(0))           // 读取MUX通道0
    {
        return BMS_ERR_COMMS;
    }
    bms68_setGpo45(0b11);           // 恢复默认GPIO状态

    bms_parseTemps();                // 解析温度
    bms_calculateStats(VOLTAGE_TEMP);// 计算温度统计
    if (DEBUG_SERIAL_VOLTAGE_ENABLED)   bms_printVoltage(VOLTAGE_TEMP);
    if (DEBUG_SERIAL_AUX_ENABLED)      bms_printTemps();

    return BMS_OK;
}

/**
 * @brief 设置指定芯片、指定电芯的PWM均衡占空比
 * 
 * @param ic_index 芯片索引(0起始)
 * @param cell 电芯索引(0起始，函数内部会转为1起始)
 * @param dutyCycle PWM占空比(4位，0b0000~0b1111)
 * 
 * @note 1-12号电芯对应pwma寄存器，13-16号电芯对应pwmb寄存器。
 */
void bms_setPwm(uint8_t ic_index, uint8_t cell, uint8_t dutyCycle)
{
    cell++;                                 // 从0索引改为1索引

    switch (cell) {
        case 1:  ic_ad68.pwma[ic_index].pwm1 = dutyCycle;  break;   // 电芯1-12对应pwma寄存器
        case 2:  ic_ad68.pwma[ic_index].pwm2 = dutyCycle;  break;
        case 3:  ic_ad68.pwma[ic_index].pwm3 = dutyCycle;  break;
        case 4:  ic_ad68.pwma[ic_index].pwm4 = dutyCycle;  break;
        case 5:  ic_ad68.pwma[ic_index].pwm5 = dutyCycle;  break;
        case 6:  ic_ad68.pwma[ic_index].pwm6 = dutyCycle;  break;
        case 7:  ic_ad68.pwma[ic_index].pwm7 = dutyCycle;  break;
        case 8:  ic_ad68.pwma[ic_index].pwm8 = dutyCycle;  break;
        case 9:  ic_ad68.pwma[ic_index].pwm9 = dutyCycle;  break;
        case 10: ic_ad68.pwma[ic_index].pwm10 = dutyCycle; break;
        case 11: ic_ad68.pwma[ic_index].pwm11 = dutyCycle; break;
        case 12: ic_ad68.pwma[ic_index].pwm12 = dutyCycle; break;
        case 13: ic_ad68.pwmb[ic_index].pwm13 = dutyCycle; break;
        case 14: ic_ad68.pwmb[ic_index].pwm14 = dutyCycle; break;
        case 15: ic_ad68.pwmb[ic_index].pwm15 = dutyCycle; break;
        case 16: ic_ad68.pwmb[ic_index].pwm16 = dutyCycle; break;
        default: break; // 处理无效情况
    }
}


/*
 * 将压差阈值(所有电芯电压之间的最大差值阈值)
 * 转换为
 * 放电阈值(电芯停止均衡放电的目标电压)
 */
float bms_calculateBalancing(float deltaThreshold)
{
    float min = ic_common.v_pack_min; // 包内最低电压
    float max = ic_common.v_pack_max; // 包内最高电压

    if (max - min > deltaThreshold) // 如果压差超过阈值
    {
        return min + deltaThreshold; // 放电目标 = 最低电压 + 阈值
    }
    else
    {
        return 1000;  // 不需要均衡，将电压阈值设得极高以禁止放电
    }
}

/**
 * @brief 启动被动均衡放电
 * 
 * @param dischargeThreshold 均衡放电的目标停止电压
 * 
 * @note 核心均衡逻辑：
 *       1. 统计每个IC需要均衡的电芯数量。
 *       2. 动态分配PWM占空比：同时放电的电芯越多，单个电芯占空比越小，防止芯片内部过热。
 *       3. 电压高于阈值的电芯设置PWM并标记放电标志；低于阈值的关闭PWM。
 *       4. 设置放电超时定时器(DCTO=1分钟)。
 *       5. 将配置写入CONFIG_B、PWM_A、PWM_B寄存器生效。
 */
void bms_startDischarge(float dischargeThreshold)
{
    uint32_t cellDischargeCount;                    // 记录每个模块有多少节电芯需要放电
    uint8_t dutyCycle = 0;                          // 4位PWM占空比(周期937ms)
    const uint8_t maxDischarge = (0b0111 * 16);    // 所有的电芯都以一半占空比放电时的总放电量限制

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        cellDischargeCount = 0;

        // 统计当前IC需要均衡的电芯数
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                cellDischargeCount++;
            }
        }

        if (cellDischargeCount == 0) {cellDischargeCount = 1; }; // 防止除0

        // 动态分配占空比：同时放电的电芯越多，单个电芯占空比越小，防止芯片过热
        dutyCycle = maxDischarge / cellDischargeCount;
        if (dutyCycle > 0b1111) // 限制最大占空比
        {
            dutyCycle = 0b1111;
        }

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            if (ic_ad68.v_cell[dischargeVoltageType][ic][c] > dischargeThreshold)
            {
                printfDma("放电均衡: IC %d, 电芯 %d, 占空比 %d \n", ic+1, c+1, dutyCycle);
                BIT_SET(ic_ad68.isDischarging[ic], c); // 置位放电标志
                bms_setPwm(ic, c, dutyCycle);          // 设置PWM
            }
            else
            {
                BIT_CLEAR(ic_ad68.isDischarging[ic], c); // 清除放电标志
                bms_setPwm(ic, c, 0b0000);               // 关闭该电芯PWM放电
            }
        }

        // PWM放电功能在待机、基准上电、扩展均衡和测量状态下可用
        // 并且在放电超时未过期时(DCTO ≠ 0)
        ic_ad68.cfb_Tx[ic].dcto = 1;     // 放电计时器1分钟 (DTRNG = 0)
        ic_ad68.cfb_Tx[ic].dtmen = 0;    // 禁用放电计时器监控(DTM)
    }

    bms_writeRegister(REG_CONFIG_B);   // 发送DCTO计时器配置
    bms_writeRegister(REG_PWM_A);      // 发送PWM A配置
    bms_writeRegister(REG_PWM_B);      // 发送PWM B配置
}

/**
 * @brief 停止所有电芯的被动均衡放电
 * 
 * @note 遍历所有6830芯片和电芯，将PWM占空比设为0，清除放电标志，
 *       将放电超时定时器清零，并写入寄存器生效。
 */
void bms_stopDischarge(void)
{
    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            bms_setPwm(ic, c, 0b0000);    // 关闭该电芯PWM放电
            BIT_CLEAR(ic_ad68.isDischarging[ic], c); // 清除放电标志
        }

        ic_ad68.cfb_Tx[ic].dcto = 0;     // 关闭放电计时器
        ic_ad68.cfb_Tx[ic].dtmen = 0;    // 禁用放电计时器监控
    }
    bms_writeRegister(REG_CONFIG_B);   // 发送DCTO配置
    bms_writeRegister(REG_PWM_A);      // 发送PWM配置
    bms_writeRegister(REG_PWM_B);      // 发送PWM配置
}

/**
 * @brief 对菊花链上的所有BMS芯片执行软件复位
 * 
 * @note 唤醒菊花链后发送SRST命令，芯片将重启恢复初始状态。
 */
void bms_softReset(void)
{
    bms_wakeupChain();
    bms_transmitCmd(SRST);      // 发送软件复位命令
    printfDma("\n  ---  软件复位  ---  \n");
}

/**
 * @brief 读取2950芯片测量的电池包总压
 * 
 * @return BMS_OK 读取成功，BMS_ERR_COMMS 通讯失败
 * 
 * @note 从RDVB寄存器读取两路高压测量值，分别乘以各自的LSB和分压系数转换为实际电压。
 */
BMS_StatusTypeDef bms29_readVB(void)
{
    if (TOTAL_AD29)
    {
        bms_receiveData(RDVB, rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }

        // 解析总压: 原始值 * LSB * 分压系数
        ic_ad29.vb1 = *((int16_t *)(rxData[0] + 2)) *  0.000100 * 396.604395604;
        ic_ad29.vb2 = *((int16_t *)(rxData[0] + 4)) * -0.000085 * 751;
        if (DEBUG_SERIAL_MASTER_MEASUREMENTS)
        {
            printfDma("包总压: %fV, %fV  \n", ic_ad29.vb1, ic_ad29.vb2);
        }
    }
    else
    {
        if (DEBUG_SERIAL_MASTER_MEASUREMENTS)
        {
            printfDma("包总压: (AD29未启用!) \n");
        }
    }
    return BMS_OK;
}


/**
 * @brief 读取2950芯片测量的电池包电流
 * 
 * @return BMS_OK 读取成功，BMS_ERR_COMMS 通讯失败
 * 
 * @note 从RDI寄存器读取两路24位有符号原始数据，进行符号扩展后，
 *       除以1000000转换为伏特，再除以50微欧的采样电阻得到安培。
 *       两路电流符号相反，说明传感器方向或接线不同。
 */
BMS_StatusTypeDef bms29_readCurrent(void)
{
    if (TOTAL_AD29)
    {
        bms_receiveData(RDI, rxData, rxPec, rxCc);
        if (bms_checkRxFault(rxData, rxPec, rxCc))
        {
            return BMS_ERR_COMMS;
        }

        // 微伏
        int32_t i1v = 0;
        int32_t i2v = 0;

        // 拼接3字节原始数据
        i1v = ((uint32_t)rxData[0][0]) | ((uint32_t)rxData[0][1] << 8) | ((int32_t)rxData[0][2] << 16);
        i2v = ((uint32_t)rxData[0][3]) | ((uint32_t)rxData[0][4] << 8) | ((int32_t)rxData[0][5] << 16);

        // 24位有符号数扩展: 检查第24位(符号位)并扩展符号
        if (i1v & (UINT32_C(1) << 23)) { i1v |= 0xFF000000; };
        if (i2v & (UINT32_C(1) << 23)) { i2v |= 0xFF000000; };

        const float SHUNT_RESISTANCE = 0.000050; // 50微欧的采样电阻

        // 电流 = (微伏 / 1000000转换为伏特) / 采样电阻
        // 注意两路电流符号相反，说明传感器方向或接线不同
        ic_ad29.current1 = ((float)i1v / -1000000.0f) / SHUNT_RESISTANCE;
        ic_ad29.current2 = ((float)i2v /  1000000.0f) / SHUNT_RESISTANCE;
        if (DEBUG_SERIAL_MASTER_MEASUREMENTS)
        {
            printfDma("电流: %fA, %fA  \n", ic_ad29.current1 , ic_ad29.current2);
        }
    }
    else
    {
        if (DEBUG_SERIAL_MASTER_MEASUREMENTS)
        {
            printfDma("电流: (AD29未启用!) \n");
        }
    }
    return BMS_OK;
}


/**
 * @brief 在均衡放电期间测量电芯电压
 * 
 * @return BMS_OK 读取成功，BMS_ERR_COMMS 通讯失败
 * 
 * @note 6830的ADSV命令触发单次冗余S转换，此过程会临时暂停PWM放电，
 *       而主C-ADC不受影响。等待S转换完成后读取电压，确保均衡期间读数准确。
 */
BMS_StatusTypeDef bms_balancingMeasureVoltage(void)
{
    // 6830
    // ADSV 用于触发单次冗余S转换(会暂停PWM)，而C主ADC不受影响
    // 所以这会停止PWM并等待S转换完成
    // 然后读取S电压
    // 潜在改进: S与C的ADC比较

    ADSV.CONT = 0;      // 单次转换
    ADSV.DCP  = 0;      // 允许放电
    ADSV.OW   = 0b00;   // 禁止开路检测

    bms_transmitCmd((uint8_t *)&ADSV); // 发送ADSV命令

    bms_transmitPoll(PLSADC);          // 轮询S-ADC
    if (bms_readCellVoltage(dischargeVoltageType)) // 读取用于均衡判断的电压
    {
        return BMS_ERR_COMMS;
    }

    return BMS_OK;
}

/**
 * @brief 启动电池包被动均衡
 * 
 * @param deltaThreshold 允许的最大电芯压差
 * 
 * @note 先计算停止放电的电压阈值，再调用 bms_startDischarge 执行实际的放电操作。
 */
void bms_startBalancing(float deltaThreshold)
{
    float dischargeThreshold = bms_calculateBalancing(deltaThreshold); // 计算停止放电的电压线
    bms_startDischarge(dischargeThreshold); // 启动放电
}

/**
 * @brief 将BMS采集的数据打包成CAN报文存入发送缓冲区
 * 
 * @param buff 指向CAN报文缓冲区指针的指针（输出参数）
 * @param len  缓冲区内报文的数量（输出参数）
 * 
 * @note 报文分配规则：
 *       1. 每个模块(IC)发一条状态报文(包含模块电压、芯片温度、故障标志)。
 *       2. 如果通讯正常，每节电芯发一条单体报文(包含电压、压差、温度、均衡/故障标志)。
 *       3. 2950主控发一条报文(包含包总压、总电流，总压使用6830累加值覆盖)。
 *       4. 最后追加一条充电机配置报文。
 *       所有数值发送时放大了倍数(电压*1000，温度*100)以避免浮点传输。
 */
void BMS_GetCanData(CanTxMsg** buff, uint32_t* len)
{
    uint32_t bufferlen = 0;
    uint32_t id = BASE_CAN_ID;
    FDCAN_TxHeaderTypeDef txHeader;

    /* 准备CAN发送头 */
    txHeader.Identifier = id;
    txHeader.IdType = FDCAN_EXTENDED_ID;   // 扩展帧
    txHeader.TxFrameType = FDCAN_DATA_FRAME;// 数据帧
    txHeader.DataLength = 8;               // 8字节数据
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF; // 不可变波特率
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 经典CAN格式
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        int32_t v_segment       = ic_ad68.v_segment[ic] * 1000; // 模块电压放大1000倍发送
        int16_t temp_ic         = ic_ad68.temp_ic[ic]   * 100;  // 芯片温度放大100倍发送
        uint8_t isCommsError    = ic_common.isCommsError[ic+TOTAL_AD29];
        uint8_t isFaultDetected = ic_common.isFaultDetected[ic+TOTAL_AD29];

        memcpy(&canTxBuffer[bufferlen].data[0], &v_segment, 4);
        memcpy(&canTxBuffer[bufferlen].data[4], &temp_ic, 2);
        canTxBuffer[bufferlen].data[6] = (uint8_t)((isCommsError << 0) | (isFaultDetected << 1));

        txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL + ic; // 模块状态报文ID
        canTxBuffer[bufferlen].header = txHeader;
        bufferlen++;

        if (isCommsError)
        {
            continue; // 通讯错误则不发送该模块的电芯电压和温度(数据无效)
        }

        for (int c = 0; c < TOTAL_CELL; c++)
        {
            int16_t cellVoltage = (int16_t)(ic_ad68.v_cell[dischargeVoltageType][ic][c] * 1000); // 放大1000倍
            int16_t voltageDiff = (int16_t)(ic_ad68.v_cell_diff[dischargeVoltageType][ic][c] * 1000);
            int16_t cellTemp    = (int16_t)(ic_ad68.temp_cell[ic][c] * 100); // 放大100倍
            uint8_t isDischarging       = ((ic_ad68.isDischarging[ic]       >> c) & 0x01U);
            uint8_t isCellFaultDetected = ((ic_ad68.isCellFaultDetected[ic] >> c) & 0x01U);

            canTxBuffer[bufferlen].data[0] = (uint8_t)(cellVoltage & 0xFF);
            canTxBuffer[bufferlen].data[1] = (uint8_t)((cellVoltage >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[2] = (uint8_t)(voltageDiff & 0xFF);
            canTxBuffer[bufferlen].data[3] = (uint8_t)((voltageDiff >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[4] = (uint8_t)(cellTemp & 0xFF);
            canTxBuffer[bufferlen].data[5] = (uint8_t)((cellTemp >> 8) & 0xFF);
            canTxBuffer[bufferlen].data[6] = (uint8_t)((isDischarging << 0) | (isCellFaultDetected << 1));

            uint32_t id_cell_offset = ic*TOTAL_CELL + c;

            txHeader.Identifier = id + id_cell_offset; // 单体电压报文ID
            canTxBuffer[bufferlen].header = txHeader;
            bufferlen++;
        }
    }

    if (TOTAL_AD29)
    {
        uint8_t isCommsError    = ic_common.isCommsError[0];
        uint8_t isFaultDetected = ic_common.isFaultDetected[0];

        canTxBuffer[bufferlen].data[0] = (uint8_t)((isCommsError << 0) | (isFaultDetected << 1));
        txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL + 7 + 1; // 主控状态报文ID
        canTxBuffer[bufferlen].header = txHeader;
        bufferlen++;

        if (!isCommsError)
        {
            int16_t packVoltage     = (ic_ad29.vb1 + ic_ad29.vb2) * 10 / 2;
            int16_t packCurrent     = (int16_t)(((ic_ad29.current1 + ic_ad29.current2) * 100.0f) / 2.0f);

            packVoltage = ic_common.v_pack_total * 10; // 使用从6830累加的总压覆盖2950测量的总压

            canTxBuffer[bufferlen].data[0] = (packVoltage >> 0)  & 0xFF;
            canTxBuffer[bufferlen].data[1] = (packVoltage >> 8)  & 0xFF;

            canTxBuffer[bufferlen].data[2] = (packCurrent >> 0)  & 0xFF;
            canTxBuffer[bufferlen].data[3] = (packCurrent >> 8)  & 0xFF;

            txHeader.Identifier = BASE_CAN_ID + 7*TOTAL_CELL + 7; // 包总压电流报文ID
            canTxBuffer[bufferlen].header = txHeader;
            bufferlen++;
        }
    }

    // --- 充电机配置CAN报文 --- //
    BMS_CAN_GetChargerMsg(&chargerConfig, canTxBuffer[bufferlen].data);
    txHeader.Identifier = CHARGER_CONFIG_CAN_ID;
    canTxBuffer[bufferlen].header = txHeader;
    bufferlen++;

    *len = bufferlen;
    *buff = canTxBuffer;
}

/**
 * @brief 检查是否存在温度故障
 * @return 非零值表示存在过温故障，0表示正常
 */
BMS_StatusTypeDef BMS_CheckTemps(void)
{
    return (BMS_StatusFlags & BMS_ERR_TEMP); // 检查是否有过温故障
}

/**
 * @brief 检查是否存在电压故障
 * @return 非零值表示存在过压故障，0表示正常
 */
BMS_StatusTypeDef BMS_CheckVoltage(void)
{
    return (BMS_StatusFlags & BMS_ERR_VOLTAGE); // 检查是否有电压故障
}

/**
 * @brief 检查是否存在电流故障
 * @return 非零值表示存在电流故障，0表示正常
 */
BMS_StatusTypeDef BMS_CheckCurrent(void)
{
    return (BMS_StatusFlags & BMS_ERR_CURRENT); // 检查是否有电流故障
}

/**
 * @brief 检查是否存在通讯故障
 * @return 非零值表示存在通讯故障，0表示正常
 */
BMS_StatusTypeDef BMS_CheckCommsFault(void)
{
    return (BMS_StatusFlags & BMS_ERR_COMMS); // 检查是否有通讯故障
}

/**
 * @brief 设置通讯故障
 * @param state 0表示正常，非零表示通讯故障
 */
void BMS_SetCommsFault(bool state)
{
    if (state)
        BMS_StatusFlags |= BMS_ERR_COMMS;   // 置位通讯故障
    else
        BMS_StatusFlags &= ~BMS_ERR_COMMS;  // 清除通讯故障
}

/**
 * @brief 更新BMS系统的所有故障诊断标志
 * 
 * @return 汇总后的系统状态(包含所有发生的故障位)
 * 
 * @note 依次检查：
 *       1. 2950总压/总流是否越限。
 *       2. 6830每节电芯电压/温度是否越限。
 *       3. 6830每个模块总压/芯片内部温度是否越限。
 *       如果越限则打印故障信息并置位对应故障标志，否则清除标志。
 */
BMS_StatusTypeDef BMS_UpdateStatusFlags(void)
{
    // 电池包保护阈值 (推测为7并16串，共112节电芯)
    const float MAX_PACK_VOLTAGE = 4.2 * 16 * 7; // 最高包总压
    const float MIN_PACK_VOLTAGE = 3.0 * 16 * 7; // 最低包总压

    const float MAX_CURRENT = 10.0;   // 最大允许电流
    const float MIN_CURRENT = -MAX_CURRENT; // 最小允许电流(充电)

    const float MAX_VOLTAGE = 4.2;    // 单体最高电压
    const float MIN_VOLTAGE = 3.3;    // 单体最低电压

    const float MAX_IC_VOLTAGE = 4.2 * 16; // 单模块最高电压
    const float MIN_IC_VOLTAGE = 3.0 * 16; // 单模块最低电压

    const float MAX_TEMP = 60;        // 电芯最高温度
    const float MIN_TEMP = 0;         // 电芯最低温度

    const float MAX_IC_TEMP = 70;     // 芯片最高温度
    const float MIN_IC_TEMP = 0;      // 芯片最低温度

    BMS_StatusTypeDef status = BMS_OK;
    BMS_StatusTypeDef returnStatus = BMS_OK;

    if (TOTAL_AD29)
    {
        float packVoltage = ic_common.v_pack_total; // 使用累加总压
        float packCurrent = ic_ad29.current1;

        if (packVoltage > MAX_PACK_VOLTAGE || packVoltage < MIN_PACK_VOLTAGE)
        {
            printfDma("包总压故障: %f V \n", packVoltage);
            ic_common.isFaultDetected[0] = true;
            status |= BMS_ERR_VOLTAGE;
        }

        if (packCurrent > MAX_CURRENT || packCurrent < MIN_CURRENT)
        {
            printfDma("包电流故障: %f A \n", packCurrent);
            ic_common.isFaultDetected[0] = true;
            status |= BMS_ERR_CURRENT;
        }

        if (status == BMS_OK)
        {
            ic_common.isFaultDetected[0] = false;
        }

        returnStatus |= status;
        status = BMS_OK;
    }

    for (int ic = 0; ic < TOTAL_AD68; ic++)
    {
        for (int c = 0; c < TOTAL_CELL; c++)
        {
            float cellVoltage = ic_ad68.v_cell[dischargeVoltageType][ic][c];
            float cellTemp = ic_ad68.temp_cell[ic][c];

            if (cellVoltage > MAX_VOLTAGE || cellVoltage < MIN_VOLTAGE)
            {
                printfDma("单体电压故障: 模块 %d, 电芯 %d, %f V \n", ic+1, c+1, cellVoltage);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status |= BMS_ERR_VOLTAGE;
            }

            if (cellTemp > MAX_TEMP || cellTemp < MIN_TEMP)
            {
                printfDma("电芯温度故障: 模块 %d, 电芯 %d, %f C \n", ic+1, c+1, cellTemp);
                BIT_SET(ic_ad68.isCellFaultDetected[ic], c);
                status |= BMS_ERR_TEMP;
            }

            if (status == BMS_OK)
            {
                BIT_CLEAR(ic_ad68.isCellFaultDetected[ic], c);
            }

            returnStatus |= status;
            status = BMS_OK;
        }

        float icVoltage = ic_ad68.v_segment[ic];
        float icTemp = ic_ad68.temp_ic[ic];

        if (icVoltage > MAX_IC_VOLTAGE || icVoltage < MIN_IC_VOLTAGE)
        {
            printfDma("模块电压故障: 模块 %d, %f V \n", ic+1, icVoltage);
            ic_common.isFaultDetected[ic + TOTAL_AD29] = true;
            status |= BMS_ERR_VOLTAGE;
        }

        if (icTemp > MAX_IC_TEMP || icTemp < MIN_IC_TEMP)
        {
            printfDma("芯片温度故障: 模块 %d, %f C \n", ic+1, icTemp);
            ic_common.isFaultDetected[ic + TOTAL_AD29] = true;
            status |= BMS_ERR_TEMP;
        }

        if (status == BMS_OK)
        {
            ic_common.isFaultDetected[ic + TOTAL_AD29] = false;
        }

        returnStatus |= status;
        status = BMS_OK;
    }

    BMS_StatusFlags = returnStatus; // 更新全局故障标志
    return returnStatus;
}


/**
 * @brief BMS系统的主程序循环体
 * 
 * @return 当前系统状态(包含故障标志位)
 * 
 * @note 周期性调用的主任务，依次执行：
 *       1. 唤醒菊花链。
 *       2. 读取辅助测量(温度等，约40ms)。
 *       3. 读取2950总压和电流。
 *       4. 读取用于均衡判断的电芯电压(此过程会短暂中断PWM)。
 *       5. 更新故障诊断标志。
 *       6. 如果系统无故障且使能了均衡，则启动被动均衡。
 */
BMS_StatusTypeDef BMS_ProgramLoop(void)
{
    BMS_StatusTypeDef status;
    bms_wakeupChain(); // 唤醒菊花链
    if ((status = bms_getAuxMeasurement())) return status; // 获取温度等辅助测量(约40ms)

    bms_wakeupChain();
    if ((status = bms29_readVB()))      return status; // 读总压
    if ((status = bms29_readCurrent())) return status; // 读电流
    if ((status = bms_balancingMeasureVoltage()))       return status; // 读取用于均衡的电压(会暂停PWM)

    // 只有状态正常才允许均衡/充电
    status = BMS_UpdateStatusFlags(); // 更新故障诊断

    if (enableBalancing && (status == BMS_OK)) // 如果一切正常且使能了均衡
    {
        bms_wakeupChain();
        bms_startBalancing(balancingThreshold); // 启动均衡
    }

    return status;
}




bool BMS_IsCharging(void)
{
    return !chargerConfig.disable_charging; // 充电状态取反(因为配置项是禁止标志)
}


void BMS_EnableCharging(bool enabled)
{
    chargerConfig.disable_charging = !enabled;   // 反转逻辑，因为配置项是disable_charging

    char *state = (chargerConfig.disable_charging)? "已禁用" : "已使能";
    printfDma("充电机状态: %s\n", state);
}

void BMS_ToggleBalancing(void)
{
    enableBalancing = !enableBalancing; // 切换均衡使能标志
}


void BMS_EnableBalancing(bool enabled)
{
    enableBalancing = enabled; // 设置均衡使能标志
}


void BMS_ChargingButtonLogic(void)
{
    // 检查充电机状态并使能充电
    // 或者如果已经在充电则禁用充电机

    bool chargerEnabled = !chargerConfig.disable_charging;

    if (chargerEnabled)
    {
        BMS_EnableCharging(false); // 如果已在充电，则关闭
        return;
    }

    bool statusOK = true;
    if (!HAL_GPIO_ReadPin(SDC_IN_GPIO_Port, SDC_IN_Pin)) statusOK = false;  // SDC未连接(高压未闭合)
    if (chargerStatus.output_voltage < 300.0f)      statusOK = false; // 充电机输出电压不足300V

    if (statusOK)
    {
        BMS_EnableCharging(true); // 满足条件，开启充电
    }
    else
    {
        printfDma("充电机状态异常，无法启动充电 \n");
    }
}




void BMS_WriteFaultSignal(bool state)
{
    static bool currState = 1;
    if (currState != state)
    {
        char *stateStr = (state)? "开启" : "关闭";
        printfDma("故障信号更新: %s\n", stateStr);

        HAL_GPIO_WritePin(FAULT_CTRL_GPIO_Port, FAULT_CTRL_Pin, state); // 控制MOSFET，如果开启则表示存在故障
        currState = state;
    }
}
