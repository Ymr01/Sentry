// 此文件是不断地把 BMI088 的原始数据，解算成这些角度和角速度，写进去
// struct {
//     fp32 q[4];       // 当前姿态四元数 [w,x,y,z]
//     fp32 yaw;        // 当前航向角 (度)
//     fp32 pitch;      // 当前俯仰角 (度)
//     fp32 roll;       // 当前横滚角 (度)
//     fp32 yaw_v;      // 航向角速度 (度/s)
//     fp32 pitch_v;    // 俯仰角速度 (度/s)
// } gimbal;

#include "sensor_task.h"
#include "../Components/BMI088/BMI088driver.h"
#include "../Application/robot_global.h"
#include "../Algorithm/MahonyAHRS/MahonyAHRS.h"
#include "math.h"
#include "cmsis_os.h"

// 姿态解算中间变量
static fp32 INS_q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static fp32 ins_angle[3];
static fp32 gyro[3], accel[3], temp;

// 零偏抑制相关变量
static fp32 gyro_bias[3] = {0.0f, 0.0f, 0.0f}; // 陀螺仪零偏
static uint16_t cali_count = 0;
#define CALI_SAMPLES 1000                       // 校准采样次数 (500ms)

void sensor_task_func(void const * argument) {

    // 1. 硬件初始化
    while (BMI088_init() != 0) {
        vTaskDelay(pdMS_TO_TICKS(10));//10ms后再试一次
    }

    // 2. 静态零偏校准 (核心逻辑：抑制0漂)
    // 提醒：此阶段机器人必须保持静止
    for (cali_count = 0; cali_count < CALI_SAMPLES; cali_count++) {
        BMI088_read(gyro, accel, &temp);
        gyro_bias[0] += gyro[0];
        gyro_bias[1] += gyro[1];
        gyro_bias[2] += gyro[2];
        vTaskDelay(1);
    }
    gyro_bias[0] /= CALI_SAMPLES;
    gyro_bias[1] /= CALI_SAMPLES;
    gyro_bias[2] /= CALI_SAMPLES;

    // 3. 标记传感器就绪
    robot_ctrl.monitor.sensor_ready = 1;//其他任务都在等这个

    while (1) {
        // 4.每次读取传感器数据都要减去零偏 (实现0漂抑制)
        BMI088_read(gyro, accel, &temp);
        gyro[0] -= gyro_bias[0];
        gyro[1] -= gyro_bias[1];
        gyro[2] -= gyro_bias[2];

        // 5. 更新四元数 (Mahony 算法)
        // 注意：传参时将 deg/s 转换为 rad/s (0.01745f)
        MahonyAHRSupdateIMU(INS_q,
                            gyro[0],
                            gyro[1],
                            gyro[2],
                            accel[0],
                            accel[1],
                            accel[2]);

        // 6. 姿态角结算 (欧拉角)
        ins_angle[0] = atan2f(2.0f*(INS_q[0]*INS_q[3]+INS_q[1]*INS_q[2]), 2.0f*(INS_q[0]*INS_q[0]+INS_q[1]*INS_q[1])-1.0f); // Yaw
        ins_angle[1] = asinf(-2.0f*(INS_q[1]*INS_q[3]-INS_q[0]*INS_q[2]));                                             // Roll
        ins_angle[2] = atan2f(2.0f*(INS_q[0]*INS_q[1]+INS_q[2]*INS_q[3]), 2.0f*(INS_q[0]*INS_q[0]+INS_q[3]*INS_q[3])-1.0f); // Pitch

        // 7. 同步到全局变量 (弧度)
        robot_ctrl.gimbal.q[0] = INS_q[0];
        robot_ctrl.gimbal.q[1] = INS_q[1];
        robot_ctrl.gimbal.q[2] = INS_q[2];
        robot_ctrl.gimbal.q[3] = INS_q[3];
        robot_ctrl.gimbal.yaw   = ins_angle[0];
        robot_ctrl.gimbal.pitch = ins_angle[2];
        robot_ctrl.gimbal.roll  = ins_angle[1];

        // 同步角速度反馈 (直接给 PID 使用经过消偏后的数据)
        robot_ctrl.gimbal.yaw_v   = gyro[2];
        robot_ctrl.gimbal.pitch_v = gyro[1];


        vTaskDelay(1);
    }
}