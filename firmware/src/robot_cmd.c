#include "robot.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>
#include "robot_cmd.h"
#include "Emm_V5.h"
#include "usb_stream.h"
#include "can.h"

/* LeRobot 适配新增命令 (forward declarations) */
static int robot_get_state_handle(float *param);
static int robot_set_joints_handle(float *param);
static int robot_set_torque_handle(float *param);
static int robot_e_stop_handle(float *param);

static int robot_soft_reset_handle(float *param);
static int robot_rel_rotate_handle(float *param);
static int robot_auto_handle(float *param);
static int robot_abs_rotate_handle(float *param);
static int stream_start_handle(float *param);
static int stream_stop_handle(float *param);

void robot_mqtt_handle(struct robot_cmd *cmd)
{
	float param[6] = {0};
	int strlen = 0;
	int type = 0;

	LOG("robot mqtt cmd: %s\n", cmd->cmd);

	// [MCU][TYPE][ARG0-5]
	int result = sscanf(cmd->cmd, "+MQTTSUBRECV:0,\"arm/change\",%d,[MCU][%d][%f %f %f %f %f %f]", &strlen, &type,
			&param[0], &param[1], &param[2],
			&param[3], &param[4], &param[5]);

	if (result < 8) { // 解析失败
		return;
	}

	switch (type)
	{
		case ROBOT_JOINT_ABS_ROTATE:
			robot_abs_rotate_handle(param);
			break;

		case ROBOT_AUTO_EVENT:
			robot_auto_handle(param);
			break;

		case ROBOT_JOINTS_SYNC_EVENT:
			robot_auto_handle(param);
			break;

		default:
			break;
	}
}

static int robot_remote_enable_handle(float *param)
{
	robot_soft_reset_handle(param);	/* 复位 */
	ROBOT_STATUS_SET(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	return robot_send_remote_event();
}

static int robot_remote_disable_handle(float *param)
{
	(void)param;
	ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	robot_soft_reset_handle(param);	/* 复位 */
	return pdPASS;
}

static int robot_rel_rotate_handle(float *param)
{
	uint32_t joint_id = (uint32_t)param[0];
	return robot_send_rel_rotate_event(joint_id, param[1]);
}

static int robot_abs_rotate_handle(float *param)
{
	uint32_t joint_id = (uint32_t)param[0];
	return robot_send_abs_rotate_event(joint_id, param[1]);
}

static int robot_auto_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}

static int robot_joints_sync_handle(float *param)
{
	return robot_send_auto_event((struct position *)param);
}

static int robot_hard_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(true);
}

static int robot_soft_reset_handle(float *param)
{
	(void)param;
	return robot_send_reset_event(false);
}

static int robot_time_func_handle(float *param)
{
	return robot_send_time_func_event(param[0] * 1000);
}

static int robot_remote_event_handle(float *param)
{
	if (!ROBOT_STATUS_IS(g_robot.status, ROBOT_STATUS_RMODE_ENABLE)) {
		return pdPASS;
	}

	float vx = -param[0] * ROBOT_REMOTE_MAX_VELOCITY;
	float vy = param[1] * ROBOT_REMOTE_MAX_VELOCITY;
	float vz = (param[4] - param[5]) / 2 * ROBOT_REMOTE_MAX_VELOCITY;
	float rx = -param[3] * ROBOT_REMOTE_MAX_RPM;
	float ry = param[2] * ROBOT_REMOTE_MAX_RPM;

	taskENTER_CRITICAL();
	g_remote_control.vx = vx;
	g_remote_control.vy = vy;
	g_remote_control.vz = vz;
	g_remote_control.rx = rx;
	g_remote_control.ry = ry;
	taskEXIT_CRITICAL();

	return pdPASS;
}

static int robot_zero_handle(float *param)
{
	(void)param;
	LOG("robot reset zero.\n");
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Reset_CurPos_To_Zero(i + 1);
		vTaskDelay(10);
	}
	return pdPASS;
}

/* ================================================================
   LeRobot 适配新增命令实现
   ================================================================ */

/**
 * @brief LeRobot get_state: 读取所有关节的当前角度、速度、负载
 *
 * 通过 CAN 总线逐个读取 Emm_V5 电机的 S_CPOS (位置)、S_VEL (速度)、
 * S_CPHA (相电流, 作为负载估算) 寄存器, 格式化后通过 UART1 返回。
 *
 * 响应格式: STATE:j1,j2,...,j6,v1,...,v6,l1,...,l6\n
 */
static int robot_get_state_handle(float *param)
{
	(void)param;

	char buf[256];
	float angles[ROBOT_MAX_JOINT_NUM];
	float velocities[ROBOT_MAX_JOINT_NUM];
	float loads[ROBOT_MAX_JOINT_NUM];
	uint8_t addr;
	uint32_t start_tick;

	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		addr = i + 1;
		struct joint *joint = &g_robot.joints[i];

		/* 角度: 直接取缓存值 */
		angles[i] = joint->current_angle;

		/* 速度 S_VEL (0x35) */
		vTaskSuspendAll();
		can.rxFrameFlag = false;
		start_tick = HAL_GetTick();
		while (!can.rxFrameFlag) {
			if ((HAL_GetTick() - start_tick) > ROBOT_CAN_TIMEOUT) break;
			Emm_V5_Read_Sys_Params(addr, S_VEL);
			HAL_Delay(1);
		}
		if (can.rxFrameFlag && can.rxData[0] == 0x35 && can.rxData[6] == 0x6b) {
			float vel_raw = 0;
			for (int j = 5; j >= 2; j--)
				vel_raw += (float)(((uint32_t)can.rxData[j]) << ((5 - j) << 3));
			if (can.rxData[1] == 0x01) vel_raw = -vel_raw;
			velocities[i] = vel_raw * 360.0f / 65536.0f / joint->reduction_ratio;
			if (joint->postive_direction == MOTOR_DIR_CCW) velocities[i] = -velocities[i];
		} else {
			velocities[i] = 0.0f;
		}
		xTaskResumeAll();

		/* 负载 S_CPHA (0x27, 相电流) */
		vTaskSuspendAll();
		can.rxFrameFlag = false;
		start_tick = HAL_GetTick();
		while (!can.rxFrameFlag) {
			if ((HAL_GetTick() - start_tick) > ROBOT_CAN_TIMEOUT) break;
			Emm_V5_Read_Sys_Params(addr, S_CPHA);
			HAL_Delay(1);
		}
		if (can.rxFrameFlag && can.rxData[0] == 0x27 && can.rxData[6] == 0x6b) {
			float load_raw = 0;
			for (int j = 5; j >= 2; j--)
				load_raw += (float)(((uint32_t)can.rxData[j]) << ((5 - j) << 3));
			if (can.rxData[1] == 0x01) load_raw = -load_raw;
			loads[i] = load_raw;
		} else {
			loads[i] = 0.0f;
		}
		xTaskResumeAll();
	}

	/* 格式化 STATE 响应 (使用整数格式化, nano.specs 不支持 %%f) */
	int offset = snprintf(buf, sizeof(buf), "STATE:");
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		int ipart = (int)angles[i];
		int fpart = (int)((angles[i] - ipart) * 100);
		if (fpart < 0) fpart = -fpart;
		offset += snprintf(buf + offset, sizeof(buf) - offset, "%d.%02d,", ipart, fpart);
	}
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		int ipart = (int)velocities[i];
		int fpart = (int)((velocities[i] - ipart) * 100);
		if (fpart < 0) fpart = -fpart;
		offset += snprintf(buf + offset, sizeof(buf) - offset, "%d.%02d%s",
			ipart, fpart, (i < ROBOT_MAX_JOINT_NUM - 1) ? "," : ",");
	}
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		int ipart = (int)loads[i];
		int fpart = (int)((loads[i] - ipart) * 100);
		if (fpart < 0) fpart = -fpart;
		offset += snprintf(buf + offset, sizeof(buf) - offset, "%d.%02d%s",
			ipart, fpart, (i < ROBOT_MAX_JOINT_NUM - 1) ? "," : "\n");
	}

	HAL_UART_Transmit(&huart1, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);
	return pdPASS;
}

/**
 * @brief LeRobot set_joints: 设置全部关节目标角度
 * 格式: set_joints j1 j2 j3 j4 j5 j6\n
 */
static int robot_set_joints_handle(float *param)
{
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		robot_send_abs_rotate_event(i, param[i]);
	}
	HAL_UART_Transmit(&huart1, (uint8_t *)"OK\n", 3, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)"OK\n", 3, HAL_MAX_DELAY);
	return pdPASS;
}

/**
 * @brief LeRobot set_torque: 电机扭矩使能/禁用
 * 格式: set_torque 1\n (使能) / set_torque 0\n (自由模式)
 */
static int robot_set_torque_handle(float *param)
{
	bool enable = (param[0] > 0.5f);
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_En_Control(i + 1, enable, true);
		vTaskDelay(5);
	}
	if (enable) {
		HAL_UART_Transmit(&huart1, (uint8_t *)"OK\n", 3, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart3, (uint8_t *)"OK\n", 3, HAL_MAX_DELAY);
	} else {
		HAL_UART_Transmit(&huart1, (uint8_t *)"OK:FREE\n", 8, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart3, (uint8_t *)"OK:FREE\n", 8, HAL_MAX_DELAY);
	}
	return pdPASS;
}

/**
 * @brief LeRobot e_stop: 紧急停止所有电机
 * 格式: e_stop\n
 */
static int robot_e_stop_handle(float *param)
{
	(void)param;
	for (int i = 0; i < ROBOT_MAX_JOINT_NUM; i++) {
		Emm_V5_Stop_Now(i + 1, true);
		vTaskDelay(2);
	}
	ROBOT_STATUS_CLEAR(g_robot.status, ROBOT_STATUS_RMODE_ENABLE);
	HAL_UART_Transmit(&huart1, (uint8_t *)"ESTOP\n", 6, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t *)"ESTOP\n", 6, HAL_MAX_DELAY);
	return pdPASS;
}

static int stream_start_handle(float *param)
{
	(void)param;
	LOG("stream start command received\n");
	usb_stream_start();
	return pdPASS;
}

static int stream_stop_handle(float *param)
{
	(void)param;
	LOG("stream stop command received\n");
	usb_stream_stop();
	return pdPASS;
}

static struct robot_cmd_info robot_uart1_cmd_table[] = {
	{"remote_event", robot_remote_event_handle},
	{"remote_enable", robot_remote_enable_handle},
	{"remote_disable", robot_remote_disable_handle},
	{"rel_rotate", robot_rel_rotate_handle},
	{"auto", robot_auto_handle},
	{"hard_reset", robot_hard_reset_handle},
	{"soft_reset", robot_soft_reset_handle},
	{"zero", robot_zero_handle},
	{"stream_start", stream_start_handle},
	{"stream_stop",  stream_stop_handle},
	// {"time_func", robot_time_func_handle},
	/* LeRobot 适配新增命令 */
	{"get_state", robot_get_state_handle},
	{"set_joints", robot_set_joints_handle},
	{"set_torque", robot_set_torque_handle},
	{"e_stop", robot_e_stop_handle},
	{NULL, NULL},
};

void robot_uart1_handle(struct robot_cmd *rb_cmd)
{
	static char event_type[20] = {0};
	float param[6] = {0};
	char *cmd = rb_cmd->cmd;
	int ret;

	/* ble_send: 转发文本到 USART3 + \r\n (发 AT 命令用) */
	if (strncmp(cmd, "ble_send ", 9) == 0) {
		char *payload = cmd + 9;
		HAL_UART_Transmit(&huart3, (uint8_t *)payload, strlen(payload), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart3, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)"OK\r\n", 4, HAL_MAX_DELAY);
		return;
	}

	/* ble_raw: 原始转发到 USART3, 不带换行 (发 +++ 进入 AT 模式用) */
	if (strncmp(cmd, "ble_raw ", 8) == 0) {
		char *payload = cmd + 8;
		HAL_UART_Transmit(&huart3, (uint8_t *)payload, strlen(payload), HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart1, (uint8_t *)"OK\r\n", 4, HAL_MAX_DELAY);
		return;
	}

	ret = sscanf(cmd, "%19s %f %f %f %f %f %f", event_type, &param[0], &param[1], &param[2],
		&param[3], &param[4], &param[5]);
	if (ret < 1) { // 解析失败
        LOG("event_type parse error: %s\n", cmd);
        return;
    }

	for (int i = 0; robot_uart1_cmd_table[i].event_type != NULL; i++) {
		if (strcmp(event_type, robot_uart1_cmd_table[i].event_type) == 0) {
			ret = robot_uart1_cmd_table[i].cmd_func(param);
			if (ret != pdPASS) {
				LOG("[ERROR] [jid:%d] event_type:%s param:%.2f %.2f %.2f\n", event_type, param[0], param[1], param[2]);
				return;
			}
			return;
		}
	}

	LOG("uart cmd parse error: %s\n", cmd);
	return;
}
