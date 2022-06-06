/*
 * wheel_test.c
 *
 *  Created on: 2021/05/07
 *      Author: takumi
 */


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stdint.h"
#include "wheel_test.h"
#include "main.h"
#include "nuc.h"
#include "wheel.h"
#include "Cntrl_Proc.h"
#include "conf.h"

#define TEST_TIME_INT 10	// ������� ms
#define TEST_READY	1		// �X�e�[�^�X�FREADY
#define TEST_ING	2		// �X�e�[�^�X�FING
#define TEST_COMP	3		// �X�e�[�^�X�FCOMP
#define MAX_PHASE	13		// �t�F�[�Y�ő�l
#define LOGSIZE_TEST 2600	// ���O�_��
#define TIM_CYCLE	24.39024	//�^�C�}�[�J�E���g�A�b�v����(sec��10^6�{)


#if WHEEL_TEST
typedef struct test_log{
	uint16_t	idx_;
	uint32_t	timer_[LOGSIZE_TEST];
	uint8_t		phase_[LOGSIZE_TEST];
	double		right_ref_[LOGSIZE_TEST];
	double		left_ref_[LOGSIZE_TEST];
	short		right_out_[LOGSIZE_TEST];
	short		left_out_[LOGSIZE_TEST];
	double		right_spd_[LOGSIZE_TEST];
	double		left_spd_[LOGSIZE_TEST];
	short		right_enc_[LOGSIZE_TEST];
	short		left_enc_[LOGSIZE_TEST];
}TEST_LOG;

TEST_LOG logdata;
#endif

extern uint32_t	freerun;
static double dist_per_pulse = WHEEL_DIAMETER * PI / PULS_WHEEL;
uint8_t flg_wheel_test_req = 0;
uint8_t test_status = TEST_READY;
uint8_t flg_test_start_edge;
uint16_t timer = 0;
uint16_t start_time = 0;
uint16_t phase_timer = 0;
uint16_t phase_start_time = 0;
uint8_t	phase = 0;					// ����t�F�[�Y

//					   	        0     1      2     3      4     5    6    7    8    9    10   11   12   13
//uint16_t phase_time[] = 	{1000, 1333,  2000, 1333,  1000,    0,   0,   0,   0,   0,   0,   0,   0,   0};
//double	s_ref_phase_r[] =   {0.00,  0.0,   0.8,  0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//double	e_ref_phase_r[] =   {0.00,  0.8,   0.8,  0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//double	s_ref_phase_l[] =   {0.00,  0.0,   0.8,  0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//double	e_ref_phase_l[] =   {0.00,  0.8,   0.8,  0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint16_t phase_time[] = 	{1000,  2000,  2000,  2000,  1000,    0,   0,   0,   0,   0,   0,   0,   0,   0};
double	s_ref_phase_r[] =   {0.00,   0.0,   0.8,   0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double	e_ref_phase_r[] =   {0.00,   0.8,   0.8,   0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double	s_ref_phase_l[] =   {0.00,   0.0,   0.8,   0.8,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double	e_ref_phase_l[] =   {0.00,   0.8,   0.8,   0.0,   0.0,  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double	right_ref = 0;			// �^�C�������w��[m/s]
double	left_ref = 0;
short	right_out = 0;			// ���[�^�[����ɓn���w���l(-30000�`30000)
short	left_out = 0;
double	right_spd = 0;			// ���p���x[m/s]
double	left_spd = 0;
int32_t	right_enc = 0;			// �G���R�[�_�p���X�J�E���^
int32_t	left_enc = 0;
uint8_t wheel_test_dump_req = 0;	// ���O�o�̗͂v���t���O
uint8_t wheel_test_dump_comp = 0;	// ���O�o�͂̊����t���O


void update_timer(){
/*
 * �^�C�}�[�X�V
 */
	timer = freerun - start_time;	// [ms]
}


void set_motor_ref_r(double speed){
	// ���x�w��[m/s]��-30000~30000�ɕϊ����ăZ�b�g
	right_out = (short)(speed * (double)MAX_NUC_REF / MAX_NUC_SPEED_WHEEL);
}


void set_motor_ref_l(double speed){
	// ���x�w��[m/s]��-30000~30000�ɕϊ����ăZ�b�g
	left_out = (short)(speed * (double)MAX_NUC_REF / MAX_NUC_SPEED_WHEEL);
}


short get_motor_ref_r(void){
	return right_out;
}


short get_motor_ref_l(void){
	return left_out;
}


void set_test_start_flg(){
	flg_test_start_edge = 1;
}


void reset_test_start_flg(){
	flg_test_start_edge = 0;
}


uint8_t get_test_start_flg(){
	return flg_test_start_edge;
}


void set_wheel_test_dump_req(){
	wheel_test_dump_req = 1;
}


void reset_wheel_test_dump_req(){
	wheel_test_dump_req = 0;
}


uint8_t get_wheel_test_dump_req(){
	return wheel_test_dump_req;
}


void set_wheel_test_dump_comp(){
	wheel_test_dump_comp = 1;
}


void reset_wheel_test_dump_comp(){
	wheel_test_dump_comp = 0;
}


uint8_t get_wheel_test_dump_comp(){
	return wheel_test_dump_comp;
}


void phase_update(){
	uint8_t phase_old = phase;
	uint8_t i = 0;
	uint32_t phase_time_sum = 0;
	// ���݂̃t�F�[�Y�𔻒�
	for (i = 0; i <= MAX_PHASE; i++){
		phase_time_sum += phase_time[i];
		if(phase_time_sum >= timer){
			phase = i;
			break;
		}
	}

	//�t�F�[�Y�J�n������̎��Ԃ��v�Z
	if (phase > phase_old){
		phase_start_time = freerun;
		phase_timer = 0;
	}
	phase_timer = freerun - phase_start_time;
}

void motor_spd_decision(){
/*
 * ���[�^�[�̎w�����x������
 * �ő�l��30000
 */
	phase_update();

	right_ref = s_ref_phase_r[phase] + (e_ref_phase_r[phase] - s_ref_phase_r[phase]) / (double)phase_time[phase] * (double)phase_timer;
	left_ref = s_ref_phase_l[phase] + (e_ref_phase_l[phase] - s_ref_phase_l[phase]) / (double)phase_time[phase] * (double)phase_timer;
}
void wheel_enc_count(){
/*
 * �G���R�[�_�p���X���J�E���g����
 */
	static uint32_t l_wheel_encoder = 0;
	static uint32_t r_wheel_encoder = 0;
	static uint32_t l_wheel_encoder_old = 0;
	static uint32_t r_wheel_encoder_old = 0;

	if(get_test_start_flg() == 1 ){	// TEST_ING�Ɉڍs����1JOB�ڂ̂ݎ��s
		l_wheel_encoder = 0;
		r_wheel_encoder = 0;
		l_wheel_encoder_old = get_l_wheel_encoder();
		r_wheel_encoder_old = get_r_wheel_encoder();
		left_enc = 0;
		right_enc = 0;
	}
	else{
		l_wheel_encoder = get_l_wheel_encoder();
		r_wheel_encoder = get_r_wheel_encoder();
		left_enc += (l_wheel_encoder - l_wheel_encoder_old);
		right_enc += (r_wheel_encoder_old - r_wheel_encoder);
		l_wheel_encoder_old = l_wheel_encoder;
		r_wheel_encoder_old = r_wheel_encoder;
	}
}

void wheel_spd_measure(){
/*
 * �G���R�[�_���瑬�x���擾����
 */
	double cnt_left = (double)get_l_wheel_enc_cnt();
	if ( cnt_left != 0){
		left_spd = dist_per_pulse * 1.0e6 / (TIM_CYCLE * (double)cnt_left);	// m/s
	}

	double cnt_right = (double)get_r_wheel_enc_cnt();
	if ( cnt_right != 0){
		right_spd = dist_per_pulse * 1.0e6 / (TIM_CYCLE * (double)cnt_right);	// m/s
	}
}


void test_log_update(){
/*
 * ���O���X�V����
 */
#if WHEEL_TEST
	if(logdata.idx_ < LOGSIZE_TEST){
		logdata.timer_[logdata.idx_] = timer;
		logdata.phase_[logdata.idx_] = phase;
		logdata.right_ref_[logdata.idx_] = right_ref;
		logdata.left_ref_[logdata.idx_] = left_ref;
		logdata.right_out_[logdata.idx_] = right_out;
		logdata.left_out_[logdata.idx_] = left_out;
		logdata.right_spd_[logdata.idx_] = right_spd;
		logdata.left_spd_[logdata.idx_] = left_spd;
		logdata.right_enc_[logdata.idx_] = right_enc;
		logdata.left_enc_[logdata.idx_] = left_enc;
		logdata.idx_ ++;
	}
#endif
}


void init_motor_test(){
/*
 * ��ԕϐ�������
 */
	timer = 0;
	start_time = freerun;
	phase = 0;
	phase_timer = 0;
	phase_start_time = freerun;
	right_ref = 0;
	left_ref = 0;
	right_out = 0;
	left_out = 0;
	right_spd = 0;
	left_spd = 0;
	right_enc = 0;
	left_enc = 0;
}


void init_test_log(){
/*
 * ���O�ϐ�������
 */
#if WHEEL_TEST
	logdata.idx_ = 0;
	for(int i = 0; i < LOGSIZE_TEST; i++){
		logdata.timer_[i] = 0;
		logdata.phase_[i] = 0;
		logdata.right_ref_[i] = 0;
		logdata.left_ref_[i] = 0;
		logdata.right_out_[i] = 0;
		logdata.left_out_[i] = 0;
		logdata.right_spd_[i] = 0;
		logdata.left_spd_[i] = 0;
		logdata.right_enc_[i] = 0;
		logdata.left_enc_[i] = 0;
	}
#endif
}

void wheel_test_log_dump() {
/*
 * ���O�̏o�� main����R�[�������
 */
#if WHEEL_TEST
	extern void uart2_transmitte(char *p);

	char buf[600];

	memset(buf, 0x00, sizeof(buf));
	sprintf(buf, "\r\ntimer, phase, right_ref, left_ref, right_out, left_out, right_spd, left_spd, right_enc, left_enc\r\n");
	uart2_transmitte(buf);

	memset(buf, 0x00, sizeof(buf));
	sprintf(buf, "[msec], [-], [m/s], [m/s], [-], [-], [m/s], [m/s], [-], [-]\r\n");
	uart2_transmitte(buf);

	for(int i = 0; i < LOGSIZE_TEST; i++) {
		if (logdata.timer_[i] == 0 && i != 0){
			break;
		}
		memset(buf, 0x00, sizeof(buf));
		//             1   2   3   4   5   6   7,  8   9  10
		sprintf(buf, "%d, %d, %f, %f, %d, %d, %f, %f, %d, %d\r\n",
				logdata.timer_[i],		//1
				logdata.phase_[i],		//2
				logdata.right_ref_[i],	//3
				logdata.left_ref_[i],	//4
				logdata.right_out_[i],	//5
				logdata.left_out_[i],	//6
				logdata.right_spd_[i],	//7
				logdata.left_spd_[i],	//8
				logdata.right_enc_[i],	//9
				logdata.left_enc_[i]	//10
				);
		uart2_transmitte(buf);
	}
#endif
	set_wheel_test_dump_comp();
}


void wheel_test_status_ctrl(){
/*
 * �e�X�g���̃X�e�[�^�X����
 */
	switch (test_status){
		case TEST_READY:
			if (flg_wheel_test_req == 1){
				test_status = TEST_ING;
				set_test_start_flg();
			}
			break;

		case TEST_ING:
			reset_test_start_flg();
			if (phase >= MAX_PHASE || flg_wheel_test_req != 1){
				test_status = TEST_COMP;
			}
			break;

		case TEST_COMP:
			if (flg_wheel_test_req != 1){
				test_status = TEST_READY;
			}
			break;
	}
}


void wheel_test_main(void){
/*
 * �e�X�g�̃��C���֐�
 * 10ms������main����R�[�������
 */
	if (get_RxCommand() == 0x30){
		// Require Wheel Motor test
		flg_wheel_test_req = 1;
	}
	else{
		flg_wheel_test_req = 0;
	}

	wheel_test_status_ctrl();

	switch (test_status){
	case TEST_READY:
		set_motor_ref_r(0);
		set_motor_ref_l(0);
		break;

	case TEST_ING:
		if(get_test_start_flg() == 1 ){	// TEST_ING�Ɉڍs����1JOB�ڂ̂ݎ��s
			init_motor_test();
			init_test_log();
		}
		update_timer();
		motor_spd_decision();
		set_motor_ref_r(right_ref);
		set_motor_ref_l(left_ref);
		wheel_enc_count();
		wheel_spd_measure();
		test_log_update();
		break;

	case TEST_COMP:
		set_motor_ref_r(0);
		set_motor_ref_l(0);
		break;
	}
}