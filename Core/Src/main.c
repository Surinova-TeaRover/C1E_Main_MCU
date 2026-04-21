/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "math.h"
#include <stdint.h>
#include <stdlib.h>
#include "EEPROM.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define					SET									1
	#define					NULL								0
	#define					BUZZER_ON						HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, GPIO_PIN_RESET);			HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, GPIO_PIN_RESET);	    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	#define					BUZZER_OFF					HAL_GPIO_WritePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin, GPIO_PIN_SET);			HAL_GPIO_WritePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin, GPIO_PIN_SET);	        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	#define					BUZZER_TOGGLE				HAL_GPIO_TogglePin(Buzzer_1_GPIO_Port,Buzzer_1_Pin );					HAL_GPIO_TogglePin(Buzzer_2_GPIO_Port,Buzzer_2_Pin); HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);
	#define					STOP_WHEELS					for(uint8_t i = 1; i <= 4; i++){Set_Motor_Torque(i, 0);}
	
	#define					LSB												0
	#define					MSB	 											1
	#define					REMOTE										1
	#define					DATA											2
	#define					STEERING_BOUNDARY 				2
	#define         STEERINGBOUNDARY          1
	#define					STEERING_HOMING_SPEED	 	25
	#define					STEERING_KP			 					5
	#define					STEERING_MAX_VEL					50
	
	#define					VELOCITY					0
	#define					TORQUE						1
	#define					VEL_LIMIT					2
	#define					MOT_ERROR					3
	#define					ENC_ERROR					4
	#define					IQ								5
	#define					ENC_EST						6
	#define					T_RAMP						7
	#define					REQ_STATE					8
	#define					SNL_ERROR					9
	#define					SENSL_EST					10
	#define					POSITION					11
	
	#define					IMU_L							0x08
	#define					IMU_R							0x09
	#define					LF_STEER					0x10
	#define					LR_STEER					0x14
	#define					RF_STEER					0x12
	#define					RR_STEER					0x13
//	#define					L_ARM							0x11
//	#define					R_ARM							0x15
//	#define					P_ARM							0x16
//	#define					LT_SENS						0x17
	//#define					L_VERT						0x21
	#define					R_VERT						0x22
	#define					C_LMT							0x07
	#define					FL_FLAP						0x17
	#define					FR_FLAP						0x18
	#define					RL_FLAP						0x19
	#define					RR_FLAP						0x20
	#define					CMD_MASK					0x01F	
	#define         IMU_SHEAR         0x15
	
	#define					HEARTBEAT					0x01
	#define					POS_ID						0x0C
	#define					VEL_ID						0x0D
	#define					TRQ_ID						0x0E
	#define					VLMT_ID						0x0F
	#define					MERR_ID						0x03
	#define					ENERR_ID					0x04
	#define					SNERR_ID					0x05
	#define					IQM_ID						0x014
	#define					ENEST_ID					0x009
	#define					T_RAMP_ID					0x1C
	#define					REQ_STATE_ID			0x07
	#define					SENS_EST					0x015
	#define					VOLTAGE					  0x017
	#define 				FET_TEMP          0x15
	
	#define					ALL_WHEEL					1
	#define					CRAB							3
	#define					ZERO_TURN					2
	#define					WIDTH_SHRINK			4
	#define					WIDTH_EXTEND			5
	#define					FRONT_WHEEL				6
	#define					WIDTH_SPEED				10
	#define					WIDE_ANGLE				4
	#define					SHRINK_ANGLE			4
	#define					Anti_Windup_Limit		2
	#define					V_LIMIT							70
	#define					C_LIMIT							70
	#define					ARM_HOMING_SPEED	 	15
	
	#define					BT_READ						HAL_GPIO_ReadPin(UART5_State_GPIO_Port,UART5_State_Pin);
	#define					BT_READ_1						HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
	#define 				EEPROM_ADDRESS 	0xA0 // Adjust according to the EEPROM's address
	#define 				PAGE_SIZE 				64        // EEPROM page size

	#define 			ARRAY_SIZE 					70
	#define         PI             3.14
	#define       ALPHA            0.1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
DMA_HandleTypeDef hdma_uart5_tx;
DMA_HandleTypeDef hdma_uart5_rx;

/* USER CODE BEGIN PV */
/* 							BT_VARIABLES 						*/
uint8_t BT_Rx[9], BT_Count=0, RxBuff[9];
bool BT_State=0 ,  Prev_BT_State=0, BT_State1 = 0;
/* 							BT_VARIABLES 						*/


/* 							CAN_VARIABLES 						*/

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;
uint8_t TxData[8];
uint8_t RxData[8];
uint8_t TxData2[8];
uint8_t RxData2[8];
uint8_t RxData2_Temp[8];
uint8_t RxData_Temp[8];
uint32_t TxMailbox, CAN_Count=0;
uint8_t Node_Id[40],PREV_Node_Id[40], Received_Node_Id=0, Received_Command_Id=0;
uint8_t Sensor_Id[10], Axis_State[30];
float Motor_Velocity[21], Rover_Voltage=0,Motor_Current[21], Rover_Voltage_Temp=0;uint8_t Motor_Error[20], Encoder_Error[20] , Volt_Tx=0, Volt_Tx_Temp=0;
uint8_t LFD=1,LRD=2,RFD=3,RRD=4,LVert=5, RVert=6, Contour=7, LFS=8, LRS=9, RFS=10, RRS=11, L_Arm=12, R_Arm=13, P_Arm=14 , Upper_Width =16 , Lower_Width = 15, Cutter=17, Side_Belt = 18, Selective = 19, Paddle =20;

/* 							CAN_VARIABLES 						*/

/* 							IMU_VARIABLES 						*/

float L_Roll=0, L_Pitch=0, R_Roll=0, R_Pitch=0;
float Left_Roll_Pos = 1.5 - 2, Right_Roll_Pos = 0.5625, Right_Pitch_Pos = 0.4375, Left_Pitch_Pos=17.4, Left_Column_Error =0 , Left_Col_Pos = 0;
float Right_Roll_Home_Pos = 0.5625 , Right_Pitch_Home_Pos = 0.4375 ;
float Base_Pitch =0 , Base_Roll = 0;
//float Left_Roll_Pos = 1.5 - 2, Right_Roll_Pos = 1.9, Right_Pitch_Pos = 4.93, Left_Pitch_Pos=17.4, Left_Column_Error =0 , Left_Col_Pos = 0;
bool Left_IMU_State=1, Initiate_Process=0;
float Shear_Roll = 0, Shear_Pitch = 0;
/* 							IMU_VARIABLES 						*/
/* 							STEERING_VARIABLES 						*/
//uint16_t Track_Width = 1800, Min_Track_Width = 1800, Wheel_Base = 900; //Track_Width = 1730, Min_Track_Width = 1730
float Zero_Turn_Angle = 27, Track_Width = 1800, Min_Track_Width = 1800, Wheel_Base = 900;;
uint16_t Steer_Angle[5];
float LF_Steering=0, LR_Steering=0, RF_Steering=0, RR_Steering=0;	
float LF_HomePos =57, LR_HomePos= 419 , RF_HomePos= 570  , RR_HomePos = 77;	// -->	HOME POSITIONS LF_HomePos = 190, LR_HomePos= 87 , RF_HomePos= 220 , RR_HomePos = 623;
float LF_Speed=0, LR_Speed=0, RF_Speed=0, RR_Speed=0 , LF_Speed_Temp =0, LR_Speed_Temp =0 , RF_Speed_Temp=0, RR_Speed_Temp=0, LF_Error=0, LR_Error=0, RF_Error=0, RR_Error=0;		
//int LF_Speed=0, LR_Speed=0, RF_Speed=0, RR_Speed=0 , LF_Speed_Temp =0, LR_Speed_Temp =0 , RF_Speed_Temp=0, RR_Speed_Temp=0;
//float LF_Error=0, LR_Error=0, RF_Error=0, RR_Error=0;
_Bool Steering_Reset_Flag = SET , LF_SET = NULL , LR_SET = NULL, RF_SET = NULL, RR_SET = NULL , BUZZ_SW = SET;
float Inner_Angle =0 , Outer_Angle=0, Prev_Inner_Angle =0 , AW_Angle=0 , Outer_Angle_2=0 , LS_Angle=0, RS_Angle=0;
bool Angle_Ready = 0;

float LFS_Filtered = 0, Prev_LFS_Filtered = 0, LRS_Filtered = 0, Prev_LRS_Filtered = 0, RFS_Filtered = 0, Prev_RFS_Filtered = 0, RRS_Filtered = 0, Prev_RRS_Filtered = 0;
/* 							STEERING_VARIABLES 						*/
/* 							SENSING_VARIABLES 						*/

float FL_Raw =0, FR_Raw = 0, RL_Raw = 0, RR_Raw = 0;
float FL_Angle=0, FR_Angle=0, RL_Angle =0, RR_Angle=0, FL_Angle_Temp=0;
uint16_t FL_Home_Pos = 563 , FR_Home_Pos = 0, RL_Home_Pos = 0, RR_Home_Pos = 0;
int16_t Left_Arm_Motor_Count=0, Right_Arm_Motor_Count=0, Right_Arm_Motor_Value=0, Left_Arm_Motor_Value=0, Pitch_Arm_Motor_Count=0, Pitch_Arm_Motor_Value=0;
float L_Arm_Speed=0, R_Arm_Speed=0, L_Arm_Speed_Temp=0, R_Arm_Speed_Temp=0, Pitch_Arm_Speed_Temp=0, Tri_Arm_Speed=0;double Pitch_Arm_Speed=0;
_Bool Front_Left_Bush = 0, Front_Right_Bush = 0, Front_Bushes_Sensed = 0, First_Sense=0 , Rear_Bush=0;
int Flaps_Target = 60, Flap_Error=0,Flap_Error_Right = 0,Flaps_Target_Right=60, Flap_Error_Left = 0, Flaps_Target_Left = 60;
float Flap_Kp = 2, Pitch_Kp=2 ;
_Bool Front_Left_Bush_Timer = 0, Front_Left_Bush_Timer_Active = 0;

float Macro_Speed = 0;
/* 							SENSING_VARIABLES 						*/


/* 							JOYSTICK_VARIABLES 						*/
uint8_t Mode=1,Mode_Temp = 0, Speed=1, Joystick=0, Steering_Mode=1, Shearing=0, Skiffing=0, Side_Trimmer=0, Pot_Angle=90, Joystick_Temp=0, Shearing_Temp=0, Steering_Mode_Temp=1, BT_Steer_Temp=0, Speed_Temp = 0, Heartbeat = 0, Heartbeat_Temp = 0;		
uint64_t Timt_Batt = 0;
uint8_t Tx_Voltage = 0;
/* 							JOYSTICK_VARIABLES 						*/


/* 							DRIVE_WHEELS_VARIABLES 						*/
bool DRIVES_ERROR_FLAG = NULL;
float L_R_Err=0, R_R_Err=0, C_Err=0, Contour_Avg=0, Drive_Torque=1, Wheel_Torque = 10, Prev_R_R_Err = 0, Prev_C_Err = 0;
float Vel_Limit = 0, Vel_Limit_Temp=1, Torque=0, Torque_Temp=0 , Prev_Torque=0, Prev_Vel_Limit=30, Input_Vel = 0;
int Left_Wheels_Torque =0, Left_Wheels_Torque_Temp=0;
bool MODE_CHANGE_FLAG = SET;

/* 							DRIVE_WHEELS_VARIABLES 						*/

float Absolute_Position[20];
int16_t Absolute_Position_Int[20];
float Absolute_Position_Float[20];

/* 							FRAME_CONTROLS_VARIABLES 						*/
float L_Vert_Speed=0, R_Vert_Speed=0, L_Vert_Speed_Temp=0, R_Vert_Speed_Temp=0, Contour_Speed=0, Contour_Speed_Temp=0;
bool Left_Error_Flag=NULL , Right_Error_Flag=NULL , Contour_Error_Flag=NULL, FRAME_NO_ERROR_FLAG=SET,Contour_Limit=SET,Vertical_Limit=SET; 
float  R_Error_Change=0, R_Error_Slope=0, R_Error_Area=0, R_Prev_Error=0;
float R_Kp=15, R_Ki=0, R_Kd=5; 
long R_P=0, R_I=0, R_D=0;
float Error=0, L_Prev_Error=0, L_Error_Change=0, L_Error_Slope=0, L_Error_Area=0, Left_Out=0, Right_Out=0, Contour_Out=0;
float  C_Error_Change=0, C_Error_Slope=0, C_Error_Area=0, C_Prev_Error=0;
float C_Kp=11, C_Ki=2, C_Kd=5;      
long C_P=0, C_I=0, C_D=0;
double dt=0.01 ;
int Left_Vertical_Error=0;
int Current_Vel_Limit = 0, Modified_Vel_Limit = 0, Modified_Vel_Limit_Temp =0 , Prev_Mod=0; 
float Width_Motor_Speed=0, Width_Motor_Temp=0, Lower_Width_Motor_Speed = 0, Upper_Width_Motor_Speed = 0, Lower_Width_Motor_Speed_Temp=0, Upper_Width_Motor_Speed_Temp=0;

bool Right_Vertical_On_Limit = 0, Contour_On_Limit=0;
float L_Roll_Err = 0, Width_Correction_Speed = 0;
/* 							FRAME_CONTROLS_VARIABLES 						*/



/* 							EEPROM_VARIABLES 						*/

//int16_t Read_Value[28], Write_Value[28], Prev_Write_Value[28],Read_Value_1[28],Max_val=600;
int8_t Read_Value[25] = {0}, Write_Value[25], Prev_Write_Value[25],Read_Value_1[25],Start_Value,End_Value;
bool Store_Data = 0;

float Lower_Width_Motor_Count=0, Upper_Width_Motor_Count=0, Lower_Width_Motor_Value=0, Upper_Width_Motor_Value=0; // Total Counts
float Vertical_Motor_Count =0, Contour_Motor_Count =0, Vertical_Motor_Value =0, Contour_Motor_Value =0;
float Left_Macro_Motor_Count = 0, Right_Macro_Motor_Count = 0, Left_Macro_Motor_Value = 0, Right_Macro_Motor_Value = 0;

uint8_t eeprom = 0;
/* 							EEPROM_VARIABLES 						*/

bool Buzz_Switch = 0, Frame_Buzz_Switch=0;

/* 							MANUAL WHEEL CONTROLS 						*/
uint8_t Manual_Axis_1=1,Manual_Axis_2=2,Manual_Axis_3=3,Manual_Axis_4=4,Manual_Axis,All_Whel_SameTorque=1;
float Manual_Torque_1,Manual_Torque_2,Manual_Torque_3,Manual_Torque_4,Manual_Torque;
float Manual_Vel_Limit =20,Manual_Vel_Limit_1=20,Manual_Vel_Limit_2=20,Manual_Vel_Limit_3=20,Manual_Vel_Limit_4=20,Manual_Torque_Temp=0,Manual_Vel_Limit_Temp=20;
float Manual_Vel_Limit_Temp_1=20,Manual_Vel_Limit_Temp_2=20,Manual_Vel_Limit_Temp_3=20,Manual_Vel_Limit_Temp_4=20,Manual_Torque_Temp_1=0,Manual_Torque_Temp_2=0,Manual_Torque_Temp_3=0,Manual_Torque_Temp_4=0;;

//uint8_t Left_Vel_Limit = 5, Right_Vel_Limit=5, Prev_Left_Vel_Limit = 15, Prev_Right_Vel_Limit = 15, Left_Transmit_Vel=0, Right_Transmit_Vel=0;
float Left_Vel_Limit = 0, Right_Vel_Limit=0, Prev_Left_Vel_Limit = 15, Prev_Right_Vel_Limit = 15, Left_Transmit_Vel=0, Right_Transmit_Vel=0, Left_Vel_Limit_Temp = 0, Right_Vel_Limit_Temp = 0, Left_Transmit_Vel_Temp = 0, Right_Transmit_Vel_Temp = 0;
float New_Left_Vel_Limit = 0, New_Right_Vel_Limit = 0, Accel_Factor = 0;
int  Frame_Vel_Limit = 0, Left_Steering_Vel_Limit = 0, Right_Steering_Vel_Limit=0;
float Accel_Sync = 0, Left_Vel=0, Left_Diff=0, Left_Vel_Temp = 0;
int Steering_Angle =0;
double Rover_Centre_Dist=0, TimeTaken=0, Inner_Speed=0, Outer_Speed=0;
int WheelBase=900/2, TrackWidth=1800/2;  //1300/2
double kmph = 1;
int Left_Steering_Speed=0, Right_Steering_Speed=0, Left_Frame_Speed =0;
float Current_Rover_Velocity = 0, Avg_Steering_Angle = 0, Speed_Factor = 0, Calc_Outer_Speed = 0, New_Inner_Angle = 0;
//int16_t Flap_Data[ARRAY_SIZE] = {0};
//int Flap_Angle = 0, Flap_Count=0;
//float Arm_Angle=0,Left_Arm_Pos = 0, Right_Arm_Pos = 0, Pitch_Arm_Pos = 0,Tri_Arm_Pos = 0, Left_Arm_Pos_Temp = 0, Right_Arm_Pos_Temp = 0, Pitch_Arm_Pos_Temp = 0;

uint64_t left_tick_count = 0;
uint64_t right_tick_count = 0;
bool FLAG = SET;
uint8_t Prev_Joystick = 0;
int count = 0;
float Rover_Velocity = 0.0;
			float Cont=0, Cont_temp=0;


bool JOYSTICK_STATE_FLAG = NULL, AXIS_STATE_FLAG = SET, HEARTBEAT_FLAG = SET, FET_TEMP_FLAG = SET, OPERATION_MONITOR_FLAG = NULL, MOTORS_STOP_FLAG = SET;
uint64_t Tick_Count1 = 0, Tick_Count2 = 0;
uint8_t Node_Id_Temp[40];
int Node = 0, fet = 0;


float  LF_Error_Change=0, LF_Error_Slope=0, LF_Error_Area=0, LF_Prev_Error=0;
float LF_Kp=3, LF_Ki=1, LF_Kd=0;  
long LF_P=0, LF_I=0, LF_D=0;
float Left_Frame_Out=0;

uint8_t Tx_Uart[2];



//////////////////////////////////////////////////FILTER VARIABLES/////////////////////////////////////////////////////////////
double R_Pitch_Filtered = 0, Prev_R_Pitch_Filtered = 0, R_Roll_Filtered = 0, Prev_R_Roll_Filtered = 0;
double FL_LPF_Angle = 0, FR_LPF_Angle = 0, RL_LPF_Angle = 0, RR_LPF_Angle = 0, Prev_FL_LPF_Angle = 0, Prev_FR_LPF_Angle = 0, Prev_RL_LPF_Angle = 0, Prev_RR_LPF_Angle = 0;

double BLE_Roll = 0, BLE_Pitch = 0, Prev_BLE_Roll = 0, Prev_BLE_Pitch = 0;

/*                                               FILTER VARIABLES                                                       */

/////////////////////////////////////////////////////FLAP VARIABLES//////////////////////////////////////////////////////
int16_t Flap_Angle_Left = 0;
int16_t Flap_Data[ARRAY_SIZE] = {0};
float Flap_Angle = 0, Flap_Angle_Right = 0,Flap_Count=0;
int16_t Flap_Data_Right[ARRAY_SIZE] = {0}, Flap_Data_Array[ARRAY_SIZE];
bool LF_Bush_Sensed = 0, LR_Bush_Sensed = 0, RF_Bush_Sensed = 0, RR_Bush_Sensed = 0;
uint64_t Last_Tick_FL = 0, Last_Tick_RL = 0, Last_Tick_FR = 0, Last_Tick_RR = 0;
int16_t Front_Angle = 0,  Rear_Angle = 0;
float Flap_Error_LR = 0, Flap_Target_LR = 0, Flap_Error_RR = 0, Flap_Target_RR = 0, Flap_Error_FAVG = 0, Flap_Target_FAVG =0, Flap_Error_RAVG = 0, Flap_Target_RAVG =0;

/*                                                   FLAP VARIABLES                                            */

////////////////////////////////////////////////////MACRO VARIABLES///////////////////////////////////////////////////

float Left_Macro_Speed = 0, Right_Macro_Speed = 0, Left_Macro_Speed_Temp = 0, Right_Macro_Speed_Temp = 0;
float MLeft_Macro_Speed = 0, MRight_Macro_Speed = 0, MLeft_Macro_Speed_Temp = 0, MRight_Macro_Speed_Temp = 0;
float Left_Macro_Count = 0, Right_Macro_Count = 0, Macro_Error = 0, Macro_Kp = 1, Correction_Speed = 0;
uint8_t max_difference = 5;

float M_Error_Change = 0, M_Prev_Error = 0, M_Error_Slope = 0, M_Error_Area = 0;
float M_P = 0, M_I = 0, M_D = 0, M_Kp = 0, M_Ki = 0, M_Kd = 0;
float Macro_Out = 0, Macro_Max_Speed = 50;
float MMacro_Speed=0;
/*                                                 MACRO VARIABLES                                                    */


int Flap_Max_Angle = 55;
float Arm_Angle=0,Left_Arm_Pos = 0, Right_Arm_Pos = 0, Pitch_Arm_Pos = 0,Tri_Arm_Pos = 0, Left_Arm_Pos_Temp = 0, Right_Arm_Pos_Temp = 0, Pitch_Arm_Pos_Temp = 0;
int Flap_Mod_Value = 0, Flap_Mod_Value_Right = 0,Left_Flap=0,Right_Flap=0;
bool Front_Bush = 1;
uint8_t Array_Element,Arm_Max_Speed=40;
int Left_Arm_Current_Pos,Right_Arm_Current_Pos,Pitch_Arm_Current_Pos;
float Shear_Height_Diff =0, L_Arm_Travel = 0, R_Arm_Travel = 0, Shear_Roll_Angle = 0, Pitch_Compensation_mm = 0, Pitch_Target = 0;

float  P_Error_Change=0, P_Error_Slope=0, P_Error_Area=0, P_Prev_Error=0, Pitch_Out =0;


long L_P=0,L_I=0,L_D=0;
float L_Kp=40,L_Ki=5,L_Kd=0;

long RA_P=0,RA_I=0,RA_D=0;
float RA_Kp=1,RA_Ki=0,RA_Kd=0;

long P_P=0,P_I=0,P_D=0;
float P_Kp=5,P_Ki=0,P_Kd=0;

float RightArm_Out=0,RA_Error_Change=0,RA_Error_Slope=0,RA_Error_Area=0,RA_Prev_Error=0;
int8_t Test_Read,Test_Write;
int test;
int16_t LA = 0;
float L_Vel_Limit = 0, R_Vel_Limit = 0, L_Vel_Limit_Temp = 0, R_Vel_Limit_Temp = 0;
uint64_t Vel_Tick = 0;
float Turning_Radius = 0;
float Angle = 0;
float LF_Pos = 0, LR_Pos = 0, RF_Pos = 0, RR_Pos = 0, LF_Pos_Temp = 0, LR_Pos_Temp = 0, RF_Pos_Temp = 0, RR_Pos_Temp = 0; 
float Input_Velocity[20];
//uint16_t Half_Track_Width = 0, Half_Wheel_Base = 0;
float Half_Track_Width = 0, Half_Wheel_Base = 0;

/////////////////////////////////////////////////////OPERATION MONITOR VARIABLES	////////////////////////////////////////
uint64_t Heartbeat_Tick = 0, Drive_Error_Tick = 0, Fet_Temp_Tick =0, Overload_Tick = 0, Motor_Tick = 0, speed_time = 0, Joystick_Tick = 0, Vertical_Limit_Tick = 0, Contour_Limit_Tick = 0, Pitch_Limit_Tick = 0, Vertical_Tick = 0, Vert_Resp_Tick = 0, Contour_Tick = 0, Cont_Resp_Tick = 0, Pitch_Tick = 0, Pitch_Resp_Tick = 0;
bool Drive_Disconnected = NULL, Sensor_Disconnected = NULL, Drive_Errored = NULL, FET_Temp_Exceeded = NULL, Motor_Overloaded = NULL, E_Stop = NULL, Joystick_Disconnected = NULL, Vertical_Limit_Exceeded = NULL, Contour_Limit_Exceeded = NULL, Pitch_Limit_Exceeded = NULL, EEPROM_Error = NULL, Vertical_Not_Responding = NULL, Contour_Not_Responding = NULL, Pitch_Not_Responding = NULL, Steering_Boundary_Flag = NULL;
float FET_Temperature[20];
uint8_t Speed_Ref = 0;
float Vertical_Error = 0, Contour_Error = 0, Pitch_Error = 0;


uint64_t Shearing_Tick = 0, Reboot_Tick = 0, Flaps_Tick = 0, Imu_Tick = 0;
bool Shearing_Drive_Errored = NULL, Shearing_Drive_Disconnected = NULL, Flaps_Disconnected = NULL, IMU_Disconnected = NULL;

/*                                                   OPERATION MONITOR VARIABLES	                                      */


////////////////////////////////////////////////////PITCH VARIABLES////////////////////////////////////////////////////////////

float Lead_Screw_Length = 0, Vertical_Angle = 0, Pitch_Target_Angle = 0, Shear_Angle = 0, Shear_Pitch_Home_Pos = -64.1, Pitch_Arm_Error = 0;
float Shear_Roll_Home_Pos = 0, Pitch_Angle = 0, Contour_Angle = 0;
/*                                                 PITCH VARIABLES                                                         */


///////////////////////////////////////////////////////SHEARING VARIABLES/////////////////////////////////////////////////////


/*                                                   SHEARING VARIABLES                                                        */
bool message_sent = 0;
char Tx_Initial_msg[]="1.Battery percentage 2. Error Indication  3.Left vertical 4.Left Contour 5.Right vertical 6.Right contour 7.Pitch Arm 8.FET Temperature",Rx_Data[1];
uint64_t Uart_Time = 0;
int len;
volatile uint8_t transmitDataFlag = 0; 
volatile char currentCommand,id;    
volatile uint8_t commandReceived = 0;
volatile uint8_t sendContinuouslyFlag = 0;
float Main_Bt_Percentage = 0;	
char outputBuffer[1024],Main_Battery[42],Left_IMU_Data[50],Right_IMU_Data[50],Pitch_IMU_Data[50];
char Node_ID_To_Name[][10]={"","LFW","LRW","RFW","RRW","","Vert","Contour","LFS","LRS","RFS","RRS","L_Macro","R_Macro","Pitch_Arm","L_Width","U_Width"};
float LF = 0, LR = 0;
int err_count = 0;
float Var = 0, Var_Temp = 0;

float Prev_Vel = 0, Current_Vel = 0;
uint64_t Dummy_Tick = 0, Dummy_Tick_2 = 0;
float Right_roll_value = 0, Right_pitch_value = 0, Right_Pitch = 0, Right_Roll = 0, Right_Roll_Final = 0, Right_Pitch_Final = 0;
float flap_pos = 0.0f;

/*					PITCH_SHEAR_CONTROL_VARIABLES					*/

float Base_Pitch_Filtered = 0, Prev_Base_Pitch_Filtered = 0, Shear_Pitch_Filtered =0, Prev_Shear_Pitch_Filtered=0 ;
float Base_Pitch_HomePos = 0.0625, Shear_Pitch_HomePos = -63, Base_Pitch_Angle = 0, Shear_Pitch_Angle = 0;
float Shear_Pitch_Error = 0;
float Shear_Pitch_Speed = 0, Shear_Pitch_Speed_Temp = 0;
uint8_t Shear_Pitch_Kp = 8;
/*					PITCH_SHEAR_CONTROL_VARIABLES					*/


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

float New_Sensor_Pos(double Sensor_Value, double Zero_Pos);
void Start_Calibration_For (int axis_id, int command_id, uint8_t loop_times);
void Drives_Error_Check(void);
void Reboot (int Axis);
void Heal_Error(uint8_t Axis_Id);
void Stop_Motors(void);
void Set_Motor_Torque ( uint8_t Axis , float Torque );
void Set_Motor_Velocity ( uint8_t Axis , float Velocity );
void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format);
float Differintial_Angle ( double Inner_Angle_Set );
void Manual_Controls (void);
void Joystick_Reception(void);
void Drive_Wheel_Controls(void);
void Steering_Controls (void);
void Battery_Status_Indication(void);
float Contour_PID ( float Contour_Val , unsigned long long 	C_Time_Stamp );
float Right_Verticality_PID ( float Right_Roll_Value , unsigned long long 	R_Time_Stamp );
void Frame_Controls(void);
void Dynamic_Width_Adjustment (void);
void EEPROM_Store_Data (void);
void Read_EEPROM_Data(void);
void Frame_Synchronization(void);
void Top_Flap_Sensing(void);
void Manual_Wheel_Control(void);
void Wheel_Speeds_Calc(int Inner_Angle);
void New_Drive_Controls(void);
void Transmit_Motor_Torque (void);
void Left_Frame_Controls (void);
 void Clear_Error (int Axis);
 void Position_Flap_Sensing(void);
 void Shearing_Motors(void);
 void Emergency_Stop(void);
 void Operations_Monitor(void);
 float Left_Frame_PID ( float Left_Error_Value , unsigned long long 	L_Time_Stamp );
 void Top_Sensing_Roll(void);
 void Manual_Sensing_Control(void);
 void Demo(void);
 void New_Drive_Controls_V2(void);
 void Drive_Wheel_Controls_Vel_Based(void);
 void New_New_Drive_Controls(void);
 void New_Steering_Controls(void);
 float Angle_Calc(double Inner_Angle);
 void Steering_Pos_Controls (void);
 void Start_Continuous_Sending(char command);
 void UART_tx(void);
 void Initial_Msg(void);
 void Flap_Sensor_Pos(double Sensor_Value, double Zero_Pos);
 float Top_Sensing_PID ( float Flap_Value , unsigned long long 	R_Time_Stamp );
 void All_Macro_Sensing(void);
 void New_Steering_Controls_(void);
 void Pitch_Control(void);
 float convertRawDataToFloat(uint8_t* data);
 float Float16_To_Decimal(uint16_t float16);
 void Frame_Controls_Sensor_BLE(void);
 void Dynamic_Width_Corrections(void);
 void Macro(void);
 void Frame_Manual_Controls(void);
 void Flap_Sensing(void);
 void Pitch_Arm_Control_IMU(void);

 //void Set_Motor_Position (uint8_t Axis, float Position);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Update_Array(int16_t* Array, uint8_t Size, int16_t Latest_Value) 
{//uint8_t Sample_Size =5;
    for (uint8_t i = 0; i < Size - 1; i++) {
        Array[i] = Array[i + 1];
    }   
    Array[Size - 1] = Latest_Value;
		
//		for (uint8_t i = Size - 1 ; i >= Sample_Size ; i ++ )
//		{
//			Flap_Angle += Array[i];
//		}
//		
//		Flap_Angle = Flap_Angle / Sample_Size;
//	Flap_Angle = (Array[Size-1] + Array[Size-2] + Array[Size-3]+ Array[Size-4]+ Array[Size-5]) / 5; 
		
		Flap_Count++;
}

double Right_Vertical_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double Right_Contour_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double Shearing_IMU_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}
double Base_IMU_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double FL_Flap_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double FR_Flap_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double RL_Flap_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}

double RR_Flap_LPF(double input, double prev_output, double alpha) {
    return alpha * input + (1.0 - alpha) * prev_output;
}
double LFS_LPF(double input, double prev_output, double alpha)
{
	return alpha * input + (1.0 - alpha) * prev_output;
}
double LRS_LPF(double input, double prev_output, double alpha)
{
	return alpha * input + (1.0 - alpha) * prev_output;
}
double RFS_LPF(double input, double prev_output, double alpha)
{
	return alpha * input + (1.0 - alpha) * prev_output;
}
double RRS_LPF(double input, double prev_output, double alpha)
{
	return alpha * input + (1.0 - alpha) * prev_output;
}
double BLE_Vertical_LPF(double input, double prev_output, double alpha)
{
	return alpha * input + (1.0 - alpha) * prev_output;
}
double BLE_Contour_LPF(double input, double prev_output, double alpha)
{
	return alpha * input + (1.0 - alpha) * prev_output;
}
/* 							UART RECEPTION INTERRUPTS 						*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BT_State1 = BT_READ_1;
	if (huart -> Instance == UART5)
	{
			HAL_UART_Receive_DMA(&huart5,BT_Rx ,sizeof(BT_Rx));
			BT_Count++;
	}
	
	if (huart -> Instance == UART4)
	{
		HAL_UART_Receive_IT(&huart4,(uint8_t*)Rx_Data ,sizeof(Rx_Data));
		if (BT_State1  == 1) 
		{
			currentCommand=Rx_Data[0];
			UART_tx();
		} 
	}
}
/* 							UART RECEPTION INTERRUPTS 						*/
void Absolute_Position_Reception( uint8_t Node_Id )
{
  memcpy(&Absolute_Position[Node_Id],RxData2, sizeof(float)); 
	Absolute_Position_Int[Node_Id] = Absolute_Position[Node_Id]; 
	Absolute_Position_Float[Node_Id]=roundf(Absolute_Position[Node_Id]*100)/100;
}
float CAN_Reception(uint8_t byte_choice)
{
		float Can_Temp;
	
	if ( byte_choice == LSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData2[k];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else if ( byte_choice == MSB )
	{
		for (int k=0; k<=3; k++)
		{
			RxBuff[k]= RxData2[k+4];
		}
		memcpy(&Can_Temp, RxBuff,4);
		
	}
	
	else { }

	return(Can_Temp);
}
float KMPHtoRPS(float kmph)
{
    float rps=0; float Circumference = 1335.177;
    rps = (((kmph/(Circumference/1000)*1000)/3600)*150);
    return rps;
}

uint16_t CAN_SPI_READ(uint8_t Data[8] )
{ uint16_t Enc_Angle=0;
	
				Enc_Angle = Data[0];
				Enc_Angle = Enc_Angle << 8 | Data[1];
	return Enc_Angle;
}	

/*                                CAN 1  Reception                                     */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

	switch(RxHeader.StdId)
	{
			
		case (IMU_L)	 :	L_Roll = ((int16_t)(RxData[1]<<8 | RxData[0]))/16.0;	L_Pitch = ((int16_t)(RxData[3]<<8 | RxData[2]))/16.0;				  Sensor_Id[1]++; Node_Id[21]++;	break; 
		
		case (IMU_R)	 :	R_Roll = ((int16_t)(RxData[1]<<8 | RxData[0]))/16.0;	R_Pitch = ((int16_t)(RxData[3]<<8 | RxData[2]))/16.0; Base_Roll = ((int16_t)(RxData[5]<<8 | RxData[4]))/16.0; Base_Pitch = ((int16_t)(RxData[7]<<8 | RxData[6]))/16.0;	 			  Sensor_Id[2]++;Node_Id[22]++;		break;
			
		case (LF_STEER): 	Steer_Angle[1] = CAN_SPI_READ(RxData);  			LF_Steering = New_Sensor_Pos ( Steer_Angle[1] , LF_HomePos ) ;						Sensor_Id[3]++;Node_Id[23]++;	 break;
		
		case (LR_STEER): 	Steer_Angle[2] = CAN_SPI_READ(RxData);				LR_Steering = New_Sensor_Pos ( Steer_Angle[2] , LR_HomePos ) ;						Sensor_Id[4]++;Node_Id[24]++; break;
		
		case (RF_STEER): 	Steer_Angle[3] = CAN_SPI_READ(RxData);				RF_Steering = New_Sensor_Pos ( Steer_Angle[3] , RF_HomePos ) ;						Sensor_Id[5]++;Node_Id[25]++;	break;
		
		case (RR_STEER): 	Steer_Angle[4] = CAN_SPI_READ(RxData);	 			RR_Steering = New_Sensor_Pos ( Steer_Angle[4] , RR_HomePos ) ;						Sensor_Id[6]++;Node_Id[26]++; break;
		
		case (0x03):     Node_Id[28]++;
//										Right_roll_value = convertRawDataToFloat(RxData);
//										Right_pitch_value = convertRawDataToFloat(&RxData2[4]);
										Right_roll_value = RxData[0] << 8 | RxData[1];
										Right_pitch_value = RxData[2] << 8 | RxData[3];
										Right_roll_value = (uint16_t)Right_roll_value;
										Right_pitch_value = (uint16_t)Right_pitch_value;
										Right_Roll = Float16_To_Decimal(Right_roll_value);
										Right_Pitch = Float16_To_Decimal(Right_pitch_value);
										
										 Right_Roll=-(Right_Roll-180);
										Right_Roll_Final= (Right_Roll >180) ? (Right_Roll -360) : (Right_Roll<-180)? (Right_Roll+360):Right_Roll;
										Right_Pitch_Final = Right_Pitch;
										
										
												break;
//		case (LF_STEER): 	Steer_Angle[1] = CAN_SPI_READ(RxData);  		if ((Steer_Angle[1] <= 100 && Steer_Angle[1] >= 0 )|| (Steer_Angle[1] >= 640 && Steer_Angle[1] <= 720)){	LF_Steering = New_Sensor_Pos ( Steer_Angle[1] , LF_HomePos ) ; Node_Id[23]++;}	else {Steering_Boundary_Flag = SET;} break;
//		case (LR_STEER): 	Steer_Angle[2] = CAN_SPI_READ(RxData);			if ((Steer_Angle[2] <= 58 && Steer_Angle[2] >= 0) || (Steer_Angle[2] >= 598 && Steer_Angle[2] <= 720)){	LR_Steering = New_Sensor_Pos ( Steer_Angle[2] , LR_HomePos ) ;	Node_Id[24]++;} else {Steering_Boundary_Flag = SET;}break; 
//		case (RF_STEER): 	Steer_Angle[3] = CAN_SPI_READ(RxData);			if (Steer_Angle[3] >= 368 && Steer_Angle[3] <= 548)	{RF_Steering = New_Sensor_Pos ( Steer_Angle[3] , RF_HomePos ) ;	Node_Id[25]++;}	else {Steering_Boundary_Flag = SET;}break;
//		case (RR_STEER): 	Steer_Angle[4] = CAN_SPI_READ(RxData);	 		if (Steer_Angle[4] >= 408 && Steer_Angle[4] <= 588)	{RR_Steering = New_Sensor_Pos ( Steer_Angle[4] , RR_HomePos ) ;	Node_Id[26]++;} else {Steering_Boundary_Flag = SET;}break;
		default: break;
	}

			RxHeader.StdId=0;
	
	R_Roll_Filtered = Right_Contour_LPF(R_Roll, Prev_R_Roll_Filtered, ALPHA);
	Prev_R_Roll_Filtered = R_Roll_Filtered;
	
	R_Pitch_Filtered = Right_Contour_LPF (R_Pitch, Prev_R_Pitch_Filtered, ALPHA);
	Prev_R_Pitch_Filtered = R_Pitch_Filtered;
	
	LFS_Filtered = LFS_LPF(LF_Steering, Prev_LFS_Filtered, ALPHA);
	Prev_LFS_Filtered = LFS_Filtered;
	
	LRS_Filtered = LRS_LPF(LR_Steering, Prev_LRS_Filtered, ALPHA);
	Prev_LRS_Filtered = LRS_Filtered;
	
	RFS_Filtered = RFS_LPF(RF_Steering, Prev_RFS_Filtered, ALPHA);
	Prev_RFS_Filtered = RFS_Filtered;
	
	RRS_Filtered = RRS_LPF(RR_Steering, Prev_RRS_Filtered, ALPHA);
	Prev_RRS_Filtered = RRS_Filtered;
	
	BLE_Roll = BLE_Vertical_LPF(Right_Roll_Final, Prev_BLE_Roll, ALPHA);
	Prev_BLE_Roll = BLE_Roll;
	
	BLE_Pitch = BLE_Contour_LPF(Right_Pitch_Final, Prev_BLE_Pitch, ALPHA);
	Prev_BLE_Pitch = BLE_Pitch;
}

void Set_Motor_Position ( uint8_t Axis , float Position )
{
		
		CAN_Transmit(Axis,POSITION,Position,4,DATA); HAL_Delay(2);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan2)
{
	HAL_CAN_GetRxMessage(hcan2, CAN_RX_FIFO1, &RxHeader2, RxData2); // changed
	
	switch (RxHeader2.StdId)
	{
		case (IMU_SHEAR) : Shear_Roll = ((int16_t)(RxData2[1]<<8 | RxData2[0]))/16.0;	Shear_Pitch = ((int16_t)(RxData2[3]<<8 | RxData2[2]))/16.0;    Node_Id[27]++; break;
		
		case (FL_FLAP) : 	FL_Raw = CAN_SPI_READ(RxData2);      FL_Angle = New_Sensor_Pos (FL_Raw, FL_Home_Pos); 	Update_Array(Flap_Data_Array, ARRAY_SIZE, FL_Angle);				Node_Id[28]++; break;
		
		case (FR_FLAP) : 	FR_Raw = CAN_SPI_READ(RxData2);      FR_Angle = New_Sensor_Pos (FR_Raw, FR_Home_Pos);		Update_Array(Flap_Data_Right, ARRAY_SIZE, FR_Angle);     	Node_Id[29]++; break;
		
		case (RL_FLAP) : 	RL_Raw = CAN_SPI_READ(RxData2);      RL_Angle = New_Sensor_Pos (RL_Raw, RL_Home_Pos);				Node_Id[30]++; break;
		
		case (RR_FLAP) :  RR_Raw = CAN_SPI_READ(RxData2);      RR_Angle = New_Sensor_Pos (RR_Raw, RR_Home_Pos);				Node_Id[31]++; break;
		
		default:       break;
	
	}
	
	Received_Node_Id = RxHeader2.StdId >> 5;
	Received_Command_Id = RxHeader2.StdId & CMD_MASK;
		
	switch( Received_Command_Id )
	{
		case HEARTBEAT:  							Node_Id[Received_Node_Id]++;    Axis_State[Received_Node_Id] = RxData2[4]; break;
		
		case ENEST_ID:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB);  Absolute_Position_Reception(Received_Node_Id); 	   break;		

		//case SENS_EST:  							Motor_Velocity[Received_Node_Id]	= CAN_Reception(MSB); 				  			 	 break;
		
		case MERR_ID:  								Motor_Error[Received_Node_Id]			= CAN_Reception(MSB); 								 	 break;
		
		case ENERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(MSB); 					 			 	 break;
		
		case SNERR_ID:  							Encoder_Error[Received_Node_Id]		= CAN_Reception(LSB); 					  		 	 break;
		
		case IQM_ID:  								Motor_Current[Received_Node_Id]		= fabs(CAN_Reception(MSB)); 					  		 	 break;
		
		case VOLTAGE: 								memcpy(&Rover_Voltage, RxData2, 4);	 																		 	 break;
		
		case FET_TEMP:                FET_Temperature[Received_Node_Id] = CAN_Reception(LSB); 					  		 	 break;

		default: 																																													  	 	 break;

	}
	
	Rover_Velocity = (fabs(Motor_Velocity[2]) + fabs(Motor_Velocity[3]) + fabs(Motor_Velocity[4])) / 3;
	
	
	FL_LPF_Angle = FL_Flap_LPF(Flap_Angle_Left, Prev_FL_LPF_Angle, ALPHA);
	Prev_FL_LPF_Angle = FL_LPF_Angle;
	
	FR_LPF_Angle = FR_Flap_LPF(Flap_Angle_Right, Prev_FR_LPF_Angle, ALPHA);
	Prev_FR_LPF_Angle = FR_LPF_Angle;
	
	RL_LPF_Angle = RL_Flap_LPF(RL_Angle, Prev_RL_LPF_Angle, ALPHA);
	Prev_RL_LPF_Angle = RL_LPF_Angle;
	
	RR_LPF_Angle = RR_Flap_LPF(RR_Angle, Prev_RR_LPF_Angle, ALPHA);
	Prev_RR_LPF_Angle = RR_LPF_Angle;
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
////  MX_UART4_Init();
////  MX_UART5_Init();
  MX_TIM14_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	BUZZER_ON;
	HAL_Delay(1000);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	HAL_CAN_Start(&hcan2);HAL_Delay(1000);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
	
	HAL_Delay(3000);
//	for ( uint8_t i = 6 ; i < 25 ; i++ ) {	Start_Calibration_For (i, 8, 10); }
//	for ( uint8_t i = 1 ; i < 5; i++ ) { Start_Calibration_For (6, 8, 5);Start_Calibration_For (13, 8, 5);Start_Calibration_For (12, 8, 5);Start_Calibration_For (14, 8, 5);}
	
	
	/* UART INITS */
	MX_UART4_Init();
	MX_UART5_Init();
	//HAL_UART_Receive_IT(&huart5,BT_Rx ,sizeof(BT_Rx));
	HAL_UART_Receive_DMA(&huart5,BT_Rx ,sizeof(BT_Rx));
	
	HAL_UART_Receive_IT(&huart4,(uint8_t*)Rx_Data ,sizeof(Rx_Data));
	/* UART INITS */
	
//	Left_IMU_State = ( Sensor_Id[1] == 0 || Sensor_Id[2]  == 0 ) ? NULL : SET ;
//	if ( !Left_IMU_State ) Error_Handler();
//	for(uint8_t i=1 ; i < 5 ; i++) 
//	{
//	CAN_Transmit(i,VEL_LIMIT,20,4,DATA);
//	}
//	
//					for(uint8_t i=1 ; i < 5 ; i++) 
//					{
//						CAN_Transmit(i,VEL_LIMIT,30,4,DATA);
//						HAL_Delay(20);
//						Prev_Vel_Limit = 30;
//					}
//	HAL_Delay(1500);
//	Reboot(4);HAL_Delay(1500);
//	while ( Axis_State[1] != 8 || Axis_State[2] != 8 || Axis_State[3] != 8 || Axis_State[4] != 8  )
//	{
		
//			if ( Axis_State[1] != 8 ) {Reboot(1); HAL_Delay(2500);}
//			if ( Axis_State[2] != 8 ) {Reboot(2); HAL_Delay(2500);}
//			if ( Axis_State[3] != 8 ) {Reboot(3); HAL_Delay(2500);}
//			if ( Axis_State[4] != 8 ) {Reboot(4); HAL_Delay(2500);}
//	}
//Lower_Width_Motor_Count = Upper_Width_Motor_Count = 500;
//memcpy(&Write_Value[0], &Lower_Width_Motor_Count, sizeof(Lower_Width_Motor_Count));
//	memcpy(&Write_Value[4], &Upper_Width_Motor_Count, sizeof(Upper_Width_Motor_Count));
//	
//	EEPROM_Write(3, 0, (uint8_t *)Write_Value, sizeof(Write_Value));
	
	Steering_Reset_Flag = SET;
	LF_Speed= LR_Speed=RF_Speed=RR_Speed=0;
//	for (int i = 1; i <= 256; i++)
//	{
//	EEPROM_PageErase(i);
//	}
//Test_Write=11;
//EEPROM_Write(25,0, (uint8_t *)Test_Write,sizeof(Test_Write));
//HAL_Delay(2000);
//EEPROM_Read(25,0, (uint8_t *)Test_Read,sizeof(Test_Read));
								
	Prev_Write_Value[0] = 0xFE;



// Read_EEPROM_Data();	
for(int i=1;i<4;i++){Read_EEPROM_Data();	HAL_Delay(50);}	

//Lower_Width_Motor_Value = 0;	
//Upper_Width_Motor_Value = 0;
//Vertical_Motor_Value=0;
//Pitch_Arm_Motor_Value = 0;
//Left_Macro_Motor_Value = 0;
//Right_Macro_Motor_Value = 0;

//for (uint8_t i = 1; i < 21; i++)
//{
//	if (Read_Value[i] == 0)
//	{
//		eeprom++;
//	}
//}

//if (eeprom == 21)
//{
//	EEPROM_Error = SET;
//	Emergency_Stop();
//}
	//HAL_Delay(10000);
	BUZZER_OFF;
	
//	Set_Motor_Velocity(LFS, -15);
//	Set_Motor_Velocity(LRS, -15);
//	HAL_Delay(3000);
//	Set_Motor_Velocity(LFS, 0);
//	Set_Motor_Velocity(LRS, 0);
//	HAL_Delay(3000);
	//Initiate_Process = SET;
//	Error_Handler();
//HAL_TIM_Base_Start_IT(&htim14);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(6000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
	
		test++;

		BT_State = BT_READ;
		//Initial_Msg();
		Joystick_Reception();
		EEPROM_Store_Data();
////////////					Drive_Wheel_Controls_Vel_Based();
////////////					New_Steering_Controls();
		Operations_Monitor();
		
		
		if(OPERATION_MONITOR_FLAG==NULL)
		{
////////			Flap_Sensing();
			Drive_Wheel_Controls_Vel_Based();
////Left_Frame_Controls();
			New_Steering_Controls();
		//Frame_Controls_Sensor_BLE();
		//All_Macro_Sensing();
			Frame_Controls();
////////			Dynamic_Width_Adjustment();
			Shearing_Motors();
			Macro();
			Pitch_Arm_Control_IMU();
			
			
////////


//			Frame_Manual_Controls();
			
//	if (Cont != Cont_temp)
//	{
//		for(uint8_t i=0; i<3 ; i++) Set_Motor_Velocity(Contour, Cont);
//		
//		Cont_temp = Cont;
//	}
		}
	else{Emergency_Stop();}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//		if (Pitch_Arm_Speed_Temp != Pitch_Arm_Speed)
//	{
//			//Input_Velocity[13] = Right_Macro_Speed;
//			Set_Motor_Velocity(14, Pitch_Arm_Speed);
//			Pitch_Arm_Speed_Temp = Pitch_Arm_Speed;
//	}
//	
//	Current_Vel = Pitch_Arm_Speed;
//	
//	if (Prev_Vel != 0 && Current_Vel == 0)
//	{
////		if (HAL_GetTick() - Dummy_Tick_2 >= 100)
////		{
//			Start_Calibration_For(14, 1, 5);
//			//Dummy_Tick_2 = HAL_GetTick();
//		//}
//		
//		if (HAL_GetTick() - Dummy_Tick >= 200)
//		{
//			Start_Calibration_For(14, 8, 5);
//			Dummy_Tick = HAL_GetTick();
//		}
//		Prev_Vel = Current_Vel;
//	}
//	
//	else
//	{
//		Prev_Vel = Current_Vel;
//		Dummy_Tick = 0;
//		Dummy_Tick_2 = 0;
//	}
	
	
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 0;	// which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14; //14 // how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 12;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
	CAN_FilterTypeDef canfilterconfig;

	canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
	canfilterconfig.FilterBank = 14; //14 // which filter bank to use from the assigned ones
	canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	canfilterconfig.FilterIdHigh = 0x000<<5;
	canfilterconfig.FilterIdLow = 0;
	canfilterconfig.FilterMaskIdHigh = 0x000<<5;
	canfilterconfig.FilterMaskIdLow = 0x0000;
	canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
	canfilterconfig.SlaveStartFilterBank = 14;//14	// how many filters to assign to the CAN1 (master can)

	HAL_CAN_ConfigFilter(&hcan2, &canfilterconfig);

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 9000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_1_Pin|LED_2_Pin|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, B1_Pin|Buzzer_1_Pin|Buzzer_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_1_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LED_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_2_Pin PC6 */
  GPIO_InitStruct.Pin = LED_2_Pin|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : UART4_State_Pin */
  GPIO_InitStruct.Pin = UART4_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART4_State_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin Buzzer_1_Pin Buzzer_2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|Buzzer_1_Pin|Buzzer_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : UART5_State_Pin */
  GPIO_InitStruct.Pin = UART5_State_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(UART5_State_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_Transmit ( uint8_t NODE, uint8_t Command, float Tx_Data,	uint8_t Data_Size, uint8_t Frame_Format)
{
//uint64_t BUFF;
	TxHeader.ExtId = NULL;
	
	TxHeader.TransmitGlobalTime = DISABLE;
	
	TxHeader.IDE = CAN_ID_STD;
	
	TxHeader.DLC	= Data_Size;
	
	TxHeader.RTR = (Frame_Format == REMOTE) ? (CAN_RTR_REMOTE) : (Frame_Format == DATA) ? (CAN_RTR_DATA):(CAN_RTR_REMOTE);
	
	switch (Command)
	{
		case VELOCITY:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| VEL_ID;
									break;
		
		case POSITION:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| POS_ID;
									break;
		
		case TORQUE:	
									memcpy (TxData, &Tx_Data, Data_Size);					
									TxHeader.StdId = (NODE << 5)| TRQ_ID;
							//	HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
									break;
		
		case VEL_LIMIT:	

									//TxData[6] = 0x20; for 40 amps//8C - 70//0x70-60 amps
									TxData[6] = 0x8C;
									TxData[7] = 0x42;
									memcpy (TxData, &Tx_Data, 4);	
									TxHeader.DLC	= 8;
									TxHeader.StdId = (NODE << 5)| 0x00F;
									break;
		
		case MOT_ERROR: 	
									TxHeader.StdId = (NODE << 5)| MERR_ID;
									break;
		
		case ENC_ERROR:					
									TxHeader.StdId = (NODE << 5)| ENERR_ID;
									break;
		
		case SNL_ERROR:					
									TxHeader.StdId = (NODE << 5)| SNERR_ID;
									break;
		
		case IQ:						
									TxHeader.StdId = (NODE << 5)| IQM_ID;
									break;
		
		case ENC_EST:					
									TxHeader.StdId = (NODE << 5)| ENEST_ID;
									break;
									
		case T_RAMP:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| T_RAMP_ID;
									break;
		
		case SENSL_EST:					
									memcpy (TxData, &Tx_Data, Data_Size);	
									TxHeader.StdId = (NODE << 5)| 0x015;
									break;
		
		case REQ_STATE:	 
									break;
		
		case 0x017:
									TxHeader.StdId = (NODE << 5)| 0x017;
									break;
		
		default: break;
		

	}
//	
		for ( uint8_t i=0 ; i<2; i++ ) //1
	{

				HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox); 
				//if ( Initiate_Process) HAL_Delay(1);
				CAN_Count++;
	}
	
	for ( uint8_t i=0 ; i<8; i++ ) 
	{
		TxData[i] = 0;
	}
				

}
float New_Sensor_Pos(double Sensor_Value, double Zero_Pos)
{
double output;

output=((Sensor_Value-Zero_Pos)>360.0)? ((Sensor_Value-Zero_Pos)-720.0) : (Sensor_Value-Zero_Pos);
output = (output<-359.0)?(output+720.0):(output);
return (output/2);

	
/*output=((sensorvalue-zero_pos)>(360.0/2))? ((sensorvalue-zero_pos)-(720.0/2)) : (sensorvalue-zero_pos);
output = (output<-(359.0/2))?(output+(720.0/2)):(output);
return (output);*/
}
void Start_Calibration_For (int axis_id, int command_id, uint8_t loop_times)
{
				memcpy(TxData, &command_id, 4);		
				TxHeader.DLC = 4;	
				TxHeader.IDE = CAN_ID_STD;
				TxHeader.RTR = CAN_RTR_DATA;
				TxHeader.StdId = ( axis_id <<5) | 0x007 ;	
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);HAL_Delay(20); 		
}
void Drives_Error_Check(void)
{

	for(uint8_t i = 1; i < 5; i++)
	{
		if ( i != 5 ){ if ( Axis_State[i] != 8 ){ DRIVES_ERROR_FLAG = SET; Heal_Error(i); HAL_Delay(10);
			 } //HAL_Delay(3); 
		}
		
		//HAL_Delay(3);
	}
}
void Reboot (int Axis)
{	
	TxHeader.DLC = 4;	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = ( Axis <<5) | 0x016 ;	
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
	//HAL_Delay(1); 		
}
void Clear_Error(int Axis)
{
	TxHeader.DLC = 0;	
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = ( Axis <<5) | 0x018 ;	
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
}
void Set_Motor_Torque ( uint8_t Axis , float Torque )
{
	Torque =  (Axis==2)  ? -Torque : Torque ;	

	CAN_Transmit(Axis,TORQUE,Torque,4,DATA); HAL_Delay(3);//10
}
void Set_Motor_Velocity ( uint8_t Axis , float Velocity )
{
	
	if (Steering_Mode == 2)
	{
		Velocity = (Axis == 1 || Axis == 2) ? -Velocity : Velocity;   //for zero turn
	}
	Velocity = (Axis == 9 || Axis == 2) ? -Velocity : Velocity;
	CAN_Transmit(Axis,VELOCITY,Velocity,4,DATA); HAL_Delay(1);
}


void Stop_Motors(void)
{
			for(uint8_t i = 1; i <= 4; i++){Set_Motor_Torque(i, 0);}	for(uint8_t i = 8; i <= 11; i++){Set_Motor_Velocity(i, 0);}
LF_Speed_Temp = RF_Speed_Temp = LR_Speed_Temp = RR_Speed_Temp = 0;
}

void Read_EEPROM_Data(void)

{
	//Lower_Width_Motor_Count = -1000;	
//Upper_Width_Motor_Count = 188;	
//					 memcpy(&Write_Value[0], &Lower_Width_Motor_Count, sizeof(Lower_Width_Motor_Count));
//memcpy(&Write_Value[4], &Upper_Width_Motor_Count, sizeof(Upper_Width_Motor_Count));						
//	EEPROM_Write(3, 0, (uint8_t *)Write_Value, sizeof(Write_Value));
					
//	EEPROM_PageErase (3);				
	EEPROM_Read(60, 0, (uint8_t *)Read_Value, sizeof(Read_Value));

	memcpy(&Vertical_Motor_Value, &Read_Value[1],4 );
	memcpy(&Left_Macro_Motor_Value, &Read_Value[5],4 );
	memcpy(&Right_Macro_Motor_Value, &Read_Value[9],4 );
	//memcpy(&Pitch_Arm_Motor_Value, &Read_Value[13],4 );
	memcpy(&Lower_Width_Motor_Value, &Read_Value[13],4 );
	memcpy(&Upper_Width_Motor_Value, &Read_Value[17],4 );

	
	
//	if(Read_Value[0]>=600) 
//	{memcpy(&Lower_Width_Motor_Value, &Read_Value_1[28],4 );}	 
//  if(Read_Value[4]>=600)	
//	{memcpy(&Upper_Width_Motor_Value, &Read_Value_1[4],4 );}
//	if(Read_Value[8]>=600)
//	{memcpy(&Left_Arm_Motor_Value, &Read_Value_1[8],4 );}
//	if(Read_Value[12]>=600)
//	{memcpy(&Right_Arm_Motor_Value, &Read_Value_1[12],4 );}
//	if(Read_Value[16]>=600)
//	{memcpy(&Pitch_Arm_Motor_Value, &Read_Value_1[16],4 );}
//	if(Read_Value[20]>=600)
//	{memcpy(&Right_Vertical_Motor_Value, &Read_Value_1[20],4 );}
//	if(Read_Value[24]>=600)
//	{	memcpy(&Contour_Motor_Value, &Read_Value_1[24],4 );}
	
	
//	Lower_Width_Motor_Value 		 = Lower_Width_Motor_Value 			== -1 ? 0 : Lower_Width_Motor_Value;
//	Upper_Width_Motor_Value 		 = Upper_Width_Motor_Value 			== -1 ? 0 : Upper_Width_Motor_Value;
//	Left_Arm_Motor_Value    		 = Left_Arm_Motor_Value    			== -1 ? 0 : Left_Arm_Motor_Value;
//	Right_Arm_Motor_Value  		   = Right_Arm_Motor_Value   			== -1 ? 0 : Right_Arm_Motor_Value;
//	Pitch_Arm_Motor_Value   		 = Pitch_Arm_Motor_Value   			== -1 ? 0 : Pitch_Arm_Motor_Value;
//	Vertical_Motor_Value   = 		Vertical_Motor_Value   == -1 ? 0 : Right_Vertical_Motor_Value;
//	Contour_Motor_Value   			 = Contour_Motor_Value          == -1 ? 0 : Contour_Motor_Value;

}
void Heal_Error(uint8_t Axis_Id)
{
	BUZZER_ON;
	Stop_Motors();
	
	while ( Axis_State[Axis_Id] != 8 )
	{
		Reboot(Axis_Id);	HAL_Delay(2000);
		//Start_Calibration_For ( Axis_Id,  8 , 2 ); HAL_Delay(1500);
	}
	
	DRIVES_ERROR_FLAG = NULL;
	BUZZER_OFF;
}

void Joystick_Reception(void)
{
	/*				JOYSTICK VALUES ASSIGNING								*/
	if (( BT_Rx[0] == 0xAA ) &&	( BT_Rx[8] == 0xFF ))
	{	
		Mode 						 = BT_Rx[1];
		Speed 					 = BT_Rx[2]  != 0 ? BT_Rx[2] : Speed ;
		Steering_Mode 	 = BT_Rx[3];
		Pot_Angle        = BT_Rx[4]; 
		Pot_Angle = 180 - Pot_Angle;    //Switching direction of rover
		Joystick         = BT_Rx[5];
		Joystick = Joystick == 1 ? 2 : Joystick == 2 ? 1 : Joystick;   //Switching Directin of rover
		Shearing				 = BT_Rx[6];
		Heartbeat        = BT_Rx[7];

		Speed_Ref = BT_Rx[2];
		if( Steering_Mode == 0 ) Steering_Mode=1;
	
		/*				Steering Reset on every Steering Mode Change								*/
		if ( BT_Steer_Temp != BT_Rx[3])
		{	
			if ( Steering_Mode_Temp != Steering_Mode )
			{
				Steering_Reset_Flag = SET;
				Steering_Mode_Temp=Steering_Mode;
			}
			BT_Steer_Temp = BT_Rx[3];
		}
	/*				Steering Reset on every Steering Mode Change			  				*/
		
		if (Speed_Ref == 0)
	{
		if(HAL_GetTick()- speed_time >= 2000)
		{
			E_Stop = SET;
			speed_time = HAL_GetTick();
		}
	}
	else{  E_Stop = NULL; speed_time = HAL_GetTick();	}
	
	
	}
	else {}
		
		
	if (HAL_GetTick() - Joystick_Tick >= 1000)
	{
		Joystick_Disconnected = Heartbeat == Heartbeat_Temp ? SET : NULL;
		Heartbeat_Temp = Heartbeat;
		Joystick_Tick = HAL_GetTick();
	}
		
//	//Rover_Voltage = 54;
//	if((HAL_GetTick() - Timt_Batt) >= 3000)
//	{
//		Tx_Voltage = Rover_Voltage < 44 ? 0 : 1;
//		HAL_UART_Transmit_DMA(&huart5, &Tx_Voltage, sizeof(Tx_Voltage));
//		Timt_Batt = HAL_GetTick();
//	}
}
void Manual_Wheel_Control(void)
{ 
	if(All_Whel_SameTorque==1){
		
//	for(uint8_t i=1;i<5;i++){
//		if(Manua+6l_Vel_Limit != Manual_Vel_Limit_Temp)	{
//			   CAN_Transmit(i,VEL_LIMIT,Manual_Vel_Limit,4,DATA);HAL_Delay(10);}
//	   	   Manual_Vel_Limit_Temp = Manual_Vel_Limit;
//		if(Manual_Torque != Manual_Torque_Temp ) 
//			{
//	       Set_Motor_Torque (i , Manual_Torque );HAL_Delay(10);}
//         Manual_Torque_Temp = Manual_Torque;	
//			
//	}
				if ( Manual_Torque_Temp != Manual_Torque )
		{
			for(uint8_t i = 1; i < 5 ; i++ ) {Set_Motor_Torque ( i , Manual_Torque); HAL_Delay(1);} 
			Manual_Torque_Temp = Manual_Torque;
		}
		if(Manual_Vel_Limit != Manual_Vel_Limit_Temp)
		{
			   for(int i=1;i<=4;i++){ CAN_Transmit(i,VEL_LIMIT,Manual_Vel_Limit,4,DATA);HAL_Delay(10); }
	   	   Manual_Vel_Limit_Temp = Manual_Vel_Limit;
		}
		
		
}
	else if(All_Whel_SameTorque==0){
		
  if(Manual_Torque_1 != Manual_Torque_Temp_1 ){
	      Set_Motor_Torque (Manual_Axis_1 , Manual_Torque_1 );HAL_Delay(1);
	      Manual_Torque_1 = Manual_Torque_Temp_1;}
	if(Manual_Vel_Limit_1 != Manual_Vel_Limit_Temp_1){
	      CAN_Transmit(Manual_Axis_1,VEL_LIMIT,Manual_Vel_Limit_1,4,DATA);HAL_Delay(1);
	      Manual_Vel_Limit_1 = Manual_Vel_Limit_Temp_1;
	}
	
	  if(Manual_Torque_2 != Manual_Torque_Temp_2 || Manual_Vel_Limit_2 != Manual_Vel_Limit_Temp_2 ){
	      Set_Motor_Torque (Manual_Axis_2 , Manual_Torque_2 );HAL_Delay(1);
			  Manual_Torque_2 = Manual_Torque_Temp_2;
	      CAN_Transmit(Manual_Axis_2,VEL_LIMIT,Manual_Vel_Limit_2,4,DATA);HAL_Delay(1);
	      Manual_Vel_Limit_2 = Manual_Vel_Limit_Temp_2;
	}
		
		 if(Manual_Torque_3 != Manual_Torque_Temp_3 || Manual_Vel_Limit_3 != Manual_Vel_Limit_Temp_3 ){
	         Set_Motor_Torque (Manual_Axis_3 , Manual_Torque_3 );HAL_Delay(1);
			     Manual_Torque_3 = Manual_Torque_Temp_3;
	         CAN_Transmit(Manual_Axis_3,VEL_LIMIT,Manual_Vel_Limit_3,4,DATA);HAL_Delay(1);	       
	         Manual_Vel_Limit_3 = Manual_Vel_Limit_Temp_3;
	 }
		
		  if(Manual_Torque_4 != Manual_Torque_Temp_4 || Manual_Vel_Limit_4 != Manual_Vel_Limit_Temp_4 ){
	          Set_Motor_Torque (Manual_Axis_4 , Manual_Torque_4 );HAL_Delay(1);
				    Manual_Torque_4 = Manual_Torque_Temp_4;
	          CAN_Transmit(Manual_Axis_4,VEL_LIMIT,Manual_Vel_Limit_4,4,DATA);HAL_Delay(1);
	          Manual_Vel_Limit_4 = Manual_Vel_Limit_Temp_4;
		}
	
	}
}

void Manual_Sensing_Control(void)
{
if(Mode==1)
{
	if(Joystick!=Joystick_Temp){
	L_Arm_Speed=(Left_Arm_Motor_Count<-1)?5:Left_Arm_Motor_Count>1?-5:0;
		R_Arm_Speed=(Right_Arm_Motor_Count<-1)?5:Right_Arm_Motor_Count>1?-5:0;
		Pitch_Arm_Speed=(Pitch_Arm_Motor_Count<-1)?5:Pitch_Arm_Motor_Count>1?-5:0;
		Joystick_Temp=Joystick;}
}
else if(Mode==2)
{
	if(Joystick!=Joystick_Temp){
	if(Joystick==3) {L_Arm_Speed= R_Arm_Speed=5;Pitch_Arm_Speed=4;}
	else if( Joystick==4) {L_Arm_Speed= R_Arm_Speed=-5;Pitch_Arm_Speed=4;}
	else {L_Arm_Speed= R_Arm_Speed=Pitch_Arm_Speed=0;}
	Joystick_Temp=Joystick;}
}

else if(Mode==3){
	if(Joystick!=Joystick_Temp){
	if(Joystick==3) {L_Arm_Speed=5; R_Arm_Speed=-5;}
	else if(Joystick==4) {L_Arm_Speed=-5; R_Arm_Speed=5;}
	else {L_Arm_Speed= R_Arm_Speed=Pitch_Arm_Speed=0;}
		Joystick_Temp=Joystick;}
}

else{}

L_Arm_Speed=((Left_Arm_Motor_Count>=20)||(Left_Arm_Motor_Count<=-20))?0:L_Arm_Speed;
R_Arm_Speed=((Right_Arm_Motor_Count>=20)||(Right_Arm_Motor_Count<=-20))?0:R_Arm_Speed;
Pitch_Arm_Speed=((Pitch_Arm_Motor_Count>=16)||(Pitch_Arm_Motor_Count<=-16))?0:Pitch_Arm_Speed;

if(L_Arm_Speed!=L_Arm_Speed_Temp){
	Set_Motor_Velocity( 12,L_Arm_Speed);
	L_Arm_Speed_Temp=L_Arm_Speed;
}

if(R_Arm_Speed!=R_Arm_Speed_Temp){
	Set_Motor_Velocity( 13,R_Arm_Speed);
	R_Arm_Speed_Temp=R_Arm_Speed;
}

if(Pitch_Arm_Speed!=Pitch_Arm_Speed_Temp){
	Set_Motor_Velocity( 13,Pitch_Arm_Speed);
	Pitch_Arm_Speed_Temp=Pitch_Arm_Speed;
}


}
void Manual_Controls (void)
{
	/*------------------------------WHEEL_MOTORS----------------------------------- */
	if ( Left_Wheels_Torque != Left_Wheels_Torque_Temp )
	{
		CAN_Transmit(1,VEL_LIMIT,30,4,DATA);HAL_Delay(1);
		CAN_Transmit(2,VEL_LIMIT,30,4,DATA);HAL_Delay(1);
		Set_Motor_Torque ( 1 , Left_Wheels_Torque );HAL_Delay(1);
		Set_Motor_Torque ( 2 , Left_Wheels_Torque );
		
		Left_Wheels_Torque_Temp = Left_Wheels_Torque;
	}
	
	/*------------------------------WHEEL_MOTORS----------------------------------- */
	/*------------------------------FRAME_MOTORS----------------------------------- */
	
//		if( L_Vert_Speed_Temp != L_Vert_Speed ) 																															// checking if the new value is not equal to old value
//			{
//				Set_Motor_Velocity (LVert , L_Vert_Speed );
//				L_Vert_Speed_Temp = L_Vert_Speed ;																																// Overwriting old value with new value. 
//			}		

		if( R_Vert_Speed_Temp != R_Vert_Speed ) 																															// checking if the new value is not equal to old value
			{
				Set_Motor_Velocity (RVert , R_Vert_Speed );	
				R_Vert_Speed_Temp = R_Vert_Speed ;																																// Overwriting old value with new value.
			} 

		if( Contour_Speed_Temp != Contour_Speed ) 																														// checking if the new value is not equal to old value
			{
				Set_Motor_Velocity (Contour , Contour_Speed );
				Contour_Speed_Temp = Contour_Speed ;																															// Overwriting old value with new value.
			}	
	/*------------------------------FRAME_MOTORS----------------------------------- */
	/*------------------------------ARM_MOTORS------------------------------------- */
		if( L_Arm_Speed_Temp != L_Arm_Speed )
		{
			Set_Motor_Velocity (L_Arm , L_Arm_Speed );	
			L_Arm_Speed_Temp = L_Arm_Speed ;
		}
		
		if( R_Arm_Speed_Temp != R_Arm_Speed )
		{
			Set_Motor_Velocity (R_Arm , R_Arm_Speed );	
			R_Arm_Speed_Temp = R_Arm_Speed ;
		}
		if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed ) 
		{
			Set_Motor_Velocity (P_Arm , Pitch_Arm_Speed );	
			Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
		}
	/*------------------------------ARM_MOTORS------------------------------------- */
	/*---------------------------STEERING_MOTORS----------------------------------- */
	if ( LF_Speed_Temp != LF_Speed )
	{
		LF_Speed= LF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LF_Speed; 
		Set_Motor_Velocity( LFS , -LF_Speed );
		LF_Speed_Temp = LF_Speed ;
	}
	
	if ( LR_Speed_Temp != LR_Speed )
	{
		LR_Speed= LR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LR_Speed;
		 Set_Motor_Velocity( LRS , -LR_Speed );
		LR_Speed_Temp = LR_Speed ;
	}
	if ( RF_Speed_Temp != RF_Speed )
	{	
		RF_Speed= RF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RF_Speed;
		Set_Motor_Velocity( RFS , -RF_Speed );
		RF_Speed_Temp = RF_Speed ;
	}

	if ( RR_Speed_Temp != RR_Speed )
	{
		RR_Speed= RR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RR_Speed;
		Set_Motor_Velocity( RRS , -RR_Speed );
		RR_Speed_Temp = RR_Speed ;
	}
	/*---------------------------STEERING_MOTORS----------------------------------- */
	
	/*------------------------------WIDTH_MOTORS----------------------------------- */
//	if ( Width_Motor_Speed != Width_Motor_Temp)
//	{
//		Set_Motor_Velocity ( Upper_Width, Width_Motor_Speed); 
//	//	Set_Motor_Velocity ( Lower_Width, Width_Motor_Speed);
//		Width_Motor_Temp = Width_Motor_Speed; 
//	}
	/*------------------------------WIDTH_MOTORS----------------------------------- */
	


}
void Drive_Wheel_Controls(void)
{
	/* Actuates the Drive Wheels only when the Steering Resets have been completed and the Bluetooth is in Connection. */

//	if ( (!Steering_Reset_Flag) && (Speed!= 0) && (BT_State)) // ----> ACUTAL CONDITION
	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
  {
		
//  if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) ) {Joystick = 0; }// Safety STOP  }
		
//	Vel_Limit = Joystick == 0 ? 20 : Speed * 30;
//	Vel_Limit = Joystick == 0 ? 20 : Speed * 16; // actual
		//Vel_Limit = Speed == 1 ? 25 : Speed == 2 ? 35 : Speed == 3 ? 45: Speed;
		if ( Joystick == 1 )
		{
		if (Speed == 1 && Motor_Velocity[3] > 15 ) Vel_Limit = 30;
		if ( Speed == 2 && Motor_Velocity[3] > 15) Vel_Limit = 45;
		if ( Speed == 3 && Motor_Velocity[3] > 15) Vel_Limit = 65;
		}
		else if ( Joystick == 2 )
		{
		if (Speed == 1 && Motor_Velocity[3] < -15 ) Vel_Limit = 30;
		if ( Speed == 2 && Motor_Velocity[3] < -15) Vel_Limit = 45;
		if ( Speed == 3 && Motor_Velocity[3] < -15) Vel_Limit = 65;
		}
		
		
		Vel_Limit = Joystick == 0 ? 20 : Vel_Limit;
		
		if ( Motor_Velocity[3] < 15 && Motor_Velocity[3] > -15 )
		{
			if ( Joystick_Temp != Joystick )
			{
				switch (Joystick)
				{
					case 0 :   Torque =  NULL; 							break;								
					case 1 :   Torque =	 Wheel_Torque;			break; 
					case 2 :   Torque = -Wheel_Torque;			break; 
					default:																break;
				}
				
				if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
				{
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{
						Torque = (i==3 )  ? -Torque : Torque ;   
						Set_Motor_Torque ( i , Torque );
					}	
				}
				else
				{     
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{ 
							Set_Motor_Torque ( i , Torque );
					}
				}
				Joystick_Temp = Joystick;
			}
		}
			
		
			if ( (Vel_Limit_Temp != Vel_Limit) && Vel_Limit != 0  )
			{
			
				if( Prev_Vel_Limit > Vel_Limit ) 	
				{ for(uint8_t i = 8; i <= 11; i++){Set_Motor_Velocity(i, 0);}
				 for ( uint8_t  v = Prev_Vel_Limit-2; v >= Vel_Limit  ; v=v-2 )
				 {	
					 for(uint8_t i=1 ; i < 5 ; i++) 
					{
						if(v>10){CAN_Transmit(i,VEL_LIMIT,v,4,DATA);HAL_Delay(3);}
					}
					HAL_Delay(50);
					 
				 }		
				 Prev_Vel_Limit = Vel_Limit ;
				}
				else
				{for(uint8_t i = 8; i <= 11; i++){Set_Motor_Velocity(i, 0);}
				 for ( uint8_t v = Prev_Vel_Limit+2; v <= Vel_Limit  ; v=v+2 )
				 {	
					 for(uint8_t i=1 ; i < 5 ; i++) 
					{
						if(v>10){CAN_Transmit(i,VEL_LIMIT,v,4,DATA);HAL_Delay(3);}
					}
					HAL_Delay(100);
				 }
				 Prev_Vel_Limit = Vel_Limit ;
				}
			 Vel_Limit_Temp = Vel_Limit;
			}
			
		
		if ( Motor_Velocity[3] > 15 || Motor_Velocity[3] < -15 )
		{
			if ( Joystick_Temp != Joystick )
			{
				switch (Joystick)
				{
					case 0 :   Torque =  NULL; 							break;								
					case 1 :   Torque =	 Wheel_Torque;			break; 
					case 2 :   Torque = -Wheel_Torque;			break; 
					default:																break;
				}
				
				if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
				{
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{
						Torque = (i==3 )  ? -Torque : Torque ;   
						Set_Motor_Torque ( i , Torque );
					}	
				}
				else
				{     
					for ( uint8_t i = 1 ; i < 5 ; i++ )
					{ 
							Set_Motor_Torque ( i , Torque );
					}
				}
				Joystick_Temp = Joystick;
			}
		}
			
		if ( (!BT_State ) && Joystick != 0 )
		{
			for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL );}
			Joystick = 0 ;
			BUZZER_ON;
		}
	
	}
}
void Transmit_Motor_Torque (void)
{
		if ( Joystick_Temp != Joystick )
		{
			switch (Joystick)
			{
				case 0 :   Torque = NULL; 							break;								
				case 1 :   Torque =	Wheel_Torque;				break; 
				case 2 :   Torque =-Wheel_Torque;				break; 
				default :																break;
			}
			Joystick_Temp = Joystick;
		}
		
		if ( Torque_Temp != Torque )
		{
			if ( Steering_Mode == ZERO_TURN ) //Mode - 3 : zero turn
			{
				for ( uint8_t i = 1 ; i < 5 ; i++ )
				{
				 Torque = (i==3 ) ? -Torque : Torque ;   
				 Set_Motor_Torque ( i , Torque );
				}	
			}
			else
			{
			 for ( uint8_t i = 1 ; i < 5 ; i++ )
			 { 
				Set_Motor_Torque ( i , Torque ); 
			 }
			}
			Torque_Temp = Torque;
		}
		Prev_Joystick = Joystick;
}
void New_Drive_Controls(void)
{
	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
	{
		//Rover_Velocity = (fabs(Motor_Velocity[1]) + fabs(Motor_Velocity[2]) + fabs(Motor_Velocity[3]) + fabs(Motor_Velocity[4])) / 4;
//		Vel_Limit = Speed*15;
//		Vel_Limit = Vel_Limit > 50 ? 50 : Vel_Limit < 10 ? 10 : Vel_Limit;

	//	if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) || (Left_Vertical_Error > 6 || Left_Vertical_Error < -6) ){ Joystick = 0;}// Stop_Motors(); }// Safety STOP  (L_R_Err > 5 || L_R_Err < -5)
	
//		if ( Joystick == 1 )
//		{
//		if ( Motor_Velocity[3] > 10 ) Vel_Limit = Speed*15;
//		}
//		else if ( Joystick == 2 )
//		{
//		if ( Motor_Velocity[3] < -10 ) Vel_Limit = Speed*15;
//		}
		
		Vel_Limit = Joystick == 0 ? 10 : Speed * 15;  		//15
		if(Joystick != 0 && Steering_Mode != 1)
		{
			Vel_Limit = 15;
			Left_Frame_Speed = 0;
		}
		
		else if (Joystick == 0 && Steering_Mode != 1)
		{
			Vel_Limit = 0;
			Left_Frame_Speed = 0;
		}
 if ((Joystick!=0)&&(Steering_Mode==ALL_WHEEL)&&(LF_Steering >=25 || RF_Steering<=-25)){	Vel_Limit = 15;}
		
		
		if ( Steering_Mode != ALL_WHEEL ){ Left_Steering_Speed = Right_Steering_Speed = 0; }
	//		Left_Frame_Speed=0;
			Left_Vel_Limit = Vel_Limit  + Left_Steering_Speed + Left_Frame_Speed +4;
			Right_Vel_Limit = Vel_Limit + Right_Steering_Speed+4;
	/////////////////////////////////////////////////////////////////////////APPLYING TORQUE/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		if ( Motor_Velocity[3] < 10 && Motor_Velocity[3] > -10 )
//		{
//		Transmit_Motor_Torque();
//		}
		
			
				// if (Joystick == 1 || Joystick == 2)
				// {
				// 	if (Joystick != Joystick_Temp)
				// 	{
				// 		Transmit_Motor_Torque();
				// 		Joystick_Temp = Joystick;
				// 		//FLAG = SET;
				// 	}
				// }
				
				// if (Joystick == 0)
				// {
				// 	//if ((Motor_Velocity[1] <= Left_Vel_Limit + 2 && Motor_Velocity[1] >= Left_Vel_Limit - 2) && (Motor_Velocity[3] <= Right_Vel_Limit + 2 && Motor_Velocity[3] >= Right_Vel_Limit - 2))
				// 	//{
				// 		Transmit_Motor_Torque();
				// 		//FLAG = NULL;
				// 	//}
				
				if (( Joystick != 0 && Prev_Joystick == 0) || ( Joystick == 0 && Prev_Joystick != 0))
				{
					if( Rover_Velocity < 12 && Rover_Velocity > -12) Transmit_Motor_Torque();
					else {}
				}
				else
				{
//					 Transmit_Motor_Torque();
//						count++;
				}
					

				// if ( Joystick == 0 && Prev_Joystick != 0)
				// {
				// 	if( Motor_Velocity[3] < 12 ) Transmit_Motor_Torque();
				// 	else {}
				// 	Prev_Joystick = Joystick;
				// }
				

			if(Left_Vel_Limit != Left_Transmit_Vel)
			{
				if(HAL_GetTick() - left_tick_count >= 50)
				{
					if(Left_Vel_Limit > Left_Transmit_Vel)
					{
						Left_Transmit_Vel++;
					}
					
					else if(Left_Vel_Limit < Left_Transmit_Vel)
					{
						//Left_Transmit_Vel--;
						
						if ( fabs(Motor_Velocity[3]) <= 20 ) Left_Transmit_Vel--;
						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Left_Transmit_Vel = Left_Transmit_Vel - 3 ;
						else Left_Transmit_Vel = Left_Transmit_Vel - 5;
					}
					else{}
					
					for(uint8_t i=1 ; i <= 2 ; i++) { CAN_Transmit(i,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1);}
					left_tick_count = HAL_GetTick();
				
			}
		}
			
			if(Right_Vel_Limit != Right_Transmit_Vel)
			{
				if(HAL_GetTick() - right_tick_count >= 50)
				{
					if(Right_Vel_Limit > Right_Transmit_Vel)
					{
						Right_Transmit_Vel++;
					}
					
					else if(Right_Vel_Limit < Right_Transmit_Vel)
					{
						//Right_Transmit_Vel--;
						
						if ( fabs(Motor_Velocity[3]) <= 20 ) Right_Transmit_Vel--;
						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Right_Transmit_Vel = Right_Transmit_Vel - 3 ;
						else Right_Transmit_Vel = Right_Transmit_Vel - 5;
					}
					
					else{}
						
					for(uint8_t i=3 ; i <= 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1);}
					right_tick_count = HAL_GetTick();
				}
			}

	}	
	else 
	{
		for ( uint8_t i = 1 ; i < 5 ; i++ )
		{ 
				Set_Motor_Torque ( i , 0 ); HAL_Delay(1);
		}
	}
	
//	if ( (!BT_State ) && Joystick != 0 )
//	{
//		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL ); HAL_Delay(1);}
//		Joystick = 0 ;
//	}
}


void New_New_Drive_Controls(void)
{
	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
	{
		//Rover_Velocity = (fabs(Motor_Velocity[1]) + fabs(Motor_Velocity[2]) + fabs(Motor_Velocity[3]) + fabs(Motor_Velocity[4])) / 4;
//		Vel_Limit = Speed*15;
//		Vel_Limit = Vel_Limit > 50 ? 50 : Vel_Limit < 10 ? 10 : Vel_Limit;

	//	if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) || (Left_Vertical_Error > 6 || Left_Vertical_Error < -6) ){ Joystick = 0;}// Stop_Motors(); }// Safety STOP  (L_R_Err > 5 || L_R_Err < -5)
	
//		if ( Joystick == 1 )
//		{
//		if ( Motor_Velocity[3] > 10 ) Vel_Limit = Speed*15;
//		}
//		else if ( Joystick == 2 )
//		{
//		if ( Motor_Velocity[3] < -10 ) Vel_Limit = Speed*15;
//		}
		
		Vel_Limit = Joystick == 0 ? 5 : Speed * 15;  		//15
		if(Joystick != 0 && Steering_Mode != 1)
		{
			Vel_Limit = 15;
			Left_Frame_Speed = 0;
		}
		
		else if (Joystick == 0 && Steering_Mode != 1)
		{
			Vel_Limit = 0;
			Left_Frame_Speed = 0;
		}
 if ((Joystick!=0)&&(Steering_Mode==ALL_WHEEL)&&(LF_Steering >=25 || RF_Steering<=-25)){	Vel_Limit = 15;}
		
		
		if ( Steering_Mode != ALL_WHEEL ){ Left_Steering_Speed = Right_Steering_Speed = 0; }
	//		Left_Frame_Speed=0;
			Left_Vel_Limit = Vel_Limit  + Left_Steering_Speed + Left_Frame_Speed +4;
			Right_Vel_Limit = Vel_Limit + Right_Steering_Speed+4;
	/////////////////////////////////////////////////////////////////////////APPLYING TORQUE/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		if ( Motor_Velocity[3] < 10 && Motor_Velocity[3] > -10 )
//		{
//		Transmit_Motor_Torque();
//		}
		
			
				// if (Joystick == 1 || Joystick == 2)
				// {
				// 	if (Joystick != Joystick_Temp)
				// 	{
				// 		Transmit_Motor_Torque();
				// 		Joystick_Temp = Joystick;
				// 		//FLAG = SET;
				// 	}
				// }
				
				// if (Joystick == 0)
				// {
				// 	//if ((Motor_Velocity[1] <= Left_Vel_Limit + 2 && Motor_Velocity[1] >= Left_Vel_Limit - 2) && (Motor_Velocity[3] <= Right_Vel_Limit + 2 && Motor_Velocity[3] >= Right_Vel_Limit - 2))
				// 	//{
				// 		Transmit_Motor_Torque();
				// 		//FLAG = NULL;
				// 	//}
				
				if (( Joystick != 0 && Prev_Joystick == 0) || ( Joystick == 0 && Prev_Joystick != 0))
				{
//					if( Rover_Velocity < 12 && Rover_Velocity > -12) Transmit_Motor_Torque();
//					else {}
				}
				else
				{
//					 Transmit_Motor_Torque();
//						count++;
				}
					

				if (Left_Vel_Limit > Right_Vel_Limit)
				{
					Accel_Factor = Left_Vel_Limit / Right_Vel_Limit;
					New_Left_Vel_Limit = Left_Vel_Limit;
					New_Right_Vel_Limit =  Left_Vel_Limit / Accel_Factor ;
				}
				else if (Right_Vel_Limit > Left_Vel_Limit)
				{
					Accel_Factor = Right_Vel_Limit / Left_Vel_Limit;
					New_Right_Vel_Limit = Right_Vel_Limit;
					New_Left_Vel_Limit = Right_Vel_Limit / Accel_Factor;
				}
				else
				{
					New_Left_Vel_Limit = Left_Vel_Limit;
					New_Right_Vel_Limit = Right_Vel_Limit;
				}
				New_Left_Vel_Limit = roundf(New_Left_Vel_Limit);
				New_Right_Vel_Limit = roundf(New_Right_Vel_Limit);
				
				New_Left_Vel_Limit = New_Left_Vel_Limit < 5 ? 5 : New_Left_Vel_Limit;
				New_Right_Vel_Limit = New_Right_Vel_Limit < 5 ? 5 : New_Right_Vel_Limit;
				

			if(New_Left_Vel_Limit != Left_Transmit_Vel)
			{
				if(HAL_GetTick() - left_tick_count >= 500)
				{
					if(New_Left_Vel_Limit > Left_Transmit_Vel)
					{
						Left_Transmit_Vel++;
					}
					
					else if(New_Left_Vel_Limit < Left_Transmit_Vel)
					{
						Left_Transmit_Vel--;
						
//						if ( fabs(Motor_Velocity[3]) <= 20 ) Left_Transmit_Vel--;
//						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Left_Transmit_Vel = Left_Transmit_Vel - 3 ;
//						else Left_Transmit_Vel = Left_Transmit_Vel - 5;
					}
					else{}
					
					for(uint8_t i=1 ; i <= 2 ; i++) { CAN_Transmit(i,VEL_LIMIT,Left_Transmit_Vel,4,DATA);HAL_Delay(1);}
					left_tick_count = HAL_GetTick();
				
			}
		}
			
			if(New_Right_Vel_Limit != Right_Transmit_Vel)
			{
				if(HAL_GetTick() - right_tick_count >= 500)
				{
					if(New_Right_Vel_Limit > Right_Transmit_Vel)
					{
						Right_Transmit_Vel++;
					}
					
					else if(New_Right_Vel_Limit < Right_Transmit_Vel)
					{
						Right_Transmit_Vel--;
						
//						if ( fabs(Motor_Velocity[3]) <= 20 ) Right_Transmit_Vel--;
//						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Right_Transmit_Vel = Right_Transmit_Vel - 3 ;
//						else Right_Transmit_Vel = Right_Transmit_Vel - 5;
					}
					
					else{}
						
					for(uint8_t i=3 ; i <= 4 ; i++) { CAN_Transmit(i,VEL_LIMIT,Right_Transmit_Vel,4,DATA);HAL_Delay(1);}
					right_tick_count = HAL_GetTick();
				}
			}

	}	
	else 
	{
		for ( uint8_t i = 1 ; i < 5 ; i++ )
		{ 
				Set_Motor_Torque ( i , 0 ); HAL_Delay(1);
		}
	}
	
//	if ( (!BT_State ) && Joystick != 0 )
//	{
//		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL ); HAL_Delay(1);}
//		Joystick = 0 ;
//	}
}
void Wheel_Speeds_Calc(int Inner_Angle)
{

		Current_Rover_Velocity = roundf (Rover_Velocity);            //CHANGE
		kmph = Current_Rover_Velocity / 30;                          //CHANGE
    Steering_Angle = fabs((float)Inner_Angle); // new float
    Rover_Centre_Dist = ((WheelBase/sin(Steering_Angle*(3.14/180)))+TrackWidth)/1000;
    TimeTaken =  Rover_Centre_Dist/(kmph*0.277);
    Inner_Speed = ((Rover_Centre_Dist- 0.9)/TimeTaken)*3.6;
    Inner_Speed = KMPHtoRPS(Inner_Speed) -  Current_Rover_Velocity;     //CHANGE
    
    Outer_Speed = ((Rover_Centre_Dist+ 0.9)/TimeTaken)*3.6;
    Outer_Speed = KMPHtoRPS(Outer_Speed) - Current_Rover_Velocity;
	
		Speed_Factor = Outer_Speed / Inner_Speed;
		Calc_Outer_Speed = Speed_Factor * Inner_Speed;
		
    if ( Inner_Angle <= -3 )
    {
				if (Joystick == 1)
				{
        Left_Steering_Speed = Inner_Speed;
        Right_Steering_Speed = Outer_Speed;
				}
			
					else if (Joystick == 2)
					{
						Left_Steering_Speed = Outer_Speed;      //Switching rover direction
						Right_Steering_Speed = Inner_Speed;      //Switching rover direction
					}
					
					else
					{
						Left_Steering_Speed = Right_Steering_Speed = 0;
					}
    }
    else if ( Inner_Angle > 2 ) // Right Turn of the Rover
		{
			if (Joystick == 1)
			{
				Left_Steering_Speed = Outer_Speed;
				Right_Steering_Speed = Inner_Speed;
			}
			
			else if (Joystick == 2)
			{
					Left_Steering_Speed = Inner_Speed;         //Switching rover direction
					Right_Steering_Speed = Outer_Speed;          //Switching rover direction
			}
			
			else
			{
				Left_Steering_Speed = Right_Steering_Speed = 0;
			}
		}
		else
		{
				Left_Steering_Speed = Right_Steering_Speed = NULL;		
		}

}
float Differintial_Angle ( double Inner_Angle_Set )
{
	double TAN=0;float theta=0;
		
		TAN = tan( (Inner_Angle_Set) / 180 * 3.14 );
		
		theta = (atan ( 900 / ((900 / TAN ) + (Track_Width*2 ))) * (180/3.14)) ; //1557
	
		return ( theta );

}

void Steering_Controls (void)
{
/*	If the Steering Reset Flag is SET, all the Steering wheels will return to their Home Position.	
		Flag Sets on Power Up and at every Steering Mode Change.																*/	
	
	if ( Steering_Reset_Flag )
	{
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
		else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
		
		if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
		else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		
		if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
		else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
		
		if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
		else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		
		LF_Error = LR_Error = RF_Error = RR_Error = 1;
		if ((LF_SET) && (LR_SET) && (RF_SET) && (RR_SET) )	
		{
			LF_SET = LR_SET = RF_SET = RR_SET = NULL ;
			Steering_Reset_Flag=NULL;
		}
		else{}
	}
		
	else if( !Steering_Reset_Flag  )
	{
		switch ( Steering_Mode )
		{
//			case 0:
//						if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
//						else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
//						
//						if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
//						else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
//						
//						if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
//						else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
//						
//						if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
//						else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
//						break;
			case ALL_WHEEL :	 //	 --> ALL WHEEL STEERING  
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
							
							Inner_Angle =	(( Pot_Angle / 2 ) - 45);
			
		
							
							if ( Inner_Angle <= -1 )  // Left Turn of the Rover
							{
								 Left_Steering_Speed = Inner_Speed;
								Right_Steering_Speed = Outer_Speed;
								
								LF_Error = (-Inner_Angle - (LF_Steering)) ;						LF_Speed = LF_Error * STEERING_KP;
								LR_Error = (Inner_Angle - (LR_Steering)) ; 						LR_Speed = LR_Error * STEERING_KP;							
												
								Outer_Angle = Differintial_Angle((-Inner_Angle ))	;		RF_Error = (-Outer_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;									
								Outer_Angle_2	= Differintial_Angle((-Inner_Angle ))	;	RR_Error = (-Outer_Angle - (RR_Steering)) ; RR_Speed = RR_Error * STEERING_KP;
							
								Prev_Inner_Angle = Inner_Angle;
							}

							else if ( Inner_Angle >= 1 ) // Right Turn of the Rover   //0
							{
								
								Left_Steering_Speed = Outer_Speed;
								Right_Steering_Speed = Inner_Speed;
								
								RF_Error = (Inner_Angle - (-RF_Steering)) ;						RF_Speed = RF_Error * STEERING_KP;	
								RR_Error = (Inner_Angle - (RR_Steering)) ; 						RR_Speed = RR_Error * STEERING_KP;	
								
								Outer_Angle = Differintial_Angle((Inner_Angle ))	;		LF_Error = (-Outer_Angle- (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;	
								Outer_Angle_2	= Differintial_Angle((Inner_Angle ));		LR_Error = (Outer_Angle_2 - (LR_Steering)) ; LR_Speed = LR_Error * STEERING_KP;
									
								Prev_Inner_Angle = Inner_Angle;
							}	
							
							else // Home Pos of the Rover
							{							
									
								Left_Steering_Speed = Right_Steering_Speed = NULL;	
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
					
							Wheel_Speeds_Calc(Inner_Angle);
							break;
			/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/				
			case CRAB :							//	--> CRAB STEERING			
			/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/
			
							AW_Angle = ( Pot_Angle - 90) ; 
			
							LF_Speed = (LF_Steering > AW_Angle -STEERING_BOUNDARY && LF_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LR_Steering > AW_Angle -STEERING_BOUNDARY && LR_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;					
							RF_Speed = (RF_Steering > AW_Angle -STEERING_BOUNDARY && RF_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RF_Steering < AW_Angle-STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED: ( RF_Steering > AW_Angle+STEERING_BOUNDARY ) ?  STEERING_HOMING_SPEED : 0;
							RR_Speed = (RR_Steering > AW_Angle -STEERING_BOUNDARY && RR_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RR_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( RR_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;
							break;
				/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/	

				case ZERO_TURN :
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
							AW_Angle = Zero_Turn_Angle ; 
			
							LF_Speed = (LF_Steering > -AW_Angle -STEERING_BOUNDARY && LF_Steering < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( LF_Steering < -AW_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > -AW_Angle ) ? -STEERING_HOMING_SPEED :  0;		
							LR_Speed = (LR_Steering > AW_Angle -STEERING_BOUNDARY  && LR_Steering < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( LR_Steering < AW_Angle )  ? STEERING_HOMING_SPEED: ( LR_Steering > AW_Angle )  ? -STEERING_HOMING_SPEED :  0;
							RF_Speed = (RF_Steering > AW_Angle -STEERING_BOUNDARY  && RF_Steering < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( RF_Steering < AW_Angle )  ? -STEERING_HOMING_SPEED: ( RF_Steering > AW_Angle )  ? STEERING_HOMING_SPEED :  0;		
							RR_Speed = (RR_Steering > -AW_Angle -STEERING_BOUNDARY && RR_Steering < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( RR_Steering < -AW_Angle ) ? STEERING_HOMING_SPEED: ( RR_Steering > -AW_Angle ) ? -STEERING_HOMING_SPEED	:  0;

							break;
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
	/*			case FRONT_WHEEL :	 //	 --> FRONT WHEEL STEERING  */
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
			/*				Inner_Angle =	( Pot_Angle / 2 ) - 45;
								
							if ( Inner_Angle <= -1 )  
							{
							
							LF_Error = (-Inner_Angle - (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Outer_Angle = FWD_Differintial_Angle((-Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							RF_Error = (-Outer_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;									
							Outer_Angle_2	= FWD_Differintial_Angle((-Inner_Angle ))	;					
							Prev_Inner_Angle = Inner_Angle;
							}
							
							
							else if ( Inner_Angle >= 0 ) 
							{
							RF_Error = (Inner_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;	
							Outer_Angle = FWD_Differintial_Angle((Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							Outer_Angle_2	= FWD_Differintial_Angle((Inner_Angle ))	;		
							LF_Error = (-Outer_Angle- (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Prev_Inner_Angle = Inner_Angle;
							
							}	
							
							else 
							{								
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
							
							if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
							else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
							
							if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
							else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	

							break;*/
							
				case WIDTH_EXTEND: 
					
							LS_Angle = WIDE_ANGLE; 
							RS_Angle =-WIDE_ANGLE; 
							LF_Speed = (LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LR_Steering > LS_Angle -STEERING_BOUNDARY && LR_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY )&& (RF_Steering > RS_Angle -STEERING_BOUNDARY && RF_Steering < RS_Angle +STEERING_BOUNDARY ))? SET: NULL;
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;
											
							break;
				
				case WIDTH_SHRINK: 
								
							LS_Angle = - SHRINK_ANGLE; 
							RS_Angle = SHRINK_ANGLE; 
							LF_Speed = (LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							LR_Speed = (LR_Steering > LS_Angle -STEERING_BOUNDARY && LR_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;			
							break;
										
			default :  break;
		}
	}
					
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
	LF_Speed = LF_Speed > 0 && LF_Steering >= 90 ?  0 : LF_Speed < 0 && LF_Steering <= -90 ? 0 : LF_Speed;
	LR_Speed = LR_Speed > 0 && LR_Steering >= 90 ?  0 : LR_Speed < 0 && LR_Steering <= -90 ? 0 : LR_Speed;
	RF_Speed = RF_Speed < 0 && RF_Steering >= 90 ?  0 : RF_Speed > 0 && RF_Steering <= -90 ? 0 : RF_Speed;
	RR_Speed = RR_Speed > 0 && RR_Steering >= 90 ?  0 : RR_Speed < 0 && RR_Steering <= -90 ? 0 : RR_Speed;
	
			if ( LF_Speed_Temp != LF_Speed )
			{
				LF_Speed= LF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LF_Speed; 
				Input_Velocity[8] = LF_Speed;
				for (uint8_t i = 0; i < 5; i++)
				{
					Set_Motor_Velocity( LFS , -LF_Speed );
				}
				LF_Speed_Temp = LF_Speed ;
			}
			
			if ( LR_Speed_Temp != LR_Speed )
			{
				LR_Speed= LR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LR_Speed;
				Input_Velocity[9] = LR_Speed;
				for(uint8_t i = 0; i < 5; i++)
				{
				 Set_Motor_Velocity( LRS , -LR_Speed );
				}
				LR_Speed_Temp = LR_Speed ;
			}
						
			if ( RF_Speed_Temp != RF_Speed )
			{	
				RF_Speed= RF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RF_Speed;
				Input_Velocity[10] = RF_Speed;
				for (uint8_t i =0; i< 5; i++)
				{
					Set_Motor_Velocity( RFS , -RF_Speed );
				}
				RF_Speed_Temp = RF_Speed ;
			}
										
			if ( RR_Speed_Temp != RR_Speed )
			{
				RR_Speed= RR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RR_Speed;
				Input_Velocity[11] = RR_Speed;
				for (uint8_t i = 0; i < 5; i++)
				{
				Set_Motor_Velocity( RRS , -RR_Speed );
				}
				RR_Speed_Temp = RR_Speed ;
			}	
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
}

void Battery_Status_Indication(void)
{BT_State = BT_READ ;

		if ( Rover_Voltage >= 46 ) Volt_Tx = 1;
		else Volt_Tx = 0;
		
		if (( Volt_Tx_Temp != Volt_Tx) || ( Prev_BT_State != BT_State))
		{
		HAL_UART_Transmit_IT(&huart5,&Volt_Tx,sizeof(Volt_Tx));
		Volt_Tx_Temp = Volt_Tx;
		Prev_BT_State = BT_State;
		}
		

}
void Operations_Monitor(void)
{
	
	if (HAL_GetTick() - Heartbeat_Tick >= 1500)
	{
		for (uint8_t i = 1; i < 17; i++)
		{
			if (i != 5 && i != 17 && i != 14 && i!= 16)
			{
				if (Node_Id[i] == Node_Id_Temp[i]) 
				{
					Drive_Disconnected = SET;
				}
				Node_Id_Temp[i] = Node_Id[i];
			}
		}
		
		for (uint8_t i = 21; i < 27; i++)
		{

				if (Node_Id[i] == Node_Id_Temp[i]) 
				{
					Sensor_Disconnected = SET;
				}
				Node_Id_Temp[i] = Node_Id[i];
			
		}
		Heartbeat_Tick = HAL_GetTick();
	}
	
	if (HAL_GetTick() - Drive_Error_Tick >= 1000)
	{
		for (uint8_t k = 1; k < 15; k++)
		{
			if ((k != 5) && (k != 17) && (k != 14) &&(k != 16) )
			{
				if (Axis_State[k] != 8)
				{
					Drive_Errored = SET;
					err_count++;
				}
			}
		}
		Drive_Error_Tick = HAL_GetTick();
	}
	
	if (HAL_GetTick() - Fet_Temp_Tick >= 1000)
	{
		
		for (uint8_t i = 1; i < 16; i++)
		{
			if (i != 5)
			{
				FET_Temp_Exceeded = FET_Temperature[i] > 90 ? SET : NULL;
				if (FET_Temp_Exceeded)
				{
					break;
				}
			}
		}
		
		
		Fet_Temp_Tick = HAL_GetTick();
		fet = 0;
	}
	
//	for (uint8_t i = 18; i <= 20; i++)
//	{
//		CAN_Transmit(i, IQ, NULL, 4, REMOTE);
//		HAL_Delay(2);
//	}
	
	if (HAL_GetTick() - Overload_Tick >= 1000)
	{
		for (uint8_t i = 1; i < 20; i++)
		{
			if ( Motor_Current[i] >= 10)
			{
		
				if (HAL_GetTick() - Motor_Tick >= 3000)
				{
					Motor_Overloaded = SET;
					count = HAL_GetTick() - Motor_Tick;
					Motor_Tick = HAL_GetTick();
					break;
				}
			}
		
			
		}
		Overload_Tick = HAL_GetTick();
		//Motor_Tick = HAL_GetTick();
	}
	
	
	if (HAL_GetTick() - Vertical_Limit_Tick >= 2000)
	{
		Vertical_Limit_Exceeded = Vertical_Angle >= 30 && Vertical_Angle <= -30 ? SET : NULL;
		Vertical_Limit_Tick = HAL_GetTick();
	}
	
	if (HAL_GetTick() - Contour_Limit_Tick >= 2000)
	{
		Contour_Angle = Shear_Roll_Home_Pos - Shear_Roll;
		Contour_Limit_Exceeded = Contour_Angle >= 30 && Contour_Angle <= -30 ? SET : NULL;
		Contour_Limit_Tick = HAL_GetTick();
	}
	
	if (HAL_GetTick() - Pitch_Limit_Tick >= 2000)
	{
		Pitch_Angle = Shear_Pitch_Home_Pos - Shear_Pitch;
		Pitch_Limit_Exceeded = Pitch_Angle >= 30 && Pitch_Angle <= -30 ? SET : NULL;
		Pitch_Limit_Tick = HAL_GetTick();
	}
	
	Vertical_Error = Right_Pitch_Pos - R_Pitch;
	if (HAL_GetTick() - Vertical_Tick >= 1000)
	{
		if ((Vertical_Error > 10 || Vertical_Error < -10) && Input_Velocity[6] != 0 && Motor_Velocity[6] < 4)
		{
			if (HAL_GetTick() - Vert_Resp_Tick >= 3000)
			{
				Vertical_Not_Responding = SET;
				Vert_Resp_Tick = HAL_GetTick();
			}
		}
		
		else
		{
			Vert_Resp_Tick = HAL_GetTick();
		}
		
		Vertical_Tick = HAL_GetTick();
	}
	
	Contour_Error = Right_Roll_Pos - R_Roll;
	if (HAL_GetTick() - Contour_Tick >= 1000)
	{
		if ((Contour_Error > 10 || Contour_Error < -10) && Input_Velocity[7] != 0 && Motor_Velocity[7] < 4)
		{
			if (HAL_GetTick() - Cont_Resp_Tick >= 3000)
			{
				Contour_Not_Responding = SET;
				Cont_Resp_Tick = HAL_GetTick();
			}
		}
		
		else
		{
			Cont_Resp_Tick = HAL_GetTick();
		}
		Contour_Tick = HAL_GetTick();
	}
	
	Pitch_Error = Shear_Pitch_Home_Pos - Shear_Pitch;
	if (HAL_GetTick() - Pitch_Tick >= 1000)
	{
		if ((Pitch_Error > 10 || Pitch_Error < -10) && Input_Velocity[14] != 0 && Motor_Velocity[14] < 4)
		{
			if (HAL_GetTick() - Pitch_Resp_Tick >= 3000)
			{
				Pitch_Not_Responding = SET;
				Pitch_Resp_Tick = HAL_GetTick();
			}
		}
		
		else
		{
			Pitch_Resp_Tick = HAL_GetTick();
			
		}
		
		Pitch_Tick = HAL_GetTick();
	}
		
	OPERATION_MONITOR_FLAG = Drive_Disconnected == SET || Sensor_Disconnected == SET || Drive_Errored == SET || FET_Temp_Exceeded == SET ? SET : NULL;// && FET_Temp_Exceeded == SET && Motor_Overloaded == SET && E_Stop == SET && Joystick_Disconnected == SET && Vertical_Limit_Exceeded == SET && Contour_Limit_Exceeded == SET && Pitch_Limit_Exceeded == SET && Vertical_Not_Responding == SET && Contour_Not_Responding == SET ? SET : NULL;
}

void Emergency_Stop(void)
{
	BUZZER_ON;
	

	
//	if (Rover_Velocity < 3 || Rover_Velocity == NAN)
//	{
		
			for (uint8_t i = 1; i <= 20; i++)
	{
		Set_Motor_Velocity(i , 0);
		Input_Velocity[i] = 0;
	}
	
		if (Drive_Disconnected == SET)
		{
			for (uint8_t i = 1; i < 21; i++)
			{
				if(i != 5 && i != 17 && i != 14)
				{
					while (Node_Id[i] == Node_Id_Temp[i])
					{ 
						Node++;
					}
				}
			}
			Drive_Disconnected = NULL;
		}
		Node = 0;
		
		if (Sensor_Disconnected == SET)
		{
			for (uint8_t i = 21; i < 27; i++)
			{
				while (Node_Id[i] == Node_Id_Temp[i]){}
			}
			Sensor_Disconnected = NULL;
		}
		
		if (Drive_Errored == SET)
		{
			for (uint8_t i = 1; i < 21; i++)
			{
				if (i != 5 && i != 17 && i != 14)
				{
					while (Axis_State[i] != 8)
					{
						if(i == 6 || i == 12 || i == 13 || i == 15 || i == 16)
						{
							Clear_Error(i);
							HAL_Delay(2000);
							Start_Calibration_For (i, 8, 10);
							HAL_Delay(1000);
						}
						
						else
						{
							Reboot(i);
							HAL_Delay(2000);
						}
					}
				}
			}
			
			Drive_Errored = NULL;
		}
		
		
		
		if (Motor_Overloaded == SET)
		{
			for (uint8_t i = 1; i < 20; i++)
			{
				if (Motor_Velocity[i] == 0 && Input_Velocity[i] == 0 && Motor_Current[i] < 5)
				{
					Motor_Overloaded = NULL;
				}
			}
		}
		
		while (EEPROM_Error == SET)
		{
			
		}
		
		if (Vertical_Not_Responding == SET)
		{
			if ((Vertical_Error < 10 || Vertical_Error > -10))
			{
				Vertical_Not_Responding = NULL;
			}
		}
		
		if (Contour_Not_Responding == SET)
		{
			if (Contour_Error < 10 || Contour_Error > -10)
			{
				Contour_Not_Responding = NULL;
			}
		}
		
		if (Pitch_Not_Responding == SET)
		{
			if (Pitch_Error < 10 || Pitch_Error > -10)
			{
				Pitch_Not_Responding = NULL;
			}
		}
	//}
	
	OPERATION_MONITOR_FLAG =  Drive_Disconnected == NULL && Sensor_Disconnected == NULL && Drive_Errored == NULL && FET_Temp_Exceeded == NULL ? NULL : SET;// && FET_Temp_Exceeded == NULL && Motor_Overloaded == NULL && E_Stop == NULL && Joystick_Disconnected == NULL && Vertical_Limit_Exceeded == NULL && Contour_Limit_Exceeded == NULL && Pitch_Limit_Exceeded == NULL && Vertical_Not_Responding == NULL && Contour_Not_Responding == NULL && Pitch_Not_Responding == NULL ? NULL : SET;
	if (OPERATION_MONITOR_FLAG == NULL) {BUZZER_OFF;}
	
	
}


void Frame_Controls(void)
{	
/*-----------------------------PID CONTROL-------------------------------------*/
  if (!Left_IMU_State )
	{
	L_Vert_Speed = R_Vert_Speed = Contour_Speed = 0;
	//BUZZER_ON;
	}
	else if ( Left_IMU_State ) 


	{	// ADD ZERO ERROR CLR FLAG AND HEARTBEAT OK FLAG	
	/*	L_R_Err =  Left_Roll_Pos - L_Roll ;                                               										// L roll error = Target value(0.68) - current value. 
		
		L_Vert_Speed = Left_Verticality_PID ( L_R_Err , NULL );         																			// L Vertical speed from left verticality pid function. 

		L_Vert_Speed = (( L_Vert_Speed <= 3 ) && ( L_Vert_Speed >= -3 ) ) ? 0 : L_Vert_Speed;                 // Assigning 0 to L Vertical Speed if it is between - 2 to 2 (to avoid oscillations)

		Left_Error_Flag =( L_Vert_Speed == 0 ) ? NULL : SET;																									// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE
	
*/
		//R_R_Err =  Right_Pitch_Pos - R_Pitch 	;	
	
		R_R_Err =  Right_Pitch_Pos - R_Pitch_Filtered 	;																																	// R roll error = Target value(-3.0625) - current value.
	
		R_Vert_Speed = Right_Verticality_PID ( R_R_Err , NULL );																							// R Vertical speed from right verticality pid function. 
	
	  R_Vert_Speed = (( R_Vert_Speed <= 2 ) && ( R_Vert_Speed >= -2 ) ) ? 0 : R_Vert_Speed;									// Assigning 0 to R Vertical Speed if it is between - 2 to 2 (to avoid oscillations)

		Right_Error_Flag =( R_Vert_Speed == 0 ) ? NULL : SET;																									// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE


		//Contour_Avg =	R_Pitch ;																																						
		
		//C_Err =   Right_Roll_Pos - R_Roll ;	
		
		C_Err =   Right_Roll_Pos - R_Roll_Filtered ;	
		
		// (CHECK) Contour error = Target value(2.5) - current value.
	
	//	if ( C_Err > 1 || C_Err < -1)  // Error Boundary
	//	{
		Contour_Speed = Contour_PID( C_Err , NULL );																													// Contour speed from contour pid function.
		
		Contour_Speed = (( Contour_Speed <= 2 ) && ( Contour_Speed >= -2) ) ? 0 : Contour_Speed;							// Assigning 0 to Contour Speed if it is between - 1 to 1 (to avoid oscillations)

		Contour_Error_Flag = ( Contour_Speed == 0 ) ? NULL : SET;																							// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE
	//	}
	//	else Contour_Speed = 0;
	
			
		FRAME_NO_ERROR_FLAG = ( !Left_Error_Flag && !Right_Error_Flag && !Contour_Error_Flag ) ? NULL : SET;		// (CHECK) for basic testing. 

/*-----------------------------PID CONTROL-------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-----------------------------CHECK FOR LIMITS AND SET VELOCITIES TO MOTOR-------------------------------------*/
		
//		if ( L_Vert_Speed > 0 && Left_Vertical < -FRAME_LIMIT) L_Vert_Speed = 0;
//		if ( L_Vert_Speed < 0 && Left_Vertical > 	FRAME_LIMIT) L_Vert_Speed = 0;
		
		if( L_Vert_Speed_Temp != L_Vert_Speed ) 																															// checking if the new value is not equal to old value
		{
			//Set_Motor_Velocity (LVert , L_Vert_Speed );
			L_Vert_Speed_Temp = L_Vert_Speed ;																																// Overwriting old value with new value. 
		}		
			
		//R_Vert_Speed = R_Vert_Speed > 0 && Right_Vertical_Motor_Count >= 550 ? 0 : R_Vert_Speed < 0 && Right_Vertical_Motor_Count <= -550 ? 0 : R_Vert_Speed ;
	  // 	Right_Vertical_On_Limit = Right_Vertical_Motor_Count >= 550 ||  Right_Vertical_Motor_Count <= -550 ? SET : NULL;
		if( R_Vert_Speed_Temp != R_Vert_Speed ) 																															// checking if the new value is not equal to old value
		{
			Input_Velocity[6] = R_Vert_Speed;
			Set_Motor_Velocity (RVert , -R_Vert_Speed );	  //-
			R_Vert_Speed_Temp = R_Vert_Speed ;																																// Overwriting old value with new value.
		} 
	//	Contour_Speed = Contour_Speed > 0 && Contour_Motor_Count >= 550 ? 0 : Contour_Speed < 0 && Contour_Motor_Count <= -550 ? 0 : Contour_Speed ;
	//	Contour_On_Limit = Contour_Motor_Count >= 550 ||  Contour_Motor_Count <= -550 ? SET : NULL;		
		if( Contour_Speed_Temp != Contour_Speed ) 																														// checking if the new value is not equal to old value
		{
			Input_Velocity[7] = Contour_Speed;
			Set_Motor_Velocity (Contour , Contour_Speed );
			Contour_Speed_Temp = Contour_Speed ;																															// Overwriting old value with new value.
		}		
		
		Frame_Buzz_Switch = ( Right_Vertical_On_Limit == 1 ) || (Contour_On_Limit == 1) ?  1 : 0;

			/*-----------------------------CHECK FOR LIMITS AND SET VELOCITIES TO MOTOR-------------------------------------*/				
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////						
}
}	

void Frame_Controls_Sensor_BLE(void)
{	
/*-----------------------------PID CONTROL-------------------------------------*/
  if (!Left_IMU_State )
	{
	L_Vert_Speed = R_Vert_Speed = Contour_Speed = 0;
	//BUZZER_ON;
	}
	else if ( Left_IMU_State ) 


	{	// ADD ZERO ERROR CLR FLAG AND HEARTBEAT OK FLAG	
	/*	L_R_Err =  Left_Roll_Pos - L_Roll ;                                               										// L roll error = Target value(0.68) - current value. 
		
		L_Vert_Speed = Left_Verticality_PID ( L_R_Err , NULL );         																			// L Vertical speed from left verticality pid function. 

		L_Vert_Speed = (( L_Vert_Speed <= 3 ) && ( L_Vert_Speed >= -3 ) ) ? 0 : L_Vert_Speed;                 // Assigning 0 to L Vertical Speed if it is between - 2 to 2 (to avoid oscillations)

		Left_Error_Flag =( L_Vert_Speed == 0 ) ? NULL : SET;																									// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE
	
*/
		//R_R_Err =  Right_Pitch_Pos - R_Pitch 	;	
		
//	
//		R_R_Err =  Right_Pitch_Home_Pos - BLE_Pitch 	;																																	// R roll error = Target value(-3.0625) - current value.
//		
//		R_R_Err = R_R_Err > 20 || R_R_Err < -20 ? Prev_R_R_Err : R_R_Err;
//		
//		R_R_Err = R_R_Err < 0.2 && R_R_Err > -0.2 ? 0 : R_R_Err;
//	
//		R_Vert_Speed = Right_Verticality_PID ( R_R_Err , NULL );																							// R Vertical speed from right verticality pid function. 
//	
//	  //R_Vert_Speed = (( R_Vert_Speed <= 2 ) && ( R_Vert_Speed >= -2 ) ) ? 0 : R_Vert_Speed;									// Assigning 0 to R Vertical Speed if it is between - 2 to 2 (to avoid oscillations)

//		Right_Error_Flag =( R_Vert_Speed == 0 ) ? NULL : SET;																									// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE


//		//Contour_Avg =	R_Pitch ;																																						
//		
//		//C_Err =   Right_Roll_Pos - R_Roll ;	
//		
//		C_Err =   Right_Roll_Home_Pos - BLE_Roll ;	
//		
//		C_Err = C_Err > 20 || C_Err < -20 ? Prev_C_Err : C_Err;
//		
//		C_Err = C_Err < 0.2 && C_Err > -0.2 ? 0 : C_Err;
		
		// (CHECK) Contour error = Target value(2.5) - current value.
	
	//	if ( C_Err > 1 || C_Err < -1)  // Error Boundary
	//	{
		Contour_Speed = Contour_PID( C_Err , NULL );																													// Contour speed from contour pid function.
		
		//Contour_Speed = (( Contour_Speed <= 2 ) && ( Contour_Speed >= -2) ) ? 0 : Contour_Speed;							// Assigning 0 to Contour Speed if it is between - 1 to 1 (to avoid oscillations)

		Contour_Error_Flag = ( Contour_Speed == 0 ) ? NULL : SET;																							// (CHECK) for basic testing. to set once the vertical speed is zero(correction completed) ISSUE
	//	}
	//	else Contour_Speed = 0;
		Prev_R_R_Err = R_R_Err;
		Prev_C_Err = C_Err;
			
		FRAME_NO_ERROR_FLAG = ( !Left_Error_Flag && !Right_Error_Flag && !Contour_Error_Flag ) ? NULL : SET;		// (CHECK) for basic testing. 

/*-----------------------------PID CONTROL-------------------------------------*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*-----------------------------CHECK FOR LIMITS AND SET VELOCITIES TO MOTOR-------------------------------------*/
		
//		if ( L_Vert_Speed > 0 && Left_Vertical < -FRAME_LIMIT) L_Vert_Speed = 0;
//		if ( L_Vert_Speed < 0 && Left_Vertical > 	FRAME_LIMIT) L_Vert_Speed = 0;
		
		if( L_Vert_Speed_Temp != L_Vert_Speed ) 																															// checking if the new value is not equal to old value
		{
			//Set_Motor_Velocity (LVert , L_Vert_Speed );
			L_Vert_Speed_Temp = L_Vert_Speed ;																																// Overwriting old value with new value. 
		}		
			
		//R_Vert_Speed = R_Vert_Speed > 0 && Right_Vertical_Motor_Count >= 550 ? 0 : R_Vert_Speed < 0 && Right_Vertical_Motor_Count <= -550 ? 0 : R_Vert_Speed ;
	  // 	Right_Vertical_On_Limit = Right_Vertical_Motor_Count >= 550 ||  Right_Vertical_Motor_Count <= -550 ? SET : NULL;
		if( R_Vert_Speed_Temp != R_Vert_Speed ) 																															// checking if the new value is not equal to old value
		{
			Input_Velocity[6] = R_Vert_Speed;
			Set_Motor_Velocity (RVert , R_Vert_Speed );	  //-
			R_Vert_Speed_Temp = R_Vert_Speed ;																																// Overwriting old value with new value.
		} 
	//	Contour_Speed = Contour_Speed > 0 && Contour_Motor_Count >= 550 ? 0 : Contour_Speed < 0 && Contour_Motor_Count <= -550 ? 0 : Contour_Speed ;
	//	Contour_On_Limit = Contour_Motor_Count >= 550 ||  Contour_Motor_Count <= -550 ? SET : NULL;		
		if( Contour_Speed_Temp != Contour_Speed ) 																														// checking if the new value is not equal to old value
		{
			Input_Velocity[7] = Contour_Speed;
			Set_Motor_Velocity (Contour , -Contour_Speed );
			Contour_Speed_Temp = Contour_Speed ;																															// Overwriting old value with new value.
		}		
		
		Frame_Buzz_Switch = ( Right_Vertical_On_Limit == 1 ) || (Contour_On_Limit == 1) ?  1 : 0;

			/*-----------------------------CHECK FOR LIMITS AND SET VELOCITIES TO MOTOR-------------------------------------*/				
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////						
}

}
float Right_Verticality_PID ( float Right_Roll_Value , unsigned long long 	R_Time_Stamp )
{
		//dt = Time_Stamp - time;

					R_Error_Change = Right_Roll_Value - R_Prev_Error;
					R_Error_Slope = R_Error_Change / dt;
					R_Error_Area = R_Error_Area + ( R_Error_Change * dt ) ;
			
				
				
			R_P = R_Kp * Right_Roll_Value;
			 
			R_I	= R_Ki * R_Error_Area;						 R_I = R_I > Anti_Windup_Limit ? Anti_Windup_Limit : R_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : R_I ;	

			R_D = R_Kd * R_Error_Slope; 
				
			
			
				Right_Out = R_P + R_I + R_D ;
		
			
			//	Right_Out = (Right_Roll_Value < V_BOUNDARY && Right_Roll_Value	> -V_BOUNDARY) ?	0: Right_Out;

				Right_Out = Right_Out > V_LIMIT ? V_LIMIT : Right_Out < -V_LIMIT ? -V_LIMIT : Right_Out;

		
			R_Prev_Error = Right_Roll_Value;
		//	time = Time_Stamp;
			
			return Right_Out;

}

float Contour_PID ( float Contour_Val , unsigned long long 	C_Time_Stamp )
{
		//dt = Time_Stamp - time;

					C_Error_Change = Contour_Val - C_Prev_Error;
					C_Error_Slope = C_Error_Change / dt;
					C_Error_Area = C_Error_Area + ( C_Error_Change * dt ) ;
			
				
			C_P = C_Kp * Contour_Val;
			 
			C_I	= C_Ki * C_Error_Area;						 C_I = C_I > Anti_Windup_Limit ? Anti_Windup_Limit : C_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : C_I ;	

			C_D = C_Kd * C_Error_Slope; 
				
			
			
				Contour_Out = C_P + C_I + C_D ;
			

				Contour_Out = Contour_Out > C_LIMIT ? C_LIMIT : Contour_Out < -C_LIMIT ? -C_LIMIT : Contour_Out;

		
			C_Prev_Error = Contour_Val;
		//	time = Time_Stamp;
			
			return Contour_Out;

}

float Left_Frame_PID ( float Left_Error_Value , unsigned long long 	L_Time_Stamp )
{	
	

					LF_Error_Change = Left_Error_Value - LF_Prev_Error;
					LF_Error_Slope = LF_Error_Change / dt;
					LF_Error_Area = LF_Error_Area + ( LF_Error_Change * dt ) ;
			
				
				
			LF_P = LF_Kp * Left_Error_Value;
			 
			LF_I	= LF_Ki * LF_Error_Area;						 LF_I = LF_I > Anti_Windup_Limit ? Anti_Windup_Limit : LF_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : LF_I ;	

			LF_D = LF_Kd * LF_Error_Slope; 
				
			
			
				Left_Frame_Out = LF_P + LF_I + LF_D ;
		
			
			//	Right_Out = (Right_Roll_Value < V_BOUNDARY && Right_Roll_Value	> -V_BOUNDARY) ?	0: Right_Out;

				Left_Frame_Out = Left_Frame_Out > 10 ? 10 : Left_Frame_Out < -10 ? -10 : Left_Frame_Out;

		
			LF_Prev_Error = Left_Error_Value;
		//	time = Time_Stamp;
			
			return Left_Frame_Out;

}

float Left_Arm_PID ( float Left_Flap_Error , unsigned long long 	L_Time_Stamp )
{
		//dt = Time_Stamp - time;
	


			L_Error_Change = Left_Flap_Error - L_Prev_Error;
			L_Error_Slope  = L_Error_Change / dt;
			L_Error_Area   = L_Error_Area + ( L_Error_Change * dt ) ;
			
				
				
			L_P = L_Kp * Left_Flap_Error;
			 
			L_I	= L_Ki * L_Error_Area;						 L_I = L_I > Anti_Windup_Limit ? Anti_Windup_Limit : L_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : L_I ;	

			L_D = L_Kd * L_Error_Slope; 
				
			
			
			Left_Out = L_P + L_I + L_D ;

			Left_Out = Left_Out > Arm_Max_Speed ? Arm_Max_Speed : Left_Out < -Arm_Max_Speed ? -Arm_Max_Speed : Left_Out;

			L_Prev_Error = Left_Flap_Error;
			
			return Left_Out;

}
float Right_Arm_PID ( float Right_Flap_Error , unsigned long long 	RA_Time_Stamp )
{
		//dt = Time_Stamp - time;
	


			RA_Error_Change = Right_Flap_Error - RA_Prev_Error;
			RA_Error_Slope  = RA_Error_Change / dt;
			RA_Error_Area   = RA_Error_Area + ( RA_Error_Change * dt ) ;
			
				
				
			RA_P = RA_Kp * Right_Flap_Error;
			 
			RA_I	= RA_Ki * RA_Error_Area;						 RA_I = RA_I > Anti_Windup_Limit ? Anti_Windup_Limit : RA_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : RA_I ;	

			RA_D = RA_Kd * RA_Error_Slope; 
				
			
			
			RightArm_Out = RA_P + RA_I + RA_D ;

			RightArm_Out = RightArm_Out > Arm_Max_Speed ? Arm_Max_Speed : RightArm_Out < -Arm_Max_Speed ? -Arm_Max_Speed : RightArm_Out;

			RA_Prev_Error = Right_Flap_Error;
			
			return RightArm_Out;

}
float Pitch_Arm_PID ( float Pitch_Error , unsigned long long 	R_Time_Stamp )
{
		//dt = Time_Stamp - time;
	


			P_Error_Change = Pitch_Error - P_Prev_Error;
			P_Error_Slope  = P_Error_Change / dt;
			P_Error_Area   = P_Error_Area + ( P_Error_Change * dt ) ;
			
				
				
			P_P = P_Kp * Pitch_Error;
			 
			P_I	= P_Ki * P_Error_Area;						 P_I = P_I > Anti_Windup_Limit ? Anti_Windup_Limit : P_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : P_I ;	

			P_D = P_Kd * P_Error_Slope; 
				
			
			
			Pitch_Out = P_P + P_I + P_D ;

			Pitch_Out = Pitch_Out > Arm_Max_Speed ? Arm_Max_Speed : Pitch_Out < -Arm_Max_Speed ? -Arm_Max_Speed : Pitch_Out;

			P_Prev_Error = Pitch_Error;
			
			return Pitch_Out;

}
void EEPROM_Store_Data (void)
{
	Vertical_Motor_Count = Vertical_Motor_Value + Absolute_Position_Float[6];
	Left_Macro_Motor_Count = Left_Macro_Motor_Value + Absolute_Position_Float[12];
	Right_Macro_Motor_Count = Right_Macro_Motor_Value + Absolute_Position_Float[13];
	//Pitch_Arm_Motor_Count = Pitch_Arm_Motor_Value + Absolute_Position_Float[14];
	Lower_Width_Motor_Count = Lower_Width_Motor_Value + Absolute_Position_Float[15];
	Upper_Width_Motor_Count = Upper_Width_Motor_Value + Absolute_Position_Float[16];

																																																											
	memcpy(&Write_Value[1], &Vertical_Motor_Count, sizeof(Vertical_Motor_Count));
	memcpy(&Write_Value[5], &Left_Macro_Motor_Count, sizeof(Left_Macro_Motor_Count));
	memcpy(&Write_Value[9], &Right_Macro_Motor_Count, sizeof(Right_Macro_Motor_Count));
	//memcpy(&Write_Value[13], &Pitch_Arm_Motor_Count, sizeof(Pitch_Arm_Motor_Count));
	memcpy(&Write_Value[13], &Lower_Width_Motor_Count, sizeof(Lower_Width_Motor_Count));
	memcpy(&Write_Value[17], &Upper_Width_Motor_Count, sizeof(Upper_Width_Motor_Count));

	
//	
		for(uint8_t i = 0; i < 21; i++)
		{
			if ( !Store_Data)
			{
				if(Prev_Write_Value[i] != Write_Value[i])
				{
					Store_Data= 1;
					Prev_Write_Value[i] = Write_Value[i];
				}
				else Store_Data = 0;
			}
		}
		
		
		if ( Store_Data)
		{
			EEPROM_Write(60, 0, (uint8_t *)Write_Value, sizeof(Write_Value)); //HAL_Delay(10);
			Store_Data = 0;
		}
		
//		Lead_Screw_Length = Vertical_Motor_Count * 0.5;      
//		Vertical_Angle = Lead_Screw_Length * 0.222;

	}
void Frame_Synchronization(void)
{  
      Left_Vertical_Error =  Left_Pitch_Pos - L_Pitch ; //L_R_Err = x;
      
      Current_Vel_Limit = Vel_Limit ;
      
      if ( Left_Vertical_Error > 3 )  Modified_Vel_Limit = Joystick == 2 ? Current_Vel_Limit - 10 : Current_Vel_Limit + 10  ;//(Left_Vertical_Error * Width_Kp) ;
    
      else if ( Left_Vertical_Error < -3 )  Modified_Vel_Limit = Joystick == 2 ? Current_Vel_Limit + 10 : Current_Vel_Limit - 10;//(Left_Vertical_Error * Width_Kp) ;
    
      else Modified_Vel_Limit = Current_Vel_Limit ;
  
      if ( Modified_Vel_Limit != Modified_Vel_Limit_Temp )
      {
        if( Modified_Vel_Limit > Current_Vel_Limit )   
        {
				 for ( uint8_t v = Current_Vel_Limit+3; v <= Modified_Vel_Limit  ; v=v+3 )
				 {  
					 for(uint8_t i=1 ; i < 3 ; i++) 
					 {
						 CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
					 }
						HAL_Delay(5);
					}    
				 Prev_Mod = Modified_Vel_Limit;  
        }
      else if( Modified_Vel_Limit < Current_Vel_Limit )
      {
			 for ( uint8_t v = Current_Vel_Limit-3; v >= Modified_Vel_Limit  ; v=v-3 )
			 {  
				for(uint8_t i=1 ; i < 3 ; i++) 
				{
				  CAN_Transmit(i,VEL_LIMIT,v,4,DATA);
				}
				HAL_Delay(5);
			 }
			 Prev_Mod = Modified_Vel_Limit;
      }
      else 
      {
        if ( Prev_Mod > Current_Vel_Limit )
        {          
					for ( uint8_t v = Prev_Mod-1; v >= Modified_Vel_Limit  ; v=v-1 )
					{  
					 for(uint8_t i=1 ; i < 3 ; i++) 
					 {
						CAN_Transmit(i,VEL_LIMIT,Current_Vel_Limit,4,DATA);
					 }
					 HAL_Delay(5); 
					}
        }
        else if ( Prev_Mod < Current_Vel_Limit )
        {          
					for ( uint8_t v = Prev_Mod+1; v <= Modified_Vel_Limit  ; v=v+1 )
					 {  
						 for(uint8_t i=1 ; i < 3 ; i++) 
						 {
							CAN_Transmit(i,VEL_LIMIT,Current_Vel_Limit,4,DATA);
						 }
					  HAL_Delay(5);
					 }
        }
        
        else {}
          Prev_Mod = Current_Vel_Limit;   
      }
      Modified_Vel_Limit_Temp = Modified_Vel_Limit ; 
      }

  else { }
}

void Left_Frame_Controls (void)
 {
		 
	 if(Joystick == 2)
	 {
		 Left_Vertical_Error =  Left_Pitch_Pos - L_Pitch ;
			Left_Frame_Speed=Left_Frame_PID(Left_Vertical_Error,0);
		 Left_Frame_Speed = Left_Frame_Speed < 3 && Left_Frame_Speed > -3 ? 0 : Left_Frame_Speed;
	 }
	 else
	 {
		 Left_Frame_Speed = 0;
	 }
		//Left_Frame_Speed = Left_Frame_Speed < 2 && Left_Frame_Speed > -2 ? 0 : Left_Frame_Speed;
	//Left_Frame_Speed = abs(Left_Frame_Speed);
	//Left_Frame_Speed = Joystick == 1 ? -Left_Frame_Speed: Joystick == 2 ? Left_Frame_Speed : 0;
//		if ( Left_Vertical_Error >= 2 )  Left_Frame_Speed = Joystick == 2 ? 15: -15  ;//(Left_Vertical_Error * Width_Kp) ;
//	
//		else if ( Left_Vertical_Error <= -2 )  Left_Frame_Speed = Joystick == 2 ? -15 : 15;//(Left_Vertical_Error * Width_Kp) ;
//	
//		else Left_Frame_Speed = 0 ;


}
void Top_Flap_Sensing(void)
{
	Array_Element    =  Speed + 4 ;
	Flap_Angle = Flap_Data[Array_Element];
  
	Front_Left_Bush  =  Flap_Angle > 40 ? 1 : 0;
	
	if ( Mode != 2)
	{
		L_Arm_Speed 		 = Left_Arm_Motor_Count <= 1 && Left_Arm_Motor_Count  >=-1 ? 0 : Left_Arm_Motor_Count < -1 ? 10 : -10;
		R_Arm_Speed 		 = Right_Arm_Motor_Count<= 1 && Right_Arm_Motor_Count >= -1 ? 0 : Right_Arm_Motor_Count < -1 ? 10 : -10;
		Pitch_Arm_Speed  = Pitch_Arm_Motor_Count <= 1 && Pitch_Arm_Motor_Count >= -1 ? 0 : Pitch_Arm_Motor_Count < -1 ? 10 : -10;
		//Tri_Arm_Speed =  -ARM_HOMING_SPEED ; 
		
		First_Sense = 0;
	}
	else if (Mode == 2)
	{
		if ( Front_Left_Bush ) First_Sense = SET;
		
		if (!First_Sense)
		{
			Tri_Arm_Speed =  ARM_HOMING_SPEED ; 
		}
		else
		{
			if ( Front_Left_Bush )
			{
				Flap_Error = Flaps_Target - Flap_Angle ;
				//Tri_Arm_Speed = Flap_Error < 2 && Flap_Error > -2 ? 0 : Flap_Error * Flap_Kp;
				Tri_Arm_Speed = Flap_Error * Flap_Kp;
			}
			else Tri_Arm_Speed = 0;
		}
		
		Tri_Arm_Speed = Tri_Arm_Speed > 30 ? 30 : Tri_Arm_Speed < -30 ? -30 : Tri_Arm_Speed;
		
		

		//L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = 0;
//		L_Arm_Speed 		 = Left_Arm_Motor_Count > 58 && Left_Arm_Motor_Count  <62 ? 0 : Left_Arm_Motor_Count < 58 ? 10 : -10;
//		R_Arm_Speed 		 = Right_Arm_Motor_Count > 58 && Right_Arm_Motor_Count  <62 ? 0 : Right_Arm_Motor_Count < 58 ? 10 : -10;
//		Pitch_Arm_Speed = Pitch_Arm_Motor_Count > 58 && Pitch_Arm_Motor_Count  <62 ? 0 : Pitch_Arm_Motor_Count < 58 ? 10 : -10;
		L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = Tri_Arm_Speed;
	}
	else {L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = 0;}
	
	
	 
	
	//L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count >= 45 ? 10 : L_Arm_Speed < 0 && Left_Arm_Motor_Count <= 10 ? -10 : L_Arm_Speed ;
	L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count >= 50 ? 0 : L_Arm_Speed < 0 && Left_Arm_Motor_Count <= 5 ? 0 : L_Arm_Speed ;
	if( L_Arm_Speed_Temp != L_Arm_Speed )
	{
		Set_Motor_Velocity (L_Arm , L_Arm_Speed );	
		L_Arm_Speed_Temp = L_Arm_Speed ;
	}
	//R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count >= 45 ? 10 : R_Arm_Speed < 0 && Right_Arm_Motor_Count <= 10 ? -10 : R_Arm_Speed ;
	R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count >= 50 ? 0 : R_Arm_Speed < 0 && Right_Arm_Motor_Count <= 5 ? 0 : R_Arm_Speed ;
	if( R_Arm_Speed_Temp != R_Arm_Speed )
	{
		Set_Motor_Velocity (R_Arm , R_Arm_Speed );	
		R_Arm_Speed_Temp = R_Arm_Speed ;
	}
	Pitch_Arm_Speed = Pitch_Arm_Speed/2.39;
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count >= 13  ? 5 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count <= 10 ? -5 : Pitch_Arm_Speed ;
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count >= 23  ? 0 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count <= 0 ? 0 : Pitch_Arm_Speed ;
	if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed ) 
	{
		Set_Motor_Velocity (P_Arm , Pitch_Arm_Speed );	
		Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
	}

}
void Top_Sensing_Roll(void)
{
	Array_Element    =  Speed + 4 ;

//	Flap_Mod_Value = Flap_Data[Array_Element];
//	Flap_Mod_Value_Right = Flap_Data_Right[Array_Element];
	Flap_Mod_Value=Left_Flap;
	Flap_Mod_Value_Right=Right_Flap;

	Front_Left_Bush  =  Flap_Mod_Value < 40 ? 0 : 1;
	Front_Right_Bush = 	Flap_Mod_Value_Right < 40 ? 0: 1;
	
	Front_Bush = (Front_Left_Bush == 1 || Front_Right_Bush == 1) ? 1 : 0;

	if (Mode != 3 ) // Drive / Height Adjustment Modes
	{
		L_Arm_Speed 		= Left_Arm_Motor_Count  > 1 ? -ARM_HOMING_SPEED : Left_Arm_Motor_Count  < -1 ? ARM_HOMING_SPEED : 0;   //0.5
		R_Arm_Speed 		= Right_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Right_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;    //0.5
		Pitch_Arm_Speed 	= Pitch_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Pitch_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;   //0.5
		First_Sense = NULL;
	}
	
	else if(Mode == 3)  // Sensing_Mode
	{
		if ( Front_Bush ) First_Sense = SET; // Flap Not Yet Sensed - Initially
		
		if (!First_Sense) // Move Down to Sense the Bush
		{
			L_Arm_Speed = R_Arm_Speed  =  ARM_HOMING_SPEED ; 
            Pitch_Arm_Speed = ARM_HOMING_SPEED * 0.81; 
		}
	
		else  // Bush - Sensed Succesfully
		{
			if ( Front_Bush ) // Gap - Not Detected
			{
				Flap_Error = Front_Left_Bush == 1 ? Flaps_Target - Flap_Mod_Value : Right_Arm_Current_Pos - Left_Arm_Current_Pos; 
				L_Arm_Speed  = Left_Arm_PID ( Flap_Error , NULL );

				Flap_Error_Right = Front_Right_Bush == 1 ? Flaps_Target_Right - Flap_Mod_Value_Right : Left_Arm_Current_Pos - Right_Arm_Current_Pos; 
				R_Arm_Speed  = Right_Arm_PID ( Flap_Error_Right , NULL );

				Shear_Height_Diff = L_Arm_Travel - R_Arm_Travel;
				Shear_Roll_Angle = (atan ( Shear_Height_Diff / 2245 )) * (180/3.14);

				Pitch_Compensation_mm = fabs(Shear_Height_Diff * 0.435f) ;
			// Pitch_Compensation_Rot = Pitch_Compensation_mm / 3.32 ;
				Pitch_Compensation_mm = Shear_Roll_Angle > 0 ? -Pitch_Compensation_mm : Pitch_Compensation_mm;

			// Pitch_Target = (Left_Arm_Current_Pos * 0.81 ) + Pitch_Compensation_Rot;
				Pitch_Target = ((L_Arm_Travel + Pitch_Compensation_mm) / 3.32f) * 0.81f ;
				Pitch_Error = Pitch_Target - Pitch_Arm_Current_Pos;
				Pitch_Arm_Speed  = Pitch_Arm_PID ( Pitch_Error , NULL );
					
			}
			
			else // Gap - Detected
			{
				//L_Arm_Speed 		= Left_Arm_Motor_Count  > 1 ? -ARM_HOMING_SPEED : Left_Arm_Motor_Count  < -1 ? ARM_HOMING_SPEED : 0;    //0.5
				//R_Arm_Speed 		= Right_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Right_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;   //0.5
				//Pitch_Arm_Speed 	= Pitch_Arm_Motor_Count > 1 ? -ARM_HOMING_SPEED : Pitch_Arm_Motor_Count < -1 ? ARM_HOMING_SPEED : 0;   //0.5
				L_Arm_Speed = R_Arm_Speed = Pitch_Arm_Speed = 0;
			}
			
		}
		
		//L_Arm_Speed = R_Arm_Speed = Tri_Arm_Speed ; 
		//Pitch_Arm_Speed = Tri_Arm_Speed * 0.81 ; 

	}
	
	else {}
		
	L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count > 25 ? 10 : L_Arm_Speed < 0 && Left_Arm_Motor_Count < 15 ? -10 : L_Arm_Speed;
	L_Arm_Speed = L_Arm_Speed > 0 && Left_Arm_Motor_Count > 35 ? 0 : L_Arm_Speed < 0 && Left_Arm_Motor_Count < 0 ? 0 : L_Arm_Speed; // Arm - Boundaries
	
	if( L_Arm_Speed_Temp != L_Arm_Speed )
	{
		for(uint8_t i = 0; i <= 2; i++)
		{
			Set_Motor_Velocity (12 , L_Arm_Speed );	
		}
		L_Arm_Speed_Temp = L_Arm_Speed ;
	}
	R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count > 25 ? 10 : R_Arm_Speed < 0 && Right_Arm_Motor_Count < 15 ? -10 : R_Arm_Speed;
	R_Arm_Speed = R_Arm_Speed > 0 && Right_Arm_Motor_Count > 35 ? 0 : R_Arm_Speed < 0 && Right_Arm_Motor_Count < 0 ? 0 : R_Arm_Speed;
	if( R_Arm_Speed_Temp != R_Arm_Speed )
	{
		for(uint8_t i = 0; i <= 2; i++)
		{
			Set_Motor_Velocity (13 , R_Arm_Speed );	
		}
		R_Arm_Speed_Temp = R_Arm_Speed ;
	}
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count > 15 ? 10 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count < 5 ? -10 : Pitch_Arm_Speed;
	Pitch_Arm_Speed = Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count > 20 ? 0 : Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count < 0 ? 0 : Pitch_Arm_Speed;
	if( Pitch_Arm_Speed_Temp != Pitch_Arm_Speed )
	{
		for(uint8_t i = 0; i <= 2; i++)
		{
			Set_Motor_Velocity (14 , Pitch_Arm_Speed );	
		}
		Pitch_Arm_Speed_Temp = Pitch_Arm_Speed ;
	}

}
void Dynamic_Width_Adjustment (void)
{
	//float Width_Speed=46.5/2 - 5;
	float Width_Speed= 40;
	
	if ( !Steering_Reset_Flag  && Steering_Mode >= 4 && Rover_Velocity > 10 )
	{
		if ( Steering_Mode == WIDTH_SHRINK && Angle_Ready ) 
		{
			if ( Joystick == 1 )Width_Motor_Speed =  Width_Speed;
			else if ( Joystick == 2 ) Width_Motor_Speed  = -Width_Speed;
			else Width_Motor_Speed = 0;
		}
		
		if ( Steering_Mode == WIDTH_EXTEND  && Angle_Ready ) 
		{
			if ( Joystick ==1 ) Width_Motor_Speed = -Width_Speed;
			else if ( Joystick == 2 ) Width_Motor_Speed = Width_Speed;
			else Width_Motor_Speed = 0;
		}
		
		else if ( Steering_Mode < 4 )
		{
			Width_Motor_Speed = 0;
	
		}

	}
	else 
	{
		Width_Motor_Speed = 0 ;
		Dynamic_Width_Corrections();
	}
	
	if( Steering_Mode < 4 ) Width_Motor_Speed = 0;
	Lower_Width_Motor_Speed = Upper_Width_Motor_Speed = Width_Motor_Speed ;
	
//	if (Steering_Mode >= 4)
//	{
//		L_Roll_Err = Left_Roll_Pos - L_Roll;
//		L_Roll_Err = L_Roll_Err <= 0.3 && L_Roll_Err >= -0.3 ? 0 : L_Roll_Err;
//		Width_Correction_Speed = Left_Arm_PID(L_Roll_Err, 0);
//		Width_Correction_Speed = Width_Correction_Speed >= 40 ? 40 : Width_Correction_Speed < -40 ? -40 : Width_Correction_Speed;
//		Upper_Width_Motor_Speed = Upper_Width_Motor_Speed + Width_Correction_Speed;
//	}
	
		Lower_Width_Motor_Speed = Lower_Width_Motor_Speed < 0 && Lower_Width_Motor_Count <= 0 ? 0 : Lower_Width_Motor_Speed > 0 && Lower_Width_Motor_Count >= 600 ? 0 : Lower_Width_Motor_Speed;
		Upper_Width_Motor_Speed = Upper_Width_Motor_Speed < 0 && Upper_Width_Motor_Count  <= 0 ? 0 : Upper_Width_Motor_Speed > 0 && Upper_Width_Motor_Count >= 600 ? 0 : Upper_Width_Motor_Speed;
	//Lower_Width_Motor_Speed=(( Lower_Width_Motor_Speed < 0) && (Lower_Width_Motor_Count<=-550  )) ? -10:(( Lower_Width_Motor_Speed > 0) && ( Lower_Width_Motor_Count >= -50 ))?10:Lower_Width_Motor_Speed;
	//if (( Lower_Width_Motor_Speed < 0) && (( Lower_Width_Motor_Count >= -605 && Lower_Width_Motor_Count<=-595  )|| Lower_Width_Motor_Count <= -605)) Lower_Width_Motor_Speed = 0;
	//else if (( Lower_Width_Motor_Speed > 0) && (( Lower_Width_Motor_Count >=-5 && Lower_Width_Motor_Count<=5 )||Lower_Width_Motor_Count>=5 )) Lower_Width_Motor_Speed = 0;
	//else{}
	
	//Upper_Width_Motor_Speed=(( Upper_Width_Motor_Speed < 0) && (Upper_Width_Motor_Count<=-550  )) ? -10:(( Upper_Width_Motor_Speed > 0) && ( Upper_Width_Motor_Count >= -50 ))?10:Upper_Width_Motor_Speed;
	//if (( Upper_Width_Motor_Speed < 0) &&(( Upper_Width_Motor_Count >= -605 && Upper_Width_Motor_Count<=-595  )|| Upper_Width_Motor_Count <= -605))  Upper_Width_Motor_Speed = 0;
	//else if (( Upper_Width_Motor_Speed > 0) &&(( Upper_Width_Motor_Count >=-5 && Upper_Width_Motor_Count<=5 )||Upper_Width_Motor_Count>=5 )) Upper_Width_Motor_Speed = 0;
	//else{}
	
	//Upper_Width_Motor_Speed = Upper_Width_Motor_Count < Lower_Width_Motor_Count + 5 && Upper_Width_Motor_Count > Lower_Width_Motor_Count - 5 ? 0 : Upper_Width_Motor_Count < Lower_Width_Motor_Count - 5 ? -Width_Speed : Upper_Width_Motor_Count > Lower_Width_Motor_Count + 5 ? Width_Speed : Upper_Width_Motor_Speed; 
	
	if ( Width_Motor_Speed != Width_Motor_Temp || Lower_Width_Motor_Speed != Lower_Width_Motor_Speed_Temp || Upper_Width_Motor_Speed != Upper_Width_Motor_Speed_Temp )
	{
		Input_Velocity[15] = Lower_Width_Motor_Speed;
		Input_Velocity[16] = Upper_Width_Motor_Speed;
		for ( uint8_t i = 0 ; i < 4 ; i++)
		{
		Set_Motor_Velocity ( 15, Lower_Width_Motor_Speed); 
		Set_Motor_Velocity ( 16, Upper_Width_Motor_Speed);
		}
		Width_Motor_Temp = Width_Motor_Speed; 
		Lower_Width_Motor_Speed_Temp=Lower_Width_Motor_Speed;
		Upper_Width_Motor_Speed_Temp=Upper_Width_Motor_Speed;
		
	}

}

void Dynamic_Width_Corrections(void)
{
	
	Half_Track_Width = (1800 + (Lower_Width_Motor_Count * 0.25f)) / 2;  //Track_Width
	Half_Wheel_Base = Wheel_Base / 2;
	Track_Width = 1800 + (Lower_Width_Motor_Count * 0.25f);    //TRack_Width
	Zero_Turn_Angle = atan(Half_Wheel_Base / Half_Track_Width);
	Zero_Turn_Angle = Zero_Turn_Angle * (180 / 3.14);
}

void Shearing_Motors (void)
{
		if ( Shearing_Temp != Shearing )
	{
		if ( Shearing == 2 )
		{
//			if (HAL_GetTick() - Shearing_Tick >= 1500)
//			{
//				for (uint8_t i = 18; i <= 20; i++)
//				{
//					
////					if (Node_Id[i] != Node_Id_Temp[i])
////					{
////						Shearing_Drive_Disconnected = NULL;
////						Node_Id_Temp[i] = Node_Id[i];
////					}
////					
////					else
////					{
////						Shearing_Drive_Disconnected = SET;
////						Set_Motor_Velocity( 17 , 0 ); 
////						Set_Motor_Velocity( 18 , 0 ); 
////						Set_Motor_Velocity( 19 , 0 );
////					}
//					
//					if (Axis_State[i] != 8)
//					{
//						Shearing_Drive_Errored = SET;
//						Set_Motor_Velocity( 20 , 0 ); 
//						Set_Motor_Velocity( 18 , 0 ); 
//						Set_Motor_Velocity( 19 , 0 ); 
//						if (HAL_GetTick() - Reboot_Tick >= 3000)
//						{
//							Reboot(i);
//							Reboot_Tick = HAL_GetTick();
//						}
//					}
//					
//					else
//					{
//						Reboot_Tick = 0;
//						Shearing_Drive_Errored = NULL;
//					}
//					
//				}
//				
//				Shearing_Tick = HAL_GetTick();
		//	}
			
//			if (Shearing_Drive_Errored == NULL && Shearing_Drive_Disconnected == NULL)
//			{
					for ( int i=0; i < 2; i++)
					{			
						Set_Motor_Velocity( 18 , 40 );// HAL_Delay(10); // SELECTIVE
						Set_Motor_Velocity( 20 , 20 );// HAL_Delay(10); // MAIN PADDLE
						Set_Motor_Velocity( 19 , 20 ); //HAL_Delay(10);	// SIDE PADDLE
						//Set_Motor_Velocity( 20 , 20 ); 							// CUTTER
					}
			//}
		}
		else 
		{	
			for ( int i=0; i < 4 ; i++)
			{
				Set_Motor_Velocity( 18 , 0 ); 
				Set_Motor_Velocity( 20 , 0 ); 
				Set_Motor_Velocity( 19 , 0 ); 
				//Set_Motor_Velocity( 20 , 0 );				
			}
		}
		
		 Shearing_Temp = Shearing ;
	}
}

void Position_Flap_Sensing(void)
{
		FL_Angle = Flap_Data[5] ;
		Front_Left_Bush  =  FL_Angle > 45 ? 0 : 1;
	
	
	if (Mode == 0 )
	{
			Tri_Arm_Pos = 0;
		Left_Arm_Pos = Right_Arm_Pos = Pitch_Arm_Pos = Tri_Arm_Pos;
	}
	
	else if ( Mode == 1)
	{
		if ( Front_Left_Bush )
		{
				
			Arm_Angle = (FL_Angle /3.2f) -1 ;//3.2
			Tri_Arm_Pos = (Arm_Angle/360)*600;
			Left_Arm_Pos = Right_Arm_Pos = Tri_Arm_Pos;
			Pitch_Arm_Pos = Tri_Arm_Pos * 0.6082f ;
		}
		else 
		{
		}
	}
	
	
	if( Left_Arm_Pos_Temp != Left_Arm_Pos ) 
	{
		Set_Motor_Position (L_Arm , Left_Arm_Pos );	
		Left_Arm_Pos_Temp = Left_Arm_Pos ;
	}
	
	if( Right_Arm_Pos_Temp != Right_Arm_Pos ) 
	{
		Set_Motor_Position (R_Arm , Right_Arm_Pos );	
		Right_Arm_Pos_Temp = Right_Arm_Pos ;
	}
	
	if( Pitch_Arm_Pos_Temp != Pitch_Arm_Pos ) 
	{
		Set_Motor_Position (P_Arm , Pitch_Arm_Pos );	
		Pitch_Arm_Pos_Temp = Pitch_Arm_Pos ;
	}

}

void Demo()
{
	switch (Mode)
	{
		case 1:
		{
			L_Arm_Speed = Left_Arm_Motor_Count > 1 ? -5 : Left_Arm_Motor_Count < -1 ? 5 : 0;
			R_Arm_Speed = Right_Arm_Motor_Count > 1 ? -5 : Right_Arm_Motor_Count < -1 ? 5 : 0;
			Pitch_Arm_Speed = Pitch_Arm_Motor_Count > 1 ? -5 : Pitch_Arm_Motor_Count < -1 ? 5 : 0;
		}
		break;
		
		case 2:
		{
			switch (Joystick)
			{
				case 3:	
				L_Arm_Speed = 10;
				R_Arm_Speed = 10;
				Pitch_Arm_Speed = 10;
				break;
				
				case 4:
				L_Arm_Speed = -10;
				R_Arm_Speed = -10;
				Pitch_Arm_Speed = -10;
				break;
				
				default: 
				L_Arm_Speed = 0;
				R_Arm_Speed = 0;
				Pitch_Arm_Speed = 0;
				break;
			}
		}
		break;
		
				case 3:
		{
			switch (Joystick)
			{
				case 3: 
				L_Arm_Speed = 5;
				R_Arm_Speed = -5;
				Pitch_Arm_Speed = 0;
				break;
				
				case 4:
				L_Arm_Speed = -5;
				R_Arm_Speed = 5;
				Pitch_Arm_Speed = 0;
				break;
				
				default:
				L_Arm_Speed = 0;
				R_Arm_Speed = 0;
				Pitch_Arm_Speed = 0;
				break;
			}
		}
		break;
		
		default: break;
	}
	
	L_Arm_Speed = (L_Arm_Speed > 0 && Left_Arm_Motor_Count >= 35) ? 0 : (L_Arm_Speed < 0 && Left_Arm_Motor_Count <= 0) ? 0 : L_Arm_Speed;
	R_Arm_Speed = (R_Arm_Speed > 0 && Right_Arm_Motor_Count >= 35) ? 0 : (R_Arm_Speed < 0 && Right_Arm_Motor_Count <= 0) ? 0 : R_Arm_Speed;
	Pitch_Arm_Speed = (Pitch_Arm_Speed > 0 && Pitch_Arm_Motor_Count >= 17) ? 0 : (Pitch_Arm_Speed < 0 && Pitch_Arm_Motor_Count <= 0) ? 0 : Pitch_Arm_Speed;
	Pitch_Arm_Speed = Joystick == 3 ? Pitch_Arm_Speed * 0.5 : Joystick == 4 ? Pitch_Arm_Speed * 0.5 : Pitch_Arm_Speed;
	if (L_Arm_Speed != L_Arm_Speed_Temp)
	{
		for (uint8_t i = 0; i < 5; i++)
		{
			Set_Motor_Velocity (12, L_Arm_Speed);
		}
		L_Arm_Speed_Temp = L_Arm_Speed;
	}
	
	if (R_Arm_Speed != R_Arm_Speed_Temp)
	{
		for (uint8_t i = 0; i < 5; i++)
		{
			Set_Motor_Velocity (13, R_Arm_Speed);
		}
		R_Arm_Speed_Temp = R_Arm_Speed;
	}
	
	if (Pitch_Arm_Speed != Pitch_Arm_Speed_Temp)
	{
		for (uint8_t i = 0; i < 5; i++)
		{
			Set_Motor_Velocity (14, Pitch_Arm_Speed);
		}
		Pitch_Arm_Speed_Temp = Pitch_Arm_Speed;
	}
}
void New_Drive_Controls_V2(void)
{
	if ( (Speed!= 0) && Left_IMU_State  ) //&& (Steering_Mode!= 1) )//&& (BT_State))   // mode == 2 added
	{
		Rover_Velocity = (fabs(Motor_Velocity[1]) + fabs(Motor_Velocity[2]) + fabs(Motor_Velocity[3]) + fabs(Motor_Velocity[4])) / 4;
//		Vel_Limit = Speed*15;
//		Vel_Limit = Vel_Limit > 50 ? 50 : Vel_Limit < 10 ? 10 : Vel_Limit;

	//	if ( (R_R_Err > 6 || R_R_Err < -6) || (C_Err > 6 || C_Err < -6) || (Left_Vertical_Error > 6 || Left_Vertical_Error < -6) ){ Joystick = 0;}// Stop_Motors(); }// Safety STOP  (L_R_Err > 5 || L_R_Err < -5)
	
//		if ( Joystick == 1 )
//		{
//		if ( Motor_Velocity[3] > 10 ) Vel_Limit = Speed*15;
//		}
//		else if ( Joystick == 2 )
//		{
//		if ( Motor_Velocity[3] < -10 ) Vel_Limit = Speed*15;
//		}
		
		Vel_Limit = Joystick == 0 ? 0 : Speed * 15;  		//15
		//Vel_Limit = Joystick == 1 ? Speed * 15 : Joystick == 2 ? Speed * -15 : 0;
		if(Joystick != 0 && Steering_Mode != 1)
		{
			Vel_Limit = 15;
			Left_Frame_Speed = 0;
		}
		
		else if (Joystick == 0 && Steering_Mode != 1)
		{
			Vel_Limit = 0;
			Left_Frame_Speed = 0;
		}
 //if ((Joystick!=0)&&(Steering_Mode==ALL_WHEEL)&&(LF_Steering >=25 || RF_Steering<=-25)){	Vel_Limit = 15;}
		
		
		if ( Steering_Mode != ALL_WHEEL ){ Left_Steering_Speed = Right_Steering_Speed = 0; }
	//		Left_Frame_Speed=0;
			Left_Vel_Limit = Vel_Limit;  //+ Left_Steering_Speed + Left_Frame_Speed +4;
			Right_Vel_Limit = Vel_Limit; //+ Right_Steering_Speed+4;
	/////////////////////////////////////////////////////////////////////////APPLYING TORQUE/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		if ( Motor_Velocity[3] < 10 && Motor_Velocity[3] > -10 )
//		{
//		Transmit_Motor_Torque();
//		}
		
			
				// if (Joystick == 1 || Joystick == 2)
				// {
				// 	if (Joystick != Joystick_Temp)
				// 	{
				// 		Transmit_Motor_Torque();
				// 		Joystick_Temp = Joystick;
				// 		//FLAG = SET;
				// 	}
				// }
				
				// if (Joystick == 0)
				// {
				// 	//if ((Motor_Velocity[1] <= Left_Vel_Limit + 2 && Motor_Velocity[1] >= Left_Vel_Limit - 2) && (Motor_Velocity[3] <= Right_Vel_Limit + 2 && Motor_Velocity[3] >= Right_Vel_Limit - 2))
				// 	//{
				// 		Transmit_Motor_Torque();
				// 		//FLAG = NULL;
				// 	//}
				
//				if (( Joystick != 0 && Prev_Joystick == 0) || ( Joystick == 0 && Prev_Joystick != 0))
//				{
//					if( Rover_Velocity < 12 && Rover_Velocity > -12) Transmit_Motor_Torque();
//					else {}
//				}
//				else
//				{
////					 Transmit_Motor_Torque();
////						count++;
//				}
					

				// if ( Joystick == 0 && Prev_Joystick != 0)
				// {
				// 	if( Motor_Velocity[3] < 12 ) Transmit_Motor_Torque();
				// 	else {}
				// 	Prev_Joystick = Joystick;
				// }
				

			if(Left_Vel_Limit != Left_Transmit_Vel)
			{
				if(HAL_GetTick() - left_tick_count >= 50)
				{
//					if (Joystick == 1)
//					{
						if(Left_Vel_Limit > Left_Transmit_Vel)
						{
							Left_Transmit_Vel++;
						}
						
						else if(Left_Vel_Limit < Left_Transmit_Vel)
						{
							//Left_Transmit_Vel--;
							
							if ( fabs(Motor_Velocity[3]) <= 20 ) Left_Transmit_Vel--;
							else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Left_Transmit_Vel = Left_Transmit_Vel - 3 ;
							else Left_Transmit_Vel = Left_Transmit_Vel - 5;
						}
						else{}
				//	}
					
//					else if (Joystick == 2)
//					{
//						if(Left_Vel_Limit < Left_Transmit_Vel)
//						{
//							Left_Transmit_Vel++;
//						}
//						
//						else if(Left_Vel_Limit > Left_Transmit_Vel)
//						{
//							//Left_Transmit_Vel--;
//							
//							if ( fabs(Motor_Velocity[3]) <= 20 ) Left_Transmit_Vel--;
//							else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Left_Transmit_Vel = Left_Transmit_Vel - 3 ;
//							else Left_Transmit_Vel = Left_Transmit_Vel - 5;
//						}
//						else{}
//					}
					
					//else{}
					Left_Transmit_Vel = Joystick == 2 ? -Left_Transmit_Vel : Left_Transmit_Vel;
					//for(uint8_t i=1 ; i <= 2 ; i++) { Set_Motor_Velocity(i, Left_Transmit_Vel); HAL_Delay(1);}
					left_tick_count = HAL_GetTick();
				
			}
		}
			
			if(Right_Vel_Limit != Right_Transmit_Vel)
			{
				if(HAL_GetTick() - right_tick_count >= 50)
				{
//					if (Joystick == 1)
//					{
						if(Right_Vel_Limit > Right_Transmit_Vel)
						{
							Right_Transmit_Vel++;
						}
						
						else if(Right_Vel_Limit < Right_Transmit_Vel)
						{
							//Right_Transmit_Vel--;
							
							if ( fabs(Motor_Velocity[3]) <= 20 ) Right_Transmit_Vel--;
							else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Right_Transmit_Vel = Right_Transmit_Vel - 3 ;
							else Right_Transmit_Vel = Right_Transmit_Vel - 5;
						}
						
						else{}
					//}	
					
//					else if (Joystick == 2)
//					{
//						if(Right_Vel_Limit < Right_Transmit_Vel)
//						{
//							Right_Transmit_Vel++;
//						}
//						
//						else if(Right_Vel_Limit > Right_Transmit_Vel)
//						{
//							//Right_Transmit_Vel--;
//							
//							if ( fabs(Motor_Velocity[3]) <= 20 ) Right_Transmit_Vel--;
//							else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Right_Transmit_Vel = Right_Transmit_Vel - 3 ;
//							else Right_Transmit_Vel = Right_Transmit_Vel - 5;
//						}
//						
//						else{}
//					}
//					
//					else{}
					Right_Transmit_Vel = Joystick == 2 ? -Right_Transmit_Vel : Right_Transmit_Vel;
					//for(uint8_t i=3 ; i <= 4 ; i++) {Set_Motor_Velocity(i, Right_Transmit_Vel) ;HAL_Delay(1);}
					right_tick_count = HAL_GetTick();
				}
			}

	}	
	else 
	{
		for ( uint8_t i = 1 ; i < 5 ; i++ )
		{ 
				Set_Motor_Velocity ( i , 0 ); HAL_Delay(1);
		}
	}
	
//	if ( (!BT_State ) && Joystick != 0 )
//	{
//		for ( uint8_t i = 1 ; i < 5 ; i++ ){Set_Motor_Torque ( i , NULL ); HAL_Delay(1);}
//		Joystick = 0 ;
//	}
}

void Drive_Wheel_Controls_Vel_Based(void)
{
	if (Mode == 1)
	{
	Input_Vel = Speed * 35 ;
	Input_Vel = Steering_Mode != 1 ? 15 : Input_Vel;
	Left_Steering_Speed = Steering_Mode != 1 ? 0 : Left_Steering_Speed;
	Right_Steering_Speed = Steering_Mode != 1 ? 0 : Right_Steering_Speed;
	
	if ( Joystick_Temp != Joystick || Speed != Speed_Temp)
	{
			switch(Joystick)
			{
				case 1 : Vel_Limit = Input_Vel; break;
				case 2 : Vel_Limit = -Input_Vel; break;
				case 0 : Vel_Limit = 0; break;
				default: break;
			}
			
			Joystick_Temp = Joystick;
			Speed_Temp = Speed;
	}
	
		Left_Vel_Limit = Vel_Limit + Left_Steering_Speed;  
		Right_Vel_Limit = Vel_Limit + Right_Steering_Speed;

		
//	if (Steering_Mode < 4)
//	{
	if(Left_Vel_Limit != Left_Transmit_Vel)
		{
				Accel_Sync = fabs(Right_Vel_Limit - Right_Transmit_Vel);
				Accel_Sync = Accel_Sync == 0 ? 1 : Accel_Sync;
				Left_Diff = fabs ((Left_Vel_Limit - Left_Transmit_Vel) / Accel_Sync ) ; 
				Left_Diff = Left_Diff > -100 && Left_Diff < 100 ? Left_Diff : 0;
				
				
				if(HAL_GetTick() - left_tick_count >= 50)
				{
					if(Left_Vel_Limit > Left_Transmit_Vel)
					{
						//Left_Transmit_Vel++;
							Left_Transmit_Vel = Left_Transmit_Vel + Left_Diff;
					}
					
					else if(Left_Vel_Limit < Left_Transmit_Vel)
					{
						//Left_Transmit_Vel--;
						Left_Transmit_Vel = Left_Transmit_Vel - Left_Diff;
//						if ( fabs(Motor_Velocity[3]) <= 20 ) Left_Transmit_Vel--;
//						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Left_Transmit_Vel = Left_Transmit_Vel - 3 ;
//						else Left_Transmit_Vel = Left_Transmit_Vel - 5;
					}
					else{}
					Input_Velocity[1] = Left_Transmit_Vel;
					Input_Velocity[2] = Left_Transmit_Vel;
					for(uint8_t i=1 ; i <= 2 ; i++) { Set_Motor_Velocity(i, Left_Transmit_Vel); } //HAL_Delay(1);
					left_tick_count = HAL_GetTick();
				
			}
		}
			
			if(Right_Vel_Limit != Right_Transmit_Vel)
			{
				if(HAL_GetTick() - right_tick_count >= 50)
				{
					if(Right_Vel_Limit > Right_Transmit_Vel)
					{
						Right_Transmit_Vel++;
					}
					
					else if(Right_Vel_Limit < Right_Transmit_Vel)
					{
						Right_Transmit_Vel--;
						
//						if ( fabs(Motor_Velocity[3]) <= 20 ) Right_Transmit_Vel--;
//						else if ( fabs(Motor_Velocity[3]) > 20 && fabs(Motor_Velocity[3]) <= 30 ) Right_Transmit_Vel = Right_Transmit_Vel - 3 ;
//						else Right_Transmit_Vel = Right_Transmit_Vel - 5;
					}
					
					else{}
						
						
					Input_Velocity[3] = Right_Transmit_Vel;
					Input_Velocity[4] = Right_Transmit_Vel;
					for(uint8_t i=3 ; i <= 4 ; i++) { Set_Motor_Velocity(i, Right_Transmit_Vel); }  //HAL_Delay(1);
					right_tick_count = HAL_GetTick();
				}
			}

	MODE_CHANGE_FLAG = SET;
			
	}
	
	else
	{
		if (MODE_CHANGE_FLAG == SET)
		{
				for (uint8_t i = 1; i < 5; i++)
				{
					for (uint8_t j = 0; j < 5; j++)
					{
						Set_Motor_Velocity (i, 0);
					}
				}
				
				MODE_CHANGE_FLAG = NULL;
		}
	}
}

void New_Steering_Controls_ (void)
{
/*	If the Steering Reset Flag is SET, all the Steering wheels will return to their Home Position.	
		Flag Sets on Power Up and at every Steering Mode Change.																*/	
	
	if ( Steering_Reset_Flag )
	{
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		if ( (LFS_Filtered <= STEERING_BOUNDARY ) && ( LFS_Filtered >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
		else {LF_Speed = ( LFS_Filtered > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LFS_Filtered < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
		
		if ( (LRS_Filtered <= STEERING_BOUNDARY ) && ( LRS_Filtered >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
		else {LR_Speed = ( LRS_Filtered > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LRS_Filtered < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		
		if ( (RFS_Filtered <= STEERING_BOUNDARY ) && ( RFS_Filtered >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
		else {RF_Speed = ( RFS_Filtered > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RFS_Filtered < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
		
		if ( (RRS_Filtered <= STEERING_BOUNDARY ) && ( RRS_Filtered >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
		else {RR_Speed = ( RRS_Filtered > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RRS_Filtered < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		
		LF_Error = LR_Error = RF_Error = RR_Error = 1;
		if ((LF_SET) && (LR_SET) && (RF_SET) && (RR_SET) )	
		{
			LF_SET = LR_SET = RF_SET = RR_SET = NULL ;
			Steering_Reset_Flag=NULL;
		}
		else{}
	}
		
	else if( !Steering_Reset_Flag  )
	{
		switch ( Steering_Mode )
		{
//			case 0:
//						if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
//						else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
//						
//						if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
//						else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
//						
//						if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
//						else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
//						
//						if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
//						else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
//						break;
			case ALL_WHEEL :	 //	 --> ALL WHEEL STEERING  
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
							
							//Inner_Angle =	(( Pot_Angle / 2 ) - 45);
								Inner_Angle = (Pot_Angle - 90) / 3;
		
							
							if ( Inner_Angle <= -1 )  // Left Turn of the Rover
							{
								Avg_Steering_Angle = fabs((fabs(LFS_Filtered) + fabs(LRS_Filtered)) / 2);     //CHANGE
								LF = fabs(LFS_Filtered);
								LR = fabs(LRS_Filtered);
								New_Inner_Angle =  fabs((fabs(LFS_Filtered) + fabs(LRS_Filtered)) / 2);     //both should be either +ve or -ve
								New_Inner_Angle = -New_Inner_Angle;
								
								LF_Error = (-Inner_Angle - (LFS_Filtered)) ;		
								LR_Error = (Inner_Angle - (LRS_Filtered)) ; 		
								
								LF_Error = LF_Error <= 0.8f && LF_Error >= -0.8f ? 0 : LF_Error;
								LR_Error = LR_Error <= 0.8f && LR_Error >= -0.8f ? 0 : LR_Error;

								LF_Speed = LF_Error * STEERING_KP;
								LR_Speed = LR_Error * STEERING_KP;							
								
								//Outer_Angle = Differintial_Angle((-Inner_Angle ))	;
								Outer_Angle = Differintial_Angle((Avg_Steering_Angle ))	;		//CHANGE
									
								//Outer_Angle_2	= Differintial_Angle((-Inner_Angle ))	;
								Outer_Angle_2	= Differintial_Angle((Avg_Steering_Angle ))	;	   //CHANGE
								
								RF_Error = (-Outer_Angle - (-RFS_Filtered)) ;
								RR_Error = (-Outer_Angle - (RRS_Filtered)) ; 
								
								RF_Error = RF_Error <= 0.8f && RF_Error >= -0.8f ? 0 : RF_Error;
								RR_Error = RR_Error <= 0.8f && RR_Error >= -0.8f ? 0 : RR_Error;
								
								RF_Speed = RF_Error * STEERING_KP;
								RR_Speed = RR_Error * STEERING_KP;
							
								Prev_Inner_Angle = Inner_Angle;
							
							}

							else if ( Inner_Angle >= 0 ) // Right Turn of the Rover   //0
							{
								
								Avg_Steering_Angle = fabs((fabs(RFS_Filtered) + fabs(RRS_Filtered)) / 2);      //CHANGE
								New_Inner_Angle = fabs((fabs(RFS_Filtered) + fabs(RRS_Filtered)) / 2);     //both should be either +ve or -ve    
								
								RF_Error = (Inner_Angle - (-RFS_Filtered)) ;			
								RR_Error = (Inner_Angle - (RRS_Filtered)) ; 

								RF_Error = RF_Error <= 0.8f && RF_Error >= -0.8f ? 0 : RF_Error;
								RR_Error = RR_Error <= 0.8f && RR_Error >= -0.8f ? 0 : RR_Error;

								RF_Speed = RF_Error * STEERING_KP;
								RR_Speed = RR_Error * STEERING_KP;	
								
								//Outer_Angle = Differintial_Angle((Inner_Angle ))	;	
								Outer_Angle = Differintial_Angle((Avg_Steering_Angle ))	;		      //CHANGE
									
								//Outer_Angle_2	= Differintial_Angle((Inner_Angle ));
								Outer_Angle_2	= Differintial_Angle((Avg_Steering_Angle ));		    //CHANGE
								
								LF_Error = (-Outer_Angle- (LFS_Filtered)) ;
								LR_Error = (Outer_Angle_2 - (LRS_Filtered)) ; 
								
								LF_Error = LF_Error <= 0.8f && LF_Error >= -0.8f ? 0 : LF_Error;
								LR_Error = LR_Error <= 0.8f && LR_Error >= -0.8f ? 0 : LR_Error;
								
								LF_Speed = LF_Error * STEERING_KP;
								LR_Speed = LR_Error * STEERING_KP;
								
									
								Prev_Inner_Angle = Inner_Angle;
								
								
							}	
							
							else // Home Pos of the Rover
							{							
									
								Left_Steering_Speed = Right_Steering_Speed = NULL;	
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
					
							
							//Wheel_Speeds_Calc(Inner_Angle);
							Wheel_Speeds_Calc(New_Inner_Angle);                 //CHANGE
							break;
			/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/				
			case CRAB :							//	--> CRAB STEERING			
			/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/
			
							AW_Angle = ( Pot_Angle - 90) ; 
			
							LF_Speed = (LFS_Filtered > AW_Angle -STEERING_BOUNDARY && LFS_Filtered < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LFS_Filtered < AW_Angle ) ? STEERING_HOMING_SPEED: ( LFS_Filtered > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LRS_Filtered > AW_Angle -STEERING_BOUNDARY && LRS_Filtered < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LRS_Filtered < AW_Angle ) ? STEERING_HOMING_SPEED: ( LRS_Filtered > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;					
							RF_Speed = (RFS_Filtered > AW_Angle -STEERING_BOUNDARY && RFS_Filtered < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RFS_Filtered < AW_Angle-STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED: ( RFS_Filtered > AW_Angle+STEERING_BOUNDARY ) ?  STEERING_HOMING_SPEED : 0;
							RR_Speed = (RRS_Filtered > AW_Angle -STEERING_BOUNDARY && RRS_Filtered < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RRS_Filtered < AW_Angle ) ? STEERING_HOMING_SPEED: ( RRS_Filtered > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;
							break;
				/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/	

				case ZERO_TURN :
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
							AW_Angle = Zero_Turn_Angle ; 
			
							LF_Speed = (LFS_Filtered > -AW_Angle -STEERING_BOUNDARY && LFS_Filtered < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( LFS_Filtered < -AW_Angle ) ? STEERING_HOMING_SPEED: ( LFS_Filtered > -AW_Angle ) ? -STEERING_HOMING_SPEED :  0;		
							LR_Speed = (LRS_Filtered > AW_Angle -STEERING_BOUNDARY  && LRS_Filtered < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( LRS_Filtered < AW_Angle )  ? STEERING_HOMING_SPEED: ( LRS_Filtered > AW_Angle )  ? -STEERING_HOMING_SPEED :  0;
							RF_Speed = (RFS_Filtered > AW_Angle -STEERING_BOUNDARY  && RFS_Filtered < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( RFS_Filtered < AW_Angle )  ? -STEERING_HOMING_SPEED: ( RFS_Filtered > AW_Angle )  ? STEERING_HOMING_SPEED :  0;		
							RR_Speed = (RRS_Filtered > -AW_Angle -STEERING_BOUNDARY && RRS_Filtered < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( RRS_Filtered < -AW_Angle ) ? STEERING_HOMING_SPEED: ( RRS_Filtered > -AW_Angle ) ? -STEERING_HOMING_SPEED	:  0;

							break;
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
	/*			case FRONT_WHEEL :	 //	 --> FRONT WHEEL STEERING  */
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
			/*				Inner_Angle =	( Pot_Angle / 2 ) - 45;
								
							if ( Inner_Angle <= -1 )  
							{
							
							LF_Error = (-Inner_Angle - (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Outer_Angle = FWD_Differintial_Angle((-Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							RF_Error = (-Outer_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;									
							Outer_Angle_2	= FWD_Differintial_Angle((-Inner_Angle ))	;					
							Prev_Inner_Angle = Inner_Angle;
							}
							
							
							else if ( Inner_Angle >= 0 ) 
							{
							RF_Error = (Inner_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;	
							Outer_Angle = FWD_Differintial_Angle((Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							Outer_Angle_2	= FWD_Differintial_Angle((Inner_Angle ))	;		
							LF_Error = (-Outer_Angle- (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Prev_Inner_Angle = Inner_Angle;
							
							}	
							
							else 
							{								
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
							
							if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
							else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
							
							if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
							else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	

							break;*/
							
				case WIDTH_EXTEND: 
					
							LS_Angle = -WIDE_ANGLE; 
							RS_Angle =-WIDE_ANGLE; 
							LF_Speed = (LFS_Filtered > LS_Angle -STEERING_BOUNDARY && LFS_Filtered < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LFS_Filtered < LS_Angle ) ? STEERING_HOMING_SPEED: ( LFS_Filtered > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LRS_Filtered > LS_Angle -STEERING_BOUNDARY && LRS_Filtered < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LRS_Filtered < LS_Angle ) ? STEERING_HOMING_SPEED: ( LRS_Filtered > LS_Angle )? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LFS_Filtered > LS_Angle -STEERING_BOUNDARY && LFS_Filtered < LS_Angle +STEERING_BOUNDARY )&& (RFS_Filtered > RS_Angle -STEERING_BOUNDARY && RFS_Filtered < RS_Angle +STEERING_BOUNDARY ))? SET: NULL;
							Angle_Ready = ((LFS_Filtered > LS_Angle -STEERING_BOUNDARY && LFS_Filtered < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;
											
							break;
				
				case WIDTH_SHRINK: 
								
							LS_Angle = SHRINK_ANGLE; 
							RS_Angle = SHRINK_ANGLE; 
							LF_Speed = (LFS_Filtered > LS_Angle -STEERING_BOUNDARY && LFS_Filtered < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LFS_Filtered < LS_Angle ) ? STEERING_HOMING_SPEED: ( LFS_Filtered > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							LR_Speed = (LRS_Filtered > LS_Angle -STEERING_BOUNDARY && LRS_Filtered < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LRS_Filtered < LS_Angle ) ? STEERING_HOMING_SPEED: ( LRS_Filtered > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LFS_Filtered > LS_Angle -STEERING_BOUNDARY && LFS_Filtered < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;			
							break;
										
			default :  break;
		}
	}
					
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
	LF_Speed = LF_Speed > 0 && LFS_Filtered >= 90 ?  0 : LF_Speed < 0 && LFS_Filtered <= -90 ? 0 : LF_Speed;
	LR_Speed = LR_Speed > 0 && LRS_Filtered >= 90 ?  0 : LR_Speed < 0 && LRS_Filtered <= -90 ? 0 : LR_Speed;
	RF_Speed = RF_Speed < 0 && RFS_Filtered >= 90 ?  0 : RF_Speed > 0 && RFS_Filtered <= -90 ? 0 : RF_Speed;
	RR_Speed = RR_Speed > 0 && RRS_Filtered >= 90 ?  0 : RR_Speed < 0 && RRS_Filtered <= -90 ? 0 : RR_Speed;
	
			if ( LF_Speed_Temp != LF_Speed )
			{
				LF_Speed= LF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LF_Speed; 
				for (uint8_t i = 0; i < 5; i++)
				{
					Input_Velocity[8] = -LF_Speed;
					Set_Motor_Velocity( LFS , -LF_Speed );
				}
				LF_Speed_Temp = LF_Speed ;
			}
			
			if ( LR_Speed_Temp != LR_Speed )
			{
				LR_Speed= LR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LR_Speed;
				for(uint8_t i = 0; i < 2; i++)
				{
					Input_Velocity[9] = -LR_Speed;
				 Set_Motor_Velocity( LRS , -LR_Speed );
					
				}
				LR_Speed_Temp = LR_Speed ;
			}
						
			if ( RF_Speed_Temp != RF_Speed )
			{	
				RF_Speed= RF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RF_Speed;
				for (uint8_t i =0; i< 5; i++)
				{
					Input_Velocity[10] = -RF_Speed;
					Set_Motor_Velocity( RFS , -RF_Speed );
				}
				RF_Speed_Temp = RF_Speed ;
			}
										
			if ( RR_Speed_Temp != RR_Speed )
			{
				RR_Speed= RR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RR_Speed;
				for (uint8_t i = 0; i < 5; i++)
				{
					Input_Velocity[11] = -RR_Speed;
					Set_Motor_Velocity( RRS , -RR_Speed );
				}
				RR_Speed_Temp = RR_Speed ;
			}	
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
}
void New_Steering_Controls (void)
{
/*	If the Steering Reset Flag is SET, all the Steering wheels will return to their Home Position.	
		Flag Sets on Power Up and at every Steering Mode Change.																*/	
	
	if ( Steering_Reset_Flag )
	{
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
		else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
		
		if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
		else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		
		if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
		else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
		
		if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
		else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
		/*///////////////////////////////////////////////////////////	STEERING RESET CONTROLLER /////////////////////////////////////////////////////////	*/
		
		LF_Error = LR_Error = RF_Error = RR_Error = 1;
		if ((LF_SET) && (LR_SET) && (RF_SET) && (RR_SET) )	
		{
			LF_SET = LR_SET = RF_SET = RR_SET = NULL ;
			Steering_Reset_Flag=NULL;
		}
		else{}
	}
		
	else if( !Steering_Reset_Flag  )
	{
		switch ( Steering_Mode )
		{
//			case 0:
//						if ( (LF_Steering <= STEERING_BOUNDARY ) && ( LF_Steering >= -STEERING_BOUNDARY ) ) { LF_Speed = 0;	LF_SET = SET;} 
//						else {LF_Speed = ( LF_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LF_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}		
//						
//						if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
//						else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
//						
//						if ( (RF_Steering <= STEERING_BOUNDARY ) && ( RF_Steering >= -STEERING_BOUNDARY ) ) { RF_Speed = 0;	RF_SET = SET;} 
//						else {RF_Speed = ( RF_Steering > STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : ( RF_Steering < STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : 0;}	
//						
//						if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
//						else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
//						break;
			case ALL_WHEEL :	 //	 --> ALL WHEEL STEERING  
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
							
							//Inner_Angle =	(( Pot_Angle / 2 ) - 45);
								Inner_Angle = (Pot_Angle - 90) / 3.6;
		
							
							if ( Inner_Angle <= -1 )  // Left Turn of the Rover
							{
								Avg_Steering_Angle = fabs((fabs(LF_Steering) + fabs(LR_Steering)) / 2);     //CHANGE
								LF = fabs(LF_Steering);
								LR = fabs(LR_Steering);
								New_Inner_Angle =  fabs((fabs(LF_Steering) + fabs(LR_Steering)) / 2);     //both should be either +ve or -ve
								New_Inner_Angle = -New_Inner_Angle;
								
								LF_Error = (-Inner_Angle - (LF_Steering)) ;		
								LR_Error = (Inner_Angle - (LR_Steering)) ; 		
								
								LF_Error = LF_Error <= 0.8f && LF_Error >= -0.8f ? 0 : LF_Error;
								LR_Error = LR_Error <= 0.8f && LR_Error >= -0.8f ? 0 : LR_Error;

								LF_Speed = LF_Error * STEERING_KP;
								LR_Speed = LR_Error * STEERING_KP;							
								
								//Outer_Angle = Differintial_Angle((-Inner_Angle ))	;
								Outer_Angle = Differintial_Angle((Avg_Steering_Angle ))	;		//CHANGE
									
								//Outer_Angle_2	= Differintial_Angle((-Inner_Angle ))	;
								Outer_Angle_2	= Differintial_Angle((Avg_Steering_Angle ))	;	   //CHANGE
								
								RF_Error = (-Outer_Angle - (-RF_Steering)) ;
								RR_Error = (-Outer_Angle - (RR_Steering)) ; 
								
								RF_Error = RF_Error <= 0.8f && RF_Error >= -0.8f ? 0 : RF_Error;
								RR_Error = RR_Error <= 0.8f && RR_Error >= -0.8f ? 0 : RR_Error;
								
								RF_Speed = RF_Error * STEERING_KP;
								RR_Speed = RR_Error * STEERING_KP;
							
								Prev_Inner_Angle = Inner_Angle;
							
							}

							else if ( Inner_Angle >= 0 ) // Right Turn of the Rover   //0
							{
								
								Avg_Steering_Angle = fabs((fabs(RF_Steering) + fabs(RR_Steering)) / 2);      //CHANGE
								New_Inner_Angle = fabs((fabs(RF_Steering) + fabs(RR_Steering)) / 2);     //both should be either +ve or -ve    
								
								RF_Error = (Inner_Angle - (-RF_Steering)) ;			
								RR_Error = (Inner_Angle - (RR_Steering)) ; 

								RF_Error = RF_Error <= 0.8f && RF_Error >= -0.8f ? 0 : RF_Error;
								RR_Error = RR_Error <= 0.8f && RR_Error >= -0.8f ? 0 : RR_Error;

								RF_Speed = RF_Error * STEERING_KP;
								RR_Speed = RR_Error * STEERING_KP;	
								
								//Outer_Angle = Differintial_Angle((Inner_Angle ))	;	
								Outer_Angle = Differintial_Angle((Avg_Steering_Angle ))	;		      //CHANGE
									
								//Outer_Angle_2	= Differintial_Angle((Inner_Angle ));
								Outer_Angle_2	= Differintial_Angle((Avg_Steering_Angle ));		    //CHANGE
								
								LF_Error = (-Outer_Angle- (LF_Steering)) ;
								LR_Error = (Outer_Angle_2 - (LR_Steering)) ; 
								
								LF_Error = LF_Error <= 0.8f && LF_Error >= -0.8f ? 0 : LF_Error;
								LR_Error = LR_Error <= 0.8f && LR_Error >= -0.8f ? 0 : LR_Error;
								
								LF_Speed = LF_Error * STEERING_KP;
								LR_Speed = LR_Error * STEERING_KP;
								
									
								Prev_Inner_Angle = Inner_Angle;
								
								
							}	
							
							else // Home Pos of the Rover
							{							
									
								Left_Steering_Speed = Right_Steering_Speed = NULL;	
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
					
							
							//Wheel_Speeds_Calc(Inner_Angle);
							Wheel_Speeds_Calc(New_Inner_Angle);                 //CHANGE
							break;
			/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/				
			case CRAB :							//	--> CRAB STEERING			
			/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/
			
							AW_Angle = ( Pot_Angle - 90) * 0.9; 
			
							LF_Speed = (LF_Steering > AW_Angle -STEERING_BOUNDARY && LF_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LR_Steering > AW_Angle -STEERING_BOUNDARY && LR_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;					
							RF_Speed = (RF_Steering > AW_Angle -STEERING_BOUNDARY && RF_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RF_Steering < AW_Angle-STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED: ( RF_Steering > AW_Angle+STEERING_BOUNDARY ) ?  STEERING_HOMING_SPEED : 0;
							RR_Speed = (RR_Steering > AW_Angle -STEERING_BOUNDARY && RR_Steering < AW_Angle +STEERING_BOUNDARY ) ? 0 : ( RR_Steering < AW_Angle ) ? STEERING_HOMING_SPEED: ( RR_Steering > AW_Angle ) ? -STEERING_HOMING_SPEED : 0;
							break;
				/*///////////////////////////////////////////////////////////////////////////////////	CRAB STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////////	*/	

				case ZERO_TURN :
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
							AW_Angle = Zero_Turn_Angle ; 
			
							LF_Speed = (LF_Steering > -AW_Angle -STEERING_BOUNDARY && LF_Steering < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( LF_Steering < -AW_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > -AW_Angle ) ? -STEERING_HOMING_SPEED :  0;		
							LR_Speed = (LR_Steering > AW_Angle -STEERING_BOUNDARY  && LR_Steering < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( LR_Steering < AW_Angle )  ? STEERING_HOMING_SPEED: ( LR_Steering > AW_Angle )  ? -STEERING_HOMING_SPEED :  0;
							RF_Speed = (RF_Steering > AW_Angle -STEERING_BOUNDARY  && RF_Steering < AW_Angle +STEERING_BOUNDARY )  ? 0  : ( RF_Steering < AW_Angle )  ? -STEERING_HOMING_SPEED: ( RF_Steering > AW_Angle )  ? STEERING_HOMING_SPEED :  0;		
							RR_Speed = (RR_Steering > -AW_Angle -STEERING_BOUNDARY && RR_Steering < -AW_Angle +STEERING_BOUNDARY ) ? 0  : ( RR_Steering < -AW_Angle ) ? STEERING_HOMING_SPEED: ( RR_Steering > -AW_Angle ) ? -STEERING_HOMING_SPEED	:  0;

							break;
				/*///////////////////////////////////////////////////////////////////////////////////	ZERO TURN - STEERING FUNCTION ////////////////////////////////////////////////////////////////////////////////////////	*/
	/*			case FRONT_WHEEL :	 //	 --> FRONT WHEEL STEERING  */
							/*///////////////////////////////////////////////////////////////////////////////////	ALL WHEEL STEERING  - STEERING FUNCTION ///////////////////////////////////////////////////////////////////////////////	*/
			/*				Inner_Angle =	( Pot_Angle / 2 ) - 45;
								
							if ( Inner_Angle <= -1 )  
							{
							
							LF_Error = (-Inner_Angle - (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Outer_Angle = FWD_Differintial_Angle((-Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							RF_Error = (-Outer_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;									
							Outer_Angle_2	= FWD_Differintial_Angle((-Inner_Angle ))	;					
							Prev_Inner_Angle = Inner_Angle;
							}
							
							
							else if ( Inner_Angle >= 0 ) 
							{
							RF_Error = (Inner_Angle - (-RF_Steering)) ;RF_Speed = RF_Error * STEERING_KP;	
							Outer_Angle = FWD_Differintial_Angle((Inner_Angle ))	;	Flash_Factor = Inner_Angle / Outer_Angle; Outer_Steering_Speed = Inner_Steering_Speed / Flash_Factor;
							Outer_Angle_2	= FWD_Differintial_Angle((Inner_Angle ))	;		
							LF_Error = (-Outer_Angle- (LF_Steering)) ;LF_Speed = LF_Error * STEERING_KP;
							Prev_Inner_Angle = Inner_Angle;
							
							}	
							
							else 
							{								
								Steering_Reset_Flag = SET;
								LF_Error = LR_Error = RF_Error = RR_Error = 1;
							}
							
							if ( (LR_Steering <= STEERING_BOUNDARY ) && ( LR_Steering >= -STEERING_BOUNDARY ) ) { LR_Speed = 0;	LR_SET = SET;} 
							else {LR_Speed = ( LR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( LR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	
							
							if ( (RR_Steering <= STEERING_BOUNDARY ) && ( RR_Steering >= -STEERING_BOUNDARY ) ) { RR_Speed = 0;	RR_SET = SET;} 
							else {RR_Speed = ( RR_Steering > STEERING_BOUNDARY ) ? -STEERING_HOMING_SPEED : ( RR_Steering < STEERING_BOUNDARY ) ? STEERING_HOMING_SPEED : 0;}	

							break;*/
							
				case WIDTH_EXTEND: 
					
							LS_Angle = -WIDE_ANGLE; 
							RS_Angle =-WIDE_ANGLE; 
							LF_Speed = (LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;		
							LR_Speed = (LR_Steering > LS_Angle -STEERING_BOUNDARY && LR_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > LS_Angle )? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY )&& (RF_Steering > RS_Angle -STEERING_BOUNDARY && RF_Steering < RS_Angle +STEERING_BOUNDARY ))? SET: NULL;
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;
											
							break;
				
				case WIDTH_SHRINK: 
								
							LS_Angle = SHRINK_ANGLE; 
							RS_Angle = SHRINK_ANGLE; 
							LF_Speed = (LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LF_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LF_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							LR_Speed = (LR_Steering > LS_Angle -STEERING_BOUNDARY && LR_Steering < LS_Angle +STEERING_BOUNDARY ) ? 0 : ( LR_Steering < LS_Angle ) ? STEERING_HOMING_SPEED: ( LR_Steering > LS_Angle ) ? -STEERING_HOMING_SPEED : 0;
							RF_Speed= RR_Speed=0;			
							Angle_Ready = ((LF_Steering > LS_Angle -STEERING_BOUNDARY && LF_Steering < LS_Angle +STEERING_BOUNDARY ))? SET: NULL;			
							break;
										
			default :  break;
		}
	}
					
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
	LF_Speed = LF_Speed > 0 && LF_Steering >= 90 ?  0 : LF_Speed < 0 && LF_Steering <= -90 ? 0 : LF_Speed;
	LR_Speed = LR_Speed > 0 && LR_Steering >= 90 ?  0 : LR_Speed < 0 && LR_Steering <= -90 ? 0 : LR_Speed;
	RF_Speed = RF_Speed < 0 && RF_Steering >= 90 ?  0 : RF_Speed > 0 && RF_Steering <= -90 ? 0 : RF_Speed;
	RR_Speed = RR_Speed > 0 && RR_Steering >= 90 ?  0 : RR_Speed < 0 && RR_Steering <= -90 ? 0 : RR_Speed;
	
			if ( LF_Speed_Temp != LF_Speed )
			{
				LF_Speed= LF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LF_Speed; 
				for (uint8_t i = 0; i < 5; i++)
				{
					Input_Velocity[8] = -LF_Speed;
					Set_Motor_Velocity( LFS , -LF_Speed );
				}
				LF_Speed_Temp = LF_Speed ;
			}
			
			if ( LR_Speed_Temp != LR_Speed )
			{
				LR_Speed= LR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : LR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : LR_Speed;
				for(uint8_t i = 0; i < 2; i++)
				{
					Input_Velocity[9] = -LR_Speed;
				 Set_Motor_Velocity( LRS , -LR_Speed );
					
				}
				LR_Speed_Temp = LR_Speed ;
			}
						
			if ( RF_Speed_Temp != RF_Speed )
			{	
				RF_Speed= RF_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RF_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RF_Speed;
				for (uint8_t i =0; i< 5; i++)
				{
					Input_Velocity[10] = -RF_Speed;
					Set_Motor_Velocity( RFS , -RF_Speed );
				}
				RF_Speed_Temp = RF_Speed ;
			}
										
			if ( RR_Speed_Temp != RR_Speed )
			{
				RR_Speed= RR_Speed > STEERING_MAX_VEL ? STEERING_MAX_VEL : RR_Speed < -STEERING_MAX_VEL ? -STEERING_MAX_VEL : RR_Speed;
				for (uint8_t i = 0; i < 5; i++)
				{
					Input_Velocity[11] = -RR_Speed;
					Set_Motor_Velocity( RRS , -RR_Speed );
				}
				RR_Speed_Temp = RR_Speed ;
			}	
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
}

void Steering_Pos_Controls (void)
{
/*	If the Steering Reset Flag is SET, all the Steering wheels will return to their Home Position.	
		Flag Sets on Power Up and at every Steering Mode Change.																*/	
	
	switch (Steering_Mode)
	{
		case 1:
		
		//Inner_Angle =	(( Pot_Angle / 2 ) - 45);
		Inner_Angle = (Pot_Angle - 90) / 3;
		
		if ( Inner_Angle < -1 ) 
		{
			Outer_Angle = Differintial_Angle((-Inner_Angle ))	;
			LF_Pos = Inner_Angle * (5.0 / 3.0);
			LR_Pos = Inner_Angle * (5.0 / 3.0);
			RF_Pos = Outer_Angle * (5.0 / 3.0);
			RR_Pos = Outer_Angle * (5.0 / 3.0);
		}
		
		else if (Inner_Angle > 1)
		{
			Outer_Angle = Differintial_Angle((Inner_Angle ));
			LF_Pos = Outer_Angle * (5.0 / 3.0);
			LR_Pos = Outer_Angle * (5.0 / 3.0);
			RF_Pos = -(Inner_Angle * (5.0 / 3.0));
			RR_Pos = -(Inner_Angle * (5.0 / 3.0));
		}
		
		else
		{
			LF_Pos = LR_Pos = RF_Pos = RR_Pos = 0;
		}
			
		Wheel_Speeds_Calc(Inner_Angle);
		
		break;
		
		case 2:
			
		AW_Angle = Zero_Turn_Angle ; 
		LF_Pos = AW_Angle * (5.0 / 3.0);
		LR_Pos = AW_Angle * (5.0 / 3.0);
		RF_Pos = AW_Angle * (5.0 / 3.0);
		RR_Pos = AW_Angle * (5.0 / 3.0);
		
		break;
		
		case 3:
		
		AW_Angle = ( Pot_Angle - 90);
		LF_Pos = AW_Angle * (5.0 / 3.0);
		LR_Pos = -(AW_Angle * (5.0 / 3.0));
		RF_Pos = -(AW_Angle * (5.0 / 3.0));
		RR_Pos = AW_Angle * (5.0 / 3.0);
		
		break;
		
		case 4:
			
		LS_Angle = -SHRINK_ANGLE;
		LF_Pos = -(LS_Angle * (5.0 / 3.0));
		LR_Pos = LS_Angle * (5.0 / 3.0);
		RF_Pos = RR_Pos = 0;
		break;
		
		case 5:
			
		LS_Angle = WIDE_ANGLE; 
		LF_Pos = -(LS_Angle * (5.0 / 3.0));
		LR_Pos = LS_Angle * (5.0 / 3.0);
		RF_Pos = RR_Pos = 0;
		break;
		
		default:   break;
	
	}
	
			if ( LF_Pos_Temp != LF_Pos )
			{
				 
				for (uint8_t i = 0; i < 5; i++)
				{
					Set_Motor_Position( LFS , LF_Pos );
				}
				LF_Pos_Temp = LF_Pos ;
			}
			
			if ( LR_Pos_Temp != LR_Pos )
			{
				
				for(uint8_t i = 0; i < 5; i++)
				{
					Set_Motor_Position( LRS , LR_Pos );
					
				}
				LR_Pos_Temp = LR_Pos ;
			}
						
			if ( RF_Pos_Temp != RF_Pos )
			{	
				
				for (uint8_t i =0; i< 5; i++)
				{
					Set_Motor_Position( RFS , RF_Pos );
				}
				RF_Pos_Temp = RF_Pos ;
			}
										
			if ( RR_Pos_Temp != RR_Pos )
			{
				
				for (uint8_t i = 0; i < 5; i++)
				{
					Set_Motor_Position( RRS , RR_Pos );
				}
				RR_Pos_Temp = RR_Pos ;
			}

					
			/*----------------------------- STEERING VELOCITY CONTROLLER --------------------------------*/
}

void Initial_Msg(void)
{
		//BT_State1 = BT_READ_1;
	  if (BT_State1 == 1 && message_sent == 0) {
        HAL_UART_Transmit_IT(&huart4, (uint8_t*)Tx_Initial_msg, sizeof(Tx_Initial_msg));
        message_sent = 1;
    }
		else if (BT_State1 == 0) {
    message_sent = 0;
		}
		else{}

		if(message_sent==1)
		{
			if((HAL_GetTick() - Uart_Time) >= 250){
				Start_Continuous_Sending(currentCommand);	
				Uart_Time = HAL_GetTick();
			}
		}
		
//		if((HAL_GetTick()-Timt_Batt)>10000){
//	Tx_Voltage=Rover_Voltage < 44 ? 1 : 0;
//	HAL_UART_Transmit(&huart5,&Tx_Voltage,sizeof(Tx_Voltage), 1000);
//	Timt_Batt=HAL_GetTick();
//}
}

void Start_Continuous_Sending(char command)
{
    currentCommand = command;
    UART_tx();  
}

void UART_tx(void) {
    static int len = 0; 
        switch (currentCommand) {
            case '1': { 
                if (Rover_Voltage >= 40 && Rover_Voltage <= 55) {
                    Main_Bt_Percentage = (6.67f * Rover_Voltage) - 273.33f;
                } else {
                    Main_Bt_Percentage = 0;
                }
                len = snprintf(Main_Battery, sizeof(Main_Battery), "Voltage: %f\nPercentage: %f\n", Rover_Voltage, Main_Bt_Percentage);
                HAL_UART_Transmit_IT(&huart4, (uint8_t*)Main_Battery, len);
                break;
            }

		case '2':{
					len = snprintf(outputBuffer, sizeof(outputBuffer), "\nOPERATION_MONITOR_FLAG: %d \n", OPERATION_MONITOR_FLAG);
					
					if(Drive_Errored){
	for (int8_t i = 1; i < 21; i++)
	{
		if (i != 5 && i!=17)
		{
		if (Axis_State[i] != 8) { 
			len += snprintf(outputBuffer + len, sizeof(outputBuffer) - len, "Drive_Errored(%s): %d\n",Node_ID_To_Name[i],i);
		}
}}}
					
		if (Drive_Disconnected) {
    for (int8_t i = 1; i < 21; i++) {
        if (i != 5 && i!=17 && Node_Id[i] == Node_Id_Temp[i]) {
            len += snprintf(outputBuffer + len, sizeof(outputBuffer) - len, "Drive_Disconnected (%s): %d\n",Node_ID_To_Name[i],i);
        }
    }
}

		if (Sensor_Disconnected) {
    for (int8_t i = 21; i <= 27; i++) {
        if (Node_Id[i] == Node_Id_Temp[i]) {
            len += snprintf(outputBuffer + len, sizeof(outputBuffer) - len, "Sensor_Disconnected: %d\n",i);
        }}
		}
		
		if (Joystick_Disconnected) {
    len += snprintf(outputBuffer + len, sizeof(outputBuffer) - len, 
                    "Joystick_Disconnected ");
		
	}
		
	if(FET_Temp_Exceeded){
	for (int8_t i = 1; i < 17; i++)
		{
			if (i != 5)
			{		
				if(FET_Temperature[i] >= 90) {
				len+=snprintf(outputBuffer+len,sizeof(outputBuffer)-len,"FET_Temp_Exceeded (%s): %d\n ",Node_ID_To_Name[i],i);
				}}}	  
		}
					HAL_UART_Transmit_IT(&huart4, (uint8_t*)outputBuffer, len);
					break;
	}

            case '3': {
                len = snprintf(Left_IMU_Data, sizeof(Left_IMU_Data), "%f\n", L_Pitch);
                HAL_UART_Transmit_IT(&huart4, (uint8_t*)Left_IMU_Data, len);
                break;
            }
						
						case '4': {
                len = snprintf(Left_IMU_Data, sizeof(Left_IMU_Data), "%f\n", L_Roll);
                HAL_UART_Transmit_IT(&huart4, (uint8_t*)Left_IMU_Data, len);
                break;
								 }
						case '5':{
							//len = snprintf(Right_IMU_Data, sizeof(Right_IMU_Data), "%f\n", R_Pitch_Filtered);
							len = snprintf(Right_IMU_Data, sizeof(Right_IMU_Data), "%f\n", BLE_Pitch);
                HAL_UART_Transmit_IT(&huart4, (uint8_t*)Right_IMU_Data, len);
                break;
						 }
						case '6':{
							//len = snprintf(Right_IMU_Data, sizeof(Right_IMU_Data), "%f\n", R_Roll_Filtered);
							len = snprintf(Right_IMU_Data, sizeof(Right_IMU_Data), "%f\n", BLE_Roll);
                HAL_UART_Transmit_IT(&huart4, (uint8_t*)Right_IMU_Data, len);
                break;
						 }
            case '7': {
                len = snprintf(Pitch_IMU_Data, sizeof(Pitch_IMU_Data), "%f\n", Shear_Pitch);
                HAL_UART_Transmit_IT(&huart4, (uint8_t*)Pitch_IMU_Data, len);
                break;
            } 
//						
						case '8' :{
						for (uint8_t i = 1; i < 17; i++)
		     {
			if (i != 5)
			{
				len += snprintf(outputBuffer + len, sizeof(outputBuffer) - len,  "(%s): %0.1f\n", Node_ID_To_Name[i], FET_Temperature[i]);
//				len+=snprintf(outputBuffer+len,sizeof(outputBuffer)-len," FET_Temperature[%d]: %0.1f\n ",i,FET_Temperature[i]);
				
			}}	  
				 HAL_UART_Transmit_IT(&huart4, (uint8_t*)outputBuffer, len);
					break;		
						}

            default: {
                break; 
            }
        }
    
}

 void All_Macro_Sensing(void){
	
		Array_Element = ARRAY_SIZE - (65 / pow(Speed, 1.025));
	
	 Flap_Angle_Left = Flap_Data_Array[Array_Element];
	Flap_Angle_Right = Flap_Data_Right[Array_Element];
	
//	LF_Bush_Sensed=(FL_Angle>20)?1:0;
//	LR_Bush_Sensed=(RL_Angle>20)?1:0;
//	RF_Bush_Sensed=(FR_Angle>20)?1:0;
//	RR_Bush_Sensed=(RR_Angle>20)?1:0;
	
//	LF_Bush_Sensed=(FL_LPF_Angle>=10)?1:0;
//	LR_Bush_Sensed=(RL_LPF_Angle>=10)?1:0;
//	RF_Bush_Sensed=(FR_LPF_Angle>=10)?1:0;
//	RR_Bush_Sensed=(RR_LPF_Angle>=10)?1:0;
	 
	 if (FL_LPF_Angle >= 10) {
        if (Last_Tick_FL == 0) { 
            Last_Tick_FL = HAL_GetTick();
        } else if ((HAL_GetTick() - Last_Tick_FL) >= 1000) {
            LF_Bush_Sensed = 1;
        }
    } else {
        Last_Tick_FL = 0;
        LF_Bush_Sensed = 0;
    }
    if (RL_LPF_Angle >= 10) {
        if (Last_Tick_RL == 0) { 
            Last_Tick_RL = HAL_GetTick();
        } else if ((HAL_GetTick() - Last_Tick_RL) >= 1000) {
            LR_Bush_Sensed = 1;
        }
    } else {
        Last_Tick_RL = 0;
        LR_Bush_Sensed = 0;
    }
    if (FR_LPF_Angle >= 10) {
        if (Last_Tick_FR == 0) { 
            Last_Tick_FR = HAL_GetTick();
        } else if ((HAL_GetTick() - Last_Tick_FR) >= 1000) {
            RF_Bush_Sensed = 1;
        }
    } else {
        Last_Tick_FR = 0;
        RF_Bush_Sensed = 0;
    }
    if (RR_LPF_Angle >= 10) {
        if (Last_Tick_RR == 0) { 
            Last_Tick_RR = HAL_GetTick();
        } else if ((HAL_GetTick() - Last_Tick_RR) >= 1000) {
            RR_Bush_Sensed = 1;
        }
    } else {
        Last_Tick_RR = 0;
        RR_Bush_Sensed = 0;
    }

	
//	Front_Angle=(FL_Angle+FR_Angle)/2;
//	Rear_Angle=(RL_Angle + RR_Angle)/2;
	
	Front_Angle=(FL_LPF_Angle + FR_LPF_Angle)/2;
	Rear_Angle=(RL_LPF_Angle + RR_LPF_Angle)/2;
	
	if(Mode == 2)
{
 if (Joystick_Temp != Joystick)
        {
            switch (Joystick)
            {
                case 0: Macro_Speed = 0;   break;
                case 1: Macro_Speed = 20;  break;
                case 2: Macro_Speed = -20; break;
                default: Macro_Speed = 0;  break;
            }
            Joystick_Temp = Joystick;
        }
				
}
	else if(Mode==3){
		
//		if (HAL_GetTick() - Flaps_Tick <= 1500)
//	{
//		for (uint8_t i = 28; i < 32; i++)
//		{
//			if (Node_Id[i] == Node_Id_Temp[i])
//			{
//				Flaps_Disconnected = SET;
//				Node_Id_Temp[i] = Node_Id[i];
//			}
//			
//			else
//			{
//				Flaps_Disconnected = NULL;
//			}
//		}
//		Flaps_Tick = HAL_GetTick();
//	}
//	
//	if( !LR_Bush_Sensed && LF_Bush_Sensed && !RF_Bush_Sensed && !RR_Bush_Sensed)
//		{	
////		Macro_Speed=Top_Sensing_PID(FL_Angle,NULL);
////		Macro_Speed=(FL_Angle>=35 && FL_Angle<=45)?0:(FL_Angle<35)?-Macro_Speed:(FL_Angle>45)?Macro_Speed:0;
//		
//			Flap_Error_Left = Flaps_Target_Left - FL_LPF_Angle ;
//			Macro_Speed=-Top_Sensing_PID(Flap_Error_Left,NULL);
//			
//	}
//	else if(!LR_Bush_Sensed && !LF_Bush_Sensed && RF_Bush_Sensed && !RR_Bush_Sensed)
//		{
//			
////		Macro_Speed=Top_Sensing_PID(FR_Angle,NULL);
////		Macro_Speed=(FR_Angle>=35 && FR_Angle<=45)?0:(FR_Angle<35)?-Macro_Speed:(FR_Angle>45)?Macro_Speed:0;
//	
//	    Flap_Error_Right = Flaps_Target_Right - FR_LPF_Angle ;
//			Macro_Speed=-Top_Sensing_PID(Flap_Error_Right,NULL);

//	}
//	else if(LR_Bush_Sensed && !RR_Bush_Sensed ){
////		  Macro_Speed=Top_Sensing_PID(RL_Angle,NULL);
////			Macro_Speed=(RL_Angle>=35 && RL_Angle<=45)?0:(RL_Angle<35)?-Macro_Speed:(RL_Angle>45)?Macro_Speed:0;	
//		
//		Flap_Error_LR=Flap_Target_LR-RL_LPF_Angle;
//		Macro_Speed=-Top_Sensing_PID(Flap_Error_LR,NULL);

//	}	
//	else if(!LR_Bush_Sensed && RR_Bush_Sensed){
////		Macro_Speed=Top_Sensing_PID(RR_Angle,NULL);
////		Macro_Speed=(RR_Angle>=35 && RR_Angle<=45)?0:(RR_Angle<35)?-Macro_Speed:(RR_Angle>45)?Macro_Speed:0;

//	  	Flap_Error_RR=Flap_Target_RR-RR_LPF_Angle;
//			Macro_Speed=-Top_Sensing_PID(Flap_Error_RR,NULL);

//	}
//	else if(!LR_Bush_Sensed && LF_Bush_Sensed && RF_Bush_Sensed && !RR_Bush_Sensed){
//		
//		Flap_Error_FAVG=Flap_Target_FAVG-Front_Angle;
//		Macro_Speed=-Top_Sensing_PID(Flap_Error_FAVG,NULL);
//		
//	}
//	else if( LR_Bush_Sensed && RR_Bush_Sensed){
//		
//		Flap_Error_RAVG=Flap_Target_RAVG-Rear_Angle;
//			Macro_Speed=-Top_Sensing_PID(Flap_Error_RAVG,NULL);
//		
//	}
//		else {
//			 Macro_Speed=0;
//		}
	}
	else if(Mode==1) 	{ Macro_Speed=0;	}
else{Macro_Speed=0;}
	
Left_Macro_Speed = Right_Macro_Speed = Macro_Speed;
				
//				Left_Macro_Speed = Left_Macro_Speed > 0 && Left_Macro_Motor_Count >= -30 ? 5 : Left_Macro_Speed < 0 && Left_Macro_Motor_Count <= -170 ? -5 : Left_Macro_Speed;
//				Right_Macro_Speed = Right_Macro_Speed > 0 && Right_Macro_Motor_Count  >= -30 ? 5 : Right_Macro_Speed < 0 && Right_Macro_Motor_Count <= 170 ? -5 : Right_Macro_Speed;
//				
				//Left_Macro_Speed = Left_Macro_Speed > 0 && Left_Macro_Motor_Count >= 0 ? 0 : Left_Macro_Speed < 0 && Left_Macro_Motor_Count <= -300 ? 0 : Left_Macro_Speed;
				//Right_Macro_Speed = Right_Macro_Speed > 0 && Right_Macro_Motor_Count  >= 0 ? 0 : Right_Macro_Speed < 0 && Right_Macro_Motor_Count <= -300 ? 0 : Right_Macro_Speed;
	
//	  if (fabs(fabs(Right_Macro_Motor_Count) - fabs(Left_Macro_Motor_Count)) > max_difference)
//		{
//				if (fabs(fabs(Right_Macro_Motor_Count) - fabs(Left_Macro_Motor_Count)) > 10)
//				{
//					if(Mode==2)
//					{
//					 Left_Macro_Speed = Joystick != 0 ? 0 : Left_Macro_Speed;
//					 Right_Macro_Speed = Joystick != 0 ? 0 : Right_Macro_Speed;
//					}
//					
////							else if(Mode==3){
////                Left_Macro_Speed = Macro_Speed != 0 ? 0 : Left_Macro_Speed;
////                Right_Macro_Speed = Macro_Speed != 0 ? 0 : Right_Macro_Speed;}
//					
//						else {}
//							
//				}
//				Macro_Error = Right_Macro_Motor_Count - Left_Macro_Motor_Count; 
//				//Macro_Error = -Macro_Error;
//				Correction_Speed = Macro_Error < 2 && Macro_Error > -2 ? 0 : (Macro_Error * Macro_Kp);
////           Correction_Speed = Correction_Speed < 2 && Correction_Speed > -2 ? 0 : Correction_Speed;
//				Correction_Speed = Correction_Speed > 10 ? 10 : Correction_Speed < -10 ? -10 : Correction_Speed;
//		}
//		else 
//		{
				Correction_Speed = 0;
//		}
        Left_Macro_Speed = Left_Macro_Speed + Correction_Speed;
//	
	if (Left_Macro_Speed_Temp != Left_Macro_Speed)
	{
			Input_Velocity[12] = Left_Macro_Speed;
			Set_Motor_Velocity(12, Left_Macro_Speed);
			Left_Macro_Speed_Temp = Left_Macro_Speed;
	}
	if (Right_Macro_Speed_Temp != Right_Macro_Speed)
	{
			Input_Velocity[13] = Right_Macro_Speed;
			Set_Motor_Velocity(13, Right_Macro_Speed);
			Right_Macro_Speed_Temp = Right_Macro_Speed;
	}

}
 
float Top_Sensing_PID ( float Flap_Value , unsigned long long 	R_Time_Stamp )
{
		//dt = Time_Stamp - time;

			M_Error_Change = Flap_Value - M_Prev_Error;
			M_Error_Slope  = M_Error_Change / dt;
			M_Error_Area   = M_Error_Area + ( M_Error_Change * dt ) ;			
				
			M_P = M_Kp * Flap_Value;
			 
			M_I	= M_Ki * M_Error_Area;						 M_I = M_I > Anti_Windup_Limit ? Anti_Windup_Limit : M_I < -Anti_Windup_Limit ? -Anti_Windup_Limit : M_I ;	

			M_D = M_Kd * M_Error_Slope; 
				
			
			Macro_Out = M_P + M_I + M_D ;

			Macro_Out = Macro_Out > Macro_Max_Speed ? Macro_Max_Speed : Macro_Out < -Macro_Max_Speed ? -Macro_Max_Speed : Macro_Out;
	    Macro_Out = (Macro_Out >-5 && Macro_Out<5)?0:Macro_Out;
	
			M_Prev_Error = Flap_Value;
			
			return Macro_Out;

}

void Pitch_Control(void)
{
	

//	if (HAL_GetTick() - Imu_Tick >= 1000)
//	{
//		if(Node_Id[27] == Node_Id_Temp[27])
//		{
//			IMU_Disconnected = SET;
//		}
//		
//		else
//		{
//			IMU_Disconnected = NULL;
//		}
//		
//		Imu_Tick = HAL_GetTick();
//	}
	
	Lead_Screw_Length = Vertical_Motor_Count * 0.25f;        //to be included in EEPROM function
	Vertical_Angle = Lead_Screw_Length * 0.222f;            // to be included in EEPROM function
	
	Pitch_Target_Angle = roundf(Vertical_Angle * 10) / 10;
	Shear_Angle = Shear_Pitch_Home_Pos - Shear_Pitch;
	
	Pitch_Arm_Error = Pitch_Target_Angle - Shear_Angle;
	Pitch_Arm_Error = Pitch_Arm_Error <= 1 && Pitch_Arm_Error >= -1	? 0 : Pitch_Arm_Error;
	Pitch_Arm_Speed = -Pitch_Arm_PID ( Pitch_Arm_Error , NULL);
	
	Pitch_Arm_Speed = Pitch_Arm_Speed >= 10 ? 10 : Pitch_Arm_Speed <= -10 ? -10 : Pitch_Arm_Speed;
	
	if (Pitch_Arm_Speed != Pitch_Arm_Speed_Temp)
	{
		Input_Velocity[14] = Pitch_Arm_Speed;
		Set_Motor_Velocity(14, Pitch_Arm_Speed);
		Pitch_Arm_Speed_Temp = Pitch_Arm_Speed;
	}
	
}

float convertRawDataToFloat(uint8_t* data) {
    int32_t raw = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    return *((float*)&raw);
}

float Float16_To_Decimal(uint16_t float16)
{
	 uint16_t sign = (float16 >> 15) & 0x1;
   uint16_t exponent = (float16 >> 10) & 0x1F;
   uint16_t mantissa = float16 & 0x3FF;
	
//	int exp = (int)exponent - 15;
	float frac = (float)mantissa / 1024.0f;
	double decimal_value = pow(-1, sign) * (1 + mantissa/ 1024.0) * pow(2, exponent - 15);
	
	return decimal_value;
}

void Macro()
{
	if (Mode == 2)
	{
		if (Joystick != Joystick_Temp)
		{
			switch (Joystick)
			{
				case 0: Macro_Speed = 0; break;
				case 1: Macro_Speed = 30; break;
				case 2: Macro_Speed = -30; break;
				default: break;
			}
			
			Joystick_Temp = Joystick;
		}
	}
	
	else
	{
		Macro_Speed = 0;	
	}
	
	Left_Macro_Speed = Right_Macro_Speed = Macro_Speed;
	
	if (Left_Macro_Speed != Left_Macro_Speed_Temp)
	{
		for(uint8_t i=0; i<3 ; i++) Set_Motor_Velocity(12, Left_Macro_Speed);
		
		Left_Macro_Speed_Temp = Left_Macro_Speed;
	}
	
	if (Right_Macro_Speed != Right_Macro_Speed_Temp)
	{
		for(uint8_t i=0; i<3 ; i++) Set_Motor_Velocity(13, Right_Macro_Speed);
		
		Right_Macro_Speed_Temp = Right_Macro_Speed;
	}
}

void Frame_Manual_Controls(void)
{
    static int zero_command_count = 0;
    int vert_speed = 0;
    int cont_speed = 0;

    if (Mode == 3)
    {
        zero_command_count = 0;  // Reset latch on manual mode

        switch (Joystick)
        {
            case 1:
                vert_speed = 10;
                break;
            case 2:
                vert_speed = -10;
                break;
            case 3:
                cont_speed = 10;
                break;
            case 4:
                cont_speed = -10;
                break;
            case 0:
            default:
                vert_speed = 0;
                cont_speed = 0;
                break;
        }

        R_Vert_Speed = vert_speed;
        Contour_Speed = cont_speed;

        // Set speeds only once here, using a consolidated command
//        Set_Motor_Speed("VERT_MOTOR", Vert_Speed);
//        Set_Motor_Speed("CONT_MOTOR", Cont_Speed);
				
//				Set_Motor_Velocity (RVert , R_Vert_Speed );
				Set_Motor_Velocity (14 , R_Vert_Speed );
//				Set_Motor_Velocity (Contour , Contour_Speed );
    }
    else
    {
        // Latch zero speed commands up to 3 times only
        if (zero_command_count < 3)
        {
            Set_Motor_Velocity (RVert , 0 );
            Set_Motor_Velocity (Contour , 0 );
            zero_command_count++;
        }
        R_Vert_Speed = 0;
        Contour_Speed = 0;
    }
}

void Flap_Sensing(void)
{
	
	    /* -- 1. Sensor validity flag with 2-second debounce -- */
    if (FL_Angle > 15)
    {
        if (!Front_Left_Bush_Timer_Active)
        {
            Front_Left_Bush_Timer    = HAL_GetTick();
            Front_Left_Bush_Timer_Active = 1;
        }

        if ((HAL_GetTick() - Front_Left_Bush_Timer) >= 2000)
        {
            Front_Left_Bush = 1;
        }
    }
    else
    {
        /* Angle dropped below 15 – reset timer and flag */
        Front_Left_Bush          = 0;
        Front_Left_Bush_Timer_Active = 0;
        Front_Left_Bush_Timer    = 0;
    }

    /* -- 2. Only run in shearing mode 3, macro mode 2 -- */
    if ((Shearing == 3) && (Mode == 2))
    {
        if (Front_Left_Bush)
        {
            /* -- Angle > 15 for 2s : Active sensing – PID-style control -- */
            Flap_Error   = Flaps_Target - FL_Angle;
            MMacro_Speed = (Flap_Error <= 2 && Flap_Error >= -2)
                           ? 0
                           : (int16_t)(Flap_Error * Flap_Kp);
        }
        else
        {
            /* -- Not confirmed yet or angle <= 15 : stop motor -- */
            MMacro_Speed = 0;
        }

        /* -- 3. Clamp output -- */
        MMacro_Speed = (MMacro_Speed >  30) ?  30 :
                       (MMacro_Speed < -30) ? -30 : MMacro_Speed;
    }
    else
    {
        MMacro_Speed = 0;
    }

    /* -- 4. Mirror speed to both sides -- */
    MLeft_Macro_Speed  = MMacro_Speed;
    MRight_Macro_Speed = MMacro_Speed;

    /* -- 5. Apply only on change (left) -- */
    if (MLeft_Macro_Speed != MLeft_Macro_Speed_Temp)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            Set_Motor_Velocity(12, MLeft_Macro_Speed);
        }
        MLeft_Macro_Speed_Temp = MLeft_Macro_Speed;
    }

    /* -- 6. Apply only on change (right) -- */
    if (MRight_Macro_Speed != MRight_Macro_Speed_Temp)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            Set_Motor_Velocity(13, MRight_Macro_Speed);
        }
        MRight_Macro_Speed_Temp = MRight_Macro_Speed;
    }
	
	
	
//////////    flap_pos = -(FL_Angle + 32.5f);

//////////    /* If not in shearing mode, stop motor and exit */
//////////    if ((Shearing == 3) && (Mode != 2))
//////////    {


//////////    /* Convert angle:
//////////       -32.5 -> 0
//////////       More negative -> positive value */
//////////    

//////////    /* Motor control logic */
//////////    if (flap_pos >= 1.0f && flap_pos <= 15.0f)
//////////    {
//////////        MMacro_Speed = 10;
//////////    }
//////////    else if (flap_pos > 35.0f)
//////////    {
//////////        MMacro_Speed = -10;
//////////    }
//////////    else
//////////    {
//////////        MMacro_Speed = 0;
//////////    }
//////////	}
//////////		
//////////	else
//////////	{
//////////		MMacro_Speed = 0;
//////////	}
//////////		
//////////		
//////////		MLeft_Macro_Speed = MRight_Macro_Speed = MMacro_Speed;
//////////	
//////////	if (MLeft_Macro_Speed != MLeft_Macro_Speed_Temp)
//////////	{
//////////		for(uint8_t i=0; i<3 ; i++) Set_Motor_Velocity(12, MLeft_Macro_Speed);
//////////		
//////////		MLeft_Macro_Speed_Temp = MLeft_Macro_Speed;
//////////	}
//////////	
//////////	if (MRight_Macro_Speed != MRight_Macro_Speed_Temp)
//////////	{
//////////		for(uint8_t i=0; i<3 ; i++) Set_Motor_Velocity(13, MRight_Macro_Speed);
//////////		
//////////		MRight_Macro_Speed_Temp = MRight_Macro_Speed;
//////////	}

    /* Function exits after one execution */
}
void Pitch_Arm_Control_IMU(void)
{
	
	Base_Pitch_Filtered = Base_IMU_LPF(Base_Pitch, Prev_Base_Pitch_Filtered, ALPHA);
	Prev_Base_Pitch_Filtered = Base_Pitch_Filtered;
	
	Shear_Pitch_Filtered = Shearing_IMU_LPF(Shear_Pitch, Prev_Shear_Pitch_Filtered, ALPHA);
	Prev_Shear_Pitch_Filtered = Shear_Pitch_Filtered;
	
	Base_Pitch_Angle = Base_Pitch_HomePos - Base_Pitch_Filtered;
	Shear_Pitch_Angle = Shear_Pitch_HomePos - Shear_Pitch_Filtered;
	
	
//	Shear_Pitch_Error = -(Base_Pitch_Angle - Shear_Pitch_Angle);
	
	//Shear_Pitch_Error =(Base_Pitch_Angle - Shear_Pitch_Angle);
	
	 Shear_Pitch_Error = (Shear_Pitch_Angle + Base_Pitch_Angle);
	
	
	Shear_Pitch_Speed = fabs(Shear_Pitch_Error) < 0.2 ? 0 : Shear_Pitch_Error * Shear_Pitch_Kp;

//    if (Shear_Pitch_Speed > 50) Shear_Pitch_Speed = 50;
//    if (Shear_Pitch_Speed < -50) Shear_Pitch_Speed = -50;
	
	Shear_Pitch_Speed = Shear_Pitch_Speed > 25 ? 25 : Shear_Pitch_Speed < -25 ? -25 : ( (Shear_Pitch_Speed < 2) && (Shear_Pitch_Speed > -2) ) ? 0 : Shear_Pitch_Speed ;
		
	if (Shear_Pitch_Speed != Shear_Pitch_Speed_Temp)
	{
		for(uint8_t i=0; i<3 ; i++) Set_Motor_Velocity(14, Shear_Pitch_Speed);
		Shear_Pitch_Speed_Temp = Shear_Pitch_Speed;
	}
}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
