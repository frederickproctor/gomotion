#ifndef ROBOCRANE_H
#define ROBOCRANE_H

#ifdef __cplusplus
extern "C" {
#endif
#if 0
}
#endif

#define ENCODER_ERROR_SUM_CYCLES 10
#define LENGTH_ERROR_SUM_CYCLES 10

#define ENCODER_VELOCITY_ERROR_SUM_CYCLES 10
#define LENGTH_VELOCITY_ERROR_SUM_CYCLES 10

/* Servo command modes */
#define VOLTAGE_MODE 0 /* In volts */
#define ENCODER_POSITION_MODE 1  /* In encoder counts */
#define CABLE_POSITION_MODE 2  /* In mm */
#define ENCODER_VELOCITY_MODE 3  /* In encoder counts per control cycle */
#define CABLE_VELOCITY_MODE 4  /* In mm per control cycle */


/* RoboCrane's Joint Command structure [for each joint] */
/* Command structure appears in: */
/* RoboCrane's status structure - copy of received commands */
/* RoboCrane's command structure - sent commands */
typedef struct {
  int P_gain;
  int I_gain;
  int D_gain;
  int EncoderCountsPerMM;
  float CmdCableLength;
  float CmdCableVel;
  float CmdCableTens;
  float CmdMotorVolt;
  int CmdMotorPos;  /* In encoder counts */
  int CmdMotorVel;
  int CmdMotorAcc;
  char AmpEnable;
} Joint_Cmd_Type;

/* RoboCrane's Status Command structure [for each joint] */
typedef struct {
  int EncoderCnt;
  int EncoderCntLast;
  int EncoderErr[ENCODER_ERROR_SUM_CYCLES];
  int EncoderVelCurr;
  int EncoderVelLast;
  int EncoderVelErr[ENCODER_VELOCITY_ERROR_SUM_CYCLES];
  int EncoderAccCurr;
  int EncoderAccLast;
  int EncoderErrorSum;
  int EncoderVelErrorSum;

  int EncoderCntOffset; /* Value by which current encoder count is offset to correspond to *** INITIAL *** cable length */
  char EncoderFlowState; /* Under/overflow state of the encoder count's low word */

  float LengthCurr;
  float LengthLast;
  float LengthErr[LENGTH_ERROR_SUM_CYCLES];
  float LengthVelCurr;
  float LengthVelLast;
  float LengthVelErr[LENGTH_VELOCITY_ERROR_SUM_CYCLES];
  float LengthAccCurr;
  float LengthAccLast;
  float TensCurr;
  float TensLast;
  float LengthErrorSum;
  float LengthVelErrorSum;
  float CurrMotorVolt;
} Joint_Status_Type;

/* RoboCrane's auxilliary structure */
typedef struct {
  char light;
  char camera;
  char lasers;
  char platform_latch;
  char gripper;
} Aux_Type;

/* RoboCrane's status structure */
typedef struct {
  Joint_Status_Type jointStatus[6];
  Joint_Cmd_Type jointCmd[6];
  Aux_Type auxStatus;
} RoboCraneStatus_Type;

/* RoboCrane's command structure */
typedef struct {
  Joint_Cmd_Type jointCmd[6];
  Aux_Type auxCmd;
  char ServoCmdMode; /* Could be voltage (i.e., no PID), position (in encoder counts or mm), or velocity */
  int LengthErrorSumCycles;
  int LengthVelErrorSumCycles;
  int EncoderErrorSumCycles;
  int EncoderVelErrorSumCycles;
} RoboCraneCmd_Type;


/* RoboCrane's main structure */
typedef struct {
  float cartesian[6];
  float cylindrical[6];
  RoboCraneStatus_Type status;
  RoboCraneCmd_Type cmd;
} RoboCrane_Type;

#if 0
{
#endif
#ifdef __cplusplus
}
#endif

#endif

