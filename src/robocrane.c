#include "robocrane.h"

void Initialize_Aux_Type(Aux_Type *_Aux)
{
  _Aux->light=0;
  _Aux->camera=0;
  _Aux->lasers=0;
  _Aux->platform_latch=0;
  _Aux->gripper=0;
}

void Initialize_Joint_Cmd_Type(Joint_Cmd_Type *_Joint_Cmd)
{
  _Joint_Cmd->P_gain=16;
  _Joint_Cmd->I_gain=0;
  _Joint_Cmd->D_gain=0;
  _Joint_Cmd->EncoderCountsPerMM=2311.84; /* From ATR code */
  _Joint_Cmd->CmdCableLength=0;
  _Joint_Cmd->CmdCableVel=0;
  _Joint_Cmd->CmdCableTens=0;
  _Joint_Cmd->CmdMotorVolt=0;
  _Joint_Cmd->CmdMotorPos=0;  /* In encoder counts */
  _Joint_Cmd->CmdMotorVel=0;
  _Joint_Cmd->CmdMotorAcc=0;
  _Joint_Cmd->AmpEnable=0;
}

void Initialize_Joint_Status_Type(Joint_Status_Type *_Joint_Status)
{
  int i;

  _Joint_Status->EncoderCnt=0;
  _Joint_Status->EncoderCntLast=0;
  _Joint_Status->EncoderVelCurr=0;
  _Joint_Status->EncoderVelLast=0;
  _Joint_Status->EncoderAccCurr=0;
  _Joint_Status->EncoderAccLast=0;
  _Joint_Status->EncoderErrorSum=0;
  _Joint_Status->EncoderVelErrorSum=0;

  for (i=0; i<ENCODER_ERROR_SUM_CYCLES; i++)
    _Joint_Status->EncoderErr[i]=0;

  for (i=0; i<ENCODER_ERROR_SUM_CYCLES; i++)
    _Joint_Status->EncoderVelErr[i]=0;

  _Joint_Status->EncoderCntOffset=0; /* Value by which current encoder count is offset to correspond to *** INITIAL *** cable length */
  _Joint_Status->EncoderFlowState=0; /* Under/overflow state of the encoder count's low word */

  _Joint_Status->LengthCurr=0;
  _Joint_Status->LengthLast=0;
  for (i=0; i<LENGTH_ERROR_SUM_CYCLES; i++)
    _Joint_Status->LengthErr[i]=0;
  _Joint_Status->LengthVelCurr=0;
  _Joint_Status->LengthVelLast=0;
  for (i=0; i<LENGTH_VELOCITY_ERROR_SUM_CYCLES; i++)
    _Joint_Status->LengthVelErr[i]=0;
  _Joint_Status->LengthAccCurr=0;
  _Joint_Status->LengthAccLast=0;
  _Joint_Status->TensCurr=0;
  _Joint_Status->TensLast=0;
  _Joint_Status->LengthErrorSum=0;
  _Joint_Status->LengthVelErrorSum=0;
  _Joint_Status->CurrMotorVolt=0;
}

void RoboCraneInitialize(RoboCrane_Type *_RC)
{
  int i;

  /* Initialize Robocrane's coordinates */
  for(i=0;i<6;i++)
    {
      _RC->cartesian[i]=0;
      _RC->cylindrical[i]=0;
    }

  /* Initialize RoboCrane's command structure */
  for (i=0; i<6; i++)
    Initialize_Joint_Cmd_Type(&(_RC->cmd.jointCmd[i]));
  _RC->cmd.LengthErrorSumCycles=2;
  _RC->cmd.LengthVelErrorSumCycles=2;
  _RC->cmd.EncoderErrorSumCycles=2;
  _RC->cmd.EncoderVelErrorSumCycles=2;
  _RC->cmd.ServoCmdMode=-1; /* Means nothing - undefined mode */
  Initialize_Aux_Type(&(_RC->cmd.auxCmd));


  /* Initialize RoboCrane's status structure */
  for (i=0; i<6; i++)
    {
      Initialize_Joint_Status_Type(&(_RC->status.jointStatus[i]));
      Initialize_Joint_Cmd_Type(&(_RC->status.jointCmd[i]));
    }
  Initialize_Aux_Type(&(_RC->status.auxStatus));
}
