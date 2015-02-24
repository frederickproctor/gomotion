/*******************************************************************************
 * \author       NIST - BFRL - CMAG group
 * \author       This code developed for the RoboCrane controller project
 *
 * \author       Kamel Saidi
 *               NIST 301-975-6069
 *               kamel.saidi@nist.gov
 *
 * \author       Michal Baczynski
 *               NIST 301-975-6067
 *               michael.baczynski@nist.gov
 *
 * \author       Bob Bunch, Mechanical Engineer
 *               NIST 301-975-2881
 *               Cell 571-338-9220
 *               robert.bunch@nist.gov  OR  bob_bunch@yahoo.com
 *******************************************************************************/

/*******************************************************************************
 * \file         ProfibusIOInterface.c
 * \brief        Profibus IO interface functions.
 *
 *               This file includes the functions to interface with the Wago
 *               nodes on RoboCrane through the Profibus card on the controller
 *               computer.
 *               There are 4 Wago nodes  on RoboCrane: 3 nodes for the 6 motors,
 *               and 1 node for the camera, lights, lasers, inclinometer, and
 *               cable tension sensors.
 *               Each node is composed of several Wago modules.
 *******************************************************************************/

#undef CIF_DEBUG

/* Standard Linux includes */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/errno.h>	/* ENOMEM */
#include <stddef.h>		/* sizeof() */

#include <rtai.h>
#include <rtai_sem.h>		/* rtai semaphores */
#include <rtai_sched.h>		/* rtai scheduling */
#include "rtai_shm.h"		/* rtai_kmalloc(), rtai_kfree() */

/* Profibus card driver includes */
#include <cif_types.h>
#include <cif_dev_i.h>
#include <cif_dev.h>
#include <rtai_cif_api.h>
#include <rcs_user.h>	/* Include file for RCS definition    */
#include <asc_user.h>	/* Include file for ASCII protocols   */
#include <nvr_user.h>	/* Include file for 3964R protocol    */

#include "robocrane.h"		/* Defines RoboCrane's structures and initialization functions */
extern RoboCrane_Type RoboCrane;	/* Main global RoboCrane structure */

/* Define constants for Profibus card dual port memory (DPM) */
#define INBUFFERSIZE 432	/*< Size of input data buffer. */
#define OUTBUFFERSIZE 432	/*< Size of output data buffer. */

/* Define masks for writing or reading Profibus memory */
#define WRITE_MASK 0x01
#define READ_MASK 0x02

/* Define encoder flow state constant variables: over/underflow/neither */
#define ENCODER_NOT_UNDER_OVER_FLOW 0
#define ENCODER_UNDERFLOW 1
#define ENCODER_OVERFLOW 2

/* Global variables */
unsigned char ProfiInputDataBuffer[INBUFFERSIZE];
unsigned char ProfiOutputDataBuffer[OUTBUFFERSIZE];

/* Define Hi and Lo Byte manipulation functions */
/*! Construct a Hi byte from a short (8 bits) */
unsigned char HiByte(short value)
{
  unsigned char hiByte = (value >> 8);
  return hiByte;
}

/*! Construct a Lo byte from a short  (8 bits)*/
unsigned char LoByte(short value)
{
  unsigned char loByte = value;
  return loByte;
}

/*! Construct a Word from a Hi and a Lo byte  (16 bits)*/
unsigned short Word(unsigned char hiByte, unsigned char loByte)
{
  unsigned short word = (hiByte << 8) + loByte;
  return word;
}

/*! Construct a long Word from  Hi and a Lo Word  (32 bits)*/
unsigned int LongWord(unsigned int hiWord, unsigned int loWord)
{
  return (hiWord << 16) + loWord;
}


/*! \brief      Update output data buffer function.
 *
 *               Function that constructs the buffer that will be written to
 *               the Profibus card, which then sends it to the Wago nodes.
 */
int UpdateOutputDataBuffer(void)
{
  /* Define output buffer indexing - i.e., the byte address of where commands
   *  should be written in the Profibus memory, which correspond to appropriate
   *  modules in the 4 Wago nodes. */
  short iMotorVoltCmdHiByte[6] = { 12, 14, 112, 114, 212, 214 };
  short iMotorVoltCmdLoByte[6] = { 13, 15, 113, 115, 213, 215 };
  short iAmpEnableByte[6] = { 24, 24, 124, 124, 224, 224 };
  short iTNodeDigOutByte = 300;	/* Tool node dig out byte */
  /*      short iEncoderControlByte[6]= {0,  6, 100, 106, 200, 206}; */
  /*      short iEncoderSetLoByte[6]= {1,  7, 101, 107, 201, 207}; */
  /*      short iEncoderSetHiByte[6]= {2,  8, 102, 108, 202, 208}; */

  /* Define masking */
  unsigned char DigOutChn1_2 = 0x03;	/* low nibble, low 2 bits (Lights) */
  unsigned char DigOutChn3_4 = 0x0C;	/* low nibble, high 2 bits (Camera) */
  unsigned char DigOutChn5_6 = 0x30;	/* high nibble, low 2 bits (Lasers) */
  unsigned char AmpEnableMask[6] = { 0x01, 0x02, 0x01, 0x02, 0x01, 0x02 };
  short digValue[6];
  short axis;

  /* Define the zero offset for each of the 6 motor amplifier. I.e., the voltage that
   *  must be sent to the amplifier in order for the motor not to move.
   *  Normally the amplifiers are balanced manually by turning a potentiometer on each
   *  amplifier until the motor is stationary, however, we chose to do this in software
   *  in order to avoid having to take RoboCrane appart to gain access to the amplifier
   *  balance potentiometers.  */
  double AmpZeroOffset[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0878 };

  /* Update the 4th Wago node (tool node) commands - light, camera, and lasers ON/OFF */
  ProfiOutputDataBuffer[iTNodeDigOutByte] = (RoboCrane.cmd.auxCmd.light * DigOutChn1_2);	/* Light is ON/OFF */
  ProfiOutputDataBuffer[iTNodeDigOutByte] += (RoboCrane.cmd.auxCmd.camera * DigOutChn3_4);	/* Camera is ON/OFF */
  ProfiOutputDataBuffer[iTNodeDigOutByte] += (RoboCrane.cmd.auxCmd.lasers * DigOutChn5_6);	/* Lasers is ON/OFF */

  /* Update the first 3 Wago node commands (amplifier enable/disable and motor voltages) */
  for (axis = 0; axis < 6; axis++) {
    /* The calculations below are done in 16 bit whereas the analog output module
     *  (WAGO 750-556) has a 12 bit resolution for the digital input (plus
     *  1 bit for the sign - which is not documented). The module accepts a 16 bit
     *  and ignores the 3 least significant bits).
     *  The RoboCrane.cmd.jointCmd[axis].CmdMotorVolt are the output from the PID
     *  servo loops run in the realtime controller task. */
    if (RoboCrane.cmd.jointCmd[axis].CmdMotorVolt >= 10.0f)
      digValue[axis] = 65536 / 2 - 1;
    else if (RoboCrane.cmd.jointCmd[axis].CmdMotorVolt <= -10.0f)
      digValue[axis] = -65536 / 2;
    else
      digValue[axis] = (short) ((RoboCrane.cmd.jointCmd[axis].CmdMotorVolt + AmpZeroOffset[axis]) * (65536.0f / 20.0f));	/* 0.0878 is the zero offset for axis 6 */

    /* Update the amplifier enable/disable bytes */
    ProfiOutputDataBuffer[iAmpEnableByte[axis]] =
      ((ProfiOutputDataBuffer[iAmpEnableByte[axis]] & (~AmpEnableMask[axis]))
       | (RoboCrane.cmd.jointCmd[axis].AmpEnable * AmpEnableMask[axis]));
    /*                printf("ProfiOutputBuffer[iAmpEnableByte[%d]: %d\n", axis, ProfiOutputBuffer[iAmpEnableByte[axis]]); */

    /* Update the motor voltage bytes */
    ProfiOutputDataBuffer[iMotorVoltCmdHiByte[axis]] = HiByte(digValue[axis]);
    ProfiOutputDataBuffer[iMotorVoltCmdLoByte[axis]] = LoByte(digValue[axis]);
  }
  return 0;
}


/*! \brief      Profibus card intialization function.
 *
 *               Initializes the Profibus card (board). This function was adapted from
 *               demo.c file provided by Hilscher with their RTAI/Linux Profibus card
 *               driver (cif_rtai).
 */
void ProfiInit(void)
{
  IOINFO tIoInfo;
  short Ret;			/*< Function return value. */
  unsigned short usBoardNumber = 0;	/*< Profibus card (board) number. */

  /* Check if initialization falied */
  if ((Ret = rtai_cif_DevInitBoard(0)) != DRV_NO_ERROR)
    DBG_PRN("DevInitBoard          Profi Error = %5d \n", Ret);

  /* If initialization is successful, signal Profibus board that the host
   *  is ready using the rtai_cif_DevSetHostState(device number, mode,...)
   *  function. */
  else if ((Ret = rtai_cif_DevSetHostState(0, HOST_READY, 0L)) !=
	   DRV_NO_ERROR)
    DBG_PRN("DevSetHostState (HOST_READY)  Profi Error = %5d \n", Ret);

  /* Test if IO-Communication is available */
  DBG_PRN("\n\n*** Test for ExchangeIO functions ***\n\n");

  Ret = rtai_cif_DevGetInfo(usBoardNumber,
			    GET_IO_INFO,
			    sizeof(IOINFO), (unsigned char *) &tIoInfo);

  DBG_PRN("DevGetInfo (IOINFO) RetWert = %5d \n", Ret);

  /* Print IO information */
  DBG_PRN("  ExchangeIO information\n"
	  "  IO-Communication mode : %d\n"
	  "  IO-COM bit            : %d\n"
	  "  IO-Count              : %ld\n",
	  tIoInfo.bIOExchangeMode, tIoInfo.bComBit, tIoInfo.ulIOExchangeCnt);

  /* If no IO-Communication available */
  if (tIoInfo.bIOExchangeMode == 0)
    DBG_PRN("  --- IO-Communication NOT AVAILABLE ---\n");

  /* If IO-Communication is available */
  else {
    DBG_PRN("  --- IO-Communication AVAILABLE ---\n");

    /* Test if IO-Communication is running */
    if (tIoInfo.bComBit == 0)	/* No Communication is running */
      DBG_PRN("  IO-Communication is not running, so all function\n"
	      "  returning an error !.\n");
  }
}


/*! \brief      Profibus card cleanup function.
 *
 *               Exits the Profibus card (board) and closes the communication.
 */
void ProfiCleanup(void)
{
  short Ret;			/*< Function return value. */
  /* Signal the Profibus card that the host is finished. NB: This function was
   *  added by Bob, but it was not implemented in the demo.c file provided by
   *  Hilscher. It caused the controller to crash when implemented, so we commented
   *  it out. */
  /*if ( (Ret = rtai_cif_DevSetHostState(       0, HOST_NOT_READY, 0L) ) != DRV_NO_ERROR) */
  /*      DBG_PRN( "DevSetHostState (HOST_NOT_READY)  Profi Error = %5d \n", Ret ); */

  /* Close the Profibus communication */
  Ret = rtai_cif_DevExitBoard(0);
  DBG_PRN("DevExitBoard 	  Profi Status = %5d \n", Ret);
}


/*! \brief      Profibus card read/write function.
 *
 *               Reads and/or writes to/from the Profibus card memory.
 */
void ProfiDataCommProcess(char mode)
{
  unsigned short usBoardNumber = 0;	/*< Profibus card (board) number. */
  short usSendSize, usReceiveSize;	/*< Used for the DevExchangeIO() function. */
  RTIME hrtimeout = 10000000000;
  short Ret;			/*< Function return value. */

  /* Check if we are writing to or reading from the Profibus card (or both) */
  if ((mode & WRITE_MASK) == WRITE_MASK)
    usSendSize = OUTBUFFERSIZE;
  else
    usSendSize = 0;
  if ((mode & READ_MASK) == READ_MASK)
    usReceiveSize = INBUFFERSIZE;
  else
    usReceiveSize = 0;


  if ((Ret = rtai_cif_DevExchangeIO(usBoardNumber, 0,	/*< usSendOffset */
				    usSendSize,	/*< usSendSize -> 0 if no write */
				    &ProfiOutputDataBuffer[0],	/*< *pvSendData -> pointer to buffer */
				    0,	/*< usReceiveOffset */
				    usReceiveSize, &ProfiInputDataBuffer[0],	/*< *pvReceiveData -> pointer to buffer */
				    &hrtimeout)) != DRV_NO_ERROR)
    /*< ulTimeout */
    DBG_PRN("DevExchangeIO 	  Profi Error = %5d \n", Ret);

  return;
}


/*! \brief      Update the 4-byte motor encoder counts.
 *
 *               Reads the 2-byte motor encoder counts from the Wago modules
 *               and checks for under or overflow conditions in order to update
 *               the 4-byte encoder count.
 */
void UpdateEncoderCount(int axis)
{
  /* Define input buffer indexing - i.e., the byte address of where values
   *  should be read from the Profibus memory, which correspond to appropriate
   *  modules in the 4 Wago nodes. */
  short iEncoderInStatByte[6] = { 0, 6, 100, 106, 200, 206 };
  short iEncoderInLoByte[6] = { 2, 8, 102, 108, 202, 208 };
  short iEncoderInHiByte[6] = { 1, 7, 101, 107, 201, 207 };

  int EncoderStatusByte;	/*< Local underflow/overflow status byte */
  int EncoderInHiWord = (RoboCrane.status.jointStatus[axis].EncoderCnt & 0xFFFF0000) >> 16;	/*< Encoder Hi Word. */
  int EncoderInLoWord = Word(ProfiInputDataBuffer[iEncoderInHiByte[axis]],
			     ProfiInputDataBuffer[iEncoderInLoByte[axis]]);	/*< Encoder Lo Word. */

  EncoderStatusByte = ProfiInputDataBuffer[iEncoderInStatByte[axis]];	/*< Read the encoder status byte. */

  /* Check for underflow condition */
  if (((EncoderStatusByte & 0x08) == 0x08)
      && (RoboCrane.status.jointStatus[axis].EncoderFlowState !=
	  ENCODER_UNDERFLOW)) {
    EncoderInHiWord--;		/* Decrement encoder high word by 1 */
    RoboCrane.status.jointStatus[axis].EncoderFlowState = ENCODER_UNDERFLOW;	/* Update previous state */
  }

  /* Check for overflow condition */
  if (((EncoderStatusByte & 0x10) == 0x10)
      && (RoboCrane.status.jointStatus[axis].EncoderFlowState !=
	  ENCODER_OVERFLOW)) {
    EncoderInHiWord++;		/* Increment encoder high word by 1 */
    RoboCrane.status.jointStatus[axis].EncoderFlowState = ENCODER_OVERFLOW;	/* Update previous state */
  }

  /* Check for not under/overflow condition */
  if (((EncoderStatusByte & 0x08) != 0x08)
      && ((EncoderStatusByte & 0x10) != 0x10)) {
    RoboCrane.status.jointStatus[axis].EncoderFlowState = ENCODER_NOT_UNDER_OVER_FLOW;	/* Update previous state */
  }

  RoboCrane.status.jointStatus[axis].EncoderCntLast = RoboCrane.status.jointStatus[axis].EncoderCnt;	/*< Stores the last 4-byte encoder count. */

  RoboCrane.status.jointStatus[axis].EncoderCnt = LongWord(EncoderInHiWord, EncoderInLoWord);	/*< Stores the most recent 4-byte encoder count. */
}
