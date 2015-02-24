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
* \file         ProfibusIOInterface.h
* \brief        Declarations for Profibus IO interface functions.
*
*               This file includes the functions to interface with the Wago
*               nodes on RoboCrane through the Profibus card on the controller
*               computer.
*               There are 4 Wago nodes  on RoboCrane: 3 nodes for the 6 motors,
*               and 1 node for the camera, lights, lasers, inclinometer, and
*               cable tension sensors.
*               Each node is composed of several Wago modules.
*******************************************************************************/

#ifndef PROFIBUS_IO_INTERFACE_H
#define PROFIBUS_IO_INTERFACE_H

extern int UpdateOutputDataBuffer(void);
extern void ProfiDataCommProcess(char);
extern void UpdateEncoderCount(int axis);
extern void ProfiInit(void);
extern void ProfiCleanup(void);
#define WRITE_MASK 0x01
#define READ_MASK 0x02

#endif

