#ifndef PCANDEVICE_H
#define PCANDEVICE_H

#include <stdio.h>
#include <dlfcn.h>
#include <libpcan.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include "wired_control/PCAN_Data.h"

#define DEFAULT_NODE "/dev/pcan32"

typedef DWORD   (*funCAN_Init_TYPE)(HANDLE hHandle, WORD wBTR0BTR1, int nCANMsgType);
typedef HANDLE  (*funLINUX_CAN_Open_TYPE)(const char *szDeviceName, int nFlag);
typedef DWORD   (*funCAN_VersionInfo_TYPE)(HANDLE hHandle, LPSTR lpszTextBuff);
typedef DWORD   (*funCAN_Close_TYPE)(HANDLE hHandle);
typedef DWORD   (*funLINUX_CAN_Read_TYPE)(HANDLE hHandle, TPCANRdMsg* pMsgBuff);
typedef DWORD   (*funLINUX_CAN_Write_TYPE)(HANDLE hHandle, TPCANMsg* pMsgBuff);
typedef DWORD   (*funCAN_Status_TYPE)(HANDLE hHandle);
typedef int     (*funnGetLastError_TYPE)(void);

class PcanDevice
{
public:
  PcanDevice();

  void recvPcanData(wired_control::PCAN_Data *msg);
  void sendPcanData(const wired_control::PCAN_Data::ConstPtr &msg);

private:
  void *libmHandle;
  HANDLE pcanHandle;

  funCAN_Init_TYPE        funCAN_Init;
  funLINUX_CAN_Open_TYPE  funLINUX_CAN_Open;
  funCAN_VersionInfo_TYPE funCAN_VersionInfo;
  funCAN_Close_TYPE       funCAN_Close;
  funLINUX_CAN_Read_TYPE  funLINUX_CAN_Read;
  funLINUX_CAN_Write_TYPE  funLINUX_CAN_Write;
  funCAN_Status_TYPE      funCAN_Status;
  funnGetLastError_TYPE   funnGetLastError;
};

#endif // PCANDEVICE_H
