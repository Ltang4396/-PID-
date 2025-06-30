#include "wired_control/pcandevice.h"

PcanDevice::PcanDevice()
{
  libmHandle = dlopen("libpcan.so", RTLD_LAZY);
  if(!libmHandle){

      printf("Open Error:%s.\n", dlerror());
      return;
  }

  char *errorInfo;  //error information pointer
  this->funCAN_Init = (funCAN_Init_TYPE)dlsym(libmHandle, "CAN_Init");
  this->funLINUX_CAN_Open = (funLINUX_CAN_Open_TYPE)dlsym(libmHandle, "LINUX_CAN_Open");
  this->funCAN_Close = (funCAN_Close_TYPE)dlsym(libmHandle, "CAN_Close");
  this->funCAN_VersionInfo = (funCAN_VersionInfo_TYPE)dlsym(libmHandle, "CAN_VersionInfo");
  this->funLINUX_CAN_Read = (funLINUX_CAN_Read_TYPE)dlsym(libmHandle, "LINUX_CAN_Read");
  this->funLINUX_CAN_Write = (funLINUX_CAN_Write_TYPE)dlsym(libmHandle, "CAN_Write");
  this->funCAN_Status = (funCAN_Status_TYPE)dlsym(libmHandle, "CAN_Status");
  this->funnGetLastError = (funnGetLastError_TYPE)dlsym(libmHandle, "nGetLastError");

  errorInfo = dlerror();
  if (errorInfo != NULL){

      printf("Dlsym Error:%s.\n",errorInfo);
  }

  char txt[VERSIONSTRING_LEN];                //store information of can version
  unsigned short wBTR0BTR1 = CAN_BAUD_500K;   //set the communicate baud rate of can bus
  int nExtended = CAN_INIT_TYPE_EX;           //set can message int standard model
  const char  *szDevNode = DEFAULT_NODE;      //define const pointer point to device name
  pcanHandle = funLINUX_CAN_Open(szDevNode, O_RDWR); //use mapping function

  if(pcanHandle){

      printf("CAN Bus test: %s have been opened\n", szDevNode);
      errno = funCAN_VersionInfo(pcanHandle, txt);
      if (!errno) {

          printf("CAN Bus test: driver version = %s\n", txt);
      } else {

          perror("CAN Bus test: CAN_VersionInfo()");
      }
      if (wBTR0BTR1) {

        errno = funCAN_Init(pcanHandle, wBTR0BTR1, nExtended);
        if (errno) {

            perror("CAN Init Error!");
        } else {

            printf("Device Info: %s;\n", szDevNode);
        }
      }
  } else {

      printf("CAN Bus test: can't open %s\n", szDevNode);
  }
}

void PcanDevice::sendPcanData(const wired_control::PCAN_Data::ConstPtr &msg)
{
  TPCANMsg temp;
  temp.ID = msg.get()->id;

  if(msg.get()->msgType) {

    temp.MSGTYPE = MSGTYPE_EXTENDED;
  } else {

    temp.MSGTYPE = MSGTYPE_STANDARD;
  }

  temp.LEN = msg.get()->dlc;

  for (int i=0; i<msg.get()->dlc; i++) {

    temp.DATA[i] = msg.get()->datas[i];
  }

  if (this->funLINUX_CAN_Write(this->pcanHandle, &temp)) {

       perror("receivetest: LINUX_CAN_Write()" + errno);
  }
}

void PcanDevice::recvPcanData(wired_control::PCAN_Data *msg)
{
  TPCANRdMsg m;
  __u32 status;

  if (this->funLINUX_CAN_Read(this->pcanHandle, &m)) {

      perror("receivetest: LINUX_CAN_Read()");
  } else {

    printf("%u.%3u \n", m.dwTime, m.wUsec);
    msg->id = m.Msg.ID;

    for(__u8 i=0;i<8;i++) {

      msg->datas[i] = m.Msg.DATA[i];
    }
  }

  // check if a CAN status is pending
  if (m.Msg.MSGTYPE & MSGTYPE_STATUS) {

    status = funCAN_Status(pcanHandle);
    if ((int)status < 0) {

      errno = funnGetLastError();
      perror("receivetest: CAN_Status()");
    }

    // printf("receivetest: pending CAN status 0x%04x read.\n", (__u16)status);
  }
}
