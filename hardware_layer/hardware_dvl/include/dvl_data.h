#ifndef DVL_DATA_H
#define DVL_DATA_H

#pragma pack(push, 1)

typedef struct
{
  unsigned char sync;
  unsigned char hdrSize;
  unsigned char ID;
  unsigned char family;
  unsigned short dataSize;
  unsigned short dataChecksum;
  unsigned short hdrChecksum;
} DVLHeader;

typedef struct
{
  unsigned long beam1VelValid : 1;
  unsigned long beam2VelValid : 1;
  unsigned long beam3VelValid : 1;
  unsigned long beam4VelValid : 1;
  unsigned long beam1DistValid : 1;
  unsigned long beam2DistValid : 1;
  unsigned long beam3DistValid : 1;
  unsigned long beam4DistValid : 1;
  unsigned long beam1FOMValid : 1;
  unsigned long beam2FOMValid : 1;
  unsigned long beam3FOMValid : 1;
  unsigned long beam4FOMValid : 1;
  unsigned long xVelValid : 1;
  unsigned long yVelValid : 1;
  unsigned long z1VelValid : 1;
  unsigned long z2VelValid : 1;
  unsigned long xFOMValid : 1;
  unsigned long yFOMValid : 1;
  unsigned long z1FOMValid : 1;
  unsigned long z2FOMValid : 1;
  unsigned long procIdle3 : 1;
  unsigned long procIdle6 : 1;
  unsigned long procIdle12 : 1;
  unsigned long _empty1 : 5;
  unsigned long wakeupstate : 4;
} DVLstatus;

typedef struct
{
  unsigned char version;
  unsigned char offsetOfData;
  unsigned long serialNumber;
  unsigned char year;
  unsigned char month;
  unsigned char day;
  unsigned char hour;
  unsigned char minute;
  unsigned char seconds;
  unsigned short microSeconds100;
  unsigned short nBeams;
  unsigned long error;
  DVLstatus status;
  float soundSpeed;
  float temperature;
  float pressure;
/* Beam data */
  float velBeam[4];
  float distBeam[4];
  float fomBeam[4];
  float timeDiff1Beam[4];
  float timeDiff2Beam[4];
  float timeVelEstBeam[4];
/* XYZ data */
  float velX;
  float velY;
  float velZ1;
  float velZ2;
  float fomX;
  float fomY;
  float fomZ1;
  float fomZ2;
  float timeDiff1X;
  float timeDiff1Y;
  float timeDiff1Z1;
  float timeDiff1Z2;
  float timeDiff2X;
  float timeDiff2Y;
  float timeDiff2Z1;
  float timeDiff2Z2;
  float timeVelEstX;
  float timeVelEstY;
  float timeVelEstZ1;
  float timeVelEstZ2;
} DVLData;

typedef struct {
  DVLHeader header;
  DVLData data;
} DVLformat21_t;

#pragma pack(pop)

inline unsigned short calculateChecksum(unsigned short *pData, unsigned short size)
{
  unsigned short checksum = 0xB58C;
  unsigned short nbshorts = (size >> 1);
  int i;
  for (i = 0; i < nbshorts; i++)
  {
    checksum += *pData;
    size -= 2;
    pData++;
  }
  if (size > 0)
  {
    checksum += ((unsigned short)(*pData)) << 8;
  }
  return checksum;
}

#endif //DVL_DVL_DATA_H