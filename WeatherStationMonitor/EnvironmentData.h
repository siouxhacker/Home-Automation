#ifndef EnvironmentData_h
#define EnvironmentData_h

enum PktType
{
  THP=0,
  WIND,
  RN
};

typedef struct
{
  uint8_t type;
  float   batLvl;         // Battery level in volts
} CommonHdr;

typedef struct
{
  CommonHdr hdr;
  float     temperature;	// Temperature Deg F
  float     humidity;	// Relative Humidity %
  float     pressure;	// Barometric pressure In Hg
} THPData;

typedef struct
{
  CommonHdr hdr;
  float  windSpeed;   // Speed of wind MPH
  char   windDir[4];  // Wind direction
} WindData;

typedef struct
{
  CommonHdr hdr;
  float     rainAmount;   // Rainfall in inches    
} RainData;

#endif
