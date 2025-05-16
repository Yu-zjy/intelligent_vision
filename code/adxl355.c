#include "adxl355.h"

void ADXL355_Filter(uint8_t hpf, uint8_t lpf);
void ADXL355_Range(uint8_t range);
int32_t ADXL355_AccDataConversion(uint32_t ui32SensorData);
void ADXL355_Startup(void);
void ADXL355_Standby(void);
void ADXL355_Init(ADXL355_HandleTypeDef *ADXL355_t);
void ADXL355_WriteRegister(uint8_t ui8address, uint8_t ui8Data);
uint32_t ADXL355_ReadRegister(uint8_t ui8address, enRegsNum enRegs);
void adxlGetParameter(uint16 num);

/****************************** Global Data ***********************************/

int32_t volatile i32SensorX;
int32_t volatile i32SensorY;
int32_t volatile i32SensorZ;
int32_t volatile i32SensorT;
uint32_t volatile ui32SensorX;
uint32_t volatile ui32SensorY;
uint32_t volatile ui32SensorZ;
uint32_t volatile ui32SensorT;
float volatile AZ,AY,AX,AT;
static float Xmax=-100,Xmin=100,Xaverage=0,Ymax=-100,Ymin=100,Yaverage=0;
/************************* Static functions *****************************/

/**
 * @brief Select ADXL355 by resetting CHIP SELECT Pin
 *
 * @return none
 **/
//static void ADXL355_Select() {
//	HAL_GPIO_WritePin(ADXL355_CS_PORT, ADXL355_CS_PIN, GPIO_PIN_RESET);
//}
/**
 * @brief Deselect ADXL355 by setting CHIP SELECT Pin
 *
 * @return none
 **/
//static void ADXL355_Unselect() {
//	HAL_GPIO_WritePin(ADXL355_CS_PORT, ADXL355_CS_PIN, GPIO_PIN_SET);
//}

/************************* Global scope functions *****************************/

/**
 * @brief Initialize with parameters and Start ADXL355
 *
 * @param SPI handle Structure
 * @param ADXL355 handle Structure
 *
 * @return none
 **/
void accADXL355_Init(void)
{
  ADXL355_HandleTypeDef ADXL355_t;
  ADXL355_t.ADXL355_Range = ADXL355_RANGE_8G;
  ADXL355_t.ADXL355_LowPass = ADXL355_ODR_1000;
  ADXL355_t.ADXL355_HighPass = 0x00;
  ADXL355_Init(&ADXL355_t);
  adxlGetParameter(10000);
}
void ADXL355_Init(ADXL355_HandleTypeDef *ADXL355_t) {
  system_delay_ms(10);  //上电延时
  (void)spi_init(ADXL_SPI_NUM, ADXL_SPI_CS_PIN, 10*1000*1000, ADXL_SPI_SCK_PIN, ADXL_SPI_MOSI_PIN,ADXL_SPI_MISO_PIN,ADXL_SPI_CS_PIN);//硬件SPI初始化
  if( ADXL355_ReadRegister(ADXL355_PARTID,SPI_READ_ONE_REG) != 0xED)
  {
    while(1);//检查接线
  }
  ADXL355_Range(ADXL355_t->ADXL355_Range); /*Set G range of ADXL355*/
  ADXL355_Filter(ADXL355_t->ADXL355_HighPass, ADXL355_t->ADXL355_LowPass); /*Set filters of ADXL355*/
  ADXL355_Startup(); /*Turn on measurement mode of ADXL355*/
}

/**
 * @brief Write data to a register in requested address
 * @param SPI handle Structure
 * @param Register address
 * @param Data to write
 *
 * @return none
 **/
void ADXL355_WriteRegister(uint8_t ui8address,	uint8_t ui8Data) 
{
  uint8_t data[3];
  data[0] = ((ui8address << 1) | ADXL355_WRITE); /* Combine write register address and Write command */
  data[1] = ui8Data;
  data[2] = 0xff;
  spi_mosi(ADXL_SPI_NUM,ADXL_SPI_CS_PIN,data,data,3,1);

}

/**
 * @brief Read data from a register in requested address
 *
 * @param SPI handle Structure
 * @param Register address
 * @param Number of register to read (SPI_READ_ONE_REG,SPI_READ_TWO_REG or SPI_READ_THREE_REG )
 *
 * @return Data in the register
 **/
uint32_t ADXL355_ReadRegister(uint8_t ui8address,enRegsNum enRegs) 
{

  uint8_t ui24Result[4];
  uint32_t ui32Result = 0;
  uint8_t ui8writeAddress[4];

  ui8writeAddress[0] = ((ui8address << 1) | ADXL355_READ); /* Combine read register address and READ command */
//  HAL_SPI_Transmit(hspi, &ui8writeAddress, 1, 1); /* Send register address */
  if (enRegs == SPI_READ_ONE_REG) {
    spi_mosi(ADXL_SPI_NUM,ADXL_SPI_CS_PIN,&ui8writeAddress[0],ui24Result,2,1);
//    HAL_SPI_Receive(hspi, ui24Result, 1, 1);
    ui32Result = ui24Result[1];

  }
  else if (enRegs == SPI_READ_TWO_REG) { /* Only used for Temp & X,Y,Z offset and threshold registers*/

//          HAL_SPI_Receive(hspi, ui24Result, 2, 1);
    spi_mosi(ADXL_SPI_NUM,ADXL_SPI_CS_PIN,&ui8writeAddress[0],ui24Result,3,1);
    /* Combine 2Bit register into one uint32 */
    ui32Result = ((ui24Result[1] << 8) | ui24Result[2]);

  } else { /* Only used for X,Y,Z axis data registers*/

//          HAL_SPI_Receive(hspi, ui24Result, 3, 1);
    spi_mosi(ADXL_SPI_NUM,ADXL_SPI_CS_PIN,&ui8writeAddress[0],ui24Result,4,1);
    /* Combine 3Bit register into one uint32 */
    ui32Result = ((ui24Result[1] << 16) | (ui24Result[2] << 8) | ui24Result[3]);

  }

  return ui32Result;
}

/**
 * @brief Turns ADXL355 measurement mode.
 *
 * @param SPI handle Structure
 *
 * @return none
 *
 **/
void ADXL355_Startup(void) 
	{
	uint8_t ui8temp;

	ui8temp = (uint8_t) ADXL355_ReadRegister(ADXL355_POWER_CTL, SPI_READ_ONE_REG); /*Read POWER_CTL register, before modifying it */

	ui8temp &= ~(0x01); /* Set measurement bit in POWER_CTL register */

	ADXL355_WriteRegister(ADXL355_POWER_CTL, ui8temp); /* Write the new value to POWER_CTL register */
//        ui8temp = (uint8_t) ADXL355_ReadRegister(ADXL355_POWER_CTL, SPI_READ_ONE_REG);
//        ui8temp = (uint8_t) ADXL355_ReadRegister(ADXL355_POWER_CTL, SPI_READ_ONE_REG);
}

/**
 * @brief Puts ADXL355 into standby mode.
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ADXL355_Standby(void) {
    uint8_t ui8temp;

    ui8temp = (uint8_t) ADXL355_ReadRegister(ADXL355_POWER_CTL, SPI_READ_ONE_REG); /*Read POWER_CTL register, before modifying it */

    ui8temp =0x01;//|= 0x01; /* Clear measurement bit in POWER_CTL register */

    ADXL355_WriteRegister(ADXL355_POWER_CTL, ui8temp); /* Write the new value to POWER_CTL register */
//    ui8temp = (uint8_t) ADXL355_ReadRegister(ADXL355_POWER_CTL, SPI_READ_ONE_REG);
//    ui8temp = (uint8_t) ADXL355_ReadRegister(ADXL355_POWER_CTL, SPI_READ_ONE_REG);

}

/**
 * @brief Reads the accelerometer data.
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ADXL355_ReadData(void) {

	/* Receive raw acceleration datas from accelerometer */
	ui32SensorX = ADXL355_ReadRegister(ADXL355_XDATA3, SPI_READ_THREE_REG);
	ui32SensorY = ADXL355_ReadRegister(ADXL355_YDATA3, SPI_READ_THREE_REG);
	ui32SensorZ = ADXL355_ReadRegister(ADXL355_ZDATA3, SPI_READ_THREE_REG);
//	ui32SensorT = ADXL355_ReadRegister(ADXL355_TEMP2, SPI_READ_TWO_REG);

	/* Receive signed integer raw datas */
	i32SensorX = ADXL355_AccDataConversion(ui32SensorX);
	i32SensorY = ADXL355_AccDataConversion(ui32SensorY);
	i32SensorZ = ADXL355_AccDataConversion(ui32SensorZ);
//	i32SensorT = ADXL355_AccDataConversion(ui32SensorT);
        
        AZ = (float) i32SensorZ / ADXL355_RANGE_8G_SCALE ; /* Convert raw acceleration data into g and write into the buffer*/
        AX = (float) i32SensorX / ADXL355_RANGE_8G_SCALE ;
        AY = (float) i32SensorY / ADXL355_RANGE_8G_SCALE ;
//        AT = (float)(ui32SensorT - ADXL355_TEMP_BIAS)/(ADXL355_TEMP_SLOPE)+25;
}

/**
 * @brief Changes high pass and low pass filter
 *
 * @param SPI handle Structure
 * @param High pass filter value
 * @param Low pass filter value(ie: ADXL355_ODR_1000, ADXL355_ODR_2000, ADXL355_ODR_4000);
 *
 * @return none
 *
 **/
void ADXL355_Filter(uint8_t hpf, uint8_t lpf) {
    uint8_t filter = 0;

    filter = (hpf << 4) | lpf; /* Combine high pass and low pass filter values to send */
    ADXL355_Standby();
    ADXL355_WriteRegister(ADXL355_FILTER, filter);/* Set filter values within FILTER register */
    ADXL355_Startup();
}

/**
 *  @brief Changes ranges to 2g, 4g or 8g.
 *
 * @param SPI handle Structure
 *
 * @return none
 **/
void ADXL355_Range(uint8_t range) {

	ADXL355_WriteRegister(ADXL355_RANGE, range); /* Set sensor range within RANGE register */

}

/**
 * @brief Convert the two's complement data in X,Y,Z registers to signed integers
 *
 * @param Raw data from register
 *
 * @return int32_t converted signed integer data
 **/
int32_t ADXL355_AccDataConversion(uint32_t ui32SensorData) {
	int32_t volatile i32Conversion = 0;

	ui32SensorData = (ui32SensorData >> 4);
	ui32SensorData = (ui32SensorData & 0x000FFFFF);

	if ((ui32SensorData & 0x00080000) == 0x00080000) {

		i32Conversion = (ui32SensorData | 0xFFF00000);

	} else {
		i32Conversion = ui32SensorData;
	}

	return i32Conversion;
}

void adxlGetParameter(uint16 num)
{
  double  Xadd,Yadd;
  for(int i = 0; i < num ; i++)
  {
    ADXL355_ReadData();
    Xadd+=AX;
    if(AX > Xmax)
    {
      Xmax = AX;
    }
    else if(AX < Xmin)
    {
      Xmin = AX;
    }
    Yadd+=AY;
    if(AY > Ymax)
    {
      Ymax = AY;
    }
    else if(AY < Ymin)
    {
      Ymin = AY;
    }
  }
  
  Xaverage = Xadd/num;
  Yaverage = Yadd/num;
  
}

void adxlWindowFilter(void)
{
  if(AX<= Xmax && AX >= Xmin)
  {
    AX = 0;
  }
  else{
    AX -= Xaverage;
  }
  
  if(AY<= Ymax && AY >= Ymin)
  {
    AY = 0;
  }
  else{
    AY -= Yaverage;
  }
  
  
}


