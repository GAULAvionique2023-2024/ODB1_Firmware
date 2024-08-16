# Drivers

### WS2812 RGB LED

To use it you need to declare 2 struct, 1 is the buffer that contain the color of the led and the second 1 it contain the info of the channel. Its not clean cause at the begening the driver was for multiple led on up to 16 channels, but I strip down some feature to use less memory and CPU power. (if we have time we could redo this)

```C
struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];
struct led_channel_info led_channels[WS2812_NUM_CHANNELS];
```

Then to be sure that we start with a clean buffer we do a memset.

```C
  memset(led_channels, 0, sizeof(led_channels));
```

After we gave the pointer and its size of the buffer to the channel.
The last step is to Init the driver.

```C
  led_channels[0].framebuffer = channel_framebuffers;
  led_channels[0].length = FRAMEBUFFER_SIZE * sizeof(struct pixel);

  WS2812_Init();
```

The now we only need to call WS2812_Solid_Colors to set a color. the firt 2 parameter its the buffer and the channel. (Again some work can make this cleaner). The last 3 parameter is the RGB values (0-255).

```C
  //Green
  WS2812_Solid_Colors(channel_framebuffers[0], led_channels, 0, 255, 0);

  //Red
  WS2812_Solid_Colors(channel_framebuffers[0], led_channels, 255, 0, 0);

  //Blue
  WS2812_Solid_Colors(channel_framebuffers[0], led_channels, 0, 0, 255);
  ```

### ICM 20602 Driver

To use the ICM driver you need to declare to struc in the main and use the Init fonction like so:

```C
    ICM20602 icm;
    ICM20602_Init(&icm);
```

Then all you need is to check if the data is ready and use the fonction to update all the data.
```C
	  if(ICM20602_Data_Ready())
	  {
		  ICM20602_Update_All(&icm);

		  //printf("Roll: %.2f	Pitch: %.2f \n", icm.angleRoll, icm.anglePitch);
		  printf("Roll: %.2f	Pitch: %.2f \n", icm.kalmanAngleRoll, icm.kalmanAnglePitch);

	  }
```

Here all the data you get:
```C
// Sensor struct
typedef struct{

	// Raw data
	int16_t 	gyroXRaw;
	int16_t 	gyroYRaw;
	int16_t 	gyroZRaw;

	int16_t 	accXRaw;
	int16_t 	accYRaw;
	int16_t 	accZRaw;

	// Real data
	// In degree per second
	float 	gyroX;
	float 	gyroY;
	float 	gyroZ;

  	// In G force
	float 	accX;
	float 	accY;
	float 	accZ;

	float 	temperatureC;
  
	float	angleRoll;
	float	anglePitch;

  	// The angle with the kalman filter
	float	kalmanAngleRoll;
	float	kalmanAnglePitch;

}ICM20602;
```

### RunTimer
RunTimer is a timer that you can use to calculate the elapsed time for its creation. It has 2 functions for the moment.
```c
void  RunTimerInit(RunTimer*  dev);
void  UpdateTime(RunTimer*  dev);
```
The first one is just an Init, and its where the timer begins. The second one, its update the timer. After the update you can access the attribute.
```c
typedef  struct  {
	uint32_t  start_time;
	uint32_t  elapsed_time_ms;
	uint8_t  elapsed_time_s;
	uint16_t  elapsed_time_m;
	uint16_t  elapsed_time_remaining_ms;
}  RunTimer;
```

### Printt(const  char  *format,  ...)
printt() is an overloaded function that put a timestamp in front of your string. It needs a global Runtimer run_timer; I use a global struct here only for eliminating the need to pass the timer in parameters.
```c
printt("Hello World!");
```
Will appear like:
```
[01:24:457] Hello World!
```

### Buzzer Driver
The Buzzer driver provides functions and definitions for controlling a buzzer on the STM32 microcontroller. This driver allows you to manage buzzer behavior with different routines and parameters. Only one function is use.
```c
void Buzz(TIM_HandleTypeDef *htim, uint32_t channel, buzzRoutines_t routine);
```
The `buzzParametres_t` structure defines the parameters for buzzer control:
```c
typedef struct {
    uint8_t nbBips;               // Number of beeps
    int frequencyStart;           // Starting frequency of the beep
    int frequencyEnd;             // Ending frequency of the beep
    uint32_t delayModulation;     // Delay for frequency modulation
    uint32_t delayPause;          // Delay between beeps
} buzzParametres_t;
```
The buzzRoutines_t enumeration defines different sound routines implemented.
```c
typedef enum {
    STOP,    // Stop buzzer
    START,   // Start buzzer
    PENDING, // Buzzer is pending
    ARMED,   // Buzzer is armed
    CRASH    // Buzzer indicates crash
} buzzRoutines_t;
```

### BMP280 Driver
The BMP280 driver provides functions and definitions for interacting with the BMP280 sensor on the STM32 microcontroller. This driver allows you to read temperature and pressure data, perform calibration and change consumption mode.
```c
uint8_t BMP280_Init(BMP280 *dev);
```
Initializes the BMP280 sensor with the specified parameters.
```c
void BMP280_Read_Temperature_Pressure(BMP280 *dev);
```
Reads the temperature and pressure data from the sensor.
```c
float BMP280_PressureToAltitude(float pressure, float sea_level_pressure);
```
Converts pressure readings to altitude based on sea level pressure.
```c
void BMP280_Read_Calib_Data(BMP280 *dev);
```
Reads the calibration data from the BMP280 sensor.
```c
uint8_t BMP280_SwapMode(uint8_t mode);
```
Swaps the mode of operation for the BMP280 sensor.
You can use only 1 function, BMP280_Read_Temperature_Pressure to set temperature/pressure data in the struct. After that, you can use all variables in the BMP280's struct. 
Otherwise, the function BMP280_SwapMode can be used to manually change the module's power consumption mode
```c
typedef struct {
    SPI_TypeDef *SPIx;            // SPI interface for communication
    uint8_t cs_pin;               // Chip select pin
    GPIO_TypeDef *cs_port;        // Chip select port

    float pressure_Pa;            // Pressure in Pascals
    float pressure_kPa;           // Pressure in Kilopascals

    float altitude_MSL;           // Altitude above Mean Sea Level
    float altitude_m;             // Altitude in meters
    float altitude_filtered_m;    // Filtered altitude in meters
    float alpha;                  // Smoothing factor for EMA filter

    float temp_C;                 // Temperature in degrees Celsius
    BMP280_CalibData calib_data;  // Calibration data
    int32_t t_fine;               // Fine temperature data
    float temperature_ref;        // Temperature reference value
    float pressure_ref;           // Pressure reference value
} BMP280;
```
The `BMP280_CalibData` structure holds calibration data for the BMP280 sensor.
```c
typedef struct {
    uint16_t dig_T1;  // Temperature calibration data
    int16_t dig_T2;   // Temperature calibration data
    int16_t dig_T3;   // Temperature calibration data
    uint16_t dig_P1;  // Pressure calibration data
    int16_t dig_P2;   // Pressure calibration data
    int16_t dig_P3;   // Pressure calibration data
    int16_t dig_P4;   // Pressure calibration data
    int16_t dig_P5;   // Pressure calibration data
    int16_t dig_P6;   // Pressure calibration data
    int16_t dig_P7;   // Pressure calibration data
    int16_t dig_P8;   // Pressure calibration data
    int16_t dig_P9;   // Pressure calibration data
} BMP280_CalibData;
```