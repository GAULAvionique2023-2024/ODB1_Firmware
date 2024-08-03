# ODB1_Firmware

![GAUL Banner](doc/logo-full.webp)

Code for the on-board computer (ODB1) of the GAUL.

## Drivers

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
