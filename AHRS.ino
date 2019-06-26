#include "HyperDisplay_UG2856KLBAG01.h"
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <avr/pgmspace.h>
#include <Math.h>

#define SPI_PORT SPI        // Used if USE_SPI == 1

#define RES_PIN 2
#define CS_PIN 19
#define DC_PIN 18

#define RADIAN_CONVERSION 0.01745329

Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -3.66F, -37.46F, 49.11F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  1.025,  -0.016,  -0.015 },
                                    {  -0.016,  0.949, 0.030 },
                                    {  -0.015, 0.030,  1.029 } };

float mag_field_strength        = 41.23F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony filter;

UG2856KLBAG01_SPI myTOLED;  // Declare a SPI-based Transparent OLED object called myTOLED

uint8_t color = 0x01;
uint8_t noColor = 0x00;

int windowWidth = 64;
int windowHeight = 64;
int pitchScaleLength = (128 - windowWidth) / 2;

//uint8_t horizonWindowMem[64*64];    // Reserve 64*64 pixels worth of memory

wind_info_t defaultWindow, horizonWindow;

float heading = 0;
float pitch = 0;
float roll = 0;

int pitchOffset = 0;
int xCenter = windowWidth / 2;
int xOffset = 0;
int yOffset = 0;

int lastPitchOffset = 0;
int lastXOffset = 0;
int lastYOffset = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("AHRS Begin."); Serial.println("");
  
  SPI_PORT.begin();
  myTOLED.begin(CS_PIN, DC_PIN, SPI_PORT);                  // Begin for SPI requires that you provide the CS and DC pin numbers

  myTOLED.setWindowDefaults(&defaultWindow);
  myTOLED.setWindowDefaults(&horizonWindow);
  
  //coordinates of top left corner of horizon window
  horizonWindow.xMin = (128 - windowWidth) / 2;
  horizonWindow.yMin = 0;

  //coordinates of bottom right corner caluclated with window width and height
  horizonWindow.xMax = horizonWindow.xMin + windowWidth;
  horizonWindow.yMax = windowHeight;
  
//  myTOLED.setWindowMemory(&horizonWindow, (color_t)horizonWindowMem, windowWidth*windowHeight);    // Make sure that the window knows about the memory

  initSensors();
}

void loop() {
  //get Attitude data from the sensors
  getAttitude();

  //calculate the offsets that will be used to draw the display
  calculateOffsets();

  //draw the display
  displayAttitude();
  drawPitchScale();
}

void displayAttitude()
{
  myTOLED.pCurrentWindow = &horizonWindow;
  
  myTOLED.lineClear(xCenter - lastXOffset, 32 - lastYOffset, xCenter + lastXOffset , 32 + lastYOffset);
  
  myTOLED.lineSet(xCenter - xOffset, 32 - yOffset, xCenter + xOffset , 32 + yOffset);
  
  lastXOffset = xOffset;
  lastYOffset = yOffset;
}

void drawPitchScale()
{
  myTOLED.pCurrentWindow = &defaultWindow;
  
  myTOLED.lineClear(0, lastPitchOffset, pitchScaleLength, lastPitchOffset);
  myTOLED.lineClear(128 - pitchScaleLength, lastPitchOffset, 128, lastPitchOffset);
  
  myTOLED.lineSet(0, pitchOffset, pitchScaleLength, pitchOffset);
  myTOLED.lineSet(128 - pitchScaleLength, pitchOffset, 128, pitchOffset);

  lastPitchOffset = pitchOffset;
}

void calculateOffsets()
{
  //deal with this later
  pitchOffset = (windowHeight / 2) - pitch;

  float angleInRadians = (90 - roll) * RADIAN_CONVERSION;
//  Serial.print("Radians: ");
//  Serial.print(angleInRadians);
  
  xOffset = sin(angleInRadians) * (windowWidth / 2);
//  Serial.print(" xOffset: ");
//  Serial.print(xOffset);

  float tangent = tan(angleInRadians);

  //tan might be zero, don't divide by zero
  if (tangent != 0)
    if(xOffset != 0)
      yOffset = xOffset / tangent;
    else
      yOffset = windowHeight / 2;
    
//  Serial.print(" yOffset: ");
//  Serial.println(yOffset);
}

void getAttitude()
{
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;

  // Get new data samples
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx, gy, gz,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  roll = filter.getRoll();
  pitch = filter.getPitch();
  heading = filter.getYaw();

//  Serial.print("Orientation: ");
//  Serial.print(heading);
//  Serial.print(" ");
//  Serial.print(pitch);
//  Serial.print(" ");
//  Serial.println(roll);
}

void initSensors()
{
  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
  
  // Filter expects x samples per second
  filter.begin(5);
}
