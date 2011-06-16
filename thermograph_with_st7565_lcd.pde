/*

  This sketch measures the temperature using a Maxim DS18B20 digital
  thermometer and graphs the result on an ST7565 128x64 LCD screen.
  
  The scaling used plots temperatures between +/- 32C.
  
  The DS18B20 uses the Maxim 1-Wire protocol to communicate with the 
  Arduino
  
  Assumes there is a single DS18B20 connected to the one-wire bus
  
  Uses Dallas Temperature library from 
  http://www.milesburton.com/?title=Dallas_Temperature_Control_Library

--
Copyright (C) 2010 by Integrated Mapping Ltd

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/


#include <OneWire.h>
#include <DallasTemperature.h>
#include "ST7565.h"

// how often to update the graph in milliseconds
#define UPDATE_PERIOD 120000L

// arduino pins that lcd SID, SCLK, A0, RST, CS are connected to
ST7565 glcd(9, 8, 7, 6, 5);

#define LCD_WIDTH 128
#define LCD_HEIGHT 64

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress theThermometer;

 
unsigned long startTime;
unsigned long ct = 0;

// the graph starts at the left edge - x-axis is time
int graphX0 = 0;

// the graph is offset down by this many pixels to create
// a space for the text at the top
int graphY0 = 9;

// this is the centre of the graph - the y-axis at y = 0
int graphyYc = graphY0 + (LCD_HEIGHT-graphY0)/2;

 
/*----------------------------------------------------------------*/
void setup(void) {

  // initialize inputs/outputs
  // start serial port
  Serial.begin(9600);
  
  // init the lcd
  glcd.st7565_init();
  glcd.st7565_command(CMD_DISPLAY_ON);
  glcd.st7565_command(CMD_SET_ALLPTS_NORMAL);
  glcd.st7565_set_brightness(0x15);

  // clear the screen
  glcd.clear();
  
  // update the screen. note that the lcd uses a buffer in the
  // Arduino memory. The glcd functions update that, the the 
  // glcd.display() function transfer the buffer to the lcd
  glcd.display(); 
  
  // the current Arduino time
  startTime = millis();
  
  // draw the axes on the lcd
  drawAxes();
  
  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");
  
  if (!sensors.getAddress(theThermometer, 0)) 
    Serial.println("Unable to find address for Device 0");   
  
  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(theThermometer, 12);
 
}
 
 
 
 
/*----------------------------------------------------------------*/
void loop(void) {
  
  int Whole, Fract;
  static int xPos = 0;
  
  byte i;
  byte present = 0;
  byte data[12];

  // Send the command to get temperatures
  sensors.requestTemperatures();
  
  // retrieve the temp
  float tempC = sensors.getTempC(theThermometer);
  
  // sprintf doesn't do floats, so split the whole and fractional parts out
  Whole = int(tempC); 
  Fract = (tempC - Whole) * 10;
  
  char buf[50];
  sprintf(buf, "%d.%01d ", Whole, Fract);
  
  Serial.println(buf);
  
  // update the graph every UPDATE_PERIOD milliseconds - all day to update
  if (millis() - startTime > UPDATE_PERIOD) {
        
    // erase the last time tick
    glcd.drawline(xPos, graphyYc, xPos, graphyYc-2, WHITE);
    
    ct++;
    xPos = ct % LCD_WIDTH;

    // clear the column that we are about to write into
    for (int i = graphY0;  i++;  i < LCD_HEIGHT) {
      glcd.setpixel(xPos,i, WHITE);  
    } 
  
    // draw the temperature value, then refresh the axis
    glcd.setpixel(xPos, int(graphyYc - tempC), BLACK);

    // draw the axes and y tick marks
    drawAxes();

    // put a tick on the x-axis to show where we are up to
    glcd.drawline(xPos, graphyYc, xPos, graphyYc-2, BLACK);
    
    startTime = millis();
  }
  
  // write the temperature on the top left of the screen every time round the
  // loop - every second or so
  glcd.drawstring(0,0, buf);
  glcd.display();
  
  delay(1000);
  
}




/*----------------------------------------------------------------*/
void drawAxes() {
  
  // draw a y-axis at temp=0, draw short 5-degree ticks and longer
  // 10 degree ticks up the left edge of the disply

  int tickLen;
  
  // y = 0 axis
  glcd.drawline(0, graphyYc, 127, graphyYc, BLACK);  
  
  // 5 degree ticks.  10 degree ticks are longer
  for (int i = -30; i < 26; i+=5) {
    
    if (i % 10 == 0)
      tickLen = 3;
    else
      tickLen = 1;
    
    glcd.drawline(0, graphyYc-i, tickLen, graphyYc-i, BLACK);  
  
  }
}


