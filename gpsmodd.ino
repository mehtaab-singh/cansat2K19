//Adafruit and TinyGPS libraries are available online. You need to manually install them or else the code will throw errors
//Adafruit: https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/arduino-test
//TinyGPS : http://arduiniana.org/libraries/tinygpsplus/
//Simply download the Zip file in both cases. In arduino IDE, go to Sketch -> Include Library ->Add .Zip Library and select the zip file.
#include<Wire.h>
#include<SPI.h>
#include <Adafruit_BMP280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

String TeamID = "2196";

unsigned long MTime = 0;   //Mission Time
unsigned long PCount = 0;  //Packet Count 

//BMP Variables
float Alti;   //in meters
float Press;  //in kPa
float Temp;   //in *C
Adafruit_BMP280 bme;

//Voltage Sensor Variables
float Volt;   //in Volts

//GPS Variables

//GPS Time zone is UTC. Provide correction here to convert to local time in USA. 
//FYI, offset for India is 5:30
int hourOffset = 5;  
int minOffset  = 30;

double GPS_Lat = 0.0;
double GPS_Long = 0.0;
double GPS_Alt = 0.0;
int GPS_Sats = 0;    //No. of satellites connected
int GPS_Hr = 0;
int GPS_Min = 0;
int GPS_Sec = 0;
TinyGPSPlus gps;
SoftwareSerial ss(4, 3); //Rx, Tx

//Tilt Variables
int TiltX = 0;
int TiltY = 0;
int TiltZ = 0;
int CS1 = 10;
char POWER_CTL = 0x2D;     //Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32; //X-Axis Data 0
char DATAX1 = 0x33; //X-Axis Data 1
char DATAY0 = 0x34; //Y-Axis Data 0
char DATAY1 = 0x35; //Y-Axis Data 1
char DATAZ0 = 0x36; //Z-Axis Data 0
char DATAZ1 = 0x37; //Z-Axis Data 1
char values[10];

String SoftState = "alpha"; //Current state of software, i.e, boot, idle, launch detect, deploy.

String Telemetry_Pkt = "";  //This the comma separated string that will be transmitted to ground station.

unsigned long Curr, Prev;

//------------------------SETUP----------------------------
void setup() {
  Serial.begin(115200);
  
  ss.begin(9600);      //Begin serial communication with GPS module

  bme.begin();
  
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, HIGH);
  //Turn on the ADXL345
  writeRegister(DATA_FORMAT, 0x01, CS1);
  writeRegister(POWER_CTL, 0x08, CS1); //Measurement mode

  Prev = millis();
}



//------------------------MAIN-----------------------------
void loop() {
 /* //BMP Sensor readings
  Press = bme.readPressure() * 0.001;  //kPa
  Temp  = bme.readTemperature();      //in *C
  Alti  = bme.readAltitude(1013.25);  //meters 

  //Dummy value of volatge. Aditya, make the voltage divider and implement a simple function to read ADC value and convert it to voltage.
  //Volt = VoltRead();
  Volt = 0.0;
*/
  while (ss.available() > 0)
  {
    gps.encode(ss.read());
    if (gps.location.isUpdated())
    {
    //GPS Lat, Long, Alt, Sats
    GPS_Lat  = gps.location.lat();
    GPS_Long = gps.location.lng();
    GPS_Alt  = gps.altitude.meters();
    GPS_Sats = gps.satellites.value();

    //GPS Time. Make sure the offset variables hourOffset & minOffset are correct.
    GPS_Hr  = gps.time.hour();
    GPS_Min = gps.time.minute();
    GPS_Sec = gps.time.second();
    GPS_Correct_Offset();
    }
  }
   //gps.encode(ss.read());
  Serial.println(gps.location.lat());
  Serial.println(gps.location.lng());
  Serial.println(gps.altitude.meters());
   Serial.println(gps.satellites.value());
  delay(1000);
  //Tilt x, Y, Z
/*  readRegister(DATAX0, 6, values, CS1);
  TiltX = ((int)values[1] << 8) | (int)values[0];
  TiltY = ((int)values[3] << 8) | (int)values[2];
  TiltZ = ((int)values[5] << 8) | (int)values[4];

  Curr = millis();
  if(Curr - Prev >= 1000)
  {
    MTime ++;
    PCount++;    //Not Correct
    Prev = Curr;
    Make_Telemetry_Pkt();
    //Send_Telemetry_Pkt();   //To be filled by Harish and Praneeth
    Serial.print(Telemetry_Pkt); //Just for testing. Remove in final version
  }*/
}





//----------------------VoltRead---------------------------
//Function to read voltage level of the battery
//Input :  None
//Return:  None
void VoltRead()
{
  //TO BE FILLED BY ADITYA
  //Read value from ADC port
  //Convert to voltage
  //Use voltage divider equation to get the battery voltage
  //Update the global variable Volt
}


//--------------------GPS_Correct_Offset-------------------
//Function to correct time offset of GPS
//Input :  None
//Return:  None
void GPS_Correct_Offset()
{
  GPS_Hr=GPS_Hr+5;
  GPS_Min=GPS_Min+30;
  if(GPS_Hr>=24)
    GPS_Hr=GPS_Hr-24;
  if(GPS_Min>=60)
  {
    GPS_Min=GPS_Min-60;
    GPS_Hr++;
  }
}


//----------------------writeRegister-----------------------
//Function to write to the register of Tilt Sensor
//Input :  reg address, value and Chip Select
//Return:  None
void writeRegister(char registerAddress, char value, int CS) 
{
  digitalWrite(CS, LOW);
  SPI.transfer(registerAddress);
  SPI.transfer(value);
  digitalWrite(CS, HIGH);
}


//----------------------readRegister-----------------------
//Function to read from the data register of Tilt Sensor
//Input :  reg address, no. of bytes, values and Chip Select
//Return:  None
void readRegister(char registerAddress, int numBytes, char *values, int CS) 
{
  char address = 0x80 | registerAddress;
  if (numBytes > 1)address = address | 0x40;

  digitalWrite(CS, LOW);
   SPI.transfer(address);
  for (int i = 0; i < numBytes; i++)
  {
    values[i] = SPI.transfer(0x00);
  }
   digitalWrite(CS, HIGH);
}


//--------------------Make_Telemetry_Pkt--------------------
//Function to make a string packet with all the telemetry variables
//Input :  None
//Return:  None
void Make_Telemetry_Pkt()
{
  Telemetry_Pkt = "";
  Telemetry_Pkt = TeamID + "," + String(MTime) + "," + String(PCount) + "," + String(Alti) + "," + String(Press) + "," + String(Temp) + "," + String(Volt) + "," + String(GPS_Hr) + ":" + String(GPS_Min) + ":" + String(GPS_Sec) + "," + String(GPS_Lat) + "," + String (GPS_Long) + "," + String(GPS_Alt) + "," + String(GPS_Sats) + "," + String(TiltX) + "," + String(TiltY) + "," + String(TiltZ) + "," + SoftState + "\n";
      
}      


//--------------------Send_Telemetry_Pkt--------------------
//TO BE FILLED BY HARISH AND PRANEETH
//Function to transmit Telemetry_Pkt via Xbee
//Input :  None
//Return:  None
void Send_Telemetry_Pkt()
{

}
