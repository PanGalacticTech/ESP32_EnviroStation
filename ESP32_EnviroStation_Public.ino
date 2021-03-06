/************************************************************************************
    Copyright (c) January 2019, version 1.0     Paul van Haastrecht

    Version 1.1 Paul van Haastrecht
    - Changed the I2C information / setup.

    =========================  Highlevel description ================================

    This basic reading example sketch will connect to an SPS-30 and a BME280 for
    getting data and display the available data.

    =========================  Hardware connections =================================
    /////////////////////////////////////////////////////////////////////////////////
    ## UART UART UART UART UART UART UART UART UART UART UART UART UART UART UART  ##
    /////////////////////////////////////////////////////////////////////////////////

    Sucessfully test has been performed on an ESP32:

    Using serial port1, setting the RX-pin(25) and TX-pin(26)
    Different setup can be configured in the sketch.

    SPS30 pin     ESP32
    1 VCC -------- VUSB
    2 RX  -------- TX  pin 26
    3 TX  -------- RX  pin 25
    4 Select      (NOT CONNECTED)
    5 GND -------- GND

    Also successfully tested on Serial2 (default pins TX:17, RX: 16)
    NO level shifter is needed as the SPS30 is TTL 5V and LVTTL 3.3V compatible
    ..........................................................
    Successfully tested on ATMEGA2560
    Used SerialPort2. No need to set/change RX or TX pin
    SPS30 pin     ATMEGA
    1 VCC -------- 5V
    2 RX  -------- TX2  pin 16
    3 TX  -------- RX2  pin 17
    4 Select      (NOT CONNECTED)
    5 GND -------- GND

    Also tested on SerialPort1 and Serialport3 successfully
    .........................................................
    Failed testing on UNO
    Had to use softserial as there is not a separate serialport. But as the SPS30
    is only working on 115K the connection failed all the time with CRC errors.

    Not tested ESP8266
    As the power is only 3V3 (the SPS30 needs 5V)and one has to use softserial,
    I have not tested this.

    //////////////////////////////////////////////////////////////////////////////////
    ## I2C I2C I2C  I2C I2C I2C  I2C I2C I2C  I2C I2C I2C  I2C I2C I2C  I2C I2C I2C ##
    //////////////////////////////////////////////////////////////////////////////////
    NOTE 1:
    Depending on the Wire / I2C buffer size we might not be able to read all the values.
    The buffer size needed is at least 60 while on many boards this is set to 32. The driver
    will determine the buffer size and if less than 64 only the MASS values are returned.
    You can manually edit the Wire.h of your board to increase (if you memory is larg enough)
    One can check the expected number of bytes with the I2C_expect() call as in this example
    see detail document.

    NOTE 2:
    As documented in the datasheet, make sure to use external 10K pull-up resistor on
    both the SDA and SCL lines. Otherwise the communication with the sensor will fail random.

    ..........................................................
    Successfully tested on ESP32

    SPS30 pin     ESP32
    1 VCC -------- VUSB
    2 SDA -------- SDA (pin 21)
    3 SCL -------- SCL (pin 22)
    4 Select ----- GND (select I2c)
    5 GND -------- GND

    The pull-up resistors should be to 3V3
    ..........................................................
    Successfully tested on ATMEGA2560

    SPS30 pin     ATMEGA
    1 VCC -------- 5V
    2 SDA -------- SDA
    3 SCL -------- SCL
    4 Select ----- GND  (select I2c)
    5 GND -------- GND

    ..........................................................
    Successfully tested on UNO R3

    SPS30 pin     UNO
    1 VCC -------- 5V
    2 SDA -------- A4
    3 SCL -------- A5
    4 Select ----- GND  (select I2c)
    5 GND -------- GND

    When UNO-board is detected the UART code is excluded as that
    does not work on UNO and will save memory. Also some buffers
    reduced and the call to GetErrDescription() is removed to allow
    enough memory.
    ..........................................................
    Successfully tested on ESP8266

    SPS30 pin     External     ESP8266
    1 VCC -------- 5V
    2 SDA -----------------------SDA
    3 SCL -----------------------SCL
    4 Select ----- GND --------- GND  (select I2c)
    5 GND -------- GND --------- GND

    The pull-up resistors should be to 3V3 from the ESP8266.

    ===============  BME280 sensor =========================
    BME280
    VCC  ------ VCC  (3V3 or 5V depending on board)
    GND  ------ GND
    SCK  ------ SCL
    SDI  ------ SDA

    ================================= PARAMETERS =====================================

    From line 149 there are configuration parameters for the program

    ================================== SOFTWARE ======================================
    Sparkfun ESP32

      Make sure :
        - To select the Sparkfun ESP32 thing board before compiling
        - The serial monitor is NOT active (will cause upload errors)
        - Press GPIO 0 switch during connecting after compile to start upload to the board

    Sparkfun BME280 library : https://github.com/sparkfun/SparkFun_BME280_Arduino_Library

    ================================ Disclaimer ======================================
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    ===================================================================================

    NO support, delivered as is, have fun, good luck !!
*/

#include "sps30.h"
#include "SparkFunBME280.h"
#include "InfluxDb.h"

// include library to read and write from flash memory
#include <EEPROM.h>

// define the number of bytes you want to access
#define EEPROM_SIZE 1



///////////////////////////////////////////////////// INFLUX DB MODS////////////////////////////////////

define INFLUXDB_HOST "XXX.XXX.XXX.XXX"     // Set to Static IP of DataBase Server


#define INFLUXDB_USER "admin"      // default change to match your influx db install
#define INFLUXDB_PASS "admin"

//#define WIFI_SSID "SSID"
//#define WIFI_PASS "WIFI"

// Home Network
#define WIFI_SSID "SSID"       // provide your network credentials
#define WIFI_PASS "PASSWORD"

// Phone Hotspot
#define ELMO_SSID ""            // alt values could be used as backup
#define ELMO_PASS ""

bool wifiHome = true;  // if true uses Home Network // if false uses ELMO Network credentials

WiFiClient client;
//MHZ19 myMHZ19;        // no CO2 Sensory
//HTU21D myHumidity;     // Humidity handled by BME280

unsigned long getDataTimer = 0;

// ########################## Sensor Values to Send to Influx ##########
//~~~~~~~~~~~~~~~~~~~~~~~~~~SPS30~~~~~~~~~~~~~~~
float PM25value = 0;
float PM10value = 0;
float  PM25num = 0;
float  PM10num = 0;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~BME280~~~~~~~~~~~~~~~~~~~~~~~
float pressure = 0;
float humd;
float temp;


float tempOffset = 2.50;   // Fudge value for temp offset. More testing to find a better solution or closer offset. Probably between 2.00 and 3.00 for now


bool notFirstTime = false;
bool firstTime = true;

int CO2;   // Buffer for CO2
int flipData = 0;

float last_humd;
float last_temp;

Influxdb influx(INFLUXDB_HOST);

/////////////////////////////////////////////// Wifi Copied in from working Sketch /////////////////////////////

/// Depreciated Section but might be useful later.
//boolean setAccessPoint = false;    // if true set up as Access Point

//boolean setLocalNetwork = true;     // if true set up as Local Network Station

//char wifiType[] = {"Local"};  // char array set to Local or to AP?


// Replace with your network credentials for Local Network
const char* Localssid     = "SSID";  //Wifi Name  - SSID
const char* Localpassword = "PASS"; //Router Password - PASSWORD

// These credentials are used if ESP32 set up as an access point
const char* APssid     = "ESP32-wifi-dev";              //Wifi Name  - SSID
const char* APpassword = "12345678";                  //Router Password - PASSWORD       // Minimum 8 Characters

//WiFiUDP udp;    // disabled line to use ESP board on local networ


/*
  // Optional Set Static IP Lines
  // Set your Static IP address
  IPAddress local_IP(192, 168, 1, 10);
  // Set your Gateway IP address
  IPAddress gateway(192, 168, 1, 254);

  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);   //optional
  IPAddress secondaryDNS(8, 8, 4, 4); //optional

*/

//boolean setStaticIP = true; // If true sets static IP address when connecting to Local Network
//if false uses router DNS service.

// if softAP set
//wiFi.softAPIP - This defaults to |192.168.4.1|

// Set web server port number to 80
//WiFiServer server(80);
/////////////////////////////////////////END OF DEPRECIATED VALUES//////////////////////////////////////////



////////////////////////////////////////////////////////////////////////SPS & BME ORIGONAL STUFF//////////////////////////////////////
/////////////////////////////////////////////////////////////
/*define communication channel to use for SPS30
  valid options:
     I2C_COMMS              use I2C communication
     SOFTWARE_SERIAL        Arduino variants (NOTE)
     SERIALPORT             ONLY IF there is NO monitor attached
     SERIALPORT1            Arduino MEGA2560, Sparkfun ESP32 Thing : MUST define new pins as defaults are used for flash memory)
     SERIALPORT2            Arduino MEGA2560 and ESP32
     SERIALPORT3            Arduino MEGA2560 only for now

   NOTE: Softserial has been left in as an option, but as the SPS30 is only
   working on 115K the connection will probably NOT work on any device. */
/////////////////////////////////////////////////////////////
#define SP30_COMMS I2C_COMMS

/////////////////////////////////////////////////////////////
/* define RX and TX pin for softserial and Serial1 on ESP32
   can be set to zero if not applicable / needed           */
/////////////////////////////////////////////////////////////
#define TX_PIN 26
#define RX_PIN 25

/////////////////////////////////////////////////////////////
/* define driver debug
   0 : no messages
   1 : request sending and receiving
   2 : request sending and receiving + show protocol errors */
//////////////////////////////////////////////////////////////
#define DEBUG 0

///////////////////////////////////////////////////////////////
//                          BME280                           //
///////////////////////////////////////////////////////////////
/* define the BME280 address.
   Use if address jumper is closed (SDO - GND) : 0x76.*/
#define I2CADDR 0x76

/* Define reading in Fahrenheit or Celsius
    1 = Celsius
    0 = Fahrenheit */
#define TEMP_TYPE 1

/* define whether hight Meters or Foot
    1 = Meters
    0 = Foot */
#define BME_HIGHT 1

//#define SEALEVELPRESSURE_HPA (1013.25)   // standard
#define SEALEVELPRESSURE_HPA (1030)   // lydd airport 3m
//#define SEALEVELPRESSURE_HPA (1029)   // heathrow airport 25m

///////////////////////////////////////////////////////////////
/////////// NO CHANGES BEYOND THIS POINT NEEDED ///////////////
///////////////////////////////////////////////////////////////

// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_all();

// create constructors
SPS30 sps30;
BME280 mySensor; //Global sensor object

/*
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme; // alternative bmeobject
*/

// status
bool detect_BME280 = false;

int acc =  0;  // used to trigger reprinting of header on serial output

byte loopNumber = 0;  // simple loopcounter to count loops to ensure not going mad


bool performCleanNow = false;



void setup() {

  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);


  ///////////////////////////// Added From Envirostation //////////////////////////
  Serial.println(F("Trying to connect to WiFi"));

  //////////////////////////////////////////////////////////////////////////////////////BUG FIX POWER UP WIFI ISSUE??????????????????????????????
  WiFi.disconnect(true);                     // disconnects STA Mode
  delay(1000);
  WiFi.softAPdisconnect(true);           // disconnects AP Mode
  delay(1000);
  /////////////////////////////////////////////////////////////////////////////////////////
  delay(2000);  // delay for power issue during wifi startup // decoupling cap added to board as well.

  if (wifiHome) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);    // still needs function for IP address change
  } else {
    WiFi.begin(ELMO_SSID, ELMO_PASS);
  }

  delay(1000);

  int wificheck = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    if ((millis() - wificheck) > 10000)
    {
      Serial.print("....RESETTING!");
      ESP.restart();;
    }
  }
  Serial.println("WiFi Connected Successfully");


  influx.setDbAuth("envirostation", INFLUXDB_USER, INFLUXDB_PASS);

  ///////////////////////////// Added From Envirostation //////////////////////////


  // serialTrigger("SPS30-Example5: Basic reading + BME280. press <enter> to start");

  Serial.println(F("Trying to connect Sensors"));

  // set driver debug level
  sps30.EnableDebugging(DEBUG);

  // set pins to use for softserial and Serial1 on ESP32
  if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN, TX_PIN);

  // Begin communication channel;
  if (sps30.begin(SP30_COMMS) == false) {
    Errorloop("could not initialize communication channel.", 0);
  }

  // check for SPS30 connection
  if (sps30.probe() == false) {
    Errorloop("could not probe / connect with SPS30", 0);
  }
  else
    Serial.println(F("Detected SPS30"));

  // reset SPS30 connection
  if (sps30.reset() == false) {
    Errorloop("could not reset", 0);
  }

  // read device info
  GetDeviceInfo();

  // set BME280 I2C address.
  mySensor.setI2CAddress(I2CADDR);

  if (mySensor.beginI2C() == false) // Begin communication over I2C
    Serial.println("The BME280 did not respond. Please check wiring.");
  else
  {
    detect_BME280 = true;
    Serial.println(F("Detected BME280"));
  }

  // start measurement
  if (sps30.start() == true)
    Serial.println(F("Measurement started"));
  else
    Errorloop("Could NOT start measurement", 0);

  // serialTrigger("Hit <enter> to continue reading");

  if (SP30_COMMS == I2C_COMMS) {
    if (sps30.I2C_expect() == 4)
      Serial.println(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available !!! \n"));
  }



  recallInterval();    // recall amount of hours passed since last autoclean incase of shutdown
  // if clean now requested
  autoClean();




  //////////////////////////////////////////// Added from Envirostation/////////////////////////////

  //  humd = myHumidity.readHumidity();
  //      temp = myHumidity.readTemperature();


  //humd = (mySensor.readFloatHumidity(), 1);           /// changed these lines to work with the objects created
  //  temp = (mySensor.readTempC(), 2);
  humd = mySensor.readFloatHumidity();           /// changed these lines to work with the objects created
  temp = mySensor.readTempC();

  last_humd = humd;
  last_temp = temp;




  //////////////////////////////////////////// Added from Envirostation/////////////////////////////

}




void loop() {
  read_all();
  postData();   // post data to influx db
  delay(60000);

  // tessting to see where program is hanging up
  Serial.println("Loop Still Running...");
  Serial.print("Loop: ");
  Serial.println(loopNumber);
  loopNumber++;


  saveInterval();   // saves a byte every hour to track hours between fan autocleans. Triggers autoclean if alloted time has passed.
  autoClean();   // triggers autoclean if performCleanNow is true
}




/**
   @brief : read and display device info
*/
void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    Serial.print(F("\tSerial number : "));
    if (strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess("could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("\tProduct name  : "));

    if (strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess("could not get product name.", ret);

  // try to get article code
  ret = sps30.GetArticleCode(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("\tArticle code  : "));

    if (strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess("could not get Article code .", ret);
}








/**
   @brief : read and display all values
*/
bool read_all()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH) {

      if (error_cnt++ > 3) {
        ErrtoMess("Error during reading values: ", ret);
        return (false);
      }
      delay(1000);
    }

    // if other error
    else if (ret != ERR_OK) {
      ErrtoMess("Error during reading values: ", ret);
      return (false);
    }

  } while (ret != ERR_OK);

  // ~~~~~~~~~~~~~~~~Header Reprinter~~~~~~~~~~~~~~~
  acc++;

  if (acc > 30 ) {
    header = true;
    acc = 0;
  }
  // ~~~~~~~~~~~~~~~~Header Reprinter~~~~~~~~~~~~~~~

  // only print header first time
  if (header) {

    Serial.print(F("==================================== SPS30 ====================================="));
    if (detect_BME280) Serial.print(F(" ================ BME280 =============="));

    Serial.print(F("\n-------------Mass -----------    ------------- Number --------------   -Average-"));
    if (detect_BME280) Serial.print(F(" Pressure Humidity Altitude Temperature"));

    Serial.print(F("\n     Concentration [μg/m3]             Concentration [#/cm3]             [μm]"));
    if (detect_BME280) {
      Serial.print(F("     [hPa]\t    [%]      "));

      if (BME_HIGHT) Serial.print(F("Meter\t"));
      else Serial.print(F("Foot\t"));

      if (TEMP_TYPE) Serial.print(F("[*C]"));
      else Serial.print(F("[*F]"));
    }

    Serial.println(F("\nPM1.0\tPM2.5\tPM4.0\tPM10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPrtSize\n"));

    header = false;
  }

  Serial.print(val.MassPM1);
  Serial.print(F("\t"));
  Serial.print(val.MassPM2);
  Serial.print(F("\t"));
  Serial.print(val.MassPM4);
  Serial.print(F("\t"));
  Serial.print(val.MassPM10);
  Serial.print(F("\t"));
  Serial.print(val.NumPM0);
  Serial.print(F("\t"));
  Serial.print(val.NumPM1);
  Serial.print(F("\t"));
  Serial.print(val.NumPM2);
  Serial.print(F("\t"));
  Serial.print(val.NumPM4);
  Serial.print(F("\t"));
  Serial.print(val.NumPM10);
  Serial.print(F("\t"));
  Serial.print(val.PartSize);

  PM25value = val.MassPM2;   // added lines here
  PM10value = val.MassPM10;  // added lines here

  if (detect_BME280) {
    
     Serial.print(F("\t  "));
    pressure = (mySensor.readFloatPressure() / 100); //  I dont know why this is divided by 100 but okay
               //   Serial.print(mySensor.readFloatPressure() / 100, 0);
               Serial.print(pressure, 0);   // (value, no of .Points);

    Serial.print(F("\t   "));

    humd =  mySensor.readFloatHumidity() ;
    //   Serial.print(mySensor.readFloatHumidity(), 1);
    Serial.print(humd, 1);

    Serial.print(F("\t     "));


    if (BME_HIGHT) Serial.print(mySensor.readFloatAltitudeMeters(), 1);
    else Serial.print(mySensor.readFloatAltitudeFeet(), 1);
    Serial.print(F("\t"));


    temp = (mySensor.readTempC()- tempOffset);   // offset value a fudge for now

    //   if (TEMP_TYPE)  Serial.print(mySensor.readTempC(), 2);

    if (TEMP_TYPE)  Serial.print(temp, 2);
    else Serial.print(mySensor.readTempF(), 2);

  }

  Serial.print(F("\n"));

}





/**
    @brief : continued loop after fatal error
    @param mess : message to display
    @param r : error code

    if r is zero, it will only display the message
*/
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for (;;) delay(100000);
}







/**
    @brief : display error message
    @param mess : message to display
    @param r : error code

*/
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

/**
   serialTrigger prints repeated message, then waits for enter
   to come in from the serial port.
*/
void serialTrigger(char * mess)
{
  Serial.println();

  while (!Serial.available()) {
    Serial.println(mess);
    delay(2000);
  }

  while (Serial.available())
    Serial.read();
}







// ##########################################################################################
// ##########################################################################################
void postData()
{

  InfluxData row("temperature");
  row.addTag("location", "southdarenth");
  row.addValue("value", temp);
  // influx.write(row);

  InfluxData row2("humidity");
  row2.addTag("location", "southdarenth");
  row2.addValue("value", humd);
  //  influx.write(row2);



  InfluxData row3("PM25");
  row3.addTag("location", "southdarenth");
  row3.addValue("value", PM25value);
  //  influx.write(row3);

  //Test Line
  // Serial.print("PM2.5value: ");
  //   Serial.print(PM25value);

  InfluxData row4("PM10");
  row4.addTag("location", "southdarenth");
  row4.addValue("value", PM10value);
  //  influx.write(row4);
  //Test Line
  // Serial.print("PM10value: ");
  //  Serial.print(PM10value);


  InfluxData row5("pressure");
  row5.addTag("location", "southdarenth");
  row5.addValue("value", pressure);
 

  influx.prepare(row);
  influx.prepare(row2);
  influx.prepare(row3);
  influx.prepare(row4);
  influx.prepare(row5);

  influx.write();

  Serial.println("POSTED");

  //  client.stop(); // ?!?!?! who knows seems to work without atm.


}



/*

  void altBMEsetup() {
  unsigned status;

  // default settings
  status = bme.begin();
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(), 16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();


  }

  void altBMEvalues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
  }

*/
