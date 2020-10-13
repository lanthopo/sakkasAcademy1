#include <LiquidCrystal.h>
#include <dht.h>

#include "Seeed_BMP280.h"
#include <Wire.h>



#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>


//#include <Adafruit_Sensor.h>
//#include <Adafruit_BMP280.h>

//===========================================================================
//==========  GENARAL  VARIABLES  ===========================================
const int Send_B = 19;
const int Menu_B = 18;
int Send_B_State = 0; 
int Menu_B_State = 0; 


int loop_cnt                  =0;
int view_menu                 =0;
int run_one_time              =0;



int loop_cnt_time_to_send     =0;
int time_to_send              =120;
int time_to_send_cnt          =0;

int loop_cnt_time_to_check_network =0;


char lcd_buffer[18];
//===========================================================================
//==============  LCD VARIABLES  ===========================================
const int rs = A3, en = 6, d4 = 9, d5 = 8, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


//===========================================================================
//==============    GSM FONA      ===========================================
#define FONA_RX 10
#define FONA_TX 11
#define FONA_RST 4
char replybuffer[255];

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
// Use this for FONA 800 and 808s
Adafruit_FONA fona = Adafruit_FONA(FONA_RST);
// Use this one for FONA 3G
//Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;

int network_signal        = 0;


int network_status        = 0;
int GPRS_status          = 0;
int upload_data_status    =0;




//===========================================================================
//==============  DHT 11         ===========================================
dht DHT;
#define DHT11_PIN 7
int dht_rh =0;
int dht_temp =0;

//===========================================================================
//==============  SOIL MOISTURE SENSOR   ====================================
int val = 0; 
int soilPin = A0;
int soil_persentage =0;

//===========================================================================
//==============  BMP 280                ====================================
BMP280 bmp280;
//Adafruit_BMP280 bmp; 

float BMP280_pressure_pa;
float BMP280_pressure_hpa;

float BMP280_temp;
float BMP280_altitude;

//===========================================================================
//==============  GUVA-S12SD UV Sensor    ====================================
float sensorVoltage; 
float sensorValue;

//===========================================================================
//==============  flame Sensor    ====================================
int flame_status         = 0;
int flame_sensorValue;





void setup() 
{
pinMode(Send_B, INPUT);
pinMode(Menu_B, INPUT);
  
digitalWrite(Send_B, HIGH);
digitalWrite(Menu_B, HIGH);


lcd.begin(16, 2);
lcd.setCursor(0, 0);
lcd.print("EKPEDEYT.  SAKKA");


while (!Serial); 
Serial.begin(9600);
fonaSerial->begin(4800);
if (! fona.begin(*fonaSerial)) 
                              {
                              Serial.println(F("Couldn't find GSM MODULE"));
                              while (1);
                              }
type = fona.type();
Serial.println(F("FONA is OK"));
Serial.print(F("Found "));
switch (type) 
            {
            case FONA800L:
            Serial.println(F(" 800L")); break;
            case FONA800H:
            Serial.println(F(" 800H")); break;
            case FONA808_V1:
            Serial.println(F(" 808 (v1)")); break;
            case FONA808_V2:
            Serial.println(F(" 808 (v2)")); break;
            case FONA3G_A:
            Serial.println(F(" 3G (American)")); break;
            case FONA3G_E:
            Serial.println(F(" 3G (European)")); break;
            default: 
            Serial.println(F("???")); break;
            }
fona.setGPRSNetworkSettings(F("internet"), F(""), F(""));

//=================================================================================
//      LCD INIT
//=================================================================================
lcd.setCursor(0, 0);
lcd.print("EKPEDEYT.  SAKKA");
lcd.setCursor(0, 1);
lcd.print(" Hackathon 2018 ");
Serial.println(F("z1"));
if(!bmp280.init())
{
    Serial.println("Device error!");
}
Serial.println(F("z2"));
delay(2000);
lcd.clear();
loop_cnt_time_to_check_network =5000;
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//DHT
void get_dht11()
{
int chk = DHT.read11(DHT11_PIN);

dht_rh =(int)DHT.humidity;
dht_temp =(int)DHT.temperature; 
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//moisture
void get_moisture_analog_sensor()
{
soil_persentage = analogRead(soilPin);
// calibration in percentage
soil_persentage = soil_persentage /3.8;
if(soil_persentage >=100) {soil_persentage = 100;}
}


void get_bmp280()
{
BMP280_pressure_pa          = bmp280.getPressure();
BMP280_pressure_hpa         = BMP280_pressure_pa /100;

BMP280_temp                 = bmp280.getTemperature();
BMP280_altitude             = bmp280.calcAltitude(BMP280_pressure_pa); 
}


void get_UV_Sensor()
{
sensorValue = analogRead(A2);
sensorVoltage = sensorValue/1024*3.3; 
}


void get_flame_Sensor()
{
flame_sensorValue = analogRead(A1);

if(flame_sensorValue<=100) {flame_status =1;}
else {flame_status=0;}
}


void Gsm_check_status()
{
uint8_t n = fona.getNetworkStatus();

network_status = n;
Serial.print(F("Network status "));
Serial.print(n);
Serial.print(F(": "));
if (n == 0) Serial.println(F("Not registered"));
if (n == 1) Serial.println(F("Registered (home)"));
if (n == 2) Serial.println(F("Not registered (searching)"));
if (n == 3) Serial.println(F("Denied"));
if (n == 4) Serial.println(F("Unknown"));
if (n == 5) Serial.println(F("Registered roaming")); 
}


void Gsm_check_signal()
{
uint8_t n = fona.getRSSI();
int8_t r;

Serial.print(F("RSSI = ")); Serial.print(n); Serial.print(": ");
if (n == 0) r = -115;
if (n == 1) r = -111;
if (n == 31) r = -52;
if ((n >= 2) && (n <= 30)) 
{
r = map(n, 2, 30, -110, -54);
}
network_signal = (int)n*100/30;
Serial.print(r); Serial.println(F(" dBm"));  
}


void activate_gprs()
{
uint16_t returncode;

Serial.print("\r\n"); 

if (!fona.getGSMLoc(&returncode, replybuffer, 250))
Serial.println(F("Failed!"));
if (returncode == 0) 
                    {
                    Serial.println(replybuffer);
                    GPRS_status=1;
                    } 
else 
                    {
                    Serial.print(F("Fail code #")); Serial.println(returncode);
                    if(!fona.enableGPRS(true))
                    Serial.println(F("Failed to turn on"));
                    
                    GPRS_status=0;
                    }

Serial.print("\r\n"); 
Serial.print("\r\n"); 

}

void upload_data()
{
uint16_t statuscode;
int16_t length;
char url[80];
upload_data_status =0;

//sprintf(url,"d-tel.eu/sakkas_pcb_files/pcb_upload.php?data=$50_1212_1254_25_55_0_55_8.8.8.8*41");
sprintf(url,"d-tel.eu/sakkas_pcb_files/pcb_upload.php?data=%d_%d_%d_%d_%d_%d_%d",(int)soil_persentage,(int)sensorValue,(int)BMP280_pressure_hpa,(int)BMP280_temp,(int)dht_rh,(int)flame_status,(int)network_signal);
Serial.println(url);

Serial.println(F("****"));
if (!fona.HTTP_GET_start(url, &statuscode, (uint16_t *)&length)) 
                                                                  {
                                                                  Serial.println("Failed!");
                                                                  upload_data_status = 2;
                                                                  }
Serial.println(statuscode);
if(statuscode == 200) {upload_data_status = 1;} else {upload_data_status = statuscode;}
while (length > 0) 
                    {
                    while (fona.available()) 
                                            {
                                            char c = fona.read();
                                            
                                            // Serial.write is too slow, we'll write directly to Serial register!
                                            #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
                                            loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
                                            UDR0 = c;
                                            #else
                                            Serial.write(c);
                                            #endif
                                            length--;
                                            if (! length) break;
                                            }
                    }
Serial.println(F("\n****"));
fona.HTTP_GET_end();
}


void loop() 
{
 // Serial.println(F("loop"));
get_dht11();  
get_moisture_analog_sensor();
get_bmp280();
get_UV_Sensor();
get_flame_Sensor();





if(flame_status==1) {time_to_send_cnt=time_to_send;}

//======================================================================================================================
//======================================================================================================================
//          
//======================================================================================================================
//======================================================================================================================

/*
loop_cnt++;
if(loop_cnt>=80) 
                  {
                  loop_cnt=0;
                  view_menu ++;
                  
                  if(view_menu >=5) {view_menu =0;}
                  run_one_time = 1;

                  //view_menu =2;
                  
                  }

*/


                  

loop_cnt_time_to_send++;
if(loop_cnt_time_to_send >= 14)
                              {
                              loop_cnt_time_to_send =0;
                              time_to_send_cnt++;
                              if(time_to_send_cnt>=time_to_send) 
                                                                {
                                                                time_to_send_cnt = 0;
                                                                lcd.clear();
                                                                lcd.setCursor(0, 0);
                                                                lcd.print("  Upload  Data  ");
                                                                
                                                                upload_data();


                                                                lcd.setCursor(0, 1);
                                                                if(upload_data_status ==1) {lcd.print(" Upload  OK!!!! ");}
                                                                else
                                                                {lcd.print("Error : "); lcd.print(upload_data_status);}
                                                                delay(2200);
                                                                
                                                                }
                               
                              }


loop_cnt_time_to_check_network++;
if(loop_cnt_time_to_check_network>=350)
                                      {
                                      loop_cnt_time_to_check_network =0;
                                      Gsm_check_status();
                                      if(network_status == 1)
                                                            {
                                                            lcd.clear();
                                                            lcd.setCursor(0, 0);
                                                            lcd.print(" Check  Network ");
                                                            lcd.setCursor(0, 1);
                                                            lcd.print("    Conection   ");
                                                            Gsm_check_signal();
                                                            
                                                            activate_gprs();
                                                            }


                                        
                                      }




//=========================================================================================
//=========================================================================================
//=========================================================================================
if(network_status != 1)
                        {
                        lcd.setCursor(0, 0); 
                        sprintf(lcd_buffer,"GPRS OFF SG=%03d%c",network_signal,'%');
                        lcd.print(lcd_buffer);
                        lcd.setCursor(0, 1);
                        lcd.print(" Try To Connect ");
                         //GPRS_status
                        }
else
if(network_status == 1)
                        {
                        lcd.setCursor(0, 0);
                        sprintf(lcd_buffer,"GPRS OK! SG=%03d%c",network_signal,'%');
                        lcd.print(lcd_buffer);

                        
                        if(digitalRead(Send_B) == LOW)  {time_to_send_cnt = 1000; Serial.println("\r\n SEND DATA BUTTON \r\n");}
                        
                        
                        if(digitalRead(Menu_B) == LOW)  
                                                      {
                                                      while(digitalRead(Menu_B) == LOW);  
                                                      view_menu ++;
                                                      
                                                      if(view_menu >=5) {view_menu =0;}
                                                      run_one_time = 1;  
                                                      
                                                      }






                        
                        //lcd.setCursor(13, 1);
                        //sprintf(lcd_buffer,"%03d",time_to_send_cnt);
                        //lcd.print(lcd_buffer);
                       // view_menu =5;
                        
                        if(view_menu == 0)
                                          {
                                          if(run_one_time == 1) {run_one_time = 0; lcd.setCursor(0, 1); lcd.print("                ");}
                                          lcd.setCursor(0, 1);
                                          sprintf(lcd_buffer,"RH=%03d%c      %03d",(int)dht_rh,'%',time_to_send_cnt);
                                          lcd.print(lcd_buffer);
                                          }
                        else
                        if(view_menu == 1)
                                          {
                                          if(run_one_time == 1) {run_one_time = 0;  lcd.setCursor(0, 1); lcd.print("                ");}
                                            lcd.setCursor(0, 1);
                                            sprintf(lcd_buffer,"SOIL=%03d%c    %03d",(int)soil_persentage,'%',time_to_send_cnt);
                                            lcd.print(lcd_buffer);
                                          }
                        else
                        if(view_menu == 2)
                                          {
                                          if(run_one_time == 1) {run_one_time = 0;   lcd.setCursor(0, 1); lcd.print("                ");}
                                          lcd.setCursor(0, 1); lcd.print(""); lcd.print(BMP280_pressure_hpa,2); lcd.print("Hpa    ");  
                                          sprintf(lcd_buffer,"%03d",time_to_send_cnt);
                                          lcd.setCursor(13, 1);
                                          lcd.print(lcd_buffer); 
                                          }
                        else
                        if(view_menu == 3)
                                          {
                                          if(run_one_time == 1) {run_one_time = 0;   lcd.setCursor(0, 1); lcd.print("                ");}

                                          lcd.setCursor(0, 1); lcd.print(""); lcd.print(BMP280_temp,2);lcd.print((char)176);lcd.print("C");  
                                          sprintf(lcd_buffer,"%03d",time_to_send_cnt);
                                          lcd.setCursor(13, 1);
                                          lcd.print(lcd_buffer); 
                                          }
                        else
                        if(view_menu == 4)
                                          {
                                          if(run_one_time == 1) {run_one_time = 0;  lcd.setCursor(0, 1); lcd.print("                ");}
                                          lcd.setCursor(0, 1);
                                          sprintf(lcd_buffer,"UV=%03dmJ/cm2 %03d",(int)sensorValue,time_to_send_cnt);
                                          lcd.print(lcd_buffer);
                                          }
                        else
                        if(view_menu == 5)
                                          {
                                          if(run_one_time == 1) {run_one_time = 0;  lcd.setCursor(0, 1); lcd.print("                ");}
                                          lcd.setCursor(0, 1);
                                          if(flame_status==1) {lcd.print(" Fire  Detected ");}
                                          else
                                          if(flame_status==0) {lcd.print("    No Fire     ");}
                                          }

                          
                        }








}
