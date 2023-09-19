/*          ____   ____  ___        _____   ___  ____    _____   ___ 
           |    \ |    |/   \      / ___/  /  _]|    \  / ___/  /  _]
           |  o  ) |  ||     |    (   \_  /  [_ |  _  |(   \_  /  [_ 
           |     | |  ||  O  |     \__  ||    _]|  |  | \__  ||    _]
           |  O  | |  ||     |     /  \ ||   [_ |  |  | /  \ ||   [_ 
           |     | |  ||     |     \    ||     ||  |  | \    ||     |
           |_____||____|\___/       \___||_____||__|__|  \___||_____|V5

/ / / / / / / / / / EECE244 Applications of Embedded Systems \ \ \ \ \ \ \ \ \ \
/ / / / /  Written by Jacob Seman & Lillian Tucker for the ESP32 Pico  \ \ \ \ \
                                   March 2023
                                 Functionality:
                        WiFi Access Point Implementation
           Read differential MPU data to detect involuntary contractions
                     Measure heartrate and oxygen percentage
                         Log data to SD card at 100Hz
                        Indicate read/write status LED
                      Webserver interface to start/stop
                     Utilize threading and hardware timers
\ \ \ \ \ \ \ \ \ \ \ \  NOTE: DO NOT RUN FROM USB HUB / / / / / / / / / / / / /
\ \ \ \ \ \ \ \ \ THE ESP32 WILL BROWNOUT ON LOW QUALITY HUBS  / / / / / / / /*/

#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESP32WebServer.h>
#include <ESPmDNS.h>
#include "CSS.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "esp_task_wdt.h" // to configure core tasks' watchdogs
#include "esp_timer.h"

#define DEF_ADDR 0x55
#define AVG_AMT  10
#define RESETPIN 4
#define MFIOPIN  2
#define DATAFREQ 100 // [Hz] modify here for data rate, 100Hz default
#define ISRSPEED 1000 // [1/seconds] timer divider, millis by default
#define servername "bioSense"

/** FAUXLIST: vector class is not defined for Arduino:
    - not all functions of vector type are needed here.
    - simple custom class works for now
    - this functions similarly to a list,
      by updating 'pos' and overwriting old values.
      this saves on time spent in loops while in data collection state.       */
class fauxList{
  float vals[AVG_AMT];
  int pos;
  public:
    // this factor should be updated at logging start:
      // used to scale logging data to 1.0 from the initial value
    float cxFact;
    fauxList(){
      for(int i=0;i<AVG_AMT;i++)vals[i]=0;
      pos=0;
      cxFact=1.0;
    }
    void pushback(float newVal){ // push to 'pos' in list
      vals[pos]=newVal; // replace next 'pos', it will always be oldest in list
      pos++; // update 'pos' in FIFO scheme, roll through whole list
      if(pos>=AVG_AMT)pos=0; // reset at end of cycle
    }
    float getAvg(){ // now only calculated on request and not every pushback
      float avg=0;
      for(int i=0;i<AVG_AMT;i++){ // get avg
        avg+=fabs(vals[i]);
      }
      avg/=AVG_AMT;
      avg*=cxFact;
      return avg;
    }
};

/** PROTOTYPES:*/
void initMPU();
float getAccelDiff();
void initDisplay();
void initSPO();
void initSD();
void initTimer();
void ICACHE_RAM_ATTR onTimerISR();
/** ONTIMERISR: hardware interrupt to set takeSample flag on data rate interval:
    - could alternatively look into micros() or microsISR() from "time.h"
      but running this function from RAM allows some addtional event control
    - overall logging state cycle time for all processes is fast anough to hit
      ~800Hz. when set for 1kHz, the cycle is usually 1ms, but longer when
      bioData reads occur, up to 9ms.
    - data resolution is likely sufficient at 100Hz, and when configured for
      this rate the ESP32 is able to consistently hit 10ms cycle times
      (plus/minus only 1ms), and this is corrected by retaining interval
      overruns in the ISRcount calculation.                                   */
void initThreading(); // start threads
/** INITTHREADING: threading is used to take advantage of the ESP32's two cores:
    - this allows splitting function calls between core #0 and #1, to reach 1kHz
    - note that the I2C bus should only be accessed by a single core
      and not both cores independently
    - else thread-unsafe behavior occurs,
      mainly corruption of the content pushed to the OLED display
    - this means that whatever core reads the MPU6050 sensors
      must also run the OLED display functions                                */
void sensorRead(void *param); // assigned to core #0
void sdWrite(void *param); // assigned to core #1
void handleFileUpload();
//void runFunc(); // function under test
//void measure_function(); // timing FUT for 5000 iterations

/** GLOBALVARIABLES:*/
int16_t acc1,acc2; // containers for accelerometer readings
uint8_t takeSample,loggingState; // flag set on DATAFREQ interval, logging state
uint fileNum,sampleRateMillis; // filename session number, etc.
ulong elapsedTime,previousTime,bioElapsed,bioPrevious,
      isrCount,isrElapsed,isrPrevious;
char nameBuffer[21],dataBuffer[29],countBuffer[3]; // buffers for snprintf
hw_timer_t *timer=NULL; // ISR timer
TaskHandle_t core0,core1; // threading handlers
fauxList accelArray; // averaging fauxList, reduces noise
File dataFile; // SD card file
Adafruit_SSD1306 display=Adafruit_SSD1306(128,32,&Wire); // OLED
SparkFun_Bio_Sensor_Hub bioHub(RESETPIN,MFIOPIN); // pulse oximeter
bioData body; // pulseox container

/*IPAddress local_ip(192,168,4,1);
IPAddress gateway(192,168,4,1);
IPAddress subnet(255,255,255,0);*/
ESP32WebServer server(80);

/** SETUP:*/
void setup(){
  Serial.begin(115200);
  setCpuFrequencyMhz(240); // maximum safe (default) CPU speed
  Serial.print("CPU freq set to ");
  Serial.print(getCpuFrequencyMhz());
  Serial.println(" MHz");
  Serial.print("setup() is running on core ");
  Serial.println(xPortGetCoreID());
  pinMode(26,INPUT_PULLUP); // init button
  pinMode(12,OUTPUT); // init write LED
  initTimer(); // begin ISR timer
  initThreading(); // begin threaded tasks
}

void loop(){} // not sure if arduino, or real-time operating system...unused.

/** INITMPU:*/
void initMPU(){
  Wire.begin(); // Initialize comunication
  Wire.setClock(400000L);
  Wire.beginTransmission(0x68); // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //end the transmission
  Wire.beginTransmission(0x69); // Start communication with MPU6050 // MPU=0x69
  Wire.write(0x6B); // Talk to the register 6B
  Wire.write(0x00); // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); //end the transmission
}

/** GETACCELDIFF: only read the y-axes from both MPUs, and store difference*/
float getAccelDiff(){
  Wire.beginTransmission(0x68);
  Wire.write(0x3D); // Start with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x68,(uint8_t)2,(uint8_t)true); // Read 2 registers
    // ***typecasting required to reduce compiler overload candidate guessing
  acc1=(Wire.read()<<8|Wire.read()); // Y-axis value from mpu1
  Wire.beginTransmission(0x69);
  Wire.write(0x3D); // Start with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)0x69,(uint8_t)2,(uint8_t)true); // Read 2 registers
    // ***typecasting required to reduce compiler overload candidate guessing
  acc2=(Wire.read()<<8|Wire.read()); // Y-axis value from mpu2
  return (acc2-acc1)/16384.0; // LSB/g
}

/** INITDISPLAY:*/
void initDisplay(){
  if (!display.begin(SSD1306_SWITCHCAPVCC,0x3C)) { // address 0x3C for OLED
    Serial.println("SSD1306 init failed");
    while(1)yield();
  }
  Serial.println("SSD1306 init complete");
  display.display();
  delay(500); // pause for 0.5 seconds to catch up
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.clearDisplay();
  display.display();
}

/** INITSPO:*/
void initSPO(){
  Wire.begin();
  int result=bioHub.begin();
  if(!result)Serial.println("biosensor init started...");
  else Serial.println("could not communicate with the sensor.");
  Serial.println("configuring biosensor...");
  int error=bioHub.configBpm(MODE_TWO); // Configuring BPM settings
  if(!error)Serial.println("biosensor configured.");
  else{
    Serial.println("error configuring sensor.");
    Serial.print("error: ");
    Serial.println(error);
  }
  delay(2000); // pause for 2 seconds to catch up
}

/** INITSD:*/
void initSD(){
  // SD card setup:
  Serial.println("initializing SD card...");
  if (!SD.begin(SS)){
    Serial.println("initialization failed.");
    while(1);
  }else Serial.println("SD init complete, card present");
   // print the type of card:
  Serial.print("card type: ");
  switch (SD.cardType()){
    case CARD_NONE:
      Serial.println("NONE");
      break;
    case CARD_MMC:
      Serial.println("MMC");
      break;
    case CARD_SD:
      Serial.println("SD");
      break;
    case CARD_SDHC:
      Serial.println("SDHC");
      break;
    default:
      Serial.println("Unknown");
  }
  // print the size of the first FAT-type volume:
  Serial.print("Card size: ");
  Serial.println((float)SD.cardSize()/1000);
  Serial.print("Total bytes: ");
  Serial.println(SD.totalBytes());
  Serial.print("Used bytes: ");
  Serial.println(SD.usedBytes());
  loggingState=0; // set state
  fileNum=0; // set data log file number
  //SD.end();
}

/** INITTIMER: set hardware clock division to be used by ISR*/
void initTimer(){
  sampleRateMillis=ISRSPEED/DATAFREQ;
  takeSample=0;
  isrCount=0;
	timer=timerBegin(0,20,true);
	timerAttachInterrupt(timer,&onTimerISR,true);
	timerAlarmWrite(timer,4000000/ISRSPEED,true); // 4MHz clock div data-frequency
  timerAlarmEnable(timer);
  Serial.print("ISR timer initialized at ");
  Serial.print(ISRSPEED);
  Serial.println(" Hz.");
}

/** ONTIMERISR: timer sets the flag at sample rate by clock division count*/
void ICACHE_RAM_ATTR onTimerISR(){
  isrCount++;
  isrElapsed=isrCount-isrPrevious;
  if(isrElapsed>=sampleRateMillis&&!takeSample){
    takeSample=1;
    // correction for millisecond losses over time:
    isrPrevious=isrCount;//-(isrElapsed-sampleRateMillis);
  }
}

/** INITTHREADING: pin functions to cores, run two loops in parallel*/
void initThreading(){
  // core 0, 10kbyte stack:
  xTaskCreatePinnedToCore(sensorRead,"sensorRead",10000,NULL,1,&core0,0);
  // core 1, 10kbyte stack:
  xTaskCreatePinnedToCore(sdWrite,"sdWrite",10000,NULL,1,&core1,1);
}

/** SENSORREAD: this function is a task set to run on core #0:
    - timing should be as fast as possible, this dictates sample resolution
    - fauxList averaging should be adjusted to reflect this                   */
void sensorRead(void *param){
  esp_task_wdt_init(30,false);
  Serial.print("sensorRead() is running on core ");
  Serial.println(xPortGetCoreID());
  // avoid crossing the I2C streams, init all I2C devices here:
  initMPU();
  initDisplay();
  initSPO();
  // for timing functions only:
  // measure_function();
  for(;;){ // task loop
    if(loggingState==2){ // first check if logging, this needs to execute fafb!
      bioElapsed=isrCount-bioPrevious; // update time since a bio read
      accelArray.pushback(getAccelDiff());
      // ^^^NOTE^^^ this takes 1150 MICROseconds
      if(bioElapsed>=ISRSPEED){ // update bio values every 1sec
        body=bioHub.readBpm();
        // keep portion of elapsedTime that is beyond 1sec to sync:
        bioPrevious=isrCount-(bioElapsed-ISRSPEED);
      }
    }else{ // same thing but make other checks if not logging
      bioElapsed=isrCount-bioPrevious; // update time since a bio read
      accelArray.pushback(getAccelDiff());
      // ^^^NOTE^^^ this takes 1150 MICROseconds
      if(bioElapsed>=ISRSPEED){ // update bio values every 1sec
        body=bioHub.readBpm();
        // keep portion of elapsedTime that is beyond 1sec to sync:
        bioPrevious=isrCount-(bioElapsed-ISRSPEED);
      }
      if(loggingState==3){ // logging start
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Begin logging in"); // print to OLED
        display.print(ISRSPEED-elapsedTime); // countdown
        display.println("..."); // tension
        display.print("Offset: ");
        display.println(accelArray.cxFact,6);
        display.display();
        if(elapsedTime>=(ISRSPEED)){ // only progress after 1 second
          display.clearDisplay();
          display.setCursor(0,0);
          display.println("Logging..."); // print to OLED
          display.println("Do not power off or");
          display.println("remove SD card!");
          display.print("CX Factor:");
          display.print(accelArray.cxFact,6); // display correction factor
          display.display();
        }
      }
      else if(loggingState==1&&elapsedTime<(2*ISRSPEED)){ // stop logging
        display.clearDisplay();
        display.setCursor(0,0);
        display.println("Done logging..."); // print to OLED
        display.println("Closing data log...");
        display.println("Resetting CX Factor.");
        display.display();
      }
      if(!loggingState){ // not logging, display current stats
        //display to OLED:
        display.clearDisplay();
        display.setCursor(0,0);
        display.print("Hr: "); // print to OLED
        display.print(body.heartRate,0);
        //display.print(body.heartRate);
        display.print("@");
        display.print(body.confidence);
        display.setCursor(76,0);
        display.print("Ox: ");
        display.print(body.oxygen,0);
        //display.print(body.oxygen);
        display.println("%");
        display.print("St: ");
        display.print(body.status);
        // display second count
        display.setCursor(52,8);
        display.print("100 Hz: ");
        snprintf(countBuffer,3,"%02i",(int)((isrCount%ISRSPEED)*0.1)); // Hz
        display.println(countBuffer);
        display.fillRect(0,17,(114.0*accelArray.getAvg()),4,WHITE);
        display.fillRect(0,22,(0.64*body.heartRate),4,WHITE);
        display.fillRect(0,27,(1.28*body.oxygen),4,WHITE);
        display.display();
      }
    }
  }
}

/** SDWRITE: this function is a task set to run on core #1:
    - timing is dictated by ISR setting the takeSample flag
    - logging will run at 100 Hz, 10ms per cycle, +/- 1ms cycle
    - both functions could be optimized further                               */
void sdWrite(void *param){
  esp_task_wdt_init(60,false);
  Serial.print("sdWrite() is running on core ");
  Serial.println(xPortGetCoreID());
  initSD(); // this is the only SPI device, SPI must run on core #1
    // SD init now happens only for logging, and ends after logging
    // this facilitates removing and inserting the SD card without rebooting
  loggingState=0; // set state
  fileNum=0; // set data log file number
  //WiFi.mode(WIFI_AP);
  WiFi.softAP("bioSense","12345678");
  //WiFi.softAPConfig(local_ip,gateway,subnet);
  IPAddress IP=WiFi.softAPIP();
  //Set server name "bioSense" - address will be "http://bioSense.local/"
  if (!MDNS.begin(servername)){          
    Serial.println("Error setting up MDNS responder..."); 
    ESP.restart(); 
  }
  //MDNS.addService("http","tcp",80);
  server.on("/",SD_dir);
  //server.on("/upload",File_Upload);
  server.on("/logging",Logging);
  //server.on("/fupload",HTTP_POST,[](){server.send(200);},handleFileUpload);
  server.on("/loggingst",HTTP_POST,[](){server.send(200);},loggingStart);
  server.begin();
  Serial.print("HTTP server started at: ");
  Serial.println(IP);
  for(;;){ // task loop
    elapsedTime=isrCount-previousTime;
    // logging data at DATAFREQ:
    if(loggingState==2){ // this has priority, execute fafb!
      if(takeSample){ // if ready to read new sample, set by ISR timer
        digitalWrite(12,HIGH); // indicate write
        previousTime=isrCount;
        snprintf(dataBuffer,29,"%07i,%06f,%03i,%03i",
                isrCount,
                accelArray.getAvg(),
                body.heartRate,
                body.oxygen);
        dataFile.println(dataBuffer);
        takeSample=0; // reset sample time
      }
      digitalWrite(12,LOW); // turn off LED when not writing
      if(!digitalRead(26))loggingState=1; // check button state
    }
    else if(!loggingState&&!digitalRead(26)){
      //if(SD.begin()){ // keep SD open for webserver
        loggingState=3; // read button state & debounce with state flag
        previousTime=isrCount;
        elapsedTime=0;
      //}
    }
    server.handleClient(); // listen for client connections
    if(loggingState==3){ // init logging data
      digitalWrite(12,(uint)(elapsedTime/(0.1*ISRSPEED))%2); // flash 0.1 sec
      accelArray.cxFact=1/accelArray.getAvg(); // update correction factor
      if(elapsedTime>=(ISRSPEED)){ // only progress after 1 second
        snprintf(nameBuffer,21,"/data/data-%04i.csv",fileNum+1);
        fileNum++;
        /** TODO: if file exists, increment filename and reopen*/
        dataFile=SD.open(nameBuffer,FILE_WRITE);
        dataFile.println("timestamp,accel,HR,o2%");
        previousTime=isrCount=bioPrevious=0;
        loggingState--; // move to next state
      }
    }
    else if(loggingState==1&&elapsedTime<(2*ISRSPEED)){ // finish logging data
      dataFile.close(); // close file out
      accelArray.cxFact=1;
      digitalWrite(12,HIGH); // indicate LED
    }
    else if(loggingState==1&&elapsedTime>=(2*ISRSPEED)){ // close SD file
      //SD.end(); // unmount SD card for removal
      digitalWrite(12,LOW); // turn off LED
      loggingState=0; // leave logging state
    }
    if(!loggingState){ // not logging data
      /** DEBUGGING: print to data stream
      snprintf(dataBuffer,29,"%07i,%06f,%03i,%03i",
                isrCount,
                accelArray.getAvg(),
                body.heartRate,
                body.oxygen);
      Serial.println(dataBuffer);*/
      // ^^^NOTE^^^ this will destroy sensor read rates
      // and causes duplicate data points
    }
  }
}

/*********  FUNCTIONS  **********/
// BEGIN webserver code insert:
//Initial page of the server web, list directory and give you the chance of deleting and uploading
void SD_dir(){
  //Action acording to post, dowload or delete, by MC 2022
  if(server.args()>0){ //Arguments were received, ignored if there are not arguments
    Serial.println(server.arg(0));
    String Order=server.arg(0);
    Serial.println(Order);
    if(Order.indexOf("download_")>=0){
      Order.remove(0,9);
      SD_file_download(Order);
      Serial.println(Order);
    }
    if((server.arg(0)).indexOf("delete_")>=0){
      Order.remove(0,7);
      SD_file_delete(Order);
      Serial.println(Order);
    }
  }
  File data=SD.open("/");
  if(data){
    data.rewindDirectory();
    SendHTML_Header();    
    webpage+=F("<table align='center'>");
    webpage+=F("<tr><th>Name/Type</th><th style='width:20%'>Type File/Dir</th><th>File Size</th></tr>");
    printDirectory("/data",0);
    webpage+=F("</table>");
    SendHTML_Content();
    data.close();
  }
  else{
    SendHTML_Header();
    webpage+=F("<h3>No Files Found</h3>");
  }
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();   //Stop is needed because no content length was sent
}

// page to start logging, if currently logging then cancel logging upon entering this page
void Logging(){
  if(loggingState==2)loggingState=1;
  append_page_header();
  webpage+=F("<h3>Logging State: READY</h3>"); 
  webpage+=F("<FORM action='/loggingst' method='post' enctype='multipart/form-data'>");
  webpage+=F("<input class='buttons' style='width:0%' type='file' name='loggingst' id = 'loggingst' value=''>");
  webpage+=F("<button class='buttons' style='width:25%' type='submit'>Logging Start</button><br><br>");
  /*webpage+=F("<h3>accelerometer difference: ");
  webpage+=String(accelArray.getAvg());
  webpage+=F("<br>Heartrate: ");
  webpage+=String(body.heartRate);
  webpage+=F("<br>Oxygen percentage: ");
  webpage+=String(body.oxygen);
  webpage+=F("</h3>");*/
  //webpage+=F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200,"text/html",webpage);
}

//Prints the directory, it is called in void SD_dir() 
void printDirectory(const char *dirname,uint8_t levels){
  File data=SD.open(dirname);
  if(!data)return;
  if(!data.isDirectory())return;
  File file=data.openNextFile();
  int i=0;
  while(file){
    if(webpage.length()>1000){
      SendHTML_Content();
    }
    if(file.isDirectory()){
      webpage+="<tr><td>"+String(file.isDirectory()?"Dir":"File")+"</td><td>"+String(file.name())+"</td><td></td></tr>";
      printDirectory(file.name(),levels-1);
    }
    else{
      webpage+="<tr><td>"+String(file.name())+"</td>";
      webpage+="<td>"+String(file.isDirectory()?"Dir":"File")+"</td>";
      int bytes=file.size();
      String fsize="";
      if(bytes<1024)fsize=String(bytes)+" B";
      else if(bytes<(1024*1024))fsize=String(bytes/1024.0,3)+" KB";
      else if(bytes<(1024*1024*1024))fsize=String(bytes/1024.0/1024.0,3)+" MB";
      else fsize=String(bytes/1024.0/1024.0/1024.0,3)+" GB";
      webpage+="<td>"+fsize+"</td>";
      webpage+="<td>";
      webpage+=F("<FORM action='/' method='post'>"); 
      webpage+=F("<button type='submit' name='download'"); 
      webpage+=F("' value='");
      webpage+="download_"+String(file.name());
      webpage+=F("'>Download</button>");
      webpage+="</td>";
      webpage+="<td>";
      webpage+=F("<FORM action='/' method='post'>"); 
      webpage+=F("<button type='submit' name='delete'"); 
      webpage+=F("' value='");
      webpage+="delete_"+String(file.name());
      webpage+=F("'>Delete</button>");
      webpage+="</td>";
      webpage+="</tr>";
    }
    file=data.openNextFile();
    i++;
  }
  file.close();
}

//Download a file from the SD, it is called in void SD_dir()
void SD_file_download(String filename){
    File download=SD.open("/"+filename);
    if(download){
      server.sendHeader("Content-Type","text/text");
      server.sendHeader("Content-Disposition","attachment; filename="+filename);
      server.sendHeader("Connection","close");
      server.streamFile(download,"application/octet-stream");
      download.close();
    }else ReportFileNotPresent("download");
}

// page to stop logging from, start logging upon entering this page
void loggingStart(){
  if(!loggingState)loggingState=3;
  append_page_header();
  webpage+=F("<h3>Logging State: LOGGING</h3>"); 
  webpage+=F("<FORM action='/logging' method='post' enctype='multipart/form-data'>");
  webpage+=F("<input class='buttons' style='width:0%' type='file' name='logging' id = 'logging' value=''>");
  webpage+=F("<button class='buttons' style='width:25%' type='submit'>Logging Stop</button><br><br>");
  /*webpage+=F("<h3>Accelerometer difference: ");
  webpage+=String(accelArray.getAvg());
  webpage+=F("<br>Heartrate: ");
  webpage+=String(body.heartRate);
  webpage+=F("<br>Oxygen percentage: ");
  webpage+=String(body.oxygen);
  webpage+=F("</h3>");*/
  //webpage+=F("<a href='/'>[Back]</a><br><br>");
  append_page_footer();
  server.send(200,"text/html",webpage);
}

//Delete a file from the SD, it is called in void SD_dir()
void SD_file_delete(String filename){
    SendHTML_Header();
    File dataFile=SD.open("/"+filename,FILE_READ); //Now read data from SD Card 
    if(dataFile){
      if(SD.remove("/"+filename)){
        Serial.println(F("File deleted successfully"));
        webpage+="<h3>File '"+filename+"' has been erased</h3>"; 
        webpage+=F("<a href='/'>[Back]</a><br><br>");
      }else{
        webpage+=F("<h3>File was not deleted - error</h3>");
        webpage+=F("<a href='/'>[Back]</a><br><br>");
      }
    }else ReportFileNotPresent("delete");
    append_page_footer(); 
    SendHTML_Content();
    SendHTML_Stop();
} 

//SendHTML_Header
void SendHTML_Header(){
  server.sendHeader("Cache-Control","no-cache, no-store, must-revalidate"); 
  server.sendHeader("Pragma","no-cache"); 
  server.sendHeader("Expires","-1"); 
  server.setContentLength(CONTENT_LENGTH_UNKNOWN); 
  server.send(200,"text/html",""); //Empty content inhibits Content-length header so we have to close the socket ourselves. 
  append_page_header();
  server.sendContent(webpage);
  webpage="";
}

//SendHTML_Content
void SendHTML_Content(){
  server.sendContent(webpage);
  webpage="";
}

//SendHTML_Stop
void SendHTML_Stop(){
  server.sendContent("");
  server.client().stop(); //Stop is needed because no content length was sent
}

//ReportFileNotPresent
void ReportFileNotPresent(String target){
  SendHTML_Header();
  webpage+=F("<h3>File does not exist</h3>"); 
  webpage+=F("<a href='/");
  webpage+=target +"'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}

//ReportCouldNotCreateFile
void ReportCouldNotCreateFile(String target){
  SendHTML_Header();
  webpage+=F("<h3>Could Not Create Uploaded File (write-protected?)</h3>"); 
  webpage+=F("<a href='/");
  webpage+=target+"'>[Back]</a><br><br>";
  append_page_footer();
  SendHTML_Content();
  SendHTML_Stop();
}
// END webserver code insert

//File size conversion
String file_size(int bytes){
  String fsize="";
  if(bytes<1024)fsize=String(bytes)+" B";
  else if(bytes<(1024*1024))fsize=String(bytes/1024.0,3)+" KB";
  else if(bytes<(1024*1024*1024))fsize=String(bytes/1024.0/1024.0,3)+" MB";
  else fsize = String(bytes/1024.0/1024.0/1024.0,3)+" GB";
  return fsize;
}

/** RUNFUNC: testing functions for average time to complete*/
void runFunc(){
  // first method, adafruit library, roughly 5333 microseconds
  //mpu1_accel->getEvent(&a1); // get mpu1 accel reading
  //mpu2_accel->getEvent(&a2); // get mpu2 accel reading
  //accelArray.pushback(a1.acceleration.y-a2.acceleration.y);

  // second method, directly access registers, roughly 1153 microseconds
  /*Wire.beginTransmission(0x68);
  Wire.write(0x3D); // Start with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,2,true); // Read 2 registers total
  acc1=(Wire.read()<<8|Wire.read()); // Y-axis value from mpu1
  Wire.beginTransmission(0x69);
  Wire.write(0x3D); // Start with register 0x3D (ACCEL_YOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(0x69,2,true); // Read 2 registers total
  acc2=(Wire.read()<<8|Wire.read()); // Y-axis value from mpu2
  accelArray.pushback((acc1-acc2)/16384.0);
  // MUCH BETTER, USE THIS METHOD*/
  accelArray.pushback(getAccelDiff());
}

void measure_function(){ // used for timing various functions
  const unsigned MEASUREMENTS=5000;
  uint64_t start=esp_timer_get_time();

  for (int retries=0;retries<MEASUREMENTS;retries++){
      runFunc();
  }
  uint64_t end=esp_timer_get_time();
  char buffer[64];
  snprintf(buffer,64,
            "%u iterations took %llu milliseconds (%llu microseconds per)\n",
          MEASUREMENTS,(end-start)/1000,(end-start)/MEASUREMENTS);
  Serial.println(buffer);
}
