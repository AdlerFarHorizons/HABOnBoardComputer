#include <SPI.h>
#include <SD.h>
#include <DS3231.h>

#define listen(serialbuf) if (serialbuf.port->available()) getCharSerial(&serialbuf);


void getCharGPS();
void getCharIMU();
void getCharGCR();
void parseSentence(char *S, int n_args,...);

char dataFileName[25];
 
struct SerialBuff {
  boolean sentencePending, sentenceRdy;
  char sentenceBuf[150];
  int sentenceBufIndex = 0;
  HardwareSerial *port;
};


// serial port structures for the GPS, IMU and CMS
struct SerialBuff GPS_Serial,IMU_Serial,GCR_Serial;

File myFile;        // file for SD card

int latDegrees,longDegrees,timeHours,timeMinutes;
float timeSeconds, latMinutes,longMinutes,altitude;

int i=0;
unsigned long mtime1,mtime2,mtime3;
unsigned long lastGPSmillis=0,lastIMUmillis=0,lastGCRmillis=0;
unsigned long lastIMUPmillis=0,lastIMUOmillis=0,lastIMUMmillis=0,lastIMUAmillis=0;
unsigned long lastCameraRunMillis=0,lastStateLogMillis=0;
unsigned long startMillis;
int startHours,startMinutes;
float startSeconds;
float Pitch,Roll,Yaw,Pressure,Temp;
float AccX,AccY,AccZ;
int magX,magY,magZ;
float Count_Duration;
int Counts;


// Init the DS3231 using the hardware interface
DS3231  rtc(SDA, SCL);

void setup() {
  int i,j;
  char dateString[20];
  char timeString[20];
char collisionAvoid[]="t0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

  // Initialize the rtc object
  rtc.begin();
  
  // Open serial communications and wait for port to open:
  Serial.begin( 115200 );
  while( !Serial );
  
  // clear input buffers
  while ( Serial.available() > 0 ) Serial.read();


char test[100];

sprintf(test,"this is a test of the system");

Serial.println(test);
stringreplace(test,' ' ,'/');
Serial.println(test);

 

  // Serial buffer initialization
  initSerialBuff(&GPS_Serial,&Serial1,9600);     //GPS
  initSerialBuff(&IMU_Serial,&Serial2,57600);    //IMU
//  initSerialBuff(&GCR_Serial,&Serial3,9600);     //Geiger
 Serial3.begin(57600); 
 
  // SD card initialization
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("     initialization failed!");
    return;
  }
  Serial.println("      initialization done.");


  // make data file name and check if it already exists
  strcpy(dateString,rtc.getDateStr());
  strcpy(timeString,rtc.getTimeStr());
  Serial.println(timeString);
  sprintf(dataFileName,"%c%c%c%c%c%c%c%c.da%c",
            dateString[3],dateString[4],dateString[0],dateString[1],
            timeString[0],timeString[1],timeString[3],timeString[4],collisionAvoid[i=0]);
  Serial.println(dataFileName);
  while (SD.exists(dataFileName)) { 
      sprintf(dataFileName,"%c%c%c%c%c%c%c%c.da%c",
            dateString[3],dateString[4],dateString[0],dateString[1],
            timeString[0],timeString[1],timeString[3],timeString[4],
            collisionAvoid[++i]);
      Serial.println(dataFileName);
  }
  
  if (!(myFile = SD.open(dataFileName, FILE_WRITE))) { 
    Serial.println("logfile failed to open");
    return;
  } 

  // reinitialize GPS to send "TF" and "ZDA" sentences
 // Serial1.print("$PTNLSNM,0220,01*57\r\n");

} // done with setup


void initSerialBuff(struct SerialBuff *SB, HardwareSerial *port, int Baud)
{
  SB->sentencePending = false;
  SB->sentenceRdy = false;
  SB->sentenceBuf[0] = 0;
  SB->sentenceBufIndex = 0;
  SB->port=port;

  SB->port->begin( Baud );
  // clear input buffers
  while ( SB->port->available() > 0 ) SB->port->read();

}

void loop() {
  String testString;
  int L;
  int currentHours,currentMinutes;
  float currentSeconds;
  int timeSinceLastCameraRun,timeSinceLastStateLog;
  char cameraString[200];

  // listen to the serial ports, one call for each port being listened to
  listen(GPS_Serial);
  listen(IMU_Serial);
 // listen(GCR_Serial);

  // when we have a full sentence from the serial ports then deal with it 
  if (GPS_Serial.sentenceRdy) processGPS();     // parse string, print to terminal, log, and sync time 
  if (IMU_Serial.sentenceRdy) processIMU();
//  if (GCR_Serial.sentenceRdy) processGCR();

 // do actions

  // we will log the state variables every 5 seconds
  // as soon as we get to it
  timeSinceLastStateLog = millis() - lastStateLogMillis;  // time since last

  if (timeSinceLastStateLog>5000) {
    lastStateLogMillis=millis();
    logStateVariables();
/*
  sprintf(cameraString,
        "$%s_%d_%d_%.2f$\n",rtc.getDateStr(),timeHours,timeMinutes,timeSeconds);
      Serial3.print(cameraString);
      myFile.print(cameraString);
      myFile.flush();
      Serial.print(cameraString);
*/
}
  
 

 
}

void logStateVariables() {

  // print the current millis, 
  // the most recent GPS information with millis timestamp,
  // and the most recent IMU information with millis timestamp
  char logString[300];
  sprintf(logString,"$STATE %8ld %8ld %2d %02d %5.2f %4d %5.2f %4d %5.2f %8.1f %8ld %6.2f %6.2f %6.2f %8ld %6.2f %6.2f %8ld %5d %5d %5d %8ld %6.2f %6.2f %6.2f %8ld %6.2f %5d\n", 
                     millis(),lastGPSmillis,
                     timeHours,timeMinutes,timeSeconds,
                     latDegrees,latMinutes,
                     longDegrees,longMinutes,
                     altitude,
                     lastIMUOmillis,
                     Pitch,Roll,Yaw,
                     lastIMUPmillis,
                     Pressure,Temp,
                     lastIMUMmillis,
                     magX,magY,magZ,
                     lastIMUAmillis,
                     AccX,AccY,AccZ,
                     lastGCRmillis,
                     Count_Duration,Counts);

  myFile.print(logString);
  myFile.flush();
  Serial.print(logString);     //send it also to the Serial monitor
}

void processGPS() {
  static int firstFix=1;
  String testString;
  char cameraString[200];
/*  Serial.println(GPS_Serial.sentenceBuf);

  if (strncmp(GPS_Serial.sentenceBuf,"$GPGGA",6)==0) {
    if (stringreplace(GPS_Serial.sentenceBuf,',',' ')!=14) Serial.println("malformed $GPGGA sentence!");
    else 
      {
        sscanf(GPS_Serial.sentenceBuf+6,"%f %d",&Count_Duration,&Counts);  
        lastGPSmillis=millis();
      }
  }

 if (strncmp(GPS_Serial.sentenceBuf,"$GPZDA",6)==0) {
  Serial.println(GPS_Serial.sentenceBuf);
    if (stringreplace(GPS_Serial.sentenceBuf,',',' ')!=14) Serial.println("malformed $GPGGA sentence!");
    else 
      {
        sscanf(GPS_Serial.sentenceBuf+6,"%f %d",&Count_Duration,&Counts);  
        lastGPSmillis=millis();
      }
 */
 
 // $GPZDA,hhmmss.ss,xx,xx,xxxx,,*hh<CR><LF>
 /*

  S=toksplit(S,',',t,20);
   S=toksplit(S,',',t,20); tTime=atof(t);       //extract the time
    S=toksplit(S,',',t,20); latitude=atof(t);    //extract the latitude
    S=toksplit(S,',',t,20); hemiP= t[0];         //extract the hemisphere
    S=toksplit(S,',',t,20); longitude=atof(t);   //extract the longitude
    S=toksplit(S,',',t,20); hemiC= t[0];         //extract the longitude
    //skip next three fields
    S=toksplit(S,',',t,20); S=toksplit(S,',',t,20); S=toksplit(S,',',t,20); 
    S=toksplit(S,',',t,20); altitude=atof(t);    //extract the altitude

*/
 
   // parse buffer
   parseSentence(GPS_Serial.sentenceBuf,8,&timeHours,&timeMinutes,&timeSeconds,
                     &latDegrees,&latMinutes,&longDegrees,&longMinutes,&altitude);
//Serial.println(GPS_Serial.sentenceBuf);

   // set the milli clock timestamp for this GPS fix
   lastGPSmillis=millis();


   //reinitialize buffer for next sentence
      GPS_Serial.sentenceRdy = false;
      GPS_Serial.sentenceBuf[0] = 0;
      GPS_Serial.sentenceBufIndex = 0;

    if ((timeSeconds==16)||(timeSeconds==48)) {

      sprintf(cameraString,
        "$%s_%d_%d_%5.2f$\n",rtc.getDateStr(),timeHours,timeMinutes,timeSeconds);
      Serial3.print(cameraString);
      myFile.print(cameraString);
      myFile.flush();
      Serial.print(cameraString);
    }

}


void processIMU() {
  String testString;
  
   // parse buffer if the sentence is $FHIPR
   if (strncmp(IMU_Serial.sentenceBuf,"$FHIPR",6)==0) {
      parseSentence(IMU_Serial.sentenceBuf,3,&Pitch,&Roll,&Yaw);
      // timestamp IMU data with millis
      lastIMUOmillis=millis();
   }

   // parse buffer if the sentence is $FHIAC
   if (strncmp(IMU_Serial.sentenceBuf,"$FHIAC",6)==0) {
      parseSentence(IMU_Serial.sentenceBuf,3,&AccX,&AccY,&AccZ);
      // timestamp IMU data with millis
      lastIMUAmillis=millis();
   }

   // parse buffer if the sentence is $FHPRS
   if (strncmp(IMU_Serial.sentenceBuf,"$FHPRS",6)==0) {
      parseSentence(IMU_Serial.sentenceBuf,2,&Temp,&Pressure);
      // timestamp IMU data with millis
      lastIMUPmillis=millis();
   }

   // parse buffer if the sentence is $FHIMR
   if (strncmp(IMU_Serial.sentenceBuf,"$FHIMR",6)==0) {
      parseSentence(IMU_Serial.sentenceBuf,3,&magX,&magY,&magZ);
      // timestamp IMU data with millis
      lastIMUMmillis=millis();
   }


   //reinitialize buffer
   IMU_Serial.sentenceRdy = false;
   IMU_Serial.sentenceBuf[0] = 0;
   IMU_Serial.sentenceBufIndex = 0;
}

void processGCR() {
  String testString;


  if (strncmp(GCR_Serial.sentenceBuf,"$FHGCR",6)==0) {
    if (stringreplace(GCR_Serial.sentenceBuf,',',' ')!=2) Serial.println("malformed $FHGCR sentence!");
    else 
      {
        sscanf(GCR_Serial.sentenceBuf+6,"%f %d",&Count_Duration,&Counts);  
        lastGCRmillis=millis();
      }
  }
  
   //reinitialize buffer
   GCR_Serial.sentenceRdy = false;
   GCR_Serial.sentenceBuf[0] = 0;
   GCR_Serial.sentenceBufIndex = 0;

}

void parseSentence(char *S, int n_args,...) {
  va_list ap;
  char t[20];

  va_start(ap,n_args);
  //what kind of sentence is it?

  if (strncmp(S,"$GPGGA",6)==0) {   //extract time location altitude

    // start by defining the local variables
   int *tmpHours;
   int *tmpMinutes;
   float *tmpSeconds;
   int *tmpLatDegrees,*tmpLongDegrees;
   float *tmpLatMinutes,*tmpLongMinutes,*tmpAltitude;

   float tTime,latitude,longitude,altitude,tSeconds;
   char hemiP,hemiC;
   int tHours,tMinutes;
   int tLatDegrees; float tLatMinutes;

   //first get the variables pointers we will drop these into
    tmpHours=va_arg(ap,int*);
   tmpMinutes=va_arg(ap,int*);
   tmpSeconds=va_arg(ap,float*);
   tmpLatDegrees=va_arg(ap,int*);
   tmpLatMinutes=va_arg(ap,float*);
   tmpLongDegrees=va_arg(ap,int*);
   tmpLongMinutes=va_arg(ap,float*);
   tmpAltitude=va_arg(ap,float*);

   S=toksplit(S,',',t,20);
   S=toksplit(S,',',t,20); tTime=atof(t);       //extract the time
    S=toksplit(S,',',t,20); latitude=atof(t);    //extract the latitude
    S=toksplit(S,',',t,20); hemiP= t[0];         //extract the hemisphere
    S=toksplit(S,',',t,20); longitude=atof(t);   //extract the longitude
    S=toksplit(S,',',t,20); hemiC= t[0];         //extract the longitude
    //skip next three fields
    S=toksplit(S,',',t,20); S=toksplit(S,',',t,20); S=toksplit(S,',',t,20); 
    S=toksplit(S,',',t,20); altitude=atof(t);    //extract the altitude


    //massage the values a bit
   *tmpHours=(int) (tTime/10000.0);
   *tmpMinutes=(int)((tTime-10000*(*tmpHours))/100.0);
   *tmpSeconds=(tTime-10000*(*tmpHours)-(*tmpMinutes)*100);
  
   *tmpLatDegrees= (int) (latitude/100.0);
   *tmpLatMinutes= (latitude-100*(*tmpLatDegrees));

   *tmpLongDegrees= (int) (longitude/100.0);
   *tmpLongMinutes= (longitude-100*(*tmpLongDegrees));
  
   *tmpAltitude= altitude;
 
  }

  if (strncmp(S,"$FHIPR",6)==0) {
 
    float *tmpPitch,*tmpRoll,*tmpYaw;
  
    //first get the variables pointers we will drop these into
    tmpPitch=va_arg(ap,float*);
    tmpRoll=va_arg(ap,float*);
    tmpYaw=va_arg(ap,float*);

    S=toksplit(S,',',t,20);
    S=toksplit(S,',',t,20);                      //ignore the first value
    S=toksplit(S,',',t,20); *tmpPitch=atof(t);   //extract the pitch
    S=toksplit(S,',',t,20); *tmpRoll=atof(t);    //extract the roll
    S=toksplit(S,',',t,20); *tmpYaw= atof(t);    //extract the yaw 
   
  }

if (strncmp(S,"$FHIAC",6)==0) {
 
  float *tmpAccX,*tmpAccY,*tmpAccZ;
  
  //first get the variables pointers we will drop these into
  tmpAccX=va_arg(ap,float*);
  tmpAccY=va_arg(ap,float*);
  tmpAccZ=va_arg(ap,float*);

  S=toksplit(S,',',t,20);
  S=toksplit(S,',',t,20);                      //ignore the first value
  S=toksplit(S,',',t,20); *tmpAccX=atof(t);   //extract the X accel
  S=toksplit(S,',',t,20); *tmpAccY=atof(t);    //extract the Y accel
  S=toksplit(S,',',t,20); *tmpAccZ=atof(t);    //extract the Z accel
    
  }
  
if (strncmp(S,"$FHPRS",6)==0) {       // temperature and pressure 
 
  float *tmpTemp,*tmpPressure;
  
  //first get the variables pointers we will drop these into
  tmpTemp=va_arg(ap,float*);
  tmpPressure=va_arg(ap,float*);

  S=toksplit(S,',',t,20);
  S=toksplit(S,',',t,20);                      //ignore the first value
  S=toksplit(S,',',t,20); *tmpTemp=atof(t);   //extract the temperature
  S=toksplit(S,',',t,20); *tmpPressure=atof(t);    //extract the pressure
   
  }

if (strncmp(S,"$FHIMR",6)==0) { // extract raw magnetometer data
 
  int *tmpMX,*tmpMY,*tmpMZ;
  
  //first get the variables pointers we will drop these into
  tmpMX=va_arg(ap,int*);
  tmpMY=va_arg(ap,int*);
  tmpMZ=va_arg(ap,int*);

  S=toksplit(S,',',t,20);
  S=toksplit(S,',',t,20);                      //ignore the first value
  S=toksplit(S,',',t,20); *tmpMX=atoi(t);    //extract the X component
  S=toksplit(S,',',t,20); *tmpMY=atoi(t);    //extract the Y component
  S=toksplit(S,',',t,20); *tmpMZ=atoi(t);    //extract the Z component
   
  }
  
if (strncmp(S,"$FHGCR",6)==0) {       // Geiger Data 
 
  float *tmpDuration;
  int *tmpCounts;
  
  //first get the variables pointers we will drop these into
  tmpDuration=va_arg(ap,float*);
  tmpCounts=va_arg(ap,int*);

  S=toksplit(S,',',t,20);
  S=toksplit(S,',',t,20); *tmpDuration=atof(t);   //extract the temperature
  S=toksplit(S,',',t,20); *tmpCounts=atoi(t);    //extract the pressure
   
  }

va_end(ap);

}

char *toksplit(char *src, char tokchar, char *token, size_t lgh)
{
  if (src) {
    while (' ' == *src) *src++;

    while (*src && (tokchar != *src)) {
      if (lgh) {
        *token++ = *src;
        --lgh;
      }
      src++;
    }
  if (*src && (tokchar == *src)) src++;
  }
  *token = '\0';
  return src;
} /* toksplit */



void getCharSerial(struct SerialBuff *SB) {
  
  char c;
//  Serial.println(sentenceBufGPS);

  if ( !SB->sentenceRdy ) {   // don't process if we have a sentence ready

      c = SB->port->read();  // read the next character

      
      if ( c == 36 ) {  // dollar sign -- beginning of sentence
        SB->sentenceBufIndex = 0;
        SB->sentenceRdy = false;
        SB->sentencePending = true;
      }
      if ( SB->sentencePending && (c == 10) ) { // Line feed -- end of sentence
        SB->sentenceRdy = true;
        SB->sentencePending = false;
      }

  if (SB->sentencePending) {  //we have a sentence building

      SB->sentenceBuf[SB->sentenceBufIndex] = c;   // store the character
      SB->sentenceBufIndex++;                     // increment the index
      SB->sentenceBuf[SB->sentenceBufIndex] = 0;   // make sure we always end on a null
      if (SB->sentenceBufIndex>140) { //something is wrong... sentence is too long
 
        // truncate it now -- 
        // this means we will throw the results to the parser and not accumulate 
        // anything until we hit another "$"
        SB->sentenceRdy = true;
        SB->sentencePending = false;
      }
    }
  } 
}


// in place replacement of one character by another in a string
int stringreplace(char *S, char o, char d)
{
 
 int rv=0,j=-1;
  while (S[++j]!='\0') if (S[j]==o) {S[j]=d; rv++;}
 return rv;
}

int addspace(char *S)
{


}



