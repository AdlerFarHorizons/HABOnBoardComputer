#include <SPI.h>
#include <SD.h>
#include <DS3231.h>

void getCharGPS();
void getCharIMU();
void getCharCMS();
void parseSentence(char *S, int n_args,...);

char dataFileName[25];

File myFile;        // file for SD card

int latDegrees,longDegrees,timeHours,timeMinutes;
float timeSeconds, latMinutes,longMinutes,altitude;

// serial port variables for the GPS, IMU and CMS
boolean sentencePendingGPS, sentenceRdyGPS;
char sentenceBufGPS[150];
int sentenceBufIndexGPS = 0;

boolean sentencePendingIMU, sentenceRdyIMU;
char sentenceBufIMU[150];
int sentenceBufIndexIMU = 0;

boolean sentencePendingCMS, sentenceRdyCMS;
char sentenceBufCMS[150];
int sentenceBufIndexCMS = 0;

int i=0;
unsigned long mtime1,mtime2,mtime3,lastGPSmillis=0,lastIMUmillis=0;
unsigned long lastIMUPmillis=0,lastIMUOmillis=0,lastIMUMmillis=0,lastIMUAmillis=0;
unsigned long lastCameraRunMillis=0,lastStateLogMillis=0;
unsigned long startMillis;
int startHours,startMinutes;
float startSeconds;
float Pitch,Roll,Yaw,Pressure,Temp;
float AccX,AccY,AccZ;
int magX,magY,magZ;


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
  Serial1.begin( 9600 );
  Serial2.begin( 57600 );
  
  while( !Serial );
  
  // clear input buffers
  while ( Serial.available() > 0 ) Serial.read();
  while ( Serial1.available() > 0 ) Serial1.read();
  while ( Serial2.available() > 0 ) Serial2.read();
  
  // IMU initialization
  sentencePendingIMU = false;
  sentenceRdyIMU = false;
  sentenceBufIMU[0] = 0;
  sentenceBufIndexIMU = 0;

  // GPS initialization
  sentencePendingGPS = false;
  sentenceRdyGPS = false;
  sentenceBufGPS[0] = 0;
  sentenceBufIndexGPS = 0;

  // CMS (CaMera System) initialization
  sentencePendingCMS = false;
  sentenceRdyCMS = false;
  sentenceBufCMS[0] = 0;
  sentenceBufIndexCMS = 0;


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

} // done with setup

void loop() {
  String testString;
  int L;
  int currentHours,currentMinutes;
  float currentSeconds;
  int timeSinceLastCameraRun,timeSinceLastStateLog;

  // listen to the serial ports, one call for each port being listened to
  if (Serial1.available()) getCharGPS();
  if (Serial2.available()) getCharIMU();

  // when we have a full sentence from the serial ports then deal with it 
  if (sentenceRdyGPS) processGPS();     // parse string, print to terminal, log, and sync time 
  if (sentenceRdyIMU) processIMU();

  // do actions

  // we will log the state variables every 5 seconds
  // as soon as we get to it
  timeSinceLastStateLog = millis() - lastStateLogMillis;  // time since last

  if (timeSinceLastStateLog>5000) {
    lastStateLogMillis=millis();
    logStateVariables();
  }

 
}

void logStateVariables() {

  // print the current millis, 
  // the most recent GPS information with millis timestamp,
  // and the most recent IMU information with millis timestamp
  char logString[200];
  sprintf(logString,"$STATE %ld %ld %2d %02d %5.2f %4d %5.2f %4d %5.2f %8.1f %ld %6.2f %6.2f %6.2f %ld %6.2f %6.2f %ld %5d %5d %5d %ld %6.2f %6.2f %6.2f\n", 
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
                     AccX,AccY,AccZ);

  myFile.print(logString);
  myFile.flush();
  Serial.print(logString);     //send it also to the Serial monitor
}

void processGPS() {
  static int firstFix=1;
  String testString;
  
   // parse buffer
   parseSentence(sentenceBufGPS,8,&timeHours,&timeMinutes,&timeSeconds,
                     &latDegrees,&latMinutes,&longDegrees,&longMinutes,&altitude);

   // set the milli clock timestamp for this GPS fix
   lastGPSmillis=millis();

   //reinitialize buffer for next sentence
      sentenceRdyGPS = false;
      sentenceBufGPS[0] = 0;
      sentenceBufIndexGPS = 0;
}


void processIMU() {
  String testString;
  
   // parse buffer if the sentence is $FHIPR
   if (strncmp(sentenceBufIMU,"$FHIPR",6)==0) {
      parseSentence(sentenceBufIMU,3,&Pitch,&Roll,&Yaw);
      // timestamp IMU data with millis
      lastIMUOmillis=millis();
   }

   // parse buffer if the sentence is $FHIAC
   if (strncmp(sentenceBufIMU,"$FHIAC",6)==0) {
      parseSentence(sentenceBufIMU,3,&AccX,&AccY,&AccZ);
      // timestamp IMU data with millis
      lastIMUAmillis=millis();
   }

   // parse buffer if the sentence is $FHPRS
   if (strncmp(sentenceBufIMU,"$FHPRS",6)==0) {
      parseSentence(sentenceBufIMU,2,&Temp,&Pressure);
      // timestamp IMU data with millis
      lastIMUPmillis=millis();
   }

   // parse buffer if the sentence is $FHIMR
   if (strncmp(sentenceBufIMU,"$FHIMR",6)==0) {
      parseSentence(sentenceBufIMU,3,&magX,&magY,&magZ);
      // timestamp IMU data with millis
      lastIMUMmillis=millis();
   }


   //reinitialize buffer
   sentenceRdyIMU = false;
   sentenceBufIMU[0] = 0;
   sentenceBufIndexIMU = 0;
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


void getCharGPS() {
  
  char c;
//  Serial.println(sentenceBufGPS);

  if ( !sentenceRdyGPS ) {   // don't process if we have a sentence ready

      c = Serial1.read();  // read the next character

      
      if ( c == 36 ) {  // dollar sign -- beginning of sentence
        sentenceBufIndexGPS = 0;
        sentenceRdyGPS = false;
        sentencePendingGPS = true;
      }
      if ( sentencePendingGPS && (c == 10) ) { // Line feed -- end of sentence
        sentenceRdyGPS = true;
        sentencePendingGPS = false;
      }

  if (sentencePendingGPS) {  //we have a sentence building

      sentenceBufGPS[sentenceBufIndexGPS] = c;   // store the character
      sentenceBufIndexGPS++;                     // increment the index
      sentenceBufGPS[sentenceBufIndexGPS] = 0;   // make sure we always end on a null
      if (sentenceBufIndexGPS>140) { //something is wrong... sentence is too long
 
        // truncate it now -- 
        // this means we will throw the results to the parser and not accumulate 
        // anything until we hit another "$"
        sentenceRdyGPS = true;
        sentencePendingGPS = false;
      }
    }
  } 
}

void getCharIMU() {
  
  char c;
//  Serial.println(sentenceBufIMU);

  if ( !sentenceRdyIMU ) {   // don't process if we have a sentence ready

      c = Serial2.read();  // read the next character
      
      if ( c == 36 ) {  // dollar sign -- beginning of sentence
        sentenceBufIndexIMU = 0;
        sentenceRdyIMU = false;
        sentencePendingIMU = true;
      }
      if ( sentencePendingIMU && (c == 10) ) { // Line feed -- end of sentence
        sentenceRdyIMU = true;
        sentencePendingIMU = false;
      }

  if (sentencePendingIMU) {  //we have a sentence building

      sentenceBufIMU[sentenceBufIndexIMU] = c;   // store the character
      sentenceBufIndexIMU++;                     // increment the index
      sentenceBufIMU[sentenceBufIndexIMU] = 0;   // make sure we always end on a null
      if (sentenceBufIndexIMU>140) { //something is wrong... sentence is too long
 
        // truncate it now -- 
        // this means we will throw the results to the parser and not accumulate 
        // anything until we hit another "$"
        sentenceRdyIMU = true;
        sentencePendingIMU = false;
      }
    }
  }   
  
}


