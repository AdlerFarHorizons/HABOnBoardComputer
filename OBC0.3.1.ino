#include <SPI.h>
#include <SD.h>

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
unsigned long lastCameraRunMillis=0,lastStateLogMillis=0;
unsigned long startMillis;
int startHours,startMinutes;
float startSeconds;
float Pitch,Roll,Yaw;

void setup() {
  int i,j;
  
  // Open serial communications and wait for port to open:
  Serial.begin( 115200 );
  Serial1.begin( 9600 );
  Serial2.begin( 57600 );
  Serial3.begin( 9600 );
  
  while( !Serial );
  
  // clear input buffers
  while ( Serial.available() > 0 ) Serial.read();
  while ( Serial1.available() > 0 ) Serial1.read();
  while ( Serial2.available() > 0 ) Serial2.read();
  while ( Serial3.available() > 0 ) Serial3.read();
  
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
  sprintf(dataFileName,"OBCLg%03d.dat",i=0);
  Serial.println(dataFileName);
  while (SD.exists(dataFileName)) { 
    sprintf(dataFileName,"OBCLg%03d.dat",++i);
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
  if (Serial3.available()) getCharCMS();

  // when we have a full sentence from the serial ports then deal with it 
  if (sentenceRdyGPS) processGPS();     // parse string, print to terminal, log, and sync time 
  if (sentenceRdyIMU) processIMU();

  // do actions
  // we will start a camera run every five minutes 
  // with a tolerance of 5 sec

  timeSinceLastCameraRun = millis()-lastCameraRunMillis;  // time since last

  if ((timeSinceLastCameraRun>300000) && (timeSinceLastCameraRun<305000)) {
    startCameraRun(15,250);
    lastCameraRunMillis=millis();
  }

  // we will log the state variables every 5 seconds
  // with a tolerance of 0.1 sec
  timeSinceLastStateLog = millis() - lastStateLogMillis;  // time since last

  if ((timeSinceLastStateLog>5000) && (timeSinceLastStateLog<5100)) {
    logStateVariables();
    lastStateLogMillis=millis();
  }

  // we may need code here to close and reopen the SD file every once 
  // in a while to make sure we don't lose lots of data when the OBC 
  // is turned off

 
}

void logStateVariables() {

// print the current millis, 
// the most recent GPS information with millis timestamp,
// and the most recent IMU information with millis timestamp
char logString[200];
sprintf(logString,"$STATE %ld %ld %2d %02d %5.2f %4d %5.2f %4d %5.2f %8.1f %ld %6.2f %6.2f %6.2f\n", 
                   millis(),lastGPSmillis,
                   timeHours,timeMinutes,timeSeconds,
                   latDegrees,latMinutes,
                   longDegrees,longMinutes,
                   altitude,
                   lastIMUmillis,
                   Pitch,Roll,Yaw);

myFile.print(logString);
myFile.flush();
Serial.print(logString);
}

void processGPS() {
  static int firstFix=1;
  String testString;
  
   // write to serial monitor
//   Serial.print( String(testString + String(i) + ": " + sentenceBufGPS) );

   // parse buffer
   parseSentence(sentenceBufGPS,8,&timeHours,&timeMinutes,&timeSeconds,
                     &latDegrees,&latMinutes,&longDegrees,&longMinutes,&altitude);

   // now that we have a GPS time we set the milli clock start point
   lastGPSmillis=millis();

   if (firstFix) {
      startMillis=millis();
      startHours=timeHours;
      startMinutes=timeMinutes;
      startSeconds=timeSeconds;
    }
    
   // log it to our logfile
//      if (i<116) { myFile.print(sentenceBufGPS);}

   //reinitialize buffer for next sentence
      sentenceRdyGPS = false;
      sentenceBufGPS[0] = 0;
      sentenceBufIndexGPS = 0;
}


void processIMU() {
  String testString;
  
   // write to serial monitor
 //  Serial.print( String(testString + String(i) + ": " + sentenceBufIMU) );

   // parse buffer if the sentence is $FHIPR
   if (strncmp(sentenceBufIMU,"$FHIPR",6)==0) {
      parseSentence(sentenceBufIMU,3,&Pitch,&Roll,&Yaw);

      // timestamp IMU data with millis
      lastIMUmillis=millis();
   }

   //reinitialize buffer
   sentenceRdyIMU = false;
   sentenceBufIMU[0] = 0;
   sentenceBufIndexIMU = 0;
}


void startCameraRun(int delayLength, int duration) {
  int cTimeH,cTimeM;
  float cTimeS;
  char outString[150];

  // find the current time
  currentTime(&cTimeH,&cTimeM,&cTimeS);

  // construct the command string
  sprintf(outString,"%02d:%02d:%05.2f:%05d:%05d",
                      cTimeH,cTimeM,cTimeS,delayLength,duration);
  // send the command string
  Serial3.println(outString);

  // now log the command string and the most recent state variables
  myFile.println(outString);          // command string
myFile.flush();
Serial.println(outString);
  logStateVariables();                // state variables
}


void currentTime(int *currentHours, int *currentMinutes, float *currentSeconds) {
int overflowHours,overflowMinutes;
  *currentSeconds=timeSeconds+(millis()-lastGPSmillis)/1000.0;
  *currentMinutes=timeMinutes;
  *currentHours=timeHours;

  if (*currentSeconds>=60.0) {

    overflowMinutes=(int) (*currentSeconds/60.0);
    *currentSeconds-=60.0*overflowMinutes;
    *currentMinutes+=overflowMinutes;

    if (*currentMinutes>=60.0) {
      overflowHours=(int) (*currentMinutes/60.0);
      *currentMinutes-=60.0*overflowHours;
      *currentHours+=overflowHours;
    }
  }
}




void parseSentence(char *S, int n_args,...) {

float tTime,latitude,longitude,altitude,tSeconds;
char hemiP,hemiC;
int tHours,tMinutes;
int tLatDegrees; float tLatMinutes;
va_list ap;
int *tmpHours;
int *tmpMinutes;
float *tmpSeconds;
int *tmpLatDegrees,*tmpLongDegrees;
float *tmpLatMinutes,*tmpLongMinutes,*tmpAltitude;
char t[20];

va_start(ap,n_args);
//what kind of sentence is it?

if (strncmp(S,"$GPGGA",6)==0) {
  //extract time location altitude

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
  
  c = Serial1.read();
    if ( !sentenceRdyGPS ) {
      if ( c == 36 ) {  // dollar sign -- beginning of sentence
//        sentence = String();
        sentenceBufIndexGPS = 0;
        sentencePendingGPS = true;
      }
      if ( sentencePendingGPS && c == 10 ) { // Line feed -- end of sentence
        sentenceRdyGPS = true;
        sentencePendingGPS = false;
      }
//      sentence += c;
      sentenceBufGPS[sentenceBufIndexGPS] = c;   // store the character
      sentenceBufIndexGPS++;
      sentenceBufGPS[sentenceBufIndexGPS] = 0;   // make sure we end on a null
    }
  
}

void getCharIMU() {
  
  char c;
  
  c = Serial2.read();
    if ( !sentenceRdyIMU ) {
      if ( c == 36 ) {  // dollar sign -- beginning of sentence
//        sentence = String();
        sentenceBufIndexIMU = 0;
        sentencePendingIMU = true;
      }
      if ( sentencePendingIMU && c == 10 ) { // Line feed -- end of sentence
        sentenceRdyIMU = true;
        sentencePendingIMU = false;
      }
//      sentence += c;
      sentenceBufIMU[sentenceBufIndexIMU] = c;   // store the character
      sentenceBufIndexIMU++;
      sentenceBufIMU[sentenceBufIndexIMU] = 0;   // make sure we end on a null
    }
  
}

void getCharCMS() {
  
  char c;
  
  c = Serial3.read();
    if ( !sentenceRdyCMS ) {
      if ( c == 36 ) {  // dollar sign -- beginning of sentence
//        sentence = String();
        sentenceBufIndexCMS = 0;
        sentencePendingCMS = true;
      }
      if ( sentencePendingCMS && c == 10 ) { // Line feed -- end of sentence
        sentenceRdyCMS = true;
        sentencePendingCMS = false;
      }
//      sentence += c;
      sentenceBufCMS[sentenceBufIndexCMS] = c;   // store the character
      sentenceBufIndexCMS++;
      sentenceBufCMS[sentenceBufIndexCMS] = 0;   // make sure we end on a null
    }
  
}


/*

int i;
 String baseString = "Testing: ";
 
 
  // if the file opened okay, write to it:
  if (myFile) {
    Serial.print("Writing to test.txt...");
for (i=0;i<10;i++) {    myFile.println(String(baseString + String(i)));}
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }

//close the file
    myFile.close();

  
  }

*/
