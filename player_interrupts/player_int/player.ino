/***************************************************
//PLAYER
 ****************************************************/

// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

// These are the pins used for the breakout example
#define BREAKOUT_RESET  9      // VS1053 reset pin (output)
#define BREAKOUT_CS     10     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    8      // VS1053 Data/command select pin (output)
                               // These are the pins used for the music maker shield
#define SHIELD_RESET    -1     // VS1053 reset pin (unused!)
#define SHIELD_CS       7      // VS1053 chip select pin (output)
#define SHIELD_DCS      6      // VS1053 Data/command select pin (output)
#define LED             13
                               // These are common pins between breakout and shield
#define CARDCS          4      // Card chip select pin
                               // DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ            3      // VS1053 Data request, ideally an Interrupt pin
#define HOSTINTERRUPT   2      // Pin synchrinized with Controller TX

#define PLAYER_READY    (0xFF)
#define PLAYER_LISNING  (0xEE)

#define TRACK_STOP      (0x11)
#define TRACK_START     (0x0F)
#define TRACK_FIN       (0xF0)
#define NEXT_TRACK      (0x1F)

#define ALL_CONNECTED   (0xFF)
#define NO_CONNECTION   (0xAA)
#define CONNECTION_TIMEOUT (0xEE)

#define VOLUME          (20)
#define USEINTERRUPTS
//#define PLAYCONTINUOUS
//#define PLAYWITHSTOP
//#define PRINTLOG
#define PRINT_LOG(a) Serial.print(a);

uint8_t actualTrack = 0xFF;
bool operate_flag = true;
bool isConnecting = false;
bool isConnected = false;
bool fcnRec = false;
uint8_t command = 0;
uint8_t argument = 0;

volatile bool detectedInt = FALSE;
volatile uint8_t commandInt = 0;
volatile uint8_t argumentInt = 0;


Adafruit_VS1053_FilePlayer musicPlayer =
  // create breakout-example object!
  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
// create shield-example object!
//Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);


void setup() {
  delay(9000);
  Serial.begin(9600);
  Serial.println("Adafruit VS1053 Library Test");


  // initialise the music player
  if (! musicPlayer.begin()) { // initialise the music player
    Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
    while (1);
  }
  Serial.println(F("VS1053 found"));

  musicPlayer.sineTest(0x44, 500);    // Make a tone to indicate VS1053 is working

  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    while (1);  // don't do anything more
  }
  Serial.println("SD OK!");

  // list files
  printDirectory(SD.open("/"), 0);

  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(VOLUME, VOLUME);

  /***** Two interrupt options! *******/
  // This option uses timer0, this means timer1 & t2 are not required
  // (so you can use 'em for Servos, etc) BUT millis() can lose time
  // since we're hitchhiking on top of the millis() tracker
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);

  // This option uses a pin interrupt. No timers required! But DREQ
  // must be on an interrupt pin. For Uno/Duemilanove/Diecimilla
  // that's Digital #2 or #3
  // See http://arduino.cc/en/Reference/attachInterrupt for other pins
  // *** This method is preferred
  if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
    Serial.println(F("DREQ pin is not an interrupt pin"));

  // Two Interrupt pins :
  // For Player Purpose :   DREQ
  // For Host Puspose   :   HOSTINTERRUPT

  #ifdef USEINTERRUPTS
  // HOSTINTERRUPT - if pin change the value and then controller send 2bytes value via Serial1
  // interrupt will generate an event
  attachInterrupt(digitalPinToInterrupt(HOSTINTERRUPT), intHandlerHost, CHANGE);
  #endif
}

void loop() {

  Serial1.begin(9600);



  Serial.print("\nPLAYER LOOP STARTING");
  uint8_t rx_data[2];
  uint8_t rx_len = 2;

  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB
    Serial.print("PLAYER LOOP STARTING");
    delay(1000);
  }

  digitalWrite(LED, HIGH);
  musicPlayer.playFullFile("XSTART.mp3");
  /*
  if ((isConnecting == false) && (isConnected == false)) {
    musicPlayer.playFullFile("XSTART.mp3");
    delay(1000);
  }*/

  while (1) {

  }
}


void printDirectory(File dir, int numTabs) {
  while (true) {

    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      //Serial.println("**nomorefiles**");
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

/* *********************** FUNCTION *********************** */

void trackStopDelay (){
    for (int i = 0; i < (255 - VOLUME) ; i++) {
        musicPlayer.setVolume((VOLUME + i), (VOLUME + i));
        delay(10);
    }

    musicPlayer.stopPlaying();
    delay(56);
    musicPlayer.setVolume(VOLUME, VOLUME);

}

bool trackSelect(uint8_t track){
  switch (track){
  case 0x01:
      if (! musicPlayer.startPlayingFile("WODA.mp3")) {
        return FALSE;
      }
      break;
  case 0x02:
      if (! musicPlayer.startPlayingFile("KAMIEN.mp3")) {
        return FALSE;
      }
      break;
  case 0x03:
      if (! musicPlayer.startPlayingFile("DREWNO.mp3")) {
        return FALSE;
      }
      break;
  case 0x04:
      if (! musicPlayer.startPlayingFile("WIATR.mp3")) {
        return FALSE;
      }
      break;
  case 0x11:
      if (! musicPlayer.startPlayingFile("WODA_1.mp3")) {
        return FALSE;
      }
      break;
  case 0x21:
      if (! musicPlayer.startPlayingFile("KAMIEN_1.mp3")) {
        return FALSE;
      }
      break;
  case 0x12:
      if (! musicPlayer.startPlayingFile("DREWNO_1.mp3")) {
        return FALSE;
      }
      break;
  case 0x22:
      if (! musicPlayer.startPlayingFile("WIATR_1.mp3")) {
        return FALSE;
      }
      break;
  case 0x13:
      if (! musicPlayer.startPlayingFile("WODA_2.mp3")) {
        return FALSE;
      }
      break;
  case 0x23:
      if (! musicPlayer.startPlayingFile("KAMIEN_2.mp3")) {
        return FALSE;
      }
      break;
  case 0x14:
      }
      if (! musicPlayer.startPlayingFile("DREWNO_2.mp3")) {
        return FALSE;
      break;
  case 0x24:
      if (! musicPlayer.startPlayingFile("WIATR_2.mp3")) {
        return FALSE;
      }
    break;
  case NO_CONNECTION:
      if (! musicPlayer.startPlayingFile("XCONN.mp3")) {
        return FALSE;
      }
    break;
  case ALL_CONNECTED:
      if (! musicPlayer.playFullFile("XCONND.mp3")) {
        return FALSE;
      }
    break;
  case CONNECTION_TIMEOUT:
      if (! musicPlayer.playFullFile("XNOCONN.mp3")) {
        return FALSE;
      }
    break;
    default:
      return FALSE;
    break;
  }
  return TRUE;
}

bool trackPlay(uint8_t track){
  #ifdef PLAYCONTINUOUS
  if(!musicPlayer.playingMusic){
    if(trackSelect(track)){
      return TRUE;
    }else{
      return FALSE;
    }
  }else{
    return FALSE;
  }
  #elif PLAYWITHSTOP
  if(!musicPlayer.playingMusic){
    if(trackSelect(track)){
      return TRUE;
    }
  }else{
    trackStopDelay();
    if(trackSelect(track)){
      return TRUE;
    }else{
      return FALSE;
    }
  }
  #else
  if(!musicPlayer.playingMusic){
    if(trackSelect(track)){
      return TRUE;
    }
  }else{
    musicPlayer.stopPlaying();
    if(trackSelect(track)){
      return TRUE;
    }else{
      return FALSE;
    }
  }
  #endif
}

bool player (uint8_t fc_command, uint8_t fc_argument){
    uint8_t rxdata[2];

    while(1){
        if (fc_command == TRACK_START){
            if (trackPlayUntilRX(fc_argument, rxdata) == true){
                fc_command = rxdata[0];
                fc_argument = rxdata[1];
                Serial.print(fc_command);
                Serial.print(fc_argument);
            }
        }
        if (fc_command == NEXT_TRACK){
            musicPlayer.stopPlaying();
            if (trackPlayUntilRX(fc_argument, rxdata) == true){
                fc_command = rxdata[0];
                fc_argument = rxdata[1];
                Serial.print(fc_command);
                Serial.print(fc_argument);
            }
        }
        if (fc_command == TRACK_STOP){
            trackStopDelay();
            Serial.print("Stop playing");
            return true;
        }
        if ((fc_command == 0x00) && (fc_argument == 0x00)){
            Serial.print("\n PLAYER FALSE");
            return false;
        }
    }
}

void intCallback(bool detection){
  /*
  #define TRACK_STOP      (0x11)
  #define TRACK_START     (0x0F)
  #define TRACK_FIN       (0xF0)
  #define NEXT_TRACK      (0x1F)
  */
  if(detection){
    switch(commandInt){
      case TRACK_STOP :
        trackStopDelay();
      break;
      case TRACK_START :
        trackPlay(argumentInt);
      break;
      case TRACK_FIN :
        musicPlayer.stopPlaying();
      break;
      case NEXT_TRACK :
        trackPlay(argumentInt);
      break;
      default : Serial.print("Can't recognize !"); break;
    }
  }else{
    ;
  }
}

#ifdef USEINTERRUPTS
void intHandlerHost(void){
  uint8_t rx[2], len = 2;
  if((Serial1.readBytes(rx, len)) == 2){ //Received command and the argumet
    commandInt  = rx[0];
    argumentInt = rx[1];
    detectedInt = TRUE;                 //Interrupt TRUE
  }else{
    detectedInt = FALSE;                //Interrupt FALSE
  }
}
#endif

/*
into main loop (after while)
if ((musicPlayer.playingMusic == false) && (actualTrack != 0xFF)) {

  command = TRACK_START;
  argument = actualTrack;
  operate_flag = true;
}

if (((Serial1.readBytes(rx_data, rx_len)) == 2)) {
  operate_flag = true;
  command = rx_data[0];
  argument = rx_data[1];
  Serial.print("\nODEBRANO DANE:\n");
  Serial.print("command : ");
  Serial.print(command);
  Serial.print("\nargument : ");
  Serial.print(argument);

  if (operate_flag == true) {
      if (player(command, argument) == true){
      } else {
          Serial.print("\n Nie mozna odtworzyc utworu !\n");
      }
  }
  operate_flag = false;
}
}
*/
