/***************************************************
*PLAYER
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
#define SHIELD_RESET  -1      // VS1053 reset pin (unused!)
#define SHIELD_CS     7      // VS1053 chip select pin (output)
#define SHIELD_DCS    6      // VS1053 Data/command select pin (output)

// These are common pins between breakout and shield
#define CARDCS 4     // Card chip select pin
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

#define PLAYER_READY (0xFF)
#define PLAYER_LISNING (0xEE)

#define TRACK_STOP (0x11)
#define TRACK_START (0x0F)
#define TRACK_FIN (0xF0)
#define NEXT_TRACK (0x1F)

#define ALL_CONNECTED (0xFF)
#define NO_CONNECTION (0xAA)
#define CONNECTION_TIMEOUT (0xEE)

uint8_t actualTrack = 0xFF;
bool operate_flag = true;
bool isConnecting = false;
bool isConnected = false;
bool fcnRec = false;
uint8_t command = 0;
uint8_t argument = 0;

Adafruit_VS1053_FilePlayer musicPlayer =
  // create breakout-example object!
  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);
// create shield-example object!
//Adafruit_VS1053_FilePlayer(SHIELD_RESET, SHIELD_CS, SHIELD_DCS, DREQ, CARDCS);

////

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
  musicPlayer.setVolume(20, 20);

  /***** Two interrupt options! *******/
  //musicPlayer.useInterrupt(VS1053_FILEPLAYER_TIMER0_INT);

  if (! musicPlayer.useInterrupt(VS1053_FILEPLAYER_PIN_INT))
    Serial.println(F("DREQ pin is not an interrupt pin"));

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
    
  if ((isConnecting == false) && (isConnected == false)) {
    musicPlayer.playFullFile("XSTART.mp3");
    delay(1000);
  }

  while (1) {

    if ((musicPlayer.playingMusic == false) && (actualTrack != 0xFF)) {
      /*
        2 mozliwosci restartowania zakonczonego utworu,
        1 opcja - restartujemy utwor bez kontaktu z esp_klientem
        2 opcja - wysylamy info odnosnie aktualnie zakonczonego utworu do esp_klient
      */
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
          if (player() == true){
              
          } else {
              Serial.print("\n Nie mozna odtworzyc utworu !\n");
          }
      }
      operate_flag = false;
    }
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

/* FUNCKJE DO PROGRAMU */

uint8_t* trackPlayUntilRX (uint8_t track){
    uint8_t rx[2], faul_tab[] = {0x00, 0x00};
    int len = 2;
    bool faul = false;
    while (((Serial1.readBytes(rx, len)) == 0) && (faul == false)) {
        if (musicPlayer.playingMusic == false) {
            switch (track){
            case 0x01:
                if (! musicPlayer.startPlayingFile("WODA.mp3")) {
                    faul = true;
                }
                break;
            case 0x02:
                if (! musicPlayer.startPlayingFile("KAMIEN.mp3")) {
                    faul = true;
                }
                break;
            case 0x03:
                if (! musicPlayer.startPlayingFile("DREWNO.mp3")) {
                    faul = true;
                }
                break;
            case 0x04:
                if (! musicPlayer.startPlayingFile("WIATR.mp3")) {
                    faul = true;
                }
                break;
            case 0x11:
                if (! musicPlayer.startPlayingFile("WODA_1.mp3")) {
                    faul = true;
                }
                break;
            case 0x21:
                if (! musicPlayer.startPlayingFile("KAMIEN_1.mp3")) {
                    faul = true;
                }
                break;
            case 0x12:
                if (! musicPlayer.startPlayingFile("DREWNO_1.mp3")) {
                    faul = true;
                }
                break;
            case 0x22:
                if (! musicPlayer.startPlayingFile("WIATR_1.mp3")) {
                    faul = true;
                }
                break;
            case 0x13:
                if (! musicPlayer.startPlayingFile("WODA_2.mp3")) {
                    faul = true;
                }
                break;
            case 0x23:
                if (! musicPlayer.startPlayingFile("KAMIEN_2.mp3")) {
                    faul = true;
                }
                break;
            case 0x14:
                if (! musicPlayer.startPlayingFile("DREWNO_2.mp3")) {
                    faul = true;
                }
                break;
            case 0x24:
                if (! musicPlayer.startPlayingFile("WIATR_2.mp3")) {
                    faul = true;
                }
              break;
            case NO_CONNECTION:
                if (! musicPlayer.startPlayingFile("XCONN.mp3")) {
                    faul = true;
                }
              break;
            case ALL_CONNECTED:
                if (! musicPlayer.startPlayingFile("XCONND.mp3")) {
                    faul = true;
                }
              break;
            case CONNECTION_TIMEOUT:
                if (! musicPlayer.startPlayingFile("XNOCONN.mp3")) {
                    faul = true;
                }
              break;
            default:
                    faul = true;
              break;    
            }
            actualTrack = track;
        }
    }
    delay(1);
    /* ODEBRANE DANE */
    if (faul == true){
    return faul_tab
    }
    return rx;
}

void trackStopDelay (int deleay_time){
    for (int i = 20; i > 0; i--) {
        musicPlayer.setVolume(i, i);
        delay(delay_time/20);
    }
    musicPlayer.stopPlaying();
    musicPlayer.setVolume(20, 20);
}

void trackNext (uint8_t nexttrack){
    musicPlayer.stopPlaying();
    trackPlay(naxttrack);
}

bool player (uint8_t fc_command, uint8_t fc_argument){
    uint8_t* rxdata[2];
    
    if (fc_command == TRACK_PLAY){
        rxdata = trackPlayUntilRX(fc_argument);
        fc_command = rxdata[0];
        fc_argument = rxdata[1];
    }
    if (fc_command == NEXT_TRACK){
        rxdata = trackPlayUntilRX(fc_argument);
        fc_command = rxdata[0];
        fc_argument = rxdata[1];
    }
    if (fc_command == TRACK_STOP){
        trackStop(400);
    }
    if ((fc_command == 0x00) && (fc_argument == 0x00)){
        Serial.print("\n PLAYER FALSE");
        return false;
    }
    
    return true;
}

