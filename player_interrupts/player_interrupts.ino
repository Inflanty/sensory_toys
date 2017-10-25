/***************************************************
  This is an example for the Adafruit VS1053 Codec Breakout

  Designed specifically to work with the Adafruit VS1053 Codec Breakout
  ----> https://www.adafruit.com/products/1381

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
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
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 3       // VS1053 Data request, ideally an Interrupt pin

#define PLAYER_READY (0xFF)
#define PLAYER_LISNING (0xEE)

#define TRACK_STOP (0x11)
#define TRACK_START (0x0F)
#define TRACK_FIN (0xF0)

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


}

void loop() {

  Serial1.begin(9600);
  // Alternately, we can just play an entire file at once
  // This doesn't happen in the background, instead, the entire
  // file is played and the program will continue when it's done!
  //musicPlayer.playFullFile("DREWNO.MP3");
  // Start playing a file, then we can do stuff while waiting for it to finish


  /*if (! musicPlayer.startPlayingFile("CONNECTING.MP3")) {
    Serial.println("Could not open file start.mp3");
    while (1);
    }*/
  //Serial.println(F("Started playing"));
  /*while ((Serial1.readBytes(buffer, length)) == 0){
    Serial.println(".");
    musicPlayer.playFullFile("CONN.MP3");
    delay(2);
    }

    while ((Serial1.readBytes(buffer, length)) == 0){
    if(buffer[0] == NO_CONNECTION){
      while ((Serial1.readBytes(buffer, length)) == 0) {
    musicPlayer.playFullFile("CONN.MP3");
    }
    }

    //Serial.println("Done playing music");
    if(buffer[0] == ALL_CONNECTED){
    musicPlayer.startPlayingFile("CONND.MP3");
    }else{
    for(int i = 0; i <= 5; i++){
      musicPlayer.startPlayingFile("FAIL.MP3");
      delay(345);
    }

    goto start;
    }
    }*/
  /*
    while(1){
      Serial1.readBytes(buffer, length);

      switch (buffer[0]){
        case ALL_CONNECTED:
          musicPlayer.startPlayingFile("CONND.MP3");
        break;
        case NO_CONNECTION:
          musicPlayer.playFullFile("CONN.MP3");
        break;
        default:
        Serial.println("default command :");
        Serial.println((int) buffer[0]);
        break;
        delay(100);
      }
    }*/


  //loop_player_TAB();
  //}

  //void loop_player_TAB() {

  Serial.print("\nPLAYER LOOP STARTING");
  uint8_t rx_data[2];
  uint8_t rx_len = 2;

  //Serial1.begin(9600);
  while (!Serial1) {
    ; // wait for serial port to connect. Needed for native USB
    Serial.print("PLAYER LOOP STARTING");
    delay(1000);
  }


  delay(500);
  Serial.print("\nTEMP_FLAG!");

  if ((isConnecting == false) && (isConnected == false)) {
    musicPlayer.playFullFile("NOCONN.mp3");
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

      //LUB

      //command = TRACK_FIN;
      //argument = actualTrack;// ZMIANA NIE SPRAWDZONA
      /*uint8_t tx_data[2];
        tx_data[0] = TRACK_FIN;
        tx_data[1] = actualTrack; // ZMIANA NIE SPRAWDZONA
        operate_flag = false;

        delay(1);
        Serial1.write(tx_data, 2);
        Serial.print("\n Wyslano potwierdzenie konca utworu \n");
        Serial.print(tx_data[0]);
        Serial.print("\n");
        Serial.print(tx_data[1]);
        delay(14);*/
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


      char *track[11];

      if (operate_flag == true) {
        if (command == TRACK_STOP) {
          for (int i = 20; i > 0; i--) {
            musicPlayer.setVolume(i, i);
            delay(100);
          }
          musicPlayer.stopPlaying();
          musicPlayer.setVolume(20, 20);
        } else if (command == NO_CONNECTION) {
          Serial.print("Connecting");
          isConnecting = true;
          actualTrack = 0xAA;
          if (! musicPlayer.startPlayingFile("CONN.mp3")) {
            Serial.println("Could not open file");
            while (1);
          }
        } else if (command == ALL_CONNECTED) {
          Serial.print("All devices connected");
          isConnected = true;
          musicPlayer.playFullFile("CONND.mp3");

        } else if (command == CONNECTION_TIMEOUT) {
          Serial.print("Connection timeout");
          musicPlayer.playFullFile("FAIL.mp3");
        } else if (command == TRACK_START) {
          switch (argument) {
            case 0x01:
              actualTrack = 0x01;
              Serial.print("\n Playing 'drewno.mp3'");
              while ((Serial1.readBytes(rx_data, rx_len)) == 0) {
                if (musicPlayer.playingMusic == false) {
                  if (! musicPlayer.startPlayingFile("KAMIEN.mp3")) {
                    Serial.println("Could not open file");
                    while (1);
                  }
                }

              }
              command == rx_data[0];
              argument = rx_data[1];
              Serial.print(rx_data[1]);
              fcnRec = true;
              delay(2);
              if (rx_data[0] == TRACK_STOP) {
                musicPlayer.stopPlaying();
                Serial.println("Track stop sw");
              }
              break;
            case 0x02:
              *track = 'KAMIEN.mp3';
              break;
            case 0x03:
              *track = 'WIATR.mp3';
              break;
            case 0x04:
              *track = 'WODA.mp3';
              break;
            case 0x11:
              *track = 'track011.mp3';
              break;
            case 0x21:
              *track =  'track021.mp3';
              break;
            case 0x12:
              *track =  'track012.mp3';
              break;
            case 0x22:
              *track = 'track022.mp3';
              break;
            case 0x13:
              *track =  'track013.mp3';
              break;
            case 0x23:
              *track = 'track023.mp3';
              break;
            case 0x14:
              *track = 'track014.mp3';
              break;
            case 0x24:
              *track = 'track024.mp3';
              break;
            case 0xAA:
              Serial.print("Connecting");
              if (! musicPlayer.startPlayingFile("CONN.mp3")) {
                Serial.println("Could not open file");
                while (1);
              }
              break;
            default:
              Serial.println("XXX");
              break;
          }

          if (command == TRACK_STOP) {
            for (int i = 20; i > 0; i--) {
              musicPlayer.setVolume(i, i);
              delay(100);
            }
            musicPlayer.stopPlaying();
            Serial.println("Track stop");
            musicPlayer.setVolume(20, 20);

          }
        }
      }

      /*if ((command == NO_CONNECTION) || (command == TRACK_START)) {
        actualTrack = argument;
        } else {
        actualTrack = 0xFF;    //ZMIANA NIE SPRAWDZONA
        }*/
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

