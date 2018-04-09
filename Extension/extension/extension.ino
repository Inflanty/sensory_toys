/*
This is code for Extension purpose in sensoric game 
There is sendAddress() for sending address of extension for server's usability
There is receiveAdress() for receive address of servers which cooperate with.
*/
#define IRDA Serial1
#define MY_ADDRESS 0x10

#define WATER 0x01
#define STONE 0x02
#define WOODEN 0x03
#define AIR 0x04

void setup() {
  // put your setup code here, to run once:
  

}

void loop() {
  // put your main code here, to run repeatedly:

  IRDA.begin(9600);
  int recData = 0;

  
  Serial.print("\nPLAYER LOOP STARTING");
  uint8_t rx_data[2];
  uint8_t rx_len = 2;

  while (!IRDA) {
    ; // wait for serial port to connect. Needed for native USB
    Serial.print("PLAYER LOOP STARTING");
    delay(1000);
  }
  // _________________________________MAIN LOOP_________________________________
  while(1){
    recData = receiveAddress();
    if (recData != 0){
      switch(recData){
        case WATER:

        break;
        case STONE:

        break;
        case WOODEN:

        break;
        case AIR:

        break;
        default:

        break;
      }
    }else if (recData == 0){
      sendAddress();
    }else {
      
    }
    delay(1000);
  }
  // _________________________________MAIN LOOP_________________________________
}

void sendAddress(){
  IRDA.write(MY_ADDRESS);
}

int receiveAddress(){
  int recData = 0;
  if(IRDA.available() > 0){
    //Mayby SerialX.available() is not working on Leonardo...
    recData = IRDA.read();
  }
  return recData;
}

