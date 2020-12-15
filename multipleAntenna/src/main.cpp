//Library for controlling the M6E Nano module
#include "SparkFun_UHF_RFID_Reader.h" 

RFID nano, nano2, nano3, nano4;

#define SERIAL1 SerialUSB
#define SERIAL2 Serial1  //19, 18
#define SERIAL3 Serial3  //15, 14
#define SERIAL4 Serial2  //17, 16
#define SERIAL5 Serial   //0, 1
//タグの枚数を変更した時は下記を変更する
#define tagsNum 100
#define idNum 5
#define INTERVAL 10

//関数定義
boolean setupNano(long baudRate);
boolean setupNano2(long baudRate);
boolean setupNano3(long baudRate);
boolean setupNano4(long baudRate);
void multipleAntennaDetection(char out_CurrentTags[tagsNum][idNum]);
void detectDefferentialTags();
void showAllTags();
char global_CurrentTags[tagsNum][idNum] = {0};
char arrayNum[tagsNum][idNum] = {0};
char global_DifferenceTags[tagsNum][idNum] = {0};

unsigned long long time_before = 0;
unsigned long long time_after  = 0;
unsigned long long time_length = 0;
int arrayNumber = sizeof(arrayNum) / sizeof(arrayNum[0]);
int time_count   = 0;
int diff_count   = 0;
int switchPin    = 8;
int switchPin2   = 9;
int limitSwitch  = 52;
int first_detect = 0;


void setup()
{
  SERIAL1.begin(115200);

  pinMode(switchPin, INPUT);
  pinMode(switchPin2, INPUT);
  pinMode(limitSwitch, INPUT);
  
  while (!SERIAL1); //Wait for the SERIAL1 port to come online

  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    SERIAL1.println(F("Module failed to respond. Please check wiring."));
    while (1); //Freeze!
  }
  nano.setRegion(REGION_NORTHAMERICA); //Set to North America
  nano.setReadPower(2700); //5.00 dBm. Higher values may caues USB port to brown out

  if (setupNano2(38400) == false) //Configure nano to run at 38400bps
  {
    SERIAL1.println(F("Module2 failed to respond. Please check wiring."));
    while (1); //Freeze!
  }
 
  nano2.setRegion(REGION_NORTHAMERICA);
  nano2.setReadPower(2700);

  if (setupNano3(38400) == false) //Configure nano to run at 38400bps
  {
    SERIAL1.println(F("Module3 failed to respond. Please check wiring."));
    while (1); //Freeze!
  }
 
  nano3.setRegion(REGION_NORTHAMERICA);
  nano3.setReadPower(2700);

  if (setupNano4(38400) == false) //Configure nano to run at 38400bps
  {
    SERIAL1.println(F("Module4 failed to respond. Please check wiring."));
    while (1); //Freeze!
  }
 
  nano4.setRegion(REGION_NORTHAMERICA);
  nano4.setReadPower(2700);
  
  SERIAL1.println(F("Start Detection..."));

}

void loop(){
  
  char CurrentTags[tagsNum][idNum] = {0};
  nano.stopReading();
  nano2.stopReading();
  nano3.stopReading();
  nano4.stopReading();

  // 初期状態を読み込み
  if(first_detect == 0){
    SERIAL1.println("First detection is ...");
    // 下記もglobal変数を渡せば良さそう
    multipleAntennaDetection(CurrentTags);
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(global_CurrentTags[i]);
    }
    nano.stopReading();
    nano2.stopReading();
    nano3.stopReading();
    nano4.stopReading();
    first_detect++;
  
  }

  if(first_detect != 0){
      SERIAL1.println("Detection is ...");
      memset(global_DifferenceTags, '\0', sizeof(global_DifferenceTags));
      detectDefferentialTags();
      SERIAL1.println("Result is ...");
      for (int i = 0; i < arrayNumber; i++){
        SERIAL1.println(global_DifferenceTags[i]);
      }

      SERIAL1.println("All tags are ...");
      for (int i = 0; i < arrayNumber; i++){
        SERIAL1.println(global_CurrentTags[i]);
      }

      nano.stopReading();
      nano2.stopReading();
      nano3.stopReading();
      nano4.stopReading();

      diff_count++;
  }

  delay(100);
}

void multipleAntennaDetection(char TemporaryTags[tagsNum][idNum]){
  int k = 1;
  int jj = 0;
  memset(global_DifferenceTags, '\0', sizeof(global_DifferenceTags));

  nano.startReading(); //Begin scanning for tags
  float now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;

    if (nano.check() == true) { 
      //Check to see if any new data has come in from module
      byte responseType = nano.parseResponse(); 
      //Break response into tag ID, RSSI, frequency, and timestamp
      if (responseType == RESPONSE_IS_KEEPALIVE){
        SERIAL1.println(F("Scanning"));
      }else if (responseType == RESPONSE_IS_TAGFOUND){
        //Get the number of bytes of EPC from response
        byte tagEPCBytes = nano.getTagEPCBytes(); 
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {

          // if (nano.msg[31 + x] < 0x10) {
          //   SERIAL1.print(F("0"));
          // } 

          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }

        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            //重複がみつかったらカウントする
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          //重複なしなら配列wineTags2に順次格納
          if (count == 0){
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE)
      {
        SERIAL1.println("Bad CRC");
      }
      else
      {
        //Unknown response
        SERIAL1.print("Unknown error");
      }
    }
  }
  SERIAL1.println("Initial setting is done...");
  //発熱防止のため計測がおわったら、stopReading()をいれる。
  nano.stopReading();
  SERIAL1.println("Stop module...");

  //k = 1;
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");
  
  //2つ目のモジュール計測開始
  SERIAL1.println("Start module2");
  nano2.startReading();
  now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano2.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano2.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano2.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano2.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }        
        // 1個目に格納するための処理
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }
        
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  nano2.stopReading();
  SERIAL1.println("Initial2 setting is done...");
  SERIAL1.println("Stop module2...");

  //k = 1;
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //3つ目のモジュール計測開始
  SERIAL1.println("Start module3");
  nano3.startReading();
  now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano3.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano3.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano3.getTagEPCBytes(); //Get the number of bytes of EPC from response
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano3.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }        
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  nano3.stopReading();
  SERIAL1.println("Initial3 setting is done...");
  SERIAL1.println("Stop module3...");

  //k = 1;
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //4つ目のモジュール計測開始
  SERIAL1.println("Start module3");
  nano4.startReading();
  now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano4.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano4.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano4.getTagEPCBytes(); //Get the number of bytes of EPC from response
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano4.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }
        
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  nano4.stopReading();
  SERIAL1.println("Initial4 setting is done...");
  SERIAL1.println("Stop module4...");

  //k = 1;
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //現在の状況をglobal変数に格納する
  for(int i = 0; i<arrayNumber; i++){
      strcpy(global_CurrentTags[i],TemporaryTags[i]);
  }
}

//ワインの差分の検出部分
//global_CurrentTags消す
void detectDefferentialTags(){
  int k = 1;
  int jj = 0;
  int dz = 0;
  SERIAL1.println("Detecting...");
  char TemporaryTags[tagsNum][idNum] = {0};

  nano.startReading(); //Begin scanning for tags
  float now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;

    if (nano.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            //重複がみつかったらカウントする
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          //重複なしなら配列wineTags2に順次格納
          if (count == 0){
            //strcpy 配列にchar型のEPCを格納する関数
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }        
      }
      //複数タグのレスポンスが重なると下記が表示されるようだ。
      else if (responseType == ERROR_CORRUPT_RESPONSE)
      {
        SERIAL1.println("Bad CRC");
      }
      else
      {
        //Unknown response
        SERIAL1.print("Unknown error");
      }
    }
  }
  SERIAL1.println("Initial setting is done...");
  //発熱防止に計測がおわったら、stopReading()をいれる。
  nano.stopReading();
  SERIAL1.println("Stop module...");

  // k = 1;
  // jj = 0;
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //2つ目のモジュール計測開始
  SERIAL1.println("Start module2");
  nano2.startReading();
  now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano2.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano2.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano2.getTagEPCBytes(); //Get the number of bytes of EPC from response
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano2.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  nano2.stopReading();
  SERIAL1.println("Initial2 setting is done...");
  SERIAL1.println("Stop module2...");
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //3つ目のモジュール計測開始
  SERIAL1.println("Start module3");
  nano3.startReading();
  now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano3.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano3.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano3.getTagEPCBytes(); //Get the number of bytes of EPC from response
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano3.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  nano3.stopReading();
  SERIAL1.println("Initial3 setting is done...");
  SERIAL1.println("Stop module3...");
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //4つ目のモジュール計測開始
  SERIAL1.println("Start module3");
  nano4.startReading();
  now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano4.check() == true) //Check to see if any new data has come in from module
    {
      byte responseType = nano4.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp

      if (responseType == RESPONSE_IS_KEEPALIVE)
      {
        SERIAL1.println(F("Scanning"));
      }
      else if (responseType == RESPONSE_IS_TAGFOUND)
      {
        byte tagEPCBytes = nano4.getTagEPCBytes(); //Get the number of bytes of EPC from response
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          String epc = String(nano4.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        if(jj == 0){
          strcpy(TemporaryTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }
        
      }
      else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  nano4.stopReading();
  SERIAL1.println("Initial4 setting is done...");
  SERIAL1.println("Stop module4...");
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //////////////////////////
  //下記が差分検出のアルゴリズム
  //////////////////////////

    for( int di = 0; di<arrayNumber; di++){
      int dk = 0;
      for( int dj = 0; dj<arrayNumber; dj++){
        if(strcmp(global_CurrentTags[di], TemporaryTags[dj]) == 0){
          dk = dk + 1;
        }
      }
      if(dk == 0){
        strcpy(global_DifferenceTags[dz], global_CurrentTags[di]); 
        SERIAL1.println(global_DifferenceTags[dz]);
        dz += 1;       
      }
    }

    //現在の状況をglobal変数に格納する
    SERIAL1.println("Renew global_CurrentTags");
    for(int i = 0; i<arrayNumber; i++){
      strcpy(global_CurrentTags[i],TemporaryTags[i]);
    }

    SERIAL1.println("detectDefferentialTags Done...");
}


//////////////////////////////////////////////////////////////
//Gracefully handles a reader that is already configured and already reading continuously
//Because Stream does not have a .begin() we have to do this outside the library
boolean setupNano(long baudRate)
{
  nano.begin(SERIAL2); //Tell the library to communicate over software SERIAL1 port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  SERIAL2.begin(baudRate); //For this test, assume module is already at our desired baud rate
  //while (SERIAL2.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (SERIAL2.available()) SERIAL2.read();

  nano.getVersion();

  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano.stopReading();

    SERIAL1.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    SERIAL2.begin(115200); //Start software SERIAL1 at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    SERIAL2.begin(baudRate); //Start the software SERIAL1 port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano.setTagProtocol(); //Set protocol to GEN2

  nano.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}

//////////////////////////////////////////////////////////
//2つ目のRFIDモジュールを使うため
boolean setupNano2(long baudRate)
{
  nano2.begin(SERIAL3); //Tell the library to communicate over software SERIAL1 port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  SERIAL3.begin(baudRate); //For this test, assume module is already at our desired baud rate
  //while (SERIAL2.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (SERIAL3.available()) SERIAL3.read();

  nano2.getVersion();

  if (nano2.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano2.stopReading();

    SERIAL1.println(F("Module 2 continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    SERIAL3.begin(115200); //Start software SERIAL1 at 115200

    nano2.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    SERIAL3.begin(baudRate); //Start the software SERIAL1 port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano2.getVersion();
  if (nano2.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano2.setTagProtocol(); //Set protocol to GEN2

  nano2.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}
///////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//3つ目のRFIDモジュールを使うため
boolean setupNano3(long baudRate)
{
  nano3.begin(SERIAL4); //Tell the library to communicate over software SERIAL1 port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  SERIAL4.begin(baudRate); //For this test, assume module is already at our desired baud rate
  //while (SERIAL2.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (SERIAL4.available()) SERIAL4.read();

  nano3.getVersion();

  if (nano3.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano3.stopReading();

    SERIAL1.println(F("Module 3 continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    SERIAL4.begin(115200); //Start software SERIAL1 at 115200

    nano3.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    SERIAL4.begin(baudRate); //Start the software SERIAL1 port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano3.getVersion();
  if (nano3.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano3.setTagProtocol(); //Set protocol to GEN2

  nano3.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}
///////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//4つ目のRFIDモジュールを使うため
boolean setupNano4(long baudRate)
{
  nano4.begin(SERIAL4); //Tell the library to communicate over software SERIAL1 port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  SERIAL5.begin(baudRate); //For this test, assume module is already at our desired baud rate
  //while (SERIAL2.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (SERIAL5.available()) SERIAL5.read();

  nano3.getVersion();

  if (nano4.msg[0] == ERROR_WRONG_OPCODE_RESPONSE)
  {
    //This happens if the baud rate is correct but the module is doing a ccontinuous read
    nano4.stopReading();

    SERIAL1.println(F("Module 3 continuously reading. Asking it to stop..."));

    delay(1500);
  }
  else
  {
    //The module did not respond so assume it's just been powered on and communicating at 115200bps
    SERIAL5.begin(115200); //Start software SERIAL1 at 115200

    nano4.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    SERIAL5.begin(baudRate); //Start the software SERIAL1 port, this time at user's chosen baud rate

    delay(250);
  }

  //Test the connection
  nano4.getVersion();
  if (nano4.msg[0] != ALL_GOOD) return (false); //Something is not right

  //The M6E has these settings no matter what
  nano4.setTagProtocol(); //Set protocol to GEN2

  nano4.setAntennaPort(); //Set TX/RX antenna ports to 1

  return (true); //We are ready to rock
}
///////////////////////////////////////////////////////////