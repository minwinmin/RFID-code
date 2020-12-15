//class、参照渡しをつかってプログラムをよりシンプルにするように心がける
//RFIDモジュールを4つ使ったパターン

//#include <SoftwareSERIAL1.h> //Used for transmitting to the device
//SoftwareSERIAL1 SERIAL2(2, 3); //RX, TX
//#include "setting.hpp"

#include <algorithm>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

////////////////////////////////
//有線LAN接続処理
////////////////////////////////
#include <SPI.h>
#include <Ethernet2.h>
// シールドのMACアドレス
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED // ここをEthernet Shieldに書いてあるMacアドレスに変更
};
// Arduinoの固定IP
IPAddress ip(192, 168, 1, 177); // ここを適当なIPアドレス（ルーター内で空いているもの）に変更
// PCのIPアドレスを入力すること
// 無線環境を変えたら下記を変更すること
char server[] = "192.168.11.60"; // ここを適当なIPアドレス（PCのIPアドレス）に変更
// クライアント
EthernetClient client;
////////////////////////////////

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
#define INTERVAL 20

//関数定義
boolean setupNano(long baudRate);
boolean setupNano2(long baudRate);
boolean setupNano3(long baudRate);
boolean setupNano4(long baudRate);
void multipleAntennaDetection(char out_CurrentWineTags[tagsNum][idNum]);
void detectDefferentialWines();
void detectWinebottles();
void detectWinebottles2();
void showAllTags();
void detectDefferentialWines();
void postAPI();

char global_CurrentWineTags[tagsNum][idNum] = {0};
//char out_TemporaryWineTags[tagsNum][idNum] = {};
char arrayNum[tagsNum][idNum] = {0};
char global_DifferenceWineTags[tagsNum][idNum] = {0};
//3個ごとに判断して検出できているかどうかみる

char originalID[][tagsNum][idNum] = {{"952b", "a234", "a2c", "xxxx"}, {"947b", "a0ca", "a0c9", "a1db"}, {"a0d2", "955c", "a0aa",  "a279"}, 
                                   {"94fb", "a0d1", "a0b2", "a22c"}, {"a13" , "94d4", "a12" , "a233"}, {"a0da", "9524", "a0fc", "xxxx"},
                                   {"a0e2", "a0fb", "94a3", "a214"}, {"952c", "a132", "a133", "a23"}, {"a113", "a12c", "94ca", "a299"}, 
                                   {"a12b", "a1c" , "9554","a273"}, {"a169", "943a", "a15c", "xxxx"}, {"9433", "a149", "a16a", "a24"},
                                   {"a151", "93e2", "a164", "a2ac"}, {"a1a2", "a1a1", "941" , "a2a1"}, {"a19b", "a181", "949a", "a269"}, 
                                   {"a17a", "a19a", "94f3", "xxxx"}, {"a1d1", "a1d2", "9561", "a24a"}, {"a1b2", "a1cb", "958b", "a26a"},
                                   {"a1ca", "a1ab", "953c", "a22b"}, {"a1fb", "a1fc", "94c3", "a2b4"}};

bool boolID[] = {false, false, false,false, false, 
                 false, false, false,false, false, 
                 false, false, false,false, false,
                 false, false, false,false, false, 
                 false, false, false,false, false};

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
  
  //有線LAN接続処理//
  // if (Ethernet.begin(mac) == 0) {
  //   SERIAL1.println("Failed to configure Ethernet using DHCP");
  //   Ethernet.begin(mac, ip);
  // }
  /////////////////

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
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling

  SERIAL1.println(originalID[10][0]);

  SERIAL1.println(F("Press a key to begin scanning for tags."));
  while (!SERIAL1.available()); //Wait for user to send a character
  SERIAL1.read(); //Throw away the user's character

  // 初期タグ取得
  //multipleAntennaDetection();

}

void loop(){
  // 下記はループ処理なのですぐに初期化されてしまう
  
  char CurrentWineTags[tagsNum][idNum] = {0};
  nano.stopReading();
  nano2.stopReading();
  nano3.stopReading();
  nano4.stopReading();

  // 初期状態を読み込み
  if(first_detect == 0){
    SERIAL1.println("First detection is ...");
    // 下記もglobal変数を渡せば良さそう
    multipleAntennaDetection(CurrentWineTags);
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(global_CurrentWineTags[i]);
    }
    nano.stopReading();
    nano2.stopReading();
    nano3.stopReading();
    nano4.stopReading();
    first_detect++;

    //boolIDを初期化
    bool boolID[] = {false, false, false,false, false, 
                    false, false, false,false, false, 
                    false, false, false,false, false,
                    false, false, false,false, false, 
                    false, false, false,false, false};

    for(int a=0; a<20; a++){
      for(int b=0; b<4; b++){
        for(int c=0; c<arrayNumber; c++){
          //1つではcontinueを書いて処理から抜け出しTrueにする。
          if(strcmp(originalID[a][b], global_CurrentWineTags[c]) == 0){
            SERIAL1.println("true!");
            boolID[a] = true;
            break;
          }
        }
      }
    }
    SERIAL1.println("");
    SERIAL1.print("4段目");SERIAL1.print(boolID[0]);SERIAL1.print(boolID[1]);SERIAL1.print(boolID[2]);SERIAL1.print(boolID[3]);SERIAL1.println(boolID[4]);
    SERIAL1.print("3段目");SERIAL1.print(boolID[5]);SERIAL1.print(boolID[6]);SERIAL1.print(boolID[7]);SERIAL1.print(boolID[8]);SERIAL1.println(boolID[9]);
    SERIAL1.print("2段目");SERIAL1.print(boolID[10]);SERIAL1.print(boolID[11]);SERIAL1.print(boolID[12]);SERIAL1.print(boolID[13]);SERIAL1.println(boolID[14]);      
    SERIAL1.print("1段目");SERIAL1.print(boolID[15]);SERIAL1.print(boolID[16]);SERIAL1.print(boolID[17]);SERIAL1.print(boolID[18]);SERIAL1.println(boolID[19]);
    SERIAL1.println("");
  
  }

  // 本命処理
  // if (digitalRead(limitSwitch) == 1){
  //   if (time_count == 0){
  //     time_before = millis() / 1000;
  //     time_count++;
  //     SERIAL1.println("lock");
  //   }
  //   if(diff_count == 0 && ((millis()/1000 - time_before) > 10)){
  //     memset(global_DifferenceWineTags, '\0', sizeof(global_DifferenceWineTags));
  //     detectDefferentialWines();
  //     SERIAL1.println("Result is ...");
  //     for (int i = 0; i < arrayNumber; i++){
  //       SERIAL1.println(global_DifferenceWineTags[i]);
  //     }

  //     SERIAL1.println("All tags are ...");
  //     for (int i = 0; i < arrayNumber; i++){
  //       SERIAL1.println(global_CurrentWineTags[i]);
  //     }
  //     diff_count++;
  //   }
  // }else{
  //   // 扉が開いている時の処理
  //   // SERIAL1.println("open");
  //   // 変数初期化
  //   time_count = 0;
  //   diff_count = 0;
  // }

  int buttonState = digitalRead(switchPin);
  if(buttonState == HIGH){
    SERIAL1.println("Detection is ...");
    // 検証用
    //multipleAntennaDetection(global_CurrentWineTags);
    // 差分検知
    // グローバル変数 global_DifferenceWineTagsを初期化する
    memset(global_DifferenceWineTags, '\0', sizeof(global_DifferenceWineTags));
    detectDefferentialWines();
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(global_DifferenceWineTags[i]);
    }

    SERIAL1.println("All tags are ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(global_CurrentWineTags[i]);
    }

    nano.stopReading();
    nano2.stopReading();
    nano3.stopReading();
    nano4.stopReading();

    //boolIDを初期化
    bool boolID[] = {false, false, false,false, false, 
                    false, false, false,false, false, 
                    false, false, false,false, false,
                    false, false, false,false, false, 
                    false, false, false,false, false};

    //差分取得
    for(int a=0; a<20; a++){
      for(int b=0; b<4; b++){
        for(int c=0; c<arrayNumber; c++){
          //1つではcontinueを書いて処理から抜け出しTrueにする。
          if(strcmp(originalID[a][b], global_CurrentWineTags[c]) == 0){
            SERIAL1.println("true!");
            boolID[a] = true;
            continue;
          }
        }
      }
    }
    SERIAL1.println("");
    SERIAL1.print("4段目");SERIAL1.print(boolID[0]);SERIAL1.print(boolID[1]);SERIAL1.print(boolID[2]);SERIAL1.print(boolID[3]);SERIAL1.println(boolID[4]);
    SERIAL1.print("3段目");SERIAL1.print(boolID[5]);SERIAL1.print(boolID[6]);SERIAL1.print(boolID[7]);SERIAL1.print(boolID[8]);SERIAL1.println(boolID[9]);
    SERIAL1.print("2段目");SERIAL1.print(boolID[10]);SERIAL1.print(boolID[11]);SERIAL1.print(boolID[12]);SERIAL1.print(boolID[13]);SERIAL1.println(boolID[14]);      
    SERIAL1.print("1段目");SERIAL1.print(boolID[15]);SERIAL1.print(boolID[16]);SERIAL1.print(boolID[17]);SERIAL1.print(boolID[18]);SERIAL1.println(boolID[19]);
    SERIAL1.println("");
  }

  delay(100);
}

void multipleAntennaDetection(char TemporaryWineTags[tagsNum][idNum]){
  int k = 1;
  int jj = 0;
  memset(global_DifferenceWineTags, '\0', sizeof(global_DifferenceWineTags));

  nano.startReading(); //Begin scanning for tags
  float now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;

    if (nano.check() == true) { //Check to see if any new data has come in from module
      byte responseType = nano.parseResponse(); //Break response into tag ID, RSSI, frequency, and timestamp
      if (responseType == RESPONSE_IS_KEEPALIVE){
        SERIAL1.println(F("Scanning"));
      }else if (responseType == RESPONSE_IS_TAGFOUND){
        //If we have a full record we can pull out the fun bits
        // int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read
        // long freq = nano.getTagFreq(); //Get the frequency this tag was detected at
        // long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message
        byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response

        //Print EPC bytes, this is a subsection of bytes from the response/msg array
        //SERIAL1.print(F(" epc["));
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
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();

        //重要なコード部分、タグを重複なしで配列wineTags2に格納する
        //ここが配列に格納するタイミングかな
        //string型をcharに変換して配列に挿入
        //参考 https://teratail.com/questions/44651
        //char temp = uniqueEPC.c_str();
        //初期状態はEPCが格納されていないので、まず挿入。ただし、良い書き方がありそう。

        //しも4桁に絞る
        //uniqueEPC = uniqueEPC.substring(21,24);
        // 1個目に格納するための処理
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          //arrayNumberの数は最初から決まっている
          //つまり、配列wineTags2の初期値が入力されているから。
          for (int i = 0; i<arrayNumber; i++){
            //charからstringへの変更、下記が参考になった
            //https://detail.chiebukuro.yahoo.co.jp/qa/question_detail/q13177596828
            //下記怪しい気がする
            String s_temp = TemporaryWineTags[i];
            //重複がみつかったらカウントする
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          //重複なしなら配列wineTags2に順次格納
          if (count == 0){
            //strcpy 配列にchar型のEPCを格納する関数
            
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            //配列の要素数がいっぱいになった時の処理を書く
            k += 1;
          }
        }

        //格納されているタグを表示する部分
        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags[j]);
        // }
        
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
        //If we have a full record we can pull out the fun bits
        int rssi = nano2.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano2.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano2.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano2.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano2.msg[31 + x] < 0x10){
          //   SERIAL1.print(F("0")); //Pretty print
          // } 
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano2.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //最後にnullを加えると良さそう？
        //文字列の終わり指定する
        //uniqueEPC.concat("/0");
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();
        
        // 1個目に格納するための処理
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryWineTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }

        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags[j]);
        // }
        
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
        //If we have a full record we can pull out the fun bits
        int rssi = nano3.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano3.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano3.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano3.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano2.msg[31 + x] < 0x10){
          //   SERIAL1.print(F("0")); //Pretty print
          // } 
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano3.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //最後にnullを加えると良さそう？
        //文字列の終わり指定する
        //uniqueEPC.concat("/0");
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();
        
        // 1個目に格納するための処理
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryWineTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }

        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags[j]);
        // }
        
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
        //If we have a full record we can pull out the fun bits
        int rssi = nano4.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano4.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano4.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano4.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano2.msg[31 + x] < 0x10){
          //   SERIAL1.print(F("0")); //Pretty print
          // } 
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano4.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //最後にnullを加えると良さそう？
        //文字列の終わり指定する
        //uniqueEPC.concat("/0");
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();
        
        // 1個目に格納するための処理
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryWineTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }

        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags[j]);
        // }
        
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
      strcpy(global_CurrentWineTags[i],TemporaryWineTags[i]);
  }
}

//ワインの差分の検出部分
//global_CurrentWineTags消す
void detectDefferentialWines(){
  int k = 1;
  int jj = 0;
  int dz = 0;
  SERIAL1.println("Detecting...");
  char TemporaryWineTags[tagsNum][idNum] = {0};
  // それぞれのモジュールに代入して格納していくイメージ
  // 最後にこれを結合したい...そちらの方が確実な気がする
  char TemporaryWineTags2[tagsNum][idNum] = {0};
  char TemporaryWineTags3[tagsNum][idNum] = {0};
  char TemporaryWineTags4[tagsNum][idNum] = {0};

  char TemporaryWineTags5[tagsNum][idNum] = {0};
  
  //////////////////////////////////////////////////
  //ここで取得したものが何故かのこっている、初期化しているはずなのだけれど...?
  //1つ目のモジュール計測開始
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
        
        //If we have a full record we can pull out the fun bits
        // int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read

        // long freq = nano.getTagFreq(); //Get the frequency this tag was detected at

        // long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano.getTagEPCBytes(); //Get the number of bytes of EPC from response

        //Print EPC bytes, this is a subsection of bytes from the response/msg array
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();

        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          //arrayNumberの数は最初から決まっている
          //つまり、配列wineTags2の初期値が入力されているから。
          for (int i = 0; i<arrayNumber; i++){
            //charからstringへの変更、下記が参考になった
            //https://detail.chiebukuro.yahoo.co.jp/qa/question_detail/q13177596828
            //下記怪しい気がする
            String s_temp = TemporaryWineTags[i];
            //重複がみつかったらカウントする
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          //重複なしなら配列wineTags2に順次格納
          if (count == 0){
            //strcpy 配列にchar型のEPCを格納する関数
            
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            //配列の要素数がいっぱいになった時の処理を書く
            k += 1;
          }
        }

        //格納されているタグを表示する部分
        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags[j]);
        // }
        
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
        //If we have a full record we can pull out the fun bits
        int rssi = nano2.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano2.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano2.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano2.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano2.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano2.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryWineTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }

        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags2[j]);
        // }
        
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

  // k = 1;
  // jj = 0;
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
        //If we have a full record we can pull out the fun bits
        int rssi = nano3.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano3.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano3.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano3.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano2.msg[31 + x] < 0x10){
          //   SERIAL1.print(F("0")); //Pretty print
          // } 
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano3.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //最後にnullを加えると良さそう？
        //文字列の終わり指定する
        //uniqueEPC.concat("/0");
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();
        
        // 1個目に格納するための処理
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryWineTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }

        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags3[j]);
        // }
        
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

  // k = 1;
  // jj = 0;
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
        //If we have a full record we can pull out the fun bits
        int rssi = nano4.getTagRSSI(); //Get the RSSI for this tag read

        long freq = nano4.getTagFreq(); //Get the frequency this tag was detected at

        long timeStamp = nano4.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

        byte tagEPCBytes = nano4.getTagEPCBytes(); //Get the number of bytes of EPC from response
        //SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          // if (nano2.msg[31 + x] < 0x10){
          //   SERIAL1.print(F("0")); //Pretty print
          // } 
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano4.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        //最後にnullを加えると良さそう？
        //文字列の終わり指定する
        //uniqueEPC.concat("/0");
        //SERIAL1.print(uniqueEPC);
        //SERIAL1.print(F("]"));
        //SERIAL1.println();
        
        // 1個目に格納するための処理
        if(jj == 0){
          //SERIAL1.println("uniqueEPC is : ");
          //SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        // 下記はarraynumberはdefine or const
        if (arrayNumber != 0){
          for (int i = 0; i<arrayNumber; i++){
            String s_temp = TemporaryWineTags[i];
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){   
            strcpy(TemporaryWineTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }

        // SERIAL1.println("All tags is : ");
        // for(int j = 0; j <arrayNumber; j++){
        //   SERIAL1.println(TemporaryWineTags[j]);
        // }
        
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

  // k = 1;
  // jj = 0;
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  // for( int di = 0; di<arrayNumber; di++){
  //   int dk = 0;
  //   for( int dj = 0; dj<arrayNumber; dj++){
  //     if(strcmp(TemporaryWineTags5[di], TemporaryWineTags[dj]) == 0){
  //       dk = dk + 1;
  //       //SERIAL1.println(dk);
  //     }
  //   }
  //     //DifferenceWineTagsに追加
  //   if(dk == 0){
  //       strcpy(TemporaryWineTags5[dz], TemporaryWineTags[dj]); 
  //       SERIAL1.println(global_differential_CurrentWineTags[dz]);
  //       dz += 1;       
  //   }
  // }

  //////////////////////////
  //下記が差分検出のアルゴリズム
  //////////////////////////

    for( int di = 0; di<arrayNumber; di++){
      int dk = 0;
      for( int dj = 0; dj<arrayNumber; dj++){
        if(strcmp(global_CurrentWineTags[di], TemporaryWineTags[dj]) == 0){
          dk = dk + 1;
          //SERIAL1.println(dk);
        }
      }
      //DifferenceWineTagsに追加
      if(dk == 0){
        strcpy(global_DifferenceWineTags[dz], global_CurrentWineTags[di]); 
        SERIAL1.println(global_DifferenceWineTags[dz]);
        dz += 1;       
      }
    }

    //現在の状況をglobal変数に格納する
    SERIAL1.println("Renew global_CurrentWineTags");
    for(int i = 0; i<arrayNumber; i++){
      strcpy(global_CurrentWineTags[i],TemporaryWineTags[i]);
    }

    SERIAL1.println("DetectDefferentialWines Done...");
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