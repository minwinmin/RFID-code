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
char server[] = "192.168.11.60"; // ここを適当なIPアドレス（PCのIPアドレス）に変更
// クライアント
EthernetClient client;
////////////////////////////////

//Library for controlling the M6E Nano module
#include "SparkFun_UHF_RFID_Reader.h" 

RFID nano, nano2;

#define SERIAL1 SerialUSB
#define SERIAL2 Serial1
#define SERIAL3 Serial2
#define tagsNum 45
#define idNum 5
#define INTERVAL 10

//関数定義
boolean setupNano(long baudRate);
boolean setupNano2(long baudRate);
void multipleAntennaDetection(char out_CurrentWineTags[tagsNum][idNum]);
void detectDefferentialWines(char out_CurrentWineTags[tagsNum][idNum], char differential_CurrentWineTags[tagsNum][idNum]);
void detectWinebottles();
void detectWinebottles2();
void showAllTags();
void detectDefferentialWines();

char global_CurrentWineTags[tagsNum][idNum] = {0};
//char out_TemporaryWineTags[tagsNum][idNum] = {};
char arrayNum[tagsNum][idNum] = {0};
char global_DifferenceWineTags[tagsNum][idNum] = {0};

unsigned long long time_before = 0;
unsigned long long time_after  = 0;
unsigned long long time_length = 0;
int arrayNumber = sizeof(arrayNum) / sizeof(arrayNum[0]);
int time_count   = 0;
int diff_count   = 0;
int k = 1;
int jj = 0;
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
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling

  SERIAL1.println(F("Press a key to begin scanning for tags."));
  while (!SERIAL1.available()); //Wait for user to send a character
  SERIAL1.read(); //Throw away the user's character

  //割り込み処理の追加
  //attachInterrupt(limitSwitch, detectWinebottles, FALLING);

  //初期タグ取得
  //multipleAntennaDetection();

}

void loop(){
  //下記はループ処理なのですぐに初期化されてしまう
  char CurrentWineTags[tagsNum][idNum]              = {0};
  char differential_CurrentWineTags[tagsNum][idNum] = {0};
  
  // 旧初期化処理（冗長）
  // for(int i = 0; i < arrayNumber; i++){
  //   strcpy(out_CurrentWineTags[i],"\0");
  // }

  // 本命処理
  // if (digitalRead(limitSwitch) == 1){
  //   if (time_count == 0){
  //     time_before = millis() / 1000;
  //     time_count++;
  //     SERIAL1.println("lock");
  //   }
  //   if(diff_count == 0 && ((millis()/1000 - time_before) > 10)){
  //     detectDefferentialWines();
  //     diff_count++;
  //   }
  //   //showAllTags();
  // }else{
  //   //SERIAL1.println("open");
  //   time_count = 0;
  //   diff_count = 0;
  // }

  // 初期状態を読み込み
  if(first_detect == 0){
    SERIAL1.println("First detection is ...");
    // 下記もglobal変数を渡せば良さそう
    multipleAntennaDetection(global_CurrentWineTags);
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(global_CurrentWineTags[i]);
    }
    first_detect++;
  }

  int buttonState = digitalRead(switchPin);
  if(buttonState == HIGH){
    // //multipleAntennaDetection(out_CurrentWineTags);
    // detectDefferentialWines(out_CurrentWineTags, differential_CurrentWineTags);
    // SERIAL1.println("Result is ...");
    // for (int i = 0; i < arrayNumber; i++){
    //   SERIAL1.println(differential_CurrentWineTags[i]);
    // }
    // for(int i = 0; i < arrayNumber; i++){
    //   strcpy(out_CurrentWineTags[i],"\0");
    // }
    SERIAL1.println("Detection is ...");
    // 検証用
    multipleAntennaDetection(global_CurrentWineTags);
    // 差分検知
    //detectDefferentialWines(global_CurrentWineTags, differential_CurrentWineTags)
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(global_CurrentWineTags[i]);
    }
  }

  // if(digitalRead(3) == HIGH){
  // SERIAL1.println("RFID tags");
  //   // 関数から返ってきた文字列を表示
  //   for (int i = 0; i < arrayNumber; i++){
  //       SERIAL1.println(out_CurrentWineTags[i]);
  //   }
  // }
  delay(100);
}

void multipleAntennaDetection(char TemporaryWineTags[tagsNum][idNum]){
  //char CurrentWineTags[tagsNum][idNum]   = {0};
  //char TemporaryWineTags[tagsNum][idNum] = {0};
  //1つ目のモジュール計測開始
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
        SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          if (nano.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        SERIAL1.print(uniqueEPC);
        SERIAL1.print(F("]"));
        SERIAL1.println();

        //重要なコード部分、タグを重複なしで配列wineTags2に格納する
        //ここが配列に格納するタイミングかな
        //string型をcharに変換して配列に挿入
        //参考 https://teratail.com/questions/44651
        //char temp = uniqueEPC.c_str();
        //初期状態はEPCが格納されていないので、まず挿入。ただし、良い書き方がありそう。

        //しも4桁に絞る
        //uniqueEPC = uniqueEPC.substring(21,24);
        if(jj == 0){
          SERIAL1.println("uniqueEPC is : ");
          SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          //arrayNumberの数は最初から決まっている
          //つまり、配列wineTags2の初期値が入力されているから。
          for (int i = 0; i<=arrayNumber; i++){
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
        SERIAL1.println("All tags is : ");
        for(int j = 0; j <arrayNumber; j++){
          SERIAL1.println(TemporaryWineTags[j]);
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
        SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          if (nano2.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
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
        SERIAL1.print(uniqueEPC);
        SERIAL1.print(F("]"));
        SERIAL1.println();
        if(jj == 0){
          SERIAL1.println("uniqueEPC is : ");
          SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<=arrayNumber; i++){
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

        SERIAL1.println("All tags is : ");
        for(int j = 0; j <arrayNumber; j++){
          SERIAL1.println(TemporaryWineTags[j]);
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
  
  //現在の状況をglobal変数に格納する
  for(int i = 0; i<arrayNumber; i++){
      strcpy(global_CurrentWineTags[i],TemporaryWineTags[i]);
  }
}

//ワインの差分の検出部分
void detectDefferentialWines(char global_CurrentWineTags[tagsNum][idNum], char differential_CurrentWineTags[tagsNum][idNum]){
  int dz = 0;
  SERIAL1.println("Detecting...");
  char TemporaryWineTags[tagsNum][idNum] = {0};

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
        SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          if (nano.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        SERIAL1.print(uniqueEPC);
        SERIAL1.print(F("]"));
        SERIAL1.println();

        //重要なコード部分、タグを重複なしで配列wineTags2に格納する
        //ここが配列に格納するタイミングかな
        //string型をcharに変換して配列に挿入
        //参考 https://teratail.com/questions/44651
        //char temp = uniqueEPC.c_str();
        //初期状態はEPCが格納されていないので、まず挿入。ただし、良い書き方がありそう。

        //しも4桁に絞る
        //uniqueEPC = uniqueEPC.substring(21,24);
        if(jj == 0){
          SERIAL1.println("uniqueEPC is : ");
          SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          //arrayNumberの数は最初から決まっている
          //つまり、配列wineTags2の初期値が入力されているから。
          for (int i = 0; i<=arrayNumber; i++){
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
        SERIAL1.println("All tags is : ");
        for(int j = 0; j <arrayNumber; j++){
          SERIAL1.println(TemporaryWineTags[j]);
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
        SERIAL1.print(F(" epc["));
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          if (nano2.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
          //SERIAL1.print(nano.msg[31 + x], HEX);
          //SERIAL1.print(F(" "));
          String epc = String(nano2.msg[31 + x], HEX);
          if(tagEPCBytes-2 <= x){
            uniqueEPC.concat(epc);
          }
        }
        SERIAL1.print(uniqueEPC);
        SERIAL1.print(F("]"));
        SERIAL1.println();
        if(jj == 0){
          SERIAL1.println("uniqueEPC is : ");
          SERIAL1.println(uniqueEPC.c_str());
          strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
          jj += 1;
        }

        int count = 0;
        if (arrayNumber != 0){
          for (int i = 0; i<=arrayNumber; i++){
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

        SERIAL1.println("All tags is : ");
        for(int j = 0; j <arrayNumber; j++){
          SERIAL1.println(TemporaryWineTags[j]);
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
  //////////////////////////
  //下記が差分検出のアルゴリズム
  //////////////////////////

    for( int di = 0; di<arrayNumber; di++){
      int dk = 0;
      for( int dj = 0; dj<arrayNumber; dj++){
        if(strcmp(global_CurrentWineTags[di], TemporaryWineTags[dj])){
          dk = dk + 1;
          // SERIAL1.println("");
          // SERIAL1.print("dk is :");
          // SERIAL1.print(dk);
          // SERIAL1.println("");
          //SERIAL1.println(temp_CurrentWineTag);
        }
      }
      //DifferenceWineTagsに追加
      if(dk == 0){
        SERIAL1.print("A Removed wine is :");
        strcpy(differential_CurrentWineTags[dz], global_CurrentWineTags[di]); 
        dz += 1;       
      }
    }

    //現在の状況をglobal変数に格納する
    SERIAL1.println("Renew global_CurrentWineTags");
    for(int i = 0; i<arrayNumber; i++){
        strcpy(global_CurrentWineTags[i],TemporaryWineTags[i]);
    }

    SERIAL1.println("Removed tags is : ");
    for(int j = 0; j <arrayNumber; j++){
        SERIAL1.println(differential_CurrentWineTags[j]);
    }
    SERIAL1.println("DetectDefferentialWines Done...");
}

//ボタンが押されたときに（ワインセラーに設置されたリミットスイッチが押されたときに）
//計測を始めたい（全部のワイン検知を目指す）
void detectWinebottles(){
  char TemporaryWineTags[tagsNum][idNum] = {};
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
      SERIAL1.print(F(" epc["));
      for (byte x = 0 ; x < tagEPCBytes ; x++)
      {
        if (nano.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
        //SERIAL1.print(nano.msg[31 + x], HEX);
        //SERIAL1.print(F(" "));
        String epc = String(nano.msg[31 + x], HEX);
        if(tagEPCBytes-2 <= x){
          uniqueEPC.concat(epc);
        }
      }
      SERIAL1.print(uniqueEPC);
      SERIAL1.print(F("]"));
      SERIAL1.println();

      //重要なコード部分、タグを重複なしで配列wineTags2に格納する
      //ここが配列に格納するタイミングかな
      //string型をcharに変換して配列に挿入
      //参考 https://teratail.com/questions/44651
      //char temp = uniqueEPC.c_str();
      //初期状態はEPCが格納されていないので、まず挿入。ただし、良い書き方がありそう。

      //しも4桁に絞る
      //uniqueEPC = uniqueEPC.substring(21,24);
      if(jj == 0){
        SERIAL1.println("uniqueEPC is : ");
        SERIAL1.println(uniqueEPC.c_str());
        strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
        
        jj += 1;
      }

      int count = 0;
      if (arrayNumber != 0){
        //arrayNumberの数は最初から決まっている
        //つまり、配列wineTags2の初期値が入力されているから。
        for (int i = 0; i<=arrayNumber; i++){
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
      SERIAL1.println("All tags is : ");
      for(int j = 0; j <arrayNumber; j++){
        SERIAL1.println(TemporaryWineTags[j]);
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

void detectWinebottles2(){
  char TemporaryWineTags[tagsNum][idNum] = {};
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
      SERIAL1.print(F(" epc["));
      for (byte x = 0 ; x < tagEPCBytes ; x++)
      {
        if (nano2.msg[31 + x] < 0x10) SERIAL1.print(F("0")); //Pretty print
        //SERIAL1.print(nano.msg[31 + x], HEX);
        //SERIAL1.print(F(" "));
        String epc = String(nano2.msg[31 + x], HEX);
        if(tagEPCBytes-2 <= x){
          uniqueEPC.concat(epc);
        }
      }
      SERIAL1.print(uniqueEPC);
      SERIAL1.print(F("]"));
      SERIAL1.println();
      if(jj == 0){
        SERIAL1.println("uniqueEPC is : ");
        SERIAL1.println(uniqueEPC.c_str());
        strcpy(TemporaryWineTags[0], uniqueEPC.c_str());
        jj += 1;
      }

      int count = 0;
      if (arrayNumber != 0){
        for (int i = 0; i<=arrayNumber; i++){
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

      SERIAL1.println("All tags is : ");
      for(int j = 0; j <arrayNumber; j++){
        SERIAL1.println(TemporaryWineTags[j]);
      }
      
    }
    else if (responseType == ERROR_CORRUPT_RESPONSE){
      SERIAL1.println("Bad CRC");
    }else{
      SERIAL1.print("Unknown error");
    }
  }
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