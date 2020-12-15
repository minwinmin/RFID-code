/*
  Reading multiple RFID tags, simultaneously!
  By: Nathan Seidle @ SparkFun Electronics
  Date: October 3rd, 2016
  https://github.com/sparkfun/Simultaneous_RFID_Tag_Reader

  Constantly reads and outputs any tags heard

  If using the Simultaneous RFID Tag Reader (SRTR) shield, make sure the SERIAL1 slide
  switch is in the 'SW-UART' position
*/
//普通にこれで複数タグ検出できてるけれど...
//変数の容量がたりない...

//#include <SoftwareSERIAL1.h> //Used for transmitting to the device
//SoftwareSERIAL1 SERIAL2(2, 3); //RX, TX
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

#include "SparkFun_UHF_RFID_Reader.h" //Library for controlling the M6E Nano module
RFID nano; //Create instance

int switchPin = 8;
int switchPin2 = 9;
//関数定義
boolean setupNano(long baudRate);
void detectWinebottles();

#define SERIAL1 SerialUSB
#define SERIAL2 Serial1
#define SERIAL3 Serial2

//#define tagsNum 10
//#define idNum 25

//ここでIDの番号を整理する
//心なしか4桁だけにしたら早く読み込めるようになった気がする。
#define tagsNum 50
#define idNum 5

char CurrentWineTags[tagsNum][idNum] = {};
char TemporaryWineTags[tagsNum][idNum] = {};
char DifferenceWineTags[tagsNum][idNum] = {};
//char empty[10][25] = {};
//static std::String mojiretu[10];
//int i = 0;
int k = 1;
int jj = 0;

//デバイス起動時の処理を考える。
//入っているワインを全てpostする処理が必要そう
//それとも初期コマンドっぽいものが必要かもしれぬ
//デバイス起動時にCurrentWineTagsに追加、APIにポストする（初期設定）（ワインが設置されているとする。）
//内容(data)も一気に取得できるようにしておくべき
void setup()
{
  SERIAL1.begin(115200);
  

  //下記、繋いでいる時はコメントアウトを外す
  //有線LAN接続処理//
  // if (Ethernet.begin(mac) == 0) {
  //   SERIAL1.println("Failed to configure Ethernet using DHCP");
  //   Ethernet.begin(mac, ip);
  // }
  /////////////////

  pinMode(switchPin, INPUT);
  pinMode(switchPin2, INPUT);
  
  while (!SERIAL1); //Wait for the SERIAL1 port to come online

  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    SERIAL1.println(F("Module failed to respond. Please check wiring."));
    while (1); //Freeze!
  }

  nano.setRegion(REGION_NORTHAMERICA); //Set to North America

  //ここの大きさでどれくらいの距離を通信可能か調べておこう。
  nano.setReadPower(2700); //5.00 dBm. Higher values may caues USB port to brown out
  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling

  SERIAL1.println(F("Press a key to begin scanning for tags."));
  while (!SERIAL1.available()); //Wait for user to send a character
  SERIAL1.read(); //Throw away the user's character

  nano.startReading(); //Begin scanning for tags

  //割り込み処理
  //割り込みの必要が今のところなさそうだ
  //attachInterrupt(0, detectWinebottles, FALLING);

  //初期設定
  //全てスキャンするまでにはどれくらかかるのか？
  //常時スキャンしててスイッチがLOWになったときだけ違う処理を走らせるとか？
  //一定時間スキャン時間をとるか
  //参考
  //https://detail.chiebukuro.yahoo.co.jp/qa/question_detail/q10180394955
  //unsigned long t0 = millis()/1000; 
  //1分60秒
  //変にdelayを挟むと検出精度がさがるイメージ。
  float now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < 20){
    detectWinebottles();
  }
  memcpy(CurrentWineTags, TemporaryWineTags, sizeof(CurrentWineTags));
  //配列TemporaryWineTagsは初期化する。
  //TemporaryWineTags[10][25] = {0};
  //memcpy(TemporaryWineTags, empty, sizeof(CurrentWineTags));
  // for(int ccc = 0; ccc < 25; ccc++) {
  //   TemporaryWineTags[ccc] = 'a';
  // }

  //TemporaryWineTagsは使用後、初期化しておく（初期状態として0を代入する）。
  memset(TemporaryWineTags, 0 , sizeof(TemporaryWineTags));
  SERIAL1.println("Initial setting is done...");
  
}

void loop()
{
  int arrayNumber = sizeof(CurrentWineTags) / sizeof(CurrentWineTags[0]);
  /*
  //1:open 0:close
  int buttonState = digitalRead(switchPin);
  //SERIAL1.println(buttonState);
  if(buttonState == LOW){
    //何度か取得を繰り返す
    //短期間で開け閉めされた場合はワインタグチェック処理実行しない、機能を実装する必要がある。
    //detectWinebottles();
    //格納されているタグを表示する部分
      SERIAL1.println("All tags is : ");
      for(int j = 0; j <arrayNumber; j++){
        SERIAL1.println(CurrentWineTags[j]);
      }
  }else{
    //SERIAL1.println("BUTTON LOW");
  }
  */

  //差分の検出部分
  
  ///int buttonState2 = digitalRead(switchPin2);
  int buttonState = digitalRead(switchPin);
  //if(buttonState2 == LOW){
  if(buttonState == HIGH){
    
    int dz = 0;
    SERIAL1.println("Detecting...");
    // プログラムがスタートしてからの時間を出力するから初期する必要がありそう。
    float now_time = millis() / 1000;
    while ((millis()/1000 - now_time) < 20){
      detectWinebottles();
    }
    for( int di = 0; di<arrayNumber; di++){
      int dk = 0;
      for( int dj = 0; dj<arrayNumber; dj++){
        //ここの一致がよくなさそう
        //String型に変換して比較しよう
        String temp_CurrentWineTag = CurrentWineTags[di];
        String temp_TemporaryWineTag = TemporaryWineTags[dj];
        if(temp_CurrentWineTag == temp_TemporaryWineTag){
          dk = dk + 1;
          SERIAL1.println("dk is :");
          SERIAL1.println(dk);
        }
      }
      //DifferenceWineTagsに追加
      if(dk == 0){
        strcpy(DifferenceWineTags[dz], CurrentWineTags[di]); 
        dz += 1;       
      }
    }
    SERIAL1.println("Removed tags is : ");
    for(int j = 0; j <arrayNumber; j++){
        SERIAL1.println(DifferenceWineTags[j]);
    }
    //TemporaryWineTagsは使用後、初期化しておく（初期状態として0を代入する）。
    memset(TemporaryWineTags, 0 , sizeof(TemporaryWineTags));
    SERIAL1.println("Done...");

    }
  
  delay(100);
}

//ボタンが押されたときに（ワインセラーに設置されたリミットスイッチが押されたときに）
//計測を始めたい（全部のワイン検知を目指す）
void detectWinebottles(){
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
      int rssi = nano.getTagRSSI(); //Get the RSSI for this tag read

      long freq = nano.getTagFreq(); //Get the frequency this tag was detected at

      long timeStamp = nano.getTagTimestamp(); //Get the time this was read, (ms) since last keep-alive message

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
        //wineTags2[0][30] = "HELLO";
        jj += 1;
      }
      //すでにsizeを指定しているので25個分の空要素が用意されている
      int arrayNumber = sizeof(TemporaryWineTags) / sizeof(TemporaryWineTags[0]);
      int count = 0;
      if (arrayNumber != 0){
        //arrayNumberの数は最初から決まっている
        //つまり、配列wineTags2の初期値が入力されているから。
        for (int i = 0; i<=arrayNumber; i++){
          //charからstringへの変更、下記が参考になった
          //https://detail.chiebukuro.yahoo.co.jp/qa/question_detail/q13177596828
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

//下記はあまり触らない方が良さそう。
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
  nano.begin(SERIAL3); //Tell the library to communicate over software SERIAL1 port

  //Test to see if we are already connected to a module
  //This would be the case if the Arduino has been reprogrammed and the module has stayed powered
  SERIAL3.begin(baudRate); //For this test, assume module is already at our desired baud rate
  //while (SERIAL2.isListening() == false); //Wait for port to open

  //About 200ms from power on the module will send its firmware version at 115200. We need to ignore this.
  while (SERIAL3.available()) SERIAL3.read();

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
    SERIAL3.begin(115200); //Start software SERIAL1 at 115200

    nano.setBaud(baudRate); //Tell the module to go to the chosen baud rate. Ignore the response msg

    SERIAL3.begin(baudRate); //Start the software SERIAL1 port, this time at user's chosen baud rate

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
