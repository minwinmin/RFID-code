#include "SparkFun_UHF_RFID_Reader.h" 
//インスタンス
RFID nano;

//#define SERIAL1 Serial1
#define SERIAL1 SerialUSB
#define SERIAL2 Serial2 //17(Rx), 16(Tx)

//管理するタグの枚数
#define tagsNum 20
//IDの桁数
#define idNum 5
//リーダーが電波を発する時間を指定
#define INTERVAL 5

//関数定義
boolean setupNano(long baudRate);
//タグ検出を行う関数（リーダーが電波を発する）
void oneAntennaDetection(char out_currentTags[tagsNum][idNum]);
//現在と過去のタグデータを比較し、取り出されたタグをもとめる
void detectDefferentialTags();

//////２次元配列の初期化 ///////
//現在のタグデータを保存する配列
char currentTags[tagsNum][idNum] = {0};
char arrayNum[tagsNum][idNum] = {0};
//取り出されたタグデータを保存する配列
char differenceTags[tagsNum][idNum] = {0};

//配列の要素数を導出
int arrayNumber = sizeof(arrayNum) / sizeof(arrayNum[0]);

//マイコンに電源を入れたときに初期状態を計測するために使う変数
int first_detect = 0;


void setup(){
  SERIAL1.begin(115200);
  
  while (!SERIAL1); //Wait for the SERIAL1 port to come online

  if (setupNano(38400) == false) //Configure nano to run at 38400bps
  {
    SERIAL1.println(F("Module failed to respond. Please check wiring."));
    while (1);
  }
  nano.setRegion(REGION_NORTHAMERICA); //Set to North America
  nano.setReadPower(2000); //5.00 dBm. Higher values may caues USB port to brown out

  //Max Read TX Power is 27.00 dBm and may cause temperature-limit throttling
  //電源を入れたら検出を始めるようにする場合は下記をコメントアウトする
  SERIAL1.println(F("Press a key to begin scanning for tags."));
  while (!SERIAL1.available());
  SERIAL1.read(); 
}

void loop(){
  char Tags[tagsNum][idNum] = {0};
  nano.stopReading();
  // 初期状態を検出
  if(first_detect == 0){
    SERIAL1.println("First detection is ...");
    oneAntennaDetection(Tags);
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(currentTags[i]);
    }
    nano.stopReading();
  }

  if(first_detect != 0){
    SERIAL1.println("Detection is ...");
    memset(differenceTags, '\0', sizeof(differenceTags));
    detectDefferentialTags();
    SERIAL1.println("Result is ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(differenceTags[i]);
    }

    SERIAL1.println("All tags are ...");
    for (int i = 0; i < arrayNumber; i++){
      SERIAL1.println(currentTags[i]);
    }
    nano.stopReading();
  }

  first_detect++;
  delay(5000);
}

void oneAntennaDetection(char TemporaryTags[tagsNum][idNum]){
  int k = 1;
  int jj = 0;
  memset(differenceTags, '\0', sizeof(differenceTags));

  nano.startReading(); //タグのスキャン開始
  float now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;
    if (nano.check() == true) { 
      byte responseType = nano.parseResponse(); 
      if (responseType == RESPONSE_IS_KEEPALIVE){
        SERIAL1.println(F("Scanning"));
      }else if (responseType == RESPONSE_IS_TAGFOUND){
        byte tagEPCBytes = nano.getTagEPCBytes(); 
        for (byte x = 0 ; x < tagEPCBytes ; x++)
        {
          //IDを下4桁にパース
          String epc = String(nano.msg[31 + x], HEX);
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
            //重複がみつかったらカウントする
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          //重複なしなら配列currentTagsに順次格納
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
        SERIAL1.print("Unknown error");
      }
    }

  }
  SERIAL1.println("Initial setting is done...");
  //発熱防止に計測がおわったら、stopReading()をいれる。
  nano.stopReading();
  SERIAL1.println("Stop module...");
  SERIAL1.println("");
  SERIAL1.print("k check :");
  SERIAL1.println(k);
  SERIAL1.println("");

  //現在の状況をglobal変数に格納する
  for(int i = 0; i<arrayNumber; i++){
      strcpy(currentTags[i],TemporaryTags[i]);
  }
}

//全てのタグデータの差分を検出する関数
void detectDefferentialTags(){
  int k = 1;
  int jj = 0;
  int dz = 0;
  SERIAL1.println("Detecting...");
  char TemporaryTags[tagsNum][idNum] = {0};  
  //////////////////////////////////////////////////
  //リーダーが検出を開始する
  nano.startReading();
  float now_time = millis() / 1000;
  while ((millis()/1000 - now_time) < INTERVAL){
    //検出されたタグのEPCを一時的に格納する変数
    String uniqueEPC;

    if (nano.check() == true) {
      byte responseType = nano.parseResponse(); 
      if (responseType == RESPONSE_IS_KEEPALIVE){
        SERIAL1.println(F("Scanning"));
      }else if (responseType == RESPONSE_IS_TAGFOUND){
        byte tagEPCBytes = nano.getTagEPCBytes();
        for (byte x = 0 ; x < tagEPCBytes ; x++){
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
            if(s_temp == uniqueEPC){
              count += 1;
            }   
          }
          if (count == 0){
            strcpy(TemporaryTags[k], uniqueEPC.c_str());
            k += 1;
          }
        }        
      }else if (responseType == ERROR_CORRUPT_RESPONSE){
        SERIAL1.println("Bad CRC");
      }else{
        SERIAL1.print("Unknown error");
      }
    }
  }
  SERIAL1.println("Initial setting is done...");
  //発熱防止に計測がおわったら、stopReading()をいれる。
  nano.stopReading();
  SERIAL1.println("Stop module...");
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
        if(strcmp(currentTags[di], TemporaryTags[dj]) == 0){
          dk = dk + 1;
          //SERIAL1.println(dk);
        }
      }
      if(dk == 0){
        strcpy(differenceTags[dz], currentTags[di]); 
        SERIAL1.println(differenceTags[dz]);
        dz += 1;       
      }
    }

    //現在の状況をglobal変数に格納する
    SERIAL1.println("Renew currentTags");
    for(int i = 0; i<arrayNumber; i++){
      strcpy(currentTags[i],TemporaryTags[i]);
    }

    SERIAL1.println("detectDefferentialTags Done...");
}

boolean setupNano(long baudRate){
  nano.begin(SERIAL2);
  SERIAL2.begin(baudRate);
  while (SERIAL2.available()) SERIAL2.read();
  nano.getVersion();
  if (nano.msg[0] == ERROR_WRONG_OPCODE_RESPONSE){
    nano.stopReading();
    SERIAL1.println(F("Module continuously reading. Asking it to stop..."));

    delay(1500);
  }else{
    SERIAL2.begin(115200); 
    nano.setBaud(baudRate); 
    SERIAL2.begin(baudRate);
    delay(250);
  }
  //Test the connection
  nano.getVersion();
  if (nano.msg[0] != ALL_GOOD) return (false); 
  nano.setTagProtocol(); 
  nano.setAntennaPort(); 
  return (true);
}