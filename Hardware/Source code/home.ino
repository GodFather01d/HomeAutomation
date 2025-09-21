#define FIRMWARE_VERSION 5.5

#include <WiFiManager.h>  
#include <EEPROM.h>
#include <FirebaseESP8266.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <Ticker.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <TimeLib.h>
#include <WiFiClientSecure.h>
#include <ESP8266httpUpdate.h>

#define wifiLed 2            //  D4   wifi led
#define TRIGGER_PIN 14          // wifi setup pin AP
#define TRANSFER_IND_PIN 12       //  D6   data transfer indication pin 
    
#define API_KEY "AIzaSyAQh-tAWmKpcGzwmXISv0-aY-Q4s6MAxZc"
#define DATABASE_URL "homeautomation-e6ef2-default-rtdb.asia-southeast1.firebasedatabase.app/"

//WiFiClientSecure client;  
#define EEPROM_SIZE 512
#define sys_on_time 20
#define sys_off_time 15
#define SYS_ID_ADDR 0
#define EEPROM_UPDATE 30
#define UPDATE_LINK_ADDER 35
#define MAX_SWITCH_NO 25
#define MAX_PATH_LEN 80
#define STREAM_RETRY_LIMIT 4

const char AP_ID[] = "JT_WiFi_SETUP", AP_PASS[] = "Jyoti@2000";
const long utcOffsetInSeconds = 19800;
unsigned long startTime;
const unsigned long timeout = 20000;

unsigned long offlineStartTime = 0; 
const unsigned long OFFLINE_TIMEOUT = 2UL * 60UL * 1000UL; 
char sys_id[32]; 
volatile bool buttonPressed = false, signupOK = false, offline_flag = false;
uint8_t update_ota = 1, curr_switch_on = 0, curr_switch_off = 0,time_hours = 0, time_mint = 0,currentMonth = 0,currentDay = 0;
uint16_t offline_counter = 0,currentYear = 0; 
int on_transfer_counter[MAX_SWITCH_NO + 1] = {0};
int off_transfer_counter[MAX_SWITCH_NO + 1] = {0};

unsigned long lastOnMillis[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOffMillis[MAX_SWITCH_NO + 1] = {0};
const unsigned long PRINT_INTERVAL = 2000; 



// new globals
unsigned int stream_fail_count = 0;
unsigned long lastStreamReconnect = 0;
const unsigned long STREAM_RECONNECT_INTERVAL = 10UL * 1000UL; // 10s between reconnect attempts


struct Schedule 
{
  int onHour;
  int onMin;
  int offHour;
  int offMin;
  bool valid;
};

FirebaseData fbdo ;
FirebaseAuth auth;
FirebaseConfig config;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", utcOffsetInSeconds);
Ticker timer;
WiFiManager wm;
Schedule schedules[MAX_SWITCH_NO + 1];

void IRAM_ATTR handleButtonPress() 
{
  buttonPressed = true;
}
void Blink_led(int state ,int speed)
{
  for(int i=0;i<=3;i++)
  {
    digitalWrite(wifiLed,state);
    delay(speed);
    digitalWrite(wifiLed,!state);
    delay(speed);
  }
}
void send_online()
{
  if(!Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1))
  {
    if(!Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1))
    {
      ESP.restart();
    }
  }
}
void settime()
{
  if (buttonPressed) return;
  if(currentYear < 2024)
  {
    timeClient.update();
  }
  time_hours = timeClient.getHours();
  time_mint = timeClient.getMinutes();
  unsigned long epochTime = timeClient.getEpochTime();
  currentYear = year(epochTime);
  currentMonth = month(epochTime);
  currentDay = day(epochTime);
}

void initStream() 
{
  // Example: listen to /SYSTEM/<sys_id>/BUTTONS
  String path = "/SYSTEM/";
  path += sys_id;
  path += "/BUTTONS";

  if (!Firebase.beginStream(fbdo, path))
  {
    Serial.println("Stream begin error: " + fbdo.errorReason());
    return;
  }
  else
  {
    digitalWrite(wifiLed, LOW);
  }
}
void handleStream() 
{
  if (!Firebase.readStream(fbdo)) 
  {
    // read error
    Serial.println("Stream read failed: " + fbdo.errorReason());
    // on read failure, increment counter and try re-init after a few failures or after timeout
    stream_fail_count++;
    if (stream_fail_count >= STREAM_RETRY_LIMIT) {
      Serial.println("Stream failing repeatedly - restarting stream...");
      Firebase.endStream(fbdo);
      delay(200);
      initStream();
      stream_fail_count = 0;
    }
    return;
  }

  // reset failure counter on successful read
  stream_fail_count = 0;

  if (fbdo.streamTimeout()) 
  {
    Serial.println("Stream timeout, will attempt to resume.");
    // attempt to re-init stream (but don't spam)
    if (millis() - lastStreamReconnect >= STREAM_RECONNECT_INTERVAL) {
      lastStreamReconnect = millis();
      Firebase.endStream(fbdo);
      delay(100);
      initStream();
    }
    return;
  }

  if (fbdo.streamAvailable()) 
  {
    if (fbdo.dataType() == "int") 
    {
      int value = fbdo.intData();
      String dataPath = fbdo.dataPath();  // e.g. "/SW1" or "/SW23"
      if (dataPath.startsWith("/SW")) 
      {

        String swNumber = dataPath.substring(3);
        int switch_no = swNumber.toInt();  // extract number after "SW"
        if (value == 1) 
        {
          digitalWrite(TRANSFER_IND_PIN, LOW);

          for(int i = 0;i < 2;i++)
          {
            serial_out(switch_no,1);
            
            delay(1000);
          } 
          send_switch_state_safe(switch_no,true);
          digitalWrite(TRANSFER_IND_PIN, HIGH);

          //serial_out(switch_no,1);
        } 
        else if (value == 0)
        {
          digitalWrite(TRANSFER_IND_PIN, LOW);

          for(int i = 0;i < 2;i++)
          {

            serial_out(switch_no,0);
            delay(1000);

          } 
          send_switch_state_safe(switch_no,false);
          digitalWrite(TRANSFER_IND_PIN, HIGH);

        }
        send_online();
      }
      if (dataPath.startsWith("/UPDATE")) 
      {
        if (value == 1) 
        {
          Serial.println("UPDATING .....");
          readlink_from_firebase();
        } 
      }
      if (dataPath.startsWith("/RESET")) 
      {
        if (value == 1) 
        {
          if(Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/RESET", 0))
          {
            ESP.restart();
          }
        } 
      }
      if (dataPath.startsWith("/SET_TIME")) 
      {
        if (value == 1) 
        {
          if(Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/SET_TIME", 0))
          {
            readSchedulesFromFirebase();
            Blink_led(1,100);
          }
        } 
      }
      
      if (dataPath.startsWith("/ONLINE_STATUS")) 
      {
        if (value == 0) 
        {
          send_online();
        } 
      }
    }
  }
}
void readlink_from_firebase()
{
  EEPROM.put(EEPROM_UPDATE, 1);
  String path = "ADDITIONAL_INFO/UPDATE_LINK";
  if (Firebase.getString(fbdo, path)) 
  {
    if (fbdo.dataType() == "string") 
    {
      String link = fbdo.stringData(); // Store the string from Firebase
      Serial.println("Link fetched successfully: " + link);
      char linkArray[link.length() + 1]; // Ensure enough space for null terminator
      link.toCharArray(linkArray, sizeof(linkArray));
      for (int i = 0; i < link.length() && i < (EEPROM_SIZE - UPDATE_LINK_ADDER); i++) 
      {
        EEPROM.write(UPDATE_LINK_ADDER + i, linkArray[i]);
      }
      EEPROM.write(UPDATE_LINK_ADDER + link.length(), '\0'); // Null-terminate the stored string
      String update_time = (String(time_hours)+" : "+String(time_mint)+ "_" );
      String update_date = (String(currentDay)+" / "+String(currentMonth)+" / "+ String(currentYear));
      Firebase.setString(fbdo, "SYSTEM/" + String(sys_id) + "/LAST_UPDATE_TIME/",update_time + update_date);
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/UPDATE", 0);
      EEPROM.commit();
      Firebase.endStream(fbdo);
      delay(1000);
      ESP.restart();
    } 
    else 
    {
      Serial.println("The data type is not a string!");
    }
  } 
  else 
  {
    Serial.println("Failed to fetch update link: " + fbdo.errorReason());
  }
}
bool send_switch_state_safe(int switch_no, bool state) 
{
  char path[MAX_PATH_LEN];
  // path: SYSTEM/<sys_id>/SWITCH_STATE/SW<no>
  snprintf(path, sizeof(path), "SYSTEM/%s/SWITCH_STATE/SW%d", sys_id, switch_no);
  if(send_notification_safe(switch_no,state))
  {
    return Firebase.setInt(fbdo, path, (int)state);
  }
  else 
  {
    return false;
  }
  // use Firebase.setInt and return result
  
}

// ---------- safer notification ----------
bool send_notification_safe(int switch_no,int state)
{
  char path[MAX_PATH_LEN];
  snprintf(path, sizeof(path), "SYSTEM/%s/NOTIFICATION/SW%d", sys_id, switch_no);
  // update time string (still using String for time is OK once in a while)
  String update_time = (String(time_hours)+" : "+String(time_mint)+ "_" );
  String update_date = (String(currentDay)+" / "+String(currentMonth)+" / "+ String(currentYear));
  char timePath[MAX_PATH_LEN];
  snprintf(timePath, sizeof(timePath), "SYSTEM/%s/NOTIFICATION/NOTIFICATION_TIME/", sys_id);
  String basePath = "SYSTEM/" + String(sys_id) + "/BUTTONS/SW" + String(switch_no);
  Firebase.setInt(fbdo, basePath.c_str(), 3);
  
  Firebase.setString(fbdo, timePath, update_time + update_date);
      
         
  return Firebase.setInt(fbdo, path, state);
 
}

void get_serial() 
{
  if (Serial.available()) 
  {
    String s = Serial.readStringUntil('\n');
    s.trim();

    if (s.startsWith(String(sys_id))) 
    {
      int firstUnderscore = s.indexOf('_');
      int secondUnderscore = s.indexOf('_', firstUnderscore + 1);

      if (firstUnderscore > 0 && secondUnderscore > firstUnderscore) 
      {
        int switchNo = s.substring(firstUnderscore + 1, secondUnderscore).toInt();
        int state = s.substring(secondUnderscore + 1).toInt();

        if (switchNo >= 1 && switchNo <= MAX_SWITCH_NO) 
        {
          if (state == 1) 
          {
            send_switch_state_safe(switchNo, true);
            on_transfer_counter[switchNo] = 4;
          } 
          else if (state == 0) 
          {
            send_switch_state_safe(switchNo, false);
            off_transfer_counter[switchNo] = 4;
          } 
          else 
          {
            Serial.println("Invalid state: " + String(state));
          }
        } 
        else 
        {
          Serial.println("Out of range switch no: " + String(switchNo));
        }
      } 
      else 
      {     

        Serial.println("Invalid format: " + s);
      }
    }
  }
}
void readSchedulesFromFirebase() 
{
  for (int i = 1; i <= MAX_SWITCH_NO; i++) 
  {
    String swPath = "SYSTEM/" + String(sys_id) + "/SCHEDULED/SW" + String(i); // ✅ use sys_id
    if (Firebase.getInt(fbdo, swPath + "/ON_HOUR")) 
    {
      schedules[i].onHour = fbdo.intData();
      Firebase.getInt(fbdo, swPath + "/ON_MIN");
      schedules[i].onMin = fbdo.intData();
      Firebase.getInt(fbdo, swPath + "/OFF_HOUR");
      schedules[i].offHour = fbdo.intData();
      Firebase.getInt(fbdo, swPath + "/OFF_MIN");
      schedules[i].offMin = fbdo.intData();
      schedules[i].valid = true;
      // Serial.println("switch_no "+String(i) + " valid");
      // Serial.println("ON_TIME "+ String(schedules[i].onHour)+" : " + String(schedules[i].onMin));
      // Serial.println("OFF_TIME "+ String(schedules[i].offHour)+" : " + String(schedules[i].offMin));
    } 
    else 
    {
      schedules[i].valid = false;
    }
  }
}

void applySchedules() 
{
  unsigned long now = millis();

  for (int i = 1; i <= MAX_SWITCH_NO; i++) 
  {
    if (!schedules[i].valid) continue;
    if (time_hours == schedules[i].onHour && time_mint == schedules[i].onMin) 
    {
      if (on_transfer_counter[i] < 2 && now - lastOnMillis[i] >= PRINT_INTERVAL) 
      {
        serial_out(i,1);
        on_transfer_counter[i]++;
        String basePath = "SYSTEM/" + String(sys_id) + "/BUTTONS/SW" + i;
        Firebase.setInt(fbdo, basePath, 1);
        lastOnMillis[i] = now; // update last print time
      }
      if (on_transfer_counter[i] >= 0 && on_transfer_counter[i] < 2)
      {
        digitalWrite(TRANSFER_IND_PIN, LOW);
      }
      else
      {
        digitalWrite(TRANSFER_IND_PIN, HIGH);
      }
    } 
    else 
    {
      on_transfer_counter[i] = 0;
      lastOnMillis[i] = 0;
    }

    // --- OFF TIME ---
    if (time_hours == schedules[i].offHour && time_mint == schedules[i].offMin) 
    {
      if (off_transfer_counter[i] < 2 && now - lastOffMillis[i] >= PRINT_INTERVAL) 
      {
        serial_out(i,0);
        off_transfer_counter[i]++;
        String basePath = "SYSTEM/" + String(sys_id) + "/BUTTONS/SW" + i;
        Firebase.setInt(fbdo, basePath, 0);
        lastOffMillis[i] = now;
      }
      if (off_transfer_counter[i] >= 0 && off_transfer_counter[i] < 3)
      {
        digitalWrite(TRANSFER_IND_PIN, LOW);
      }
      else
      {
        digitalWrite(TRANSFER_IND_PIN, HIGH);
      }
    } 
    else 
    {
      off_transfer_counter[i] = 0;
      lastOffMillis[i] = 0;
    }
  }
}
void serial_out(int switch_no, int state) 
{
  char buffer[16];
  // Format switch number as two digits
  snprintf(buffer, sizeof(buffer), "%02d", switch_no);
  char path[32];
  snprintf(path, sizeof(path), "%s_%s_%d", sys_id, buffer, state);
  Serial.println();
  Serial.println(path);
}
void setupConfigPortal() 
{
  // WiFiManager wm;

 Serial.println("Free RAM before cleanup: " + String(ESP.getFreeHeap()));

  // Stop Firebase
  Firebase.endStream(fbdo);
  fbdo.clear();

  // Stop NTP
  timeClient.end();

  // Disconnect WiFi
  WiFi.disconnect(true);
  delay(100);

  Serial.println("Free RAM after cleanup: " + String(ESP.getFreeHeap()));

  Blink_led(0,500);
  wm.setCaptivePortalEnable(true);
  WiFiManagerParameter sys_id_param("sys_id", "System ID", sys_id, 32);
  wm.addParameter(&sys_id_param);
  wm.setConfigPortalTimeout(180);
  delay(1000);

  if (!wm.startConfigPortal("JT_WIFI_SETUP","Jyoti@2000") )
  {
   Serial.println("Failed to connect through Config Portal");
  }
  strcpy(sys_id, sys_id_param.getValue());
  for (int i = 0; i < sizeof(sys_id); i++) 
  {
    EEPROM.write(SYS_ID_ADDR + i, sys_id[i]);
  }
  EEPROM.commit();
  Blink_led(0,500);
  ESP.restart();
}

void readSysIdFromEEPROM() 
{
  for (int i = 0; i < sizeof(sys_id); i++) 
  {
    sys_id[i] = EEPROM.read(SYS_ID_ADDR + i);
  }          
  sys_id[sizeof(sys_id) - 1] = '\0'; 
}
 String readUpdateLinkFromEEPROM() 
{
  String link = "";
  for (int i = 0; i < 100; i++) 
  {
    char c = EEPROM.read(UPDATE_LINK_ADDER + i);
    if (c == '\0') break;
    link += c;
  }
  return link;
}
void checkForUpdates(String binUrl) 
{
  Serial.println("Checking for updates...");
  Firebase.endStream(fbdo);
  fbdo.clear();
  WiFiClientSecure otaClient;  // ✅ fresh client
  otaClient.setInsecure();     // skip SSL verification

  startTime = millis();
  t_httpUpdate_return result = ESPhttpUpdate.update(otaClient,binUrl);

  if (millis() - startTime >= timeout) 
  {
    Serial.println("Update timeout occurred");
  }

  switch (result) 
  {
    case HTTP_UPDATE_FAILED:
      Serial.printf("Update failed. Error (%d): %s\n", 
      ESPhttpUpdate.getLastError(), 
      ESPhttpUpdate.getLastErrorString().c_str());
      break;

    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("No updates available.");
      break;

    case HTTP_UPDATE_OK:
      Serial.println("Update successful. Rebooting...");
      break;
  }
}

void resetSwitchesIfPowerOn() {
  String reason = ESP.getResetReason();
  Serial.println("Reset Reason: " + reason);

  // Check if reset was due to Power-on or External reset
  if (reason.indexOf("Power") != -1 || reason.indexOf("External") != -1) {
    Serial.println("Power-on detected. Resetting switches...");

    for (int i = 1; i <= MAX_SWITCH_NO; i++) {
      String sw = "SW" + String(i);

      // BUTTONS path
      String buttonsPath = "SYSTEM/" + String(sys_id) + "/BUTTONS/" + sw;
      if (!Firebase.setInt(fbdo, buttonsPath.c_str(), 3)) {
        Serial.println("Failed to reset " + buttonsPath + " : " + fbdo.errorReason());
      }

      // SWITCH_STATE path
      String statePath = "SYSTEM/" + String(sys_id) + "/SWITCH_STATE/" + sw;
      if (!Firebase.setInt(fbdo, statePath.c_str(), 3)) {
        Serial.println("Failed to reset " + statePath + " : " + fbdo.errorReason());
      }
    }
  }
}

void setup() 
{
 // ESP.wdtDisable();           // disable first
  //ESP.wdtEnable(8000);        // enable with timeout (ms) → max ~8s
  pinMode(wifiLed, OUTPUT);
  digitalWrite(wifiLed, HIGH); // OFF by default
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(TRANSFER_IND_PIN, OUTPUT);
  digitalWrite(TRANSFER_IND_PIN, HIGH); // OFF by default

  EEPROM.begin(EEPROM_SIZE);
  update_ota = EEPROM.read(EEPROM_UPDATE); // single byte
  readSysIdFromEEPROM();

  // WiFi setup
  wm.setConfigPortalTimeout(10);
  if (wm.autoConnect("JT_WIFI_SETUP", "Jyoti@2000")) 
  {
    Serial.begin(9600);
    delay(1000);
    Serial.println("WiFi connected");
    Serial.print("Firmware version: ");
    Serial.println(FIRMWARE_VERSION);
    Serial.print("system id: ");
    Serial.println(String(sys_id));
    // OTA update check
    if (update_ota == 1) 
    {
      String link = readUpdateLinkFromEEPROM();
      EEPROM.write(EEPROM_UPDATE, 0); // reset OTA flag
      EEPROM.commit();
      checkForUpdates(link);
    } 
    else if (EEPROM.read(EEPROM_UPDATE) != 0) 
    {
      EEPROM.write(EEPROM_UPDATE, 0);
      EEPROM.commit();
    }

    timeClient.begin();

    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;

    if (Firebase.signUp(&config, &auth, "", "")) 
    { // empty email/password for anonymous auth
      signupOK = true;
      config.token_status_callback = tokenStatusCallback;
      Firebase.begin(&config, &auth);
      Firebase.reconnectWiFi(true);
      delay(1000);

      Serial.println("sign-in successful!");

      // Initialize Firebase buttons and system info
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/UPDATE", 0);
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/RESET", 0);
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/SET_TIME", 0);
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);

      Firebase.set(fbdo, "SYSTEM/" + String(sys_id) + "/SYSTEM_INFO/FIRMWARE_VERSION", FIRMWARE_VERSION);
      Firebase.set(fbdo, "SYSTEM/" + String(sys_id) + "/SYSTEM_INFO/RESET_REASON", ESP.getResetInfo());
      Firebase.set(fbdo, "SYSTEM/" + String(sys_id) + "/SYSTEM_INFO/FREE_RAM", String(ESP.getFreeHeap()));
      resetSwitchesIfPowerOn();
      initStream();
      digitalWrite(wifiLed, LOW); 
    } 
    else 
    {
      Serial.println("Firebase sign-in failed!");
      return;
    }

  } 
  else 
  {
      //Serial.println("WiFi connection failed, offline mode");
    digitalWrite(wifiLed, HIGH); // LED OFF when offline
    offline_flag = true;
  }
  // Attach ISR
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleButtonPress, CHANGE);
}

void loop() 
{
  if (buttonPressed)
  {
    delay(1000);
    if(digitalRead(TRIGGER_PIN) == LOW)
    {
      buttonPressed = false;
      setupConfigPortal();
    }
   
  }
  if (WiFi.status() != WL_CONNECTED) 
  {
    digitalWrite(wifiLed, HIGH);
    if (offlineStartTime == 0) 
    {
      offlineStartTime = millis();   // mark the time when offline started
    }
    if (millis() - offlineStartTime >= OFFLINE_TIMEOUT) 
    {
      ESP.restart();   // reset ESP
    }
  } 
  else 
  {

    if (!fbdo.httpConnected() && (millis() - lastStreamReconnect >= STREAM_RECONNECT_INTERVAL)) 
    {
      lastStreamReconnect = millis();
      Serial.println("Stream not connected - attempting to begin stream");
      initStream();
    }
    digitalWrite(wifiLed, LOW);
    offlineStartTime = 0;
    handleStream();
    settime();
    //get_serial();
    applySchedules();
  }
 // ESP.wdtFeed();              // feed watchdog
}
