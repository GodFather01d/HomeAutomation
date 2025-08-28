#define firmware_version 1.2

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
#define TRIGGER_PIN 14       //  D5   wifi setup pin access point

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

const char AP_ID[] = "JT_WiFi_SETUP", AP_PASS[] = "Jyoti@2000";
const long utcOffsetInSeconds = 19800;
unsigned long startTime;
const unsigned long timeout = 20000;

unsigned long offlineStartTime = 0; 
const unsigned long OFFLINE_TIMEOUT = 2UL * 60UL * 1000UL; 
char sys_id[32]; 
volatile bool buttonPressed = false, system_status = true, signupOK = false, offline_flag = false;
uint8_t update_ota = 1, curr_switch_on = 0, curr_switch_off = 0,time_hours = 0, time_mint = 0,currentMonth = 0,currentDay = 0;
uint16_t offline_time = 5, offline_counter = 0,currentYear = 0; 
int on_transfer_counter[MAX_SWITCH_NO + 1] = {0};
int off_transfer_counter[MAX_SWITCH_NO + 1] = {0};

unsigned long lastOnMillis[MAX_SWITCH_NO + 1] = {0};
unsigned long lastOffMillis[MAX_SWITCH_NO + 1] = {0};
const unsigned long PRINT_INTERVAL = 15000; 

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
void settime()
{
  if (buttonPressed) return;
  if(currentYear < 2000)
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
    Serial.println("Stream read error: " + fbdo.errorReason());
    return;
  }
  if (fbdo.streamTimeout()) 
  {
    Serial.println("Stream timeout, resuming...");
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
          for(int i = 0;i < 3;i++)
          {
            serial_out(switch_no,1);
            delay(1000);
          } 
          //serial_out(switch_no,1);
        } 
        else if (value == 0)
        {
          for(int i = 0;i < 3;i++)
          {
            serial_out(switch_no,0);
            delay(1000);
          } 
        }
        String basePath = "SYSTEM/" + String(sys_id) + "/BUTTONS/SW" + swNumber;
        Firebase.setInt(fbdo, basePath, 3);
        system_status = Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);
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
      if (dataPath.startsWith("/ONLINE_STATUS")) 
      {
        if (value == 0) 
        {
          system_status = Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);
        } 
      }
    }
  }
}
void readlink_from_firebase()
{
  EEPROM.put(EEPROM_UPDATE, 1);
  String path = "SYSTEM/UPDATE_LINK";
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
void send_notification(int switch_no,int state)
{
  String basePath = "SYSTEM/" + String(sys_id) + "/NOTIFICATION/SW" + String(switch_no);
  String update_time = (String(time_hours)+" : "+String(time_mint)+ "_" );
  String update_date = (String(currentDay)+" / "+String(currentMonth)+" / "+String(currentYear));
  if(Firebase.setString(fbdo, "SYSTEM/" + String(sys_id) + "/NOTIFICATION/NOTIFICATION_TIME/",update_time + update_date))
  {
    Firebase.setInt(fbdo, basePath, state);
  }
}
void send_switch_state(int switch_no, bool state) 
{
  String basePath = "SYSTEM/" + String(sys_id) + "/SWITCH_STATE/SW" + String(switch_no);
  Firebase.setInt(fbdo, basePath, state);
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
            send_switch_state(switchNo, true);
            send_notification(switchNo, 1);
            on_transfer_counter[switchNo] = 4;
          } 
          else if (state == 0) 
          {
            send_switch_state(switchNo, false);
            send_notification(switchNo, 0);
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
      if (on_transfer_counter[i] < 3 && now - lastOnMillis[i] >= PRINT_INTERVAL) 
      {
        serial_out(i,1);
        on_transfer_counter[i]++;
        lastOnMillis[i] = now; // update last print time
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
      if (off_transfer_counter[i] < 3 && now - lastOffMillis[i] >= PRINT_INTERVAL) 
      {
        serial_out(i,0);
        off_transfer_counter[i]++;
        lastOffMillis[i] = now;
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
  String path = String(sys_id) + "_" + String(switch_no) + "_" + String(state);
  Serial.println(path);

}
 void setupConfigPortal() 
 {
  if (Firebase.endStream(fbdo)) 
  {
    Serial.println("Stream stopped successfully");
  } 
  else 
  {
    Serial.println("Failed to stop stream");
  }
  for(int i = 0;i<= 5;i++)
  {
    digitalWrite(wifiLed,HIGH);
    delay(500);
    digitalWrite(wifiLed,LOW);
    delay(500);
  }
  WiFiManagerParameter sys_id_param("sys_id", "System ID", sys_id, 32);
  wm.addParameter(&sys_id_param);
  wm.setConfigPortalTimeout(180);

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
  for(int i = 0;i<= 5;i++)
  {
    digitalWrite(wifiLed,HIGH);
    delay(500);
    digitalWrite(wifiLed,LOW);
    delay(500);
  }
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
void setup()
{
  ESP.wdtDisable(); 
  wm.setConfigPortalTimeout(1);
  wm.autoConnect("JT_WIFI_SETUP","Jyoti@2000");
  pinMode(wifiLed ,OUTPUT);
  digitalWrite(wifiLed, HIGH); 
  pinMode(TRIGGER_PIN ,INPUT_PULLUP);
  EEPROM.begin(EEPROM_SIZE);
  update_ota =  EEPROM.read(EEPROM_UPDATE);
  readSysIdFromEEPROM();
  Serial.begin(9600);
  delay(1000);
  Serial.print("firmware version: ");
  Serial.println(firmware_version);

  if(WiFi.status() == WL_CONNECTED ) 
  {
    Serial.println("connected");
    if(update_ota == 1) 
    {
      String link = readUpdateLinkFromEEPROM();
      EEPROM.put(EEPROM_UPDATE, 0);
      EEPROM.commit();
      checkForUpdates(link);
    }
    else
    { 
      if (EEPROM.read(EEPROM_UPDATE) != 0)
      {
        EEPROM.put(EEPROM_UPDATE, 0);
        EEPROM.commit();
      }
    }
  }
  if(WiFi.status() == WL_CONNECTED) 
  { 
    timeClient.begin();
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    if (Firebase.signUp(&config, &auth, "", ""))
    {
      signupOK = true;
      config.token_status_callback = tokenStatusCallback; 
      Firebase.begin(&config, &auth);
      Firebase.reconnectWiFi(true);
      delay(1000);
    } 
    if(signupOK)
    {
      Serial.println("Sign-in successful!");
      Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/UPDATE", 0);
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/RESET", 0);
      Firebase.setInt(fbdo, "SYSTEM/" + String(sys_id) + "/BUTTONS/ONLINE_STATUS", 1);
      Firebase.set(fbdo, "SYSTEM/" + String(sys_id) + "/FIRMWARE_VERSION", firmware_version);

      for(int i = 1;i<= MAX_SWITCH_NO;i++)
      {
        String basePath = "SYSTEM/" + String(sys_id) + "/BUTTONS/SW" + String(i);
        Firebase.setInt(fbdo, basePath, 3);
      }
      readSchedulesFromFirebase();
      initStream();
    }
    delay(1000);
  }
  else
  {
    digitalWrite(wifiLed, HIGH);
    offline_flag = true; 
  }
  attachInterrupt(digitalPinToInterrupt(TRIGGER_PIN), handleButtonPress, CHANGE);
}

void loop() 
{
  if (buttonPressed || digitalRead(TRIGGER_PIN) == LOW)
  {
    buttonPressed = false;
    setupConfigPortal();
  }
  if (offline_flag || WiFi.status() != WL_CONNECTED || !system_status) 
  {
    Serial.println("Offline");
    if (offlineStartTime == 0) 
    {
      offlineStartTime = millis();   // mark the time when offline started
    }

    if (millis() - offlineStartTime >= OFFLINE_TIMEOUT) 
    {
      Serial.println("Offline for 5 minutes, restarting...");
      ESP.restart();   // reset ESP
    }
  } 
  else 
  {
    offlineStartTime = 0;
    handleStream();
    settime();
    get_serial();
    applySchedules();
  }
}
