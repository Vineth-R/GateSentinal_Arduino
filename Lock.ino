#include <WiFi.h>
#include <FirebaseESP32.h>

// -------------------- Wi-Fi --------------------
#define WIFI_SSID "Vineth"
#define WIFI_PASSWORD "Vanitha@2003"

// -------------------- Firebase --------------------
#define MOBILE_FIREBASE_HOST "gatesentinal-default-rtdb.firebaseio.com"
#define MOBILE_FIREBASE_AUTH "Nv4ZJVskYFkURBmCAmD8xMMTrxCKoJOWruAJY0Fs"

FirebaseData fbdo;
FirebaseConfig config;
FirebaseAuth auth;

// -------------------- Pins --------------------
#define RELAY_PIN 25            // Relay controlling door lock (LOW = unlock, HIGH = lock)
#define LIMIT_SWITCH_PIN 26     // Limit switch (HIGH = door open, LOW = door closed)

// -------------------- Door State Variables --------------------
bool doorIsCurrentlyOpen = false;
unsigned long doorCloseDetectedTime = 0;
bool waitingToLock = false;
unsigned long long lastProcessedTimestamp = 0;

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // Start locked
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);

  // Determine initial door state
  if (digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
    doorIsCurrentlyOpen = true;
    Serial.println("System start: Door is OPEN.");
  } else {
    doorIsCurrentlyOpen = false;
    Serial.println("System start: Door is CLOSED.");
  }

  // Wi-Fi connection
  Serial.print("Connecting to Wi-Fi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Wi-Fi connected");
  Serial.print("📶 IP Address: ");
  Serial.println(WiFi.localIP());

  // Firebase setup
  config.host = MOBILE_FIREBASE_HOST;
  config.signer.tokens.legacy_token = MOBILE_FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  Serial.println("✅ Mobile Firebase Database initialized.");
}

void loop() {
  monitorDoorState();  // Check limit switch and auto-lock

  if (Firebase.ready()) {
    if (Firebase.getJSON(fbdo, "/gateControl")) {
      if (fbdo.httpCode() == 200) {
        FirebaseJson &json = fbdo.jsonObject();
        FirebaseJsonData result;
        String command = "";
        unsigned long long lastUpdated = 0;

        if (json.get(result, "command") && result.success) {
          command = result.to<String>();
          Serial.print("🟢 Command: ");
          Serial.println(command);
        }
        if (json.get(result, "lastUpdated") && result.success) {
          lastUpdated = (unsigned long long)result.to<int64_t>();
          Serial.print("🕒 lastUpdated: ");
          Serial.println(lastUpdated);
        }

        if (command == "unlock" && lastUpdated > lastProcessedTimestamp) {
          Serial.println("🔓 Unlock command received.");
          openDoor();

          Firebase.setString(fbdo, "/gateControl/command", "idle");
          Firebase.setString(fbdo, "/gateControl/status", "unlocked");

          lastProcessedTimestamp = lastUpdated;
        }
      }
    }
  }
  delay(200);
}

// -------------------- Functions --------------------
void monitorDoorState() {
  bool doorOpenNow = (digitalRead(LIMIT_SWITCH_PIN) == HIGH);

  if (doorOpenNow) {
    if (!doorIsCurrentlyOpen) {
      doorIsCurrentlyOpen = true;
      waitingToLock = false; // Cancel pending auto-lock
      Serial.println("🚪 Door opened.");
    }
  } else {
    if (doorIsCurrentlyOpen) {
      doorIsCurrentlyOpen = false;
      Serial.println("🚪 Door closed. Starting 5s delay to lock.");
      doorCloseDetectedTime = millis();
      waitingToLock = true;
    }

    if (waitingToLock && (millis() - doorCloseDetectedTime >= 5000)) {
      closeDoor();
      waitingToLock = false;
      Serial.println("🔒 Door locked automatically after closing delay.");
    }
  }
}

void openDoor() {
  digitalWrite(RELAY_PIN, LOW);  // Unlock
  Serial.println("✅ Door unlocked.");
}

void closeDoor() {
  digitalWrite(RELAY_PIN, HIGH); // Lock
  Serial.println("🔒 Door locked.");
}