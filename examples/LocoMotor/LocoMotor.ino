#include <Arduino.h>
#include <MaerklinMotorola.h>
#include <Adafruit_NeoPixel.h>

// ==========================================
// 1. KONFIGURATION
// ==========================================
#define MOTOR_TYPE 1  // 1=HLA (Gross), 2=Glockenanker (Klein)

// ==========================================
// 2. PIN DEFINITIONEN (Seeed XIAO RP2040)
// ==========================================
#define MOTOR_PIN_A     D7  
#define MOTOR_PIN_B     D8  
#define BEMF_PIN_A      A0  // ACHTUNG: Spannungsteiler erforderlich (max 3.3V)!
#define BEMF_PIN_B      A1  // ACHTUNG: Spannungsteiler erforderlich (max 3.3V)!
#define DCC_MM_SIGNAL   D2  // Interrupt Pin für Signal

// Debug LEDs
#define PIN_INT_RED     17
#define PIN_INT_GREEN   16
#define PIN_INT_BLUE    25
#define NEO_PIN         12 
#define NEO_PWR_PIN     11 
#define NUMPIXELS       1

// ==========================================
// 3. PARAMETER
// ==========================================

// EEPROM-gespeicherte Konfiguration
struct MotorConfig {
  int pwm_freq;
  int pwm_min_moving;
  int kick_max_time;
} config;


Adafruit_NeoPixel pixels(NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
MaerklinMotorola mm(DCC_MM_SIGNAL);

const int PWM_RANGE = 1023;
const int PWM_MAX   = 1023; 

// Parameter je nach Motortyp
#if MOTOR_TYPE == 1
  // HLA (Märklin Scheibenkollektor/Trommel)
  const int KICK_PWM        = 1023;   
  const int BEMF_THRESHOLD  = 120;
  const int BEMF_SAMPLE_INT = 15;     
#else
  // Glockenanker (Faulhaber/Maxon)
  const int KICK_PWM        = 600;    
  const int BEMF_THRESHOLD  = 80;    
  const int BEMF_SAMPLE_INT = 10;     
#endif

const int MM_ADDRESS       = 24;
const int MM_TIMEOUT_MS    = 1500;
const int MM2_LOCK_TIME    = 5000; 

// Status-Variablen
unsigned long lastCommandTime = 0;
unsigned long lastMM2Seen     = 0;
unsigned long kickstartBegin  = 0;
unsigned long lastVisUpdate   = 0;
unsigned long lastBemfMeasure = 0; 
unsigned long directionSwitchTime = 0; // Für Totzeit beim Umschalten

int targetPwm  = 0;
int lastSpeed  = 0; 
bool isKickstarting = false;
bool isBraking = false; // Status für Totzeit

MM2DirectionState currDirection = MM2DirectionState_Forward;
MM2DirectionState targetDirection = MM2DirectionState_Forward;

// MM1 Toggle Variablen
bool lastChangeDirInput = false; 
unsigned long lastChangeDirTs = 0;

// ==========================================
// 4. HELPER FUNKTIONEN
// ==========================================

void saveConfiguration() {
  EEPROM.put(0, config);
  EEPROM.commit(); // Wichtig für RP2040!
  Serial.println("Konfiguration gespeichert.");
}

void loadConfiguration() {
  EEPROM.get(0, config);

  // Prüfen ob EEPROM-Werte sinnvoll sind, sonst Defaults laden
  if (config.pwm_freq <= 0 || config.pwm_freq > 40000) {
    Serial.println("EEPROM leer oder korrupt. Lade Standardwerte.");
    #if MOTOR_TYPE == 1
      config.pwm_freq       = 400;
      config.pwm_min_moving = 350;
      config.kick_max_time  = 150;
    #else
      config.pwm_freq       = 20000;
      config.pwm_min_moving = 80;
      config.kick_max_time  = 80;
    #endif
    saveConfiguration(); // Und direkt speichern für den nächsten Start
  } else {
    Serial.println("Konfiguration aus EEPROM geladen.");
  }
}

void printConfiguration() {
  Serial.println("--- Aktuelle Konfiguration ---");
  Serial.print("PWM Frequenz (pwm_freq): ");
  Serial.println(config.pwm_freq);
  Serial.print("Min. PWM (pwm_min_moving): ");
  Serial.println(config.pwm_min_moving);
  Serial.print("Kickstart Zeit (kick_max_time): ");
  Serial.println(config.kick_max_time);
  Serial.println("------------------------------");
  Serial.println("Befehle: 'set <var> <val>', 'save', 'help'");
}


void setIntLed(int pin, bool on) {
  digitalWrite(pin, on ? LOW : HIGH); // Low-Active beim Xiao
}

// Interrupt Service Routine für den Decoder
void isr() { mm.PinChange(); }

// Hardware Ansteuerung (H-Brücke)
void writeMotorHardware(int pwm, MM2DirectionState dir) {
  if (pwm > PWM_RANGE) pwm = PWM_RANGE;
  if (pwm < 0) pwm = 0;
  
  // Sicherstellen, dass bei Richtungswechsel oder Bremsen alles sauber ist
  if (dir == MM2DirectionState_Forward) {
    digitalWrite(MOTOR_PIN_B, LOW); 
    analogWrite(MOTOR_PIN_A, pwm);
  } else {
    digitalWrite(MOTOR_PIN_A, LOW); 
    analogWrite(MOTOR_PIN_B, pwm);
  }
}

int readBEMF() {
  // 1. Motor in Freilauf schalten (PWM aus)
  digitalWrite(MOTOR_PIN_A, LOW);
  digitalWrite(MOTOR_PIN_B, LOW);
  
  // 2. Warten (Zeit für Induktionsabbau)
  delayMicroseconds(500); 
  
  // 3. Messen (Differenzverstärker Prinzip simulieren)
  int valA = analogRead(BEMF_PIN_A);
  int valB = analogRead(BEMF_PIN_B);
  
  return abs(valA - valB);
}

int getLinSpeed(int step) {
  if (step == 0) return 0;
  if (step >= 14) return PWM_MAX;
  return map(step, 1, 14, config.pwm_min_moving, PWM_MAX);
}

void updateVisualDebug(int speedStep, bool mm2Locked, bool kickstart) {
  if (millis() - lastVisUpdate < 50) return;
  lastVisUpdate = millis();

  setIntLed(PIN_INT_RED, mm2Locked); 
  setIntLed(PIN_INT_BLUE, false); // Aus, da F1 entfernt

  if (kickstart) {
    pixels.setPixelColor(0, pixels.Color(255, 255, 255)); // Weißer Blitz
  } 
  else if (targetPwm == 0) {
    int val = (millis() / 20) % 255;
    int breath = (val > 127) ? 255 - val : val;
    pixels.setPixelColor(0, pixels.Color(0, 0, breath * 2)); // Blaues Atmen
  } 
  else {
    // Fahrtanzeige
    int r = map(speedStep, 0, 14, 0, 255);
    int g = map(speedStep, 0, 14, 255, 0);
    pixels.setPixelColor(0, pixels.Color(r, g, 0));
  }
  pixels.show();
}

// ==========================================
// 5. SETUP
// ==========================================
void setup() {
  // Serielle Kommunikation & EEPROM
  Serial.begin(115200);
  delay(2000); // Zeit geben um Serial Monitor zu öffnen
  EEPROM.begin(sizeof(MotorConfig));
  loadConfiguration();
  printConfiguration();

  // LEDs
  pinMode(NEO_PWR_PIN, OUTPUT); digitalWrite(NEO_PWR_PIN, HIGH); 
  delay(10); 
  pixels.begin(); pixels.setBrightness(40); 

  pinMode(PIN_INT_RED, OUTPUT);   digitalWrite(PIN_INT_RED, HIGH);
  pinMode(PIN_INT_GREEN, OUTPUT); digitalWrite(PIN_INT_GREEN, HIGH);
  pinMode(PIN_INT_BLUE, OUTPUT);  digitalWrite(PIN_INT_BLUE, HIGH);

  // Motor & BEMF
  pinMode(BEMF_PIN_A, INPUT);
  pinMode(BEMF_PIN_B, INPUT);
  analogReadResolution(12); 

  pinMode(MOTOR_PIN_A, OUTPUT); 
  pinMode(MOTOR_PIN_B, OUTPUT);
  
  // Decoder Interrupt
  attachInterrupt(digitalPinToInterrupt(DCC_MM_SIGNAL), isr, CHANGE);
  
  // PWM
  analogWriteFreq(config.pwm_freq);
  analogWriteRange(PWM_RANGE);
  
  writeMotorHardware(0, MM2DirectionState_Forward);
  lastCommandTime = millis(); 
}

void handleSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    char cmd[20], var[30];
    int val;
    sscanf(input.c_str(), "%s %s %d", cmd, var, &val);

    if (strcmp(cmd, "set") == 0) {
      if (strcmp(var, "pwm_freq") == 0) {
        config.pwm_freq = val;
        analogWriteFreq(config.pwm_freq); // Frequenz direkt anwenden
        Serial.print("Setze pwm_freq auf "); Serial.println(val);
      } else if (strcmp(var, "pwm_min_moving") == 0) {
        config.pwm_min_moving = val;
        Serial.print("Setze pwm_min_moving auf "); Serial.println(val);
      } else if (strcmp(var, "kick_max_time") == 0) {
        config.kick_max_time = val;
        Serial.print("Setze kick_max_time auf "); Serial.println(val);
      } else {
        Serial.println("Unbekannte Variable.");
      }
    } else if (strcmp(cmd, "save") == 0) {
      saveConfiguration();
    } else if (strcmp(cmd, "help") == 0) {
      printConfiguration();
    } else {
      Serial.println("Unbekannter Befehl.");
    }
  }
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  handleSerial(); // Serielle Befehle abarbeiten
  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();
  unsigned long now = millis();

  // --- 1. Signal Dekodierung ---
  if (Data && !Data->IsMagnet && Data->Address == MM_ADDRESS) {
    lastCommandTime = now;
    setIntLed(PIN_INT_GREEN, true); // Signal OK

    bool mm2Locked = (now - lastMM2Seen < MM2_LOCK_TIME);
    if (Data->IsMM2) lastMM2Seen = now;

    // --- NEU: Robuste Richtungserkennung ---
    if (mm2Locked && Data->IsMM2) {
        static uint8_t cntFwd = 0;
        static uint8_t cntBwd = 0;

        if (Data->MM2Direction == MM2DirectionState_Forward) {
            if (cntFwd < 10) cntFwd++;
            cntBwd = 0;
        } 
        else if (Data->MM2Direction == MM2DirectionState_Backward) {
            if (cntBwd < 10) cntBwd++;
            cntFwd = 0;
        }

        // Debounce: Erst bei 3 bestätigten Paketen umschalten
        if (cntFwd >= 3 && targetDirection != MM2DirectionState_Forward) {
            targetDirection = MM2DirectionState_Forward;
        }
        if (cntBwd >= 3 && targetDirection != MM2DirectionState_Backward) {
            targetDirection = MM2DirectionState_Backward;
        }
    } 
    else if (!mm2Locked) {
      // MM1 Fallback (Toggle Logik für alte Zentralen)
      if (Data->ChangeDir && !lastChangeDirInput) {
        // Hier nutzen wir Zeit als Filter (250ms Prellschutz)
        if (now - lastChangeDirTs > 250) { 
          targetDirection = (targetDirection == MM2DirectionState_Forward) 
                            ? MM2DirectionState_Backward : MM2DirectionState_Forward;
          lastChangeDirTs = now;
        }
      }
    }
    lastChangeDirInput = Data->ChangeDir; 

    // Bei Stillstand Richtung sofort intern synchronisieren (vermeidet Anfahr-Ruckler)
    if (targetPwm == 0 && !isBraking) currDirection = targetDirection;

    // Geschwindigkeit setzen
    int displaySpeed = 0; 
    if (Data->IsMM2 && Data->MM2FunctionIndex != 0) {
       displaySpeed = lastSpeed; 
    } else {
       targetPwm = getLinSpeed(Data->Speed);
       displaySpeed = Data->Speed;
       lastSpeed = Data->Speed; 
    }

    updateVisualDebug(displaySpeed, mm2Locked, isKickstarting);
    setIntLed(PIN_INT_GREEN, false);
  } 

  // --- 2. Motor State Machine (Deadtime & Kickstart) ---

  // A. Richtungswechsel erkennen -> Bremsphase einleiten
  if (currDirection != targetDirection && !isBraking) {
      isBraking = true;
      directionSwitchTime = now;
      writeMotorHardware(0, currDirection); // Sofort Motor aus
      isKickstarting = false; // Kickstart abbrechen falls aktiv
  }

  // B. Bremsphase (Deadtime) abwarten
  if (isBraking) {
      // 200ms warten bevor neue Richtung aktiv wird
      if (now - directionSwitchTime > 200) {
          isBraking = false;
          currDirection = targetDirection; // Jetzt Richtung übernehmen
      } else {
          writeMotorHardware(0, currDirection); // Weiterhin aus
          return; // Rest des Loops überspringen
      }
  }

  // C. Kickstart Logik
  static int previousPwm = 0;
  
  // Trigger
  if (previousPwm == 0 && targetPwm > 0 && config.kick_max_time > 0 && !isBraking) {
      isKickstarting = true;
      kickstartBegin = now;
      lastBemfMeasure = 0; 
  }
  // Abbruch
  if (targetPwm == 0) isKickstarting = false;
  
  if (isKickstarting) {
    if (now - kickstartBegin >= config.kick_max_time) {
      isKickstarting = false; // Timeout
    } 
    else {
      // Messen ob Motor schon dreht
      if (now - lastBemfMeasure > BEMF_SAMPLE_INT) {
         int currentBEMF = readBEMF();
         lastBemfMeasure = now;
         if (currentBEMF > BEMF_THRESHOLD) {
            isKickstarting = false; // Erfolg -> Normalbetrieb
         }
      }
      
      if (isKickstarting) {
        writeMotorHardware(KICK_PWM, currDirection);
      }
    }
  } 
  
  // D. Normaler Fahrbetrieb
  if (!isKickstarting && !isBraking) {
      writeMotorHardware(targetPwm, currDirection);
  }
  
  previousPwm = targetPwm; 

  // --- 3. Failsafe (Signalverlust) ---
  if (now - lastCommandTime > MM_TIMEOUT_MS) {
    targetPwm = 0;
    isKickstarting = false;
    isBraking = false;
    writeMotorHardware(0, currDirection); 
    
    // Rotes Blinken
    if ((now / 250) % 2) pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    else pixels.setPixelColor(0, 0);
    pixels.show();
    setIntLed(PIN_INT_RED, false);
  }
}
