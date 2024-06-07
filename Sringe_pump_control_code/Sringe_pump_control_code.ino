// ========== USER INPUTS ==========
// Distance to move off from home position ...
#define MOVE_OFF_REVS  40 // = (60[mm] / 1.5[mm/rev])

// Motor speed (revs/min I believe) ...
#define MOTOR_SPEED 100

// Dosage
#define DOSE 60 //dose volume in microlitre [ul]

// Dose revs = 4 * dose[ul] / ( thread_pitch[mm/rev] * pi * diameter[mm]^2 )
float n_revs = 4 * DOSE / (1.5 * PI * pow(4.5, 2)); 


// ========== MOTORS ==========
// Include library
#include <Stepper.h>

// Stepper motor pins ...
int DIR_PINS[] = {
  30, 31, 32, 33, 34
};
int PUL_PINS[] = {
  22, 23, 24, 25, 26
};

// Set up motors ...
const int steps_per_rev = 200*4*4; // 200[steps/rev] * 4[microsteps/step] * 4[pulses/step from stepper library]
Stepper Steppers[] {
  Stepper(steps_per_rev, PUL_PINS[0], DIR_PINS[0]), 
  Stepper(steps_per_rev, PUL_PINS[1], DIR_PINS[1]), 
  Stepper(steps_per_rev, PUL_PINS[2], DIR_PINS[2]), 
  Stepper(steps_per_rev, PUL_PINS[3], DIR_PINS[3]), 
  Stepper(steps_per_rev, PUL_PINS[4], DIR_PINS[4])
};

byte in_Byte = B00000000;

// ========== LIMITS ==========
// Pin locations of limits ...
int LIMS[] {
  46, 47, 48, 49, 50
};

// Define travel for soft limits ...
unsigned long travel[] = {
  0, 0, 0, 0, 0
};

// ========== LCD ==========
// Set up LCD display (I2C address 0x27, 16 column and 2 rows) ...
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);


// ========== PANEL INPUTS ==========
// Button pin locations ...
#define HOME_BTN 45
#define BJOG_BTN 51
#define FJOG_BTN 52
#define DOSE_BTN 53
#define POT      A0

// Initial potentiometer value ...
int motor_select = 0;



// ========== SETUP CODE ==========
void setup() {
  // Serial setup ...
  Serial.begin(9600);
  
  // Motor setup ...
  for(int idx = 0; idx < 5; idx ++){
    Steppers[idx].setSpeed(MOTOR_SPEED);
  }
  
  // Step pins (22 - 26) as outputs ...
  DDRA = B00011111;

  // Dir pins (30 - 34) as outputs ...
  DDRC = B11111000; 

  // Limit setup ...
  for(int idx = 0; idx < 5; idx ++){
    pinMode(LIMS[idx], INPUT);
  }

  // LCD setup ...
  lcd.init(); 
  lcd.backlight();

  // Panel input setup ...
  pinMode(HOME_BTN, INPUT);
  pinMode(BJOG_BTN, INPUT);
  pinMode(FJOG_BTN, INPUT);
  pinMode(DOSE_BTN, INPUT);
  pinMode(POT, INPUT);

  // Display a nice message ...
  lcd.setCursor(2, 0);
  lcd.print("Pump it with");
  lcd.setCursor(3, 1);
  lcd.print("Team SUGAR");
  delay(2500);
  lcd.clear();

  // Perform initial homing ...
  home_motors();
}

void loop() {
  // ========== MOTOR SELECTION ==========
  // Calculate which motor to run based upon potentiometer input ...
  motor_select = map(analogRead(POT), 0, 1023, 4, 0);

  // Display selected motor to screen ...
  lcd.setCursor(0, 0);
  lcd.print("Motor ");
  lcd.print(motor_select + 1);
  lcd.print(" selected");

  // Main operations ...
  if ( !digitalRead(HOME_BTN) ) {
    home_motors();
  }
  
  if ( !digitalRead(BJOG_BTN) || !digitalRead(FJOG_BTN) ) {
    jog();
  }

  if (!digitalRead(DOSE_BTN) && travel[motor_select] < long(260000 - n_revs*steps_per_rev)) {
    dose();
  }

  // Idle ...
  lcd.setCursor(0, 1);
  lcd.print("Idle ...        ");

  for ( int idx = 0; idx < 4; idx++ ) {
    Serial.print(travel[idx]);
    Serial.print(" \t");
  }
  Serial.println(travel[4]);
}



// ========== HOMING ==========
void home_motors() {
  // Display what you're doing ...
  lcd.setCursor(0, 1);
  lcd.print("Homing all motors");

  // Motor direction down ...
  PORTC = B00000000;
  
  while (1) {
    // Check limits ...
    for ( int idx = 0; idx < 5; idx++ ) {
      bitWrite(in_Byte, idx, !digitalRead(LIMS[idx]));
    }
    
    // If all limits are hit, break loop ...
    if ( in_Byte == B00000000 ) {
      break;
    }
        
    // Drive motors into limits ...
    singleStep(in_Byte);
      
  }

  // Reset travel array ...
  for ( int idx = 0; idx <5; idx++ ) {
    travel[idx] = 0;  
  }
  
  
  for ( int idx = 0; idx < 5; idx++ ) {
    Steppers[idx].setSpeed(250);
    for ( int revs = 0; revs < 20*MOVE_OFF_REVS; revs ++ ) {
      Steppers[idx].step(-160);
      travel[idx] += 160;
    }
    Steppers[idx].setSpeed(MOTOR_SPEED);
  }   
}


// ========== SINGLE STEP - FOR HOMING ==========
void singleStep(byte _inByte) {
  for(int idx = 0; idx < 32; idx ++){
    PORTA = _inByte; //B00011111;
    PORTA = B00000000;
  }
      
  delayMicroseconds(1000);
}


// ========== JOGGING ==========
void jog() {
  // If jogging button pressed ... display that you're jogging ...
  lcd.setCursor(0, 1);
  lcd.print(digitalRead(BJOG_BTN)?"Jogging Forward ":"Jogging Backward");

  while ((!digitalRead(BJOG_BTN) && travel[motor_select] > 0) || (!digitalRead(FJOG_BTN) && travel[motor_select] < 260000)){
    // Decide direction based upon which pin is pressed ...
    Steppers[motor_select].step(digitalRead(BJOG_BTN)?-25:25);

    // Add location to travel variable
    travel[motor_select] += digitalRead(BJOG_BTN)?25:-25;
  }
}

// ========= DOSING ==========
void dose() {
  lcd.setCursor(0, 1);
  lcd.print("Dosing ");
  lcd.print(DOSE);
  lcd.print(" uL ...");

  // Dose integer n revolutions worth ...
  for (int rev = 0; rev < int(n_revs); rev ++) {
    Steppers[motor_select].step(-steps_per_rev);
    travel[motor_select] += steps_per_rev;
  }

  // Dose the remaining steps ...
  Steppers[motor_select].step(int((int(n_revs) - n_revs)*steps_per_rev));
  travel[motor_select] += int((n_revs - int(n_revs))*steps_per_rev);
}
