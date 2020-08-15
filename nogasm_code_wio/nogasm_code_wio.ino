// Nogasm for Wio Terminal
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 */
//=======Libraries===============================
#include "RunningAverage.h"
// Replace later with SD card support? No true EEPROM on WIO.
#include <FlashStorage.h>

#include "TFT_eSPI.h"
#include "Free_Fonts.h"

//=======Hardware Setup===============================
//Motor PWM out
#define MOTPIN D6

//Pressure Sensor Analog In
#define BUTTPIN A8

//Max pressure
#define MAXPRES 4030

//Motor min and max speeds
#define MOT_MAX 250 // Motor PWM maximum
#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

//Setup display.
TFT_eSPI tft;
TFT_eSprite screen = TFT_eSprite(&tft);
TFT_eSprite sprite = TFT_eSprite(&tft);

//=======States=====================================
//Define all enums for state machines
enum MainStates{STATE_MANUAL,STATE_AUTO,STATE_SETTINGS};
MainStates mainstate = STATE_MANUAL;
int laststate = -1;

enum ButtonStates{BUTTON_NONE,BUTTON_A,BUTTON_B,BUTTON_C,BUTTON_UP,BUTTON_DOWN,BUTTON_LEFT,BUTTON_RIGHT,BUTTON_PRESS};
ButtonStates buttonstate = BUTTON_NONE;

enum SettingsStates{SET_BEGIN, SET_SPEED = SET_BEGIN, SET_AREF, SET_END = SET_AREF};
SettingsStates settingsstate = SET_SPEED;

//=======Timing=====================================
//Main loop frequency
#define FREQUENCY 60 //Hz

//Period between main loop runs
#define PERIOD (1000/FREQUENCY)

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY*RA_HIST_SECONDS);

// Number of ticks between repeat button presses.
#define BUTTON_HOLD_TICKS 10

//=======Globals====================================
int pressure = 0;
int avgPressure = 0; //Running 25 second average pressure
float motorspeed = 0;
int sensitivity = 0; //orgasm detection sensitivity, persists through different states
int rampTimeS = 30; //Ramp-up time, in seconds
#define MAX_PLIMIT 600
int pLimit = MAX_PLIMIT; //Limit in change of pressure before the vibrator turns off
int maxSpeed = 170; //maximum speed the motor will ramp up to in automatic mode
int arefs[] = {AR_DEFAULT,AR_INTERNAL2V5,AR_INTERNAL2V0,AR_INTERNAL1V65,AR_INTERNAL1V0};
int aref = 0;

//=======Settings====================================
typedef struct {
    bool valid;
    int sensitivity;
    int maxSpeed;
    int aref;
} SetStruct;

FlashStorage(settings_storage, SetStruct);

// Read in saved settings and load them if valid.
SetStruct settings = settings_storage.read();

//=======Setup=======================================

void setup() {
    //Start Serial
    SerialUSB.begin(115200);

    //Start Display
    tft.begin();
    tft.setRotation(3); //Make it right side up.
    tft.fillScreen(TFT_BLACK);
    screen.setColorDepth(8);
    screen.createSprite(TFT_HEIGHT,TFT_WIDTH);
    screen.setFreeFont(&FreeMono12pt7b);
    sprite.setFreeFont(&FreeMono12pt7b);


    //Set pinmodes
    pinMode(BUTTPIN, INPUT);
    pinMode(MOTPIN, OUTPUT);
    pinMode(WIO_BUZZER, OUTPUT);
    pinMode(WIO_KEY_A, INPUT_PULLUP);
    pinMode(WIO_KEY_B, INPUT_PULLUP);
    pinMode(WIO_KEY_C, INPUT_PULLUP);
    pinMode(WIO_5S_UP, INPUT_PULLUP);
    pinMode(WIO_5S_DOWN, INPUT_PULLUP);
    pinMode(WIO_5S_LEFT, INPUT_PULLUP);
    pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
    pinMode(WIO_5S_PRESS, INPUT_PULLUP);

    // Load settings, or set to defaults.
    if (settings.valid == true){
        sensitivity = settings.sensitivity;
        maxSpeed = settings.maxSpeed;
        aref = settings.aref;
    }
    else {
        settings.valid = true;
        settings.sensitivity = sensitivity;
        settings.maxSpeed = maxSpeed;
        settings.aref = aref;
        settings_storage.write(settings);
    }

    //Set analog reference
    analogReference((eAnalogReference)arefs[aref]);
    //Set analog resolution for 0-4094 range
    analogReadResolution(12);
    //Average 32 samples on each read
    ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_32 | ADC_AVGCTRL_ADJRES(4);
}

//=======Functions====================================

// Read pressure and add it to the running average at a slower speed.
void read_pressure() {
    static uint8_t sampleTick = 0;

    //Disable motor during sensor read to reduce noise
    set_motor_speed(0);

    pressure = analogRead(BUTTPIN);

    //Alarm if over max pressure
    if (pressure > MAXPRES) {
      analogWrite(WIO_BUZZER, 200);
    } else {
      analogWrite(WIO_BUZZER, 0);
    }

    // Once per RA_TICK_PERIOD, add pressure to running average.
    sampleTick++;
    if (sampleTick % RA_TICK_PERIOD == 0){
        raPressure.addValue(pressure);
        avgPressure = raPressure.getAverage();
    }

    //Reenable motor
    set_motor_speed(motorspeed);

}

// Read all button presses. Handle repeat presses for direction keys.
void read_buttons(){
    static int laststate[9] = {0,1}; // Set the A key last state to start at 1, to avoid undesired button press on startup.
    static int currstate[9];
    static int holdcount = 0;
    static int buttons[] = {BUTTON_NONE,WIO_KEY_A,WIO_KEY_B,WIO_KEY_C,WIO_5S_UP,WIO_5S_DOWN,WIO_5S_LEFT,WIO_5S_RIGHT,WIO_5S_PRESS};

    // Skipping 0 and padding the buttons array with BUTTON_NONE so that the 'resting' buttonstate can be 0.
    for (int i = 1; i<9; i++){
        currstate[i] = digitalRead(buttons[i]);
    }

    // In all cases below, 'i' is referring to the position in the ButtonStates enum.
    for (int i = 1; i<9; i++){
        // Was pressed and now released.
        if (laststate[i] == LOW and currstate[i] == HIGH){
            // Held less than BUTTON_HOLD_TICKS? Trigger button press. Prevents extra press when exiting held state.
            if (holdcount < BUTTON_HOLD_TICKS) buttonstate = (ButtonStates)i;
            else buttonstate = BUTTON_NONE;
            // Reset hold count on button release.
            holdcount = 0;
            break;
        }
        // Direction buttons pressed and continue to be pressed.
        else if ((i == BUTTON_UP or i == BUTTON_DOWN or i == BUTTON_LEFT or i == BUTTON_RIGHT) and laststate[i] == LOW and currstate[i] == LOW){
            // Increment hold count.
            holdcount++;
            // Button held for BUTTON_HOLD_TICKS? Trigger button press.
            if (holdcount % BUTTON_HOLD_TICKS == 0) buttonstate = (ButtonStates)i;
            else buttonstate = BUTTON_NONE;
            break;
        }
        else {
            buttonstate = BUTTON_NONE;
        }
    }

    // Copy current state to last state.
    for (int i = 1; i<9; i++){
        laststate[i] = currstate[i];
    }
}

void set_state(){
    // What state are we in? Has a button been pressed that should switch states?
    switch(mainstate){
        case STATE_MANUAL:
            switch(buttonstate){
                case BUTTON_A:
                    mainstate = STATE_AUTO;
                    motorspeed = 0;
                    break;
                case BUTTON_C:
                    mainstate = STATE_SETTINGS;
                    motorspeed = 0;
                    set_motor_speed(motorspeed);
                    break;
            }
            break;
        case STATE_AUTO:
            switch(buttonstate){
                case BUTTON_A:
                    mainstate = STATE_MANUAL;
                    motorspeed = 0;
                    settings.sensitivity = sensitivity;
                    settings_storage.write(settings);
                    break;
                case BUTTON_C:
                    mainstate = STATE_SETTINGS;
                    motorspeed = 0;
                    set_motor_speed(motorspeed);
                    settings.sensitivity = sensitivity;
                    settings_storage.write(settings);
                    break;
            }
            break;
        case STATE_SETTINGS:
            switch(buttonstate){
                case BUTTON_A:
                    mainstate = STATE_MANUAL;
                    motorspeed = 0;
                    settings_storage.write(settings);
                    break;
            }
            break;
    }
}

// Run the state machine function for the current state.
void run_state(){
    switch(mainstate){
        case STATE_MANUAL:
            run_state_manual();
            break;
        case STATE_AUTO:
            run_state_auto();
            break;
        case STATE_SETTINGS:
            run_state_settings();
            break;
    }
}

// Set motor speed, then fix prescaler to remove motor whine.
void set_motor_speed(int speed){
    // Write speed to MOTPIN. This resets the prescaler.
    analogWrite(MOTPIN,speed);
    // Disable TC0
    TC0->COUNT8.CTRLA.bit.ENABLE = 0;
    while (TC0->COUNT8.SYNCBUSY.bit.ENABLE);
    // Reset prescaler to 1/16
    TC0->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV16;
    // Enable TC0
    TC0->COUNT8.CTRLA.bit.ENABLE = 1;
    while (TC0->COUNT8.SYNCBUSY.bit.ENABLE);
}

// STATE_MANUAL 
void run_state_manual(){
    // Handle up/down buttons to increase/decrease speed.
    switch(buttonstate){
        case BUTTON_UP:
            motorspeed += 10;
            if (motorspeed > MOT_MAX) motorspeed = MOT_MAX;
            break;
        case BUTTON_DOWN:
            motorspeed -= 10;
            if (motorspeed < 0) motorspeed = 0;
            break;
    }
    // Set the resulting speed.
    set_motor_speed(motorspeed);
}

// STATE_AUTO
void run_state_auto(){
    static float motIncrement = 0.0;
    
    // Calculate motor increment per cycle.
    motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

    // Handle up/down buttons to increase/decrease sensitivity.
    switch(buttonstate){
        case BUTTON_UP:
            sensitivity += 10;
            if (sensitivity > MAX_PLIMIT) sensitivity = MAX_PLIMIT;
            break;
        case BUTTON_DOWN:
            sensitivity -= 10;
            if (sensitivity < 0) sensitivity = 0;
            break;
    }

    // Invert the sensitivity value to get the pressure limit.
    // Higher sensitivity means a lower pressure delta to trigger an edge.
    pLimit = map(sensitivity,0,MAX_PLIMIT,MAX_PLIMIT,0);

    // Above pLimit? Stop the motor by setting a negative speed. Else, increase speed by the motor increment.
    if (pressure - avgPressure > pLimit) {
        motorspeed = -.5*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
    }
    else if (motorspeed < (float)maxSpeed) {
        motorspeed += motIncrement;
    }

    // Speed above motor minimum? Set the motor speed. Else, set it to 0.
    if (motorspeed > MOT_MIN) {
        set_motor_speed((int)motorspeed);
    } else {
        set_motor_speed(0);
    }

}

// STATE_SETTINGS
void run_state_settings(){
    // Handle left/right buttons to move between settings pages. Loop at the ends.
    switch(buttonstate){
        case BUTTON_LEFT:
            if (settingsstate == SET_BEGIN) settingsstate = SET_END;
            else settingsstate = (SettingsStates)(settingsstate-1);
            set_motor_speed(0);
            break;
        case BUTTON_RIGHT:
            if (settingsstate == SET_END) settingsstate = SET_BEGIN;
            else settingsstate = (SettingsStates)(settingsstate+1);
            set_motor_speed(0);
            break;
    }

    // Run the function to handle the current settings page.
    switch(settingsstate){
        case SET_SPEED:
            run_set_speed();
            break;
        case SET_AREF:
            run_set_aref();
            break;
    }
}

// Handle setting the max speed of the motor
void run_set_speed(){
    // Handle up/down buttons to increase/decrease max motor speed.
    switch(buttonstate){
        case BUTTON_UP:
            maxSpeed += 10;
            if (maxSpeed > MOT_MAX) maxSpeed = MOT_MAX;
            break;
        case BUTTON_DOWN:
            maxSpeed -= 10;
            if (maxSpeed < 0) maxSpeed = 0;
            break;
    }

    // Save max speed to setting struct
    settings.maxSpeed = maxSpeed;

    // Set the motor to the current max speed.
    set_motor_speed(maxSpeed);
}

// Handle setting the aref voltage / pressure sensor range/sensitivity.
// Also display raw pressure reading and target range for resting pressure.
void run_set_aref(){
    // Handle up/down buttons to increase/decrease aref setting.
    switch(buttonstate){
        case BUTTON_UP:
            aref += 1;
            if (aref > 4) aref = 4;
            break;
        case BUTTON_DOWN:
            aref -= 1;
            if (aref < 0) aref = 0;
            break;
    }

    // Save aref to setting struct
    settings.aref = aref;

    // Set the new aref value
    analogReference((eAnalogReference)arefs[aref]);
}

// Display
void draw_display(){
    //Blank the display, then run the appropriate code based on main state
    switch(mainstate){
        case STATE_MANUAL:
            draw_manual();
            break;
        case STATE_AUTO:
            draw_auto();
            break;
        case STATE_SETTINGS:
            draw_settings();
            break;
    }
    
}

void draw_manual(){
    if (laststate != mainstate){
        screen.fillSprite(TFT_BLACK);
        screen.setTextDatum(TL_DATUM);
        screen.drawString("^SETTINGS",0,0);
        screen.drawString("^AUTO",160,0);

        screen.setTextDatum(TR_DATUM);
        screen.drawString("MODE:",155,35);
        screen.setTextDatum(TL_DATUM);
        screen.drawString("MANUAL",165,35);

        screen.setTextDatum(TR_DATUM);
        screen.drawString("Pressure:",155,70);
        screen.drawString("Speed:",155,90);

        screen.pushSprite(0, 0);
    }

    sprite.setColorDepth(8);
    sprite.createSprite(80,40);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TL_DATUM);
    sprite.drawString(String(pressure),0,0);
    sprite.drawString(String((int)motorspeed),0,20);

    sprite.pushSprite(165,70);
    sprite.deleteSprite();
}

void draw_auto(){
    if (laststate != mainstate){
        screen.fillSprite(TFT_BLACK);
        screen.setTextDatum(TL_DATUM);
        screen.drawString("^SETTINGS",0,0);
        screen.drawString("^MANUAL",160,0);

        screen.setTextDatum(TR_DATUM);
        screen.drawString("MODE:",155,35);
        screen.setTextDatum(TL_DATUM);
        screen.drawString("AUTO",165,35);

        screen.setTextDatum(TR_DATUM);
        screen.drawString("Pressure:",155,70);
        screen.drawString("Speed:",155,90);
        screen.drawString("Sensitivity:",155,110);
        screen.drawString("% of pLimit:",155,130);

        screen.pushSprite(0, 0);
    }

    sprite.setColorDepth(8);
    sprite.createSprite(80,80);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TL_DATUM);
    sprite.drawString(String(pressure),0,0);
    sprite.drawString(String((int)motorspeed),0,20);
    sprite.drawString(String(sensitivity),0,40);
    float percent_plimit = constrain(((float)(pressure - avgPressure) / (float)pLimit) * 100.0,0.0,100.0);
    sprite.drawString(String(percent_plimit,1),0,60);

    sprite.pushSprite(165,70);
    sprite.deleteSprite();
}

void draw_settings(){
    if (laststate != mainstate){
        screen.fillSprite(TFT_BLACK);
        screen.setTextDatum(TL_DATUM);
        screen.drawString("^SAVE/EXIT",160,0);
        //screen.setTextDatum(TC_DATUM);
        //screen.drawString("Under Development!",160,35);

        screen.setTextDatum(TR_DATUM);
        screen.drawString("< SETTINGS:",155,35);
        //screen.setTextDatum(TL_DATUM);
        //screen.drawString("AUTO",165,35);

        screen.pushSprite(0, 0);
    }

    switch(settingsstate){
        case SET_SPEED:
            draw_set_speed();
            break;
        case SET_AREF:
            draw_set_aref();
            break;
    }
}

void draw_set_speed(){
    sprite.setColorDepth(8);
    sprite.createSprite(160,20);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TL_DATUM);
    sprite.drawString("Max Speed >",0,0);

    sprite.pushSprite(165,35);

    sprite.deleteSprite();

    sprite.setColorDepth(8);
    sprite.createSprite(155,60);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TR_DATUM);
    sprite.drawString("Max Speed:",155,0);

    sprite.pushSprite(0,90);

    sprite.deleteSprite();

    sprite.setColorDepth(8);
    sprite.createSprite(80,80);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TL_DATUM);
    sprite.drawString(" ^",0,0);
    sprite.drawString(String((int)maxSpeed),0,20);
    sprite.drawString(" v",0,40);

    sprite.pushSprite(165,70);

    sprite.deleteSprite();
}

void draw_set_aref(){
    sprite.setColorDepth(8);
    sprite.createSprite(160,20);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TL_DATUM);
    sprite.drawString("ARef >",0,0);

    sprite.pushSprite(165,35);

    sprite.deleteSprite();

    sprite.setColorDepth(8);
    sprite.createSprite(155,60);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TR_DATUM);
    sprite.drawString("ARef:",155,0);
    sprite.drawString("Pressure:",155,40);

    sprite.pushSprite(0,90);

    sprite.deleteSprite();

    sprite.setColorDepth(8);
    sprite.createSprite(80,80);
    sprite.fillSprite(TFT_BLACK);

    sprite.setTextDatum(TL_DATUM);
    sprite.drawString("^",0,0);
    sprite.drawString(String((int)aref),0,20);
    sprite.drawString("v",0,40);
    sprite.drawString(String(pressure),0,60);

    sprite.pushSprite(165,70);

    sprite.deleteSprite();
}


// Log values to the serial console.
void run_logging(){
    SerialUSB.print(mainstate);
    SerialUSB.print(",");
    SerialUSB.print(buttonstate);
    SerialUSB.print(",");
    SerialUSB.print(settingsstate);
    SerialUSB.print(",");
    SerialUSB.print(pressure);
    SerialUSB.print(",");
    SerialUSB.print(avgPressure);
    SerialUSB.print(",");
    SerialUSB.print(sensitivity);
    SerialUSB.print(",");
    SerialUSB.print(pLimit);
    SerialUSB.print(",");
    SerialUSB.print(motorspeed);
    SerialUSB.print(",");
    SerialUSB.print(maxSpeed);
    SerialUSB.print(",");
    SerialUSB.print(aref);
    SerialUSB.println("");
}

//=======Main Loop====================================

void mainloop() {
    read_pressure();
    read_buttons();
    set_state();
    run_state();
    draw_display();
    run_logging();
    //Handling this here so that laststate will be -1 through the entire first loop.
    //Probably a better way to do this, but... ¯\_(ツ)_/¯
    laststate = mainstate; 
    delay(1); //Prevent repetitive runs if we finish faster than 1ms
}

void loop(){
    // Run the mainloop every PERIOD ms.
    if (millis() % PERIOD == 0) mainloop();
}
