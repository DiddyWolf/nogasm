// Nogasm for Wio Terminal
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 */
//=======Libraries===============================
#include "RunningAverage.h"

//=======Hardware Setup===============================
//Motor PWM out
#define MOTPIN D6

//Pressure Sensor Analog In
#define BUTTPIN A8

//Max pressure
#define MAXPRES 4030

//Motor min and max speeds
#define MOT_MAX 255 // Motor PWM maximum
#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

//=======States=====================================
//Define all enums for state machines
enum MainStates{STATE_MANUAL,STATE_AUTO,STATE_SETTINGS};
MainStates mainstate = STATE_MANUAL;

enum ButtonStates{BUTTON_NONE,BUTTON_A,BUTTON_B,BUTTON_C,BUTTON_UP,BUTTON_DOWN,BUTTON_LEFT,BUTTON_RIGHT,BUTTON_PRESS};
ButtonStates buttonstate = BUTTON_NONE;

enum SettingsStates{SET_SPEED, SET_SENSITIVITY};
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
#define BUTTON_HOLD_TICKS 20

//=======Globals====================================
int pressure = 0;
int avgPressure = 0; //Running 25 second average pressure
float motorspeed = 0;
int sensitivity = 0; //orgasm detection sensitivity, persists through different states
int rampTimeS = 30; //Ramp-up time, in seconds
#define MAX_PLIMIT 600
int pLimit = MAX_PLIMIT; //Limit in change of pressure before the vibrator turns off
int maxSpeed = 255; //maximum speed the motor will ramp up to in automatic mode

//=======Setup=======================================

void setup() {
    SerialUSB.begin(115200);

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

    //Set analog resolution for 0-4094 range
    analogReadResolution(12);
    //Average 32 samples on each read
    ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_32 | ADC_AVGCTRL_ADJRES(4);
}

//=======Functions====================================

// Read pressure and add it to the running average at a slower speed.
void read_pressure() {
    static uint8_t sampleTick = 0;

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

}

// Read all button presses. Handle repeat presses for direction keys.
void read_buttons(){
    static int laststate[9] = {0,1}; // Set the A key last state to start at 1, to avoid undesired button press on startup.
    static int currstate[9];
    static int holdcount = 0;
    static int buttons[] = {NONE,WIO_KEY_A,WIO_KEY_B,WIO_KEY_C,WIO_5S_UP,WIO_5S_DOWN,WIO_5S_LEFT,WIO_5S_RIGHT,WIO_5S_PRESS};

    // Skipping 0 and padding the buttons array with NONE so that the 'resting' buttonstate can be 0.
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
        else if ((i == UP or i == DOWN or i == LEFT or i == RIGHT) and laststate[i] == LOW and currstate[i] == LOW){
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
                    break;
            }
            break;
        case STATE_AUTO:
            switch(buttonstate){
                case BUTTON_A:
                    mainstate = STATE_MANUAL;
                    motorspeed = 0;
                    break;
                case BUTTON_C:
                    mainstate = STATE_SETTINGS;
                    motorspeed = 0;
                    break;
            }
            break;
        case STATE_SETTINGS:
            switch(buttonstate){
                case BUTTON_A:
                    mainstate = STATE_MANUAL;
                    motorspeed = 0;
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
            if (motorspeed > 250) motorspeed = 250;
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

}

// Log values to the serial console.
void run_logging(){
    SerialUSB.print(mainstate);
    SerialUSB.print(",");
    SerialUSB.print(buttonstate);
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
    SerialUSB.println("");
}

//=======Main Loop====================================

void mainloop() {
    read_pressure();
    read_buttons();
    set_state();
    run_state();
    //draw_display();
    run_logging(); 
    delay(1); //Prevent repetitive runs if we finish faster than 1ms
}

void loop(){
    // Run the mainloop every PERIOD ms.
    if (millis() % PERIOD == 0) mainloop();
}