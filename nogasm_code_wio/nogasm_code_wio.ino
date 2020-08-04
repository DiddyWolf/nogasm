// Nogasm for Wio Terminal
/* Drives a vibrator and uses changes in pressure of an inflatable buttplug
 * to estimate a user's closeness to orgasm, and turn off the vibrator
 * before that point.
 */
//=======Libraries===============================
#include "RunningAverage.h"

//=======Hardware Setup===============================
//Motor
#define MOTPIN D6

//Pressure Sensor Analog In
#define BUTTPIN A8

//Max pressure
#define MAXPRES 4030

//=======States=====================================
//Define all enums for state machines
enum MainStates{MANUAL,AUTO,SETTINGS};
MainStates mainstate = MANUAL;

enum ButtonStates{NONE,A,B,C,UP,DOWN,LEFT,RIGHT,PRESS};
ButtonStates buttonstate = NONE;

//=======Timing=====================================
#define FREQUENCY 60 //Update frequency in Hz

//Update/render period
#define PERIOD (1000/FREQUENCY)

//Running pressure average array length and update frequency
#define RA_HIST_SECONDS 25
#define RA_FREQUENCY 6
#define RA_TICK_PERIOD (FREQUENCY / RA_FREQUENCY)
RunningAverage raPressure(RA_FREQUENCY*RA_HIST_SECONDS);

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
#define MOT_MAX 255 // Motor PWM maximum
#define MOT_MIN 20  // Motor PWM minimum.  It needs a little more than this to start.

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

    analogReadResolution(12);
    //Average 32 samples
    ADC0->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_32 | ADC_AVGCTRL_ADJRES(4);
}

//=======Functions====================================

void read_pressure() {
    static uint8_t sampleTick = 0;
    pressure = analogRead(BUTTPIN);
    //Alarm if over max pressure
    if (pressure > MAXPRES) {
      analogWrite(WIO_BUZZER, 200);
    } else {
      analogWrite(WIO_BUZZER, 0);
    }

    sampleTick++;
    if (sampleTick % RA_TICK_PERIOD == 0){
        raPressure.addValue(pressure);
        avgPressure = raPressure.getAverage();
    }

}

void read_buttons(){
    static int laststate[9] = {0,1}; // Set the A key last state to start at 1, to avoid undesired button press on startup.
    static int currstate[9];
    static int holdcount = 0;
    static int buttons[] = {NONE,WIO_KEY_A,WIO_KEY_B,WIO_KEY_C,WIO_5S_UP,WIO_5S_DOWN,WIO_5S_LEFT,WIO_5S_RIGHT,WIO_5S_PRESS};

    // Skipping 0 and padding the buttons array with NONE so that the 'resting' buttonstate is 0.
    for (int i = 1; i<9; i++){
        currstate[i] = digitalRead(buttons[i]);
    }

    for (int i = 1; i<9; i++){
        if (laststate[i] == LOW and currstate[i] == HIGH){
            if (holdcount < BUTTON_HOLD_TICKS) buttonstate = (ButtonStates)i;
            else buttonstate = NONE;
            holdcount = 0;
            break;
        }
        else if ((i == UP or i == DOWN or i == LEFT or i == RIGHT) and laststate[i] == LOW and currstate[i] == LOW){
            holdcount++;
            if (holdcount % BUTTON_HOLD_TICKS == 0) buttonstate = (ButtonStates)i;
            else buttonstate = NONE;
            break;
        }
        else {
            buttonstate = NONE;
        }
    }

    for (int i = 1; i<9; i++){
        laststate[i] = currstate[i];
    }
}

void set_state(){
    switch(mainstate){
        case MANUAL:
            switch(buttonstate){
                case A:
                    mainstate = AUTO;
                    motorspeed = 0;
                    break;
                case C:
                    mainstate = SETTINGS;
                    break;
            }
            break;
        case AUTO:
            switch(buttonstate){
                case A:
                    mainstate = MANUAL;
                    motorspeed = 0;
                    break;
                case C:
                    mainstate = SETTINGS;
                    break;
            }
            break;
        case SETTINGS:
            switch(buttonstate){
                case A:
                    mainstate = MANUAL;
                    motorspeed = 0;
                    break;
            }
            break;
    }
}

void run_state(){
    switch(mainstate){
        case MANUAL:
            run_state_manual();
            break;
        case AUTO:
            run_state_auto();
            break;
        case SETTINGS:
            run_state_settings();
            break;
    }
}

// Set motor speed, then fix prescaler to remove motor whine.
void set_motor_speed(int speed){
    analogWrite(MOTPIN,speed);
    // Disable TCx
    TC0->COUNT8.CTRLA.bit.ENABLE = 0;
    while (TC0->COUNT8.SYNCBUSY.bit.ENABLE);
    // Set Timer counter Mode to 8 bits, normal PWM, prescaler 1/256
    TC0->COUNT8.CTRLA.reg = TC_CTRLA_MODE_COUNT8 | TC_CTRLA_PRESCALER_DIV16;
    // Enable TCx
    TC0->COUNT8.CTRLA.bit.ENABLE = 1;
    while (TC0->COUNT8.SYNCBUSY.bit.ENABLE);
}

void run_state_manual(){
    switch(buttonstate){
        case UP:
            motorspeed += 10;
            if (motorspeed > 250) motorspeed = 250;
            break;
        case DOWN:
            motorspeed -= 10;
            if (motorspeed < 0) motorspeed = 0;
            break;
    }
    set_motor_speed(motorspeed);
}

void run_state_auto(){
    static float motIncrement = 0.0;
    motIncrement = ((float)maxSpeed / ((float)FREQUENCY * (float)rampTimeS));

    switch(buttonstate){
        case UP:
            sensitivity += 10;
            if (sensitivity > MAX_PLIMIT) sensitivity = MAX_PLIMIT;
            break;
        case DOWN:
            sensitivity -= 10;
            if (sensitivity < 0) sensitivity = 0;
            break;
    }

    // Invert the sensitivity value to get the pressure limit.
    // Higher sensitivity means a lower pressure delta to trigger an edge.
    pLimit = map(sensitivity,0,MAX_PLIMIT,MAX_PLIMIT,0);

    if (pressure - avgPressure > pLimit) {
        motorspeed = -.5*(float)rampTimeS*((float)FREQUENCY*motIncrement);//Stay off for a while (half the ramp up time)
    }
    else if (motorspeed < (float)maxSpeed) {
        motorspeed += motIncrement;
    }

    if (motorspeed > MOT_MIN) {
        set_motor_speed((int)motorspeed);
    } else {
        set_motor_speed(0);
    }

}

void run_state_settings(){

}

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
    if (millis() % PERIOD == 0) mainloop();
}