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

enum ButtonStates{A,B,C,UP,DOWN,LEFT,RIGHT,PRESS,NONE};
ButtonStates buttonstate = NONE;

//=======Globals====================================
int pressure = 0;
int motorspeed = 0;

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
    pressure = analogRead(BUTTPIN);
    //Alarm if over max pressure
    if (pressure > MAXPRES) {
      analogWrite(WIO_BUZZER, 200);
    } else {
      analogWrite(WIO_BUZZER, 0);
    }

}

void read_buttons(){
    static int laststate[8];
    static int currstate[8];
    static int buttons[] = {WIO_KEY_A,WIO_KEY_B,WIO_KEY_C,WIO_5S_UP,WIO_5S_DOWN,WIO_5S_LEFT,WIO_5S_RIGHT,WIO_5S_PRESS};

    for (int i = 0; i<8; i++){
        currstate[i] = digitalRead(buttons[i]);
    }

    if (laststate[A] == LOW and currstate[A] == HIGH) {
        buttonstate = A;
    }
    else if (laststate[B] == LOW and currstate[B] == HIGH) {
        buttonstate = B;
    }
    else if (laststate[C] == LOW and currstate[C] == HIGH) {
        buttonstate = C;
    }
    else if (laststate[UP] == LOW and currstate[UP] == HIGH) {
        buttonstate = UP;
    }
    else if (laststate[DOWN] == LOW and currstate[DOWN] == HIGH) {
        buttonstate = DOWN;
    }
    else if (laststate[LEFT] == LOW and currstate[LEFT] == HIGH) {
        buttonstate = LEFT;
    }
    else if (laststate[RIGHT] == LOW and currstate[RIGHT] == HIGH) {
        buttonstate = RIGHT;
    }
    else if (laststate[PRESS] == LOW and currstate[PRESS] == HIGH) {
        buttonstate = PRESS;
    }
    else {
        buttonstate = NONE;
    }

    for (int i = 0; i<8; i++){
        laststate[i] = currstate[i];
    }
}

void set_state(){
    switch(mainstate){
        case MANUAL:
            switch(buttonstate){
                case A:
                    mainstate = AUTO;
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
    
}

void run_state_settings(){

}

void run_logging(){
    SerialUSB.print(mainstate);
    SerialUSB.print(",");
    SerialUSB.print(buttonstate);
    SerialUSB.print(",");
    SerialUSB.print(motorspeed);
    SerialUSB.print(",");
    SerialUSB.println(pressure);
}

//=======Main Loop====================================

void mainloop() {
    read_pressure();
    read_buttons();
    set_state();
    run_state();
    //draw_display();
    run_logging(); 
}

void loop(){
    if (millis() % 10 == 0) mainloop();
}