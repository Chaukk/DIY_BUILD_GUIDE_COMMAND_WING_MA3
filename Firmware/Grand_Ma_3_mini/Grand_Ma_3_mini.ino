//Credits: Nerd Musician
//https://www.musiconerd.com

#include <MIDIUSB.h>
#include <ResponsiveAnalogRead.h> 


//************************************************
//ENCODERS
  const int enc_num = 5; // Total number of encoders
  const int enc_pins[] = {28,30,0,1,3,4,6,7,10,9}; // Each two pins of the encoder in sequence
  int custom_midi[] = {11,12,13,14,15}; // Same length of enc_num with cc midi number
  int prev_state[enc_num];
  long prev_time[] = {10,10,10,10,10};

  int currentStateCLK;
  long previous_time; 
  byte encdir; 
//************************************************

//************************************************
// BUTTONS
  const int N_BUTTONS = 6; //Total numbers of buttons
  const int BUTTON_ARDUINO_PIN[N_BUTTONS] = {32,2,5,8,11,13}; //Pins of each button connected straight to the Arduino

  int buttonCState[N_BUTTONS] = {};        // stores the button current value
  int buttonPState[N_BUTTONS] = {};        // stores the button previous value

  //#define pin13 1 //* uncomment if you are using pin 13 (pin with led), or comment the line if not using
  byte pin13index = 12; //* puti the index of the pin 13 of the buttonPin[] array if you are using, if not, comment
  // debounce
  unsigned long lastDebounceTime[N_BUTTONS] = {0};  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;    //* the debounce time; increase if the output flickers
//************************************************

//************************************************
// POTENTIOMETERS
  const int N_POTS = 12; //* total numbers of pots (slide & rotary)
  const int POT_ARDUINO_PIN[N_POTS] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A12}; //* pins of each pot connected straight to the Arduino
 
  const int midi_channels[N_POTS] = {1,2,3,4,5,6,7,8,9,10,11,12};

  int read_value = 0;
  int total_value = 0;

  int potCState[N_POTS] = {0}; // Current state of the pot
  int potPState[N_POTS] = {0}; // Previous state of the pot
  int potVar= 0; // Difference between the current and previous state of the pot

  int midiCState[N_POTS] = {0}; // Current state of the midi value
  int midiPState[N_POTS] = {0}; // Previous state of the midi value

  const int TIMEOUT = 600; //* Amount of time the potentiometer will be read after it exceeds the varThreshold
  const int varThreshold = 2; //* Threshold for the potentiometer signal variation
  boolean potMoving = true; // If the potentiometer is moving
  unsigned long PTime[N_POTS] = {0}; // Previously stored time
  unsigned long timer[N_POTS] = {0}; // Stores the time that has elapsed since the timer was reset
//************************************************

//************************************************
// MIDI
  byte midiCh = 1; //* MIDI channel to be used
  byte note = 1; //* Lowest note to be used
  byte cc = 1; //* Lowest MIDI CC to be used 
//************************************************

void setup() {  
  Serial.begin (9600);
  int pin = 0; 
  for (pin = 0; pin < enc_num*2; pin++){
    pinMode (enc_pins[pin], INPUT);
    digitalWrite(enc_pins[pin], HIGH);
  }
  for (pin = 0; pin < enc_num*2; pin = pin + 2){
    prev_state[pin] = digitalRead(pin);
  }
  for (int i = 0; i < N_BUTTONS; i++) {
  pinMode(BUTTON_ARDUINO_PIN[i], INPUT_PULLUP);
  }
} 

void loop(){
  encoders();
  buttons();
  potentiometers();
}



void potentiometers() {

  for (int i = 0; i < N_POTS; i++) { // Loops through all the potentiometers
    
    for (int p = 0; p < 11; p++){
      read_value = analogRead(POT_ARDUINO_PIN[i]);
      total_value = total_value + read_value;
    }
    potCState[i] = total_value/10;
    total_value = 0;
    // Serial.println(potCState[10]);
    // potCState[i] = analogRead(POT_ARDUINO_PIN[i]); // reads the pins from arduino
    potCState[i] = clipValue(potCState[i], 5, 1020);

    if (i >= 0){
      midiCState[i] = map(potCState[i], 5, 1020, 0, 16383); // Maps the reading of the potCState to a value usable in midi
      midiCState[i] = clipValue(midiCState[i], 0, 16383);
    }else{
      midiCState[i] = map(potCState[i], 5, 1020, 0, 127);
      midiCState[i] = clipValue(midiCState[i], 0, 127);
    }
      
    potVar = abs(potCState[i] - potPState[i]); // Calculates the absolute value between the difference between the current and previous state of the pot

    if (potVar > varThreshold) { // Opens the gate if the potentiometer variation is greater than the threshold
      PTime[i] = millis(); // Stores the previous time
      // Serial.println(potVar);
      // Serial.println(potCState[i]);
    }

    timer[i] = millis() - PTime[i]; // Resets the timer 11000 - 11000 = 0ms

    
    // Serial.println(timer[i]);
    

    if (timer[i] < TIMEOUT) { // If the timer is less than the maximum allowed time it means that the potentiometer is still moving
      potMoving = true;
      
    }
    else {
      potMoving = false;
    }

    if (potMoving == true) { // If the potentiometer is still moving, send the change control
      if (midiPState[i] != midiCState[i]) {


        // Sends the MIDI CC accordingly to the chosen board
        
        if (i >= 0){
          usbMIDI.sendPitchBend(midiCState[i] - 8192, midi_channels[i]);
        }else{
          usbMIDI.sendControlChange(1, midiCState[i], midi_channels[i]);
        }
        

        potPState[i] = potCState[i]; // Stores the current reading of the potentiometer to compare with the next
        midiPState[i] = midiCState[i];
      }
    }
  }
}



void encoders() {  
  int pin;

  for (pin = 0; pin < enc_num*2; pin = pin + 2){
        // Read the current state of inputCLK
    currentStateCLK = digitalRead(enc_pins[pin]);

    // If the previous and the current state of the inputCLK are different then a pulse has occured
    if (currentStateCLK != prev_state[pin]){ 

      // If the inputDT state is different than the inputCLK state then 
      // the encoder is rotating counterclockwise
      if (digitalRead(enc_pins[pin + 1]) != currentStateCLK) { 
        midi_send(custom_midi[pin/2], 63);

      } else {
        // Encoder is rotating clockwise
        midi_send(custom_midi[pin/2], 65);

      }
      prev_state[pin] = currentStateCLK;
    }
  }

} 
void buttons() {

  for (int i = 0; i < N_BUTTONS; i++) {

    buttonCState[i] = digitalRead(BUTTON_ARDUINO_PIN[i]);  // read pins from arduino
      if ((millis() - lastDebounceTime[i]) > debounceDelay) {

        if (buttonPState[i] != buttonCState[i]) {
          lastDebounceTime[i] = millis();

          if (buttonCState[i] == LOW) {

      
            
            usbMIDI.sendNoteOn(note + i, 1, midiCh); // note, velocity, channel

          }
          else {



            usbMIDI.sendNoteOn(note + i, 0, midiCh); // note, velocity, channel

          }
          buttonPState[i] = buttonCState[i];
      }
    }
  }
}
void midi_send(int encoder, int encdir){
  unsigned long allowed_delay = 30;
  unsigned long current_time = millis();
  if ((current_time - prev_time[encoder]) >= allowed_delay){
    usbMIDI.sendControlChange(encoder, encdir, 1);
    prev_time[encoder] = current_time;
  }
}
int clipValue(int in, int minVal, int maxVal) {

  int out;

  minVal++;

  if (in > maxVal) {
    out = maxVal;
  }
  else if (in < minVal) {
    out = minVal -1;
  }
  else {
    out = in;
  }

  return out;
}




  
    
