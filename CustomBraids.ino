//************LIBRARIES USED**************
// include the ResponsiveAnalogRead library for analog smoothing
#include <ResponsiveAnalogRead.h>
#include <Bounce.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include <Wire.h>

#include <Audio.h>

#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <Time.h>
#include <TimeAlarms.h>
#include <LiquidCrystalFast.h>
#include <TeensyThreads.h>
#include "kinetis.h"

// initialize the library with the numbers of the interface pins
// LiquidCrystal lcd(RS, RW, Enable, D4, D5, D6, D7)
LiquidCrystalFast lcd(12, 39, 11, 5, 4, 3, 2);

//********************* BRAIDS ***********************
#include "macro_oscillator.h"
using namespace braids;

const int A_PINS = 18; // number of Analog PINS
const int D_PINS = 9; // number of Digital PINS

// define the pins you want to use and the CC ID numbers on which to send them..
const int ANALOG_PINS[A_PINS] = {A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13,A14,A15,A16,A17};
// initialize the ReponsiveAnalogRead objects
ResponsiveAnalogRead analog[]{
  {ANALOG_PINS[0],true},
  {ANALOG_PINS[1],true},
  {ANALOG_PINS[2],true},
  {ANALOG_PINS[3],true},
  {ANALOG_PINS[4],true},
  {ANALOG_PINS[5],true},
  {ANALOG_PINS[6],true},
  {ANALOG_PINS[7],true},
  {ANALOG_PINS[8],true},
  {ANALOG_PINS[9],true},
  {ANALOG_PINS[10],true},
  {ANALOG_PINS[11],true},
  {ANALOG_PINS[12],true},
  {ANALOG_PINS[13],true},
  {ANALOG_PINS[14],true},
  {ANALOG_PINS[15],true},
  {ANALOG_PINS[16],true},
  {ANALOG_PINS[17],true}
};

const int DIGITAL_PINS[D_PINS] = {25,26,24,29,30,27,28,37,10};
const int BOUNCE_TIME = 5; // 5 ms is usually sufficient
const boolean toggled = true;

const char notes[12][4]= {"C ","C#","D ","D#","E ","F ","F#","G ","G#","A ","A#","B "};

//******VARIABLES***********
// a data array and a lagged copy to tell when MIDI changes are required
volatile byte dataAnal[A_PINS];
volatile byte dataAnalLag[A_PINS]; // when lag and new are not the same then update MIDI CC value

// initialize the bounce objects
Bounce digital[] =   {
  Bounce(DIGITAL_PINS[0],BOUNCE_TIME),
  Bounce(DIGITAL_PINS[1], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[2], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[3], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[4], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[5], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[6], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[7], BOUNCE_TIME),
  Bounce(DIGITAL_PINS[8], BOUNCE_TIME)
};

Encoder knobRightF(6, 7);
Encoder knobRightS(8, 9);
volatile long positionRightF = -999;
volatile long positionRightS = -999;
volatile long newRightS;
volatile long newRightF;

volatile  int analval = 0;
volatile  int sequenceLen = 8;

const char SHAPES[60][50] ={
  "CSAW",
  "MORPH",
  "SAW SQUARE",
  "SQUARE SYNC",
  "SINE TRIANGLE",
  "BUZZ",
  "TRIPLE SAW",
  "TRIPLE SQUARE",
  "TRIPLE RING MODE",
  "SAW SWARM",
  "SAW COMB",
  "TOY",
  "DIGITAL FILTER LP",
  "DIGITAL FILTER PK",
  "DIGITAL FILTER BP",
  "DIGITAL FILTER HP",
  "VOSIM",
  "VOWEL",
  "VOWEL FOF",
  "FM",
  "FEEDBACK FM",
  "CHAOTIC FEEDBACK FM",
  "STRUCK BELL",
  "STRUCK DRUM",
  "PLUCKED",
  "BOWED",
  "BLOWN",
  "FLUTED",
  "WAVETABLES",
  "WAVE MAP",
  "WAVE LINE",
  "WAVE PARAPHONIQUE",
  "FILTERED NOISE",
  "TWIN PEAKS NOISE",
  "CLOCKED NOISE",
  "GRANULAR CLOUD",
  "PARTICLE NOISE",
  "DIGITAL MODULATION",
  "SNARE",
  #if NUM_BANKS >= 1
      "SAM1",
  #endif
  #if NUM_BANKS >= 2
      "SAM2",
  #endif
  #if NUM_BANKS >= 3
      "SAM3",
  #endif
  #if NUM_BANKS >= 4
      "SAM4",
  #endif
  "HARMONICS",

  "QUESTION MARK",
};

volatile int SEQUENCER_NOTES[8] = {60,60,60,60,60,60,60,60};
volatile int lastnote = 0;
elapsedMillis msec = 0;
elapsedMillis msec1 = 0;

int th1;
int th2;
int th3;

MacroOscillator osc;
IntervalTimer myTimer;

const uint32_t kSampleRate = 96000;
const uint16_t kAudioBlockSize = 28;

uint8_t sync_buffer[kAudioBlockSize];
int16_t bufferA[kAudioBlockSize];
int16_t bufferB[kAudioBlockSize];
uint8_t buffer_sel;

volatile uint8_t buffer_index;
volatile uint8_t wait;
// Timer interruption to put the following sample
void putSample(void){
    //threads.suspend(th1);
    uint16_t val;

    if(buffer_sel)
        val = ((uint16_t)(bufferB[buffer_index]+0x7FFF))>>4;
    else
        val = ((uint16_t)(bufferA[buffer_index]+0x7FFF))>>4;

    buffer_index++;
    analogWrite(A21, val);
    //analogWrite(A22, val);
    if(buffer_index>=kAudioBlockSize) {
        wait = 0;
        buffer_index = 0;
        buffer_sel = ~buffer_sel;
    }
    //threads.restart(th1);
}

// Globals that define the parameters of the oscillator
static volatile int16_t pitch,pre_pitch;
static volatile int16_t timbre;
static volatile int16_t color;
static volatile int16_t shapebraids;

// Handles note on events
// void OnNoteOn(byte channel, byte note, byte velocity){
//     // If the velocity is larger than zero, means that is turning on
//     lcd.setCursor(5,1);
//     if(velocity){
//         // Sets the 7 bit midi value as pitch
//         pitch = note << 7;
//         lcd.print(pitch);
//         // triggers a note
//         osc.Strike();
//     }
// }

void init_braids(){
  // Initalizes the buffers to zero
    memset(bufferA, 0, kAudioBlockSize);
    memset(bufferB, 0, kAudioBlockSize);

    // Global used to trigger the next buffer to render
    wait = 0;

    // Initializes the objects
    osc.Init();
    osc.set_shape(MACRO_OSC_SHAPE_GRANULAR_CLOUD);
    osc.set_parameters(0, 0);
    myTimer.begin(putSample,1e6/96000.0);

    //usbMIDI.setHandleNoteOn(OnNoteOn);
    attachInterrupt(DIGITAL_PINS[0], sendNotetoSequencer, CHANGE);

    pitch = 32 << 7;
}

void run_braids(){
  memset(sync_buffer, 0, sizeof(sync_buffer));
  // Set the pin to 1 to mark the begining of the render cycle
  //digitalWriteFast(13,HIGH);
  // If the pitch changes update it
  if(pre_pitch!=pitch){
      osc.set_pitch(pitch);
      pre_pitch = pitch;
  }
  // Get the timbre and color parameters from the ui and set them
  osc.set_parameters(timbre,color);

  // Trims the shape to the valid values
  shapebraids = shapebraids >= MACRO_OSC_SHAPE_QUESTION_MARK ? MACRO_OSC_SHAPE_QUESTION_MARK : shapebraids<0 ? 0 : shapebraids;

  // Sets the shape
  MacroOscillatorShape osc_shape = static_cast<MacroOscillatorShape>(shapebraids);//
  osc.set_shape(osc_shape);

  if(buffer_sel){
      osc.Render(sync_buffer, bufferA, kAudioBlockSize);
  }
  else{
      osc.Render(sync_buffer, bufferB, kAudioBlockSize);
  }
  // Reads the midi data
  //usbMIDI.read();

  // Set the pin low to mark the end of rendering and processing
  //digitalWriteFast(13,LOW);
  // Waits until the buffer is ready to render again
  wait = 1;
  while(wait);
}

///////////////////////////////////////////////////////////

//************SETUP**************
void setup() {
  // Open serial communications and wait for port to open:
  //Serial.begin(9600);

  // loop to configure input pins and internal pullup resisters for digital section
  for (int i=0;i<D_PINS;i++){
    pinMode(DIGITAL_PINS[i], INPUT_PULLUP);
  }

  // Configure the ADCs
  analogReadResolution(7);
  analogReadAveraging(4);
  analogReference(EXTERNAL);

  // Configure the DACs
  analogWriteResolution(12);

  // set up the LCD's number of rows and columns:
  lcd.begin(20, 2);

  lcd.print("*CustomBraids UP !!!!*");
  delay(1000);

  th1 = threads.addThread(midiControllerSliders);
  th2 = threads.addThread(controls_threads);
  th3 = threads.addThread(control_shape);

  init_braids();

}

//************LOOP**************
void loop() {
  run_braids();
  if (msec1 >= 100) {
    msec1 = 0;
  }
}

void controls_threads(){
  while(1){
  //// for teensy 3.5
  for (int i=0;i<2;i++){
    // update the ResponsiveAnalogRead object every loop
    analog[i].update();
    if (analog[i].hasChanged()) {
      analval = analog[i].getValue();
      if(i == 0){
        color = analval << 7;
      }
      if(i == 1){
        timbre = analval << 7;
      }
    }
  }
  /////////////////////////////////
  threads.delay(20);
  }
}

void control_shape(){
  while(1){
  newRightS = knobRightS.read()/2;
  if (newRightS != positionRightS){
    if (newRightS < 0){
      newRightS = 55;
      knobRightS.write(newRightS*2);
    }
    if (newRightS > 55){
      newRightS = 0;
      knobRightS.write(0);
    }
    positionRightS = newRightS;
    shapebraids = positionRightS;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(SHAPES[shapebraids]);
  }
  newRightF = knobRightF.read()/2;
  if (newRightF != positionRightF){
    if (newRightF <= 0){
      newRightF = 8;
      knobRightF.write(newRightF*2);
    }
    if (newRightF > 8){
      newRightF = 1;
      knobRightF.write(newRightF*2);
    }
    positionRightF = newRightF;
    lcd.setCursor(19, 0);
    lcd.print(positionRightF);
  }
  if(digital[7].update()){
    if (digital[7].fallingEdge()) {
      sequenceLen = positionRightF;
      lastnote = 0;
    }
  }
  threads.delay(20);
  }
}

void sendNotetoSequencer(){
  digital[0].update();
  if (digital[0].fallingEdge()) {
    switch(lastnote){
      case 0:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 0, 1, false);
      break;

      case 1:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 2, 1, false);
      break;

      case 2:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 4, 1, false);
      break;

      case 3:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 6, 1, false);
      break;

      case 4:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 8, 1, false);
      break;

      case 5:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 10, 1, false);
      break;

      case 6:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 12, 1, false);
      break;
      
      case 7:
      printLCDmessCHAR(NoteToName(SEQUENCER_NOTES[lastnote]), 14, 1, false);
      break;
    }
    // Sets the 7 bit midi value as pitch
    pitch = SEQUENCER_NOTES[lastnote] << 7;
    // triggers a note
    osc.Strike();
    lastnote = lastnote + 1;
    if(lastnote == sequenceLen){
      lastnote = 0;
    }
  }
}

//************MIDI CONTROLLER SECTION**************
void midiControllerSliders(){
 while(1){
 for (int i=2;i<10;i++){
   // update the ResponsiveAnalogRead object every loop
   analog[i].update();
   if (analog[i].hasChanged()) {
     dataAnal[i] = analog[i].getValue();
     if (dataAnal[i] > dataAnalLag[i]+2 || dataAnal[i] < dataAnalLag[i]-2) {
       if(dataAnal[i] >= 115){
         dataAnal[i] = 127;
       }
       if(dataAnal[i] <= 10){
         dataAnal[i] = 0;
       }
       if (dataAnal[i] != dataAnalLag[i]){
         dataAnalLag[i] = dataAnal[i];
         if(i<10){
           SEQUENCER_NOTES[i-2] = dataAnal[i];
         }
       }
     }
   }
 }
 threads.delay(20);
 }
}

const char* NoteToName(int n){
  if (n>=0 and n<=119){
    
    return notes[n/12];
  }
  return "--";
}

//***************SCREEN SECTION****************
void printLCDmessINT(int mess, int col, int row){
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(col, row);
  // Print a message to the LCD.
  lcd.print(mess);
}

void printLCDmessCHAR(const char* mess, int col, int row, bool clr){
  if(clr){
    lcd.clear();
  }
  lcd.setCursor(col, row);
  // Print a message to the LCD.
  lcd.print(mess);
}
