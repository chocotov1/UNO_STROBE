#include <Arduino.h>
#include <TM1637Display.h>

byte pwm_output_pin = 9;

byte clk_pin       = 3;  // PCINT20
byte direction_pin = 4;  // PCINT21
volatile bool rotary_update    = false;
volatile bool rotary_direction = false;
volatile bool rotary_cw        = false;
volatile byte rotary_inputs    = 0;
uint32_t last_rotary_input;

// clock speed with extra zeroes used for calculating prescaler and flashing frequency
uint32_t f_cpu_00      = F_CPU * 100;
// precalculated values
uint32_t f_cpu_00_8    = f_cpu_00 / 8;
uint32_t f_cpu_00_64   = f_cpu_00 / 64;
uint32_t f_cpu_00_256  = f_cpu_00 / 256;
uint32_t f_cpu_00_1024 = f_cpu_00 / 1024;

uint32_t prescaled_clock = F_CPU;

volatile bool update_hz_display;
bool rpm_changed;
byte timer2_ovf_counter;

uint16_t rpm = 600;
uint16_t hz  = rpm * 100ul / 60;

//uint16_t up_counts = 0;

byte TM1637_CLK = 5;
TM1637Display display1(TM1637_CLK, 6);
TM1637Display display2(TM1637_CLK, 7);

void setup(){
   pinMode(clk_pin, INPUT_PULLUP);
   pinMode(direction_pin, INPUT_PULLUP);

   display1.setBrightness(1); // doesn't show anything without setting brightness first!!
   display2.setBrightness(1); // doesn't show anything without setting brightness first!!
   Serial.begin(9600);
   Serial.println("UNO_STROBE online");

   // rotary encoder:
   // PIN change interrupt
   //PCICR = 1<<PCIE0; // PORTB: digital pin 8 to 13
   //PCICR = 1<<PCIE1; // PORTC: analog input pins
   PCICR = 1<<PCIE2; // PORTD: digital pins 0 to 7

   //10.2.6 PCMSK2 – Pin Change Mask Register 2
   //  • Bit 7..0 – PCINT23..16: Pin Change Enable Mask 23..16
   PCMSK2 = 1<<clk_pin; // alternatief 1<<PCINT20   


   pinMode(pwm_output_pin, OUTPUT);

   // setup PWM with timer1 on OC1A, pin 9 (alternatively OC1B, pin 10)
   
   //13.11 Register Description
   //13.11.1 TCCR1A – Timer/Counter1 Control Register A

   //Table 13-2. Compare Output Mode, Fast PWM(1)
   TCCR1A = 1<<COM1A1;                 // Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
   //TCCR1A = (1<<COM1A1 | 1<<COM1A0); // Set OC1A/OC1B on Compare Match, clear OC1A/OC1B at BOTTOM (inverting mode)

   //Table 13-4. Waveform Generation Mode Bit Description
   // mode 14: Fast PWM, top: ICR1
   TCCR1A |=  1<<WGM11;
   TCCR1B = (1<<WGM13 | 1<<WGM12);
   
   // timer2: rpm update timer (only update the hz display a couple of times per second)
   TCCR2A = 0;
   TCCR2B = 0;

   //Table 15-8. Waveform Generation Mode Bit Description
   // mode 1: PWM phase correct (slower: counts up and down)
   TCCR2A = 1<<WGM20;
   TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20; // prescaler: 1024
   TIMSK2 = 1<<TOIE2;

   set_rpm(rpm);
}

ISR(PCINT2_vect){
   if (clk_pin_state()){
      rotary_update = true;
      rotary_inputs++;
      rotary_direction = direction_pin_state();
   }
}

ISR(TIMER2_OVF_vect){
  timer2_ovf_counter++;

  if (timer2_ovf_counter > 10){
     timer2_ovf_counter = 0;
     update_hz_display = true;
  }
}

bool clk_pin_state() {
   return PIND & 1<<clk_pin;
}

bool direction_pin_state() {
   return PIND & 1<<direction_pin;
}

void set_rpm(uint16_t new_rpm){  
   // hz is used to calculate the correct ICR1 value, two zeroes for more precision are added
   hz = new_rpm * 100ul / 60;
   
   // find the most suitable prescaler: most precision without overflowing ICR1
   // predefined prescaler table:
   // *prescaler*  -> *min rpm*
   // no prescaler -> 14649   
   //            8 -> 1832 
   //           64 -> 229
   //          256 -> 58
   //         1024 -> 15

   if (new_rpm >= 14649){
      // no prescaler
      TCCR1B = (TCCR1B & 0xF8) | 1<<CS10;
      prescaled_clock = f_cpu_00;
      //Serial.println("no prescaler");
   } else if (new_rpm >= 1832){
      // prescaler: 8
      TCCR1B = (TCCR1B & 0xF8) | 1<<CS11;
      prescaled_clock = f_cpu_00_8;
      //Serial.println("prescaler: 8");
   } else if (new_rpm >= 230){
      // prescaler: 64
      TCCR1B = (TCCR1B & 0xF8) | 1<<CS11 | 1<<CS10;
      prescaled_clock = f_cpu_00_64;
      //Serial.println("prescaler: 64");
   } else if (new_rpm >= 58){
      // prescaler: 256
      TCCR1B = (TCCR1B & 0xF8) | 1<<CS12;
      prescaled_clock = f_cpu_00_256;
      //Serial.println("prescaler: 256");   
   } else {
      // prescaler: 1024
      TCCR1B = (TCCR1B & 0xF8) | 1<<CS12 | 1<<CS10;
      prescaled_clock = f_cpu_00_1024;
      //Serial.println("prescaler: 1024");

      if (new_rpm < 15){
         // can't go slower than this because ICR1 would then overflow
         // hack:
         hz = 25;      // 16000000 / 1024 = 15625. 15625 / 0.25 = 62500
         new_rpm = 15; // also override the requested rpm
      }      
   }
   
   //Serial.print("prescaled_clock: ");
   //Serial.println(prescaled_clock);

   // subtract one from ICR1:
   // example: fictional counter increments 4 times per second: 0123
   // to achieve 2 hz, the match needs to occur at 1, not at 2 

   ICR1 = prescaled_clock / hz - 1;

   //Serial.print("ICR1: ");
   //Serial.println(ICR1);

   OCR1A = ICR1 * 0.03;
   rpm   = new_rpm;

   // in some cases the desired rpm isn't exactly achievable because even the most ideal prescaler and ICR1 value arent precise enough
   // display the theoretical rpm value based on the set prescaler and ICR1
   
   uint16_t display_rpm  = prescaled_clock / 100 * 60 / (ICR1 + 1);

   display1.showNumberDec(display_rpm);
}

void process_rotary_update(){
   static byte streak      = 0;
   static byte streak_up   = 0;
   static byte streak_down = 0;
   
   uint32_t time_since_last_rotary_input = millis() - last_rotary_input;
   //Serial.println(time_since_last_rotary_input);

   last_rotary_input = millis();

   // updating one TM1637 display takes in excess of 20 ms
   // turning the rotary encoder moderately quick is also around 25 ms per click
   // rotary_inputs: accumulated rotary_inputs since the last loop iteration

   static bool lock_direction_state;
   static bool lock_direction;
   if (time_since_last_rotary_input < 300){

      streak++;

      if (lock_direction_state){
         rotary_direction = lock_direction;
      } else {
         // direction lock: more input speed
         // also counter acts false readings of the rotary encoder
         
         if (!rotary_direction){
            streak_up++;
         } else {
            streak_down++;        
         }

         // override measured direction in case of streak
         if (streak_up > streak_down){
            rotary_direction = 0;
         } else if (streak_down > streak_up) {
            rotary_direction = 1;
         }

         // lock the direction after 4 inputs
         if (streak == 4){
            lock_direction_state = true;
            lock_direction       = rotary_direction;
         }
      }
      
   } else {
      streak      = 0;
      streak_up   = 0;
      streak_down = 0;

      lock_direction_state  = false;
   }

   uint16_t multiplier;

   if (streak < 19){
      multiplier = 1;
   } else if (streak < 35){
      multiplier = 10;
   } else {
      multiplier = 100;
   }

   multiplier   *= rotary_inputs;
   rotary_inputs = 0;
   
   uint16_t new_rpm = rpm;
      
   if (!rotary_direction){
      new_rpm += multiplier;
   } else {
      new_rpm -= multiplier;
   }
      
   // unsigned rpm flaw: rpm - 100 can lead to new_rpm overflowing to 65000+, set new_rpm to 15 instead
   if (new_rpm > 65000){
      new_rpm = 15;
   }
   
   set_rpm(new_rpm);
}

void show_hz(){
   if (hz < 10000){
      display2.showNumberDecEx(hz, 64); // colon or point (top led masked by tape)
   } else {
      display2.showNumberDec(hz / 100);
   }  
}

void loop(){
   if (rotary_update){
      process_rotary_update();
      rotary_update = false;
      rpm_changed   = true;
   }

   if (update_hz_display && (millis() - last_rotary_input) > 400){
      // to make rotary input a bit more responsive the hz display is only updated a few times per second
      update_hz_display = false;
      rpm_changed       = false;
     
      show_hz();
   }
}
