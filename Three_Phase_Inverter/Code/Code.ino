#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "driver/mcpwm_prelude.h"
#include <iostream>
#include <string>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define STOP_BUTTON 25
#define REVERSE_BUTTON 26
#define FORWARD_BUTTON 27

Adafruit_SSD1306 display(SCREEN_WIDTH,SCREEN_HEIGHT, OLED_RESET);
// //The MCPWM peripheral is a versatile PWM generator, 
// which contains various submodules to make it a key element
// in power electronic applications like motor control, digital power, and so on.
// Documentation: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/mcpwm.html#mcpwm-resource-allocation-and-initialization
//resolution = 1 MHz
// period = 50 ticks
// PWM frequency = 1,000,000 / 50
//               = 20 kHz

// MCPWM Timer: The time base of the final PWM signal. 
// It also determines the event timing of other submodules.
mcpwm_timer_handle_t timer = NULL;
mcpwm_timer_config_t timer_config = {
    .group_id = 0,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT, //PLL clock 160Hz
    .resolution_hz = 1000000,
    .period_ticks = 50,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP
};

// MCPWM Operator: The key module that is responsible
//  for generating the PWM waveforms. It consists of other 
//  submodules, like comparator, PWM generator, dead time, 
//  and carrier modulator.
mcpwm_operator_config_t operator_config = {
  .group_id = 0
};

//MCPWM Comparator: The compare module takes the time-base 
// count value as input, and continuously compares it to the 
// threshold value configured. When the timer is equal to any 
// of the threshold values, a compare event will be generated 
// and the MCPWM generator can update its level accordingly.
mcpwm_comparator_config_t comparator_config= {
  .flags.update_cmp_on_tez = true,
};

//MCPWM Generator: One MCPWM generator can generate a pair of PWM waves,
//  complementarily or independently, based on various events triggered 
//  by other submodules like MCPWM Timer and MCPWM Comparator.
mcpwm_generator_config_t gen_config;

struct PWM_Channel{
  const int pinHigh;
  const int pinLow;
  mcpwm_oper_handle_t op = NULL;
  mcpwm_cmpr_handle_t comp = NULL;
  mcpwm_gen_handle_t high, low = NULL;
};

//phase pins
PWM_Channel channels[] = {{.pinHigh = 36, .pinLow = 4},
                          {.pinHigh = 5, .pinLow = 12},
                          {.pinHigh = 13, .pinLow = 32}};

const int pwmFreq = 20000; 
int outputFreq = 50;
const int resolution = 10;

float theta = 0;

//Battery sensing
const int batt = 35;
const float battRatio = 9.18;

//States
volatile bool InverterOn = false;
volatile bool ReverseMode = false;

//Debounce
bool LastStateFwrd = LOW;
bool LastStateRvrs = LOW;
bool LastStateStp = LOW;
unsigned long lastDebounceForward = 0;
unsigned long lastDebounceReverse = 0;
unsigned long lastDebounceStop = 0;

int battDividerRatio = 9.18;


bool seqForward[6][3] = {
  {1,0,0},  // Step 1
  {1,1,0},  // Step 2
  {0,1,0},  // Step 3
  {0,1,1},  // Step 4
  {0,0,1},  // Step 5
  {1,0,1}   // Step 6
};
void setup() {
  Serial.begin(115200);

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC,0x3C); 

  pinMode(FORWARD_BUTTON, INPUT);
  pinMode(REVERSE_BUTTON, INPUT);
  pinMode(STOP_BUTTON, INPUT);

  mcpwm_new_timer(&timer_config, &timer);
  for(int i = 0; i < sizeof(channels)/sizeof(channels[0]); i++){
    pinMode(channels[i].pinHigh, OUTPUT);
    pinMode(channels[i].pinLow, OUTOUT);
    //creating operator
    mcpwm_new_operator(&operator_config,&channels[i].op);
    mcpwm_operator_connect_timer(channels[i].op,timer);
    //creating comparator
    mcpwm_new_comparator(channels[i].op,&comparator_config,&channels[i].comp);
    mcpwm_comparator_set_compare_value(channels[i].comp,25);
    //creating generators
    gen_config.gen_gpio_num = channels[i].pinHigh;
    mcpwm_new_generator(channels[i].op, &gen_config, &channels[i].high);
    gen_config.gen_gpio_num = channels[i].pinLow;
    mcpwm_new_generator(channels[i], &gen_config, &channels[i].low);
    //pwm behavio
    mcpwm_generator_set_action_on_timer_event(
      channel[i].high,
      MCPWM_GEN_TIMER_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP,
          MCPWM_TIMER_EVENT_EMPTY,
          MCPWM_GEN_ACTION_HIGH
      )
    );  
    mcpwm_generator_set_action_on_compare_event(
      channels[i].high,
      MCPWM_GEN_COMPARE_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP,
          channels[i].comp,
          MCPWM_GEN_ACTION_LOW
      )
    );
    //dead time configuration
    mcpwm_dead_time_config_t dt_config = {
      .posedge_delay_ticks = 2,
      .negedge_delay_ticks = 2,
      .flags.invert_output = true
    };
    mcpwm_generator_set_dead_time(channels[i].high, channels[i].low, &dt_config);
  }

  //starting timer
  mcpwm_timer_enable(timer);
  mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
}
void loop() {
  float battVoltage = readBatteryVoltage();
  if(battVoltage < 10.0){
    InverterOn = false;
  }
  oledRun(battVoltage,ReverseMode);

  // phases();
  

  //Forward nutton pressed + dbounce checking
  bool forwardPress = digitalRead(FORWARD_BUTTON);
  if(forwardPress != LastStateFwrd && (millis()-lastDebounceForward > 50)){
      InverterOn = true;
      ReverseMode = false;
      lastDebounceForward = millis();
  }
  LastStateFwrd = forwardPress;

  //Reverse button pressed + debounce checking
  bool reversePress = digitalRead(REVERSE_BUTTON);
  if(reversePress != LastStateRvrs && (millis()-lastDebounceReverse > 50)){
    InverterOn = true;
    ReverseMode = true;
    lastDebounceReverse = millis();
  }
  LastStateRvrs = reversePress;

  //Stop button pressed + debounce checking
  bool stopPress = digitalRead(STOP_BUTTON);
  if(stopPress != LastStateStp && (millis()-lastDebounceForward) > 50){
    InverterOn = false;
    lastDebounceStop = millis();
  }
  LastStateStp = millis();

}

void oledRun(float val, bool dir){
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(2);
  display.print("Waiting....");
  delay(2);
  display.clearDisplay();
  display.setCursor(0,32);
  display.print("Battery Level: ");
  display.println(val);
  if(dir){
    display.print("Reverse");
  }
  else{
    display.print("Forward");
  }
  display.display();
}
float readBatteryVoltage(){
  int sum = 0;
  for(int i = 0; i < 10; i++){
    sum += analogRead(batt);
  }
  delay(2);
  float adcVal = sum / 10.0;
  return (adcVal*3.3/4095) * battDividerRatio;
}

void updateSPWM(){
  //generating sine waves 120 degrees shifts
  float A = sin(theta);
  float B = sin(theta - 2.0 * PI / 3.0);
  float C = sin(theta + 2.0 * PI / 3.0);
  //converting angles to duty cycles
  int dutyA = (A + 1.0) * 25.0;
  int dutyB = (B + 1.0) * 25.0;
  int dutyC = (C + 1.0) * 25.0;
  //updating comparators
  mcpwm_comparator_set_compare_value(channels[0].comp, dutyA);
  mcpwm_comparator_set_compare_value(channels[1].comp, dutyB);
  mcpwm_comparator_set_compare_value(channels[2].comp, dutyC);
  //repeating sine waves by updating theta @outputFreq
  theta += 2.0 * PI * outputFreq / 20000.0;
  if (theta >= 2.0 * PI) {
      theta -= 2.0 * PI;
  }
}