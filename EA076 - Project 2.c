/* EA076 - Project 2
 * Lucas A. P. and Vitor O. E.
 */

/**************************************************************************************************
 *                                        Definitions
 **************************************************************************************************/
// Keyboard module definitions:
// Defining pins that check the state of the columns and rows of the keyboard matrix
#define C1 4
#define C2 5
#define C3 6
#define L1 7
#define L2 8
#define L3 9
#define L4 10

#include <SPI.h>
#include <LoRa.h>

// Waiting time in ms for the confirmation of the level logic change
#define T_debounce 30
// General variables of the keyboard
unsigned int row, flag_timer=0, flag_print=0, confirmed_row, previous_active=0;
char character;
// General variables of the memory
unsigned int collection_time=0;
// Temperature sensor and 7-segment display module definitions
// Library for I2C
#include <Wire.h>
// Pin definitions:
#define tempPin A1
// Number of samples for the average filter
#define N_samples 50

// Variables:
unsigned int time = 0, temp_time = 0, sample_count = 0;
float temperature, final_temperature=0, sum;
float analog_temp;
// Controls temperature measurements
bool flag_readTemperature = 1;
float sample_list[N_samples];
// Variables and flags for I2C transmission control:
volatile int i2c_count, flag_switchDigit, end = 1;
// Memory variables
#define static_address 0b1010 // static address needed for writing to memory
unsigned int datapointer; // data pointer

/*************************************************************************************************
 *                                           LCD
 *************************************************************************************************/
// Library for LCD 16x2
#include <LiquidCrystal.h>
// Initial pin configuration for LCD:
const int rs = 13, en = 12, d4 = 11, d5 =3, d6 = 2, d7 = A0;
// Function used to determine the MCU pins responsible for the LCD pins
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

/**************************************************************************************************
 *                                            FSMs
 **************************************************************************************************/
// Planned states for the keyboard
typedef enum keyboard_state_machine{
  TRANSITIONAL,
  DISPLAY_CHARACTER,
  TRANSITIONAL2,
  PRESSED,
  ROW_1,
  ROW_2,
  ROW_3,
  ROW_4,
  COLUMN_1,
  COLUMN_2,
  COLUMN_3,
} keyboard_state_machine;

// Planned states for the menu
typedef enum menu_state_machine{
  RESET,
  STATUS,
  START_COLLECTION,
  STOP_COLLECTION,
  DATA_TRANSFER,
  INITIAL,
} menu_state_machine;

// State variables and initial state
keyboard_state_machine keyboard_state = ROW_1;
menu_state_machine menu_state = INITIAL;
// Vectors that store the state of the column at different points of execution
bool column[3], aux_column[3], confirmed_column[3];
// Auxiliary variables of the menu machine
int digits[4], inc=0;
unsigned int N_measurements=0;
// flags for menu control
bool enable_transfer=0, enable_collection=0, flag_lcd_write=0, flag_rewrite=1, flag_start=0;
bool flag_collection=0;

/************************************************************************************************
 *                                          Setup
 ************************************************************************************************/
void setup() {
  // Define GPIOs
  pinMode(C1,INPUT_PULLUP); pinMode(C2,INPUT_PULLUP); pinMode(C3,INPUT_PULLUP);
  pinMode(L1,OUTPUT); pinMode(L2,OUTPUT); pinMode(L3,OUTPUT); pinMode(L4,OUTPUT);
  Serial.begin(9600);

  // Initialize LoRa
  initializeLoRa();
  // Initial state of the rows
  digitalWrite(L1,LOW);
  digitalWrite(L2,LOW);
  digitalWrite(L3,LOW);
  digitalWrite(L4,LOW);

  cli(); // disable interrupts
  configuracao_Timer0(); // configure the timer
  sei(); // enable interrupts

 // Initial configuration of the LCD display when starting the program
  lcd.begin(16,2); // initialize the LCD circuit and define the display size (16x2)

  // Temperature sensor
  pinMode(tempPin, INPUT); // selects the reading pin of the sensor
  analogReference(INTERNAL); // selects the internal reference (V_AREF) of 1.1V of the ADC

  Wire.begin(); // Initialize the I2C bus
  // necessary commands to initialize the pointer every time the system turns on
  // reading the data in the EEPROM (non-volatile) and avoiding data overwriting
  unsigned char pos1 = ler(2046);
  unsigned char pos2 = ler(2047);
  word pos3 = word(pos1,pos2);
  datapointer=pos3;
  //Serial.println(datapointer);
}

/***********************************************************************************************
 *                                           Main
 ***********************************************************************************************/
void loop() {
///////////////////////////////////// Temperature sensor ////////////////////////////////////////////
  // Condition that checks for a measurement every 40 ms
  if (flag_readTemperature == 1) {
    // Calculation of temperature from the analog value read on the sensor (explanation of the formula in the module report)
    analog_temp = analogRead(tempPin);
    // reset flag
    flag_readTemperature = 0;
    // add temperature read to the sample vector
    if (sample_count < N_samples) {
      sample_list[sample_count] = temperature;
      sum += analog_temp;
      sample_count++;
    }
    // after reading the configured number of samples, calculate the average
    else {
      sample_count = 0;
      // a moving average filter was adopted as a strategy to reduce the influence of noise in the measurements
      // it was assumed that the noise present in the signal has a Gaussian fdp with zero mean, so the moving average filter is adequate for the proposed situation
      /* The formula used was: [(1.1/1024)*sum(temperature_readings)/10mV] / (N_samples - 1)
      Given the relationship Vout = 10mV*T(in °C), and we perform the DAC conversion by (1.1/1024)*sum(temperature_readings) */
      final_temperature = 0.107421875*(sum / ((float) N_samples-1)); // average calculation;
      sum = 0;
    }
  }

  // send calculated average to the 7-segment display
  transmiteTemperatura(100.0*final_temperature);
  // sending data via LoRa
  sendLoRaData(String(final_temperature));

////////////////////////////////////
              Temperature collection
///////////////////////////////////////////////////////////

/* The verification checks if it is possible to write a new entry in the memory, to ensure a complete write */
if(flag_collection==1){
  escrever_temperatura(final_temperature); // write memory
  flag_collection=0;
 }
/////////////////////////////////////////////
                      Menu
//////////////////////////////////////////////////

// The following finite state machine handles the situation of the menu that the user can select the actions to be performed in the system

  switch(menu_state){

  // initial case: shows on the LCD display the initial menu and asks for a command to be performed
  case INITIAL:

  // the start flag is used to detect if it is the first command (start of the entire operation)
  if(flag_start==0 && flag_rewrite==1){
      lcd.setCursor(0,0);
      lcd.write("Datalogger EA076   "); // writes Datalogger EA076 on the first line of the display
      lcd.setCursor(0,1);
      lcd.write("Choose 1-5        "); // writes Choose 1-5 on the first line of the display
      flag_rewrite=0;
  // verification of a new command during system operation (other than initialization)
  }else if(flag_start==1 && flag_rewrite==1){
      lcd.setCursor(0,1);
      lcd.write("Choose 1-5        "); // writes Choose 1-5 on the first line of the display
      flag_rewrite=0;
  }
  flag_lcd_write=0;
  // switch-case structure that decides which state of the machine should be executed based on the character detected on the keyboard
    switch(character){
      case '1':
      menu_state=RESET;
      break;
      case '2':
      menu_state=STATUS;
      break;
      case '3':
      menu_state=START_COLLECTION;
      break;
      case '4':
      menu_state=STOP_COLLECTION;
      break;
      case '5':
      menu_state=DATA_TRANSFER;
      break;
      default:
      break;
    }
  break;

  // this case is responsible for showing the status of the memory (# of available positions and written)
  case STATUS:
  // command confirmation
  if(flag_lcd_write==0){
      lcd.setCursor(0,0);
      lcd.write("Show status?     "); //
      lcd.setCursor(0,1);
      lcd.write("Yes (#) No (*)");
      flag_lcd_write=1;
  }
  // case confirmed: shows from the value of the pointer stored in memory the status of the memory
   switch(character){
      case '#': // confirm
      lcd.setCursor(0,0);
      lcd.write("# records:");
      lcd.print(datapointer);
      lcd.write("            ");
      lcd.setCursor(0,1);
      lcd.write("# available:");
      lcd.print(1023-datapointer);
      lcd.write("            ");
      menu_state = INITIAL;
      break;
      // returns to the initial menu if not confirmed
      case '*':
      menu_state = INITIAL;
      flag_start=0;
      flag_rewrite=1;
      break;
    }
  break;

  // case to completely erase the memory
  case RESET:
  // confirm command
  if(flag_lcd_write==0){
      lcd.setCursor(0,0);
      lcd.write("Erase memory? ");
      lcd.setCursor(0,1);
      lcd.write("Yes (#) No (*)");
      flag_lcd_write=1;
  }
      switch(character){
      case '#': // confirmed command shows the message 'Memory erased' and erases the memory resetting the pointer
      lcd.setCursor(0,0);
      lcd.write("Memory erased");
      lcd.setCursor(0,1);
      lcd.write("                ");
      flag_start=1;
      flag_rewrite=1;
      menu_state = INITIAL;
      escrever(2046,0x00);
      _delay_ms(5);
      escrever(2047,0x00);
      _delay_ms(5);
      datapointer=0;
      break;
      // case not confirmed returns to the initial menu
      case '*':
      menu_state = INITIAL;
      flag_start=0;
      flag_rewrite=1;
      break;
    }
  break;

  // case to activate data collection and write to memory
  case START_COLLECTION:

  // confirm command on display
  if(flag_lcd_write==0){
      lcd.setCursor(0,0);
      lcd.write("Start collection?       ");
      lcd.setCursor(0,1);
      lcd.write("Yes (#) No (*)");
      flag_lcd_write=1;
  }

      switch(character){
      case '#': // confirm
      enable_collection=1; // confirm a flag to enable collection;
      // indicates on the display the start of the collection
      lcd.setCursor(0,0);
      lcd.write("Collection started      ");
      lcd.setCursor(0,1);
      lcd.write("                     ");
      flag_start=1;
      flag_rewrite=1;
      menu_state=INITIAL;
      break;
      // returns to the initial menu if not confirmed
      case '*':
      menu_state = INITIAL;
      flag_start=0;
      flag_rewrite=1;
      break;
      }
  break;

  // case to finalize data collection and writing to memory
  case STOP_COLLECTION:
  // confirm command on display
  if(flag_lcd_write==0){
      lcd.setCursor(0,0);
      lcd.write("End of collection?     ");
      lcd.setCursor(0,1);
      lcd.write("Yes (#) No (*)");
     flag_lcd_write=1;
  }
      switch(character){
      case '#': // confirm
      enable_collection=0; // confirm a flag to finalize collection;
      lcd.setCursor(0,0);
      lcd.write("Collection finished   ");
      lcd.setCursor(0,1);
      lcd.write("                     ");
      flag_start=1;
      flag_rewrite=1;
      menu_state=INITIAL;
      break;
      // returns to the initial menu if not confirmed
      case '*':
      menu_state = INITIAL;
      flag_start=0;
      flag_rewrite=1;
      break;
      }
  break;

  // case used to confirm data transfer and ask how many data to be transferred
  case DATA_TRANSFER:
  if(flag_lcd_write==0){
      lcd.setCursor(0,0);
      lcd.write("Transfer data?   ");
      lcd.setCursor(0,1);
      lcd.write("Yes (#) No (*)");
      flag_lcd_write=1;
   }
   // verify confirmation + data to be transferred
      if (character == '#' || enable_transfer == 1){
        if(enable_transfer==0){
          lcd.setCursor(0,0);
          lcd.write("How many data?    ");
          lcd.setCursor(0,1);
          lcd.write("                  ");
          lcd.setCursor(0,1);
          enable_transfer=1;
          character='%';
          break;
        }
        if(character!='%'){
          // structure that detects which digit is and how many digits have already been entered (0 to 1023)
          switch(character){
            case '#':
            // structure to mount the value of measurements that should be read
            switch(inc){
            case 1:
              N_measurements = digits[0];
            break;
            case 2:
              N_measurements = digits[0]*10+digits[1]*1;
            break;
            case 3:
              N_measurements = digits[0]*100+digits[1]*10+digits[2]*1;
            break;
            case 4:
              N_measurements = digits[0]*1000+digits[1]*100+digits[2]*10+digits[3]*1;
            break;
            default:
            break;
            }
             // verify if number N is in ]0;1023]
              if(inc<=4 && (N_measurements<=1023 && N_measurements>1)){
                transfereSerial(N_measurements); // transfer function
                menu_state=INITIAL;
                flag_start=0;
                flag_rewrite=1;
                enable_transfer=0; // finalizes transfer
                inc=0;
              }
              // case to deal with an invalid value N and show the corresponding message on the LCD display
              else{
                lcd.setCursor(0,0);
                lcd.write("Invalid value:(");
                menu_state=INITIAL;
                flag_start=1;
                flag_rewrite=1;
                inc=0;
                enable_transfer=0;
              }
          break;
          case '*':
          break;
          default:
          // Verification to see how many digits have already been entered and store in a vector to form the number N
            if(inc<=4)
              digits[inc] = character - '0';
            inc++; // increment variable for the next digit to be entered
            lcd.write(character); // writes the selected digit on the LCD display
            character='%';
          break;
          }
     }
    }else if (character == '*'){menu_state = INITIAL;flag_start=0;flag_rewrite=1;}
  break;
  default:
  break;
}

////////////////////////////////////// Matrix keyboard ///////////////////////////////////////////
  // Finite state machine responsible for handling the keyboard and detecting a pressed key
  switch(keyboard_state){
  // The first 4 states LINE_N have similar structures: activate the keyboard line corresponding to the state
  // and check if the same has any column pressed, which is an event that leads to a transitional state (Debounce treatment).
  case ROW_1:
    // Configures the logical level of the lines for the corresponding state
     digitalWrite(L1,LOW);
     digitalWrite(L2,HIGH);
     digitalWrite(L3,HIGH);
     digitalWrite(L4,HIGH);
    // Reads the logical level of the columns
     column[0] = digitalRead(C1);
     column[1] = digitalRead(C2);
     column[2] = digitalRead(C3);
     row = 1;
     // Checks if there was any key pressed and changes state
     if(column[0] == 0 || column[1] == 0 || column[2] == 0){
        // stores pressed row and columns
        aux_column[0] = column[0];
        aux_column[1] = column[1];
        aux_column[2] = column[2];
        confirmed_row=row;
        keyboard_state = TRANSITIONAL;
     }else{keyboard_state = ROW_2;} // Otherwise, tests the next line
  break;
  case ROW_2:
    // Configures the logical level of the lines for the corresponding state
     digitalWrite(L1,HIGH);
     digitalWrite(L2,LOW);
     digitalWrite(L3,HIGH);
     digitalWrite(L4,HIGH);
     // Reads the logical level of the columns
     column[0] = digitalRead(C1);
     column[1] = digitalRead(C2);
     column[2] = digitalRead(C3);
     row = 2;
     // Checks if there was any key pressed and changes state
     if(column[0] == 0 || column[1] == 0 || column[2] == 0){
        // stores pressed row and columns
        aux_column[0] = column[0];
        aux_column[1] = column[1];
        aux_column[2] = column[2];
        confirmed_row=row;
        keyboard_state = TRANSITIONAL;
     }else{keyboard_state = ROW_3;} // Otherwise, tests the next line
  break;
  case ROW_3:
    // Configures the logical level of the lines for the corresponding state
     digitalWrite(L1,HIGH);
     digitalWrite(L2,HIGH);
     digitalWrite(L3,LOW);
     digitalWrite(L4,HIGH);
     // Reads the logical level of the columns
     column[0] = digitalRead(C1);
     column[1] = digitalRead(C2);
     column[2] = digitalRead(C3);
     row = 3;
     // Checks if there was any key pressed and changes state
     if(column[0] == 0 || column[1] == 0 || column[2] == 0){
        // stores pressed row and columns
        aux_column[0] = column[0];
        aux_column[1] = column[1];
        aux_column[2] = column[2];
        confirmed_row=row;
        keyboard_state = TRANSITIONAL;
     }else{keyboard_state = ROW_4;} // Otherwise, tests the next line
  break;
  case ROW_4:
    // Configures the logical level of the lines for the corresponding state
     digitalWrite(L1,HIGH);
     digitalWrite(L2,HIGH);
     digitalWrite(L3,HIGH);
     digitalWrite(L4,LOW);
     // Reads the logical level of the columns
     column[0] = digitalRead(C1);
     column[1] = digitalRead(C2);
     column[2] = digitalRead(C3);
     row = 4;
     // Checks if there was any key pressed and changes state
     if(column[0] == 0 || column[1] == 0 || column[2] == 0){
        // stores pressed row and columns
        aux_column[0] = column[0];
        aux_column[1] = column[1];
        aux_column[2] = column[2];
        confirmed_row=row;
        keyboard_state = TRANSITIONAL;
     }else{keyboard_state = ROW_1;} // Otherwise, tests the next line
  break;
  case TRANSITIONAL:
      if(flag_timer==0){ // Resets transitional state timer flag
        time=0;
        flag_timer=1;
        keyboard_state = ROW_1;
      }else if (time >= T_debounce){ // If the debounce time has passed, checks if the state remains the same, stores the state of the columns and changes the state of the machine
        if((aux_column[0] == column[0]) &&  (aux_column[1] == column[1]) && (aux_column[2] == column[2])){
          keyboard_state = COLUMN_1;
          confirmed_column[0] = aux_column[0];
          confirmed_column[1] = aux_column[1];
          confirmed_column[2] = aux_column[2];
          keyboard_state = COLUMN_1;
        }
      }
      else{keyboard_state = ROW_1;}
      break;
  case DISPLAY_CHARACTER: // state used for debugging
    // Prints the pressed key on the LCD and resets flag and state
    //Serial.println(character);
    //lcd.print(String(character));
    //lcd.setCursor(0,2);
    keyboard_state=ROW_1;
    flag_timer=0;
  break;
  case PRESSED:
  // Checks if there is any change in the key, which would correspond to the button being released
    column[0] = digitalRead(C1);
    column[1] = digitalRead(C2);
    column[2] = digitalRead(C3);
    if(column[previous_active] == 1){
        aux_column[0] = column[0];
        aux_column[1] = column[1];
        aux_column[2] = column[2];
        confirmed_row=row;
        keyboard_state = TRANSITIONAL2;
     }
  break;
  case TRANSITIONAL2:
      if(flag_timer==0){ // Resets transitional state timer flag
        time=0;
        flag_timer=1;
        keyboard_state = ROW_1;
      }else if (time >= T_debounce){ // If the debounce time has passed, changes the state of the machine to print the pressed key
       keyboard_state = DISPLAY_CHARACTER;
      }
      break;
  case COLUMN_1:
      // Checks if the pressed key is from column 1, changes the state of the variables and finds the corresponding character
      if (confirmed_column[0] == 0){
        character = dicionario(1,confirmed_row);
        keyboard_state = PRESSED;
        previous_active = 0;
        confirmed_column[0] = 1;
        confirmed_column[1] = 1;
        confirmed_column[2] = 1;
        aux_column[0] = 1;
        aux_column[1] = 1;
        aux_column[2] = 1;
        }else {keyboard_state = COLUMN_2;}
  break;
  case COLUMN_2:
      // Checks if the pressed key is from column 2, changes the state of the variables and finds the corresponding character
      if (confirmed_column[1] == 0){
        character =  dicionario(2,confirmed_row);
        keyboard_state = PRESSED;
        previous_active = 1;
        confirmed_column[0] = 1;
        confirmed_column[1] = 1;
        confirmed_column[2] = 1;
        aux_column[0] = 1;
        aux_column[1] = 1;
        aux_column[2] = 1;
        }else {keyboard_state = COLUMN_3;}
  break;
  case COLUMN_3:
  // Checks if the pressed key is from column 3, changes the state of the variables and finds the corresponding character
      if (confirmed_column[2] == 0){
        character =  dicionario(3,confirmed_row);
        keyboard_state = PRESSED;
        previous_active = 2;
        confirmed_column[0] = 1;
        confirmed_column[1] = 1;
        confirmed_column[2] = 1;
        aux_column[0] = 1;
        aux_column[1] = 1;
        aux_column[2] = 1;
       }else {keyboard_state = ROW_1;}
  break;
  default:
  keyboard_state = ROW_1;
 }
}

/*******************************************************************************************
 *                                           Timers
 *******************************************************************************************/
void configuracao_Timer0(){
  // Timer 0 (8 bits) configuration to generate periodic interrupts
// every 1.6ms in CTC mode
  // Clock = 16e6 Hz
  // Prescaler = 1024
  // Range = 25 (counting from 0 to OCR0A = 24)
  // Interval between interrupts: (Prescaler/Clock)*Range =
// (1024/16e6)*(24+1) = 1.6ms

  // TCCR0A – Timer/Counter Control Register A
  // COM0A1 COM0A0 COM0B1 COM0B0 – – WGM01 WGM00
  // 0      0      0      0          1     0
  TCCR0A = 0x02;

  // OCR0A – Output Compare Register A
  OCR0A = 24;

  // TIMSK0 – Timer/Counter Interrupt Mask Register
  // – – – – – OCIE0B OCIE0A TOIE0
  // – – – – – 0      1      0
  TIMSK0 = 0x02;

  // TCCR0B – Timer/Counter Control Register B
  // FOC0A FOC0B – – WGM02 CS02 CS01 CS0
  // 0     0         0     1    0    1
  TCCR0B = 0x05;
}

// Interrupt service routine of the timer
ISR(TIMER0_COMPA_vect){
  time++; // increments variable that stores fixed periods

  // the condition is checked when the user selects the data collection mode
  if(enable_collection==1){
    collection_time++; // counts in this case the time spent on the collection
  }

  temp_time++; // increments variable that stores the period for the management of the temperature measurement
  i2c_count++; // variable responsible for counting the time in the scope of I2C transmission (that's why a new variable)
  // counting of 2 x 1.6 ms = 3.2 ms, to verify flag for switching digit and start new transmission. The period must be less than
  // 1/30 s = 33 ms due to human persistence of vision that will allow not perceiving the digit switching that is being constantly performed
  if (i2c_count > 2) {
    // configure the flags for new time count and new transmission
    i2c_count = 0;
    flag_switchDigit = 1;
  }

  // allows the evaluation of the temperature
  if (temp_time > 25) {
    flag_readTemperature = 1;
    temp_time = 0;
  }
  // case the collection is enabled and the time passed from this collection is 1200*1.6ms = 2 ms (approximately) (~ time for new measurement)
  // allows a new writing of the collection made and zeros collection time for a new count
  if((enable_collection==1) && (collection_time>1200)){
      flag_collection=1; // allows new writing of the collection made
      collection_time=0;
  // case the number of data stored in the memory exceeds the limit, it should be restarted the count.
      if(datapointer==1022){
        datapointer=0;
      }
  }
}

/*******************************************************************************************
 *                                    Dictionary
 *******************************************************************************************/
char dicionario(int column, int row){ // Function that receives the pressed column and row and returns the corresponding character
  int key;
  char temp[2];
  if(row!=4){ // case it is from the first three rows
  row--;
  key = column+row*3;
  itoa(key,temp,10); // transfer from integer to char vector
  }else if(row==4 && column==1){ // case it is the first column of the last row
    temp[0]='*';
  }else if(row==4 && column==2){ // case it is the second column of the last row
    temp[0]='0';
  }else if(row==4 && column==3){ // case it is the third column of the last row
    temp[0]='#';
  }
  return temp[0]; // returns character
}

/*******************************************************************************************
 *                                  7-Segment Display
 *******************************************************************************************/
// Module 6:
// flags created to verify each digit transmission
int unitTransmitted = 1, tenTransmitted = 1, hundredTransmitted = 1, thousandTransmitted = 1;

/*
   Function to handle temperature and I2C transmission for the
7-segment displays
   Input: estimated speed (integer part)
   This function is responsible for multiplexing in time the control signals (cathode to be activated)
   and data (value to be displayed on the display), by means of a port expander, a BCD decoder and
   using the I2C interface present in the microcontroller.

*/

void initializeLoRa() {
  // Set the LoRa pin configuration
  LoRa.setPins(SS, RST, DIO0);

  // Initialize LoRa with the appropriate frequency
  if (!LoRa.begin(915E6)) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  Serial.println("LoRa initialized successfully.");
}

void sendLoRaData(String data) {
  LoRa.beginPacket();   // Start the LoRa packet
  LoRa.print(data);     // Add the data to the packet
  LoRa.endPacket();     // End and send the packet
  Serial.println("Data sent via LoRa: " + data);
}

String receiveLoRaData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();  // Append each byte received to the string
    }
    Serial.println("Data received via LoRa: " + received);
    return received;
  }
  return "";
}


void transmiteTemperatura(float temperature_measured) {
  // casting of temperature (float -> int)
  int temperature_measured_int;
  temperature_measured_int = (int) temperature_measured;

  // Isolating units, tens, hundreds and thousands:
  // by means of division and remainder operations.
  int unit = 0, ten = 0, hundred = 0, thousand = 0 ;

  unit = temperature_measured_int % 10;
  ten = (temperature_measured_int / 10) % 10;
  hundred = (temperature_measured_int / 100) % 10;
  thousand = (temperature_measured_int) / 1000;

  /* With the numbers isolated for each decimal place, it is necessary to
select which display should be activated,
    corresponding to the digit in question. For this, the other displays
should be deactivated, and change the content of the decimal place
    corresponding to the new available measurement. Thus, it was used
masks with the following pattern:

    Decimal place | Mask
    Thousands     | 0b00001110
    Hundreds      | 0b00001101
    Tens          | 0b00001011
    Units         | 0b00000111

    By performing the bitwise OR logical operation, it was possible to maintain the original value of the estimated digit and force the pattern necessary for
    the cathodes, noticing their activation in low. In addition, it is
fundamental to note that the construction of the masks is interlinked to the
connections made
    in hardware, defining thus the order and construction of the bits of the
mask. By the hardware construction, it was necessary to perform a
shifting of the bits of the
    measurements of 4 positions, freeing thus the space for the control bits.

  */

  // masks created to select the corresponding display

  thousand = (14) | (thousand << 4);
  hundred = (13) | (hundred << 4);
  ten = (11) | (ten << 4);
  unit = (7) | (unit << 4);
  // I2C character transmission
  /*
     In this part of the function, a if-else-if flow condition was used, which
evaluated whether a digit had already been
     transmitted we would have the authorization to go to the next I2C transmission. This was done by means of control flags. The flag flag_switchDigit is
responsible for
     the verification in the question of the time to switch the digits, that is, the interval between a transmission and another. Already the flags of type
(decimal place)Transmitted,
     were true at the end of the previous decimal place transmission in order to avoid transmissions without having made the previous one.
  */
  /* Here we transmit each digit, the logic for all is similar*/
  // thousands
  if (flag_switchDigit == 1 && unitTransmitted == 1) {
    // initializing flags for transmission
    unitTransmitted = 0;
    Wire.beginTransmission(0x20); // selects the address of the port expander and starts the transmission
    Wire.write(thousand); // transmits digit corresponding
    Wire.endTransmission(); // ends the transmission

    // modify the flags to authorize the next transmission
    thousandTransmitted = 1;
    flag_switchDigit = 0;
  }

  // hundreds
  else if (flag_switchDigit == 1 && thousandTransmitted == 1) {
    thousandTransmitted = 0;
    Wire.beginTransmission(0x20);
    Wire.write(hundred);
    Wire.endTransmission();
    hundredTransmitted = 1;
    flag_switchDigit = 0;
  }
  // tens
  else if (flag_switchDigit == 1 && hundredTransmitted == 1) {
    hundredTransmitted = 0;
    Wire.beginTransmission(0x20);
    Wire.write(ten);
    Wire.endTransmission();
    tenTransmitted = 1;
    flag_switchDigit = 0;
  }
  // unit
  else if (flag_switchDigit == 1  && tenTransmitted == 1) {
    tenTransmitted = 0;
    Wire.beginTransmission(0x20);
    Wire.write(unit);
    Wire.endTransmission();
    unitTransmitted = 1;
    flag_switchDigit = 0;
  }
}
/*******************************************************************************************
 *                                 Memory
 *******************************************************************************************/

 /* The following functions are responsible for writing and reading the temperature in the EEPROM memory.
  *
  *   Temperature writing function
  *   Input: measured temperature
  *   The function below performs the treatment of storing the bytes corresponding to the measured value
  *   of the temperature in memory, manipulating them in order to allocate them in the correct addresses, as well as
  *   update the count for the next writing

  */
void escrever_temperatura(float temperature){

  unsigned char data=highByte(int(temperature*10));  // separates the most significant byte of the unsigned char temperature
  unsigned int write_address=datapointer*2; // sets the write address to twice the pointer of the last occupied position.
  escrever(write_address,data); // writes in memory the most significant byte
  _delay_ms(5); // waits 5ms for the writing process to be completed
  data=lowByte(int(temperature*10)); // isolates the least significant byte of the unsigned char temperature
  write_address++; // sets the address for the next memory position in order to store the least significant byte
  escrever(write_address,data); // writes in memory the least significant byte
  _delay_ms(5); // waits 5ms for the writing process to be completed

  // store datapointer
  datapointer++; // increments pointer of last address occupied to avoid overwriting in the next writing
  escrever(2046,highByte(datapointer)); // writes in address 2046 the most significant byte of the pointer
  _delay_ms(5); // waits for writing to be completed
  escrever(2047,lowByte(datapointer)); // writes the least significant byte of the pointer in the consecutive address (2047)
  _delay_ms(5); // waits for writing to be completed
}

/**   Temperature reading function
  *   Input: address to be read
  *   Output: temperature stored in the specified address
  *   The function below performs the treatment of reading the bytes corresponding to the measured value
  *   of the temperature in memory, manipulating them and reading in the correct addresses*/

float ler_temperatura(unsigned int data_pointer){
  unsigned int read_address=data_pointer*2; // sets address for reading
  unsigned char read1 = ler(read_address); // reads from memory in the specified address (most significant byte)
  unsigned char value_read=0; // variable to store read value
  read_address++; // increments address to read the least significant byte
  unsigned char read2 = ler(read_address); // reads least significant byte
  word read3 = word(read1,read2); // joins the bytes in the correct order
  float temperature_memory = (float) read3/10.0; // calculates the temperature stored
  return temperature_memory; // returns the read temperature from memory
}
/*
 * Function to write to EEPROM using datasheet indications
 * Input: address and data to be written
 * Output: —
 The variable add corresponds to the address to be written in memory,
 and according to the manual of the 24C16, we have 11 bits of addressing,
 so the variable must be of type unsigned int (up to 16 bits), allowing the correct storage of the variable.
 As for the variable data, corresponding to the data to be stored in memory,
 we have that the data of the memory are of 8 bits, this tells us that data can be of type unsigned char (up to 8 bits) without problems of storage, allowing its use.
*/
void escrever(unsigned int add, unsigned char data) {

  // Section to isolate the address of memory following datasheet indications
  unsigned int address = add & (0b0000011100000000);
  address = (address >> 8) | (static_address << 3);
  char address_send = (char) address;

  // start of transmission for the address obtained in the previous step
  Wire.beginTransmission(address_send);

   // obtaining the second address to be sent
  unsigned int address2 = add & (0b000000001111111111);

  // writes second address obtained earlier
  Wire.write((char) address2);
  Wire.write(data); // sends data to be written to memory
  Wire.endTransmission(); // finalizes transmission
}

/*
 * Function to read EEPROM using datasheet indications
 * Input: address to be read
 * Output: read value

 In this function only the address to be read is passed, and the value stored in the
 address provided is the output. The discussion about the variable add is the same as the escrever function. As for the output, since
 it is the stored data (8 bits), it has the type unsigned char.
*/

 unsigned char ler(unsigned int add) {
  // Section to isolate the address of memory following datasheet indications
  unsigned int address = add & (0b0000011100000000);
  address = (address >> 8) | (static_address << 3);
  char address_send = (char) address;

  // start of transmission for the address obtained in the previous step
  Wire.beginTransmission(address_send);

  // obtaining the second address to be sent
  unsigned int address2 = add & (0b000000001111111111);

  // writes second address obtained
  Wire.write(address2); // Writes second part of address
  Wire.endTransmission(); // Ends transmission
  Wire.requestFrom(address_send, 1); // Requests 1 byte from memory

  // Checks if there is available data
  while(Wire.available()) {
  char c = Wire.read(); // Receives byte
  return c; // returns read byte
  }
}
/*******************************************************************************************
 *                                 Serial Transfer
 *******************************************************************************************/

 /* The following function is responsible for transferring via serial the N measurements requested by the user
  * Input: Number of measurements requested
  The function through a for loop reads in consecutive memory addresses the N measurements necessary and transmits them via Serial.
  */
void transfereSerial(unsigned int N_measurements){
  float temperature_serial;
     for (unsigned int i = 0; i < N_measurements; i++){
       temperature_serial = ler_temperatura(i);
       Serial.println(temperature_serial);
     }
}
