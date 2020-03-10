#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Metro.h>

/******DEFINES*******/




//button pins
int button1Pin = 9;
int button2Pin = 10;
int button3Pin = 11;
const int maxState = 3;

//objects
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> CAN;
uint8_t state = 0;                                      //basic state machine state
uint8_t disableWithZeros[] = {0, 0, 0, 0, 0, 0, 0, 0};  //The message to disable the controller/cancel lockout
uint8_t enableNoTorque[] = {0, 0, 0, 0, 0, 1, 0, 0};    //The message to enable the motor with zero torque
uint8_t enableSmallTorque[] = {0, 0, 0, 0, 0, 1, 0, 0}; //The message to enable the motor with small torque

/*
 * CAN ID definitions
 */
#define ID_MC_TEMPERATURES_1 0xA0
#define ID_MC_TEMPERATURES_2 0xA1
#define ID_MC_TEMPERATURES_3 0xA2
#define ID_MC_ANALOG_INPUTS_VOLTAGES 0xA3
#define ID_MC_DIGITAL_INPUT_STATUS 0xA4
#define ID_MC_MOTOR_POSITION_INFORMATION 0xA5
#define ID_MC_CURRENT_INFORMATION 0xA6
#define ID_MC_VOLTAGE_INFORMATION 0xA7
#define ID_MC_FLUX_INFORMATION 0xA8
#define ID_MC_INTERNAL_VOLTAGES 0xA9
#define ID_MC_INTERNAL_STATES 0xAA
#define ID_MC_FAULT_CODES 0xAB
#define ID_MC_TORQUE_TIMER_INFORMATION 0xAC
#define ID_MC_MODULATION_INDEX_FLUX_WEAKENING_OUTPUT_INFORMATION 0xAD
#define ID_MC_FIRMWARE_INFORMATION 0xAE
#define ID_MC_DIAGNOSTIC_DATA 0xAF
#define ID_MC_COMMAND_MESSAGE 0xC0
#define ID_MC_READ_WRITE_PARAMETER_COMMAND 0xC1
#define ID_MC_READ_WRITE_PARAMETER_RESPONSE 0xC2

/*****PROTOTYPES*****/
void writeControldisableWithZeros();
void writeEnableNoTorque();
void writeEnableSmallTorque();
void clearErrors();
void doStartup();
void readBroadcast();
void blinkLED();
void getButtons();

void setup(void)
{
    Serial.begin(115200);
    pinMode(button1Pin, INPUT_PULLUP);
    pinMode(button2Pin, INPUT_PULLUP);
    pinMode(button3Pin, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    CAN.begin();
    CAN.setBaudRate(250000);
    CAN.mailboxStatus();
}

void loop()
{
    getButtons();
    //do state machine
    switch (state)
    {
    case 0:
        writeControldisableWithZeros();
        break;
    case 1:
        doStartup();
        state = 2;
        break;
    case 2:
        writeEnableSmallTorque();
        break;
    default:
        break;
    }
    //end doing state machine
    readBroadcast();
}

void readBroadcast()
{
    CAN_message_t rxMsg;
    if (CAN.read(rxMsg))
    {
        Serial.print("  ID: 0x");
        Serial.print(rxMsg.id, HEX);
        Serial.print(" DATA: ");
        for (uint8_t i = 0; i < 8; i++)
        {
            Serial.print(rxMsg.buf[i]);
            Serial.print(" ");
        }
        if (rxMsg.id == ID_MC_INTERNAL_STATES)
        {
            Serial.print("<< Internal States");
        }
        if (rxMsg.id == ID_MC_FAULT_CODES)
        {
            Serial.print("<< Fault Codes");
        }
        Serial.println("");
    }
    /*
    if (rxMsg.id == ID_MC_READ_WRITE_PARAMETER_RESPONSE)
    {
        if (1)
        {
            Serial.println("*********Faults successfully cleared*********");
        }
    }
    */
}

void blinkLED()
{
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(30);
    digitalWrite(LED_BUILTIN, LOW);
    delay(30);
}

void writeControldisableWithZeros()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; //OUR CONTROLLER
    memcpy(ctrlMsg.buf, disableWithZeros, sizeof(ctrlMsg.buf));
    if (CAN.write(ctrlMsg) > 0)
    {
        Serial.println("****DISABLE****");
        blinkLED();
    }
    else
    {
        Serial.print("NO CAN state: ");
        Serial.println(state);
        delay(50);
    }
}

void doStartup()
{
    /* EXPLANATION from CAN manual section "2.2.2 CAN Message Sequence Example"
    0. GA tech says you need to write torque once before disable idk
    1. write control message to disableWithZeros
    2. byte 4,5 = 1 with a torque value in 0,1
    3. change the torque a bunch
    4. disable inverter by writing byte 5 0 NO direction change
    */
    writeEnableNoTorque();
    writeControldisableWithZeros();
    writeEnableNoTorque();
    //delay(500);
}

void writeEnableNoTorque()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; //OUR CONTROLLER
    memcpy(ctrlMsg.buf, enableNoTorque, sizeof(ctrlMsg.buf));
    CAN.write(ctrlMsg);
    Serial.println("----ENABLE----");
    blinkLED();
}

void writeEnableSmallTorque()
{
    CAN_message_t ctrlMsg;
    ctrlMsg.len = 8;
    ctrlMsg.id = 0xC0; //OUR CONTROLLER
    memcpy(ctrlMsg.buf, enableSmallTorque, sizeof(ctrlMsg.buf));
    CAN.write(ctrlMsg);
    Serial.println("Small Torque");
    blinkLED();
}

void getButtons()
{
    if (digitalRead(button1Pin) == LOW)
    {
        state = 0;
        Serial.println("Button to state 0");
        delay(100);
    }
    else if (digitalRead(button2Pin) == LOW)
    {
        state = 1;
        Serial.println("Button to state 1");
        delay(100);
    }
    else if (digitalRead(button3Pin) == LOW)
    {
        while (digitalRead(button3Pin) == LOW)
            ;
        state++;
        if (state > maxState)
        {
            state = 0;
        }
        Serial.print("Button to state ");
        Serial.print(state);
        Serial.println(".");
        delay(100);
    }
}
