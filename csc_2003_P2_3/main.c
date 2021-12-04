/******************************************************************************
 * MSP DriverLib CAR Project
 *
 *                MSP432P401
 *             ------------------
 *            |               1.0|------>Red LED
 *            |               1.1|<------Test Functions S1
 *            |               1.4|<------STOP/START wheels Function S2
 *            |               1.6|<------Right Wheel Encoder
 *            |               1.7|<------Left Wheel Encoder
 *            |               2.0|------>Red LED
 *            |               2.1|------>Green LED
 *            |               2.2|------>Blue LED
 *            |               2.4|------>Left Wheel PWM(ENB)
 *            |               2.5|------>ULTRASONIC TRIG
 *            |               2.6|------>Right Wheel PWM(ENA)
 *            |               3.0|<------ULTRASONIC ECHO
 *            |               5.0|------>Right Wheel Direction (IN1)
 *            |               5.1|------>Right Wheel Direction (IN2)
 *            |               5.6|------>Left Wheel Direction (IN4)
 *            |               5.7|------>Left Wheel Direction (IN3)
 *            |                  |
 *            |              UART|<----->RasPi(BaudRate:9600, DataSize:8, Parity:None, StopBits:1) 
 * Author:
*******************************************************************************/


/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
//
//
//////////////////////////////////////////Global Variables////////////////////////////////////////////////

//Macro functions to find minimum and maximum number
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

//Start pulse for ultrasonic sensor
bool start_pulse=0;
//Distance of obstacle from ultraSonicSensor
float dist;

//Ultrasonic sensor distance(in cm) to vary motor's PWM
uint32_t minDistFromObject=100;

//Counter for wheel encoder
volatile uint32_t leftCounter =0;
volatile uint32_t rightCounter =0;

//max count for wheel encoder to turn
uint32_t rightTurnNotchMaxCount=10;
uint32_t leftTurnNotchMaxCount=35;

//Turning operation for Wheel encoder(0 = turning / 1 = stop turning)
bool turnLeft=0;
bool turnRight=0;

//Car Operation L = Left, R = Right, F = Forward, X = Stop
uint32_t carOperation = 'N';
uint32_t carOperationNew = 'F';

//PID Settings
int TARGET = 9;
float KP = 0.14;
float KD = 0.07;
float KI = 0.035;
volatile int pidLeftCounter =0;
volatile int pidRightCounter =0;
float leftSpeed = 1.0;
float rightSpeed = 1.0;
int leftError = 0;
int rightError = 0;
int leftPrevError = 0;
int rightPrevError = 0;
int leftSumError = 0;
int rightSumError = 0;
int pidFlag = 1;

//Variables to check when wheels are on the same speed
int pidRCheck = 0;
int pidLCheck = 0;


////////////////////////////////////////TIMERA0 CONFIG (FOR MOTORS) ////////////////////////////////////////////////

const Timer_A_UpModeConfig upConfigA0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,        // 1/(3M/64) = 21.33us per tick
        46875,                                 // tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                       // Clear value
};


/* Configuring P2.4(TA0.1) for Left Motor PWM */
Timer_A_PWMConfig pwmConfigLeftMotor =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_1,
        TIMER_A_OUTPUTMODE_RESET_SET,
        10000
};

/* Configuring P2.6(TA0.3) for Right Motor PWM */
Timer_A_PWMConfig pwmConfigRightMotor =
{
        TIMER_A_CLOCKSOURCE_SMCLK,
        TIMER_A_CLOCKSOURCE_DIVIDER_12,
        10000,
        TIMER_A_CAPTURECOMPARE_REGISTER_3,
        TIMER_A_OUTPUTMODE_RESET_SET,
        10000
};

////////////////////////////////////////TIMERA1 CONFIG (FOR PID) ////////////////////////////////////////////////

const Timer_A_UpModeConfig upConfigA1 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,          // 1/(3M/64) = 21.33us per tick
        46875,                                  // tick period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

///////////////////////////////////////////TIMER FOR ULTRASONIC SENSOR//////////////////////////////////

const Timer_A_ContinuousModeConfig contmConfig =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_64,          // SMCLK/1 = 3MHz
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

const Timer_A_CompareModeConfig CCR4Config =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_4,
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
        TIMER_A_OUTPUTMODE_SET_RESET,
        0x4000//0xFFFE for very slow
};

/////////////////////////////////////////////Motor Library////////////////////////////////////////////////
void startWheels(void)
{
    //PORT 5 for MOTOR OUTPUT
    GPIO_setAsOutputPin(GPIO_PORT_P5, 0xf3);//11110011
    //MOTOR PWM PINS 5.5 = left, 5.6 = right
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0x30);
    //direction
    // 5.0/5.6=1 & 5.1/5.7=0 = move foward
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, 0x41);//01000001
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, 0x82);//10000010
}
void motorLeftForward(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN7);
}
void motorLeftReverse(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
}
void motorLeftStop(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN6);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN6);
}
void motorRightForward(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
}
void motorRightReverse(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
}
void motorRightStop(){
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0);
}
void bothMotorStop(){
    motorLeftStop();
    motorRightStop();
}
void BothMotorForward()
{
    motorLeftForward();
    motorRightForward();
}
void motorTurnLeft()
{
    motorRightForward();
    motorLeftStop();
}
void motorTurnRight()
{
    motorLeftForward();
    motorRightStop();
}

/////////////////////////////////////////////Car Operation///////////////////////////////////////////////

//Car direction from UART communication
void setCarDirection(){

    //turn left or right when receiving signal from PI
    if(carOperationNew =='L' && turnLeft == 1){
        carOperation = 'L';
        motorTurnLeft();
    }
    else if(carOperationNew =='R' && turnRight == 1){
        carOperation = 'R';
        motorTurnRight();
    }

    //else forward or Stop
    else{
        if (carOperationNew != carOperation){
            if(carOperationNew =='F')
            {
               carOperation = 'F';
               BothMotorForward();
            }
            else if(carOperationNew =='X')
            {
               carOperation = 'X';
               bothMotorStop();
            }
            //Default forward
            else
                {
                  BothMotorForward();
                  carOperation = 'F';
                }
        }
    }
}

//Set car speed from ultrasonic sensor
void setCarSpeed(){

    if (dist<minDistFromObject && turnLeft != 1 && turnRight !=1){   //If distance < 10cm
            pidFlag = 0;

        pwmConfigLeftMotor.dutyCycle = 0;
        pwmConfigRightMotor.dutyCycle = 0;

        Timer_A_generatePWM(TIMER_A0_BASE,
                                &pwmConfigLeftMotor);
        Timer_A_generatePWM(TIMER_A0_BASE,
                                &pwmConfigRightMotor);
    }
    else{
        pidFlag = 1;
    }

}

///////////////////////////////////////////UART COMMUNINCATION///////////////////////////////////////////////
const eUSCI_UART_ConfigV1 uartConfig =
    {
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,                // SMCLK Clock Source
        78,                                            // BRDIV = 78
        2,                                             // UCxBRF = 2
        0,                                             // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                        // ODD Parity
        EUSCI_A_UART_LSB_FIRST,                        // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,                     // One stop bit
        EUSCI_A_UART_MODE,                             // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION, // Oversampling
        EUSCI_A_UART_8_BIT_LEN                         // 8 bit data length
};


int main(void){
//
    //SETTINGS FOR ULTRASONIC SENSOR
    //Tval1= timer value start of echo, Tval2 = Timer value at end of echo
    uint32_t tval1, tval2;
    //No. of timer ticks from start of echo to end of echo
    uint32_t noOfTicksFromUltraSonic;

    /* Halting the Watchdog */
    MAP_WDT_A_holdTimer();

    // Configure P1.0 as output
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0); //on led

    // Configure P2.0-2.2 as output
    GPIO_setAsOutputPin(GPIO_PORT_P2, 0x07);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, 0x07); //off led

    /* IVM Configure P6.7 to output timer TA2.4 (secondary module function, output)*/
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P6, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    /* IVM Configure P3.0 as input */
    MAP_GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN0);

    /* Configuring Timer_A2 CCR0 and CCR4 then set for continuous Mode */
    MAP_Timer_A_initCompare(TIMER_A2_BASE, &CCR4Config);   //CCR4 (turns OUT4 on)
    MAP_Timer_A_configureContinuousMode(TIMER_A2_BASE, &contmConfig);
    MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_CONTINUOUS_MODE);

    /* Configuring P3.0 (ECHO) as an input and enable PORT3 interrupts */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3, GPIO_PIN0);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, GPIO_PIN0);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN0);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3, GPIO_PIN0, GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_Interrupt_enableInterrupt(INT_PORT3);

    /* Configuring P1.6 for wheel encoder interrupt */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN6);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN6);

    /* Configuring P1.7 for wheel encoder interrupt */
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN7);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN7,GPIO_LOW_TO_HIGH_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN7);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN7);

    /* Configuring P2.4(TA0.1) for Left Motor PWM */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeftMotor);

    /* Configuring P2.6(TA0.3) for Right Motor PWM */
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigRightMotor);

    /* Configure TimerA0 */
    Timer_A_configureUpMode (TIMER_A0_BASE , &upConfigA0 );
    Interrupt_enableInterrupt (INT_TA0_0);
    Timer_A_startCounter (TIMER_A0_BASE , TIMER_A_UP_MODE );

    /* Configure TimerA1 */
    Timer_A_configureUpMode (TIMER_A1_BASE , &upConfigA1 );
    Timer_A_clearCaptureCompareInterrupt (TIMER_A1_BASE ,
                                          TIMER_A_CAPTURECOMPARE_REGISTER_0 );
    Interrupt_enableInterrupt (INT_TA1_0);
    Timer_A_startCounter (TIMER_A1_BASE , TIMER_A_UP_MODE );
    Interrupt_enableInterrupt(INT_PORT1);

    //UART Configuration
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
    //Setting DCO to 12MHz
    CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
    //Configuring UART Module
    UART_initModule(EUSCI_A0_BASE, &uartConfig);
    //Enable UART module
    UART_enableModule(EUSCI_A0_BASE);
    //Enabling interrupts (Rx)
    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);

    Interrupt_enableMaster();
    BothMotorForward();
    while(1)
    {
        //Set car direction from UART
        setCarDirection();

        if(start_pulse==1)
        {
          //Set car speed based on distance from object
          //Dist < 100 = Speed = 0
          setCarSpeed();

          //Get timer value of start
          tval1 = MAP_Timer_A_getCounterValue(TIMER_A2_BASE);
          //busy wait for the timer input value to drop
          while((MAP_GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN0) & 0x1)==1);
          //Get timer value of end
          tval2 = MAP_Timer_A_getCounterValue(TIMER_A2_BASE);
          //Reset Pulse
          start_pulse=0;

          //If tval1 > tval2 indicating timer overflow
          if(tval1 > tval2)   //it means the 16-bit counter hit 0xffff and wrapped back to 0x0000
               tval2=tval2+0xffff;

          //Interval of ticks between start and end
          noOfTicksFromUltraSonic = tval2-tval1;

          //Distance = (SPEED OF SOUND) * ((TimerAtickDuration *noOfTicksFromUltraSonic)/2)
          dist=( 100.0 * 340.0) * ( (0.000021333 * (float)noOfTicksFromUltraSonic )*0.5)  ;

        }
    }
}

/* Sample notch count per second calculate and set PID
 * Find amount of error from TARGET
 *  */
void TA1_0_IRQHandler ( void )
{
    if (pidFlag == 1){

    leftError =  TARGET - pidLeftCounter;
    rightError = TARGET - pidRightCounter;

    leftSpeed += (leftError * KP) + (leftPrevError * KD) + (leftSumError * KI);
    rightSpeed += (rightError * KP) + (rightPrevError * KD) + (rightSumError * KI);


    leftSpeed = MAX(MIN(1.0,leftSpeed),0);
    rightSpeed = MAX(MIN(1.0,rightSpeed),0);

    pwmConfigLeftMotor.dutyCycle = 10000*leftSpeed;
    pwmConfigRightMotor.dutyCycle = 10000*rightSpeed;

    leftPrevError = leftError;
    rightPrevError = rightError;

    leftSumError += leftError;
    rightSumError += rightError;

    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigLeftMotor);
    Timer_A_generatePWM(TIMER_A0_BASE, &pwmConfigRightMotor);

    pidRightCounter = 0;
    pidLeftCounter = 0;
    }
    Timer_A_clearCaptureCompareInterrupt (TIMER_A1_BASE , TIMER_A_CAPTURECOMPARE_REGISTER_0 );
}

//Interrupt handler for PORT 1
void PORT1_IRQHandler(void)
{
    uint32_t status;
    status = GPIO_getInterruptStatus( GPIO_PORT_P1,P1IN);
    GPIO_clearInterruptFlag ( GPIO_PORT_P1, status);

    //Right Wheel Encoder Input at P1.6
    if(status & GPIO_PIN6)
    {
        //Increment Right wheel counter with each notch in wheel Encoder
        rightCounter= rightCounter+1;
        pidRightCounter = pidRightCounter + 1;
        pidRCheck += 1;

        //If right wheel encoder makes x round
        if(rightCounter==rightTurnNotchMaxCount && carOperationNew=='L'){
            carOperationNew= 'F';
            turnLeft =0;       // Reset Right wheel counter once makes x round
        }
        if(pidRCheck == 20){//Toggle LED pin
            pidRCheck = 0;
                    GPIO_toggleOutputOnPin (GPIO_PORT_P2 , GPIO_PIN0);
       }

    }

    //Left Wheel Encoder Input at P1.7
    if(status & GPIO_PIN7)
    {
        //Increment left wheel counter with each notch in wheel Encoder
        leftCounter= leftCounter+1;
        pidLeftCounter = pidLeftCounter+1;
        pidLCheck += 1;;

        //If left wheel encoder makes x round
        if(leftCounter==leftTurnNotchMaxCount && carOperationNew=='R'){
            carOperationNew= 'F';
            turnRight = 0;    // Reset left wheel counter once makes x round
        }

        if(pidLCheck == 20){
            pidLCheck = 0;
            GPIO_toggleOutputOnPin (GPIO_PORT_P1 , GPIO_PIN0);
        }
    }
}


/* GPIO ISR for PORT 3 (the echo value) ultrasonic */
void PORT3_IRQHandler (void)
{
    uint32_t status;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3, status);

    if(status & GPIO_PIN0) //if this was triggered by PIN0
        start_pulse=1;
}

/* Interrupt when message receive from RaspPI */
void EUSCIA0_IRQHandler(void)
{
    unsigned char a = 0;

    a = UART_receiveData(EUSCI_A0_BASE);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);

    switch(a){
        case 'l':
            UART_transmitData(EUSCI_A0_BASE, 'L');

            rightCounter=0;     //Reset Right Wheel Encoder

            //Toggle Left Wheel turning Status
            if(turnLeft ==0){
                turnLeft=1;
            }
            carOperationNew='L';   //Update of car operation = Left
        break;
        case 'r':
            UART_transmitData(EUSCI_A0_BASE, 'R');
            leftCounter=0;     //Reset Right Wheel Encoder

            //Toggle Right Wheel turning Status
            if(turnRight ==0){
                turnRight=1;
            }
            carOperationNew='R';   //Update of car operation = Right
        break;
        case 'w':
            UART_transmitData(EUSCI_A0_BASE, 'F');
            carOperationNew='F';   //Update of car operation = Forward
        break;
        case 's':
            UART_transmitData(EUSCI_A0_BASE, 'X');
            carOperationNew='X';   //Update of car operation = Stop
        break;
        default:
        break;
    }
}

void uPrintf(unsigned char *TxArray)
{
    unsigned short i = 0;
    while (*(TxArray + i))
    {
        UART_transmitData(EUSCI_A0_BASE, *(TxArray + i)); // Write the character at the location specified by pointer
        i++;                                              // Increment pointer to point to the next character
    }
}

