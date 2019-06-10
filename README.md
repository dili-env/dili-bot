# dili-bot
Dilibot project repository

/*******************************************/
Dilibot is implemented EKF and work fine now.
Back up project June 10, 2019
/*******************************************/

/*Information of current project*/
This project intent to create a Dili-Bot (Ballbot software) deploy on STM32F407-Discovery board.

Up to now the configuration is basic completed.
Hardware consist of:
    - Three timer (3x2 pins) for encoder reading
    - Three timer (3 pins) for PWM generate
    - One uart port open for IMU reading
    - Three ADC (3 pins) for current sense feedback (not test yet)

IMU problem:
    The GY951 IMU sensor provide UART interface with baudrate 57600
    Some command we need to know to communicate with sensor:
        + #o0       -> Disable streaming
        + #o1       -> Enable streaming
        + #s<xy>    -> Sync message with id = <xy>, sensor will feedback '#SYNCH<xy>\r\n' in normal case
        + #ob       -> Set binary output (12 bytes for each frame = 3 x 4 float value)
        + #ot       -> Set text output   (Message template: '#YRP:[r],[p],[y]\r\n')
    Current measurement of the GY951 frequency is 50Hz (20ms per each message output)

PWM problem:
    Current configuration of PWM = 16 kHz for motor control
    Maximum frequence allowed by VNH driver is 20 kHz

Encoder problem:
    Number of counting up value of one revolution is not test at the current time (TODO:)

Driver configuration interface update Mar.18:
    |---------------|---------------|---------------|
    |PWM1   : PA15  |INA1   : PD10  |M1_CC1 : PE11  |
    |PWM2   : PB3   |INB1   : PD8   |M1_CC2 : PE9   |
    |PWM3   : PB10  |---------------|---------------|
    |---------------|INA2   : PC0   |M2_CC1 : PB7   |
    |---------------|INB2   : PC2   |M2_CC2 : PB6   |
    |EN1    : PD13  |---------------|---------------|
    |EN2    : PD14  |INA3   : PA0   |M3_CC1 : PC7   |
    |EN3    : PD15  |INB3   : PA4   |M3_CC2 : PC6   |
    |---------------|---------------|---------------|
    *Note:  PWM interface               : 5V
            ENx interface               : 5V
            INAx/INBx interface         : 5V
            Encoder Mx_CCy interface    : 3V3

/* Work log updating */
[March 18] Update API for hardware configuration base board rev.b
[March 09] Initialization commit to repository
