# dili-bot
Dilibot project repository

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

/* Work log updating */
[March 09] Initialization commit to repository
