# Concussion Detector

## Background
This product is marketed to athletes who participate in contact sports or sports where concussions are frequently sustained, from the high school level to the college level. It can be used for sports that require helmet protection and for sports that do not require helmets. This product would be used on athletes, but the data would be analyzed by the athlete, a coach or a medical professional. This wearable technology allows athletes to detect and track the impact of head trauma and head motion during athletic activity. Knowing the severity of a head impact has many benefits including recognition and further injury prevention. This device allows athletes to know if a head impact could be concussive, and informs coaches and medical staff that they need to be removed from the game, preventing injury. It also allows them to track their “baseline” (non-contact) athletic performance, so medical staff and potential researchers can compare head trauma with typical athletic movements and determine the severity of the trauma.

## Approach
This approach contains two modules: the concussion sensor, which tracks the acceleration of the head and transmits the data, and the display module, which receives the acceleration data and alerts the user of potential head trauma. The concussion sensor is positioned behind one of the subject’s ears using an adhesive. It contains an IMU, an Xbee, a Nucleo and a battery. The IMU measures the acceleration and absolute orientation of the head of the user and uses the Xbee to transmit the data from the sensor to a second Xbee within the display module of the user. Within the display module, the receiving Xbee transmits the acceleration data to the second Nucleo, which processes the data and determines if the acceleration meets the threshold of 65.1 g. If the threshold is reached, a warning of the concussion data is displayed on a LCD display in the display module to alert the user.
### Head-mounted IMU sensor

<img src="https://github.com/dhipatel21/Concussion-Detector/blob/d0866d62d8efe2876bd440115b1fc58cb325f082/sensor_module.png" alt="drawing" width="400"/>
Connect IMU; gather acceleration data measurements and transmit through Xbee
The head mounted sensor is responsible for collecting acceleration and orientation data (through the IMU) and transmitting the data wirelessly to the display unit via an Xbee pair. The Bosch BNO055 serves as our IMU for this purpose, which we selected because the sensor does its own data filtering and noise reduction, as well as calculates linear acceleration. We use the absolute orientation and linear acceleration data to determine if the concussion threshold has been reached. This raw data is transmitted to the display module, where data is processed to see if the threshold has been met. The head-mounted sensor is attached behind the right ear of the user by KT tape, a brand of reliable and waterproof athletic tape.

#### Software

The Nucleo uses constant polling and a timer-based interrupt to determine peaks in acceleration. The IMU constantly measures acceleration, and the Nucleo uses a timer to trigger an interrupt every two seconds. The Nucleo receives data from the Xbee using I2C. When the interrupt is triggered, the maximum acceleration across those two seconds is recorded and is transmitted to the Xbee using UART. The Xbee then transmits this determined peak acceleration to the Xbee in the display module.

### Display Module: Signal Processing and Delay Unit

<img src="https://github.com/dhipatel21/Concussion-Detector/blob/d0866d62d8efe2876bd440115b1fc58cb325f082/display_module.png" alt="drawing" width="400"/>
The display module’s signal processing and delay unit has three major components: the Xbee to receive the data from the head-mounted sensor, the Nucleo to process the data received on the Xbee, and the LCD display which shows warnings of concussions as well as reminding the user of the calibration sequence. The Nucleo processes the data by processing the raw data from the IMU and measuring if a certain threshold of concussive acceleration has been reached (65.1g in our case). If the threshold is reached, the Nucleo triggers a display on the LCD screen to visually warn the user, as well as triggers the Piezo Buzzer to audibly warn the user.
 
#### Software
The display module receives the peak of the acceleration via the Xbees every two seconds. The Xbee on the display module transmits the raw data to the Nucleo using UART. The Nucleo then converts the raw data into human-readable acceleration data by dividing each of the axis values by 100, taking the magnitude, and converting the magnitude into units of g. This acceleration value is then compared to the global maximum data that has been received in the past. If the new data is greater than the global maximum, but less than the concussion threshold, the Nucleo triggers the display to show the value of this current acceleration on the LCD screen. If the new acceleration is greater than the threshold for a concussion, the Nucleo writes to the Piezo Buzzer using GPIO to trigger it to turn on for 0.5 seconds, and then displays the “concussion detected” screen on the LCD display.

### Display
The TFT display is mounted above the display module Nucleo and alerts the user to if the concussion threshold of 65.1 g acceleration has been reached, the reported threshold for a likelihood of concussion. This processor handles all inputs/outputs to the capacitive touchscreen TFT display and more functionality may also be offered to the user depending on the time constraints of this project. The display also alerts the user to calibration requirements when the device is powered on, and lets the user know when the calibration has been completed. The touchscreen has a button to allow the user to acknowledge the alert. An auditory alert component, a Piezo Buzzer, is also connected to and controlled by the processor, below the TFT display.

#### Software

The TFT display uses a SPI protocol for the display and touchscreen. The drivers used for the display and the capacitive touch were both ported from Arduino versions found on Github. There is essentially only one function used for the capacitive touch, which is used to read ADC and sense any pressure on the screen. The display functions are used to initialize the screen to all white, clearing any previously written messages; then, new messages are written in different colors and sizes to emphasize importance. In order to display the acceleration values, the acceleration value is converted into a character buffer, which is then able to be displayed on the screen.

#### Display Screen

Upon power-up, the TFT Screen displays a calibration screen. The calibration screen displays the four components of the IMU that need to be calibrated: the gyroscope, the magnetometer, the accelerometer and the orientation. Each component displays a red status symbol until they are properly calibrated, at which time the status symbol turns green. When they are all green, the screen clears and shows a new screen that says “Touch anywhere to Start”. When the display is tapped by the user, a pink “running...” message appears along the bottom and the data collection and processing begins.
The “Touch anywhere to Start, running...” message clears, and is replaced by a message that says “Your Highest Acceleration So Far Is 0 g” in green type. As new maximum accelerations are recorded, this screen is updated to reflect the current maximum acceleration, as long as that maximum acceleration is below the concussion threshold of 65.1 g. If the new maximum acceleration is greater than or equal to 65.1 g, the screen is cleared and replaced by a message in red type: “Concussion Detected. Your Acceleration is [value] g.” The user can acknowledge the message by touching the screen, which takes you back to the “Touch to Start” screen.

### Wireless Communications
The wireless system must work in building and be able to work for 3 hours with a somewhat small sized battery. The main use is to transfer the raw acceleration data from the Nucleo on the concussion sensor module to the Nucleo on the display module, which processes the raw data before displaying alerts on the display module. We are not concerned about range for the bluetooth, as the distance between Xbees isbe from the back of the user’s ear to their display. Xbee modules configured for serial communication meet these needs.

### Essential System Components
The following components are necessary to implement an IMU based concussion sensor connected with a watch module to display relevant data and alerts.
1. Processor  
  a. Nucleo L412KB for concussion sensor module  
  b. Nucleo L4R5ZI-P for display module
2. Concussion Sensor  
  a. Bosch BNO055 IMU  
  b. 2 x Digi XBee 3 Cellular LTE-M/NB-IoT modem
3. Watch Display  
  a. TFT Display with Capacitive Touchscreen  
  b. Piezo Buzzer PS1240
4. Power and Batteries  
  a. 2 x 3.7V rechargeable battery  
  b. TC 1262 3.3 volt regulator
5. Enclosures and Adhesives  
  a. KT Athletic Tape  
  b. Mini pod
