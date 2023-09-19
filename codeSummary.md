Program Code:
Note that the program code differs from typical Arduino programming in that it utilizes Free-RTOS tasks to make use of
the two available cores on the ESP32-Pico. Free-RTOS is a real-time operating system kernel provided as standard with
Arduino libraries. Using task pinning, functions are able to be assigned to a specific core and will run indefinitely on
that core until the task is cancelled, similar to the typical program loop, which by default only runs on core #1. This
multi-core threading allows grouping of functions that operate across like sensors, such as those using the I2C bus, or
those using the SPI bus. In this specific case, the function `sensorRead` is pinned to core #0, while the function 
`sdWrite` is pinned to core #1. These functions were written such that all I2C bus devices are handled by core #0, while
SPI and analog pin states are handled by core #1. In short, the accelerometer readings and OLED display updates are
handled by one core, and all SD card writes as well as button state and LED pin state are handled by the other. This
breaks up the tasks handled by the microcontroller into two parallel loops that run indefinitely.

Some conflicts can occur with multi-core threading, so care was taken to ensure that no data is written to the same
location by both cores simultaneously. In this case, core #0 only reads accelerometer data, and writes temporary data to
the `fauxList` class where the most recent 10 accelerometer difference samples are stored. Core #1 reads and averages
this data before writing it out to the SD card. Because the accelerometer readings are performed over I2C, and the SD
card writes are performed over SPI, there is no conflict between the two processes.

Both cores are beholden to a state machine with four distinct states. When the microcontroller is powered on,
peripherals are initialized and both threaded tasks are started, each with their own initialization routines before
entering their primary loops. As part of this initialization, `loggingState` (an integer) is set to 0. In this state the
acclerometers and pulse-oximeter sensors are read, averaged, and reported to the OLED display with some graphical
elements. When the button is pressed, the SD card is started and `loggingState` is set to 3, where:
- A correction factor is calculated from the current accelerometer difference to be applied to all data during logging.
- The OLED is set to display a static warning and the correction factor being applied.
- A filename name string is created and formatted with a number that is incremented for each logging session.
- A file with the previous created filename is opened for writing on the SD card.
- Column headers are printed to the file.
- `loggingState` is decremented to 2.
After entering the next state, core #0 will only make sensor reads, and it will do so as quickly as possible. Core #1
will print a character buffer of formatted data at an interval controlled by a flag set by a hardware timer which runs
from `ICACHE RAM`. This interval is able to be specified by setting the `DATAFREQ` define, which is the rate (in Hertz)
that the flag is set. During this state, the OLED is static to allow the sensor reading to happen at maximum speed. When
the button is pressed a second time, `loggingState` is decremented to 1.
In `loggingState` 1, the file is closed, the LED status is turned on, and the correction factor is disabled. After two
seconds, the SD card is unmounted for removal, the LED is turned off, and `loggingState` is decremented to 0 to return
to the initial state.
