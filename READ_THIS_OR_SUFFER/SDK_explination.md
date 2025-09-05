# Python SDK Guide

This guide provides a simple explanation of the Python SDK for
controlling various functions.

## Overview

The SDK allows you to perform various tasks such as controlling LEDs,
buzzers, motors, servos, and reading sensor data via a serial port
connection with the STM32 board.

## Setup

-   **Install required packages:** You need to have Python installed,
    along with the `pyserial` library.
-   **Connect the board:** Connect your STM32 board via a serial port.
    Adjust the port settings in the code if necessary.

## Key Functions

### LED Control

Turn the LED on and off with specified intervals:

``` python
board.set_led(on_time, off_time, repeat, led_id)
```

-   `on_time`, `off_time`: duration in seconds\
-   `repeat`: number of times to repeat\
-   `led_id`: the ID of the LED to control

### Buzzer Control

Set buzzer frequency and on/off time:

``` python
board.set_buzzer(freq, on_time, off_time, repeat)
```

-   `freq`: frequency in Hz\
-   `on_time`, `off_time`: duration in seconds\
-   `repeat`: number of times to repeat

### Motor Control

Set speeds or duties for multiple motors:

``` python
board.set_motor_speed(speeds)
board.set_motor_duty(dutys)
```

-   `speeds`: list of motor ID and speed\
-   `dutys`: list of motor ID and duty cycle

### Servo Control

Position and offset for PWM servo:

``` python
board.pwm_servo_set_position(duration, positions)
board.pwm_servo_set_offset(servo_id, offset)
```

-   `duration`: how long the position should be held\
-   `positions`: list of servo ID and position\
-   `offset`: offset for the servos

### OLED Display

Set text on the OLED display:

``` python
board.set_oled_text(line, text)
```

-   `line`: OLED line number (1 or 2)\
-   `text`: text to display

### RGB Control

Set RGB color for LEDs:

``` python
board.set_rgb(pixels)
```

-   `pixels`: list of LED index and RGB values

## Reading Data

### Battery Status

``` python
battery_status = board.get_battery()
```

### Button Status

``` python
button_status = board.get_button()
```

### IMU Data

``` python
imu_data = board.get_imu()
```

### Gamepad Input

``` python
axes, buttons = board.get_gamepad()
```

### SBUS Input

``` python
sbus_data = board.get_sbus()
```

### Voice Recognition Results

``` python
asr_data = board.get_asr()
```

## Example

To start using the board, instantiate the Board class and enable
reception:

``` python
board = Board()
board.enable_reception(True)
```

To control a buzzer:

``` python
board.set_buzzer(2400, 0.1, 0.9, 1)
```

To read the battery status in a loop:

``` python
while True:
    try:
        res = board.get_battery()
        if res is not None:
            print(res)
        time.sleep(0.1)
    except KeyboardInterrupt:
        break
```
