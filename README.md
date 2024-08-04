# Communicating Raspberry Pi 4 with Distance Sensor and LCD Display

## Authors

- TA: Le Phan Nguyen Dat
- Instructor: MEng. Truong Quang Phuc 

## Hardware
- Raspberry Pi 4 Model B
- HC-SR04 Ultrasonic Sensor
- 16x2 LCD with I2C module

## Connections

### HC-SR04 Ultrasonic Sensor
| HC-SR04 Pin | Raspberry Pi Pin |
|-------------|-------------------|
| VCC         | 5V (Pin 2)        |
| GND         | GND (Pin 6)       |
| TRIG        | GPIO 18 (Pin 12)  |
| ECHO        | GPIO 24 (Pin 18)  |

### 16x2 LCD with I2C Module
| I2C Module Pin | Raspberry Pi Pin        |
|----------------|--------------------------|
| VCC            | 5V (Pin 4)               |
| GND            | GND (Pin 9)              |
| SDA            | GPIO 2 (SDA, Pin 3)      |
| SCL            | GPIO 3 (SCL, Pin 5)      |

## Enable I2C Bus
1. Open the Raspberry Pi configuration tool:
    ```bash
    sudo raspi-config
    ```

2. Navigate to `Interfacing Options` using the arrow keys and press `Enter`.

3. Select `I2C` and press `Enter`.

4. When asked to enable I2C, select `Yes` and press `Enter`.

5. Exit the `raspi-config` tool by navigating to `Finish` and pressing `Enter`.

6. Reboot your Raspberry Pi to apply the changes:
    ```bash
    sudo reboot
    ```

## Install Python Libraries
1. Update the package lists:
    ```bash
    sudo apt-get update
    ```

2. Install the `python3-smbus` and `i2c-tools` packages:
    ```bash
    sudo apt-get install -y python3-smbus i2c-tools
    ```

## Create and Run Code
1. Create a Python file using SCP or nano:
    ```bash
    nano <file_name>.py
    ```

2. Copy the following code into the file:
    ```python
    import RPi.GPIO as GPIO
    import time
    import smbus
    import sys

    # GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)

    # Set GPIO Pins
    GPIO_TRIGGER = 18
    GPIO_ECHO = 24

    # Set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

    # I2C parameters
    I2C_ADDR = 0x27  # I2C device address, if any error, check your device address
    LCD_WIDTH = 16   # Maximum characters per line

    # LCD constants
    LCD_CHR = 1  # Mode - Sending data
    LCD_CMD = 0  # Mode - Sending command
    LCD_LINE_1 = 0x80  # LCD RAM address for the 1st line
    LCD_LINE_2 = 0xC0  # LCD RAM address for the 2nd line
    LCD_BACKLIGHT = 0x08  # On

    ENABLE = 0b00000100  # Enable bit

    # Timing constants
    E_PULSE = 0.0005
    E_DELAY = 0.0005

    # Open I2C interface
    try:
        bus = smbus.SMBus(1)  # Rev 2 Pi uses 1
    except Exception as e:
        print(f"Error initializing I2C bus: {e}")
        sys.exit(1)

    def lcd_init():
        lcd_byte(0x33, LCD_CMD)  # 110011 Initialise
        lcd_byte(0x32, LCD_CMD)  # 110010 Initialise
        lcd_byte(0x06, LCD_CMD)  # 000110 Cursor move direction
        lcd_byte(0x0C, LCD_CMD)  # 001100 Display On,Cursor Off, Blink Off
        lcd_byte(0x28, LCD_CMD)  # 101000 Data length, number of lines, font size
        lcd_byte(0x01, LCD_CMD)  # 000001 Clear display
        time.sleep(E_DELAY)

    def lcd_byte(bits, mode):
        bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
        bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT

        # High bits
        bus.write_byte(I2C_ADDR, bits_high)
        lcd_toggle_enable(bits_high)

        # Low bits
        bus.write_byte(I2C_ADDR, bits_low)
        lcd_toggle_enable(bits_low)

    def lcd_toggle_enable(bits):
        time.sleep(E_DELAY)
        bus.write_byte(I2C_ADDR, (bits | ENABLE))
        time.sleep(E_PULSE)
        bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
        time.sleep(E_DELAY)

    def lcd_string(message, line):
        message = message.ljust(LCD_WIDTH, " ")

        lcd_byte(line, LCD_CMD)

        for i in range(LCD_WIDTH):
            lcd_byte(ord(message[i]), LCD_CHR)

    def lcd_clear():
        lcd_byte(0x01, LCD_CMD)

    def distance():
        # Set Trigger to HIGH
        GPIO.output(GPIO_TRIGGER, True)

        # Set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(GPIO_TRIGGER, False)

        StartTime = time.time()
        StopTime = time.time()

        # Save StartTime
        while GPIO.input(GPIO_ECHO) == 0:
            StartTime = time.time()

        # Save time of arrival
        while GPIO.input(GPIO_ECHO) == 1:
            StopTime = time.time()

        # Time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # Multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2

        return distance

    if __name__ == '__main__':
        try:
            lcd_init()
            while True:
                dist = distance()
                print(f"Measured Distance = {dist:.1f} cm")
                lcd_string(f"Distance:{dist:.1f}cm", LCD_LINE_1)
                time.sleep(1)

        except KeyboardInterrupt:
            print("Measurement stopped by User")
        except Exception as e:
            print(f"An error occurred: {e}")
        finally:
            lcd_clear()
            GPIO.cleanup()
    ```

3. Save and exit the file by pressing `Ctrl+X`, then `Y`, and `Enter`.

4. Make the file executable (optional but recommended):
    ```bash
    chmod +x <file_name>.py
    ```

5. Run the LCD program:
    ```bash
    sudo python3 <file_name>.py
    ```
