import network
import socket
from machine import Pin, PWM, ADC
import neopixel
import time
import uasyncio as asyncio

# Wi-Fi setup
wifi_ssid = 'Your_SSID'
wifi_password = 'Your_PASSWORD'

# Connect to Wi-Fi
wifi = network.WLAN(network.STA_IF)
wifi.active(True)
wifi.connect(wifi_ssid, wifi_password)

# Wait for connection
while not wifi.isconnected():
    time.sleep(0.1)
print('Connected to Wi-Fi:', wifi.ifconfig())

# Motor driver pins for two motors
IN1 = Pin(2, Pin.OUT)  # Motor 1 IN1
IN2 = Pin(4, Pin.OUT)  # Motor 1 IN2
EN1 = PWM(Pin(13), freq=1000)  # Motor 1 PWM
IN3 = Pin(14, Pin.OUT)  # Motor 2 IN1
IN4 = Pin(12, Pin.OUT)  # Motor 2 IN2
EN2 = PWM(Pin(27), freq=1000)  # Motor 2 PWM

# NeoPixel LED strip configuration
LED_PIN = Pin(22, Pin.OUT)
NUM_LEDS = 10
strip = neopixel.NeoPixel(LED_PIN, NUM_LEDS)

# Flex sensor pins (change as necessary)
flex_pins = [ADC(Pin(i)) for i in [34, 35, 32, 33, 36]]
for flex in flex_pins:
    flex.atten(ADC.ATTN_0DB)  # 0-1V range for flex sensor readings

# Setup UDP socket to communicate with itself (for simulation)
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.bind(('0.0.0.0', 12345))  # Listen on all interfaces and port 12345

# Function to read flex sensor values and send commands
async def read_flex_and_send():
    target_ip = '192.168.x.x'  # Replace with your local ESP32's IP address
    target_port = 12345  # The port on which the Motor Control ESP32 is listening

    while True:
        # Read the flex sensor values
        flex_values = [flex.read() for flex in flex_pins]
        print(flex_values)  # Print flex sensor values for debugging

        # If any flex sensor bends (value above a threshold), send the "START" command
        if any(value > 2000 for value in flex_values):  # Adjust threshold as needed
            command = "START"
        else:
            command = "STOP"

        # Send command to the motor control ESP32 via UDP
        udp_socket.sendto(command.encode(), (target_ip, target_port))

        await asyncio.sleep(0.1)  # Non-blocking sleep

# Motor control functions
def motor_forward(motor=1, speed=1023):
    if motor == 1:
        EN1.duty(speed)
        IN1.on()
        IN2.off()
    elif motor == 2:
        EN2.duty(speed)
        IN3.on()
        IN4.off()

def motor_stop():
    IN1.off()
    IN2.off()
    EN1.duty(0)
    IN3.off()
    IN4.off()
    EN2.duty(0)

# LED breathing effect function
async def breathing_effect():
    while True:
        # Breathing effect
        for brightness in range(0, 256, 5):  # Fade in
            for i in range(NUM_LEDS):
                hue = (i * 360 // NUM_LEDS)  # Different hue for each LED
                r, g, b = hsv_to_rgb(hue, 1, brightness / 255)
                strip[i] = (r, g, b)
            strip.write()
            await asyncio.sleep_ms(50)

        for brightness in range(255, -1, -5):  # Fade out
            for i in range(NUM_LEDS):
                hue = (i * 360 // NUM_LEDS)
                r, g, b = hsv_to_rgb(hue, 1, brightness / 255)
                strip[i] = (r, g, b)
            strip.write()
            await asyncio.sleep_ms(50)

# Function to convert HSV to RGB (for smooth hue cycling)
def hsv_to_rgb(h, s, v):
    i = int(h / 60) % 6
    f = h / 60 - i
    p = int(v * (1 - s) * 255)
    q = int(v * (1 - f * s) * 255)
    t = int(v * (1 - (1 - f) * s) * 255)
    v = int(v * 255)
    if i == 0:
        return v, t, p
    elif i == 1:
        return q, v, p
    elif i == 2:
        return p, v, t
    elif i == 3:
        return p, q, v
    elif i == 4:
        return t, p, v
    else:
        return v, p, q

# Function to listen for UDP commands and control motors
async def listen_for_udp_commands():
    while True:
        data, addr = udp_socket.recvfrom(1024)  # Receive data
        command = data.decode()

        if command == "START":
            motor_forward(motor=1, speed=1023)  # Start motor 1
            motor_forward(motor=2, speed=1023)  # Start motor 2
            print("Motors Started")
        elif command == "STOP":
            motor_stop()  # Stop motors
            print("Motors Stopped")

        await asyncio.sleep(0.1)

# Main function to run the tasks concurrently
async def main():
    # Run tasks concurrently
    task1 = asyncio.create_task(read_flex_and_send())
    task2 = asyncio.create_task(listen_for_udp_commands())
    task3 = asyncio.create_task(breathing_effect())

    await asyncio.gather(task1, task2, task3)

# Run the main function
asyncio.run(main())
