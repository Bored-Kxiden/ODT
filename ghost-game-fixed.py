import machine
from neopixel import NeoPixel
import time
import random
from machine import Pin, PWM, ADC
import _thread

# Define hardware pins
SERVO_PIN = 12
LIGHT_SENSOR_1 = 34
LIGHT_SENSOR_2 = 35
BUZZER_PIN = 25
NEOPIXEL_RING_PIN = 26
NEOPIXEL_STRIP_PIN = 27

# Game settings
NUM_HEALTH_LEDS = 16   # Number of LEDs in health bar
NUM_TIMER_LEDS = 10    # Number of LEDs in timer bar
GAME_DURATION = 160    # Game time in seconds
LIGHT_THRESHOLD = 500  # Lowered threshold for laser detection

# Initialize components
servo = PWM(Pin(SERVO_PIN), freq=50)
light_sensor_1 = ADC(Pin(LIGHT_SENSOR_1))
light_sensor_1.atten(ADC.ATTN_11DB)
light_sensor_2 = ADC(Pin(LIGHT_SENSOR_2))
light_sensor_2.atten(ADC.ATTN_11DB)
buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty=0)
np_ring = NeoPixel(Pin(NEOPIXEL_RING_PIN), NUM_HEALTH_LEDS)
np_strip = NeoPixel(Pin(NEOPIXEL_STRIP_PIN), NUM_TIMER_LEDS)

# Global game state flag
game_active = True

def move_servo():
    """Moves the servo in a smooth loop to simulate ghost movement."""
    global game_active
    movements = [30, 90, 150, 90, 30]  # Predefined movement sequence
    while game_active:
        for angle in movements:
            duty = int((angle / 180) * 100) + 25  # Adjusted for accuracy
            servo.duty(duty)
            time.sleep(1)

def update_health():
    """Initializes the NeoPixel ring to full health (all LEDs ON)."""
    for i in range(NUM_HEALTH_LEDS):
        np_ring[i] = (0, 0, 0)  # Green LEDs
    np_ring.write()

def update_timer(remaining_time):
    """Updates the LED strip to reflect remaining time."""
    segment_time = GAME_DURATION / NUM_TIMER_LEDS
    for i in range(NUM_TIMER_LEDS):
        if remaining_time > (i + 1) * segment_time:
            np_strip[i] = (0, 255, 0)  # Green
        elif remaining_time > (i + 1) * segment_time * 0.75:
            np_strip[i] = (255, 255, 0)  # Yellow
        elif remaining_time > (i + 1) * segment_time * 0.5:
            np_strip[i] = (255, 165, 0)  # Orange
        else:
            np_strip[i] = (255, 0, 0)  # Red
    np_strip.write()

def play_buzzer():
    """Plays a ghost dying sound effect."""
    zombie_sound = [(100, 0.1), (80, 0.2), (60, 0.15), (50, 0.2), (40, 0.25)]
    for freq, duration in zombie_sound:
        buzzer.freq(freq)
        buzzer.duty(512)
        time.sleep(duration)
    buzzer.duty(0)
    time.sleep(0.1)  # Small pause after sound

def reset_game():
    """Resets the game state and restarts."""
    global game_active
    game_active = False  # Stop the servo movement thread
    time.sleep(1)  # Allow thread to exit
    game_active = True  # Reactivate for next run
    _thread.start_new_thread(move_servo, ())  # Restart servo thread
    game_loop()

def game_loop():
    """Main game loop."""
    update_health()  # Initialize health bar
    start_time = time.time()
    led_index = 0  # Start from the first LED

    while time.time() - start_time < GAME_DURATION:
        remaining_time = GAME_DURATION - (time.time() - start_time)
        update_timer(remaining_time)  # Update timer LED strip

        # Read light sensor values
        light1 = light_sensor_1.read()
        light2 = light_sensor_2.read()

        # Debug print to help calibrate threshold
        print("Light sensors:", light1, light2)

        # Check for light detection and turn off LEDs one by one
        if led_index < NUM_HEALTH_LEDS:
            # Changed condition: Turn off LED when light is detected (above threshold)
            if light1 < LIGHT_THRESHOLD or light2 < LIGHT_THRESHOLD:
                np_ring[led_index] = (0, 100, 255)  # Turn off the LED
                np_ring.write()
                play_buzzer()
                led_index += 1  # Move to the next LED

        time.sleep(0.1)  # Reduced sleep time for better responsiveness

    # End game sequence: Blink all LEDs before resetting
    for _ in range(5):
        np_strip.fill((255, 255, 255))
        np_ring.fill((255, 255, 255))
        np_strip.write()
        np_ring.write()
        time.sleep(0.5)
        np_strip.fill((0, 0, 0))
        np_ring.fill((0, 0, 0))
        np_strip.write()
        np_ring.write()
        time.sleep(0.5)

    reset_game()  # Restart game properly

# Start the servo movement and game loop
_thread.start_new_thread(move_servo, ())
game_loop()
