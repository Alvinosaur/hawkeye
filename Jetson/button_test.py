import Jetson.GPIO as GPIO
import time

#Pin definitions
but2_pin = 11
but_pin = 12

def stop():
    print("stop")

def start():
    print("start")

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(but2_pin, GPIO.IN)
    GPIO.setup(but_pin, GPIO.IN)

    GPIO.add_event_detect(but_pin, GPIO.FALLING, callback=stop, bouncetime=10)
    #GPIO.add_event_detect(but_pin, GPIO.RISING, callback=start, bouncetime=10)
    print("starting program")
    try:
        while True:
            val = GPIO.input(but_pin)
            print(val)
    finally:
        GPIO.cleanup()

    #try
    #    while True:
    #        curr_value = GPIO.input(led_pin)

if __name__ == '__main__':
    main()

