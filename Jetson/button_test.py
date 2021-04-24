import Jetson.GPIO as GPIO
import time

out_pin = 11
but_pin = 12


def main():
    prev_value = None
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(out_pin, GPIO.OUT)
    GPIO.setup(but_pin, GPIO.IN)

    GPIO.output(out_pin, GPIO.HIGH)
    print("after GPIO out")
    try:
        while True:
            val = GPIO.input(but_pin)
            if val!=prev_value:
                print("start")
                prev_value = val
                time.sleep(1)
    finally:
        GPIO.cleanup()

    #try
    #    while True:
    #        curr_value = GPIO.input(led_pin)
    GPIO.cleanup()

if __name__ == '__main__':
    main()

