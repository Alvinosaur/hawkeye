import RPi.GPIO as GPIO
import time

#Pin definitions
but2_pin = 15
but_pin = 18

def launch(channel):
    val = GPIO.input(but2_pin)
    #print(val)
    if(val == 1):
        print("launch")
    if(val == 0):
        print("no")


def start(channel):
    val = GPIO.input(but_pin)
    if(val == 1):
        print("start")
    if(val == 0):
        print("stop")

def main():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(but2_pin, GPIO.IN)
    GPIO.setup(but_pin, GPIO.IN)

    GPIO.add_event_detect(but_pin, GPIO.FALLING, callback=start, bouncetime=50)
    GPIO.add_event_detect(but2_pin, GPIO.BOTH, callback=launch, bouncetime=50)
    #GPIO.add_event_detect(but_pin, GPIO.RISING, callback=start, bouncetime=10)
    print("starting program")
    try:
        while True:
            #val = GPIO.input(but_pin)
            #print(val)
            pass
    finally:
        GPIO.cleanup()

    #try
    #    while True:
    #        curr_value = GPIO.input(led_pin)

if __name__ == '__main__':
    main()

