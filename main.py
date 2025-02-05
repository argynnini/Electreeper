# -*- coding: utf-8 -*-
#############################################
# Project: Electreeper
# Author: Argynnini
# Date: 2025/02/05
# Description: Electreeper main code
# This code is for Electreeper which CTA is R1D2
# And battery is 3.7V 300mAh
# This code is for CTA control
# CTA0: Left BMF, CTA1: Right BMF
# Used BMF: BMF100
#############################################

from ws2812 import WS2812
from machine import Pin, ADC, PWM, UART
from utime import sleep
from time import ticks_ms
import _thread

# ---- Setting ----
# Thread setting
cta0_resistance = 0
cta1_resistance = 0

# PWM setting
PWM0 = 26  # PWM0
PWM1 = 27  # PWM1
PWMMAX = 55  # PWM max: 60% = 3V
PWMMIN = 0  # PWM min: 0%
PWMFREQ = 500  # PWM frequency: 500Hz

# ADC setting
ADC0 = 28  # ADC0
ADC1 = 29  # ADC1

# CRD setting
CCRD = 0.010  # CRD current: 10mA

# BMF setting
BMFTHRESHOLD = 15.0  # BMF threshold: 15 Ohm
BMF0RELAX = 13.5  # BMF relax: 13.5 Ohm
BMF0SHRINK = 11.24  # BMF shrink: 11.24 Ohm
BMF1RELAX = 13.5  # BMF relax: 13.5 Ohm
BMF1SHRINK = 11.11  # BMF shrink: 11.11 Ohm
BMFRELAXTHRESHOLD = 0.37  # BMF relax threshold: 0.2 Ohm
BMFSHRINKTHRESHOLD = 0.25  # BMF shrink threshold: 0.2 Ohm

# OpAmp setting
VREF = 3.3  # Vref: 3.3V
GAIN0 = 11.20  # Gain: 0 11.346100
GAIN1 = 9.37  # Gain: 1  10.752688

# Feedback setting
TIMEOUT_SHRINK = 2.0
TIMEOUT_RELAX = 4.0

# UART setting
uart = UART(0, baudrate=115200)

# LED setting
led_red = Pin(17, Pin.OUT)  # LED red
led_green = Pin(16, Pin.OUT)  # LED green
led_blue = Pin(25, Pin.OUT)  # LED blue
led_red.value(True)  # LED red off
led_green.value(True)  # LED green off
led_blue.value(True)  # LED blue off

# NeoPixel setting
BLACK = (0, 0, 0)
RED = (255, 0, 0)
YELLOW = (255, 150, 0)
GREEN = (0, 255, 0)
CYAN = (0, 255, 255)
BLUE = (0, 0, 255)
PURPLE = (180, 0, 255)
WHITE = (255, 255, 255)
neoPixel = Pin(11, Pin.OUT)
neoPixel.value(1)
led = WS2812(12, 1)  # WS2812(pin_num,led_count)

# class
class CTA():
    # Initialize function
    def __init__(self, pwm_pin, adc_pin, adc_gain, crd_current, bmf_relax, bmf_shrink):
        self.__adc = ADC(adc_pin)
        self.__pwm = PWM(Pin(pwm_pin, Pin.OUT))
        self.__gain = adc_gain
        self.__ccrd = crd_current
        self.__relax = bmf_relax
        self.__shrink = bmf_shrink
        self.__cta_resistance = 0
        self.__pwm_duty = 0
        self.__pwm_duty_max = PWMMAX
        self.__pwm_duty_min = PWMMIN

    # ---- ADC function ----
    # Get voltage function
    def get_adc_voltage(self):
        return self.__adc.read_u16() * VREF / 65535

    # Get resistance function
    def get_adc_resistance(self):
        return (self.get_adc_voltage() / self.__gain) / self.__ccrd

    # ---- CTA function ----
    # Get resistance function
    def get_bmf_resistance(self, threshold=BMFTHRESHOLD, timeout=10.0):
        start_time = ticks_ms()
        while ticks_ms() - start_time < timeout:
            if ((resistance := self.get_adc_resistance()) < threshold):
                self.__cta_resistance = resistance
                return self.__cta_resistance
        return 0

    # ---- PWM function ----
    # Init PWM function
    def init_pwm(self, freq=500, pwm_duty_min=0.0, pwm_duty_max=0.0):
        self.__pwm.freq(freq)
        self.set_pwm_duty(0)
        self.__pwm_duty_min = pwm_duty_min
        self.__pwm_duty_max = pwm_duty_max

    # Set PWM function
    def set_pwm_duty(self, percentage=0.0):
        if percentage > self.__pwm_duty_max:
            percentage = self.__pwm_duty_max
        elif percentage < self.__pwm_duty_min:
            percentage = self.__pwm_duty_min
        self.__pwm.duty_u16(int(percentage * 65535 / 100))

    # Get PWM function
    def get_pwm_duty(self):
        return self.__pwm.duty_u16()

    # End PWM function
    def end_pwm(self):
        self.set_pwm_duty(0)

    # ---- Check resistance function ----
    def check_resistance(self, duty = 0, count=1000):
        self.set_pwm_duty(duty)
        test_sum = 0
        test_count = 0
        while ((test_count) < count):
            test_sum += self.get_bmf_resistance()
            test_count += 1
        print(self.__gain, test_sum / test_count)
        return test_sum / test_count


# ---- Function ----
# change NeoPixel color function
def change_color(color=(0, 0, 0)):
    led.pixels_fill(color)
    led.pixels_show()

# core1 function (get cta resistance)
def core1(cta_left, cta_right):
    led_red.value(False)  # LED red on
    try:
        while True:
            cta_left.get_bmf_resistance()
            cta_right.get_bmf_resistance()
    except KeyboardInterrupt:
        led_red.value(True)  # LED red off
        _thread.exit()

# core0 function (pwm control)
def core0(cta_left, cta_right):
    led_blue.value(False)  # LED blue on
    cta_left.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)
    cta_right.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)
    # try:
    while True:
        change_color(RED)
        # shrink
        cta_left.set_pwm_duty(PWMMAX)
        cta_right.set_pwm_duty(PWMMAX)
        initial_time = ticks_ms()
        while ((ticks_ms() - initial_time) < (TIMEOUT_SHRINK * 1000)) and not(
            (cta_left.__cta_resistance < (BMF0SHRINK + BMFSHRINKTHRESHOLD)) and 
            (cta_right.__cta_resistance < (BMF1SHRINK + BMFSHRINKTHRESHOLD))
        ):
            print(f"{ticks_ms() - initial_time}, \
                    {BMF0SHRINK},{cta_left.__cta_resistance}, \
                    {BMF1SHRINK},{cta_right.__cta_resistance}")
        change_color(BLUE)
        # relax
        cta_left.set_pwm_duty(PWMMIN)
        cta_right.set_pwm_duty(PWMMIN)
        initial_time = ticks_ms()
        while ((ticks_ms() - initial_time) < (TIMEOUT_RELAX * 1000)) and not(
            (cta_left.__cta_resistance > (BMF0RELAX - BMFRELAXTHRESHOLD)) and 
            (cta_right.__cta_resistance > (BMF1RELAX - BMFRELAXTHRESHOLD))
        ):
            print(f"{ticks_ms() - initial_time}, \
                    {BMF0RELAX},{cta_left.__cta_resistance}, \
                    {BMF1RELAX},{cta_right.__cta_resistance}")

# ---- Main ----
change_color(WHITE)
# Initialize CTA1
cta0 = CTA(
    pwm_pin=PWM0,
    adc_pin=ADC0,
    adc_gain=GAIN0,
    crd_current=CCRD,
    bmf_relax=BMF0RELAX,
    bmf_shrink=BMF0SHRINK,
)
cta1 = CTA(
    pwm_pin=PWM1,
    adc_pin=ADC1,
    adc_gain=GAIN1,
    crd_current=CCRD,
    bmf_relax=BMF1RELAX,
    bmf_shrink=BMF1SHRINK,
)
cta0.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)  # Initialize PWM
cta1.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)  # Initialize PWM
sleep(1)
# check BMF relax resistance
# if measurement resistance < shrink threshold, set init relax resistance
if ((bmf0_relax := cta0.check_resistance(duty=0, count=1000)) > BMF0SHRINK):
    BMF0RELAX = bmf0_relax
else:
    change_color(CYAN)
if ((bmf1_relax := cta1.check_resistance(duty=0, count=1000)) > BMF1SHRINK):
    BMF1RELAX = bmf1_relax
else:
    change_color(CYAN)
sleep(1)
led_green.value(False)  # LED green on
change_color(GREEN)
### --Start thread-- ###
_thread.start_new_thread(core1, (cta0, cta1,))  # Start core1
core0(cta0, cta1)  # Start core0

# ---- End ----
sleep(0.25)
cta0.set_pwm_duty(0)
cta1.set_pwm_duty(0)
cta0.end_pwm()
cta1.end_pwm()
# change_color(BLACK)
led_red.value(True)
led_green.value(True)
led_blue.value(True)
