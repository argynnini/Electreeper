# -*- coding: utf-8 -*-
from ws2812 import WS2812
from PID import PID as simplePID
from machine import Pin, ADC, PWM, UART
import logging
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
PWMMAX = 90  # PWM max: 90%
PWMMIN = 0  # PWM min: 0%
PWMFREQ = 500  # PWM frequency: 500Hz

# ADC setting
ADC0 = 28  # ADC0
ADC1 = 29  # ADC1

# CRD setting
CCRD = 0.010  # CRD current: 10mA

# BMF setting
BMFTHRESHOLD = 29.0  # BMF threshold: 0 Ohm
BMF0RELAX = 25.50  # BMF relax: 0 Ohm
BMF0SHRINK = 21.20  # BMF shrink: 0 Ohm
BMF1RELAX = 27.10  # BMF relax: 0 Ohm
BMF1SHRINK = 22.00  # BMF shrink: 0 Ohm

# OpAmp setting
VREF = 3.3  # Vref: 3.3V
GAIN0 = 11.20  # Gain: 0 11.346100
GAIN1 = 9.37  # Gain: 1  10.752688

# PID setting
KP = 70  # Proportional gain
KI = 0  # Integral gain
KD = 0.0  # Derivative gain

# Feedback setting
TIMEOUT_FLEX = 1
TIMEOUT_RELAX = 4

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

# Logging setting
logging.basicConfig(level=logging.DEBUG)
log_electreeper = logging.getLogger("Electreeper")
log_core0 = logging.getLogger("Core0")
log_core1 = logging.getLogger("Core1")
log_cta0 = logging.getLogger("CTA0")
log_cta1 = logging.getLogger("CTA1")


# class
class CTA:
    # Initialize function
    def __init__(self, pwm_pin, adc_pin, adc_gain, crd_current, bmf_relax, bmf_shrink):
        self.__adc = ADC(adc_pin)
        self.__pwm = PWM(Pin(pwm_pin, Pin.OUT))
        self.__pid = simplePID()
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
        # logger.error("Timeout")

    # ---- PWM function ----
    # Init PWM function
    def init_pwm(self, freq=500, pwm_duty_min=0.0, pwm_duty_max=90.0):
        self.__pwm.freq(freq)
        self.set_pwm_duty(0)
        self.__pwm_duty_min = pwm_duty_min
        self.__pwm_duty_max = pwm_duty_max

    # Set PWM function
    def set_pwm_duty(self, percentage=0.0):
        if percentage > self.__pwm_duty_max:
            percentage = self.__pwm_duty_max
            # logger.warning("PWM duty is over " + str(self.__pwm_duty_max) + " %")
        elif percentage < self.__pwm_duty_min:
            percentage = self.__pwm_duty_min
            # logger.warning("PWM duty is under " + str(self.__pwm_duty_min) + " %")
        self.__pwm.duty_u16(int(percentage * 65535 / 100))

    # Get PWM function
    def get_pwm_duty(self):
        return self.__pwm.duty_u16()

    # End PWM function
    def end_pwm(self):
        self.set_pwm_duty(0)

    # ---- PID function ----
    # Init PID function
    def init_pid(self, p, i, d):
        self.__pid = simplePID(-p, -i, -d, scale="ms")
        # gain is negative because
        # the voltage is lower,
        # the resistance is higher
        self.__pid.sample_time = 100  # 100ms
        self.__pid.output_limits = (int(PWMMIN), int(PWMMAX))  # 0~90%
        self.__pid.auto_mode = True  # Auto mode
        self.__pid.proportional_on_measurement = False
        # Proportional on measurement
        # False : output=  Kp*e(t) + Ki*Σe(t) + Kd*Δe(t)
        # proportional on error
        # True  : output= -Kp[Input(t) - Input_init] + Ki*Σe(t) + Kd*Δe(t)
        # proportional on measurement

    # Set PID gain function
    def set_pid_gain(self, p, i, d):
        self.__pid.tunings = (p, i, d)

    # Set PID setpoint function
    def set_pid_setpoint(self, setpoint):
        self.__pid.setpoint = setpoint

    # Get PID gain function
    def get_pid_gain(self):
        return self.__pid.tunings

    # Get PID output function
    def get_pid_output(self):
        return self.__pid.components

    # End PID function
    def end_pid(self):
        self.__pid.auto_mode = False
        self.set_pid_gain(0, 0, 0)
        self.set_pid_setpoint(0)

    # ---- Control function ----
    # Control CTA on/off function
    def control(self, resistance, pwm, threshold=0.2, timeout=10.0):
        global initial_time
        if resistance > self.__relax:
            resistance = self.__relax
            # logger.warning("Resistance is over " + str(self.__relax) + " Ohm")
        elif resistance < self.__shrink:
            resistance = self.__shrink
            # logger.warning("Resistance is under " + str(self.__shrink) + " Ohm")
        # set PID setpoint
        self.set_pwm_duty(pwm)
        start_time = ticks_ms()
        while ticks_ms() - start_time < (timeout * 1000) and not (
            (resistance - threshold) < self.__cta_resistance < (resistance + threshold)
        ):
            # get BMF resistance
            # cta_resistance = self.get_bmf_resistance()
            print(f"{ticks_ms() - initial_time},{resistance},{self.__cta_resistance},{pwm},{(ticks_ms() - start_time) / 1000}")
            # print(
            #     "Control: {:3d}, Resistance: {:2.3f}, Setpoint: {:2.2f}, PWM: {:3d}, Time: {:2.3f}".format(
            #         pwm, self.__cta_resistance, resistance, pwm, ((ticks_ms() - start_time) / 1000)
            #     ),
            #     end="\r",
            # )

    # control CTA pid function
    def control_pid(self, resistance, threshold=0.2, timeout=10.0):
        if resistance > self.__relax:
            resistance = self.__relax
            # logger.warning("Resistance is over " + str(self.__relax) + " Ohm")
        elif resistance < self.__shrink:
            resistance = self.__shrink
            # logger.warning("Resistance is under " + str(self.__shrink) + " Ohm")
        # set PID setpoint
        self.set_pid_setpoint(resistance)
        cta_resistance = 0
        start_time = ticks_ms()  # start time
        while (ticks_ms() - start_time < (timeout * 1000)) and not (
            (resistance - threshold) < cta_resistance < (resistance + threshold)
        ):
            # get BMF resistance
            cta_resistance = self.get_bmf_resistance()
            # set PID control
            if (cta_control := self.__pid(cta_resistance)) is None:
                cta_control = 0
            self.set_pwm_duty(cta_control)
            p, i, d = self.get_pid_output()
            print(
                "Control: {:2.3f}, Resistance: {:2.3f}, Setpoint: {:2.3f}, PID: {:3.3f}, {:3.3f}, {:3.3f}".format(
                    cta_control, cta_resistance, resistance, p, i, d
                ),
                end="\r",
            )


# ---- Function ----
# change NeoPixel color function
def change_color(color=(0, 0, 0)):
    led.pixels_fill(color)
    led.pixels_show()


# core0 function (read resistance)
def core0(fore_cta, hind_cta):
    led_red.value(False)  # LED red on
    print("\n", end="\r")
    log_core0.info("Start read resistance")
    try:
        while True:
            fore_cta.get_bmf_resistance()
            hind_cta.get_bmf_resistance()
    except KeyboardInterrupt:
        print("\n", end="\r")
        log_core0.info("End read resistance")
        led_red.value(True)  # LED red off


# core1 function (pwm control)
def core1(fore_cta, hind_cta):
    led_blue.value(False)  # LED blue on
    log_core1.info("Start PWM control")
    fore_cta.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)
    hind_cta.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)
    global initial_time
    initial_time = ticks_ms()
    # try:
    while True:
        # hind-leg:on, fore-leg:off
        print("\n", end="\r")
        log_cta1.info("hind-shrink")
        change_color(RED)
        # flex
        fore_cta.set_pwm_duty(0)
        hind_cta.control(pwm=PWMMAX, resistance=BMF0SHRINK, threshold=1, timeout=TIMEOUT_FLEX)
        
        # hind-leg:off, fore-leg:on
        print("\n", end="\r")
        log_cta1.info("fore-flex")
        change_color(YELLOW)
        # relax
        hind_cta.set_pwm_duty(0)
        fore_cta.control(pwm=PWMMAX, resistance=BMF1SHRINK, threshold=1, timeout=TIMEOUT_FLEX)
        
        # hind-leg:off, fore-leg:off
        print("\n", end="\r")
        log_cta1.info("relax")
        change_color(BLUE)
        # relax
        fore_cta.control(pwm=PWMMIN, resistance=BMF1RELAX, threshold=2, timeout=TIMEOUT_RELAX)
    # except KeyboardInterrupt:
    #     cta.end_pwm()
    #     print("\n", end="\r")
    #     log_core1.info("End PWM control")
    #     led_blue.value(True)  # LED blue off
    #     _thread.exit()



# ---- Main ----
change_color(WHITE)
sleep(3)
log_electreeper.info("Start")
led_green.value(False)  # LED red on
change_color(GREEN)
# Initialize CTA0
cta0 = CTA(
    pwm_pin=PWM0,
    adc_pin=ADC0,
    adc_gain=GAIN0,
    crd_current=CCRD,
    bmf_relax=BMF0RELAX,
    bmf_shrink=BMF0SHRINK,
)
cta0.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)  # Initialize PWM
cta0.init_pid(p=KP, i=KI, d=KD)  # Initialize PID
# Initialize CTA1
cta1 = CTA(
    pwm_pin=PWM1,
    adc_pin=ADC1,
    adc_gain=GAIN1,
    crd_current=CCRD,
    bmf_relax=BMF1RELAX,
    bmf_shrink=BMF1SHRINK,
)
cta1.init_pwm(freq=PWMFREQ, pwm_duty_min=PWMMIN, pwm_duty_max=PWMMAX)  # Initialize PWM
cta1.init_pid(p=KP, i=KI, d=KD)  # Initialize PID
sleep(1)
# check resistance
# initial_time = ticks_ms()
# cta0.set_pwm_duty(0)
# sleep(3)
# test_sum = 0
# test_count = 0
# while test_count < 10000:
#     temp = cta0.get_adc_resistance()
#     if(temp < 29):
#         test_sum += temp
#         test_count += 1
#         print(f"{test_count},{temp}")
# print(GAIN0, test_sum / test_count)
# Start thread
_thread.start_new_thread(core1, (cta1, cta0,))  # Start core0
core0(cta1, cta0)  # Start core1

# End
sleep(0.25)
cta0.end_pid()
cta0.end_pwm()
cta1.end_pid()
cta1.end_pwm()
# change_color(BLACK)
led_red.value(True)
led_green.value(True)
led_blue.value(True)
print("\n", end="\r")
log_electreeper.info("End")
