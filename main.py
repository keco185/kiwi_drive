import machine
import utime
from motor import motor
from encoder import encoder
from kiwi_encoders import kiwi_encoders
import sys
import select
from machine import WDT

class main:
    """
    This main class houses the main robot execution code. main_loop() runs indefinitely
    """
    
    def __init__(self):
        # ---------- SETTINGS ---------- #
        self.MIN_CYCLE_PERIOD = 10 # Minimum time (in ms) between program iterations

        # -------- PORT MAPPING -------- #
        self.RED_LED_PORT = 28 # GP28 Pin 34
        self.GREEN_LED_PORT = 27 # GP27 Pin 32
        self.BUZZER_PORT = 22 # GP22 Pin 29
        
        self.WHEEL_1_PORT = 8 # GP8 Pin 11
        self.WHEEL_2_PORT = 12 # GP12 Pin 16
        self.WHEEL_3_PORT = 14 # GP14 Pin 19
        
        self.ENC_1A_PORT = 2 # GP2 Pin 4
        self.ENC_1B_PORT = 3 # GP3 Pin 5
        self.ENC_2A_PORT = 4 # GP4 Pin 6
        self.ENC_2B_PORT = 5 # GP5 Pin 7
        self.ENC_3A_PORT = 6 # GP6 Pin 9
        self.ENC_3B_PORT = 7 # GP7 Pin 10
        
        self.I2C_SDA = 0 # GP0 Pin 1
        self.I2C_SCL = 1 # GP1 Pin 2

        # ------------ VARS ------------ #
        self.enabled = False
        self.connected = False
        self.i2caddr = 0x0
        self.sqrt3 = 3**0.5/2 # Finding square roots (and division) is time consuming. Let's only do it once

        # --------- INITIALIZE --------- #
        self.red_led = machine.Pin(self.RED_LED_PORT, machine.Pin.OUT)
        self.red_led.value(0)
        self.green_led = machine.Pin(self.GREEN_LED_PORT, machine.Pin.OUT)
        self.green_led.value(0)
        self.buzzer = machine.Pin(self.BUZZER_PORT, machine.Pin.OUT)
        self.buzzer.value(0)
        
        self.wheel1 = motor(self.WHEEL_1_PORT)
        self.wheel2 = motor(self.WHEEL_2_PORT)
        self.wheel3 = motor(self.WHEEL_3_PORT)
        
        self.encoder1 = encoder(self.ENC_1A_PORT, self.ENC_1B_PORT)
        self.encoder2 = encoder(self.ENC_2A_PORT, self.ENC_2B_PORT)
        self.encoder3 = encoder(self.ENC_3A_PORT, self.ENC_3B_PORT)

        self.kiwi_encoder = kiwi_encoders(self.encoder1, self.encoder2, self.encoder3)
        
        sda=machine.Pin(self.I2C_SDA)
        scl=machine.Pin(self.I2C_SCL)
        self.i2c=machine.I2C(0,sda=sda, scl=scl, freq=400000)
        self.wdt = None
        
    # ----------- FUNCTS ----------- #
    def run_enabled(self):
        # Loop to run when robot is enabled
        if self.wdt is None:
            self.wdt = WDT(timeout=500)
        self.red_led.value(self.system_secs % 2)
        self.green_led.value(1)
        
    def run_disabled(self):
        # Loop to run when robot is disabled
        self.red_led.value(0)
        self.green_led.value(1)
        self.wheel1.set(0)
        self.wheel2.set(0)
        self.wheel3.set(0)
        
    def run_disconnected(self):
        # Loop to run when robot has no connection to main computer
        self.red_led.value(0)
        self.green_led.value(self.system_secs % 2)
        self.wheel1.disable()
        self.wheel2.disable()
        self.wheel3.disable()
        results = self.i2c.scan()
        if len(results) > 0:
            print("I2C devices found.Connecting to first one:")
            print(results)
            self.i2caddr = results[0]
            self.connected = True
            
        
    def buzz(self, mode):
        # Ring the buzzer
        # TODO: Throw this in another thread
        if mode == "start":
            self.buzzer.value(1)
            utime.sleep(0.1)
            self.watchdog()
            self.buzzer.value(0)
        elif mode == "en":
            self.buzz("start")
            utime.sleep(0.1)
            self.buzz("start")
        elif mode == "dis":
            self.buzz("start")
            utime.sleep(0.1)
            self.buzz("en")
        elif mode == "dconn":
            self.buzz("en")
            utime.sleep(0.1)
            self.buzz("en")
        elif mode == "conn":
            self.buzzer.value(1)
            self.watchdog()
            utime.sleep(0.4)
            self.watchdog()
            self.buzzer.value(0)
    
    def read_input(self):
        # Checks for command inputs on stdin
        if select.select([sys.stdin,],[],[],0.0)[0]:
            in_val = sys.stdin.buffer.readline().decode("utf-8")[:-2]
            print(in_val)
            if len(in_val) == 0:
                return []
            in_val = in_val.upper().strip()
            if in_val == "CONN" and not self.connected:
                self.connected = True
                self.buzz("conn")
            if in_val == "EN" and self.connected and not self.enabled:
                self.enabled = True
                self.buzz("en")
            if in_val == "DIS" and self.enabled:
                self.enabled = False
                self.run_disabled()
                self.buzz("dis")
            if in_val == "DCONN" and self.connected:
                self.connected = False
                self.run_disconnected()
                self.buzz("dconn")
            params = in_val.split(" ")
            if params[0] == "CMD":
                return params[1:]
        return []
    
    def watchdog(self):
        # Feeds the watchdog timer. Keep the canine fed please
        if not self.wdt is None:
            self.wdt.feed()
            
    def CMD_V(self, params):
        # V Command (voltage control)
        # This func parses and executes parameters for the voltage control input
        if not (self.enabled and self.connected):
            return
        x = 0
        y = 0
        r = 0
        try:
            for param in params:
                if param[0] == "X":
                    x = float(param[1:])
                elif param[0] == "Y":
                    y = float(param[1:])
                elif param[0] == "R":
                    r = float(param[1:])
        except:
            print("CMD ERR")
        mxo2 = -x/2
        ysq3 = self.sqrt3*y
        m1 = mxo2-ysq3+r
        m2 = mxo2+ysq3+r
        m3 = x+r
        scale = max(abs(m1), abs(m2), abs(m3))
        if scale > 1:
            m1 /= scale
            m2 /= scale
            m3 /= scale
        self.wheel1.set(m1)
        self.wheel2.set(m2)
        self.wheel3.set(m3)
        
    def main_loop(self):
        # This function never ends. Keeps updating robot
        print()
        self.buzz("start") # Audible tone to tell anyone nearby the robot is turning on
        while True:
            self.system_time = utime.ticks_ms()
            self.system_secs = int(self.system_time/1000)
            self.watchdog()
            
            # Execute commands. Only 1 command can be executed each iteration
            command = self.read_input()
            if len(command) > 0:
                if command[0] == "V":
                    self.CMD_V(command[1:])
            
            # Run update functions
            if not self.connected:
                self.enabled = False
            if self.enabled:
                self.run_enabled()
            elif self.connected:
                self.run_disabled()
            else:
                self.run_disconnected()
            
            self.watchdog()
            
            # Wait to begin next update loop
            dt = utime.ticks_ms() - self.system_time
            if self.MIN_CYCLE_PERIOD > dt:
                utime.sleep((self.MIN_CYCLE_PERIOD - dt)/1000)


# Run the codes
robot_class = main()
robot_class.main_loop()
