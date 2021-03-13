import machine

class motor:
    """
    Motor object. Used to control a Victor 888 PWM motor controller
    """

    def __init__(self, pin_id):
        pwm = machine.PWM(machine.Pin(pin_id))
        pwm.freq(200)
        self.pwm = pwm
        self.top_dead = 0.305
        self.max = 0.3838
        self.bottom_dead = 0.298
        self.min = 0.2224
        self.disable()


    def set(self, val):
        """
        Set the output (-1 to 1) for the motor speed.
        """
        dead = ((self.top_dead+self.bottom_dead)/2)
        duty = dead
        if val == 0:
            duty = dead
        elif val <= -1:
            duty = self.min-0.01
        elif val >= 1:
            duty = self.max+0.01
        elif val > 0:
            duty = (self.max-self.top_dead)*val+self.top_dead
        elif val < 0:
            duty = (self.bottom_dead-self.min)*val+self.bottom_dead
        self.pwm.duty_u16((int)(duty*2**16))

    def disable(self):
        """
        Disable all PWM signals to motor controller
        """
        self.pwm.duty_u16(0)
