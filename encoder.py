from machine import Pin

class encoder:
    """
    Encoder object. Keeps track of the displacement seen by a quadrature encoder
    """
    state = 0
    disp = 0
    pulses_per_distance = 10*80/(8*3.14159) #gear ratio * pulses per rotation / inches per rotation
    update_func = None

    def __init__(self, pin_a, pin_b):
        self.pina = Pin(pin_a, Pin.IN, Pin.PULL_UP)
        self.pina.irq(lambda pin: self._update_a_(pin.irq().flags()),
        Pin.IRQ_FALLING + Pin.IRQ_RISING)
        self.pinb = Pin(pin_b, Pin.IN, Pin.PULL_UP)
        self.pinb.irq(lambda pin: self._update_b_(pin.irq().flags()),
        Pin.IRQ_FALLING + Pin.IRQ_RISING)
        self.reset()


    def set_update_func(self, update_func):
        """
        Runs provided function any time encoder position updates
        """
        self.update_func = update_func


    def reset(self):
        """
        Resets encoder position to 0
        """
        self.disp = 0
        self.reset_state()


    def get_dist(self):
        """
        Returns the encoder-measured displacement
        """
        return self.disp/self.pulses_per_distance


    def reset_state(self):
        """
        Recalculates the encoder's current state in the quadrature state machine
        """
        if self.pina.value() == 1:
            if self.pinb.value() == 1:
                self.state = 1
            else:
                self.state = 0
        else:
            if self.pinb.value() == 1:
                self.state = 2
            else:
                self.state = 3


    def _update_a_(self, flag):
        """
        Update the encoder due to new a channel input state
        """
        pstate = self.pina.value()
        old_disp = self.disp
        if self.state == 0:
            if not pstate:
                self.state = 3
                self.disp -= 1
        elif self.state == 1:
            if not pstate:
                self.state = 2
                self.disp += 1
        elif self.state == 2:
            if pstate:
                self.state = 1
                self.disp -= 1
        else:
            if pstate:
                self.state = 0
                self.disp += 1
        if self.update_func is not None:
            self.update_func(self.disp/self.pulses_per_distance, old_disp/self.pulses_per_distance)


    def _update_b_(self, flag):
        """
        Update the encoder due to new b channel input state
        """
        pstate = self.pinb.value()
        old_disp = self.disp
        if self.state == 0:
            if pstate:
                self.state = 1
                self.disp += 1
        elif self.state == 1:
            if not pstate:
                self.state = 0
                self.disp -= 1
        elif self.state == 2:
            if not pstate:
                self.state = 3
                self.disp += 1
        else:
            if pstate:
                self.state = 2
                self.disp -= 1
        if self.update_func is not None:
            self.update_func(self.disp/self.pulses_per_distance, old_disp/self.pulses_per_distance)
