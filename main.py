import array, time
from machine import Pin
import rp2

# Nunchucks
class Nunchuck:
    INIT_COMMANDS = [b'\xf0\x55', b'\xfb\x00']

    def __init__(self, i2c, poll=True, poll_interval=50):
        self.i2c = i2c
        self.address = 0x52
        self.buffer = bytearray(6)
        self.last_poll = time.ticks_ms()
        self.polling_threshold = poll_interval if poll else -1

        self._initialize_nunchuck()

    def _initialize_nunchuck(self):
        for command in self.INIT_COMMANDS:
            self.i2c.writeto(self.address, command)

    def _update(self):
        self.i2c.writeto(self.address, b'\x00')
        self.i2c.readfrom_into(self.address, self.buffer)

    def _poll(self):
        if self.polling_threshold > 0 and time.ticks_diff(time.ticks_ms(), self.last_poll) > self.polling_threshold:
            self._update()
            self.last_poll = time.ticks_ms()

    def buttons(self):
        self._poll()
        return (
            not (self.buffer[5] & 0x02),  # C button
            not (self.buffer[5] & 0x01)   # Z button
        )

    def joystick(self):
        self._poll()
        return self.buffer[0], self.buffer[1]

# WS2812
brightness = 0.3

# Definition of the PIO program for the WS2812 LEDs
@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,  # Initial state of the side-set pins.
    out_shiftdir=rp2.PIO.SHIFT_LEFT,  # Direction in which data is shifted out of the shift register.
    autopull=True,  # Automatic reloading of the shift register from the FIFO (First In, First Out memory).
    pull_thresh=24  # Threshold in bits at which data is automatically reloaded from the FIFO.
)
def ws2812():
    # Timing constants for bit transmission
    T1, T2, T3 = 2, 5, 3
    wrap_target()
    label("bitloop")
    # Transmit one bit, starting with the MSB; set the line to low and delay T3-1 cycles
    out(x, 1).side(0)[T3 - 1]
    # Jump to "do_zero" if the bit is 0, and set the line to high for T1-1 cycles
    jmp(not_x, "do_zero").side(1)[T1 - 1]
    # Set the line back to high and repeat the loop for the next bit
    jmp("bitloop").side(1)[T2 - 1]
    label("do_zero")
    # Set the line to low and delay for T2-1 cycles to send the signal for a 0
    nop().side(0)[T2 - 1]
    wrap()


def hsb_to_rgb(h, s, b):
    # Converts an HSB color value to an RGB color value.

    if s == 0:
        return int(b * 255), int(b * 255), int(b * 255)

    h = h % 360

    h = h / 60
    i = int(h)
    f = h - i
    p = b * (1 - s)
    q = b * (1 - s * f)
    t = b * (1 - s * (1 - f))
    
    p, q, t = int(p * 255), int(q * 255), int(t * 255)
    b = int(b * 255)
    
    if i == 0:
        return b, t, p
    elif i == 1:
        return q, b, p
    elif i == 2:
        return p, b, t
    elif i == 3:
        return p, q, b
    elif i == 4:
        return t, p, b
    elif i == 5:
        return b, p, q

def update_pix(brightness_input=brightness):
    # Scale the color values based on the desired brightness
    dimmer_array = array.array("I", (int(((c >> 16) & 0xFF) * brightness_input) << 16 |
                                     int(((c >> 8) & 0xFF) * brightness_input) << 8 |
                                     int((c & 0xFF) * brightness_input) for c in pixel_array))
    # Transmit the scaled color values to the LEDs
    state_mach.put(dimmer_array, 8)
    #time.sleep_ms(10)

def set_led(ii, color):
    # Set the color value of a single LED
    pixel_array[ii] = (color[1] << 16) + (color[0] << 8) + color[2]

def set_all(color):
    # Set the color value of all LEDs
    for ii in range(led_count):
        set_led(ii, color)

# Number of LEDs in the ring
led_count = 144

# GPIO pin number to which the data line of the LEDs is connected
PIN_NUM = 0

# Array for the color values of the LEDs
pixel_array = array.array("I", [0] * led_count)

# Create and activate the StateMachine object for the PIO program
state_mach = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))
state_mach.active(1)
    
@micropython.native
def main():
    # Initialize the I2C bus
    i2c_devices = [
        machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16), freq=100000),
        machine.I2C(1, scl=machine.Pin(19), sda=machine.Pin(18), freq=100000)
    ]

    nunchuks = [Nunchuck(i2c, poll=True, poll_interval=100) for i2c in i2c_devices]
    
    
    background = (15, 15, 15)  # Background
    cycles = 20000
    trail_length = 1
    
    color = (255,0,0)
    time.sleep(1.5)
    
    for ii in range(int(cycles * led_count) + 1):
        for idx, nunchuk in enumerate(nunchuks, 1):
            if nunchuk.buttons()[0] == True and idx == 1:
                print("nc 1 pressed")
                color = (0,255,0)
            elif nunchuk.buttons()[0] == True and idx == 2:
                print("nc 2 pressed")
                color = (0,0,255)
                
        set_led(ii % led_count, color)

        set_led((ii - trail_length - 1) % led_count, background)
        update_pix()

if __name__ == "__main__":
    main()