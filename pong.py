import array
import time
from machine import Pin, I2C
import rp2
import micropython

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LED_COUNT = 144
PIN_NUM = 0

# Initial speed settings
BASE_SPEED = 15
MIN_SPEED = 3

# Initial zone length
INITIAL_ZONE_LENGTH = 20
MIN_ZONE_LENGTH = 10

# After every 10 hits, zone length shortens by 1
HITS_PER_ZONE_REDUCTION = 10

# Colors (R, G, B)
PLAYER1_COLOR = (0, 0, 255)        # Blue
PLAYER2_COLOR = (255, 0, 0)        # Red
BALL_COLOR = (255, 255, 255)       # White
BACKGROUND_COLOR = (1, 1, 1)       # Dim background

BRIGHTNESS = 0.3

# Game states
IDLE = 0
SERVE = 1
IN_GAME = 2

# Buttons: 
# - C button to serve
# - Z button to hit

# ---------------------------------------------------------------------------
# Nunchuck class
# ---------------------------------------------------------------------------

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
        """
        Returns (C_pressed, Z_pressed).
        """
        self._poll()
        return (
            not (self.buffer[5] & 0x02),  # C button
            not (self.buffer[5] & 0x01)   # Z button
        )

# ---------------------------------------------------------------------------
# WS2812 (NeoPixel) LED Handling with RP2040 PIO
# ---------------------------------------------------------------------------

@rp2.asm_pio(
    sideset_init=rp2.PIO.OUT_LOW,
    out_shiftdir=rp2.PIO.SHIFT_LEFT,
    autopull=True,
    pull_thresh=24
)
def ws2812():
    T1, T2, T3 = 2, 5, 3
    wrap_target()
    label("bitloop")
    out(x, 1).side(0)[T3 - 1]
    jmp(not_x, "do_zero").side(1)[T1 - 1]
    jmp("bitloop").side(1)[T2 - 1]
    label("do_zero")
    nop().side(0)[T2 - 1]
    wrap()

pixel_array = array.array("I", [0] * LED_COUNT)

state_mach = rp2.StateMachine(0, ws2812, freq=8_000_000, sideset_base=Pin(PIN_NUM))
state_mach.active(1)

def update_pixels(brightness_input=BRIGHTNESS):
    dimmer_array = array.array(
        "I",
        (
            (int(((c >> 16) & 0xFF) * brightness_input) << 16) |
            (int(((c >> 8) & 0xFF) * brightness_input) << 8) |
            (int((c & 0xFF) * brightness_input))
            for c in pixel_array
        )
    )
    state_mach.put(dimmer_array, 8)

def set_led(index, color):
    if 0 <= index < LED_COUNT:
        r, g, b = color
        pixel_array[index] = (g << 16) + (r << 8) + b

def set_all(color):
    for i in range(LED_COUNT):
        set_led(i, color)

# ---------------------------------------------------------------------------
# Game Logic
# ---------------------------------------------------------------------------

def ball_in_home_zone(ball_pos, direction, zone_length):
    """
    Check if the ball is in the home zone of the player it is moving toward.
    direction == 1: target is player 2 (right side)
    direction == -1: target is player 1 (left side)
    """
    if direction == 1:
        # Right player's zone: last zone_length LEDs on the right
        return ball_pos >= LED_COUNT - zone_length
    else:
        # Left player's zone: first zone_length LEDs on the left
        return ball_pos < zone_length

def compute_new_speed(ball_pos, direction, zone_length):
    """
    Compute new speed after a hit based on how deep into the zone the ball is hit.
    Deeper hit = faster speed.
    """
    if direction == 1:
        # Ball moving right, right player hits
        # Zone spans from LED_COUNT - zone_length to LED_COUNT - 1
        zone_start = LED_COUNT - zone_length
        zone_depth = ball_pos - zone_start  # 0 at front, zone_length-1 at deep end
    else:
        # Ball moving left, left player hits
        # Zone spans from 0 to zone_length-1
        # Deeper means closer to 0, front means closer to zone_length-1
        zone_depth = (zone_length - 1) - ball_pos

    max_depth = zone_length - 1
    factor = zone_depth / max_depth
    speed = BASE_SPEED - factor * (BASE_SPEED - MIN_SPEED)
    return int(speed)

def blink_ball(ball_pos, duration=2.0, blink_interval=0.3, zone_length=INITIAL_ZONE_LENGTH, 
               player1_score=0, player2_score=0):
    """
    Blink the ball at the given position for a certain duration.
    """
    start_time = time.ticks_ms()
    on = True
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        draw_field(zone_length, player1_score, player2_score)
        if on:
            set_led(ball_pos, BALL_COLOR)
        update_pixels()
        on = not on
        time.sleep(blink_interval)

def draw_scoreboard(player1_score, player2_score):
    """
    Draw the scoreboard in the middle of the field.
    Scores are shown around the center (LED_COUNT//2).
    One LED gap between each point to make them easier to read.
    Example:
    - center = 72
    - Player1's points go on the left side (decreasing indices)
    - Player2's points go on the right side (increasing indices)

    Pattern: [gap][point][gap][point] ... from the center outwards
    Start with one gap LED next to center on each side.
    """
    center = LED_COUNT // 2

    # If no one has scored yet, no scoreboard
    if player1_score == 0 and player2_score == 0:
        return

    # We'll use a pattern:
    # Left side (player1): center-1 gap, center-2 point, center-3 gap, center-4 point...
    # Right side (player2): center+1 gap, center+2 point, center+3 gap, center+4 point...

    # Draw player1 score on the left side
    left_pos = center - 1
    for i in range(player1_score):
        # gap
        set_led(left_pos, BACKGROUND_COLOR)
        left_pos -= 1
        # point
        set_led(left_pos, PLAYER1_COLOR)
        left_pos -= 1

    # If ended on a point, next would be a gap - if we have leftover space, set it
    # Actually not needed, since the next redraw will set background anyway.

    # Draw player2 score on the right side
    right_pos = center + 1
    for i in range(player2_score):
        # gap
        set_led(right_pos, BACKGROUND_COLOR)
        right_pos += 1
        # point
        set_led(right_pos, PLAYER2_COLOR)
        right_pos += 1

def draw_field(zone_length, player1_score, player2_score):
    """
    Redraw the entire field:
    - Background
    - Player zones (with current zone_length)
    - Scoreboard (if any scores)
    Does NOT draw the ball. The ball is drawn after this.
    """
    set_all(BACKGROUND_COLOR)

    # Draw Player 1 zone (left)
    for i in range(zone_length):
        set_led(i, PLAYER1_COLOR)

    # Draw Player 2 zone (right)
    for i in range(zone_length):
        set_led(LED_COUNT - 1 - i, PLAYER2_COLOR)

    # Draw scoreboard if any points have been scored
    draw_scoreboard(player1_score, player2_score)

@micropython.native
def main():
    # Initialize I2C and Nunchucks
    i2c_devices = [
        I2C(0, scl=Pin(17), sda=Pin(16), freq=100000),
        I2C(1, scl=Pin(19), sda=Pin(18), freq=100000)
    ]
    nunchucks = [Nunchuck(i2c, poll=True, poll_interval=100) for i2c in i2c_devices]

    player1_score = 0
    player2_score = 0

    # Initial conditions
    game_state = IDLE
    ball_pos = LED_COUNT // 2
    ball_direction = 1
    current_speed = BASE_SPEED
    serve_player = None
    zone_length = INITIAL_ZONE_LENGTH
    hits_count = 0  # Count successful hits for zone length reduction

    draw_field(zone_length, player1_score, player2_score)
    set_led(ball_pos, BALL_COLOR)
    update_pixels()

    while True:
        p1_c, p1_z = nunchucks[0].buttons()
        p2_c, p2_z = nunchucks[1].buttons()

        if game_state == IDLE:
            # Wait for first C press
            if p1_c:
                serve_player = 1
                game_state = SERVE
                # Blink while waiting for second C
                blink_ball(ball_pos, duration=0.0, blink_interval=0)
            elif p2_c:
                serve_player = 2
                game_state = SERVE
                # Blink while waiting for second C
                blink_ball(ball_pos, duration=0.0, blink_interval=0)

        elif game_state == SERVE:
            # We have a serve_player who pressed C first.
            # Wait for the other player to press C to start the game
            blink_ball(ball_pos, duration=2.0, blink_interval=0.3, zone_length=zone_length, 
                       player1_score=player1_score, player2_score=player2_score)
            other_pressed_c = (p2_c if serve_player == 1 else p1_c)
            if other_pressed_c:
                # Start the game
                ball_direction = 1 if serve_player == 1 else -1
                game_state = IN_GAME

        elif game_state == IN_GAME:
            # Move the ball one step
            old_pos = ball_pos
            ball_pos += ball_direction

            # Redraw field and ball
            draw_field(zone_length, player1_score, player2_score)
            set_led(ball_pos % LED_COUNT, BALL_COLOR)
            update_pixels()
            time.sleep_ms(current_speed)

            # Determine which player's hit matters
            relevant_player = 2 if ball_direction == 1 else 1

            # Check button press to hit
            # Players use Z button to hit
            relevant_pressed_z = p2_z if relevant_player == 2 else p1_z

            if relevant_pressed_z:
                # Player tries to hit
                if not ball_in_home_zone(ball_pos, ball_direction, zone_length):
                    # Early hit -> opponent scores
                    if relevant_player == 1:
                        player2_score += 1
                        print("Player 2 scores! Total:", player2_score)
                    else:
                        player1_score += 1
                        print("Player 1 scores! Total:", player1_score)

                    # Reset game
                    game_state = IDLE
                    zone_length = INITIAL_ZONE_LENGTH
                    hits_count = 0
                    draw_field(zone_length, player1_score, player2_score)
                    ball_pos = LED_COUNT // 2
                    set_led(ball_pos, BALL_COLOR)
                    update_pixels()
                    continue

                # Valid hit inside home zone
                hits_count += 1
                if hits_count >= HITS_PER_ZONE_REDUCTION:
                    hits_count = 0
                    zone_length = max(zone_length - 1, MIN_ZONE_LENGTH)

                # Reverse direction
                ball_direction *= -1
                current_speed = compute_new_speed(ball_pos, -ball_direction, zone_length)

            # Check scoring condition if ball goes off the ends
            if ball_pos < 0:
                # Ball past left end
                player2_score += 1
                print("Player 2 scores! Total:", player2_score)
                game_state = IDLE
            elif ball_pos >= LED_COUNT:
                # Ball past right end
                player1_score += 1
                print("Player 1 scores! Total:", player1_score)
                game_state = IDLE

            if game_state == IDLE:
                # Reset field for next round, keep scores, keep zone length and hits as if resetting?
                # As specified, we reset the zone length and hits count after each score
                zone_length = INITIAL_ZONE_LENGTH
                hits_count = 0
                draw_field(zone_length, player1_score, player2_score)
                ball_pos = LED_COUNT // 2
                current_speed = BASE_SPEED
                set_led(ball_pos, BALL_COLOR)
                update_pixels()

if __name__ == "__main__":
    main()
