from machine import Pin, PWM, Timer, UART
import utime, time

machine.freq(200000000)

# Motor 1 depan kiri, 2 depan kanan, 3 belakang kiri, 4 belakang kanan
# Define GPIO pins for each encoder channel
encoder_channels = [(8, 9), (27, 26), (10, 11), (21, 20)]  # Each tuple contains (channel_A, channel_B) pins for an encoder
# Define driver pins for motors
motor_pins = [(6, 7), (19, 18), (14, 15), (17, 16)]  # Each tuple contains (RPWM, LPWM) pins for a motor

# Set up the GPIO pins as outputs for motors
motors = []
for rpwm_pin, lpwm_pin in motor_pins:
    rpwm = Pin(rpwm_pin, Pin.OUT)
    lpwm = Pin(lpwm_pin, Pin.OUT)
    motors.append((PWM(rpwm), PWM(lpwm)))

# Set PWM frequency and initial duty cycle for all motors
for rpwm, lpwm in motors:
    rpwm.freq(10000)
    lpwm.freq(10000)
    rpwm.duty_u16(0)
    lpwm.duty_u16(0)

encoder_counts = [0, 0, 0, 0]

# Setup GPIO pins as inputs with pull-down resistors for encoders
encoders = []
encoder_handlers = []

def handle_encoder(encoder_index):
    global encoder_counts

    encoder_a, encoder_b = encoders[encoder_index]
    current_encoder_state = (encoder_a.value() << 1) | encoder_b.value()
    if current_encoder_state == 0b00 and previous_encoder_states[encoder_index] == 0b01:
        encoder_counts[encoder_index] -= 1
    elif current_encoder_state == 0b00 and previous_encoder_states[encoder_index] == 0b10:
        encoder_counts[encoder_index] += 1
    previous_encoder_states[encoder_index] = current_encoder_state

# Set interrupt for encoder reading
previous_encoder_states = [0, 0, 0, 0]
for i in range(len(encoder_channels)):
    channel_A, channel_B = encoder_channels[i]
    encoder_a = Pin(channel_A, Pin.IN, Pin.PULL_DOWN)
    encoder_b = Pin(channel_B, Pin.IN, Pin.PULL_DOWN)
    encoders.append((encoder_a, encoder_b))

    def create_handler(index):
        def handler(pin):
            handle_encoder(index)
        return handler

    encoder_handler = create_handler(i)
    encoder_handlers.append(encoder_handler)
    encoder_a.irq(handler=encoder_handler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    encoder_b.irq(handler=encoder_handler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)

def tahan_waktu(waktu):
    myTime = utime.ticks_ms()
    flag = True
    while flag:
        currentTime = utime.ticks_ms()
        if utime.ticks_diff(currentTime, myTime) < waktu:
            flag = True
        else:
            flag = False
    flag = True  # reset flag to true for reuse

def stop_motors():
    for rpwm, lpwm in motors:
        rpwm.duty_u16(0)
        lpwm.duty_u16(0)

def drive_motor(motor_index, R, L):
    rpwm, lpwm = motors[motor_index]
    rpwm.duty_u16(R)
    lpwm.duty_u16(L)
    # kalau 256 itu 8 bit, kalau 16 bit 65535

def case_maju():
    pwm_value = 30000  # Nilai PWM untuk menggerakkan motor maju
    drive_motor(0, 0, pwm_value)
    drive_motor(1, 0, pwm_value)
    drive_motor(2, 0, pwm_value)
    drive_motor(3, 0, pwm_value)

def case_mundur():
    pwm_value = 30000  # Nilai PWM untuk menggerakkan motor mundur
    drive_motor(0, pwm_value, 0)
    drive_motor(1, pwm_value, 0)
    drive_motor(2, pwm_value, 0)
    drive_motor(3, pwm_value, 0)

# Main program loop
while True:
    case_maju()
    tahan_waktu(10000)  # Maju selama 10 detik
    stop_motors()
    tahan_waktu(1000)   # Berhenti sebentar selama 1 detik
    case_mundur()
    tahan_waktu(10000)  # Mundur selama 10 detik
    stop_motors()
    tahan_waktu(1000)   # Berhenti sebentar selama 1 detik
