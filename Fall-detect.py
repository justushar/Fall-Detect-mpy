import machine
import utime
import sys
from ssd1306 import SSD1306_I2C
import writer
import freesans20
import framebuf
# Define the I2C address of the MPU6500
address = 0x68

speaker_pin = machine.Pin(15)
pwm = machine.PWM(speaker_pin)

# Initialize the I2C bus
i2c = machine.SoftI2C(scl=machine.Pin(13), sda=machine.Pin(12), freq=400000)
i2c1=machine.SoftI2C(scl=machine.Pin(1),sda=machine.Pin(0),freq=400000)
display=SSD1306_I2C(128,64,i2c1)
display.fill(0)
# Define the notes and their corresponding frequencies
notes = {
    'H': 784,
    'E': 659,
    'L': 587,
    'P': 523
}
# Read raw values from the MPU6500
def read_raw_values():
    data = i2c.readfrom_mem(address, 0x3B, 14)
    accel_x = (data[0] << 8) + data[1]
    accel_y = (data[2] << 8) + data[3]
    accel_z = (data[4] << 8) + data[5]
    temp = (data[6] << 8) + data[7]
    gyro_x = (data[8] << 8) + data[9]
    gyro_y = (data[10] << 8) + data[11]
    gyro_z = (data[12] << 8) + data[13]
    return accel_x, accel_y, accel_z, temp, gyro_x, gyro_y, gyro_z

# Calibrate the sensor by averaging readings
def calibrate_sensor(count=256, delay=0):
    accel_sum = [0.0, 0.0, 0.0]
    gyro_sum = [0.0, 0.0, 0.0]

    for _ in range(count):
        accel_x, accel_y, accel_z, _, gyro_x, gyro_y, gyro_z = read_raw_values()

        accel_sum[0] += accel_x
        accel_sum[1] += accel_y
        accel_sum[2] += accel_z

        gyro_sum[0] += gyro_x
        gyro_sum[1] += gyro_y
        gyro_sum[2] += gyro_z

        utime.sleep_ms(delay)

    accel_offset = [accel_sum[0] / count, accel_sum[1] / count, accel_sum[2] / count]
    gyro_offset = [gyro_sum[0] / count, gyro_sum[1] / count, gyro_sum[2] / count]

    return accel_offset, gyro_offset

# Convert raw values to corresponding units
def convert_raw_to_units(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
    ax = (accel_x - 2050) / 16384.00
    ay = (accel_y - 77) / 16384.00
    az = (accel_z - 1947) / 16384.00
    gx = (gyro_x + 270) / 131.07
    gy = (gyro_y - 351) / 131.07
    gz = (gyro_z + 136) / 131.07
    return ax, ay, az, gx, gy, gz

# Calculate amplitude vector for 3 axes
def calculate_amplitude(ax, ay, az):
    raw_amp = (ax**2 + ay**2 + az**2)**0.5
    amp = raw_amp * 10
    return amp

# Print the filtered values and detect falls
def print_filtered_values():
    global trigger1, trigger2, trigger3, fall, trigger1count, trigger2count, trigger3count

    accel_x, accel_y, accel_z, _, gyro_x, gyro_y, gyro_z = read_raw_values()
    ax, ay, az, gx, gy, gz = convert_raw_to_units(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z)
    amp = calculate_amplitude(ax, ay, az)
    print(amp)
    if amp <= 54 and not trigger2:
        trigger1 = True
        print("TRIGGER 1 ACTIVATED")

    if trigger1:
        trigger1count += 1
        if amp >= 12:
            trigger2 = True
            print("TRIGGER 2 ACTIVATED")
            trigger1 = False
            trigger1count = 0

    if trigger2:
        trigger2count += 1
        angleChange = (gx**2 + gy**2 + gz**2)**0.5
        #print(angleChange)
        if 700 <= angleChange <= 800:
            trigger3 = True
            trigger2 = False
            trigger2count = 0
            print(angleChange)
            print("TRIGGER 3 ACTIVATED")

    if trigger3:
        trigger3count += 1
        if trigger3count >= 10:
            angleChange = (gx**2 + gy**2 + gz**2)**0.5
            print(angleChange)
            if 620 <= angleChange <= 856:
                fall = True
                trigger3 = False
                trigger3count = 0
                print(angleChange)
            else:
                trigger3 = False
                trigger3count = 0
                print("TRIGGER 3 DEACTIVATED")

    if fall:
        print("FALL DETECTED")
        display.fill(0)
        TH=bytearray(b'\xfc\x00\x00\x00p\x00\x00\x1f\xfc\x00\x00\x00\xfc\x00\x00\x1f\xfc\x00\x00\x00\xfc\x00\x00\x1f\xfc\x00\x00\x00\xfc\x00\x00\x1f\xfc\x00\x00\x00\xfe\x00\x00\x1f\xfc\x00\x00\x00\xfe\x00\x00\x1f\xfc\x00\x00\x00~\x00\x00\x1f\xfc\x00\x00\x00\x7f\x00\x00\x1f\xfc\x00\x00\x00?\x00\x00\x1f\xfc\x00\x00\x00?\x004\x1f\xfc\x00\x00\x00?\x80\xfe\x1f\xfc\x00\x00\x00\x1f\x83\xff\x9f\xfc\x00\x00\x00\x1f\xc3\xff\x9f\xfc\x00\x00\x00\x1f\xc7\xff\xdf\xfc\x00\x00\x00\x0f\xc7\xff\xdf\xfc\x00\x00\x00\x0f\xe7\xff\xdf\xfc\x00\x00\x00\x07\xe7\xff\xdf\xfc\x00\x00\x00\x07\xf7\xff\xdf\xfc\x00\x00\x00\x07\xf3\xff\x9f\xfc\x00\x00\x00\x03\xf3\xff\x9f\xfc\x00\x00\x00\x03\xf9\xff\x1f\xfc\x00\x00\x00\x03\xf8\xfe\x1f\xfc\x00\x00\x00\x01\xfe\x10\x1f\xfc\x00\x00\x00\x01\xff\x80\x1f\xfc\x00\x00\x00\x00\xff\xe0\x1f\xfc\x00\x00\x00\x00\xff\xe0\x1f\xfc\x00\x00\x00\x01\xff\xf0\x1f\xfc\x00\x00\x00\x01\xff\xf8\x1f\xfc\x00\x00\x00\x01\xff\xf8\x1f\xfc\x00\x00\x00\x03\xff\xfc\x1f\xfc\x00\x03\x00\x07\xff\xfc\x1f\xfc\x00\x1f\xe0\x07\xff\xfe\x1f\xfc\x00\x7f\xf8\x1f\xff\xff\x1f\xfc\x03\xff\xfe?\xfe\x7f\x1f\xfc\x0f\xff\xff\xff\xfe?\x9f\xfc\x7f\xff\xff\xff\xfc?\x9f\xfd\xff\xff\xff\xff\xfc\x1f\xdf\xff\xff\xfc\xff\xff\xf8\x1f\xdf\xff\xff\xe0?\xff\xf8\x0f\xdf\xff\xff\x80\x0f\xff\xf0\x0f\xdf\xff\xfc\x00\x03\xff\xf0\x0f\xff\xfd\xf0\x00\x00\xff\xe0\x0f\xdf\xfc\x00\x00\x00\x7f\xe0\x07\xff\xfc\x00\x00\x01\xff\xc0\x0f\xdf\xfc\x00\x00\x03\xff\xc0\x07\xff\xfc\x00\x00\x0f\xff\x00\x0f\xff\xfc\x00\x00\x1f\xfe\x00\x07\xff\xfc\x00\x00\x7f\xf8\x00\x07\xff\xfc\x00\x00\xff\xf0\x00\x0f\xff\xfc\x00\x03\xff\xc0\x00\x07\xff\xfc\x00\x07\xff\x80\x00\x03\xdf\xfc\x00\x1f\xfe\x00\x00\x01\x1f\xfc\x00?\xfc\x00\x00\x00\x1f\xfc\x00\x7f\xf0\x0e\x00\x00\x1f\xfc\x00\xff\xe0>\x00\x00\x1f\xfc\x00\x7f\x81\xf4\x00\x00\x1f\xfc\x00\xff\x0f@\x00\x00\x1f\xfc\x00| \x03\xc0\x00\x1f\xfc\x004\x00\xbf\xe0\x00\x1f\xfc\x00\x00/\xfe\xc0\x00\x1f\xfc\x00\x00\x00\x00\x00\x00\x1f\xfc\x00\x00\x12\x00\x00\x00\x1f\xfc\x00\x00\x0b\xff\x00\x00\x1f\xfc\x00\x00\x00\x1f\x00\x00\x1f')
        fb = framebuf.FrameBuffer(TH,64,64, framebuf.MONO_HLSB)
        display.blit(fb,32,0)
        display.show()
        play_help_sound()
        utime.sleep(5)
        display.fill(0)
        font_writer=writer.Writer(display,freesans20)
        font_writer.set_textpos(5,30)
        font_writer.printstring("Fall Detected")
        display.show()
        utime.sleep(5)
        sys.exit()
        # Perform necessary actions for fall detection

    if trigger2count >= 6:
        trigger2 = False
        trigger2count = 0
        print("TRIGGER 2 DEACTIVATED")

    if trigger1count >= 6:
        trigger1 = False
        trigger1count = 0
        print("TRIGGER 1 DEACTIVATED")

    utime.sleep_ms(100)

# Initialize sensor calibration
accel_offset, gyro_offset = calibrate_sensor()

# Initialize fall detection variables
trigger1 = False
trigger2 = False
trigger3 = False
fall = False
trigger1count = 0
trigger2count = 0
trigger3count = 0

# Function to play a note for a given duration
def play_note(note, duration):
    frequency = notes.get(note, 0)
    if frequency > 0:
        pwm.freq(frequency)
        pwm.duty_u16(32768)  # Set the duty cycle to 50% (32768/65536)
        utime.sleep_ms(duration)
        pwm.duty_u16(0)  # Turn off the PWM

# Play the "HELP" sound
def play_help_sound():
    melody = [
        ('H', 500), ('E', 500), ('L', 500), ('P', 1000)
    ]
    for note, duration in melody:
        play_note(note, duration)


# Main loop for fall detection
while True:
    print_filtered_values()
