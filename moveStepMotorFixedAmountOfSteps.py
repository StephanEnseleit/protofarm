import serial
import time

bluetooth_port = 'COM5'  # Replace 'COMx' with the actual COM port of your Bluetooth module
baud_rate = 9600

try:
    ser = serial.Serial(bluetooth_port, baud_rate, timeout=1)
    time.sleep(2)
    print(f"Connected to {bluetooth_port} at {baud_rate} baud.")
except serial.SerialException:
    print(f"Failed to connect to {bluetooth_port}. Check the COM port and try again.")
    exit()

def send_command(command):
    ser.write(f"{command}\n".encode())
    print(f"Sent {command} steps to Arduino")

while True:
    user_input = input("Enter the number of steps to move the stepper motor (or 'exit' to quit): ")
    if user_input.lower() == 'exit':
        break
    try:
        send_command(user_input)
        response = ser.readline().decode().strip()
        print(f"Arduino Response: {response}")
    except ValueError:
        print("Invalid input. Please enter a valid command ('h' or 'e') or 'exit'.")

ser.close()