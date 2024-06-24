import serial
import time


### Bluetooth
bluetooth_port = 'COM5'  # Replace 'COMx' with the actual COM port of your Bluetooth module
baud_rate = 9600

try:
    ser = serial.Serial(bluetooth_port, baud_rate, timeout=1)
    time.sleep(2)  # Allow time for the Arduino to reset after establishing the serial connection
    print(f"Connected to {bluetooth_port} at {baud_rate} baud.")
except serial.SerialException:
    print(f"Failed to connect to {bluetooth_port}. Check the COM port and try again.")
    exit()



def receive_and_send_array():
    while True:
        # Eingabe des Arrays vom Benutzer über die Konsole
        array_str = input("Geben Sie die Werte des Arrays durch Komma getrennt ein (oder 'exit' zum Beenden): ")
        
        if array_str.lower() == 'exit':
            break
        
        array = [int(x) for x in array_str.split(',')]
        
        try:
            # Array in eine Zeichenfolge konvertieren und an Arduino senden
            array_str = ','.join(map(str, array))
            ser.write(array_str.encode())
            
            # Startzeit für die Zeitmessung
            start_time = time.time()
            
            # Warten auf eine Antwort vom Arduino für maximal 30 Sekunden
            while (time.time() - start_time) < 30:  # Maximal 30 Sekunden warten
                if ser.in_waiting > 0:
                    received_value = int(ser.readline().decode().strip())
                    print("Empfangener Integer-Wert vom Arduino:", received_value)
                    break  # Aus der Schleife ausbrechen, wenn Daten empfangen wurden
            
            # Überprüfen, ob keine Daten empfangen wurden
            if (time.time() - start_time) >= 30:
                print("Keine Daten vom Arduino empfangen.")
                
        except serial.SerialException as e:
            print(f"Fehler beim Senden/Empfangen über die serielle Verbindung. Error: {e}")


if __name__ == "__main__":
    receive_and_send_array()
    
    