import serial
import struct
import numpy as np

ser = serial.Serial('COM10', 115200, timeout=0)

def read_packets():
    buffer = b''
    data = []

    try:
        while True:
            buffer += ser.read(ser.in_waiting or 1)

            while len(buffer) >= 38:
                start = buffer.find(b'\xAA\x55')
                if start == -1:
                    buffer = b''
                    break

                if len(buffer) < start + 38:
                    break

                packet = buffer[start+2:start+38]
                values = struct.unpack('<9f', packet)
                data.append(values)

                buffer = buffer[start+38:]

    except KeyboardInterrupt:
        print("Capture stopped.")
        return np.array(data)

    finally:
        ser.close()

# Run the capture
data_array = read_packets()

# Also save as a CSV for easy viewing in Excel/plots
np.savetxt('analog_sine_100mHzv2.csv', data_array, delimiter=',', header='y_measured,v_filtered,x1m,x2m,error,u,theta1,theta2,uc', comments='')

print(f"Saved {data_array.shape[0]} samples.")
