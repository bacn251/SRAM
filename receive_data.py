import serial
import time
import struct
import sys

# CẤU HÌNH
PORT = 'COM62'  # Thay đổi thành cổng COM của bạn
BAUDRATE = 115200 # USB CDC thường bỏ qua baudrate, nhưng cứ để
TIMEOUT = 2

def decode_24bit_signed(b3, b2, b1):
    """
    Chuyển đổi 3 bytes (Big Endian) thành signed 24-bit int.
    b3: MSB, b1: LSB
    """
    val = (b3 << 16) | (b2 << 8) | b1
    # Kiểm tra bit dấu (bit 23)
    if val & 0x800000:
        val -= 0x1000000
    return val

def main():
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)
        print(f"Connected to {PORT}")
    except serial.SerialException:
        print(f"Error: Could not open port {PORT}. Please check connection.")
        return

    # Xóa buffer cũ
    ser.reset_input_buffer()
    
    # Gửi lệnh
    cmd = "Send\n"
    print(f"Sending command: {cmd.strip()}")
    ser.write(cmd.encode('utf-8'))
    
    # Nhận dữ liệu
    print("Receiving data...")
    
    received_samples = []
    # start_time = time.time() # Removed absolute timeout
    last_data_time = time.time()
    remainder = b''
    
    try:
        while True:
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(waiting)
                
                # Combine with leftover from previous read
                data = remainder + chunk
                
                # Process full 3-byte groups
                num_samples = len(data) // 3
                process_len = num_samples * 3
                
                # Process these bytes
                for i in range(0, process_len, 3):
                    # Manual decode is faster than slicing repeatedly
                    val = (data[i] << 16) | (data[i+1] << 8) | data[i+2]
                    if val & 0x800000:
                        val -= 0x1000000
                    received_samples.append(val)
                
                # Save new remainder
                remainder = data[process_len:]
                
                last_data_time = time.time()
                
                if len(received_samples) % 1000 == 0:
                     sys.stdout.write(f"\rReceived: {len(received_samples)} samples")
                     sys.stdout.flush()
            else:
                # No data pending
                # Quit if we have data and it's been quiet for > 3 seconds
                if len(received_samples) > 0 and (time.time() - last_data_time > 3.0):
                     print("\nTransmission finished (Idle timeout).")
                     break
                
                # Quit if no data at all for 10 seconds
                if len(received_samples) == 0 and (time.time() - last_data_time > 10.0):
                     print("\nTimeout: No start of data.")
                     break
                     
                time.sleep(0.005) # Yield CPU
                # Loop continues until one of the above break conditions triggers
                         
    except KeyboardInterrupt:
        print("\nStopped by user.")
    
    print(f"\n\nTotal samples received: {len(received_samples)}")
    
    if received_samples:
        print("First 10 samples:", received_samples[:10])
        print("Last 10 samples:", received_samples[-10:])

        # Optional: Save to file
        with open('received_data.csv', 'w') as f:
            f.write("Index,Value,Hex\n")
            for i, val in enumerate(received_samples):
                # Convert back to 24-bit unsigned for hex display
                raw_hex = val & 0xFFFFFF
                f.write(f"{i},{val},{raw_hex:06X}\n")
        print("Data saved to 'received_data.csv'")

    ser.close()
    print("Done.")

if __name__ == "__main__":
    main()
