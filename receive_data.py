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
    start_time = time.time()
    
    try:
        while True:
            # Đọc 3 bytes (vì MCU gửi 3 bytes mỗi mẫu)
            data = ser.read(3)
            
            if len(data) == 3:
                # Decode
                sample = decode_24bit_signed(data[0], data[1], data[2])
                received_samples.append(sample)
                
                # In thử một vài mẫu đầu để debug
                if len(received_samples) <= 10:
                    print(f"Sample {len(received_samples)}: {sample}")
                elif len(received_samples) % 1000 == 0:
                     sys.stdout.write(f"\rReceived: {len(received_samples)} samples")
                     sys.stdout.flush()
            else:
                # Timeout hoặc hết dữ liệu
                if len(received_samples) > 0:
                    # Nếu đã nhận được dữ liệu mà bị timeout -> coi như xong
                    break
                else:
                    # Chưa nhận được gì
                    if time.time() - start_time > 5:
                         print("\nTimeout: No data received.")
                         break
                         
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
