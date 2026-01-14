import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from collections import deque
import struct
import time
import numpy as np
from datetime import datetime

# --- CẤU HÌNH ---
COM_PORT = 'COM62' 
BAUD_RATE = 115200  
MAX_POINTS = 500000  
VIEW_WINDOW = 5000   
MEASURE_TIME = 1.0   

# Khởi tạo dữ liệu
data_x = deque(maxlen=MAX_POINTS)
data_y = deque(maxlen=MAX_POINTS)
current_index = 0
log_file = None 

# Kết nối Serial
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
    print(f"Đang kết nối với {COM_PORT}...")
    time.sleep(1) 
except Exception as e:
    print(f"Lỗi kết nối: {e}"); exit()

# Thiết lập đồ thị
fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(10, 8))
plt.subplots_adjust(bottom=0.15, hspace=0.4) 

line_time, = ax_time.plot([], [], color='red', linewidth=1)
ax_time.set_ylim(-100000000, 100000000) 
ax_time.set_title("Đồ thị thời gian (Time Domain)")
ax_time.grid(True)

line_fft, = ax_fft.plot([], [], color='blue', linewidth=1)
# Điểm đánh dấu đỉnh FFT
peak_dot, = ax_fft.plot([], [], 'ro', markersize=5, label='Peak') 
ax_fft.set_title("Phân tích phổ tần số (FFT)")
ax_fft.grid(True)

# Text hiển thị thông số Vrms và Peak Freq
text_info = fig.text(0.02, 0.02, '', fontsize=10, color='blue', fontweight='bold')

def start_record(event):
    global current_index, log_file
    now = datetime.now()
    file_name = f"Data_{now.strftime('%Y%m%d_%H%M%S')}.txt"
    if log_file: log_file.close()
    log_file = open(file_name, "w")
    data_x.clear()
    data_y.clear()
    current_index = 0
    ser.write(b"Send\n") 
    print(f"Bắt đầu ghi: {file_name}")

ax_button = plt.axes([0.7, 0.02, 0.2, 0.05])
btn_record = Button(ax_button, 'Gửi Record & Lưu', color='lightgreen')
btn_record.on_clicked(start_record)

def update(frame):
    global current_index, log_file
    
    try:
        waiting = ser.in_waiting
        if waiting >= 3:
            bytes_to_read = (waiting // 3) * 3
            raw_data = ser.read(bytes_to_read)
            # Parse 24-bit Big Endian integers using simple loop (or numpy for speed)
            # Since performance matters for plotting, try a valid approach:
            
            # METHOD: Manual bit reconstruction
            # We have raw bytes.
            values = []
            for i in range(0, len(raw_data), 3):
                b2 = raw_data[i]     # MSB
                b1 = raw_data[i+1]
                b0 = raw_data[i+2]   # LSB
                val = (b2 << 16) | (b1 << 8) | b0
                if val & 0x800000:
                    val -= 0x1000000
                values.append(val)
            
            # (Optional: conversion to numpy array if desired, but append loop works for deque)
            # values = ...
            
            if log_file:
                log_file.write("\n".join([f"{v:.3f}" for v in values]) + "\n")

            for v in values:
                data_y.append(v)
                data_x.append(current_index)
                current_index += 1
            
            line_time.set_data(data_x, data_y)
            if len(data_x) > 0:
                ax_time.set_xlim(max(0, data_x[-1] - VIEW_WINDOW), data_x[-1] + 100)

            # # --- TÍNH TOÁN FFT & TÌM ĐỈNH ---
            if len(data_y) > 100:
                y_array = np.array(data_y)
                y_ac = y_array - np.mean(y_array) # Loại bỏ DC
                N = len(y_ac)
                fs = N / MEASURE_TIME 
                
                yf = np.fft.rfft(y_ac)
                xf = np.fft.rfftfreq(N, 1/fs)
                amplitude = (2.0/N) * np.abs(yf)
                
                # TÌM ĐỈNH (PEAK)
                # Bỏ qua thành phần tần số cực thấp sát 0Hz nếu cần
                idx_start = 2 
                if len(amplitude) > idx_start:
                    peak_idx = np.argmax(amplitude[idx_start:]) + idx_start
                    peak_freq = xf[peak_idx]
                    peak_amp = amplitude[peak_idx]
                    
                    # Cập nhật đồ thị FFT
                    line_fft.set_data(xf, amplitude)
                    peak_dot.set_data([peak_freq], [peak_amp]) # Vẽ dấu chấm tại đỉnh
                    
                    ax_fft.set_xlim(0, fs)
                    ax_fft.set_ylim(0, max(amplitude) * 1.2 if max(amplitude) > 0 else 1)
                    
                    # Tính Vrms
                    vrms = np.sqrt(np.mean(y_array**2))
                    
                    # Hiển thị thông tin
                    text_info.set_text(
                        f"Vrms: {vrms:.3f} | Peak Freq: {peak_freq:.2f} Hz | Amp: {peak_amp:.3f}"
                    )
                
    except Exception as e:
        print(f"Lỗi: {e}")

    return line_time, line_fft, peak_dot, text_info

ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
plt.show()

if log_file: log_file.close()
ser.close()