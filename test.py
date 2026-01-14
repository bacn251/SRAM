import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from collections import deque
import struct
import time
import numpy as np
from datetime import datetime
import threading
import queue

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

# Queue để chuyển dữ liệu từ Thread đọc sang Thread vẽ
data_queue = queue.Queue()
is_recording = False
read_thread = None

# Kết nối Serial
try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
    # Cố gắng tăng buffer nếu HĐH cho phép (Windows có thể cần set trong Device Manager)
    ser.set_buffer_size(rx_size=100000, tx_size=128000) 
    print(f"Đang kết nối với {COM_PORT}...")
    time.sleep(1) 
except Exception as e:
    # set_buffer_size có thể lỗi trên một số version/OS, cứ ignore nếu lỗi
    pass 
except:
    print(f"Lỗi nối Serial")

# Thiết lập đồ thị
fig, (ax_time, ax_fft) = plt.subplots(2, 1, figsize=(10, 8))
plt.subplots_adjust(bottom=0.15, hspace=0.4) 

line_time, = ax_time.plot([], [], color='red', linewidth=1)
ax_time.set_ylim(-100000000, 100000000) 
ax_time.set_title("Đồ thị thời gian (Time Domain)")
ax_time.grid(True)

line_fft, = ax_fft.plot([], [], color='blue', linewidth=1)
peak_dot, = ax_fft.plot([], [], 'ro', markersize=5, label='Peak') 
ax_fft.set_title("Phân tích phổ tần số (FFT)")
ax_fft.grid(True)

text_info = fig.text(0.02, 0.02, '', fontsize=10, color='blue', fontweight='bold')

def serial_read_loop():
    global log_file, is_recording
    remainder = b''
    
    print("Read thread started.")
    while is_recording:
        try:
            waiting = ser.in_waiting
            if waiting > 0:
                chunk = ser.read(waiting)
                data = remainder + chunk
                
                num_samples = len(data) // 3
                if num_samples > 0:
                    process_len = num_samples * 3
                    raw_data = data[:process_len]
                    remainder = data[process_len:]
                    
                    values = []
                    for i in range(0, process_len, 3):
                        val = (raw_data[i] << 16) | (raw_data[i+1] << 8) | raw_data[i+2]
                        if val & 0x800000:
                            val -= 0x1000000
                        values.append(val)
                    
                    # Ghi file ngay trong thread đọc để đảm bảo tốc độ
                    if log_file:
                        log_file.write("\n".join([f"{v:.3f}" for v in values]) + "\n")
                    
                    # Đẩy sang GUI
                    for v in values:
                         data_queue.put(v)
                else:
                    remainder = data
            else:
                time.sleep(0.001) # Ngủ cực ngắn để không chiếm 100% CPU
        except Exception as e:
            print(f"Error in read thread: {e}")
            break
    print("Read thread stopped.")

def start_record(event):
    global current_index, log_file, is_recording, read_thread
    if is_recording: return # Đang chạy thì thôi
    
    now = datetime.now()
    file_name = f"Data_{now.strftime('%Y%m%d_%H%M%S')}.txt"
    if log_file: log_file.close()
    log_file = open(file_name, "w")
    
    data_x.clear()
    data_y.clear()
    current_index = 0
    # Xả queue cũ
    while not data_queue.empty(): data_queue.get()
    
    ser.reset_input_buffer()
    ser.write(b"Send\n") 
    print(f"Bắt đầu ghi: {file_name}")
    
    is_recording = True
    read_thread = threading.Thread(target=serial_read_loop, daemon=True)
    read_thread.start()

def stop_record(event):
    global is_recording, log_file
    is_recording = False
    if log_file:
        log_file.close()
        log_file = None
    print("Đã dừng ghi.")

ax_btn_start = plt.axes([0.6, 0.02, 0.15, 0.05])
btn_start = Button(ax_btn_start, 'Start', color='lightgreen')
btn_start.on_clicked(start_record)

ax_btn_stop = plt.axes([0.8, 0.02, 0.15, 0.05])
btn_stop = Button(ax_btn_stop, 'Stop', color='salmon')
btn_stop.on_clicked(stop_record)

def update(frame):
    global current_index
    
    points_added = 0
    # Lấy dữ liệu từ queue (tối đa 2000 điểm mỗi frame để tránh lag GUI)
    while not data_queue.empty() and points_added < 2000:
        val = data_queue.get()
        data_y.append(val)
        data_x.append(current_index)
        current_index += 1
        points_added += 1
    
    if points_added > 0:
        line_time.set_data(data_x, data_y)
        if len(data_x) > 0:
            ax_time.set_xlim(max(0, data_x[-1] - VIEW_WINDOW), data_x[-1] + 100)

        # FFT Logic (giữ nguyên, chỉ chạy nếu có dữ liệu mới)
        if len(data_y) > 100:
            y_array = np.array(data_y)
            y_ac = y_array - np.mean(y_array) 
            N = len(y_ac)
            fs = N / MEASURE_TIME 
            yf = np.fft.rfft(y_ac)
            xf = np.fft.rfftfreq(N, 1/fs)
            amplitude = (2.0/N) * np.abs(yf)
            
            idx_start = 2 
            if len(amplitude) > idx_start:
                peak_idx = np.argmax(amplitude[idx_start:]) + idx_start
                peak_freq = xf[peak_idx]
                peak_amp = amplitude[peak_idx]
                
                line_fft.set_data(xf, amplitude)
                peak_dot.set_data([peak_freq], [peak_amp]) 
                ax_fft.set_xlim(0, fs)
                ax_fft.set_ylim(0, max(amplitude) * 1.2 if max(amplitude) > 0 else 1)
                
                vrms = np.sqrt(np.mean(y_array**2))
                text_info.set_text(f"Vrms: {vrms:.3f} | Peak Freq: {peak_freq:.2f} Hz | Amp: {peak_amp:.3f}")

    return line_time, line_fft, peak_dot, text_info

ani = FuncAnimation(fig, update, interval=20, blit=False, cache_frame_data=False)
plt.show()

if log_file: log_file.close()
ser.close()
is_recording = False