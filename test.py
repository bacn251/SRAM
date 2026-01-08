import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from collections import deque
import struct
import time
from datetime import datetime # Thêm thư viện ngày tháng

# --- CẤU HÌNH ---
COM_PORT = 'COM62' 
BAUD_RATE = 115200  
MAX_POINTS = 100000  
VIEW_WINDOW = 5000   

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
    print(f"Lỗi kết nối: {e}")
    exit()

# Thiết lập đồ thị
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(bottom=0.2) 

line, = ax.plot([], [], label='Dữ liệu STM32', color='red', linewidth=1)
ax.set_ylim(0, 1200) 
ax.set_xlabel('Mẫu số')
ax.set_ylabel('Giá trị')
ax.grid(True)

# Hàm xử lý khi nhấn nút "Record"
def start_record(event):
    global current_index, log_file
    
    # 1. Tạo tên file theo ngày giờ: Data_YYYYMMDD_HHMMSS.txt
    now = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    file_name = f"Data_{timestamp}.txt"
    
    # 2. Đóng file cũ nếu đang mở
    if log_file:
        log_file.close()
    
    # 3. Mở file mới
    log_file = open(file_name, "w")
    print(f"Bắt đầu ghi vào file mới: {file_name}")
    
    # 4. Reset đồ thị
    data_x.clear()
    data_y.clear()
    current_index = 0
    
    # 5. Gửi lệnh cho STM32
    ser.write(b"Record\n") 

# Cấu hình nút bấm
ax_button = plt.axes([0.35, 0.05, 0.3, 0.075])
btn_record = Button(ax_button, 'Gửi Record & Lưu File Mới', color='lightgreen', hovercolor='green')
btn_record.on_clicked(start_record)

def update(frame):
    global current_index, log_file
    
    try:
        waiting = ser.in_waiting
        if waiting >= 4:
            bytes_to_read = (waiting // 4) * 4
            raw_data = ser.read(bytes_to_read)
            
            num_floats = bytes_to_read // 4
            fmt = f'<{num_floats}f'
            values = struct.unpack(fmt, raw_data)
            
            # Ghi vào file nếu file đang mở
            if log_file:
                # Tạo chuỗi dữ liệu (float với 3 chữ số thập phân)
                lines = "\n".join([f"{v:.3f}" for v in values]) + "\n"
                log_file.write(lines)

            # Cập nhật mảng để vẽ
            for v in values:
                data_y.append(v)
                data_x.append(current_index)
                current_index += 1
            
            # Cập nhật đồ thị
            line.set_data(data_x, data_y)
            if len(data_x) > 0:
                last_x = data_x[-1]
                ax.set_xlim(max(0, last_x - VIEW_WINDOW), last_x + 100)
                
    except Exception as e:
        print(f"Lỗi khi nhận dữ liệu: {e}")

    return line,

# Animation
ani = FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)

print("--- PHẦN MỀM NHẬN DỮ LIỆU ---")
print("Nhấn nút trên cửa sổ đồ thị để bắt đầu mỗi lần ghi.")

try:
    plt.show()
finally:
    if log_file:
        log_file.close()
    ser.close()
    print("Đã đóng kết nối và file.")