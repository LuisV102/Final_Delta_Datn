import tkinter as tk
from tkinter import messagebox
import serial
import time
import threading
import FunctionButton as bh
from PIL import Image, ImageTk
import ObjectDetection as od

home_set = False

# Setup Serial
try:
    ser = serial.Serial('COM4', 9600, timeout=1, write_timeout=1)
    time.sleep(1)
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    ser = None

# Giao diện chính
window = tk.Tk()
window.title("GUI")
window.configure(bg="#f0f0f5")
# window.geometry("1200x850") # đặt kích thước cửa sổ cố định

font_title = ("Arial", 20, "bold")
font_label = ("Arial", 12)
font_entry = ("Arial", 12)
font_button = ("Arial", 12, "bold")

controllable_buttons = []
mode_var = tk.StringVar(value="manual")

def toggle_mode():
    mode = mode_var.get()
    # new_state = tk.DISABLED if mode == "auto" else tk.NORMAL
    new_state_manual_buttons = tk.DISABLED  # Mặc định là vô hiệu hóa nút manual khi ở auto

    if mode == "manual":
        radio_manual.config(relief=tk.SUNKEN, bg="#f4f716") #SUNKEN: nhấn xuống
        radio_auto.config(relief=tk.RAISED, bg="#f0f0f5") # nút auto có trạng thái nổi lên
        new_state_manual_buttons = tk.NORMAL  # Kích hoạt nút manual
        # Nếu đang ở manual, có thể bạn muốn dừng camera nếu nó đang chạy từ chế độ auto
        bh.stop_camera_stream_handler(label_cam) # Tùy chọn: dừng camera khi chuyển về manual
        od.set_operation_mode(False)
    else:  # auto mode
        radio_auto.config(relief=tk.SUNKEN, bg="#f4f716")
        radio_manual.config(relief=tk.RAISED, bg="#f0f0f5")
        radio_manual.config(state=tk.DISABLED)
        radio_auto.config(state=tk.DISABLED)
        btn_namcham.config(bg="#eb3b3b")
        btn_bangtai.config(bg="#eb3b3b")
        od.set_operation_mode(True)
        # Truyền hàm gửi serial, widget label camera, và đối tượng serial
        bh.run_auto_mode_sequence(send_command_to_serial, label_cam, ser)

    for btn in controllable_buttons: # duyệt qua từng nút trong danh sách controllable_buttons
        if btn:
        # if btn in [btn_run, button_traj, btn_calc_ik, btn_namcham, btn_bangtai]:
            try:
                # btn.config(state=new_state)
                btn.config(state=new_state_manual_buttons)
            except tk.TclError:
                pass

def send_command_to_serial(command_str):
    if ser and ser.is_open:
        try:
            ser.write(command_str.encode())
            # print(f"Gửi: {command_str.strip()}")
            return True
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Error writing to serial port: {e}")
            return False
    else:
        # messagebox.showwarning("Serial Port Error", "Serial port not available.") # Can be noisy
        print("Serial port not available for sending command.")
        return False

def start_all():
    global home_set
    if send_command_to_serial('start\r'):
        home_set = False  # Reset trạng thái home
        btn_home.config(state=tk.NORMAL)
        radio_manual.config(state=tk.DISABLED)
        radio_auto.config(state=tk.DISABLED)
        btn_startall.config(state=tk.DISABLED)  # 🔒 Khóa nút START
        # Khóa toàn bộ nút khác (ngoại trừ START và HOME)
        for btn in controllable_buttons:
            if btn != btn_home and btn != btn_startall:
                # chỉ vô hiệu hóa các nút khác, trừ sethome và start
                btn.config(state=tk.DISABLED)

def stop_robot():
    global home_set
    if send_command_to_serial('s\r'):
        home_set = False
        # Cho phép nhấn lại nút START
        btn_startall.config(state=tk.NORMAL)
        radio_manual.config(state=tk.DISABLED)
        radio_auto.config(state=tk.DISABLED)
        btn_start_cam.config(state=tk.DISABLED)
        btn_stop_cam.config(state=tk.DISABLED)
        bh.stop_camera_stream_handler(label_cam)
        btn_namcham.config(text="ON MAG",bg="#eb3b3b")
        btn_bangtai.config(text="ON CONV",bg="#eb3b3b")
        od.reset_object_memory()
        # Chuyển chế độ về manual
        mode_var.set("manual")
        toggle_mode()
        # Vô hiệu hóa các nút điều khiển (sẽ bật lại sau khi home)
        for btn in controllable_buttons:
            btn.config(state=tk.DISABLED)
        btn_home.config(state=tk.DISABLED)

def handle_set_home():
    success = bh.set_home_handler(send_command_to_serial, entry_x, entry_y, entry_z)
    if success:
        global home_set
        home_set = True
        # Kích hoạt lại các nút sau khi đã set home
        for btn in controllable_buttons:
            btn.config(state=tk.NORMAL)
        radio_manual.config(state=tk.NORMAL)
        radio_auto.config(state=tk.NORMAL)
        btn_start_cam.config(state=tk.NORMAL)
        btn_stop_cam.config(state=tk.NORMAL)

def read_serial():
    while True:
        if ser and ser.is_open and ser.in_waiting > 0:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').rstrip()
                # rstrip: xóa khoảng trắng || ser.readline(): đọc dữ liệu từ cổng COM
                # python nhận dữ liệu từ serial nên phải giải mã từ bytes sang chuỗi, dùng utf-8 là chuẩn mã hóa kí tự
                if line:
                    # print(f"Nhận: {line}")
                    window.after(0, lambda l=line: (
                        text_box.insert(tk.END, l + "\n"), # \n là xuống dòng
                        text_box.see(tk.END) # cuộn text box xuống để thấy dòng dữ liệu newest
                    ))
            except serial.SerialException as e:
                print(f"Serial read error: {e}");
                break
            except Exception as e:
                print(f"Error processing serial data: {e}")
        time.sleep(0.1)


cap = None
camera_running = False

magnet_state = 0
conveyor_state = 0

def toggle_namcham_wrapper():
    global magnet_state # Cần global để sửa giá trị
    new_state = bh.toggle_namcham_handler(magnet_state, btn_namcham, send_command_to_serial)
    if new_state is not None: # Nếu handler trả về trạng thái mới (tức là gửi lệnh thành công)
        magnet_state = new_state

def toggle_conveyor_wrapper():
    global conveyor_state
    new_state = bh.toggle_conveyor_handler(conveyor_state, btn_bangtai, send_command_to_serial)
    if new_state is not None:
        conveyor_state = new_state

def update_count_display():
    memory = od.get_object_memory()
    mapping = {
        'red_star': ('star', 'red'),
        'green_star': ('star', 'grn'),
        'yellow_star': ('star', 'yel'),
        'red_triangle': ('tri', 'red'),
        'green_triangle': ('tri', 'grn'),
        'yellow_triangle': ('tri', 'yel'),
        'red_square': ('sqr', 'red'),
        'green_square': ('sqr', 'grn'),
        'yellow_square': ('sqr', 'yel'),
    }

    for mem_key, (row_key, col_key) in mapping.items():
        label = count_labels.get((row_key, col_key))
        if label:
            label.config(text=str(memory[mem_key]['count']))
    for color in ['red', 'grn', 'yel']:
        total = 0
        for shape in ['star', 'tri', 'sqr']:
            label = count_labels.get((shape, color))
            if label:
                try:
                    total += int(label.cget("text"))
                except ValueError:
                    pass
        if sum_labels.get(color):
            sum_labels[color].config(text=str(total))
    # Lên lịch gọi lại sau 500ms
    window.after(500, update_count_display)


# --- GUI LAYOUT DEFINITION ---
frame_main_content = tk.Frame(window, bg="#f0f0f5")
frame_main_content.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
# Section 1: Header Image
# label_image = tk.Label(window, bg="#f0f0f5")
# try:
#     image_path = "Banner0.png"
#     try:
#         image = Image.open(image_path)
#     except FileNotFoundError:
#         print(f"Warning: Image file '{image_path}' not found. Using placeholder.")
#         image = Image.new('RGB', (850, 100), color='skyblue')  # Simpler placeholder
#     image = image.resize((1000, 100))
#     photo = ImageTk.PhotoImage(image)
#     label_image.config(image=photo)
#     label_image.image = photo
# except Exception as e:
#     print(f"Lỗi khi tải ảnh banner: {e}")
#     # label_image might remain empty or you can set a text placeholder
# Section 1: Header with two images and center text
frame_banner = tk.Frame(window, bg="#f0f0f5", height=100)
frame_banner.pack(fill=tk.X, padx=10, pady=(10, 0))

# Left image
left_image_label = tk.Label(frame_banner, bg="#f0f0f5")
try:
    left_image_path = "university.png"
    left_image = Image.open(left_image_path).resize((130, 130))
except FileNotFoundError:
    left_image = Image.new('RGB', (100, 100), color='lightblue')
left_photo = ImageTk.PhotoImage(left_image)
left_image_label.config(image=left_photo)
left_image_label.image = left_photo
left_image_label.pack(side=tk.LEFT, padx=(0, 10))

# Center title
title_frame = tk.Frame(frame_banner, bg="#f0f0f5")
title_frame.pack(side=tk.LEFT, expand=True)

# First line: Main title
banner_title = tk.Label(title_frame, text="TRƯỜNG ĐẠI HỌC SƯ PHẠM KỸ THUẬT THÀNH PHỐ HỒ CHÍ MINH", font=font_title, bg="#f0f0f5", fg="#333")
banner_title.pack()

# Second line: Subtitle
banner_subtitle = tk.Label(title_frame, text="KHOA ĐIỆN - ĐIỆN TỬ", font=font_title, bg="#f0f0f5", fg="#333")
banner_subtitle.pack()

# Third line: GUI title (with spacing)
banner_gui_title = tk.Label(title_frame, text="DELTA ROBOT", font=("Arial", 22, "bold"), bg="#f0f0f5", fg="#0055aa")
banner_gui_title.pack(pady=(20, 0))  # cách dòng trên 20 pixel
# Right image
right_image_label = tk.Label(frame_banner, bg="#f0f0f5")
try:
    right_image_path = "faculty.jpg"
    right_image = Image.open(right_image_path).resize((130, 130))
except FileNotFoundError:
    right_image = Image.new('RGB', (100, 100), color='lightgreen')
right_photo = ImageTk.PhotoImage(right_image)
right_image_label.config(image=right_photo)
right_image_label.image = right_photo
right_image_label.pack(side=tk.RIGHT, padx=(10, 0))


# Section 2: Title
title = tk.Label(window, text="ROBOT DELTA", font=font_title, bg="#f0f0f5", fg="#333")

# Section 3: Mode Selection (MANUAL/AUTO)
frame_mode_selection_container = tk.Frame(window, bg="#f0f0f5")
frame_mode_selection = tk.Frame(frame_mode_selection_container, bg="#f0f0f5")
radio_manual = tk.Radiobutton(frame_mode_selection, text="MANUAL", variable=mode_var, value="manual",
                              command=toggle_mode, activebackground="#c0e0c0", indicatoron=0, width=10,
                              font=font_button, relief=tk.RAISED, bd=2) #bd: độ dày đường viền
# variable: Khi một radio button được chọn, giá trị của biến này sẽ được cập nhật thành value của nút đó
# value: Giá trị mà variable sẽ nhận khi radio button này được chọn. Ví dụ, khi radio_manual được chọn, mode_var sẽ có giá trị là "manual".
# indicatron = 0: loại bỏ vòng tròn mặc định của nút do hàm radiobutton tạo ra
# relief: đặt đường viền nút là "nổi", tạo hiệu ứng 3D
radio_auto = tk.Radiobutton(frame_mode_selection, text="AUTO", variable=mode_var, value="auto",
                            command=toggle_mode, activebackground="#ffe0b0", indicatoron=0, width=10, font=font_button,
                            relief=tk.RAISED, bd=2)
# side: ch định phía mà widget được gói vào
radio_manual.pack(side=tk.LEFT, padx=(0, 5))
radio_auto.pack(side=tk.LEFT, padx=5)
frame_mode_selection.pack(side=tk.LEFT) # Sẽ pack vào frame_mode_selection_container sau

# Section 4: Bottom Bar (Text Box and Control Buttons) - DEFINE IT BEFORE MAIN CONTENT FOR PACKING ORDER
bottom_bar_frame = tk.Frame(window, bg="#f0f0f5")
bottom_bar_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)

# ========== TEXT LOG ==========
frame_text_bottom_right = tk.Frame(bottom_bar_frame, bg="#f0f0f5")
frame_text_bottom_right.grid(row=0, column=1, sticky="nsew", padx=(0, 10))  # Right side
# row: chỉ định số hàng widget đc đặt
# rowspan: widget chiếm bao nhiêu hàng
# sticky: chỉnh cách widget dính vào bên nào của 1 ô lưới
# hoặc mở rộng theo hướng chỉ định để lấp đầy không gian trong ô lưới đó
text_box = tk.Text(frame_text_bottom_right, font=("Courier New", 11), width=60, height=5)
scrollbar = tk.Scrollbar(frame_text_bottom_right, command=text_box.yview)
text_box.config(yscrollcommand=scrollbar.set)
text_box.pack(side=tk.LEFT, fill="both", expand=True)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

# ========== ROBOT CONTROLS ==========
frame_control_buttons_bottom = tk.Frame(bottom_bar_frame, bg="#f0f0f5")
frame_control_buttons_bottom.grid(row=0, column=0, sticky="nw", padx=(10, 0))

frame_control_box = tk.LabelFrame(frame_control_buttons_bottom, text="ROBOT CONTROLS",
                                  font=("Helvetica", 11, "bold"), bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_control_box.pack(padx=5, pady=3)

# COLUMN 1
col1 = tk.Frame(frame_control_box, bg="#f0f0f5")
col1.grid(row=0, column=0, padx=(5, 10))
btn_startall = tk.Button(col1, text="START", command=start_all,
                         font=font_button, bg="#4CAF50", fg="white", width=6)
btn_startall.pack(pady=2, fill=tk.X)
btn_stop = tk.Button(col1, text="STOP", command=stop_robot,
                     font=font_button, bg="#f44336", fg="white", width=6)
btn_stop.pack(pady=2, fill=tk.X)

# COLUMN 2
col2 = tk.Frame(frame_control_box, bg="#f0f0f5")
col2.grid(row=0, column=1, padx=(0, 5))
btn_home = tk.Button(col2, text="SET HOME", command=handle_set_home,
                     font=font_button, bg="#edaa1a", fg="white", width=8)
btn_home.pack(pady=2, fill=tk.X)
btn_namcham = tk.Button(col2, text="ON MAG", command=toggle_namcham_wrapper,
                        font=font_button, bg="#eb3b3b", fg="white", width=8)
btn_namcham.pack(pady=2, fill=tk.X)
btn_bangtai = tk.Button(col2, text="ON CONV", command=toggle_conveyor_wrapper,
                        font=font_button, bg="#eb3b3b", fg="white", width=8)
btn_bangtai.pack(pady=2, fill=tk.X)

# Section 5: Main Content Area (Kinematics, Trajectory, Camera)
frame_main = tk.Frame(window, bg="#f0f0f5")

# Main Content -> Left Side (Inputs: Forward/Inverse Kinematics, Trajectory)
frame_inputs = tk.Frame(frame_main, bg="#f0f0f5")
# Forward Kinematics
frame_fk_group = tk.LabelFrame(frame_inputs, text="FORWARD KINEMATIC", font=("Helvetica", 17, "bold"),
                               bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_fk_group.grid(row=0, column=0, columnspan=3, padx=10, pady=(10, 5), sticky="nsew")

label_theta1 = tk.Label(frame_fk_group, text="Theta 1 (°):", font=font_label, bg="#f0f0f5")
label_theta1.grid(row=0, column=0, padx=10, pady=5, sticky="w")
entry_theta1 = tk.Entry(frame_fk_group, font=font_entry, width=10)
entry_theta1.grid(row=0, column=1, pady=5)

label_theta2 = tk.Label(frame_fk_group, text="Theta 2 (°):", font=font_label, bg="#f0f0f5")
label_theta2.grid(row=1, column=0, padx=10, pady=5, sticky="w")
entry_theta2 = tk.Entry(frame_fk_group, font=font_entry, width=10)
entry_theta2.grid(row=1, column=1, pady=5)

btn_run = tk.Button(frame_fk_group, text="RUN", command=lambda: bh.send_angles_handler(
                        entry_theta1, entry_theta2, entry_theta3,
                        entry_x, entry_y, entry_z,
                        ser),
                    font=font_button, bg="#b5b0a7", fg="black", width=8)
btn_run.grid(row=1, column=2, padx=5)

label_theta3 = tk.Label(frame_fk_group, text="Theta 3 (°):", font=font_label, bg="#f0f0f5")
label_theta3.grid(row=2, column=0, padx=10, pady=5, sticky="w")
entry_theta3 = tk.Entry(frame_fk_group, font=font_entry, width=10)
entry_theta3.grid(row=2, column=1, pady=5)

# Position Display
frame_pos_group = tk.LabelFrame(frame_inputs, text="POSITION (mm)", font=("Helvetica", 17, "bold"),
                                bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_pos_group.grid(row=1, column=0, columnspan=3, padx=10, pady=(5, 5), sticky="nsew")

# Count Display
frame_count_group = tk.LabelFrame(frame_inputs, text="PRODUCTS", font=("Helvetica", 17, "bold"),
                                  bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_count_group.grid(row=0, column=3, rowspan=2, columnspan=3, padx=(10, 10), pady=(10, 5), sticky="nsew")

# Tên cột
col_names = ["RED", "GRN", "YEL"] # chữ GRN ở đây phải giống trong hàm update_count_display
for j, name in enumerate(col_names):
    tk.Label(frame_count_group, text=name, font=font_label, bg="#f0f0f5").grid(row=0, column=j+1, padx=5, pady=5)

# Tạo tiêu đề hàng (star, tri, sqr) và các ô đếm
row_names = ["STAR", "TRI", "SQR"]
count_labels = {}  # Dictionary lưu các nhãn để cập nhật

for i, shape in enumerate(row_names):
    # Tên hàng
    tk.Label(frame_count_group, text=shape, font=font_label, bg="#f0f0f5").grid(row=i + 1, column=0, padx=5, pady=5)

    for j, color in enumerate(col_names):
        lbl = tk.Label(frame_count_group, text="0", font=font_label,
                       relief="sunken", bg="white", width=6, anchor="center")
        lbl.grid(row=i + 1, column=j + 1, padx=5, pady=5, sticky="nsew")
        count_labels[(shape.lower(), color.lower())] = lbl
tk.Label(frame_count_group, text="SUM", font=font_label, bg="#f0f0f5").grid(row=4, column=0, padx=5, pady=5)

sum_labels = {}  # Dictionary lưu các nhãn tổng

for j, color in enumerate(col_names):
    lbl = tk.Label(frame_count_group, text="0", font=font_label,
                   relief="sunken", bg="#e0e0e0", width=6, anchor="center")
    lbl.grid(row=4, column=j + 1, padx=5, pady=5, sticky="nsew")
    sum_labels[color.lower()] = lbl
update_count_display()

# Position Display
frame_pos_group = tk.LabelFrame(frame_inputs, text="POSITION (mm)", font=("Helvetica", 17, "bold"),
                                bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_pos_group.grid(row=1, column=0, columnspan=3, padx=10, pady=(5, 5), sticky="nsew")

frame_xyz = tk.Frame(frame_pos_group, bg="#f0f0f5")
frame_xyz.pack(padx=10, pady=5, anchor="w")

label_x = tk.Label(frame_xyz, text="X:", font=font_label, bg="#f0f0f5")
label_x.pack(side=tk.LEFT, padx=(0, 2))
entry_x = tk.Entry(frame_xyz, font=font_entry, width=8, state='readonly')
entry_x.pack(side=tk.LEFT, padx=(0, 10))

label_y = tk.Label(frame_xyz, text="Y:", font=font_label, bg="#f0f0f5")
label_y.pack(side=tk.LEFT, padx=(0, 2))
entry_y = tk.Entry(frame_xyz, font=font_entry, width=8, state='readonly')
entry_y.pack(side=tk.LEFT, padx=(0, 10))

label_z = tk.Label(frame_xyz, text="Z:", font=font_label, bg="#f0f0f5")
label_z.pack(side=tk.LEFT, padx=(0, 2))
entry_z = tk.Entry(frame_xyz, font=font_entry, width=8, state='readonly')
entry_z.pack(side=tk.LEFT, padx=(0, 10))

# INVERSE KINEMATIC SECTION
frame_ik_group = tk.LabelFrame(frame_inputs, text="INVERSE KINEMATIC",
                               font=("Helvetica", 17, "bold"), bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_ik_group.grid(row=2, column=0, columnspan=3, padx=10, pady=(10, 5), sticky="nsew")

# Left column: X, Y, Z
label_x_ik = tk.Label(frame_ik_group, text="X:", font=font_label, bg="#f0f0f5")
label_x_ik.grid(row=0, column=0, sticky="e", padx=(10, 2), pady=6)
entry_x_ik = tk.Entry(frame_ik_group, font=font_entry, width=8)
entry_x_ik.grid(row=0, column=1, sticky="w", padx=(0, 10), pady=6)

label_y_ik = tk.Label(frame_ik_group, text="Y:", font=font_label, bg="#f0f0f5")
label_y_ik.grid(row=1, column=0, sticky="e", padx=(10, 2), pady=6)
entry_y_ik = tk.Entry(frame_ik_group, font=font_entry, width=8)
entry_y_ik.grid(row=1, column=1, sticky="w", padx=(0, 10), pady=6)

label_z_ik = tk.Label(frame_ik_group, text="Z:", font=font_label, bg="#f0f0f5")
label_z_ik.grid(row=2, column=0, sticky="e", padx=(10, 2), pady=6)
entry_z_ik = tk.Entry(frame_ik_group, font=font_entry, width=8)
entry_z_ik.grid(row=2, column=1, sticky="w", padx=(0, 10), pady=6)

# Right column: Theta1, Theta2, Theta3
label_theta1_ik = tk.Label(frame_ik_group, text="Theta1:", font=font_label, bg="#f0f0f5")
label_theta1_ik.grid(row=0, column=2, sticky="e", padx=(10, 2), pady=6)
entry_theta1_ik = tk.Entry(frame_ik_group, font=font_entry, width=8, state='readonly')
entry_theta1_ik.grid(row=0, column=3, sticky="w", padx=(0, 10), pady=6)

label_theta2_ik = tk.Label(frame_ik_group, text="Theta2:", font=font_label, bg="#f0f0f5")
label_theta2_ik.grid(row=1, column=2, sticky="e", padx=(10, 2), pady=6)
entry_theta2_ik = tk.Entry(frame_ik_group, font=font_entry, width=8, state='readonly')
entry_theta2_ik.grid(row=1, column=3, sticky="w", padx=(0, 10), pady=6)

label_theta3_ik = tk.Label(frame_ik_group, text="Theta3:", font=font_label, bg="#f0f0f5")
label_theta3_ik.grid(row=2, column=2, sticky="e", padx=(10, 2), pady=6)
entry_theta3_ik = tk.Entry(frame_ik_group, font=font_entry, width=8, state='readonly')
entry_theta3_ik.grid(row=2, column=3, sticky="w", padx=(0, 10), pady=6)

# Button CAL IK nằm ở dòng thấp hơn (row=4) để thẳng hàng nút RUN TRAJECTORY
btn_calc_ik = tk.Button(frame_ik_group, text="CAL IK",
                        command=lambda: bh.calculate_inv_kinematic_handler(
                            entry_x_ik, entry_y_ik, entry_z_ik,
                            entry_theta1_ik, entry_theta2_ik, entry_theta3_ik
                        ),
                        font=font_button, bg="#b5b0a7", fg="black", width=12)
btn_calc_ik.grid(row=4, column=0, columnspan=4, pady=(12, 6), sticky="w", padx=10)

# Trajectory Points
frame_traj_group = tk.LabelFrame(frame_inputs, text="TRAJECTORY POINTS",
                                 font=("Helvetica", 17, "bold"), bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_traj_group.grid(row=2, column=3, columnspan=3, padx=10, pady=(10, 5), sticky="nsew")

label_p0 = tk.Label(frame_traj_group, text="P0: (X0, Y0, Z0) | C0", font=font_label, bg="#f0f0f5")
label_p0.pack(anchor="w", padx=10, pady=(5, 0))
frame_p0 = tk.Frame(frame_traj_group, bg="#f0f0f5"); frame_p0.pack(pady=2, anchor="w", padx=10)
entry_x0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_x0.pack(side=tk.LEFT, padx=3); entry_x0.insert(0, "0.0")
entry_y0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_y0.pack(side=tk.LEFT, padx=3); entry_y0.insert(0, "0.0")
entry_z0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_z0.pack(side=tk.LEFT, padx=3); entry_z0.insert(0, "-307.38")
entry_c0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_c0.pack(side=tk.LEFT, padx=3)

label_pf = tk.Label(frame_traj_group, text="Pf: (Xf, Yf, Zf) | tf", font=font_label, bg="#f0f0f5")
label_pf.pack(anchor="w", padx=10, pady=(5, 0))
frame_pf = tk.Frame(frame_traj_group, bg="#f0f0f5"); frame_pf.pack(pady=2, anchor="w", padx=10)
entry_xf = tk.Entry(frame_pf, font=font_entry, width=7); entry_xf.pack(side=tk.LEFT, padx=3)
entry_yf = tk.Entry(frame_pf, font=font_entry, width=7); entry_yf.pack(side=tk.LEFT, padx=3)
entry_zf = tk.Entry(frame_pf, font=font_entry, width=7); entry_zf.pack(side=tk.LEFT, padx=3)
entry_tf = tk.Entry(frame_pf, font=font_entry, width=7); entry_tf.pack(side=tk.LEFT, padx=3)

button_traj = tk.Button(frame_traj_group, text="RUN TRAJECTORY",
                        command=lambda: bh.send_trajectory_handler(
                            entry_x0, entry_y0, entry_z0, entry_c0,
                            entry_xf, entry_yf, entry_zf, entry_tf,
                            send_command_to_serial
                        ),
                        font=font_button, bg="#b5b0a7", fg="black", width=15)
button_traj.pack(pady=(5, 7), padx=10, anchor="w")

frame_inputs.pack(side=tk.LEFT, fill="y", padx=(0,10))

# Main Content -> Right Side (Camera)
frame_right_zone = tk.Frame(frame_main, bg="#f0f0f5")
frame_right_zone.pack(side=tk.RIGHT, fill="both", expand=True, padx=10) # expand True để frame này cũng cố gắng chiếm không gian

frame_camera_display_area = tk.Frame(frame_right_zone, bg="#d0d0d5", bd=1, relief=tk.SOLID)
frame_camera_display_area.pack(pady=10, padx=10, fill="both", expand=True)

label_cam = tk.Label(frame_camera_display_area, bg="black")
# Đặt kích thước ban đầu cho label_cam để nó chiếm không gian
# Kích thước này nên khớp với DISPLAY_WIDTH, DISPLAY_HEIGHT trong FunctionButton.py
label_cam.config(width=bh.DISPLAY_WIDTH//10, height=bh.DISPLAY_HEIGHT//20) # Ước lượng, sẽ được thay bằng ảnh
label_cam.pack(padx=10, pady=10, anchor="center", fill="both", expand=True)

frame_cam_buttons = tk.Frame(frame_camera_display_area, bg="#d0d0d5")
frame_cam_buttons.pack(side=tk.BOTTOM, pady=5)

# Truyền ser vào start_camera_handler
btn_start_cam = tk.Button(frame_cam_buttons, text="START CAMERA",
                          command=lambda: bh.start_camera_handler(label_cam, ser),
                          font=font_button, bg="#4CAF50", fg="white", width=16)
btn_start_cam.pack(side=tk.LEFT, padx=5)

btn_stop_cam = tk.Button(frame_cam_buttons, text="STOP CAMERA",
                         command=lambda: bh.stop_camera_stream_handler(label_cam),
                         font=font_button, bg="#f44336", fg="white", width=16)
btn_stop_cam.pack(side=tk.LEFT, padx=5)

# --- PACKING THE MAIN LAYOUT SECTIONS INTO THE WINDOW ---
# Order is important: top fixed, bottom fixed, then central expanding
######################
# label_image.pack(side=tk.TOP, fill=tk.X, pady=(10, 5))
# title.pack(side=tk.TOP, pady=(5,10)) # Giảm pady trên của title
######################

# Giảm pady cho frame_mode_selection_container
frame_mode_selection_container.pack(side=tk.TOP, fill=tk.X, padx=20, pady=(0, 5))

# Giảm pady cho bottom_bar_frame
bottom_bar_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(5, 5))

frame_main.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=20, pady=0)

# --- FINAL SETUP ---
controllable_buttons.extend([
    btn_run, button_traj, btn_namcham, btn_bangtai,
    btn_calc_ik, btn_home
    # ,btn_start_cam, btn_stop_cam
])
toggle_mode()
# Trạng thái ban đầu khi mới mở giao diện
for btn in controllable_buttons:
    btn.config(state=tk.DISABLED)
radio_manual.config(state=tk.DISABLED)
radio_auto.config(state=tk.DISABLED)
btn_start_cam.config(state=tk.DISABLED)
btn_stop_cam.config(state=tk.DISABLED)

if ser:
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()
else:
    text_box.insert(tk.END, "Serial port (COM5) not available. Check connection.\n")
def on_closing():
    global bh_camera_running # Để truy cập biến từ FunctionButton
    if bh.bh_camera_running: # Sử dụng bh.bh_camera_running để kiểm tra
        bh.stop_camera_stream_handler(label_cam) # Dừng camera nếu đang chạy
    if ser and ser.is_open:
        print("Closing serial port.")
        ser.close()
    window.destroy()

window.protocol("WM_DELETE_WINDOW", on_closing)
window.mainloop()