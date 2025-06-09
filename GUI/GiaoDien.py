import tkinter as tk
from tkinter import messagebox
import serial
import time
import threading
import FunctionButton as bh
from PIL import Image, ImageTk
import ObjectDetection as od
import FunctionButton
import __main__

home_set = False

# Biến toàn cục, không cần khởi tạo lại ser
ser = None
serial_manager = None
window = tk.Tk()
window.title("GUI")
window.configure(bg="#f0f0f5")

font_title = ("Arial", 20, "bold")
font_label = ("Arial", 12)
font_entry = ("Arial", 12)
font_button = ("Arial", 12, "bold")

controllable_buttons = []
mode_var = tk.StringVar(value="manual")

###################################
def init_serial():
    global serial_manager, ser
    try:
        port = 'COM4'
        baudrate = 9600
        # SerialManager sẽ quản lý kết nối
        serial_manager = od.SerialManager(port, baudrate)
        serial_manager.start() # Khởi chạy thread của SerialManager
        ser = serial_manager.serial_port # Lấy đối tượng serial để dùng nếu cần
        if ser and ser.is_open:
            print("Serial connection initialized successfully via SerialManager.")
            return True
        else:
            raise serial.SerialException("SerialManager failed to open the port.")
    except Exception as e:
        print(f"Failed to initialize serial: {e}")
        messagebox.showerror("Serial Error", f"Không thể kết nối với Arduino trên {port}:\n{e}")
        return False

# <<< THAY ĐỔI: Hàm gửi lệnh được đơn giản hóa, tin tưởng vào SerialManager
def send_command_to_serial(command):
    print(f"Attempting to send command: {command.strip()}")
    if serial_manager:
        try:
            # print("Using SerialManager to send command") # Dòng này không cần thiết nữa
            serial_manager.send_command(command)
            return True
        except Exception as e:
            print(f"Error sending command via SerialManager: {e}")
            messagebox.showerror("Serial Error", f"Lỗi gửi lệnh: {e}")
            return False
    else:
        print("Serial Manager is not available.")
        messagebox.showwarning("Serial Warning", "Chưa kết nối với Arduino!")
        return False

# <<< THAY ĐỔI: Hàm này sẽ thay thế hoàn toàn cho thread `read_serial`
def check_serial_responses():
    """
    Sử dụng cơ chế của SerialManager để nhận dữ liệu một cách an toàn,
    và cập nhật vào text_box.
    """
    if serial_manager:
        response = serial_manager.get_response() # Lấy dữ liệu từ queue của SerialManager
        if response:
            # print(f"Received from Arduino: {response}") # In ra console để debug
            # Cập nhật vào GUI
            text_box.insert(tk.END, response + "\n")
            text_box.see(tk.END)

    # Lên lịch để hàm này được gọi lại sau 100ms
    window.after(100, check_serial_responses)

##########################################
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
    # Giả định set_home_handler sẽ gọi send_command_to_serial('h\r')
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

def toggle_mode():
    mode = mode_var.get()
    new_state_manual_buttons = tk.DISABLED

    if mode == "manual":
        radio_manual.config(relief=tk.SUNKEN, bg="#f4f716")
        radio_auto.config(relief=tk.RAISED, bg="#f0f0f5")
        new_state_manual_buttons = tk.NORMAL
        bh.stop_camera_stream_handler(label_cam)
        od.set_operation_mode(False)
    else:  # auto mode
        radio_auto.config(relief=tk.SUNKEN, bg="#f4f716")
        radio_manual.config(relief=tk.RAISED, bg="#f0f0f5")
        radio_manual.config(state=tk.DISABLED)
        radio_auto.config(state=tk.DISABLED)
        btn_namcham.config(bg="#eb3b3b")
        btn_bangtai.config(bg="#eb3b3b")
        od.set_operation_mode(True)
        bh.run_auto_mode_sequence(send_command_to_serial, label_cam, serial_manager)

    for btn in controllable_buttons:
        if btn:
            try:
                btn.config(state=new_state_manual_buttons)
            except tk.TclError:
                pass


# <<< XÓA BỎ: Toàn bộ hàm read_serial đã được loại bỏ
# def read_serial():
#     ...

cap = None
camera_running = False
magnet_state = 0
conveyor_state = 0

def toggle_namcham_wrapper():
    global magnet_state
    new_state = bh.toggle_namcham_handler(magnet_state, btn_namcham, send_command_to_serial)
    if new_state is not None:
        magnet_state = new_state

def toggle_conveyor_wrapper():
    global conveyor_state
    new_state = bh.toggle_conveyor_handler(conveyor_state, btn_bangtai, send_command_to_serial)
    if new_state is not None:
        conveyor_state = new_state

def update_count_display():
    memory = od.get_object_memory()
    mapping = {
        'red_star': ('star', 'red'), 'green_star': ('star', 'grn'), 'yellow_star': ('star', 'yel'),
        'red_triangle': ('tri', 'red'), 'green_triangle': ('tri', 'grn'), 'yellow_triangle': ('tri', 'yel'),
        'red_square': ('sqr', 'red'), 'green_square': ('sqr', 'grn'), 'yellow_square': ('sqr', 'yel'),
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
        if sum_labels.get(color): # Lấy ra đối tượng Label cho ô tổng của cột màu hiện tại.Lấy ra đối tượng Label cho ô tổng của cột màu hiện tại.
            sum_labels[color].config(text=str(total))
            # Cập nhật văn bản của ô tổng đó bằng giá trị total đã tính được.
    window.after(500, update_count_display)


# --- GUI LAYOUT DEFINITION (Không thay đổi phần này) ---
frame_main_content = tk.Frame(window, bg="#f0f0f5")
frame_main_content.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
frame_banner = tk.Frame(window, bg="#f0f0f5", height=100)
frame_banner.pack(fill=tk.X, padx=10, pady=(10, 0))
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
title_frame = tk.Frame(frame_banner, bg="#f0f0f5")
title_frame.pack(side=tk.LEFT, expand=True)
banner_title = tk.Label(title_frame, text="TRƯỜNG ĐẠI HỌC SƯ PHẠM KỸ THUẬT THÀNH PHỐ HỒ CHÍ MINH", font=font_title, bg="#f0f0f5", fg="#333")
banner_title.pack()
banner_subtitle = tk.Label(title_frame, text="KHOA ĐIỆN - ĐIỆN TỬ", font=font_title, bg="#f0f0f5", fg="#333")
banner_subtitle.pack()
banner_gui_title = tk.Label(title_frame, text="DELTA ROBOT", font=("Arial", 22, "bold"), bg="#f0f0f5", fg="#0055aa")
banner_gui_title.pack(pady=(20, 0))
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
title = tk.Label(window, text="ROBOT DELTA", font=font_title, bg="#f0f0f5", fg="#333")
frame_mode_selection_container = tk.Frame(window, bg="#f0f0f5")
frame_mode_selection = tk.Frame(frame_mode_selection_container, bg="#f0f0f5")
radio_manual = tk.Radiobutton(frame_mode_selection, text="MANUAL", variable=mode_var, value="manual",
                              command=toggle_mode, activebackground="#c0e0c0", indicatoron=0, width=10,
                              font=font_button, relief=tk.RAISED, bd=2)
radio_auto = tk.Radiobutton(frame_mode_selection, text="AUTO", variable=mode_var, value="auto",
                            command=toggle_mode, activebackground="#ffe0b0", indicatoron=0, width=10, font=font_button,
                            relief=tk.RAISED, bd=2)
radio_manual.pack(side=tk.LEFT, padx=(0, 5))
radio_auto.pack(side=tk.LEFT, padx=5)
frame_mode_selection.pack(side=tk.LEFT)
bottom_bar_frame = tk.Frame(window, bg="#f0f0f5")
bottom_bar_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=5)
frame_text_bottom_right = tk.Frame(bottom_bar_frame, bg="#f0f0f5")
frame_text_bottom_right.grid(row=0, column=1, sticky="nsew", padx=(0, 10))
text_box = tk.Text(frame_text_bottom_right, font=("Courier New", 11), width=60, height=5)
scrollbar = tk.Scrollbar(frame_text_bottom_right, command=text_box.yview)
text_box.config(yscrollcommand=scrollbar.set)
text_box.pack(side=tk.LEFT, fill="both", expand=True)
scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
frame_control_buttons_bottom = tk.Frame(bottom_bar_frame, bg="#f0f0f5")
frame_control_buttons_bottom.grid(row=0, column=0, sticky="nw", padx=(10, 0))
frame_control_box = tk.LabelFrame(frame_control_buttons_bottom, text="ROBOT CONTROLS",
                                  font=("Helvetica", 11, "bold"), bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_control_box.pack(padx=5, pady=3)
col1 = tk.Frame(frame_control_box, bg="#f0f0f5")
col1.grid(row=0, column=0, padx=(5, 10))
btn_startall = tk.Button(col1, text="START", command=start_all,
                         font=font_button, bg="#4CAF50", fg="white", width=6)
btn_startall.pack(pady=2, fill=tk.X)
btn_stop = tk.Button(col1, text="STOP", command=stop_robot,
                     font=font_button, bg="#f44336", fg="white", width=6)
btn_stop.pack(pady=2, fill=tk.X)
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
frame_main = tk.Frame(window, bg="#f0f0f5")
frame_inputs = tk.Frame(frame_main, bg="#f0f0f5")
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
                        send_command_to_serial),
                    font=font_button, bg="#b5b0a7", fg="black", width=8)
btn_run.grid(row=1, column=2, padx=5)
label_theta3 = tk.Label(frame_fk_group, text="Theta 3 (°):", font=font_label, bg="#f0f0f5")
label_theta3.grid(row=2, column=0, padx=10, pady=5, sticky="w")
entry_theta3 = tk.Entry(frame_fk_group, font=font_entry, width=10)
entry_theta3.grid(row=2, column=1, pady=5)
frame_pos_group = tk.LabelFrame(frame_inputs, text="POSITION (mm)", font=("Helvetica", 17, "bold"),
                                bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_pos_group.grid(row=1, column=0, columnspan=3, padx=10, pady=(5, 5), sticky="nsew")
frame_count_group = tk.LabelFrame(frame_inputs, text="PRODUCTS", font=("Helvetica", 17, "bold"),
                                  bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_count_group.grid(row=0, column=3, rowspan=2, columnspan=3, padx=(10, 10), pady=(10, 5), sticky="nsew")
col_names = ["RED", "GRN", "YEL"]
for j, name in enumerate(col_names):
    tk.Label(frame_count_group, text=name, font=font_label, bg="#f0f0f5").grid(row=0, column=j+1, padx=5, pady=5)
row_names = ["STAR", "TRI", "SQR"]
count_labels = {}
for i, shape in enumerate(row_names):
    tk.Label(frame_count_group, text=shape, font=font_label, bg="#f0f0f5").grid(row=i + 1, column=0, padx=5, pady=5)
    for j, color in enumerate(col_names):
        lbl = tk.Label(frame_count_group, text="0", font=font_label,
                       relief="sunken", bg="white", width=6, anchor="center")
        lbl.grid(row=i + 1, column=j + 1, padx=5, pady=5, sticky="nsew")
        count_labels[(shape.lower(), color.lower())] = lbl
tk.Label(frame_count_group, text="SUM", font=font_label, bg="#f0f0f5").grid(row=4, column=0, padx=5, pady=5)
sum_labels = {}
for j, color in enumerate(col_names):
    lbl = tk.Label(frame_count_group, text="0", font=font_label,
                   relief="sunken", bg="#e0e0e0", width=6, anchor="center")
    lbl.grid(row=4, column=j + 1, padx=5, pady=5, sticky="nsew")
    sum_labels[color.lower()] = lbl
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
frame_ik_group = tk.LabelFrame(frame_inputs, text="INVERSE KINEMATIC",
                               font=("Helvetica", 17, "bold"), bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_ik_group.grid(row=2, column=0, columnspan=3, padx=10, pady=(10, 5), sticky="nsew")
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
btn_calc_ik = tk.Button(frame_ik_group, text="CAL IK",
                        command=lambda: bh.calculate_inv_kinematic_handler(
                            entry_x_ik, entry_y_ik, entry_z_ik,
                            entry_theta1_ik, entry_theta2_ik, entry_theta3_ik
                        ),
                        font=font_button, bg="#b5b0a7", fg="black", width=12)
btn_calc_ik.grid(row=4, column=0, columnspan=4, pady=(12, 6), sticky="w", padx=10)
frame_traj_group = tk.LabelFrame(frame_inputs, text="TRAJECTORY POINTS",
                                 font=("Helvetica", 17, "bold"), bg="#f0f0f5", fg="#333", bd=2, relief=tk.GROOVE)
frame_traj_group.grid(row=2, column=3, columnspan=3, padx=10, pady=(10, 5), sticky="nsew")
label_p0 = tk.Label(frame_traj_group, text="P0: (X0, Y0, Z0) | C0", font=font_label, bg="#f0f0f5")
label_p0.pack(anchor="w", padx=10, pady=(5, 0))
frame_p0 = tk.Frame(frame_traj_group, bg="#f0f0f5"); frame_p0.pack(pady=2, anchor="w", padx=10)
entry_x0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_x0.pack(side=tk.LEFT, padx=3); entry_x0.insert(0, "0.0")
entry_y0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_y0.pack(side=tk.LEFT, padx=3); entry_y0.insert(0, "0.0")
entry_z0 = tk.Entry(frame_p0, font=font_entry, width=7); entry_z0.pack(side=tk.LEFT, padx=3); entry_z0.insert(0, "-307.38")
label_pf = tk.Label(frame_traj_group, text="Pf: (Xf, Yf, Zf) | tf", font=font_label, bg="#f0f0f5")
label_pf.pack(anchor="w", padx=10, pady=(5, 0))
frame_pf = tk.Frame(frame_traj_group, bg="#f0f0f5"); frame_pf.pack(pady=2, anchor="w", padx=10)
entry_xf = tk.Entry(frame_pf, font=font_entry, width=7); entry_xf.pack(side=tk.LEFT, padx=3)
entry_yf = tk.Entry(frame_pf, font=font_entry, width=7); entry_yf.pack(side=tk.LEFT, padx=3)
entry_zf = tk.Entry(frame_pf, font=font_entry, width=7); entry_zf.pack(side=tk.LEFT, padx=3)
entry_tf = tk.Entry(frame_pf, font=font_entry, width=7); entry_tf.pack(side=tk.LEFT, padx=3)
button_traj = tk.Button(frame_traj_group, text="RUN TRAJECTORY",
                        command=lambda: bh.send_trajectory_handler(
                            entry_x0, entry_y0, entry_z0,
                            entry_xf, entry_yf, entry_zf, entry_tf,
                            send_command_to_serial
                        ),
                        font=font_button, bg="#b5b0a7", fg="black", width=15)
button_traj.pack(pady=(5, 7), padx=10, anchor="w")
frame_inputs.pack(side=tk.LEFT, fill="y", padx=(0,10))
frame_right_zone = tk.Frame(frame_main, bg="#f0f0f5")
frame_right_zone.pack(side=tk.RIGHT, fill="both", expand=True, padx=10)
frame_camera_display_area = tk.Frame(frame_right_zone, bg="#d0d0d5", bd=1, relief=tk.SOLID)
frame_camera_display_area.pack(pady=10, padx=10, fill="both", expand=True)
label_cam = tk.Label(frame_camera_display_area, bg="black")
label_cam.config(width=bh.DISPLAY_WIDTH//10, height=bh.DISPLAY_HEIGHT//20)
label_cam.pack(padx=10, pady=10, anchor="center", fill="both", expand=True)
frame_cam_buttons = tk.Frame(frame_camera_display_area, bg="#d0d0d5")
frame_cam_buttons.pack(side=tk.BOTTOM, pady=5)
btn_start_cam = tk.Button(frame_cam_buttons, text="START CAMERA",
                          command=lambda: bh.start_camera_handler(label_cam, serial_manager),
                          font=font_button, bg="#4CAF50", fg="white", width=16)
btn_start_cam.pack(side=tk.LEFT, padx=5)
btn_stop_cam = tk.Button(frame_cam_buttons, text="STOP CAMERA",
                         command=lambda: bh.stop_camera_stream_handler(label_cam),
                         font=font_button, bg="#f44336", fg="white", width=16)
btn_stop_cam.pack(side=tk.LEFT, padx=5)
frame_mode_selection_container.pack(side=tk.TOP, fill=tk.X, padx=20, pady=(0, 5))
bottom_bar_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=(5, 5))
frame_main.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=20, pady=0)
controllable_buttons.extend([
    btn_run, button_traj, btn_namcham, btn_bangtai,
    btn_calc_ik, btn_home
])
toggle_mode()
for btn in controllable_buttons:
    btn.config(state=tk.DISABLED)
radio_manual.config(state=tk.DISABLED)
radio_auto.config(state=tk.DISABLED)
btn_start_cam.config(state=tk.DISABLED)
btn_stop_cam.config(state=tk.DISABLED)

# --- KHỞI CHẠY ---

if init_serial():
    # Khởi động vòng lặp kiểm tra phản hồi từ serial
    check_serial_responses()
    # Khởi động vòng lặp cập nhật số lượng sản phẩm
    update_count_display()
    # Các nút điều khiển ban đầu vẫn bị vô hiệu hóa cho đến khi nhấn START
    btn_startall.config(state=tk.NORMAL)
else:
    # Nếu không kết nối được, thông báo trong text_box
    text_box.insert(tk.END, "Không thể kết nối Serial. Vui lòng kiểm tra lại kết nối và cổng COM.\n")

# <<< XÓA BỎ: Đoạn code khởi tạo thread `read_serial` cũ
# if ser:
#     serial_thread = threading.Thread(target=read_serial, daemon=True)
#     serial_thread.start()
# else:
#     text_box.insert(tk.END, "Serial port (COM5) not available. Check connection.\n")

def on_closing():
    if bh.bh_camera_running:
        bh.stop_camera_stream_handler(label_cam)
    # Dừng SerialManager một cách an toàn
    if serial_manager:
        serial_manager.stop()
    window.destroy()

window.protocol("WM_DELETE_WINDOW", on_closing)
window.mainloop()