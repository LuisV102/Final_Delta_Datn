import tkinter as tk
from tkinter import messagebox
import Kinematic
import cv2
from PIL import Image, ImageTk
import ObjectDetection
# <<< THAY ĐỔI: Không cần import SerialManager ở đây nữa, vì file này không nên trực tiếp tạo nó
# from ObjectDetection import SerialManager
import time
import threading

# Biến cục bộ cho module này (dùng cho camera)
bh_cap = None
bh_camera_running = False
DISPLAY_WIDTH = 720
DISPLAY_HEIGHT = 370

# <<< XÓA BỎ: Hàm này không được sử dụng và gây nhầm lẫn.
# Việc khởi tạo SerialManager chỉ nên thực hiện ở file GiaoDien.py.
# def init_serial_connection(port, baudrate):
#     ...

def simple_command_handler(send_command_func, command):
    """Hàm này vẫn ổn, giữ lại để tham khảo nếu cần."""
    send_command_func(command)

# <<< THAY ĐỔI LỚN: Sửa lại hoàn toàn logic gửi lệnh của hàm này
def send_angles_handler(entry_theta1, entry_theta2, entry_theta3,
                        entry_x, entry_y, entry_z,
                        send_command_func): # Thay đổi tham số, nhận hàm gửi lệnh thống nhất
    try:
        theta1 = float(entry_theta1.get())
        theta2 = float(entry_theta2.get())
        theta3 = float(entry_theta3.get())

        # Các đoạn kiểm tra giới hạn giữ nguyên
        if not (0 <= theta1 <= 60):
            messagebox.showerror("Error", "Theta 1 phải trong khoảng 0 đến 60")
            return
        if not (0 <= theta2 <= 60):
            messagebox.showerror("Error", "Theta 2 phải trong khoảng 0 đến 60")
            return
        if not (0 <= theta3 <= 60):
            messagebox.showerror("Error", "Theta 3 phải trong khoảng 0 đến 60")
            return

        # Tính toán forward kinematic
        result = Kinematic.forward_kinematic(theta1, theta2, theta3)
        if result[0] is False:
            messagebox.showerror("Error", "Không thể tính vị trí. Kiểm tra lại góc đầu vào.")
            return

        _, x, y, z = result

        # Các đoạn kiểm tra giới hạn tọa độ giữ nguyên
        if not (-100 <= x <= 87):
            messagebox.showerror("Lỗi", f"X={x:.2f} nằm ngoài giới hạn robot")
            return
        if not (-80 <= y <= 130):
            messagebox.showerror("Lỗi", f"Y={y:.2f} nằm ngoài giới hạn robot")
            return
        if not (-397 <= z <= -307.38):
            messagebox.showerror("Lỗi", f"Z={z:.2f} nằm ngoài giới hạn robot")
            return

        # Xây dựng và gửi lệnh MỘT LẦN DUY NHẤT qua hàm đã được truyền vào
        data = f"{theta1}A{theta2}B{theta3}C\r"
        if not send_command_func(data):
            # Nếu hàm gửi lệnh trả về False (thất bại), thì dừng lại
            messagebox.showwarning("Gửi lệnh thất bại", "Không thể gửi lệnh góc tới robot.")
            return

        # Cập nhật giao diện nếu gửi lệnh thành công
        entry_x.config(state='normal')
        entry_y.config(state='normal')
        entry_z.config(state='normal')
        entry_x.delete(0, tk.END); entry_x.insert(0, f"{x:.2f}")
        entry_y.delete(0, tk.END); entry_y.insert(0, f"{y:.2f}")
        entry_z.delete(0, tk.END); entry_z.insert(0, f"{z:.2f}")
        entry_x.config(state='readonly')
        entry_y.config(state='readonly')
        entry_z.config(state='readonly')

    except ValueError:
        messagebox.showerror("Error", "Vui lòng nhập đúng định dạng là số!")
    except Exception as e:
        print(f"Lỗi khi tính forward kinematic: {e}")
        messagebox.showerror("Lỗi", "Không thể tính vị trí. Kiểm tra lại hàm forward_kinematic.")


def calculate_inv_kinematic_handler(entry_x_ik, entry_y_ik, entry_z_ik,
                                    entry_theta1_ik, entry_theta2_ik, entry_theta3_ik):
    try:
        x_val = float(entry_x_ik.get())
        y_val = float(entry_y_ik.get())
        z_val = float(entry_z_ik.get())
        angles = Kinematic.inverse_kinematic(x_val, y_val, z_val)
        entry_theta1_ik.config(state=tk.NORMAL); entry_theta1_ik.delete(0, tk.END); entry_theta1_ik.insert(0, f"{angles[0]:.2f}"); entry_theta1_ik.config(state='readonly')
        entry_theta2_ik.config(state=tk.NORMAL); entry_theta2_ik.delete(0, tk.END); entry_theta2_ik.insert(0, f"{angles[1]:.2f}"); entry_theta2_ik.config(state='readonly')
        entry_theta3_ik.config(state=tk.NORMAL); entry_theta3_ik.delete(0, tk.END); entry_theta3_ik.insert(0, f"{angles[2]:.2f}"); entry_theta3_ik.config(state='readonly')
    except ValueError:
        messagebox.showerror("Lỗi", "Vui lòng nhập giá trị số hợp lệ cho X, Y, Z.")
    except Exception as e:
        messagebox.showerror("Lỗi tính toán", str(e))


def send_trajectory_handler(entry_x0, entry_y0, entry_z0,
                            entry_xf, entry_yf, entry_zf, entry_tf,
                            send_command_func):
    try:
        x0_val = float(entry_x0.get())
        y0_val = float(entry_y0.get())
        z0_val = float(entry_z0.get())
        xf_val = float(entry_xf.get())
        yf_val = float(entry_yf.get())
        zf_val = float(entry_zf.get())
        tf_val_str = entry_tf.get().strip() # Lấy và xóa khoảng trắng

        if not tf_val_str: # Kiểm tra chuỗi rỗng
            messagebox.showerror("Lỗi", "Vui lòng nhập giá trị cho thời gian (tf).")
            return
        tf_val = float(tf_val_str)

        if not (-120 <= x0_val <= 87): messagebox.showerror("Lỗi", f"X0={x0_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-110 <= y0_val <= 130): messagebox.showerror("Lỗi", f"Y0={y0_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-398.6 <= z0_val <= -307.38): messagebox.showerror("Lỗi", f"Z0={z0_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-125 <= xf_val <= 87): messagebox.showerror("Lỗi", f"Xf={xf_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-110 <= yf_val <= 130): messagebox.showerror("Lỗi", f"Yf={yf_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-398.6 <= zf_val <= -307.38): messagebox.showerror("Lỗi", f"Zf={zf_val:.2f} nằm ngoài giới hạn robot"); return

        data_to_send = f"P0:{x0_val},{y0_val},{z0_val};Pf:{xf_val},{yf_val},{zf_val};T:{tf_val}"

        if send_command_func(data_to_send):
            entry_x0.delete(0, tk.END); entry_x0.insert(0, str(xf_val))
            entry_y0.delete(0, tk.END); entry_y0.insert(0, str(yf_val))
            entry_z0.delete(0, tk.END); entry_z0.insert(0, str(zf_val))
            entry_xf.delete(0, tk.END)
            entry_yf.delete(0, tk.END)
            entry_zf.delete(0, tk.END)
            entry_tf.delete(0, tk.END)
    except ValueError:
        messagebox.showerror("Lỗi", "Vui lòng nhập giá trị số hợp lệ cho tọa độ và thời gian.")
    except Exception as e:
        messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {str(e)}")


def set_home_handler(send_command_func, entry_x, entry_y, entry_z):
    if send_command_func('h\r'):
        try:
            result = Kinematic.forward_kinematic(0, 0, 0)
            _, x, y, z = result
            entry_x.config(state='normal'); entry_x.delete(0, tk.END); entry_x.insert(0, f"{x:.2f}"); entry_x.config(state='readonly')
            entry_y.config(state='normal'); entry_y.delete(0, tk.END); entry_y.insert(0, f"{y:.2f}"); entry_y.config(state='readonly')
            entry_z.config(state='normal'); entry_z.delete(0, tk.END); entry_z.insert(0, f"{z:.2f}"); entry_z.config(state='readonly')
            return True
        except Exception as e:
            print(f"Lỗi khi tính toán hoặc cập nhật home: {e}")
            return False
    else:
        return False


def start_camera_handler(label_cam_widget, serial_object):
    global bh_cap, bh_camera_running
    if not bh_camera_running:
        try:
            bh_cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            if not bh_cap or not bh_cap.isOpened():
                bh_cap = cv2.VideoCapture(0)
            if not bh_cap or not bh_cap.isOpened():
                messagebox.showerror("Camera Error", "Không thể mở camera.")
                bh_camera_running = False
                return
            # bh_cap = cv2.VideoCapture(1)
            # print("Attempting to open camera with index 1 (Auto-selected backend)...")

            bh_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            bh_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            bh_camera_running = True
            ObjectDetection.reset_detection_state()
            update_frame_handler(label_cam_widget, serial_object)
        except Exception as e:
            messagebox.showerror("Camera Error", f"Lỗi khi khởi động camera: {e}")
            bh_camera_running = False
            if bh_cap: bh_cap.release()
            bh_cap = None


def stop_camera_stream_handler(label_cam_widget):
    global bh_cap, bh_camera_running
    bh_camera_running = False
    if bh_cap is not None:
        bh_cap.release()
        bh_cap = None
    ObjectDetection.reset_detection_state()
    try:
        if label_cam_widget.winfo_exists():
            black_img = Image.new('RGB', (DISPLAY_WIDTH, DISPLAY_HEIGHT), color='black')
            imgtk = ImageTk.PhotoImage(image=black_img)
            label_cam_widget.imgtk = imgtk
            label_cam_widget.config(image=imgtk)
    except tk.TclError:
        pass


def update_frame_handler(label_cam_widget, serial_object):
    global bh_cap, bh_camera_running
    if bh_camera_running and bh_cap and bh_cap.isOpened():
        ret, frame_from_cam = bh_cap.read()
        if ret:
            processed_frame = ObjectDetection.process_frame_for_detection(frame_from_cam, serial_object)
            if processed_frame is not None:
                frame_display_resized = cv2.resize(processed_frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
                frame_rgb = cv2.cvtColor(frame_display_resized, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                imgtk = ImageTk.PhotoImage(image=img)
                if label_cam_widget.winfo_exists():
                    label_cam_widget.imgtk = imgtk
                    label_cam_widget.config(image=imgtk)
            if label_cam_widget.winfo_exists() and bh_camera_running:
                label_cam_widget.after(10, lambda: update_frame_handler(label_cam_widget, serial_object))
        else:
            if label_cam_widget.winfo_exists() and bh_camera_running:
                label_cam_widget.after(10, lambda: update_frame_handler(label_cam_widget, serial_object))


def toggle_namcham_handler(current_magnet_state, btn_namcham_widget, send_command_func):
    cmd = 'u\r' if current_magnet_state == 0 else 'd\r'
    if send_command_func(cmd):
        if current_magnet_state == 0:
            btn_namcham_widget.config(text="OFF MAG", bg="#3bd952")
            return 1
        else:
            btn_namcham_widget.config(text="ON MAG", bg="#eb3b3b")
            return 0
    return None


def toggle_conveyor_handler(current_conveyor_state, btn_bangtai_widget, send_command_func):
    cmd = 'r\r' if current_conveyor_state == 0 else 'o\r'
    if send_command_func(cmd):
        if current_conveyor_state == 0:
            btn_bangtai_widget.config(text="OFF CONV", bg="#3bd952")
            return 1
        else:
            btn_bangtai_widget.config(text="ON CONV", bg="#eb3b3b")
            return 0
    return None


def _auto_mode_sequence_thread(send_command_func, label_cam_widget, serial_object):
    print("Chế độ AUTO: Bắt đầu chuỗi lệnh.")
    if not send_command_func("d\r"): print("Chế độ AUTO: Gửi 'd' thất bại. Dừng chuỗi."); return
    time.sleep(2)
    if not send_command_func("h\r"): print("Chế độ AUTO: Gửi 'h' thất bại. Dừng chuỗi."); return
    time.sleep(7)
    print("Chế độ AUTO: Hoàn tất gửi lệnh. Bật camera.")
    start_camera_handler(label_cam_widget, serial_object)
    time.sleep(2)
    if not send_command_func("r\r"): print("Chế độ AUTO: Gửi 'r' thất bại. Dừng chuỗi."); return


def run_auto_mode_sequence(send_command_func, label_cam_widget, serial_object):
    auto_thread = threading.Thread(target=_auto_mode_sequence_thread,
                                   args=(send_command_func, label_cam_widget, serial_object),
                                   daemon=True)
    auto_thread.start()