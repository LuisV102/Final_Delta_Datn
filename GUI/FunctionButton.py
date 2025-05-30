import tkinter as tk
from tkinter import messagebox
import Kinematic
import cv2
from PIL import Image, ImageTk
import ObjectDetection
import time
import threading

# Biến cục bộ cho module này (dùng cho camera)
# Đổi tên để tránh trùng với biến ở main nếu có
bh_cap = None
bh_camera_running = False
# Kích thước hiển thị mong muốn trên GUI
DISPLAY_WIDTH = 720
DISPLAY_HEIGHT = 370

# - 'send_command_func' sẽ là hàm send_command_to_serial từ file chính
# - Các 'entry_...' là các đối tượng widget Entry từ file chính
# - Các 'btn_...' là các đối tượng widget Button từ file chính (nếu cần thay đổi text/bg của chúng)

def simple_command_handler(send_command_func, command):
    send_command_func(command)
def send_angles_handler(entry_theta1, entry_theta2, entry_theta3,
                        entry_x, entry_y, entry_z, ser_object):  # Truyền ser_object
    try:
        theta1 = float(entry_theta1.get())
        theta2 = float(entry_theta2.get())
        theta3 = float(entry_theta3.get())

        if not (0 <= theta1 <= 60):
            messagebox.showerror("Error", "Theta 1 phải trong khoảng 0 đến 60")  # Sửa giới hạn nếu cần
            return
        if not (0 <= theta2 <= 60):
            messagebox.showerror("Error", "Theta 2 phải trong khoảng 0 đến 60")  # Sửa giới hạn nếu cần
            return
        if not (0 <= theta3 <= 60):
            messagebox.showerror("Error", "Theta 3 phải trong khoảng 0 đến 60")  # Sửa giới hạn nếu cần
            return

        try:
            result = Kinematic.forward_kinematic(theta1, theta2, theta3)
            if result[0] is False:
                messagebox.showerror("Error", "Không thể tính vị trí. Kiểm tra lại góc đầu vào.")
                return

            _, x, y, z = result

            if not (-100 <= x <= 87):
                messagebox.showerror("Lỗi", f"X={x:.2f} nằm ngoài giới hạn robot")
                return
            if not (-80 <= y <= 130):
                messagebox.showerror("Lỗi", f"Y={y:.2f} nằm ngoài giới hạn robot")
                return
            if not (-397 <= z <= -307.38):
                messagebox.showerror("Lỗi", f"Z={z:.2f} nằm ngoài giới hạn robot")
                return

            data = f"{theta1}A{theta2}B{theta3}C\r"
            # print(f"Gửi: {data.strip()}")
            if ser_object and ser_object.is_open:
                # ser_object.write(data.encode())
                import __main__  # Cách để truy cập hàm từ GiaoDien.py nếu không truyền trực tiếp
                if not __main__.send_command_to_serial(data):  # Sử dụng hàm từ GiaoDien.py
                    return  # Không cập nhật UI nếu gửi thất bại
            else:
                messagebox.showwarning("Serial Port Error", "Serial port not available.")
                return

            entry_x.config(state='normal')
            entry_y.config(state='normal')
            entry_z.config(state='normal')
            entry_x.delete(0, tk.END);
            entry_x.insert(0, f"{x:.2f}")
            entry_y.delete(0, tk.END);
            entry_y.insert(0, f"{y:.2f}")
            entry_z.delete(0, tk.END);
            entry_z.insert(0, f"{z:.2f}")
            entry_x.config(state='readonly')
            entry_y.config(state='readonly')
            entry_z.config(state='readonly')

        except Exception as e:
            print(f"Lỗi khi tính forward kinematic: {e}")
            messagebox.showerror("Lỗi", "Không thể tính vị trí. Kiểm tra lại hàm forward_kinematic.")

    except ValueError:
        messagebox.showerror("Error", "Vui lòng nhập đúng định dạng là số!")

def calculate_inv_kinematic_handler(entry_x_ik, entry_y_ik, entry_z_ik,
                                    entry_theta1_ik, entry_theta2_ik, entry_theta3_ik):
    try:
        x_val = float(entry_x_ik.get())
        y_val = float(entry_y_ik.get())
        z_val = float(entry_z_ik.get())

        angles = Kinematic.inverse_kinematic(x_val, y_val, z_val)

        entry_theta1_ik.config(state=tk.NORMAL);
        entry_theta1_ik.delete(0, tk.END);
        entry_theta1_ik.insert(0, f"{angles[0]:.2f}");
        entry_theta1_ik.config(state='readonly')
        entry_theta2_ik.config(state=tk.NORMAL);
        entry_theta2_ik.delete(0, tk.END);
        entry_theta2_ik.insert(0, f"{angles[1]:.2f}");
        entry_theta2_ik.config(state='readonly')
        entry_theta3_ik.config(state=tk.NORMAL);
        entry_theta3_ik.delete(0, tk.END);
        entry_theta3_ik.insert(0, f"{angles[2]:.2f}");
        entry_theta3_ik.config(state='readonly')

    except ValueError:
        messagebox.showerror("Lỗi", "Vui lòng nhập giá trị số hợp lệ cho X, Y, Z.")
    except Exception as e:  # Bắt lỗi cụ thể từ Kinematic.inverse_kinematic nếu có
        messagebox.showerror("Lỗi tính toán", str(e))


def send_trajectory_handler(entry_x0, entry_y0, entry_z0, entry_c0,
                            entry_xf, entry_yf, entry_zf, entry_tf,
                            send_command_func):  # send_command_func là hàm send_command_to_serial
    try:
        x0_val = float(entry_x0.get())
        y0_val = float(entry_y0.get())
        z0_val = float(entry_z0.get())
        c0_val = entry_c0.get().strip().upper() # strip để loại bỏ khoảng trắng thừa ở đầu hoặc cuối chuỗi
        xf_val = float(entry_xf.get())
        yf_val = float(entry_yf.get())
        zf_val = float(entry_zf.get())
        tf_val = float(entry_tf.get())

        # Kiểm tra giới hạn
        if not (-120 <= x0_val <= 87): messagebox.showerror("Lỗi", f"X0={x0_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-110 <= y0_val <= 130): messagebox.showerror("Lỗi", f"Y0={y0_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-397 <= z0_val <= -307.38): messagebox.showerror("Lỗi",
                                                                 f"Z0={z0_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-120 <= xf_val <= 87): messagebox.showerror("Lỗi", f"Xf={xf_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-110 <= yf_val <= 130): messagebox.showerror("Lỗi", f"Yf={yf_val:.2f} nằm ngoài giới hạn robot"); return
        if not (-397 <= zf_val <= -307.38): messagebox.showerror("Lỗi",
                                                                 f"Zf={zf_val:.2f} nằm ngoài giới hạn robot"); return

        base_data_segment = f"P0:{x0_val},{y0_val},{z0_val};Pf:{xf_val},{yf_val},{zf_val};T:{tf_val}"
        data_to_send = ""

        if c0_val and c0_val not in ['R', 'G', 'Y']:
            messagebox.showerror("Lỗi", f"Nhập C0 là R(red), G(green) hoặc Y(yellow)")
            return
        else:
            data_to_send = f"{base_data_segment}\r"

        if send_command_func(data_to_send):
            entry_x0.delete(0, tk.END);
            entry_x0.insert(0, str(xf_val))
            entry_y0.delete(0, tk.END);
            entry_y0.insert(0, str(yf_val))
            entry_z0.delete(0, tk.END);
            entry_z0.insert(0, str(zf_val))
            entry_xf.delete(0, tk.END)
            entry_yf.delete(0, tk.END)
            entry_zf.delete(0, tk.END)
            entry_tf.delete(0, tk.END)
            entry_c0.delete(0, tk.END)

    except ValueError:
        messagebox.showerror("Lỗi", "Vui lòng nhập giá trị số hợp lệ cho tọa độ và thời gian.")
    except Exception as e:
        messagebox.showerror("Lỗi", f"Đã xảy ra lỗi: {str(e)}")

def set_home_handler(send_command_func, entry_x, entry_y, entry_z):
    if send_command_func('h\r'):
        try:
            result = Kinematic.forward_kinematic(0, 0, 0)
            _, x, y, z = result
            entry_x.config(state='normal') # cho phép chỉnh sửa entry
            entry_x.delete(0, tk.END)      # xóa nội dung hiện tại của entry
            entry_x.insert(0, f"{x:.2f}")
            entry_x.config(state='readonly')

            entry_y.config(state='normal')
            entry_y.delete(0, tk.END)
            entry_y.insert(0, f"{y:.2f}")
            entry_y.config(state='readonly')

            entry_z.config(state='normal')
            entry_z.delete(0, tk.END)
            entry_z.insert(0, f"{z:.2f}")
            entry_z.config(state='readonly')

            # Re-enable lại các nút điều khiển
            # import __main__  # Truy cập biến từ file chính
            # for btn in __main__.controllable_buttons:
            #     btn.config(state=tk.NORMAL)

            return True  #  Thành công
        except Exception as e:
            print(f"Lỗi khi tính toán hoặc cập nhật: {e}")
            return False  # Có lỗi xảy ra khi xử lý
    else:
        return False  # Gửi lệnh thất bại
def start_camera_handler(label_cam_widget, serial_object):  # Thêm serial_object
    global bh_cap, bh_camera_running

    if not bh_camera_running:
        try:
            # THỬ NGAY ĐÂY: Chỉ định API Backend
            # Lựa chọn 1: Dùng DirectShow (thường nhanh cho USB cams trên Windows)
            bh_cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            print("Attempting to open camera with DirectShow (DSHOW)...")

            # Kiểm tra nếu DSHOW không thành công, thử MSMF hoặc mặc định
            if not bh_cap or not bh_cap.isOpened():
                print("DSHOW failed or camera not opened. Trying MSMF...")
                bh_cap = cv2.VideoCapture(0, cv2.CAP_MSMF) # Lựa chọn 2: Media Foundation
                if not bh_cap or not bh_cap.isOpened():
                    print("MSMF failed or camera not opened. Trying default API...")
                    bh_cap = cv2.VideoCapture(0) # Lựa chọn 3: Để OpenCV tự quyết định (như cũ)

            if not bh_cap or not bh_cap.isOpened():
                messagebox.showerror("Camera Error", "Không thể mở camera. Hãy kiểm tra kết nối và thử lại.")
                if bh_cap: # Đảm bảo release nếu đối tượng được tạo nhưng không isOpened()
                    bh_cap.release()
                bh_cap = None
                bh_camera_running = False # Đảm bảo trạng thái đúng
                return

            print(f"Camera opened successfully using API backend: {bh_cap.getBackendName()}")

            # Thiết lập kích thước frame từ camera
            # Bạn có thể thử đặt các thông số này SAU KHI camera đã mở thành công
            # và xem có ảnh hưởng đến tốc độ không. Thông thường là không đáng kể.
            bh_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            bh_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # Lấy lại kích thước thực tế sau khi set, phòng trường hợp camera không hỗ trợ chính xác
            actual_width = bh_cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = bh_cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Requested 640x480, Actual camera resolution: {actual_width}x{actual_height}")


            bh_camera_running = True
            ObjectDetection.reset_detection_state()

            update_frame_handler(label_cam_widget, serial_object)

        except Exception as e:
            messagebox.showerror("Camera Error", f"Lỗi khi khởi động camera: {e}")
            bh_camera_running = False
            if bh_cap and bh_cap.isOpened():
                bh_cap.release()
            bh_cap = None

def stop_camera_stream_handler(label_cam_widget):
    global bh_cap, bh_camera_running
    bh_camera_running = False  # Tín hiệu dừng vòng lặp update_frame_handler
    if bh_cap is not None:
        bh_cap.release()
        bh_cap = None

    ObjectDetection.reset_detection_state()  # Reset trạng thái khi dừng camera

    # Xóa ảnh khỏi label, hiển thị ảnh đen
    try:
        if label_cam_widget.winfo_exists():
            black_img = Image.new('RGB', (DISPLAY_WIDTH, DISPLAY_HEIGHT), color='black')
            imgtk = ImageTk.PhotoImage(image=black_img)
            label_cam_widget.imgtk = imgtk
            label_cam_widget.config(image=imgtk)
    except tk.TclError:  # Widget có thể đã bị hủy
        pass


def update_frame_handler(label_cam_widget, serial_object):  # Thêm serial_object
    global bh_cap, bh_camera_running

    if bh_camera_running and bh_cap and bh_cap.isOpened():
        ret, frame_from_cam = bh_cap.read()
        if ret:
            # Gọi hàm xử lý từ ObjectDetection
            # Hàm này sẽ trả về frame đã được vẽ vời các thông tin nhận diện
            processed_frame = ObjectDetection.process_frame_for_detection(frame_from_cam, serial_object)

            if processed_frame is not None:
                # Resize frame đã xử lý về kích thước hiển thị mong muốn
                frame_display_resized = cv2.resize(processed_frame, (DISPLAY_WIDTH, DISPLAY_HEIGHT))
                frame_rgb = cv2.cvtColor(frame_display_resized, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame_rgb)
                imgtk = ImageTk.PhotoImage(image=img)

                if label_cam_widget.winfo_exists():
                    label_cam_widget.imgtk = imgtk
                    label_cam_widget.config(image=imgtk)
            else:
                print("Processed frame is None.")  # Xử lý lỗi nếu cần

            # Lặp lại việc cập nhật frame
            if label_cam_widget.winfo_exists() and bh_camera_running:
                label_cam_widget.after(10, lambda: update_frame_handler(label_cam_widget, serial_object))
            else:  # Nếu widget không còn hoặc camera đã dừng
                if bh_camera_running:  # Nếu camera vẫn được cho là đang chạy nhưng widget mất
                    stop_camera_stream_handler(label_cam_widget)  # Dừng hẳn
        else:
            print("Không đọc được frame từ camera.")
            # Có thể dừng camera ở đây nếu đọc frame thất bại liên tục
            # stop_camera_stream_handler(label_cam_widget)
            if label_cam_widget.winfo_exists() and bh_camera_running:  # Vẫn thử lại nếu camera chưa bị stop
                label_cam_widget.after(10, lambda: update_frame_handler(label_cam_widget, serial_object))


def toggle_namcham_handler(current_magnet_state, btn_namcham_widget, send_command_func):
    cmd = 'u\r' if current_magnet_state == 0 else 'd\r'
    if send_command_func(cmd): # gửi lệnh qua serial, chỉ tiếp tục code nếu True
        if current_magnet_state == 0:
            btn_namcham_widget.config(text="ON MAG", bg="#3bd952")
            return 1  # Trả về trạng thái mới
        else:
            btn_namcham_widget.config(text="OFF MAG", bg="#eb3b3b")
            return 0  # Trả về trạng thái mới
    return None  # Trả về None nếu gửi lệnh thất bại, để không thay đổi trạng thái


def toggle_conveyor_handler(current_conveyor_state, btn_bangtai_widget, send_command_func):
    cmd = 'r\r' if current_conveyor_state == 0 else 'o\r'  # Theo code gốc của bạn
    if send_command_func(cmd):
        if current_conveyor_state == 0:
            btn_bangtai_widget.config(text="ON CONV", bg="#3bd952")
            return 1
        else:
            btn_bangtai_widget.config(text="OFF CONV", bg="#eb3b3b")
            return 0
    return None
# --- HÀM MỚI CHO CHẾ ĐỘ AUTO ---
def _auto_mode_sequence_thread(send_command_func, label_cam_widget, serial_object):
    """
    Hàm mục tiêu cho luồng (thread) để thực hiện chuỗi lệnh auto.
    Không gọi trực tiếp hàm này từ GUI, hãy gọi run_auto_mode_sequence.
    """
    print("Chế độ AUTO: Bắt đầu chuỗi lệnh.")
    if not send_command_func("r\r"):
        print("Chế độ AUTO: Gửi 'o' thất bại. Dừng chuỗi.")
        return
    time.sleep(2)

    if not send_command_func("d\r"):
        print("Chế độ AUTO: Gửi 'd' thất bại. Dừng chuỗi.")
        return
    time.sleep(2)

    if not send_command_func("h\r"):
        print("Chế độ AUTO: Gửi 'h' thất bại. Dừng chuỗi.")
        return
    time.sleep(1.5) # Có thể thêm một khoảng nghỉ nhỏ trước khi bật camera

    print("Chế độ AUTO: Hoàn tất gửi lệnh. Bật camera.")
    # Gọi hàm start_camera_handler.
    # start_camera_handler đã xử lý việc chạy trong luồng riêng cho việc cập nhật frame
    # nên chúng ta có thể gọi trực tiếp từ đây.
    # Tuy nhiên, để đảm bảo start_camera_handler không bị gọi chồng chéo nếu user click nhanh,
    # chúng ta nên kiểm tra bh_camera_running trước.
    # start_camera_handler đã có kiểm tra này rồi.
    start_camera_handler(label_cam_widget, serial_object)


def run_auto_mode_sequence(send_command_func, label_cam_widget, serial_object):
    """
    Hàm này sẽ được gọi từ GiaoDien.py khi người dùng chọn chế độ AUTO.
    Nó sẽ tạo một luồng mới để chạy chuỗi lệnh _auto_mode_sequence_thread.
    """
    # Tạo một luồng mới để chạy chuỗi lệnh, tránh làm treo GUI
    auto_thread = threading.Thread(target=_auto_mode_sequence_thread,
                                   args=(send_command_func, label_cam_widget, serial_object),
                                   daemon=True) # daemon=True để luồng tự tắt khi chương trình chính tắt
    auto_thread.start()