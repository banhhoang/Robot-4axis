import math

class RoArmM2SKinematics:
    def __init__(self):
        # Thông số hình học thực tế từ bản vẽ CAD (đơn vị: mm)
        self.L1 = 126.06  # Base to Shoulder
        self.L2 = 236.82  # Shoulder to Elbow
        self.L3 = 30.00   # Elbow Offset
        self.L4 = 280.15  # Elbow to Tip (Gripper)
        
        # Giới hạn tầm với tối đa (Max Reach)
        self.max_reach = 546.97 # L2 + L3 + L4

    def forward_kinematics(self, t1, t2, t3, t4):
        """
        Tính toán Động học thuận (FK)
        Input: Góc khớp theta1, theta2, theta3, theta4 (Radian)
        Output: Tọa độ X, Y, Z (mm) và góc Pitch (Radian)
        """
        # Tính khoảng cách hình chiếu trên mặt phẳng ngang r
        r = self.L2 * math.cos(t2) + \
            self.L3 * math.cos(t2 + t3) + \
            self.L4 * math.cos(t2 + t3 + t4)
            
        x = r * math.cos(t1)
        y = r * math.sin(t1)
        z = self.L1 + self.L2 * math.sin(t2) + \
            self.L3 * math.sin(t2 + t3) + \
            self.L4 * math.sin(t2 + t3 + t4)
            
        pitch = t2 + t3 + t4
        
        return round(x, 2), round(y, 2), round(z, 2), round(pitch, 3)

    def inverse_kinematics(self, x, y, z, pitch):
        """
        Tính toán Động học nghịch (IK)
        Input: Tọa độ mục tiêu X, Y, Z (mm) và góc Pitch mong muốn (Radian)
        Output: Tuple (t1, t2, t3, t4) hoặc None nếu ngoài vùng làm việc
        """
        try:
            # 1. Tính Góc Đế (theta 1)
            t1 = math.atan2(y, x)
            
            # 2. Tính hình chiếu r trên mặt phẳng ngang
            r = math.sqrt(x**2 + y**2)
            
            # 3. Tìm tọa độ tâm cổ tay (Wrist Center) bằng cách lùi lại khâu L4
            # Dựa trên góc Pitch (phi) người dùng nhập vào
            rw = r - self.L4 * math.cos(pitch)
            zw = z - self.L1 - self.L4 * math.sin(pitch)
            
            # 4. Giải bài toán 2-DOF cho L2 và L3 bằng định lý hàm số Cosin
            # Tính D để kiểm tra điều kiện Workspace
            D = (rw**2 + zw**2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            
            if not (-1 <= D <= 1):
                raise ValueError("Tọa độ nằm ngoài tầm với của robot!")
            
            # Tính theta 3 (Cấu hình Elbow-up)
            t3 = math.atan2(math.sqrt(1 - D**2), D)
            
            # Tính theta 2 (Góc Vai)
            t2 = math.atan2(zw, rw) - math.atan2(self.L3 * math.sin(t3), self.L2 + self.L3 * math.cos(t3))
            
            # 5. Tính theta 4 (Góc Cổ tay) từ tổng góc Pitch
            t4 = pitch - t2 - t3
            
            return round(t1, 3), round(t2, 3), round(t3, 3), round(t4, 3)
            
        except Exception as e:
            print(f"Lỗi tính toán IK: {e}")
            return None

# --- CHƯƠNG TRÌNH THỰC NGHIỆM ---
if __name__ == "__main__":
    robot = RoArmM2SKinematics()
    
    # Giả lập tọa độ nhập từ Web UI (Tab Tọa độ)
    # Ví dụ: X=300, Y=0, Z=240, Pitch=3.14 (hướng thẳng xuống)
    target_x, target_y, target_z, target_pitch = 300, 0, 240, 3.14
    
    print(f"--- ĐIỀU KHIỂN TỌA ĐỘ MỤC TIÊU ---")
    print(f"Input: X={target_x}mm, Y={target_y}mm, Z={target_z}mm, Pitch={target_pitch}rad")
    
    angles = robot.inverse_kinematics(target_x, target_y, target_z, target_pitch)
    
    if angles:
        theta1, theta2, theta3, theta4 = angles
        print(f"\n--- KẾT QUẢ ĐỘNG HỌC NGHỊCH (IK) ---")
        print(f"Góc khớp cần quay (Radian):")
        print(f" - Đế (Base): {theta1}")
        print(f" - Vai (Shoulder): {theta2}")
        print(f" - Khuỷu (Elbow): {theta3}")
        print(f" - Cổ tay (Pitch): {theta4}")
        
        # Kiểm tra lại bằng Động học thuận
        x_fk, y_fk, z_fk, p_fk = robot.forward_kinematics(theta1, theta2, theta3, theta4)
        print(f"\n--- KIỂM CHỨNG ĐỘNG HỌC THUẬN (FK) ---")
        print(f"Tọa độ thực tế đầu kẹp đạt được: X={x_fk}, Y={y_fk}, Z={z_fk}, Pitch={p_fk}")
