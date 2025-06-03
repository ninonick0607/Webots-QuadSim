import math
from collections import deque
from controller import Robot, GPS, Gyro, InertialUnit, Keyboard
from controller import Display

# Define the PID class (minimal necessary comments)
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self._previous_error = 0.0
        self.reset()

    def update(self, setpoint, current_value, dt_s):
        error = setpoint - current_value
        self.integral += error * dt_s
        derivative = (error - self._previous_error) / dt_s
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self._previous_error = error
        return output

    def reset(self):
        self.integral = 0.0
        self._previous_error = 0.0

def CLAMP(value, low, high):
    return max(low, min(value, high))

def draw_axes(display_dev, x, y, width, height, min_val, max_val, tick_count=5):
    """
    Draws a border, Y‐axis (with ticks+labels) and X‐axis (with ticks+pixel labels)
    inside the rect (x,y) to (x+width-1, y+height-1).  Y maps [min_val..max_val].
    """
    # 1) Border
    display_dev.setColor(0x404040)  # dark gray
    display_dev.drawRectangle(x, y, width, height)

    # 2) Y‐axis line
    display_dev.setColor(0xFFFFFF)  # white axis
    display_dev.drawLine(x, y, x, y + height - 1)

    # 3) X‐axis line
    display_dev.drawLine(x, y + height - 1, x + width - 1, y + height - 1)

    # 4) Y‐ticks + numeric labels (bigger font, moved right of axis)
    display_dev.setFont("Arial", 14, True)
    for i in range(tick_count + 1):
        frac = i / tick_count
        val = min_val + (max_val - min_val) * (1 - frac)
        yy = y + int(frac * (height - 1))
        # tick mark length = 5 px
        display_dev.drawLine(x - 5, yy, x, yy)
        txt = f"{val:.2f}"
        display_dev.setColor(0xFFFF00)  # bright yellow
        # shift labels to the right of the Y‐axis by +3 pixels
        display_dev.drawText(txt, x + 3, yy - 7)

    # 5) X‐ticks + pixel‐index labels (slightly larger font)
    display_dev.setFont("Arial", 12, True)
    for i in range(tick_count + 1):
        xx = x + int(i * (width - 1) / tick_count)
        display_dev.setColor(0xFFFFFF)
        display_dev.drawLine(xx, y + height - 1, xx, y + height - 1 - 5)
        label = str(xx - x)
        display_dev.drawText(label, xx - (len(label)*6)//2, y + height + 2)

def draw_two_series_with_axes(display_dev, hist1, hist2, x, y, width, height,
                              color1, color2, min_val, max_val, label,
                              tick_count=5):
    """
    First draws axes (using draw_axes), then overlays:
      - hist1 in color1 (desired)
      - hist2 in color2 (actual)
    The x‐axis is [0..len(hist)-1], the y‐axis is [min_val..max_val].
    """
    # 1) draw axes + ticks + labels
    draw_axes(display_dev, x, y, width, height, min_val, max_val, tick_count)

    # 2) draw label in top‐left corner (inside the border)
    display_dev.setColor(0xFFFFFF)
    display_dev.setFont("Arial", 12, True)
    display_dev.drawText(label, x + 5, y + 15)

    maxlen = hist1.maxlen
    # 3) plot hist1 (desired) in color1
    n1 = len(hist1)
    if n1 >= 2:
        pts1 = list(hist1)
        for i in range(n1 - 1):
            xx1 = x + int(i * (width - 1) / (maxlen - 1))
            yy1_val = pts1[i]
            yy1 = y + int((max_val - yy1_val) * (height - 1) / (max_val - min_val))
            yy1 = CLAMP(yy1, y, y + height - 1)

            xx2 = x + int((i + 1) * (width - 1) / (maxlen - 1))
            yy2_val = pts1[i + 1]
            yy2 = y + int((max_val - yy2_val) * (height - 1) / (max_val - min_val))
            yy2 = CLAMP(yy2, y, y + height - 1)

            display_dev.setColor(color1)
            if xx1 == xx2 and yy1 == yy2:
                display_dev.drawPixel(xx1, yy1)
            else:
                display_dev.drawLine(xx1, yy1, xx2, yy2)

    # 4) plot hist2 (actual) in color2
    n2 = len(hist2)
    if n2 >= 2:
        pts2 = list(hist2)
        for i in range(n2 - 1):
            xx1 = x + int(i * (width - 1) / (maxlen - 1))
            yy1_val = pts2[i]
            yy1 = y + int((max_val - yy1_val) * (height - 1) / (max_val - min_val))
            yy1 = CLAMP(yy1, y, y + height - 1)

            xx2 = x + int((i + 1) * (width - 1) / (maxlen - 1))
            yy2_val = pts2[i + 1]
            yy2 = y + int((max_val - yy2_val) * (height - 1) / (max_val - min_val))
            yy2 = CLAMP(yy2, y, y + height - 1)

            display_dev.setColor(color2)
            if xx1 == xx2 and yy1 == yy2:
                display_dev.drawPixel(xx1, yy1)
            else:
                display_dev.drawLine(xx1, yy1, xx2, yy2)

def main():
    robot = Robot()
    TIME_STEP_MS = int(robot.getBasicTimeStep())
    DT_S = TIME_STEP_MS / 1000.0

    # ─── Initialize the “plotDisplay” device ───
    plot_display = robot.getDevice("display")
    plot_width = plot_display.getWidth()
    plot_height = plot_display.getHeight()
    print(f"Plotting display 'plotDisplay' initialized. W:{plot_width}, H:{plot_height}")

    # Plotting parameters
    MAX_HISTORY_POINTS = plot_width
    GRAPH_COUNT = 4
    PLOT_HEIGHT_PER_GRAPH = plot_height // GRAPH_COUNT

    # Histories for desired vs actual
    altitude_desired_history = deque(maxlen=MAX_HISTORY_POINTS)
    altitude_actual_history = deque(maxlen=MAX_HISTORY_POINTS)
    roll_desired_history = deque(maxlen=MAX_HISTORY_POINTS)
    roll_actual_history = deque(maxlen=MAX_HISTORY_POINTS)
    pitch_desired_history = deque(maxlen=MAX_HISTORY_POINTS)
    pitch_actual_history = deque(maxlen=MAX_HISTORY_POINTS)
    yaw_desired_history = deque(maxlen=MAX_HISTORY_POINTS)
    yaw_actual_history  = deque(maxlen=MAX_HISTORY_POINTS)

    # --- System Constants ---
    MASS_KG = 0.5069
    GRAVITY_MPS2 = 9.81
    HOVER_THRUST_TOTAL_N = (MASS_KG * GRAVITY_MPS2)/4 # Gives thrust to each motor 

    THRUST_CONSTANT_CT = 0.00026    # N/(rad/s)^2
    TORQUE_CONSTANT_CTAU = 0.0000052 # Nm/(rad/s)^2
    GAMMA_DRAG_THRUST_RATIO = TORQUE_CONSTANT_CTAU / THRUST_CONSTANT_CT # m

    # --- Geometric Parameters ---
    com_w = {'x': -0.088, 'y': 0.0, 'z': -0.04} # COM in Webots frame (X_w:fwd, Y_w:left, Z_w:up)
    motor_raw_positions_w = {
        'FR': {'x': 0.0548537,  'y': -0.151294, 'z': -0.00280468}, # M1
        'BL': {'x': -0.177179, 'y':  0.127453, 'z': -0.0320282},  # M2
        'FL': {'x': 0.0548537,  'y':  0.151294, 'z': -0.00280468},  # M3
        'BR': {'x': -0.177179, 'y': -0.127453, 'z': -0.0320282}   # M4
    }

    motor_coords_body = {} 
    motor_arm_lengths = {} 
    motor_mapping_to_keys = {1: 'FR', 2: 'BL', 3: 'FL', 4: 'BR'}

    for idx, key in motor_mapping_to_keys.items():
        p_motor_w = motor_raw_positions_w[key]
        delta_x_w = p_motor_w['x'] - com_w['x']
        delta_y_w = p_motor_w['y'] - com_w['y']
        
        body_x = delta_y_w # Our X (front)
        body_y = delta_x_w # Our Y (left)
        motor_coords_body[idx] = {'x': body_x, 'y': -body_y}
        motor_arm_lengths[idx] = math.sqrt(body_x*body_x + body_y*body_y)

    DF = (motor_arm_lengths[1] + motor_arm_lengths[3]) / 2.0
    DR = (motor_arm_lengths[2] + motor_arm_lengths[4]) / 2.0

    thetaF_rad = math.atan2(abs(motor_coords_body[1]['y']),abs(motor_coords_body[1]['x']))
    sin_theta_F = math.sin(thetaF_rad)
    cos_theta_F = math.cos(thetaF_rad)

    thetaR_rad = math.atan2(abs(motor_coords_body[2]['y']), abs(motor_coords_body[2]['x']))
    sin_theta_R = math.sin(thetaR_rad)
    cos_theta_R = math.cos(thetaR_rad)

    den_y_eff = 2 * (DF * sin_theta_F + DR * sin_theta_R)
    den_x_eff = 2 * (DF * cos_theta_F + DR * cos_theta_R)

    # --- Motor Limits ---
    MAX_MOTOR_OMEGA_RAD_S = 576.0
    MAX_THRUST_PER_MOTOR_N = THRUST_CONSTANT_CT * (MAX_MOTOR_OMEGA_RAD_S**2) 
    MIN_THRUST_PER_MOTOR_N = 0.0

    # --- PID Gains ---
    ALTITUDE_KP, ALTITUDE_KI, ALTITUDE_KD =1.7,0.65,1.2
    
    PITCH_KP, PITCH_KI, PITCH_KD =0.3,0.8,0.15
    
    ROLL_KP, ROLL_KI, ROLL_KD = 1, 2, 0.3
    
    YAW_KP, YAW_KI, YAW_KD = 0.5,0,0.7

# ------ Gains to Test ------
# 
#     ALTITUDE_KP, ALTITUDE_KI, ALTITUDE_KD =1.5,0.5,1
#
#     PITCH_KP, PITCH_KI, PITCH_KD =0.4,0.8,0.1
#
#     ROLL_KP, ROLL_KI, ROLL_KD = 1, 2, 0.3
# 
# 
# 
# # ===============================

    altitude_pid = PID(kp=ALTITUDE_KP, ki=ALTITUDE_KI, kd=ALTITUDE_KD)
    pitch_pid = PID(kp=PITCH_KP, ki=PITCH_KI, kd=PITCH_KD)
    roll_pid = PID(kp=ROLL_KP, ki=ROLL_KI, kd=ROLL_KD)
    yaw_rate_pid = PID(kp=YAW_KP, ki=YAW_KI, kd=YAW_KD)
    
    # --- Webots Devices ---
    MOTOR_DEVICE_NAMES = ["front right propeller", "rear left propeller", "front left propeller", "rear right propeller"]
    motors = [robot.getDevice(name) for name in MOTOR_DEVICE_NAMES]
    for motor_device in motors:
        motor_device.setPosition(float('inf'))
        motor_device.setVelocity(0.0)

    imu = robot.getDevice("inertial unit"); imu.enable(TIME_STEP_MS)
    gps = robot.getDevice("gps"); gps.enable(TIME_STEP_MS)
    gyro = robot.getDevice("gyro"); gyro.enable(TIME_STEP_MS)
    keyboard = Keyboard(); keyboard.enable(TIME_STEP_MS)

    target_altitude_m = 1.0 
    target_yaw_rad = imu.getRollPitchYaw()[2]
    while robot.step(TIME_STEP_MS) != -1:
        # --- Get Sensor Data ---
        roll_pitch_yaw_rad = imu.getRollPitchYaw() # Webots: Roll, Pitch, Yaw
        actual_roll_rad = roll_pitch_yaw_rad[0]   # Positive = right wing down
        actual_pitch_rad = roll_pitch_yaw_rad[1]  # Positive = nose up
        actual_yaw_rad = roll_pitch_yaw_rad[2]

        gps_values_m = gps.getValues()
        actual_altitude_m = gps_values_m[2]

        # --- Keyboard Input for Control Targets ---
        # M1 (Pitch moment) positive = nose up.
        # M2 (Roll moment) positive = roll right.
        # M3 (Yaw moment) positive = yaw left (CCW).
        roll_input_scaled, pitch_input_scaled, yaw_input_scaled, alt_input_scaled = 0.0, 0.0, 0.0, 0.0
        key = keyboard.getKey()
        while key > 0:
            if key == Keyboard.UP: pitch_input_scaled = -1.0 # Pitch forward (nose down -> negative M1)
            elif key == Keyboard.DOWN: pitch_input_scaled = 1.0 # Pitch backward (nose up -> positive M1)
            elif key == Keyboard.LEFT: roll_input_scaled = -1.0 # Roll left (-> negative M2)
            elif key == Keyboard.RIGHT: roll_input_scaled = 1.0 # Roll right (-> positive M2)
            elif key == ord('D'): yaw_input_scaled = -1.0 # Yaw right (CW -> negative M3)
            elif key == ord('A'): yaw_input_scaled = 1.0  # Yaw left (CCW -> positive M3)
            elif key == ord('W'): alt_input_scaled = 1.0
            elif key == ord('S'): alt_input_scaled = -1.0
            key = keyboard.getKey()
        
        MAX_ROLL_PITCH_TARGET_ANGLE_DEG = 10.0
        MAX_YAW_RATE_TARGET_DEG_S = 45
        ALTITUDE_CHANGE_SPEED_MPS = 0.5

        target_altitude_m += alt_input_scaled * ALTITUDE_CHANGE_SPEED_MPS * DT_S
        target_altitude_m = CLAMP(target_altitude_m, 0.2, 5.0)

        target_pitch_angle_rad = pitch_input_scaled * math.radians(MAX_ROLL_PITCH_TARGET_ANGLE_DEG)
        target_roll_angle_rad = roll_input_scaled * math.radians(MAX_ROLL_PITCH_TARGET_ANGLE_DEG)
        
        YAW_RATE_SPEED_RAD_S = math.radians(MAX_YAW_RATE_TARGET_DEG_S)
        target_yaw_rad += yaw_input_scaled * YAW_RATE_SPEED_RAD_S * DT_S
        if target_yaw_rad >  math.pi:
            target_yaw_rad -= 2 * math.pi
        elif target_yaw_rad < -math.pi:
            target_yaw_rad += 2 * math.pi
            
        # Append desired vs actual to histories
        altitude_desired_history.append(target_altitude_m)
        altitude_actual_history.append(actual_altitude_m)
        roll_desired_history.append(target_roll_angle_rad)
        roll_actual_history.append(actual_roll_rad)
        pitch_desired_history.append(target_pitch_angle_rad)
        pitch_actual_history.append(actual_pitch_rad)
        yaw_desired_history.append(target_yaw_rad)
        yaw_actual_history.append(actual_yaw_rad)
        # --- PID Control ---
        altitude_thrust_correction_N = altitude_pid.update(target_altitude_m, actual_altitude_m, DT_S)
        M1_pitch_Nm = pitch_pid.update(target_pitch_angle_rad, actual_pitch_rad, DT_S)
        M2_roll_Nm = roll_pid.update(target_roll_angle_rad, actual_roll_rad, DT_S)
        M3_yaw_Nm = yaw_rate_pid.update(target_yaw_rad, actual_yaw_rad, DT_S)

        # --- Calculate Total Desired Thrust and Apply Mixer ---
        f_cmd_N = HOVER_THRUST_TOTAL_N + altitude_thrust_correction_N

        # Mixer Inputs: f_cmd_N, M1_pitch_Nm, M2_roll_Nm, M3_yaw_Nm
        F1_N = CLAMP(((DR*sin_theta_R*f_cmd_N-M1_pitch_Nm)/(den_y_eff)) - ((GAMMA_DRAG_THRUST_RATIO*M2_roll_Nm-DR*cos_theta_R*M3_yaw_Nm)/(den_x_eff)),MIN_THRUST_PER_MOTOR_N,MAX_THRUST_PER_MOTOR_N)
        F2_N = CLAMP(((DF*sin_theta_F*f_cmd_N+M1_pitch_Nm)/(den_y_eff)) + ((GAMMA_DRAG_THRUST_RATIO*M2_roll_Nm+DF*cos_theta_R*M3_yaw_Nm)/(den_x_eff)),MIN_THRUST_PER_MOTOR_N,MAX_THRUST_PER_MOTOR_N)
        F3_N = CLAMP(((DR*sin_theta_R*f_cmd_N-M1_pitch_Nm)/(den_y_eff)) + ((GAMMA_DRAG_THRUST_RATIO*M2_roll_Nm-DR*cos_theta_R*M3_yaw_Nm)/(den_x_eff)),MIN_THRUST_PER_MOTOR_N,MAX_THRUST_PER_MOTOR_N)
        F4_N = CLAMP(((DF*sin_theta_F*f_cmd_N+M1_pitch_Nm)/(den_y_eff)) - ((GAMMA_DRAG_THRUST_RATIO*M2_roll_Nm+DF*cos_theta_R*M3_yaw_Nm)/(den_x_eff)),MIN_THRUST_PER_MOTOR_N,MAX_THRUST_PER_MOTOR_N)

        motor_target_thrusts_N = [F1_N, F2_N, F3_N, F4_N] # Order: FR, BL, FL, BR

        # --- Convert Thrust to Motor Omega and Set Velocities ---
        # Motor spin directions for positive omega command (Webots positive velocity = CW):
        MOTOR_VELOCITY_COMMAND_SIGNS = [-1.0, -1.0, 1.0, 1.0] 
        for i in range(4):
            clamped_thrust_N = motor_target_thrusts_N[i]

            target_omega_rad_s = 0.0
            if clamped_thrust_N > 1e-9:
                omega_squared = clamped_thrust_N / THRUST_CONSTANT_CT
                target_omega_rad_s = math.sqrt(omega_squared) 
            
            target_omega_rad_s = CLAMP(target_omega_rad_s, 0.0, MAX_MOTOR_OMEGA_RAD_S)
            motors[i].setVelocity(MOTOR_VELOCITY_COMMAND_SIGNS[i] * target_omega_rad_s)

        # --- Debugging ---
        print("--- PID Control Outputs ---", flush=True)
        print(f"  Thrust Cmd (Total to Mixer): {f_cmd_N:>8.3f} N (Altitude PID Correction: {altitude_thrust_correction_N:>8.3f} N)", flush=True)
        print(f"  Pitch Moment Cmd (M1):     {M1_pitch_Nm:>8.3f} Nm", flush=True)
        print(f"  Roll Moment Cmd  (M2):     {M2_roll_Nm:>8.3f} Nm", flush=True)
        print(f"  Yaw Moment Cmd   (M3):     {M3_yaw_Nm:>8.3f} Nm", flush=True)
        print(f" Target Altitude (M): {target_altitude_m:>8.3f} M",flush=True)
        print("\n--- Target Motor Thrusts (N) ---", flush=True)
        print(f"  Front Left (F3): {F3_N:>7.2f}   Front Right (F1): {F1_N:>7.2f}", flush=True)
        print(f"  Rear Left  (F2): {F2_N:>7.2f}   Rear Right  (F4): {F4_N:>7.2f}", flush=True)
        print("", flush=True)

        # --- Plotting Desired vs Actual for Altitude, Roll, Pitch ---
        plot_display.setColor(0x000000)
        plot_display.fillRectangle(0, 0, plot_width, plot_height)
        # Insert these computations just before calling draw_two_series_with_axes for each subplot

        # 1) ALTITUDE plot bounds:
        all_alt = list(altitude_desired_history) + list(altitude_actual_history)
        if all_alt:
            min_alt = min(all_alt)
            max_alt = max(all_alt)
            if abs(max_alt - min_alt) < 1e-6:
                max_alt = min_alt + 1e-3
        else:
            min_alt, max_alt = 0.0, 1.0

        draw_two_series_with_axes(
            plot_display,
            altitude_desired_history,
            altitude_actual_history,
            0,
            0,
            plot_width,
            PLOT_HEIGHT_PER_GRAPH,
            0xFFFF00,  # desired=yellow
            0x00FF00,  # actual=green
            min_alt,
            max_alt,
            "Altitude (m)",
            tick_count=5
        )

        # 2) ROLL plot bounds:
        all_roll = list(roll_desired_history) + list(roll_actual_history)
        if all_roll:
            min_roll = min(all_roll)
            max_roll = max(all_roll)
            if abs(max_roll - min_roll) < 1e-6:
                max_roll = min_roll + 1e-3
        else:
            min_roll, max_roll = -0.2, 0.2

        draw_two_series_with_axes(
            plot_display,
            roll_desired_history,
            roll_actual_history,
            0,
            PLOT_HEIGHT_PER_GRAPH,
            plot_width,
            PLOT_HEIGHT_PER_GRAPH,
            0xFFFF00,  # desired=yellow
            0xFF0000,  # actual=red
            min_roll,
            max_roll,
            "Roll (rad)",
            tick_count=5
        )

        # 3) PITCH plot bounds:
        all_pitch = list(pitch_desired_history) + list(pitch_actual_history)
        if all_pitch:
            min_pitch = min(all_pitch)
            max_pitch = max(all_pitch)
            if abs(max_pitch - min_pitch) < 1e-6:
                max_pitch = min_pitch + 1e-3
        else:
            min_pitch, max_pitch = -0.2, 0.2

        draw_two_series_with_axes(
            plot_display,
            pitch_desired_history,
            pitch_actual_history,
            0,
            2 * PLOT_HEIGHT_PER_GRAPH,
            plot_width,
            PLOT_HEIGHT_PER_GRAPH,
            0xFFFF00,  # desired=yellow
            0x0000FF,  # actual=blue
            min_pitch,
            max_pitch,
            "Pitch (rad)",
            tick_count=5
        )
        all_yaw = list(yaw_desired_history) + list(yaw_actual_history)
        if all_yaw:
            min_yaw = min(all_yaw)
            max_yaw = max(all_yaw)
            if abs(max_yaw - min_yaw) < 1e-6:
                max_yaw = min_yaw + 1e-3
        else:
            min_yaw, max_yaw = -0.5, 0.5  # choose a default range

        draw_two_series_with_axes(
            plot_display,
            yaw_desired_history,
            yaw_actual_history,
            0,
            3 * PLOT_HEIGHT_PER_GRAPH,
            plot_width,
            PLOT_HEIGHT_PER_GRAPH,
            0xFFFF00,  # desired=yellow
            0xFF00FF,  # actual=magenta (for contrast)
            min_yaw,
            max_yaw,
            "Yaw Rate (rad/s)",
            tick_count=5
        )


if __name__ == "__main__":
    main()