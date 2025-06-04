import math
from collections import deque
from controller import Robot, GPS, Gyro, InertialUnit, Keyboard, Display
from display_manager import render_all_plots

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self._previous_error = 0.0

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

def clamp(value, low, high):
    return max(low, min(value, high))

def process_keyboard_input(keyboard):
    roll_input, pitch_input, yaw_input, alt_input = 0.0, 0.0, 0.0, 0.0
    key = keyboard.getKey()
    
    while key > 0:
        if key == Keyboard.UP:
            pitch_input = -1.0
        elif key == Keyboard.DOWN:
            pitch_input = 1.0
        elif key == Keyboard.LEFT:
            roll_input = -1.0
        elif key == Keyboard.RIGHT:
            roll_input = 1.0
        elif key == ord('D'):
            yaw_input = -1.0
        elif key == ord('A'):
            yaw_input = 1.0
        elif key == ord('W'):
            alt_input = 1.0
        elif key == ord('S'):
            alt_input = -1.0
        key = keyboard.getKey()
    
    return roll_input, pitch_input, yaw_input, alt_input

def initialize_motor_geometry():
    com_w = {'x': -0.088, 'y': 0.0, 'z': -0.04}
    motor_raw_positions_w = {
        'FR': {'x': 0.0548537,  'y': -0.151294, 'z': -0.00280468},
        'BL': {'x': -0.177179, 'y':  0.127453, 'z': -0.0320282},
        'FL': {'x': 0.0548537,  'y':  0.151294, 'z': -0.00280468},
        'BR': {'x': -0.177179, 'y': -0.127453, 'z': -0.0320282}
    }
    
    motor_coords_body = {}
    motor_arm_lengths = {}
    motor_mapping_to_keys = {1: 'FR', 2: 'BL', 3: 'FL', 4: 'BR'}
    
    for idx, key in motor_mapping_to_keys.items():
        p_motor_w = motor_raw_positions_w[key]
        delta_x_w = p_motor_w['x'] - com_w['x']
        delta_y_w = p_motor_w['y'] - com_w['y']
        
        body_x = delta_y_w
        body_y = delta_x_w
        motor_coords_body[idx] = {'x': body_x, 'y': -body_y}
        motor_arm_lengths[idx] = math.sqrt(body_x*body_x + body_y*body_y)
    
    df = (motor_arm_lengths[1] + motor_arm_lengths[3]) / 2.0
    dr = (motor_arm_lengths[2] + motor_arm_lengths[4]) / 2.0
    
    theta_f_rad = math.atan2(abs(motor_coords_body[1]['y']), abs(motor_coords_body[1]['x']))
    sin_theta_f = math.sin(theta_f_rad)
    cos_theta_f = math.cos(theta_f_rad)
    
    theta_r_rad = math.atan2(abs(motor_coords_body[2]['y']), abs(motor_coords_body[2]['x']))
    sin_theta_r = math.sin(theta_r_rad)
    cos_theta_r = math.cos(theta_r_rad)
    
    return df, dr, sin_theta_f, cos_theta_f, sin_theta_r, cos_theta_r

def calculate_motor_forces(f, m1, m2, m3, df, dr, sin_theta_f, cos_theta_f, sin_theta_r, cos_theta_r, gamma, max_thrust):
    df_sin_f = df * sin_theta_f
    dr_sin_r = dr * sin_theta_r
    df_cos_f = df * cos_theta_f
    dr_cos_r = dr * cos_theta_r
    sin_denom = df_sin_f + dr_sin_r
    cos_denom = df_cos_f + dr_cos_r
    
    f1 = clamp(((dr_sin_r*f - m1)/sin_denom) - ((gamma*m2 - dr_cos_r*m3)/(gamma*cos_denom)), 0.0, max_thrust)
    f2 = clamp(((df_sin_f*f + m1)/sin_denom) + ((gamma*m2 + df_cos_f*m3)/(gamma*cos_denom)), 0.0, max_thrust)
    f3 = clamp(((dr_sin_r*f - m1)/sin_denom) + ((gamma*m2 - dr_cos_r*m3)/(gamma*cos_denom)), 0.0, max_thrust)
    f4 = clamp(((df_sin_f*f + m1)/sin_denom) - ((gamma*m2 + df_cos_f*m3)/(gamma*cos_denom)), 0.0, max_thrust)
    
    return f1, f2, f3, f4

def main():
    robot = Robot()
    time_step_ms = int(robot.getBasicTimeStep())
    dt_s = time_step_ms / 1000.0
    
    plot_display = robot.getDevice("display")
    plot_width = plot_display.getWidth()
    plot_height = plot_display.getHeight()
    print(f"Plotting display 'plotDisplay' initialized. W:{plot_width}, H:{plot_height}")
    
    max_history_points = plot_width
    
    altitude_desired_history = deque(maxlen=max_history_points)
    altitude_actual_history = deque(maxlen=max_history_points)
    roll_desired_history = deque(maxlen=max_history_points)
    roll_actual_history = deque(maxlen=max_history_points)
    pitch_desired_history = deque(maxlen=max_history_points)
    pitch_actual_history = deque(maxlen=max_history_points)
    yaw_desired_history = deque(maxlen=max_history_points)
    yaw_actual_history = deque(maxlen=max_history_points)
    
    mass_kg = 0.5069
    gravity_mps2 = 9.81
    hover_thrust_total_n = (mass_kg * gravity_mps2) / 4
    thrust_constant_ct = 0.00026
    torque_constant_ctau = 0.0000052
    gamma = torque_constant_ctau / thrust_constant_ct
    
    df, dr, sin_theta_f, cos_theta_f, sin_theta_r, cos_theta_r = initialize_motor_geometry()
    
    max_motor_omega_rad_s = 576.0
    max_thrust_per_motor_n = thrust_constant_ct * (max_motor_omega_rad_s**2)
    
    altitude_pid = PID(kp=1.7, ki=0.65, kd=1.2)
    pitch_pid = PID(kp=0.3, ki=0.8, kd=0.15)
    roll_pid = PID(kp=1, ki=0.1, kd=0.7)
    yaw_rate_pid = PID(kp=0.0, ki=0, kd=0.0)
    
    motor_device_names = ["front right propeller", "rear left propeller", "front left propeller", "rear right propeller"]
    motors = [robot.getDevice(name) for name in motor_device_names]
    for motor_device in motors:
        motor_device.setPosition(float('inf'))
        motor_device.setVelocity(0.0)
    
    imu = robot.getDevice("inertial unit")
    imu.enable(time_step_ms)
    gps = robot.getDevice("gps")
    gps.enable(time_step_ms)
    gyro = robot.getDevice("gyro")
    gyro.enable(time_step_ms)
    keyboard = Keyboard()
    keyboard.enable(time_step_ms)
    
    target_altitude_m = 1.0
    target_yaw_rad = imu.getRollPitchYaw()[2]
    
    max_roll_pitch_target_angle_deg = 10.0
    max_yaw_rate_target_deg_s = 45
    altitude_change_speed_mps = 0.5
    thrust_coeff = math.sqrt(1 / (2 * thrust_constant_ct))
    
    while robot.step(time_step_ms) != -1:
        roll_pitch_yaw_rad = imu.getRollPitchYaw()
        actual_roll_rad = roll_pitch_yaw_rad[0]
        actual_pitch_rad = roll_pitch_yaw_rad[1]
        actual_yaw_rad = roll_pitch_yaw_rad[2]
        
        gps_values_m = gps.getValues()
        actual_altitude_m = gps_values_m[2]
        
        roll_input, pitch_input, yaw_input, alt_input = process_keyboard_input(keyboard)
        
        target_altitude_m += alt_input * altitude_change_speed_mps * dt_s
        target_altitude_m = clamp(target_altitude_m, 0.2, 5.0)
        target_pitch_angle_rad = pitch_input * math.radians(max_roll_pitch_target_angle_deg)
        target_roll_angle_rad = roll_input * math.radians(max_roll_pitch_target_angle_deg)
        
        yaw_rate_speed_rad_s = math.radians(max_yaw_rate_target_deg_s)
        target_yaw_rad += yaw_input * yaw_rate_speed_rad_s * dt_s
        if target_yaw_rad > math.pi: target_yaw_rad -= 2 * math.pi
        elif target_yaw_rad < -math.pi: target_yaw_rad += 2 * math.pi
        
        altitude_desired_history.append(target_altitude_m)
        altitude_actual_history.append(actual_altitude_m)
        roll_desired_history.append(target_roll_angle_rad)
        roll_actual_history.append(actual_roll_rad)
        pitch_desired_history.append(target_pitch_angle_rad)
        pitch_actual_history.append(actual_pitch_rad)
        yaw_desired_history.append(target_yaw_rad)
        yaw_actual_history.append(actual_yaw_rad)
        
        f = hover_thrust_total_n + altitude_pid.update(target_altitude_m, actual_altitude_m, dt_s)
        m1 = pitch_pid.update(target_pitch_angle_rad, actual_pitch_rad, dt_s)
        m2 = roll_pid.update(target_roll_angle_rad, actual_roll_rad, dt_s)
        m3 = yaw_rate_pid.update(target_yaw_rad, actual_yaw_rad, dt_s)
        
        f1, f2, f3, f4 = calculate_motor_forces(f, m1, m2, m3, df, dr, sin_theta_f, cos_theta_f, sin_theta_r, cos_theta_r, gamma, max_thrust_per_motor_n)
        
        omega_1 = thrust_coeff * math.sqrt(f1)
        omega_2 = thrust_coeff * math.sqrt(f2)
        omega_3 = thrust_coeff * math.sqrt(f3)
        omega_4 = thrust_coeff * math.sqrt(f4)
        
        clamped_omega_1 = clamp(omega_1, 0.0, max_motor_omega_rad_s)
        clamped_omega_2 = clamp(omega_2, 0.0, max_motor_omega_rad_s)
        clamped_omega_3 = clamp(omega_3, 0.0, max_motor_omega_rad_s)
        clamped_omega_4 = clamp(omega_4, 0.0, max_motor_omega_rad_s)
        
        motors[0].setVelocity(-1 * clamped_omega_1)
        motors[1].setVelocity(-1 * clamped_omega_2)
        motors[2].setVelocity(clamped_omega_3)
        motors[3].setVelocity(clamped_omega_4)
        
        print("--- PID Control Outputs ---", flush=True)
        print(f"  Thrust Cmd (Total to Mixer): {f:>8.3f} N ", flush=True)
        print(f"  Pitch Moment Cmd (M1):     {m1:>8.3f} Nm", flush=True)
        print(f"  Roll Moment Cmd  (M2):     {m2:>8.3f} Nm", flush=True)
        print(f"  Yaw Moment Cmd   (M3):     {m3:>8.3f} Nm", flush=True)
        print(f" Target Altitude (M): {target_altitude_m:>8.3f} M", flush=True)
        print("\n--- Target Motor Thrusts (N) ---", flush=True)
        print(f"  Front Left (F3): {f3:>7.2f}   Front Right (F1): {f1:>7.2f}", flush=True)
        print(f"  Rear Left  (F2): {f2:>7.2f}   Rear Right  (F4): {f4:>7.2f}", flush=True)
        print("", flush=True)
        
        render_all_plots(plot_display, plot_width, plot_height, altitude_desired_history, altitude_actual_history, roll_desired_history, roll_actual_history, pitch_desired_history, pitch_actual_history, yaw_desired_history, yaw_actual_history)

if __name__ == "__main__":
    main()