import numpy as np
import math
import matplotlib.pyplot as plt  # Import matplotlib

# --- 1. Constants and Global Parameters ---
# These constants should match your Arduino sketch
DEG_PER_PULSE = 0.225
PULSE_PER_DEG = 1.0 / DEG_PER_PULSE

# --- Robot Constants (from your Inverse Kinematics function) ---
R_Base = 60.0
R_platform = 42.62
r_const = R_Base - R_platform  # Renamed to avoid potential conflicts
re = 150.0
rf = 350.0
THRESHOLD = 0.001
ALPHA_DEG = [0, 120, 240]


# --- 2. Trajectory Function (Quintic Polynomial Trajectory) ---
def calculate_q(t, q0, delta_q, tf):
    """
    Calculates the value of q(t) at time t using a 5th-order polynomial trajectory.

    Args:
        t (float): Current time.
        q0 (float): Initial value of q (q(0)).
        delta_q (float): Total change in q (q(tf) - q(0)).
        tf (float): Total time for the trajectory.

    Returns:
        float: The value of q at time t.
    """
    if t < 0:
        return q0
    if t >= tf:
        return q0 + delta_q

    tau = t / tf
    q_t = q0 + delta_q * (10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5)
    return q_t


# --- 3. Inverse Kinematics Function (Your provided function, adapted for Python) ---
def inverse_kinematic(X_ee, Y_ee, Z_ee):
    """
    Inverse Kinematics function for a Delta/parallel robot.
    Adapted from your C++ code to Python.

    Args:
        X_ee (float): X-coordinate of the end-effector.
        Y_ee (float): Y-coordinate of the end-effector.
        Z_ee (float): Z-coordinate of the end-effector.

    Returns:
        tuple: (angle_deg_1, angle_deg_2, angle_deg_3) joint angles in degrees,
               or None if no valid solution is found.
    """
    J = [0.0, 0.0, 0.0]  # Array to store joint angles

    for i in range(3):
        alpha = math.radians(ALPHA_DEG[i])  # Convert alpha from degrees to radians
        cos_alpha = math.cos(alpha)
        sin_alpha = math.sin(alpha)

        A = -2.0 * re * (-r_const + X_ee * cos_alpha + Y_ee * sin_alpha)
        B = -2.0 * re * Z_ee
        C = (X_ee * X_ee + Y_ee * Y_ee + Z_ee * Z_ee + r_const * r_const + re * re - rf * rf
             - 2 * r_const * (X_ee * cos_alpha + Y_ee * sin_alpha))

        denominator_val = A * A + B * B
        if denominator_val < 1e-12:  # Check to avoid division by a very small number (near 0)
            return None  # Cannot calculate if denominator is too small

        denominator = math.sqrt(denominator_val)

        # Check for real solution existence for acos
        acos_arg = -C / denominator
        if abs(acos_arg) > 1.0 + 1e-6:  # Add a small tolerance for floating-point errors
            return None  # No real solution found

        theta1 = math.atan2(B, A) + math.acos(acos_arg)
        theta2 = math.atan2(B, A) - math.acos(acos_arg)

        theta = theta2  # Based on your C++ logic, choose theta2

        if abs(theta) < THRESHOLD:
            theta = 0.0

        J[i] = -math.degrees(theta)  # Store in the output array

    return J[0], J[1], J[2]


# --- 4. Motor Parameter Update Function (adapted from C++ to Python) ---
def update_motor_parameters(tf_segment, current_angle_deg_1, current_angle_deg_2, current_angle_deg_3):
    """
    Calculates nPulse and delay_run_spd for each motor based on angles and segment time.

    Args:
        tf_segment (float): Motion time for this small trajectory segment (seconds).
        current_angle_deg_1 (float): Current angle for motor 1 (degrees).
        current_angle_deg_2 (float): Current angle for motor 2 (degrees).
        current_angle_deg_3 (float): Current angle for motor 3 (degrees).

    Returns:
        tuple: (delay_run_spd_1, delay_run_spd_2, delay_run_spd_3) in microseconds.
    """
    nPulse_1 = int(abs(current_angle_deg_1) * PULSE_PER_DEG)
    nPulse_2 = int(abs(current_angle_deg_2) * PULSE_PER_DEG)
    nPulse_3 = int(abs(current_angle_deg_3) * PULSE_PER_DEG)

    delay_run_spd_1 = 0
    delay_run_spd_2 = 0
    delay_run_spd_3 = 0

    # Convert tf from seconds to microseconds (1e6)
    # Divide by 2 because each pulse (step) requires one HIGH and one LOW (if using step/dir driver)
    if tf_segment > 0:  # Ensure no division by zero
        if nPulse_1 > 0:
            delay_run_spd_1 = int((tf_segment * 1e6) / (nPulse_1 * 2))
        if nPulse_2 > 0:
            delay_run_spd_2 = int((tf_segment * 1e6) / (nPulse_2 * 2))
        if nPulse_3 > 0:
            delay_run_spd_3 = int((tf_segment * 1e6) / (nPulse_3 * 2))

    return delay_run_spd_1, delay_run_spd_2, delay_run_spd_3


# --- 5. Main function to generate data and write to .h file ---
def generate_trajectory_data(
        q0_x, qf_x,
        q0_y, qf_y,
        q0_z, qf_z,
        tf_total_trajectory,
        num_points,
        output_filename="trajectory_data.h",
        plot_results=True  # New argument to control plotting
):
    """
    Generates trajectory data, calculates angles and delay_run_spd, then saves to a .h file.

    Args:
        q0_x, qf_x (float): Initial and final X-coordinates of the end-effector.
        q0_y, qf_y (float): Initial and final Y-coordinates of the end-effector.
        q0_z, qf_z (float): Initial and final Z-coordinates of the end-effector.
        tf_total_trajectory (float): Total time to complete the entire trajectory (seconds).
        num_points (int): Number of data points in the trajectory.
        output_filename (str): Name of the .h file to save the data.
        plot_results (bool): If True, plots the trajectory and delay data.
    """
    times = np.linspace(0, tf_total_trajectory, num_points)

    # Lists to store delay_run_spd for each motor at each time point
    all_delay_run_spd_1 = []
    all_delay_run_spd_2 = []
    all_delay_run_spd_3 = []

    # Lists to store angles and coordinates (for debugging or verification)
    all_angles_1 = []
    all_angles_2 = []
    all_angles_3 = []
    all_coords_x = []
    all_coords_y = []
    all_coords_z = []

    print(f"Generating {num_points} trajectory points over {tf_total_trajectory} seconds...")

    # Calculate time segment for each step
    tf_segment = tf_total_trajectory / (num_points - 1) if num_points > 1 else tf_total_trajectory

    for t in times:
        # Calculate XYZ coordinates at current time t
        current_x = calculate_q(t, q0_x, qf_x - q0_x, tf_total_trajectory)
        current_y = calculate_q(t, q0_y, qf_y - q0_y, tf_total_trajectory)
        current_z = calculate_q(t, q0_z, qf_z - q0_z, tf_total_trajectory)

        all_coords_x.append(current_x)
        all_coords_y.append(current_y)
        all_coords_z.append(current_z)

        # Calculate joint angles using Inverse Kinematics
        angles = inverse_kinematic(current_x, current_y, current_z)

        if angles is None:
            print(
                f"Warning: IK solution not found for point t={t:.4f}s at ({current_x:.2f}, {current_y:.2f}, {current_z:.2f}). Skipping this point.")
            # Handle no solution: This will result in 0 delay_run_spd, meaning a pause.
            # You might want a more sophisticated error handling like stopping the trajectory.
            all_delay_run_spd_1.append(0)
            all_delay_run_spd_2.append(0)
            all_delay_run_spd_3.append(0)
            all_angles_1.append(0)
            all_angles_2.append(0)
            all_angles_3.append(0)
            continue  # Skip this point

        angle_deg_1, angle_deg_2, angle_deg_3 = angles

        # Update motor parameters
        delay_1, delay_2, delay_3 = update_motor_parameters(
            tf_segment,  # Time for each small segment between points
            angle_deg_1, angle_deg_2, angle_deg_3
        )

        all_delay_run_spd_1.append(delay_1)
        all_delay_run_spd_2.append(delay_2)
        all_delay_run_spd_3.append(delay_3)

        all_angles_1.append(angle_deg_1)
        all_angles_2.append(angle_deg_2)
        all_angles_3.append(angle_deg_3)

    # --- Plotting Results (New Section) ---
    if plot_results:
        fig, axs = plt.subplots(3, 2, figsize=(15, 12))  # 3 rows, 2 columns for plots
        fig.suptitle('Robot Trajectory Analysis (Generated by Python Script)')

        # Plot 1: End-effector XYZ coordinates over time
        axs[0, 0].plot(times[:len(all_coords_x)], all_coords_x, label='X-coordinate (mm)', color='r')
        axs[0, 0].plot(times[:len(all_coords_y)], all_coords_y, label='Y-coordinate (mm)', color='g')
        axs[0, 0].plot(times[:len(all_coords_z)], all_coords_z, label='Z-coordinate (mm)', color='b')
        axs[0, 0].set_title('End-effector Position')
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Position (mm)')
        axs[0, 0].legend()
        axs[0, 0].grid(True)

        # Plot 2: Joint Angles over Time
        axs[0, 1].plot(times[:len(all_angles_1)], all_angles_1, label='Joint 1 Angle (deg)', color='r')
        axs[0, 1].plot(times[:len(all_angles_2)], all_angles_2, label='Joint 2 Angle (deg)', color='g')
        axs[0, 1].plot(times[:len(all_angles_3)], all_angles_3, label='Joint 3 Angle (deg)', color='b')
        axs[0, 1].set_title('Joint Angles')
        axs[0, 1].set_xlabel('Time (s)')
        axs[0, 1].set_ylabel('Angle (degrees)')
        axs[0, 1].legend()
        axs[0, 1].grid(True)

        # Plot 3: Delay_run_spd for Joint 1
        axs[1, 0].plot(times[:len(all_delay_run_spd_1)], all_delay_run_spd_1, label='Joint 1 Delay (us)', color='r')
        axs[1, 0].set_title('Joint 1 Delay_run_spd')
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Delay (us)')
        axs[1, 0].legend()
        axs[1, 0].grid(True)

        # Plot 4: Delay_run_spd for Joint 2
        axs[1, 1].plot(times[:len(all_delay_run_spd_2)], all_delay_run_spd_2, label='Joint 2 Delay (us)', color='g')
        axs[1, 1].set_title('Joint 2 Delay_run_spd')
        axs[1, 1].set_xlabel('Time (s)')
        axs[1, 1].set_ylabel('Delay (us)')
        axs[1, 1].legend()
        axs[1, 1].grid(True)

        # Plot 5: Delay_run_spd for Joint 3
        axs[2, 0].plot(times[:len(all_delay_run_spd_3)], all_delay_run_spd_3, label='Joint 3 Delay (us)', color='b')
        axs[2, 0].set_title('Joint 3 Delay_run_spd')
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylabel('Delay (us)')
        axs[2, 0].legend()
        axs[2, 0].grid(True)

        # Plot 6: Angles vs Delay_run_spd (Example for Joint 1)
        # Note: If angle_deg_1_data contains many zeros due to IK failure, this plot might look odd.
        axs[2, 1].scatter(all_angles_1, all_delay_run_spd_1, label='Joint 1 Angle vs Delay', color='purple', s=5)
        axs[2, 1].set_title('Joint 1 Angle vs Delay_run_spd')
        axs[2, 1].set_xlabel('Angle (degrees)')
        axs[2, 1].set_ylabel('Delay (us)')
        axs[2, 1].legend()
        axs[2, 1].grid(True)

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])  # Adjust layout to prevent title overlap
        plt.show()


# --- Example Usage ---
if __name__ == "__main__":
    # Sample trajectory parameters (initial and final end-effector positions in XYZ space)
    # Ensure these values are within your robot's reachable workspace
    # For Delta robots, Z is typically height, and can be negative or positive depending on your setup.
    start_x, start_y, start_z = 0.0, 0.0, -307.38  # Starting coordinates (mm)
    end_x, end_y, end_z = 0.0, 0.0, -380.0  # Ending coordinates (mm)

    total_time = 3.0  # Total time for trajectory movement (seconds)
    num_points_in_trajectory = 200  # Number of data points in the trajectory

    generate_trajectory_data(
        q0_x=start_x, qf_x=end_x,
        q0_y=start_y, qf_y=end_y,
        q0_z=start_z, qf_z=end_z,
        tf_total_trajectory=total_time,
        num_points=num_points_in_trajectory,
        plot_results=True  # Set to False if you don't want to see the plots
    )