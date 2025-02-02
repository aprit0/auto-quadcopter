import numpy as np
import matplotlib.pyplot as plt

def collect_calibration_data():
    """
    Collect real-world velocity and corresponding optical flow data.
    
    This function should be customized to your setup. For the sake of simplicity, 
    let's assume we manually collect the data points in the form of:
    - real_velocities (real-world velocities of an object in meters/second)
    - optical_flow_dx (optical flow displacement in the x direction in pixels)
    - heights (height of the object in meters from the sensor)
    
    Returns:
        real_velocities (list of real-world velocities)
        optical_flow_dx (list of optical flow dx values)
        heights (list of object heights)
    """
    # Example collected data (real velocity, optical flow in pixels, height in meters)
    real_velocities = np.array([0.5, 1.0, 1.5, 2.0, 2.5])  # Real velocity in m/s
    optical_flow_dx = np.array([50, 100, 150, 200, 250])  # Optical flow dx in pixels
    heights = np.array([10.0, 10.0, 10.0, 10.0, 10.0])  # Height of object in meters
    
    return real_velocities, optical_flow_dx, heights

def perform_calibration(real_velocities, optical_flow_dx, heights):
    """
    Perform a linear regression to determine the calibration coefficient.
    
    We assume that the relationship is of the form:
        real_velocity = calibration_coefficient * dx * height
    
    We want to find the coefficient that satisfies the equation for all data points.
    
    Parameters:
    - real_velocities: Real-world velocities (in m/s)
    - optical_flow_dx: Optical flow displacements in the x direction (in pixels)
    - heights: Heights of the object (in meters)
    
    Returns:
    - calibration_coefficient: The calculated calibration coefficient
    """
    # Compute dx * height (this is the variable we want to calibrate)
    dx_height = optical_flow_dx * heights
    
    # Perform linear regression to find the best fitting line:
    # real_velocity = calibration_coefficient * dx * height
    calibration_coefficient = np.linalg.lstsq(dx_height[:, np.newaxis], real_velocities, rcond=None)[0][0]
    
    return calibration_coefficient

def plot_calibration(real_velocities, optical_flow_dx, heights, calibration_coefficient):
    """
    Plot the calibration data and the fitted line for visualization.
    """
    dx_height = optical_flow_dx * heights
    predicted_velocities = calibration_coefficient * dx_height
    
    plt.scatter(dx_height, real_velocities, color='blue', label='Data points')
    plt.plot(dx_height, predicted_velocities, color='red', label='Fitted line')
    plt.xlabel('dx * height (pixels * meters)')
    plt.ylabel('Real velocity (m/s)')
    plt.title('Calibration: Optical Flow vs Real Velocity')
    plt.legend()
    plt.grid(True)
    # plt.show()
    plt.savefig("abc.png")

def main():
    # Collect calibration data
    real_velocities, optical_flow_dx, heights = collect_calibration_data()
    
    # Perform calibration to get the calibration coefficient
    calibration_coefficient = perform_calibration(real_velocities, optical_flow_dx, heights)
    
    # Print the calculated calibration coefficient
    print(f"Calibration coefficient: {calibration_coefficient:.4f} m/(pixel * meter)")
    
    # Plot the calibration result
    plot_calibration(real_velocities, optical_flow_dx, heights, calibration_coefficient)

if __name__ == "__main__":
    main()
