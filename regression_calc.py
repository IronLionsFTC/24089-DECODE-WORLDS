import math

def calculate_launch_velocity(distance, angle_degrees):
    # Constants
    g = 9.8  # Gravitational acceleration (m/s^2)
    h = 0.35  # Height of the shooter (meters)

    # Convert angle to radians
    angle_radians = math.radians(angle_degrees)

    # Calculate the launch velocity using the projectile motion formula
    try:
        v0 = math.sqrt((g * distance**2) / (2 * (distance * math.tan(angle_radians) - h) * (math.cos(angle_radians))**2))
        return v0
    except ValueError:
        return "Error: Invalid input - distance or angle may be too extreme."

# Main function to input variables
if __name__ == "__main__":
    print("Enter the horizontal distance (in meters):")
    distance = float(input())  # Get distance from user input

    print("Enter the launch angle (in degrees):")
    angle = float(input())  # Get launch angle from user input

    # Calculate and display the launch velocity
    launch_velocity = calculate_launch_velocity(distance, angle)

    if isinstance(launch_velocity, float):
        print(f"The calculated launch velocity is: {launch_velocity:.2f} m/s")
    else:
        print(launch_velocity)
