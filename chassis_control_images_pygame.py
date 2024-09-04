import pygame
import sys
import math

# Initialize Pygame and the joystick
pygame.init()
pygame.joystick.init()

# Screen settings
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Tank Steering with Xbox Controller")
clock = pygame.time.Clock()

# Font for displaying wheel speeds
font = pygame.font.SysFont(None, 24)

# Load images for wheels and rover
wheel_front_image = pygame.image.load("images/car-wheel-front.jpg").convert_alpha()
wheel_side_image = pygame.image.load("images/car-wheel-side.jpg").convert_alpha()
rover_image = pygame.image.load("images/rover.png").convert_alpha()  # Placeholder image for the rover

# Resize images (adjust size as needed)
wheel_front_image = pygame.transform.scale(wheel_front_image, (50, 50))
wheel_side_image = pygame.transform.scale(wheel_side_image, (50, 50))
rover_image = pygame.transform.scale(rover_image, (150, 150))  # Resize rover image

# Wheel positions for front-view and top-down view
wheel_positions_front_view = [(450, 150), (550, 150), (450, 260), (550, 260)]  # Front view
wheel_positions_top_down = [(150, 150), (250, 150), (150, 260), (250, 260)]    # Top-down view

# Initial wheel speeds and angles
wheel_speeds = [0, 0, 0, 0]
wheel_angles = [0, 0, 0, 0]  # Rotation angles for each wheel

# Speed and control parameters
ACCELERATION = 0.2
DECELERATION = 0.1
MAX_SPEED = 10
SPEED_THRESHOLD = 0.1  # Threshold to clamp speed to zero
DEAD_ZONE = 0.1  # Dead zone for joystick input

# Rover's position and orientation
rover_position = (250, 500)  # Position near the bottom of the screen
rover_angle = 0  # Initial angle of the rover

# Initialize joystick if connected
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

# Main loop
def main():
    global wheel_speeds, wheel_angles, rover_angle
    running = True
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Joystick control
        if joystick:
            # Get the values from the left joystick axes (Y-axis for forward/backward and X-axis for turning)
            left_stick_y = -joystick.get_axis(1)  # Vertical axis of left stick (forward/backward)
            left_stick_x = joystick.get_axis(0)  # Horizontal axis of left stick (left/right)

            # Apply dead zone
            if abs(left_stick_y) < DEAD_ZONE:
                left_stick_y = 0
            if abs(left_stick_x) < DEAD_ZONE:
                left_stick_x = 0

            # Calculate wheel speeds based on tank steering logic
            left_speed = (left_stick_y - left_stick_x) * MAX_SPEED
            right_speed = (left_stick_y + left_stick_x) * MAX_SPEED

            # Update wheel speeds for tank steering
            wheel_speeds[0] = wheel_speeds[2] = left_speed   # Left wheels
            wheel_speeds[1] = wheel_speeds[3] = right_speed  # Right wheels

            # Handle turning on the spot by comparing left and right speeds
            if left_speed > right_speed:
                rover_angle += (left_speed - right_speed) * 0.5
            elif right_speed > left_speed:
                rover_angle -= (right_speed - left_speed) * 0.5

        # Update wheel rotation angles based on speeds
        for i in range(4):
            wheel_angles[i] += wheel_speeds[i]
            wheel_angles[i] %= 360  # Keep angles within 0-359 degrees

        # Clear the screen
        screen.fill((255, 255, 255))

        # Draw front-view wheels with rotation
        for i, position in enumerate(wheel_positions_front_view):
            rotated_wheel = pygame.transform.rotate(wheel_front_image, wheel_angles[i])
            rect = rotated_wheel.get_rect(center=position)
            screen.blit(rotated_wheel, rect.topleft)
            speed_text = font.render(f"Speed: {wheel_speeds[i]:.1f}", True, (0, 0, 0))
            screen.blit(speed_text, (position[0] - 30, position[1] + 60))

        # Draw top-down view wheels without rotation
        for i, position in enumerate(wheel_positions_top_down):
            rect = wheel_side_image.get_rect(center=position)
            screen.blit(wheel_side_image, rect.topleft)
            speed_text = font.render(f"Speed: {wheel_speeds[i]:.1f}", True, (0, 0, 0))
            screen.blit(speed_text, (position[0] - 30, position[1] + 60))

        # Draw the rover with rotating wheels at the bottom of the screen
        rotated_rover = pygame.transform.rotate(rover_image, rover_angle)
        rover_rect = rotated_rover.get_rect(center=rover_position)
        screen.blit(rotated_rover, rover_rect.topleft)

        # Update display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(60)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
