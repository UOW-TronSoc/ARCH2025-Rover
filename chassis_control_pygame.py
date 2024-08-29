import pygame
import sys
import math

# Initialize Pygame and the joystick
pygame.init()
pygame.joystick.init()

# Set up the screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Chassis Control with Xbox Controller")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)

# Initialize joystick
joystick = None
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

# Vehicle properties
chassis_length = 200
chassis_width = 100
wheel_length = 40
wheel_width = 10
wheel_offset = 50
max_speed = 5
acceleration = 0.1
friction = 10

# Dead zone threshold
dead_zone = 0.1

# Initial vehicle state
position = pygame.math.Vector2(WIDTH // 2, HEIGHT // 2)
velocity = pygame.math.Vector2(0, 0)
wheel_angle = 0  # Wheel steering angle
heading = 0  # Direction vehicle is facing

# Function to draw chassis and wheels
def draw_chassis(position, heading, wheel_angle):
    screen.fill(WHITE)
    
    # Rotate and translate chassis
    chassis_center = position
    chassis_corners = [
        pygame.math.Vector2(-chassis_length // 2, -chassis_width // 2),
        pygame.math.Vector2(chassis_length // 2, -chassis_width // 2),
        pygame.math.Vector2(chassis_length // 2, chassis_width // 2),
        pygame.math.Vector2(-chassis_length // 2, chassis_width // 2)
    ]
    rotated_corners = [chassis_center + corner.rotate(-heading) for corner in chassis_corners]
    pygame.draw.polygon(screen, BLACK, rotated_corners, 2)

    # Draw wheels
    wheel_positions = [
        pygame.math.Vector2(-chassis_length // 2 + wheel_offset, -chassis_width // 2),
        pygame.math.Vector2(chassis_length // 2 - wheel_offset, -chassis_width // 2),
        pygame.math.Vector2(chassis_length // 2 - wheel_offset, chassis_width // 2),
        pygame.math.Vector2(-chassis_length // 2 + wheel_offset, chassis_width // 2)
    ]
    
    for i, pos in enumerate(wheel_positions):
        wheel_center = chassis_center + pos.rotate(-heading)
        wheel_corners = [
            pygame.math.Vector2(-wheel_length // 2, -wheel_width // 2),
            pygame.math.Vector2(wheel_length // 2, -wheel_width // 2),
            pygame.math.Vector2(wheel_length // 2, wheel_width // 2),
            pygame.math.Vector2(-wheel_length // 2, wheel_width // 2)
        ]
        # Front wheels should be rotated based on wheel_angle
        angle = wheel_angle if i < 2 else 0
        rotated_wheel = [wheel_center + corner.rotate(-heading - angle) for corner in wheel_corners]
        pygame.draw.polygon(screen, RED, rotated_wheel)
    
    pygame.display.flip()

# Main loop
def main():
    global position, velocity, wheel_angle, heading
    
    clock = pygame.time.Clock()
    throttle = 0  # Initialize throttle here

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.JOYAXISMOTION:
                if joystick:
                    # Read the left stick horizontal axis (typically axis 0) for steering
                    wheel_angle = joystick.get_axis(0) * 45  # Scales joystick input to a -45 to +45 degree angle
                    # Read the right stick vertical axis (typically axis 3) for forward/backward movement
                    throttle = -joystick.get_axis(3)  # Negative because pushing forward is usually negative
                    # Apply dead zone to throttle to prevent micro-movements
                    if abs(throttle) < dead_zone:
                        throttle = 0

        # Update velocity based on throttle
        if throttle != 0:
            speed_change = throttle * acceleration
            velocity += pygame.math.Vector2(speed_change, 0).rotate(-heading)
        else:
            # Apply friction to slow down the vehicle
            if velocity.length() > friction:
                velocity.scale_to_length(velocity.length() / friction)
            else:
                velocity = pygame.math.Vector2(0, 0)

        # Cap the velocity to maximum speed
        if velocity.length() > max_speed:
            velocity.scale_to_length(max_speed)
        
        # Update heading and position
        if wheel_angle != 0 and velocity.length() > 0:
            # Simulate turning by changing heading
            turn_radius = chassis_length / math.sin(math.radians(wheel_angle))
            turn_speed = velocity.length() / turn_radius
            heading += math.degrees(turn_speed)

        position += velocity

        # Draw the chassis with updated position and angle
        draw_chassis(position, heading, wheel_angle)
        
        clock.tick(60)  # Limit to 60 frames per second

if __name__ == "__main__":
    main()
