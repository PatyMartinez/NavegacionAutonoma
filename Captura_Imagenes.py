"""camera_pid controller captura automática."""

from controller import Display, Keyboard, Robot, Camera
from vehicle import Car, Driver
import numpy as np
import cv2
from datetime import datetime
import os
import csv

# Getting image from camera
def get_image(camera):
    raw_image = camera.getImage()
    image = np.frombuffer(raw_image, np.uint8).reshape(
        (camera.getHeight(), camera.getWidth(), 4)
    )
    return image

# Image processing
def greyscale_cv2(image):
    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return gray_img

# Display image
def display_image(display, image):
    image_rgb = np.dstack((image, image, image,))
    image_ref = display.imageNew(
        image_rgb.tobytes(),
        Display.RGB,
        width=image_rgb.shape[1],
        height=image_rgb.shape[0],
    )
    display.imagePaste(image_ref, 0, 0, False)

# Initial angle and speed
manual_steering = 0
steering_angle = 0
angle = 0.0
speed = 30

# Set target speed
def set_speed(kmh):
    global speed
    speed = kmh

# Update steering angle
def set_steering_angle(wheel_angle):
    global angle, steering_angle
    # Check limits of steering
    if (wheel_angle - steering_angle) > 0.1:
        wheel_angle = steering_angle + 0.1
    if (wheel_angle - steering_angle) < -0.1:
        wheel_angle = steering_angle - 0.1
    steering_angle = wheel_angle

    # Limit range of the steering angle
    if wheel_angle > 0.5:
        wheel_angle = 0.5
    elif wheel_angle < -0.5:
        wheel_angle = -0.5
    # Update steering angle
    angle = wheel_angle

# Validate increment of steering angle
def change_steer_angle(inc):
    global manual_steering
    new_manual_steering = manual_steering + inc
    if new_manual_steering <= 25.0 and new_manual_steering >= -25.0:
        manual_steering = new_manual_steering
        set_steering_angle(manual_steering * 0.02)
    # Debugging
    if manual_steering == 0:
        print("going straight")
    else:
        turn = "left" if steering_angle < 0 else "right"
        print("turning {} rad {}".format(str(steering_angle), turn))

def return_steering_to_center():
    global manual_steering
    if manual_steering > 0:
        manual_steering -= 0.5
        if manual_steering < 0:
            manual_steering = 0
    elif manual_steering < 0:
        manual_steering += 0.5
        if manual_steering > 0:
            manual_steering = 0
    set_steering_angle(manual_steering * 0.02)

# Main
def main():
    # Create the Robot instance.
    robot = Car()
    driver = Driver()

    # Get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())

    # Create camera instance
    camera = robot.getDevice("camera")
    camera.enable(timestep)

    # Processing display
    display_img = Display("display_image")

    # Create keyboard instance
    keyboard = Keyboard()
    keyboard.enable(timestep)

    # Prepare image folder and CSV
    image_folder = os.path.join(os.getcwd(), "dataset_images")
    os.makedirs(image_folder, exist_ok=True)

    csv_path = os.path.join(os.getcwd(), "driving_log.csv")
    csv_file = open(csv_path, mode='w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["image_name", "steering_angle"])

    frame_counter = 0

    while robot.step() != -1:
        # Get image from camera
        image = get_image(camera)

        # Process and display image
        grey_image = greyscale_cv2(image)
        display_image(display_img, grey_image)

        # Read keyboard
        key = keyboard.getKey()
        # if key == keyboard.UP:
        #     set_speed(speed + 5.0)
        #     print("up")
        # elif key == keyboard.DOWN:
        #     set_speed(speed - 5.0)
        #     print("down")
        if key == keyboard.RIGHT:
            change_steer_angle(+0.5)
            print("right")
        elif key == keyboard.LEFT:
            change_steer_angle(-0.5)
            print("left")
        else:
            return_steering_to_center()

        # Update angle and speed
        driver.setSteeringAngle(angle)
        driver.setCruisingSpeed(speed)

        # Capture automatic image every 3 frames
        if frame_counter % 15 == 0:
            current_datetime = str(datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f"))
            image_name = current_datetime + ".png"
            image_path = os.path.join(image_folder, image_name)
            camera.saveImage(image_path, 1)

            # Write to CSV
            csv_writer.writerow([image_name, steering_angle])
            print(f"[Capture] {image_name} | Angle: {steering_angle:.3f}")

        frame_counter += 1

    csv_file.close()

if __name__ == "__main__":
    main()