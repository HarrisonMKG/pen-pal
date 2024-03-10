import cv2
import pandas as pd
import time
import numpy as np


# Initialize the list to store coordinates and timestamps
data = []

# Function to track the laser
def lin_interporlate(start_point, end_point, num_points):
    delta_time = (end_point[0] - start_point[0])/(num_points)
    delta_x = (end_point[1] - start_point[1])/num_points
    delta_y = (end_point[2] - start_point[2])/num_points
    faux_z = 0.02
    
    for i in range(1,num_points):
        tmp_time = start_point[0] + delta_time * i 
        tmp_x = start_point[1] + delta_x * i
        tmp_y = start_point[2] + delta_y * i
        print(f"time = {start_point} + {delta_time} = {tmp_time}")
        print(f"x = {start_point} + {delta_x} = {tmp_time}")
        data.append([tmp_time, tmp_x, tmp_y, faux_z])

    return

def moving_average(data, window_size):
    cumsum = np.cumsum(np.insert(data, 0, 0)) 
    return (cumsum[window_size:] - cumsum[:-window_size]) / window_size

def track_laser(video_path, output_csv):
    global data
    # Open the video
    cap = cv2.VideoCapture(video_path)
    faux_z = 0.02
    first_measure = True
    num_points_interp = 2
    
    # Check if video opened successfully
    if not cap.isOpened():
        print("Error opening video file")
        return

    # Read until video is completed
    count = 0
    while cap.isOpened():
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret:
            # Convert to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Define range of pink color in HSV
            lower_pink = (140, 50, 50)
            upper_pink = (170, 255, 255)
            # Threshold the HSV image to get only pink colors
            mask = cv2.inRange(hsv, lower_pink, upper_pink)
            
            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            # Find the largest contour assuming it's the laser
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    scaling_factor = 0.1  # Adjust this scaling factor as needed
                    cX = int(M["m10"] / M["m00"] * scaling_factor)
                    cY = int(M["m01"] / M["m00"] * scaling_factor)
                    # Save the timestamp and coordinates
                    curr_time = round(time.time() * 1000)
                    if first_measure: 
                        time_ref = round(time.time() * 1000)
                        time_elapsed = (curr_time -time_ref)/1000
                        first_measure = False
                    else:
                        time_elapsed = (curr_time -time_ref)/1000
                        lin_interporlate(data[-1], [time_elapsed, cX, cY, faux_z], num_points_interp)

                    data.append([time_elapsed, cX, cY, faux_z])
                    count += 1
                    
                    # Optionally, visualize the tracking
                    cv2.circle(frame, (cX, cY), 5, (255, 0, 0), -1)
                    
                else:
                    print("No laser detected.")
            
            # Display the frame
            cv2.imshow('Frame', frame)
            
            # Press Q on keyboard to exit
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break
        else:
            break

    # Release the video capture object
    cap.release()
    cv2.destroyAllWindows()
    
    # Applying moving average filter to smooth the data
    time_data = [d[0] for d in data]
    smoothed_time_data = moving_average(time_data, 3)  # Adjust window size as needed
    x_data = [d[1] for d in data]
    smoothed_x_data = moving_average(x_data, 3)  # Adjust window size as needed
    y_data = [d[2] for d in data]
    smoothed_y_data = moving_average(y_data, 3)  # Adjust window size as needed

    # Save the smoothed data to a CSV file
    smoothed_data = list(zip(smoothed_time_data, smoothed_x_data, smoothed_y_data))
    df = pd.DataFrame(data) 
    df.to_csv(output_csv, index=False, header = None)
    print(f"Smoothed data saved to {output_csv}")

# Example usage
track_laser('./laser_check.mp4', 'laser_coordinates.csv')
