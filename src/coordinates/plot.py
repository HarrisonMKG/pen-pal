import matplotlib.pyplot as plt
import pandas as pd

# Load CSV data into a pandas DataFrame
data = pd.read_csv('Pen_Pal.csv', header=None)

# Assuming CSV has columns 'x' and 'y'
x = data[1]
y = data[2]
plt.figure(figsize=(8, 6))  # Adjust width and height as needed

plt.xlabel('X')
plt.ylabel('Y')
plt.xticks(rotation=90)  # Rotate x-axis labels by 90 degrees
# Plot the data
plt.grid(True)
batch_size = 35;
for i in range(0, len(x), batch_size):
    plt.plot(y[i:i+batch_size], x[i:i+batch_size], marker='o', linestyle=':', markersize=0.5, linewidth=4, color='blue')  # Plot batch of points
    plt.pause(0.00000000000000000000000000000000000000000000000000005)  # Adjust the delay as needed
    plt.draw()  # Update the plot


plt.show()
