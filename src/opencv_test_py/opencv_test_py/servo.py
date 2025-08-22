import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import matplotlib.pyplot as plt

class VisualServoingNode(Node):
    def __init__(self):
        super().__init__('visual_servoing_node')

        # Define the target final positions for the features to track
        self.final_positions = [(428, 372), (428, 286), (512, 372), (512, 286)]
        # Lambda for the control law
        self.lamda = 0.01

        # Create a subscription to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10
        )
        # Create a publisher for the velocity commands
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)

        # Initialize CvBridge for image conversion
        self.br = CvBridge()
        # Initialize a list to store trajectory data for each feature
        self.trajectory_data = [[] for _ in range(len(self.final_positions))]  

        # Setup for Matplotlib plotting
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 800)
        self.ax.set_ylim(0, 800)
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title('Feature Trajectories')
        # Create plot lines for each feature
        self.lines = [self.ax.plot([], [], marker='o')[0] for _ in range(len(self.final_positions))]
        plt.ion()  # Enable interactive mode
        plt.show()

    def image_jacobian(self):
        # Define the image Jacobian for the feature tracking
        return np.array([[-1, 0], [0, -1], [-1, 0], [0, -1], 
                         [-1, 0], [0, -1], [-1, 0], [0, -1]])

    def inverse_robot_jacobian(self):
        # Define the inverse robot Jacobian for motion control
        return np.array([[ 0,  0, 0, 0, 0, 0, 0, 0],
                         [-2, -1, 0, 0, 0, 0, 0, 0],
                         [ 0,  0, 0, 0, 0, 0, 0, 0],
                         [ 0,  0, -2, -1, 0, 0, 0, 0],
                         [ 0,  0, 0, 0, 0, 0, 0, 0],
                         [0,  0, 0, 0, -2, -1, 0, 0],
                         [ 0,  0, 0, 0, 0, 0, 0, 0],
                         [ 0,  0, 0, 0, 0, 0, -2, -1]])

    def image_callback(self, data):
        # Convert the ROS image message to an OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data)
        # Convert to grayscale for processing
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        # Use Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        # Detect circles in the image
        circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1, minDist=20,
                                    param1=50, param2=30, minRadius=0, maxRadius=0)

        if circles is not None:
            # Convert circle data to integers
            circles = np.uint16(np.around(circles))
            current_positions = [(c[0], c[1]) for c in circles[0, :]]
            errors = []
            for i, (x, y) in enumerate(current_positions):
                # Only process if we have a corresponding final position
                if i < len(self.final_positions):
                    final_x, final_y = self.final_positions[i]
                    # Calculate errors between current and final positions
                    error_x = final_x - x
                    error_y = final_y - y
                    errors.append(error_x)
                    errors.append(error_y)

                    # Store the current position in trajectory data
                    self.trajectory_data[i].append((x, y))

            if len(errors) == 8:  # Ensure we have all errors for 4 features
                errors_array = np.array(errors).reshape((8, 1))
                # Compute the control signal using the pseudoinverse of the Jacobians
                control = np.dot(np.linalg.pinv(self.image_jacobian()), 
                                 np.linalg.pinv(self.inverse_robot_jacobian()))
                control = self.lamda * np.dot(control, errors_array)

                # Prepare and publish velocity commands
                velocity_commands = Float64MultiArray()
                velocity_commands.data = control.flatten()[:2].tolist()
                self.velocity_pub.publish(velocity_commands)

                # Draw detected circles and their centers on the frame
                for (x, y) in current_positions:
                    cv2.circle(current_frame, (x, y), 5, (0, 0, 255), -1)
                    cv2.putText(current_frame, f"Center: ({x}, {y})", (x, y), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Show the current frame with detected circles
        cv2.imshow("Detected Circles", current_frame)
        cv2.waitKey(1)  # Wait for a short time to update the display

        # Update the plot with the trajectory data
        self.update_plot()

    def update_plot(self):
        # Update the plot with the collected trajectory data
        for i, line in enumerate(self.lines):
            if self.trajectory_data[i]:
                x_data, y_data = zip(*self.trajectory_data[i])
                line.set_data(x_data, y_data)
        self.fig.canvas.draw()  # Redraw the figure
        self.fig.canvas.flush_events()  # Ensure the events are processed

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    # Create an instance of the visual servoing node
    visual_servoing_node = VisualServoingNode()
    # Spin the node to keep it active
    rclpy.spin(visual_servoing_node)
    # Clean up when done
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
