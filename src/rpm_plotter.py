#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time
import numpy as np

class RPMPlotter(Node):
    def __init__(self):
        super().__init__('rpm_plotter')
        
        # Data storage with time-based indexing
        self.max_data_points = 1000  # Keep last 1000 data points
        self.timestamps = deque(maxlen=self.max_data_points)
        self.input_rpm_l = deque(maxlen=self.max_data_points)
        self.input_rpm_r = deque(maxlen=self.max_data_points)
        self.output_rpm_l = deque(maxlen=self.max_data_points)
        self.output_rpm_r = deque(maxlen=self.max_data_points)
        
        self.start_time = time.time()
        
        # Subscribe to RPM data
        self.rpm_sub = self.create_subscription(
            Float32MultiArray, 
            '/rpm_data', 
            self.rpm_callback, 
            10
        )
        
        # Setup matplotlib
        self.setup_plots()
        
        # Start plotting in a separate thread
        self.plot_thread = threading.Thread(target=self.start_plotting, daemon=True)
        self.plot_thread.start()
        
        self.get_logger().info("RPM Plotter initialized. Listening for ESP32 RPM data...")

    def rpm_callback(self, msg):
        """Process incoming RPM data"""
        try:
            if len(msg.data) >= 4:
                current_time = time.time() - self.start_time
                
                self.timestamps.append(current_time)
                self.input_rpm_l.append(msg.data[0])
                self.input_rpm_r.append(msg.data[1])
                self.output_rpm_l.append(msg.data[2])
                self.output_rpm_r.append(msg.data[3])
                
        except Exception as e:
            self.get_logger().error(f"Error processing RPM data: {e}")

    def setup_plots(self):
        """Setup matplotlib figures and axes"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(12, 8))
        
        # Input RPM plot
        self.ax1.set_title('Input RPM vs Time', fontsize=14, fontweight='bold')
        self.ax1.set_ylabel('RPM', fontsize=12)
        self.ax1.grid(True, alpha=0.3)
        self.input_l_line, = self.ax1.plot([], [], 'b-', label='Left Motor', linewidth=2)
        self.input_r_line, = self.ax1.plot([], [], 'r-', label='Right Motor', linewidth=2)
        self.ax1.legend(loc='upper right')
        
        # Output RPM plot
        self.ax2.set_title('Output RPM vs Time', fontsize=14, fontweight='bold')
        self.ax2.set_xlabel('Time (seconds)', fontsize=12)
        self.ax2.set_ylabel('RPM', fontsize=12)
        self.ax2.grid(True, alpha=0.3)
        self.output_l_line, = self.ax2.plot([], [], 'g-', label='Left Motor', linewidth=2)
        self.output_r_line, = self.ax2.plot([], [], 'm-', label='Right Motor', linewidth=2)
        self.ax2.legend(loc='upper right')
        
        plt.tight_layout()
        plt.subplots_adjust(hspace=0.3)

    def animate(self, frame):
        """Animation function for updating plots"""
        if len(self.timestamps) > 0:
            times = list(self.timestamps)
            input_l = list(self.input_rpm_l)
            input_r = list(self.input_rpm_r)
            output_l = list(self.output_rpm_l)
            output_r = list(self.output_rpm_r)
            
            # Update input RPM plot
            self.input_l_line.set_data(times, input_l)
            self.input_r_line.set_data(times, input_r)
            
            # Update output RPM plot
            self.output_l_line.set_data(times, output_l)
            self.output_r_line.set_data(times, output_r)
            
            # Update axis limits
            if times:
                x_min, x_max = min(times), max(times)
                x_margin = max((x_max - x_min) * 0.05, 1) if x_max > x_min else 1
                
                self.ax1.set_xlim(x_min - x_margin, x_max + x_margin)
                self.ax2.set_xlim(x_min - x_margin, x_max + x_margin)
                
                # Set Y limits with some padding
                all_input_rpms = input_l + input_r
                all_output_rpms = output_l + output_r
                
                if all_input_rpms:
                    y1_min, y1_max = min(all_input_rpms), max(all_input_rpms)
                    y1_margin = max((y1_max - y1_min) * 0.1, 5) if y1_max > y1_min else 10
                    self.ax1.set_ylim(y1_min - y1_margin, y1_max + y1_margin)
                
                if all_output_rpms:
                    y2_min, y2_max = min(all_output_rpms), max(all_output_rpms)
                    y2_margin = max((y2_max - y2_min) * 0.1, 5) if y2_max > y2_min else 10
                    self.ax2.set_ylim(y2_min - y2_margin, y2_max + y2_margin)
        
        return [self.input_l_line, self.input_r_line, self.output_l_line, self.output_r_line]

    def start_plotting(self):
        """Start the matplotlib animation"""
        try:
            ani = animation.FuncAnimation(
                self.fig, 
                self.animate, 
                interval=100, 
                blit=False,
                cache_frame_data=False
            )
            plt.show()
        except Exception as e:
            self.get_logger().error(f"Error in plotting: {e}")

    def save_data_to_csv(self, filename="rpm_data.csv"):
        """Save collected data to CSV file"""
        if len(self.timestamps) > 0:
            import csv
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['Time', 'Input_RPM_Left', 'Input_RPM_Right', 'Output_RPM_Left', 'Output_RPM_Right'])
                for i in range(len(self.timestamps)):
                    writer.writerow([
                        self.timestamps[i],
                        self.input_rpm_l[i],
                        self.input_rpm_r[i],
                        self.output_rpm_l[i],
                        self.output_rpm_r[i]
                    ])
            self.get_logger().info(f"Data saved to {filename}")


def main():
    rclpy.init()
    
    plotter = RPMPlotter()
    
    try:
        rclpy.spin(plotter)
    except KeyboardInterrupt:
        plotter.get_logger().info("Shutting down RPM plotter...")
        plotter.save_data_to_csv()
    finally:
        plotter.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
