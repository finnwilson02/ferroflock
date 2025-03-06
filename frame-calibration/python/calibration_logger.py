import csv
import time
from datetime import datetime
import os

class CalibrationLogger:
    def __init__(self):
        """
        Initialize CSV logger for calibration data with timestamped filename.
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"calibration_log_{timestamp}.csv"
        
        # Ensure we write to the data directory
        data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')
        os.makedirs(data_dir, exist_ok=True)
        self.filepath = os.path.join(data_dir, self.filename)
        
        try:
            self.file = open(self.filepath, 'w', newline='')
            self.writer = csv.DictWriter(self.file, fieldnames=[
                'timestamp', 'commanded_yaw_deg', 'optitrack_yaw_deg', 'transform_angle_deg'
            ])
            self.writer.writeheader()
            print(f"Logging calibration data to {self.filepath}")
        except Exception as e:
            print(f"Error creating calibration log file: {e}")
            raise

    def log(self, commanded_yaw, optitrack_yaw, transform_angle):
        """
        Log a single row of calibration data.
        
        Args:
            commanded_yaw: Commanded yaw angle in degrees
            optitrack_yaw: OptiTrack measured yaw angle in degrees
            transform_angle: Computed transform angle in degrees
        """
        try:
            row = {
                'timestamp': time.time(),
                'commanded_yaw_deg': commanded_yaw,
                'optitrack_yaw_deg': optitrack_yaw,
                'transform_angle_deg': transform_angle
            }
            self.writer.writerow(row)
            self.file.flush()  # Ensure data is written immediately
        except Exception as e:
            print(f"Error logging calibration data: {e}")

    def close(self):
        """
        Close the log file properly.
        """
        try:
            if hasattr(self, 'file') and self.file:
                self.file.close()
                print(f"Closed calibration log file: {self.filepath}")
        except Exception as e:
            print(f"Error closing calibration log file: {e}")

# Example usage (for testing)
if __name__ == "__main__":
    # Simple test of logger functionality
    logger = CalibrationLogger()
    try:
        # Simulated loop
        for i in range(10):
            commanded_yaw = 0.0  # Example commanded yaw
            optitrack_yaw = 5.0 + (i * 0.1)  # Example OptiTrack yaw with slight drift
            transform_angle = 5.0  # Example transform angle
            logger.log(commanded_yaw, optitrack_yaw, transform_angle)
            time.sleep(0.1)  # Short delay
        print("Successfully logged 10 sample entries")
    finally:
        logger.close()