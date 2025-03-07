import os
import csv
import numpy as np
from datetime import datetime
from yaw_transform import normalize_angle, calibration_run
from calibration_logger import CalibrationLogger

def load_transform_history():
    """
    Load previous transforms from history file.
    
    Returns:
        List of historical transform angles in degrees, empty list if no history
    """
    history_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                               'data', 'transform_history.csv')
    
    if not os.path.exists(history_path):
        print(f"No transform history found at {history_path}")
        return []
    
    try:
        with open(history_path, 'r') as f:
            reader = csv.DictReader(f)
            transforms = [float(row['transform_angle_deg']) for row in reader]
            print(f"Loaded {len(transforms)} previous transforms from history")
            return transforms
    except Exception as e:
        print(f"Error loading transform history: {e}")
        return []

def save_transform(transform):
    """
    Append transform to history file.
    
    Args:
        transform: Transform angle in degrees to save
    """
    data_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'data')
    os.makedirs(data_dir, exist_ok=True)
    history_path = os.path.join(data_dir, 'transform_history.csv')
    
    file_exists = os.path.exists(history_path)
    
    try:
        with open(history_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['transform_angle_deg', 'timestamp'])
            if not file_exists:
                writer.writeheader()
            
            timestamp = datetime.now().isoformat()
            writer.writerow({
                'transform_angle_deg': transform,
                'timestamp': timestamp
            })
        print(f"Saved transform {transform:.2f} degrees to history")
    except Exception as e:
        print(f"Error saving transform to history: {e}")

def get_refined_transform():
    """
    Get the refined transform as the average of all previous transforms.
    
    Returns:
        Refined transform angle in degrees, 0.0 if no history
    """
    history = load_transform_history()
    
    if not history:
        print("No transform history available, using 0.0 as default")
        return 0.0
    
    refined = np.mean(history)
    print(f"Refined transform from {len(history)} runs: {refined:.2f} degrees")
    return refined

def run_calibration_with_refinement(commanded_yaw=0.0):
    """
    Run a calibration with the refined transform applied.
    
    Args:
        commanded_yaw: The commanded yaw angle in degrees
        
    Returns:
        Tuple of (this_run_transform, refined_transform)
    """
    # Get current refined transform from history
    refined_transform = get_refined_transform()
    
    # Adjust commanded yaw using the refined transform
    adjusted_command = normalize_angle(commanded_yaw - refined_transform)
    print(f"Adjusting commanded yaw from {commanded_yaw:.2f} to {adjusted_command:.2f} degrees using refined transform")
    
    # Run calibration with adjusted command
    transform = calibration_run(commanded_yaw=adjusted_command)
    
    # Save new transform to history
    save_transform(transform)
    
    # Get updated refined transform
    updated_refined = get_refined_transform()
    
    return (transform, updated_refined)

# Example usage (for testing)
if __name__ == "__main__":
    this_run, refined = run_calibration_with_refinement()
    print(f"This run transform: {this_run:.2f} degrees")
    print(f"Updated refined transform: {refined:.2f} degrees")