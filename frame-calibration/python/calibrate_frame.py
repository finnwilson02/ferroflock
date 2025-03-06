#!/usr/bin/env python3
"""
Frame Calibration Script for Drone Yaw Transform

This script performs frame calibration to determine the yaw angle transform 
between the drone's command frame and OptiTrack's frame.
"""

import argparse
import time
import numpy as np
from transform_refiner import run_calibration_with_refinement, get_refined_transform
from yaw_transform import normalize_angle

def main():
    parser = argparse.ArgumentParser(description='Drone Frame Calibration Tool')
    parser.add_argument('--runs', type=int, default=1, 
                        help='Number of calibration runs to perform (default: 1)')
    parser.add_argument('--command-yaw', type=float, default=0.0,
                        help='Commanded "forward" yaw in degrees (default: 0.0)')
    parser.add_argument('--samples', type=int, default=20,
                        help='Number of samples per run (default: 20)')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='Sampling rate in Hz (default: 10.0)')
    parser.add_argument('--view-only', action='store_true',
                        help='Only view the current refined transform without running calibration')
    
    args = parser.parse_args()
    
    if args.view_only:
        refined = get_refined_transform()
        print(f"Current refined transform: {refined:.2f} degrees")
        return
    
    print(f"=== Drone Frame Calibration ===")
    print(f"Starting {args.runs} calibration run(s)")
    print(f"Commanded 'forward' yaw: {args.command_yaw} degrees")
    print(f"Samples per run: {args.samples}")
    print(f"Sampling rate: {args.rate} Hz")
    print("=" * 30)
    
    for run in range(1, args.runs + 1):
        print(f"\nCalibration Run {run}/{args.runs}")
        print("-" * 20)
        
        run_transform, refined_transform = run_calibration_with_refinement(args.command_yaw)
        
        print(f"Run {run} transform: {run_transform:.2f} degrees")
        print(f"Current refined transform: {refined_transform:.2f} degrees")
        
        if run < args.runs:
            print(f"Waiting 3 seconds before next run...")
            time.sleep(3)
    
    print("\n=== Calibration Complete ===")
    print(f"Final refined transform: {refined_transform:.2f} degrees")
    print(f"Apply this transform to your commanded yaw values:")
    print(f"adjusted_yaw = commanded_yaw - {refined_transform:.2f}")
    print("=" * 30)

if __name__ == "__main__":
    main()