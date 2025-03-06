from djitellopy import Tello
import cv2
import time
import numpy as np
import sys
import tty
import termios
import select

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def kbhit():
    dr,dw,de = select.select([sys.stdin], [], [], 0)
    return dr != []

def initialize_tello():
    tello = Tello()
    tello.connect()
    tello.streamon()
    tello.set_speed(30)
    return tello

def main():
    tello = initialize_tello()
    
    # Initialize video writer
    frame = tello.get_frame_read().frame
    height, width, _ = frame.shape
    video = cv2.VideoWriter('tello_recording.avi', 
                           cv2.VideoWriter_fourcc(*'XVID'),
                           30, (width, height))
    
    print("Control scheme:")
    print("T: Takeoff  |  L: Land  |  Q: Quit")
    print("I/K/J/H: Forward/Back/Left/Right")
    print("W/S: Up/Down  |  A/D: Rotate Left/Right")
    
    # Keep track of current movement state
    lr, fb, ud, yv = 0, 0, 0, 0
    speed = 50
    is_flying = False
    
    try:
        while True:
            frame = tello.get_frame_read().frame
            video.write(frame)
            
            if kbhit():
                key = getch()
                
                if key == 't' and not is_flying:
                    print("Taking off...")
                    tello.takeoff()
                    is_flying = True
                    time.sleep(1)  # Give it time to stabilize
                
                elif key == 'l' and is_flying:
                    print("Landing...")
                    tello.land()
                    is_flying = False
                    time.sleep(1)
                
                elif key == 'q':
                    print("Quitting...")
                    if is_flying:
                        print("Landing first...")
                        tello.land()
                        is_flying = False
                        time.sleep(1)
                    break
                
                # Only process movement commands if flying
                elif is_flying:
                    # Reset all movements first
                    lr, fb, ud, yv = 0, 0, 0, 0
                    
                    # Movement controls
                    if key == 'w':
                        ud = speed
                        print("Up")
                    elif key == 's':
                        ud = -speed
                        print("Down")
                    elif key == 'a':
                        yv = -speed
                        print("Rotate Left")
                    elif key == 'd':
                        yv = speed
                        print("Rotate Right")
                    elif key == 'i':
                        fb = speed
                        print("Forward")
                    elif key == 'k':
                        fb = -speed
                        print("Backward")
                    elif key == 'j':
                        lr = -speed
                        print("Left")
                    elif key == 'h':
                        lr = speed
                        print("Right")
                    
                    tello.send_rc_control(lr, fb, ud, yv)
            
            # Add a small sleep to prevent CPU overload
            time.sleep(0.05)
            
    except Exception as e:
        print(f"Error occurred: {e}")
        if is_flying:
            try:
                tello.land()
            except:
                pass
        
    finally:
        # Clean up
        if is_flying:
            try:
                tello.land()
            except:
                pass
        video.release()
        tello.streamoff()

if __name__ == "__main__":
    main()