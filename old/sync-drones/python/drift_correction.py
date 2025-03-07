import json
import subprocess
import curses
import time
from collections import defaultdict

class DriftCalibrator:
    def __init__(self, screen):
        self.screen = screen
        # Load drone mapping
        try:
            with open('drone_mapping.json', 'r') as f:
                self.mapping = json.load(f)
        except FileNotFoundError:
            self.mapping = self.load_devices()
        
        self.current_drone = None
        self.current_bias = {'pitch': 0, 'roll': 0}
        self.fine_adjust = 0.1
        self.coarse_adjust = 1.0
        self.is_flying = False

    def load_devices(self):
        try:
            with open('dji_devices.json', 'r') as f:
                devices = json.load(f)
                return {d['mac']: {'ip': d['ip']} for d in devices}
        except FileNotFoundError:
            self.screen.addstr("\nNo device mapping found!")
            return {}

    def load_existing_biases(self):
        try:
            with open('drift_calibration.json', 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return defaultdict(lambda: {'pitch': 0, 'roll': 0})

    def save_biases(self, biases):
        with open('drift_calibration.json', 'w') as f:
            json.dump(biases, f, indent=2)

    def send_command(self, ip, command):
        subprocess.run(['./swarm', '--single', ip, '--command', command])

    def apply_bias(self):
        if not self.is_flying:
            return
            
        cmd = f"rc {int(self.current_bias['roll'])} {int(self.current_bias['pitch'])} 0 0"
        self.send_command(self.current_drone['ip'], cmd)

    def update_display(self, message=""):
        self.screen.clear()
        self.screen.addstr(0, 0, "\nDrift Calibration Tool")
        self.screen.addstr("\n======================")
        self.screen.addstr("\nControls:")
        self.screen.addstr("\nArrow Keys: Coarse adjustment (±1.0)")
        self.screen.addstr("\nIJKL: Fine adjustment (±0.1)")
        self.screen.addstr("\nT: Take off")
        self.screen.addstr("\nL: Land")
        self.screen.addstr("\nN: Next drone")
        self.screen.addstr("\nR: Retry current drone")
        self.screen.addstr("\nS: Save and quit")
        self.screen.addstr("\nQ: Quit without saving")
        
        if self.current_drone:
            self.screen.addstr(f"\n\nCurrent bias - Pitch: {self.current_bias['pitch']:.1f}, "
                             f"Roll: {self.current_bias['roll']:.1f}")
            self.screen.addstr(f"\nDrone Status: {'Flying' if self.is_flying else 'Landed'}")
        
        if message:
            self.screen.addstr(f"\n\n{message}")
        
        self.screen.refresh()

    def calibrate_drones(self):
        biases = self.load_existing_biases()
        
        for mac, info in self.mapping.items():
            self.current_drone = info
            self.current_bias = biases[mac].copy()
            self.is_flying = False
            
            self.update_display(f"Calibrating drone: {mac}")
            
            # Flash LED
            self.send_command(info['ip'], "led 255 0 0")
            time.sleep(0.5)
            self.send_command(info['ip'], "led 0 0 0")

            while True:
                key = self.screen.getch()
                
                if key == curses.KEY_UP:
                    self.current_bias['pitch'] += self.coarse_adjust
                    self.apply_bias()
                elif key == curses.KEY_DOWN:
                    self.current_bias['pitch'] -= self.coarse_adjust
                    self.apply_bias()
                elif key == curses.KEY_LEFT:
                    self.current_bias['roll'] -= self.coarse_adjust
                    self.apply_bias()
                elif key == curses.KEY_RIGHT:
                    self.current_bias['roll'] += self.coarse_adjust
                    self.apply_bias()
                elif key in [ord('i'), ord('I')]:
                    self.current_bias['pitch'] += self.fine_adjust
                    self.apply_bias()
                elif key in [ord('k'), ord('K')]:
                    self.current_bias['pitch'] -= self.fine_adjust
                    self.apply_bias()
                elif key in [ord('j'), ord('J')]:
                    self.current_bias['roll'] -= self.fine_adjust
                    self.apply_bias()
                elif key in [ord('l'), ord('L')]:
                    self.current_bias['roll'] += self.fine_adjust
                    self.apply_bias()
                elif key in [ord('t'), ord('T')] and not self.is_flying:
                    self.update_display("Taking off...")
                    self.send_command(info['ip'], "takeoff")
                    self.is_flying = True
                elif key in [ord('l'), ord('L')] and self.is_flying:
                    self.update_display("Landing...")
                    self.send_command(info['ip'], "land")
                    self.is_flying = False
                elif key in [ord('n'), ord('N')]:
                    if self.is_flying:
                        self.send_command(info['ip'], "land")
                        self.is_flying = False
                    biases[mac] = self.current_bias.copy()
                    self.update_display(f"Saved bias for {mac}")
                    time.sleep(1)
                    break
                elif key in [ord('r'), ord('R')]:
                    if self.is_flying:
                        self.send_command(info['ip'], "land")
                        self.is_flying = False
                    continue
                elif key in [ord('s'), ord('S')]:
                    if self.is_flying:
                        self.send_command(info['ip'], "land")
                    biases[mac] = self.current_bias.copy()
                    self.save_biases(biases)
                    return
                elif key in [ord('q'), ord('Q')]:
                    if self.is_flying:
                        self.send_command(info['ip'], "land")
                    return

                self.update_display()

        self.save_biases(biases)
        self.update_display("Calibration complete!")
        time.sleep(2)

def main(stdscr):
    # Set up curses
    curses.curs_set(0)  # Hide cursor
    stdscr.nodelay(0)   # Blocking input
    stdscr.keypad(1)    # Enable keypad mode
    
    # Create and run calibrator
    calibrator = DriftCalibrator(stdscr)
    calibrator.calibrate_drones()

if __name__ == "__main__":
    curses.wrapper(main)