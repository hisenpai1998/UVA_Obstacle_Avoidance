from pynput import keyboard
import numpy as np  # Required for rotation in radians

class KeyboardController:
    def __init__(self):
        """
        Initialize the keyboard controller.
        
        """
        self.keys_pressed = {
            'w': False,     # Forward
            's': False,     # Backward
            'a': False,     # Left
            'd': False, 	# Right
            'q': False,     # Up
            'e': False,     # Down
            'z': False,     # Rotate left
            'x': False,     # Rotate right
        }
        self.listener = None

    def start(self):
        """
        Start the keyboard listener.
        
        """
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()

    def stop(self):
        """
        Stop the keyboard listener.
        
        """
        if self.listener is not None:
            self.listener.stop()

    def _on_press(self, key):
        """
        Handle key press events.

        """
        try:
            if key.char in self.keys_pressed:
                self.keys_pressed[key.char] = True
        except AttributeError:
            if key == keyboard.Key.space:
                self.keys_pressed['space'] = True
            elif key == keyboard.Key.esc:
                return False

    def _on_release(self, key):
        """
        Handle key release events.

        """
        try:
            if key.char in self.keys_pressed:
                self.keys_pressed[key.char] = False
        except AttributeError:
            if key == keyboard.Key.space:
                self.keys_pressed['space'] = False

    def update_position(self, current_pos, step_size):
        
        """
        Update position based on keyboard input, with reset option.

        """
        x, y, z = current_pos
        if self.keys_pressed['w']:  # Forward (+y)
            y += step_size
        if self.keys_pressed['s']:  # Backward (-y)
            y -= step_size
        if self.keys_pressed['a']:  # Left (-x)
            x -= step_size
        if self.keys_pressed['d']:  # Right (+x)
            x += step_size
        if self.keys_pressed['q']:  # Up (+z)
            z += step_size
        if self.keys_pressed['e']:  # Down (-z)
            z -= step_size
        return [x, y, z]

    def update_yaw(self, current_yaw, yaw_step=np.deg2rad(10)):        
        """
        Update yaw angle (in radians) based on F/G keys.

        """
        if self.keys_pressed['z']:
            current_yaw += yaw_step  # Rotate left (CCW)
        if self.keys_pressed['x']:
            current_yaw -= yaw_step  # Rotate right (CW)
        
        return current_yaw
    
    def update_position_with_yaw(self, current_pos, yaw, step_size):
        """
        Move in the drone's local frame based on yaw.
        W/S = forward/backward, A/D = strafe left/right, Q/E = up/down.
        """
        x, y, z = current_pos
        dx, dy = 0.0, 0.0

        # Local axes: forward and right
        forward = np.array([np.cos(yaw), np.sin(yaw)])
        right   = np.array([np.cos(yaw + np.pi / 2), np.sin(yaw + np.pi / 2)])

        if self.keys_pressed['a']:
            dx += forward[0] * step_size
            dy += forward[1] * step_size
        if self.keys_pressed['d']:
            dx -= forward[0] * step_size
            dy -= forward[1] * step_size
        if self.keys_pressed['w']:
            dx -= right[0] * step_size
            dy -= right[1] * step_size
        if self.keys_pressed['s']:
            dx += right[0] * step_size
            dy += right[1] * step_size
        if self.keys_pressed['q']:
            z += step_size
        if self.keys_pressed['e']:
            z -= step_size

        return [x + dx, y + dy, z]


