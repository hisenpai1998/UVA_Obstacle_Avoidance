from pynput import keyboard

class KeyboardController:
    def __init__(self):
        """Initialize the keyboard controller."""
        self.keys_pressed = {'w': False, 
                             's': False, 
                             'a': False, 
                             'd': False, 
                             'q': False, 
                             'e': False, 
                             'space': False}
        self.listener = None

    def start(self):
        """Start the keyboard listener."""
        self.listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self.listener.start()

    def stop(self):
        """Stop the keyboard listener."""
        if self.listener is not None:
            self.listener.stop()

    def _on_press(self, key):
        """Handle key press events."""
        try:
            if key.char in self.keys_pressed:
                self.keys_pressed[key.char] = True
        except AttributeError:
            if key == keyboard.Key.space:
                self.keys_pressed['space']  = True
            elif key == keyboard.Key.esc:
                return False

    def _on_release(self, key):
        """Handle key release events."""
        try:
            if key.char in self.keys_pressed:
                self.keys_pressed[key.char] = False
        except AttributeError:
            if key == keyboard.Key.space:
                self.keys_pressed['space']  = False

    def update_position(self, current_pos, step_size):
        """Update position based on keyboard input, with reset option."""
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
        if self.keys_pressed['space']:  # Reset to initial position
            x, y, z = 0, 0, 0.5  # Example reset position
        return [x, y, z]