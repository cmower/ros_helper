
class Joystick:

    """Base joystick class."""

    def __init__(self, msg):
        """Initialization."""
        self._msg = msg

    def getAxisLabels(self):
        """Get all axis labels."""
        return self.axis_map.keys()

    def getButtonLabels(self):
        """Get all button labels."""
        return self.button_map.keys()

    def getAxes(self, labels):
        """Extract axis values using labels."""
        return [self._msg.axes[self.axis_map[l]] for l in labels]

    def getButtons(self, labels):
        """Extract button values using labels."""
        return [self._msg.buttons[self.button_map[l]] for l in labels]

class ThrustmasterUSBJoystick(Joystick):

    """http://www.thrustmaster.com/products/usb-joystick"""

    axis_map = {
        'HORI': 0,
        'VERT': 1,
        'SLIDER': 2,
        'THUMB_HORI': 3,
        'THUMB_VERT': 4,
    }

    button_map = {
        'TRIGGER': 0,
        'MIDDLE': 1,
        'SIDE': 2,
        'RIGHT': 3,
    }

class LogitechF710(Joystick):

    """https://www.logitech.com/en-gb/product/f710-wireless-gamepad"""

    # NOTE: this assumes that the mode setting is off (green light off, toggle
    # using mode button).

    axis_map = {
        'LHORI': 0,
        'LVERT': 1,
        'RHORI': 3,
        'RVERT': 4,
        'LT': 2,
        'RT': 5,
        'DHORI': 6,
        'DVERT': 7,
    }
    button_map = {
        'A': 0,
        'B': 1,
        'X': 2,
        'Y': 3,
        'LB': 4,
        'RB': 5,
        'BACK': 6,
        'START': 7,
        'MIDDLE': 8,
        'L3': 9,
        'R3': 10,
    }

