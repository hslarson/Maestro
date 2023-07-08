# Captures information about specific channel settings
class Channel:
    def __init__(self, channel_dict):
        # Mode
        self.mode = channel_dict["@mode"]
        self.multiplied = (self.mode == "ServoMultiplied")
        if self.multiplied: self.mode = "Servo"

        # Target limits
        self.min = int(channel_dict["@min"])
        self.max = int(channel_dict["@max"])

        # Home
        self.home_mode = channel_dict["@homemode"]
        self.home = int(channel_dict["@home"])

        # Save target position
        self.target = 0


    # Updates the stored target value of the channel
    # Coerce target to be within bounds
    def update_target(self, target):
        if target < self.min:
            target = self.min

        elif target > self.max:
            target = self.max

        self.target = target
