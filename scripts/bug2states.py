# bug2states.py

from state import State

# Start of our states
class GoalSeek(State):
    """
    The state which the robot will orient itself toward the goal.
    """

    def on_event(self, event):
        if event == 'wall_reach':
            return WallFollow()

        return self


class WallFollow(State):
    """
    The state which the robot will follow along a wall till it comes back 
    to it original path.
    """

    def on_event(self, event):
        if event == 'original_path_reach':
            return GoalSeek()

        return self
# End of our states.

