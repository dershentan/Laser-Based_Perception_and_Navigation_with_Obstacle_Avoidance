# state.py

class State(object):
    """
    We define a state object which provides some utility functions for the
    individual states within the state machine.
    """

    def __init__(self):
        """
        Leaving it empty for now
        """
        pass

    def on_event(self, event):
        """
        Handle events that are delegated to this State.
        """
        pass

    def __repr__(self):
        """
        Leverages the __str__ method to describe the State.
        Note to myself: Calling `self` only with "`" wrapping around it will call this method.
        This allow eval(`self`) to be call without throwing an error compared to __str__(self) or self
        """
        return self.__str__()

    def __str__(self):
        """
        Returns the name of the State.
        Note to myself: Calling self or variable reference will call this method
        and if __str__(self) is not define, it will call __repr__(self).
        """
        return self.__class__.__name__

