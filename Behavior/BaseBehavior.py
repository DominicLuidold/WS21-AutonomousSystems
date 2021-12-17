import abc


class BaseBehavior:
    """
    Defines methods and functionality that a specific behavior must implement.
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def is_applicable(self, distances: list, no_detection_distance: int) -> bool:
        """
        Determines whether a behavior is applicable for the current situation and the given parameters.

        :return: true if behavior is applicable, false otherwise
        """
        pass

    @abc.abstractmethod
    def calculate_motor_value(self, distances: list, no_detection_distance: int) -> (float, float):
        """
        Calculates the required motor velocity for the current behavior.

        :return: (float, float)
            left and right motor velocity
        """
        pass
