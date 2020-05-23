"""
Trapezoid functions
"""


class TrapezoidFilter:
    """
    Applies a trapezoid shape to variable's first order derivative, computes its result
    """
    def __init__(self, tolerance: float, max_first_order: float,
                 max_second_order: float, update_rate: int):
        self.tolerance = tolerance
        self.update_rate = update_rate
        self.max_second_order = max_second_order
        self.max_first_order = max_first_order

        self.output_last = 0.0
        self.output_first_order_last = 0.0

    def reset_position_to(self, value: float) -> None:
        """
        Sets the last used position to the given value.
        Resets the first order derivative of it.
        """
        self.output_last = value
        self.output_first_order_last = 0.0

    def compute(self, target: float) -> float:
        """
        Updates the trapezoid with a target for the controlled variable.
        Uses any previously updated parameter (first order derivative, previous outputs)
        """

        distance = target - self.output_last

        if abs(distance) <= self.tolerance:
            self.output_first_order_last = 0.0
            self.output_last = target
            return target

        stop_distance = (self.output_first_order_last**
                         2) / (2 * self.max_second_order)
        direction = -1 if distance < 0 else 1

        if direction * distance < stop_distance:
            self.output_first_order_last -= direction * self.max_second_order / self.update_rate
        else:
            self.output_first_order_last += direction * self.max_second_order / self.update_rate
            if direction * self.output_first_order_last > self.max_first_order:
                self.output_first_order_last = direction * self.max_first_order

        self.output_last += self.output_first_order_last / self.update_rate
        return self.output_last
