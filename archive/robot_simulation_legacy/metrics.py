import math


def mean(values):
    if not values:
        return 0.0

    return sum(values) / len(values)


def mae(errors):
    """
    Mean Absolute Error.
    """
    if not errors:
        return 0.0

    return mean([
        abs(error)
        for error in errors
    ])


def rmse(errors):
    """
    Root Mean Square Error.
    """
    if not errors:
        return 0.0

    squared_errors = [
        error ** 2
        for error in errors
    ]

    return math.sqrt(mean(squared_errors))


def max_absolute_error(errors):
    """
    Maximum absolute error.
    """
    if not errors:
        return 0.0

    return max(
        abs(error)
        for error in errors
    )


def final_error(errors):
    """
    Last recorded error.
    """
    if not errors:
        return 0.0

    return errors[-1]