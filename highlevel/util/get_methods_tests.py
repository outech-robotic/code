from highlevel.util.get_methods import get_methods, RobotFunction


class StubGateway:
    """
    Just a fake gateway to be used to test the get_method.
    """

    def my_method(self, arg1: str, arg2: int):
        """
        This method does something.
        """

    def my_other_method(self):
        pass


def test_get_methods():
    gateway = StubGateway()
    got = get_methods(gateway)
    assert got == [
        RobotFunction(
            name='StubGateway.my_method',
            documentation='This method does something.',
            args={'arg1': 'str', 'arg2': 'int'},
            func=gateway.my_method,
        ),
        RobotFunction(
            name='StubGateway.my_other_method',
            documentation=None,
            args={},
            func=gateway.my_other_method,
        )
    ]
