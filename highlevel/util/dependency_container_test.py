"""
Test for injector module.
"""
from abc import ABC, abstractmethod

import pytest

from highlevel.util.dependency_container import DependencyContainer


class DummyA:
    """
    Dummy A.
    """
    def __init__(self):
        pass


class DummyB:
    """
    Dummy B.
    """
    def __init__(self, my_dep):
        self.my_dep = my_dep


class DummyC:
    """
    Dummy C.
    """
    def __init__(self, my_other_dep, my_array):
        self.my_other_dep = my_other_dep
        self.my_array = my_array


class AbstractClass(ABC):
    """
    Abstract class, should not be instantiable.
    """
    @abstractmethod
    def do_something(self):
        """
        Do something.
        """


def test_injector_happy_path():
    """
    Happy path.
    """
    i = DependencyContainer()
    i.provide('my_dep', DummyA)
    i.provide('my_array', [1, 2, 3])
    i.provide('my_other_dep', DummyB)
    i.provide('dummy_c', DummyC)

    dummy_c = i.get('dummy_c')
    assert dummy_c is not None
    assert dummy_c.my_array == [1, 2, 3]
    assert isinstance(dummy_c.my_other_dep, DummyB)
    assert isinstance(dummy_c.my_other_dep.my_dep, DummyA)


def test_injector_double_provide():
    """
    Should not be able to provide the same thing twice.
    """
    i = DependencyContainer()
    i.provide('test', DummyA)
    with pytest.raises(Exception):
        i.provide('test', DummyA)


def test_injector_abstract_class():
    """
    Should not be able to provide an abstract class.
    """
    i = DependencyContainer()
    with pytest.raises(Exception):
        i.provide('test', AbstractClass)


def test_injector_not_provided():
    """
    Should return an exception if not provided.
    """
    i = DependencyContainer()
    with pytest.raises(Exception):
        i.get('test')


def test_injector_missing_dependencies():
    """
    Should not be able to instantiate if a dependency is missing.
    """
    i = DependencyContainer()
    i.provide('test', DummyC)
    with pytest.raises(Exception):
        i.get('test')


def test_injector_manual_argument_in_constructor():
    """
    Should be able to manually pass arguments for the constructor.
    """
    i = DependencyContainer()
    i.provide('dummy_c', DummyC, my_other_dep=1, my_array=[42])

    dummy_c = i.get('dummy_c')
    assert dummy_c is not None
    assert dummy_c.my_array == [42]
    assert dummy_c.my_other_dep == 1
