"""
Test the periodic module.
"""
import asyncio

import pytest

from src.util.periodic import periodic_callback

INTERVAL = 1 / 100
ITERATIONS = 10


@pytest.mark.asyncio
async def test_periodic_callback_pass_tick_to_callback():
    """
    Check that the tick argument is passed to the callback and that it is the right value.
    """
    count = 0

    def func_cb(tick):
        nonlocal count
        count += 1
        assert count == tick

    periodic_callback(func_cb, INTERVAL)
    await asyncio.sleep(INTERVAL * ITERATIONS)


@pytest.mark.asyncio
async def test_periodic_callback_precise_periodicity():
    """
    Check that the callback is called at a precise interval.
    """
    count = 0

    def func_cb(_):
        nonlocal count
        count += 1

    periodic_callback(func_cb, INTERVAL)

    # We need to subtract .5, otherwise the *last* execution will be executed at the same time as
    # the asyncio.sleep (just after or before, depending on the scheduling). In order for this test
    # to be fully deterministic, we subtract half an iteration so we are sure only ITERATIONS
    # loops are executed.
    await asyncio.sleep(INTERVAL * (ITERATIONS - .5))

    assert count == ITERATIONS


@pytest.mark.asyncio
async def test_periodic_callback_arguments_passed_to_callback():
    """
    Check that the arguments provided in periodical_callback are passed to the callback.
    :return: 
    """

    def func_cb(_, arg_a, arg_b):
        assert arg_a == 42
        assert arg_b == "test"

    periodic_callback(func_cb, INTERVAL, 42, "test")

    # Wait for the callback to be called at least once before ending the test.
    await asyncio.sleep(INTERVAL * 2)
