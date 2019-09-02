"""
Test for localization module.
"""
import asyncio
import time

import pytest

from src.robot.repository.localization import LocalizationRepository

INTERVAL = 0.01


@pytest.mark.asyncio
async def test_set_is_moving():
    """
    Test that the wait_for_stop_moving blocks until the robot is set moving to False.
    """
    localization_repository = LocalizationRepository()

    async def movement_finished_after_some_time():
        await asyncio.sleep(INTERVAL)
        localization_repository.set_is_moving(False)

    localization_repository.set_is_moving(True)
    asyncio.create_task(movement_finished_after_some_time())

    start_time = time.perf_counter()
    await localization_repository.wait_for_stop_moving()

    assert time.perf_counter() - start_time > INTERVAL
