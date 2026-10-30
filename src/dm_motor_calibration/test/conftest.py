import os
from pathlib import Path

import pytest


@pytest.fixture
def simulator():
    path = os.environ.get("DM_CALIBRATION_SIMULATOR")
    return Path(path) if path else None
