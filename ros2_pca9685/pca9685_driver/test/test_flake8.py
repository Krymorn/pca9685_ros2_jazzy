# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT
"""ament_flake8 compliance test."""

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.linter
@pytest.mark.flake8
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, f"Found code style errors:\n{errors}"
