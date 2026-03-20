# Copyright (c) 2024 Your Name
# SPDX-License-Identifier: MIT
"""ament_copyright compliance test."""

import pytest
from ament_copyright.main import main


@pytest.mark.linter
@pytest.mark.copyright
def test_copyright():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found copyright errors"
