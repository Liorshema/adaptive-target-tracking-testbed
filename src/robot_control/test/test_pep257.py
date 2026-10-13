
# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Validate package docstrings using the selected documentation rules."""

from pathlib import Path

import pytest
from ament_pep257.main import main

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    """Check source and test docstrings for documentation style errors."""
    paths = [
        str(PACKAGE_ROOT / 'robot_control'),
        str(PACKAGE_ROOT / 'test'),
    ]

    rc = main(
        argv=[
            '--ignore=D203,D213,D406,D407,D413',
            *paths,
        ]
    )

    assert rc == 0, 'Found documentation style errors / warnings'
