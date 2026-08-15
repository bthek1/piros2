# Copyright 2015 Open Source Robotics Foundation, Inc.
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

# Modified from the generated version: paths are anchored to this file rather
# than the CWD, so the test checks its own package no matter where pytest is
# invoked from — colcon runs it with cwd at the package root, VSCode from the
# workspace root, and the generated argv=[] form lints whichever it gets.
from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest

PACKAGE_DIR = str(Path(__file__).resolve().parents[1])


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[PACKAGE_DIR])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
