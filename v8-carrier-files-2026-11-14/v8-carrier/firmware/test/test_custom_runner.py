"""PlatformIO custom test runner for `pio test -e native` (test_framework = custom).

Parses the lines minitest.h prints, one per test:
    <file>:<line>:<test_name>:PASS
    <file>:<line>:<test_name>:FAIL
The CMake/CTest build (firmware/CMakeLists.txt) does not use this file.
"""

import re

from platformio.test.result import TestCase, TestCaseSource, TestStatus
from platformio.test.runners.base import TestRunnerBase

LINE_RE = re.compile(r"^(?P<file>.+?):(?P<line>\d+):(?P<name>[A-Za-z0-9_]+):(?P<status>PASS|FAIL)\s*$")


class CustomTestRunner(TestRunnerBase):
    EXTRA_LIB_DEPS = []

    def configure_build_env(self, env):
        env.Append(CXXFLAGS=["-std=c++17"])
        return env

    def parse_test_case(self, line):
        m = LINE_RE.match(line.strip())
        if not m:
            return None
        status = TestStatus.PASSED if m.group("status") == "PASS" else TestStatus.FAILED
        return TestCase(
            name=m.group("name"),
            status=status,
            source=TestCaseSource(filename=m.group("file"), line=int(m.group("line"))),
        )
