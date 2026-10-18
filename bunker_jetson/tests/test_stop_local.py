"""stop_local 只认本仓库的控制台进程，不误伤别的 Python。"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from stop_local import cmdline_is_local_console  # noqa: E402

ROOT = _ROOT


def test_absolute_run_local_matches():
    cmd = f"/usr/bin/python3 {ROOT / 'run_local.py'} --daemon --no-lidar"
    assert cmdline_is_local_console(cmd, "/tmp", ROOT)


def test_relative_run_agent_in_repo_matches():
    cwd = str(ROOT / "bunker_jetson")
    assert cmdline_is_local_console("python3 run_agent.py --local", cwd, ROOT)


def test_other_python_in_repo_does_not_match():
    assert not cmdline_is_local_console("python3 other_tool.py", str(ROOT), ROOT)


def test_same_script_name_outside_repo_does_not_match():
    assert not cmdline_is_local_console(
        "python3 run_local.py", "/home/someone/other", ROOT,
    )


def test_lookalike_filename_does_not_match():
    cwd = str(ROOT)
    assert not cmdline_is_local_console("python3 not_run_local.py", cwd, ROOT)


if __name__ == "__main__":
    test_absolute_run_local_matches()
    test_relative_run_agent_in_repo_matches()
    test_other_python_in_repo_does_not_match()
    test_same_script_name_outside_repo_does_not_match()
    test_lookalike_filename_does_not_match()
    print("PASS test_stop_local")
