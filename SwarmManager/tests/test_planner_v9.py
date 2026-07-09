import os
from pathlib import Path
from unittest.mock import Mock, patch

import pytest

from swarmmanager.planner_v9 import PIBTPlanner


# ---------------------------------------------------------
# fixtures
# ---------------------------------------------------------

@pytest.fixture
def planner(tmp_path):

    exe = tmp_path / "mapf.exe"
    exe.write_text("dummy")

    return PIBTPlanner(
        grid_size=(5, 5),
        obstacles=[(1, 1), (3, 3)]
    )


# ---------------------------------------------------------
# map generation
# ---------------------------------------------------------

def test_obstacles_are_written(planner):

    assert planner.grid[1][1] == "@"
    assert planner.grid[3][3] == "@"

    assert planner.grid[0][0] == "."


# ---------------------------------------------------------
# map writer
# ---------------------------------------------------------

def test_write_map(planner, tmp_path):

    map_file = tmp_path / "map.map"

    planner._write_map(str(map_file))

    contents = map_file.read_text()

    assert "type octile" in contents
    assert "height 5" in contents
    assert "width 5" in contents

    assert "@" in contents


# ---------------------------------------------------------
# instance writer
# ---------------------------------------------------------

def test_write_instance(planner, tmp_path):

    instance_file = tmp_path / "instance.txt"

    agents = [
        ((0, 0), (4, 4)),
        ((1, 0), (3, 4)),
    ]

    planner._write_instance(
        str(instance_file),
        "map.map",
        agents,
    )

    txt = instance_file.read_text()

    assert "agents=2" in txt
    assert "random_problem=0" in txt

    assert "0,0,4,4" in txt
    assert "1,0,3,4" in txt


# ---------------------------------------------------------
# parser
# ---------------------------------------------------------

def test_parse_output_missing_file(planner):

    result = planner._parse_output(
        "does_not_exist.txt"
    )

    assert result is None


def test_parse_output_unsolved(tmp_path, planner):

    result_file = tmp_path / "result.txt"

    result_file.write_text(
        """
solved=0
"""
    )

    result = planner._parse_output(
        str(result_file)
    )

    assert result is None


def test_parse_output_simple_solution(
    tmp_path,
    planner
):

    result_file = tmp_path / "result.txt"

    result_file.write_text(
        """
solved=1

solution=
0: (0,0), (4,4)
1: (1,0), (3,4)
2: (2,0), (2,4)
"""
    )

    paths = planner._parse_output(
        str(result_file)
    )

    assert paths is not None

    assert len(paths) == 2

    assert paths[0] == [
        (0, 0),
        (1, 0),
        (2, 0),
    ]

    assert paths[1] == [
        (4, 4),
        (3, 4),
        (2, 4),
    ]


# ---------------------------------------------------------
# subprocess execution
# ---------------------------------------------------------

@patch("subprocess.run")
def test_run_once_solver_failure(
    mock_run,
    planner,
):

    mock_run.return_value = Mock(
        returncode=1,
        stdout="fail",
        stderr="boom",
    )

    result = planner._run_once(
        [
            ((0, 0), (4, 4))
        ]
    )

    assert result == (False, None)


@patch("subprocess.run")
@patch.object(
    PIBTPlanner,
    "_parse_output"
)
def test_run_once_success(
    mock_parse,
    mock_run,
    planner,
):

    mock_run.return_value = Mock(
        returncode=0,
        stdout="",
        stderr="",
    )

    expected = [
        [(0, 0), (1, 0)]
    ]

    mock_parse.return_value = expected

    success, paths = planner._run_once(
        [
            ((0, 0), (4, 4))
        ]
    )

    assert success is True
    assert paths == expected


# ---------------------------------------------------------
# public API
# ---------------------------------------------------------

@patch.object(
    PIBTPlanner,
    "_run_once"
)
def test_plan(
    mock_run_once,
    planner,
):

    mock_run_once.return_value = (
        True,
        [[(0, 0)]]
    )

    success, paths = planner.plan(
        [
            {
                "start": (0, 0),
                "goal": (4, 4),
            }
        ]
    )

    assert success is True

    assert paths == [
        [(0, 0)]
    ]

    mock_run_once.assert_called_once()