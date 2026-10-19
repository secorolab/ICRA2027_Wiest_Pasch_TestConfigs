"""Raw-bag Nav2 behavior-tree coverage extractor for the standalone QD campaigns."""

import math
from pathlib import Path
from xml.etree import ElementTree

import yaml
from rosbags.highlevel import AnyReader

from robovast.search.extractor import (Extractor, ExtractResult, NoSampleError,
                                       completed_run_dirs)


BEHAVIOR_TREE_LOG_TOPIC = '/behavior_tree_log'


class SearchBtCoverageExtract(Extractor):
    def extract(self, config_dir: Path) -> ExtractResult:
        declared = _tree_node_names(config_dir, self.params.get('bt_xml'))
        runs = completed_run_dirs(config_dir)
        if not runs:
            return self._result(config_dir, 0.0)

        values = [_run_coverage(run_dir, declared) for run_dir in runs
                  if not _died_before_recording(run_dir)]
        if not values:
            raise NoSampleError(f'{config_dir}: no completed run recorded a behavior-tree bag')
        return self._result(config_dir, sum(values) / len(values))

    @staticmethod
    def _result(config_dir: Path, coverage: float) -> ExtractResult:
        return ExtractResult(
            objectives={'nav2_bt_coverage': coverage},
            measures=_search_measures(config_dir),
        )


def _tree_node_names(config_dir: Path, bt_xml) -> set[str]:
    """The distinct node names of the tree nav2 loads -- the coverage denominator.

    Same naming rule as robovast_nav's ``nav2_bt_tree`` and the system tests'
    ``bt_coverage``: a node's name is its ``name`` attribute or its XML tag,
    ``<SubTree>`` references expand inline, and the denominator is read from the
    tree, never assumed. A missing or unparseable tree makes a cell unmeasurable
    rather than a fabricated fraction of a hardcoded constant.
    """
    if not bt_xml:
        raise RuntimeError(
            f'{config_dir}: extract.params.bt_xml is unset; the QD coverage denominator '
            'must be read from the same tree nav2 loads the system tests measure')
    # During result processing, run_files are staged for the extractor alongside the
    # plugin: /config/search/<plugin>.py and /config/files/<tree>.xml. The downloaded
    # result directory is not guaranteed to contain run_files, so resolve the tree
    # from the staged project root as well.
    source_root = Path(__file__).resolve().parents[1]
    candidates = [
        config_dir / '_config' / bt_xml,
        config_dir / bt_xml,
        source_root / bt_xml,
    ]
    xml_path = next((p for p in candidates if p.is_file()), None)
    if xml_path is None:
        raise NoSampleError(
            f'{config_dir}: cannot find the behavior-tree nav2 loads ({bt_xml!r}); '
            'behavior-tree coverage is unmeasurable')
    try:
        root = ElementTree.parse(xml_path).getroot()
    except ElementTree.ParseError as exc:
        raise NoSampleError(
            f'{config_dir}: cannot parse {xml_path}: {exc}') from exc

    trees = {bt.get('ID'): bt for bt in root.iter('BehaviorTree')}
    main_id = root.get('main_tree_to_execute')
    main = trees.get(main_id) if main_id else None
    if main is None:
        main = next(iter(trees.values()), None)
    if main is None:
        raise NoSampleError(f'{config_dir}: no BehaviorTree in {xml_path}')

    names = set()

    def walk(el, active_trees):
        names.add(el.get('name') or el.tag)
        for child in list(el):
            if child.tag == 'SubTree':
                ref = child.get('ID')
                sub = trees.get(ref)
                if sub is not None and ref not in active_trees:
                    for sub_child in list(sub):
                        walk(sub_child, active_trees | {ref})
                continue
            walk(child, active_trees)

    for child in list(main):
        walk(child, frozenset({main_id} if main_id else set()))
    return names


def _run_coverage(run_dir: Path, declared: set[str]) -> float:
    bag_dir = run_dir / 'rosbag2'
    if not bag_dir.is_dir():
        raise RuntimeError(f'{run_dir}: completed run has no rosbag2/')

    reached = set()
    with AnyReader([bag_dir]) as reader:
        connections = [c for c in reader.connections if c.topic == BEHAVIOR_TREE_LOG_TOPIC]
        if not connections:
            raise RuntimeError(f'{run_dir}: bag has no {BEHAVIOR_TREE_LOG_TOPIC}')
        for connection, _timestamp, raw in reader.messages(connections=connections):
            message = reader.deserialize(raw, connection.msgtype)
            for event in message.event_log:
                if event.current_status and event.current_status != 'IDLE':
                    reached.add(event.node_name)
    return len(reached & declared) / len(declared)


def _search_measures(config_dir: Path) -> dict[str, float]:
    scenario_cfg = config_dir / '_config' / 'scenario.config'
    params = {}
    if scenario_cfg.exists():
        with open(scenario_cfg, encoding='utf-8') as handle:
            params = next(iter(yaml.safe_load(handle).values()), {})
    if not isinstance(params, dict):
        params = {}

    controller = params.get('controller_profile', '')
    planner = params.get('planner_profile', '')
    return {
        'path_length': _path_length(params),
        'static_obstacle_amount': float(len(params.get('static_objects') or [])),
        'static_obstacle_distance': _float(params.get('static_obstacle_distance')),
        'dynamic_obstacle_trigger_distance': _float(params.get('dynamic_obstacle_trigger_distance')),
        'dynamic_obstacle_lateral_distance': _float(params.get('dynamic_obstacle_lateral_distance')),
        'controller_profile': _profile_index(controller, ('files/nav2_params_mppi.yaml',
                                                           'files/nav2_params_dwb.yaml')),
        'planner_profile': _profile_index(planner, ('nav2_navfn_planner::NavfnPlanner',)),
        'use_astar': float(bool(params.get('use_astar', False))),
    }


def _died_before_recording(run_dir: Path) -> bool:
    if (run_dir / 'rosbag2').is_dir():
        return False
    report = run_dir / 'test.xml'
    if not report.exists():
        return False
    try:
        suite = ElementTree.parse(report).getroot()
        return int(suite.get('failures', 0)) + int(suite.get('errors', 0)) > 0
    except (ElementTree.ParseError, TypeError, ValueError):
        return False


def _path_length(params: dict) -> float:
    start = params.get('start_pose') or {}
    goals = params.get('goal_poses') or []
    if params.get('goal_pose'):
        goals = [params['goal_pose']]
    points = [start.get('position', {})] + [goal.get('position', {}) for goal in goals]
    return sum(math.dist((a.get('x', 0.0), a.get('y', 0.0)),
                         (b.get('x', 0.0), b.get('y', 0.0)))
               for a, b in zip(points, points[1:]))


def _profile_index(value, profiles) -> float:
    """Which archive bin this profile is, refusing one the archive has no bin for.

    An UNRECOGNISED value raises rather than defaulting to 0.0: that is a campaign whose
    search space and this mapping have drifted apart, and the cheap answer silently files
    every cell under the FIRST profile -- an archive that looks populated, measures a
    dimension it never varied, and says nothing about it.

    An ABSENT value is a different thing and not always an error. A profile the campaign
    never varies is never written into scenario.config, so it arrives here empty; where
    only one profile is declared that is unambiguous, and the constant is bin 0. Where
    several are declared it is the drift above seen from the other side -- the dimension
    was supposed to vary and nothing recorded it -- so that still raises.
    """
    if value in ("", None):
        if len(profiles) == 1:
            return 0.0
        raise ValueError(
            f"no profile was recorded, but this archive declares {len(profiles)} "
            f"of them {profiles}; a profile the search varies has to reach "
            "scenario.config, or every cell lands in one bin")
    try:
        return float(profiles.index(value))
    except ValueError:
        raise ValueError(
            f"{value!r} is not one of this archive's profiles {profiles}; the "
            "search_space values and this mapping have to name the same set") from None


def _float(value) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0
