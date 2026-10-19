# SPDX-License-Identifier: Apache-2.0
"""Behaviour-tree elicitation: how much of nav2's tree a situation made tick, and whether
the run still succeeded.

This is the measurement the whole package exists to produce, in the form a SEARCH can
optimise. Two pieces:

``BtCoverage``  postprocessing -- one ``bt_coverage.csv`` row per run.
``BtExtract``   search extractor -- one ``elicitation`` objective per cell.

**The denominator is read, never assumed.** ``nav2_bt_tree`` emits one row per node of the
XML nav2 actually loaded, with ``status_name`` INVALID for a node that never ticked, so the
tree's size and the reached set come from the same file. The 22 quoted throughout this package
is that count for ``files/navigate_to_pose_w_replanning_and_recovery.xml``; hardcoding it would keep reporting fractions of 22
after the tree changed, with no test failing.

**Why elicitation and not coverage.** Reaching many nodes is worthless on its own: a run
that fails exhausts the recovery subtree by definition, which is the mechanism upstream's own
suite reaches those nodes by. What upstream cannot show is nav2 CHOOSING a recovery behaviour
in a run that then succeeds, and that is what this objective is pointed at::

    elicitation = reached / total  +  (behaviour_kinds / 3) * arrived

The first term is a gradient the search can follow from anywhere. The second is the prize,
and it is deliberately discontinuous in ``arrived``: arriving is not a matter of degree, and
smoothing it would let a search trade a completed task for a few more nodes -- which is the
result this package already has and does not need more of.

**"Arrived" is measured on GROUND TRUTH, never on the scenario's verdict.** nav2 declares
success against its own estimated pose, so a mislocalized robot reports SUCCEEDED while
standing somewhere else: in W-3's aisles every run seeded one aisle off passed with its true
position 2.3 m from the goal, and two runs that fired all three behaviours and "passed" ended
2.1 m away. An objective built on that verdict rewards a robot for being confidently wrong,
and a search will find that cheaper than solving the situation. So the second term is gated on
the distance from the goal IN THE WORLD, and a run whose ground-truth track is missing is not
scored at all.

**Aggregated BEST over repetitions, not worst.** The question decides the aggregation:
robustness asks *can this cell fail*, so one bad draw settles it; elicitation asks *can this
cell elicit*, and one good draw settles that. Averaging is wrong for both. The rate is
reported as a MEASURE beside the objective, so a demonstration cannot be read as a frequency.
"""

import csv
import logging
from pathlib import Path
from typing import Tuple

from robovast.common.campaign_data import read_test_result
from robovast.results_processing.postprocessing_plugins import BasePostprocessingPlugin
from robovast.search.aggregate import aggregate
from robovast.search.extractor import (Extractor, ExtractResult, NoSampleError,
                                       completed_run_dirs)

logger = logging.getLogger(__name__)

#: The three behaviour-server actions upstream's behaviors/* tests call directly. Named by
#: the BT node's ``behavior_name``, which for these is the XML tag: they carry no name=
#: attribute in files/navigate_to_pose_w_replanning_and_recovery.xml.
BEHAVIOUR_NODES = ('Spin', 'Wait', 'BackUp')

#: A node that never ticked. ``nav2_bt_tree`` maps BT.CPP's IDLE and SKIPPED onto INVALID,
#: so this one string is the whole "not reached" set.
NOT_REACHED = 'INVALID'

#: Written by ``nav2_bt_tree``. The tree AND the transitions, which is why the denominator
#: does not have to be declared anywhere.
_TREE_FILE = 'nav2_behaviors.csv'

#: Written by ``rosbags_tf_to_csv``. Every TF frame, including the ground-truth one.
_POSES_FILE = 'poses.csv'

#: Substring identifying the ground-truth frame there. The world publishes it from a
#: ``ground_truth_pose`` component kept deliberately OUT of the localization chain.
_GT_FRAME = '_gt'


def _rows(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with open(path, newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def _arrived(run_dir: Path, goal: tuple, tolerance: float):
    """Did the robot END UP at the goal, in the world? ``None`` when nothing says.

    Deliberately not ``read_test_result``: that is nav2's verdict, computed against the pose
    nav2 BELIEVES it holds, and several situations here are built to corrupt exactly that
    belief.

    The tolerance is larger than nav2's own ``xy_goal_tolerance`` on purpose -- this is the
    last recorded ground-truth sample, so it carries the localization error and whatever
    settling happened after the bag closed, neither of which nav2's check sees.
    """
    last = None
    for row in _rows(run_dir / _POSES_FILE):
        if _GT_FRAME not in (row.get('frame') or ''):
            continue
        try:
            last = (float(row['position.x']), float(row['position.y']))
        except (KeyError, TypeError, ValueError):
            continue
    if last is None:
        return None
    return (last[0] - goal[0]) ** 2 + (last[1] - goal[1]) ** 2 <= tolerance ** 2


def _measure(run_dir: Path) -> dict | None:
    """Elicitation figures for one run, or ``None`` when the tree table is absent.

    ``None`` and not a row of zeros. A run whose ``/behavior_tree_log`` never published looks
    exactly like a run that ticked nothing, and the two are different facts: one is a missing
    measurement, the other is a measurement. Scoring the first as 0 coverage would be true by
    accident and false as a measurement.
    """
    rows = _rows(run_dir / _TREE_FILE)
    if not rows:
        return None

    # Counted by NAME, not by the tree's per-instance ``behavior_id``. The two differ: the
    # XML has more node INSTANCES than distinct names, because an unnamed structural node
    # (a `RecoveryNode`, a `Sequence`) appears once per instance. Every coverage number in
    # this package and in the ported baseline it is compared against -- 7/22, 21/22, 0/22 --
    # is a count of names, and a fraction of a different denominator is not comparable with
    # any of them, however defensible it is on its own.
    total, reached, kinds, ticks = set(), set(), set(), 0
    for row in rows:
        name = row.get('behavior_name') or row.get('behavior_id')
        total.add(name)
        if row.get('status_name', NOT_REACHED) != NOT_REACHED:
            reached.add(name)
            if name in BEHAVIOUR_NODES:
                kinds.add(name)
                ticks += 1
    if not total:
        return None
    return {'nodes_total': len(total), 'nodes_reached': len(reached),
            'behaviour_kinds': len(kinds), 'behaviour_ticks': ticks,
            'behaviours': '|'.join(sorted(kinds))}


class BtCoverage(BasePostprocessingPlugin):
    """Write ``bt_coverage.csv`` beside each run: what the tree did, and whether it arrived.

    Keyword parameters (from the ``.vast``):
        ``file``            filename to write (default ``bt_coverage.csv``)
        ``force``           rewrite even when the output is newer than its input
        ``goal_x``/``goal_y``  the trial's goal, in the world frame. MUST match the campaign's
                            goal_pose: a default left over from another world silently grades
                            every run against the wrong point.
        ``arrival_radius``  [m] ground-truth distance within which the robot counts as arrived
    """

    def __call__(self, results_dir: str, config_dir: str,
                 file: str = 'bt_coverage.csv', force: bool = False,
                 goal_x: float = 2.5, goal_y: float = 0.0,
                 arrival_radius: float = 0.6, **kwargs) -> Tuple[bool, str]:
        del config_dir  # every input is per-run
        written, skipped, unmeasured = 0, 0, []
        # Driven off the TREE table rather than off the run directories: a run that produced
        # no tree has nothing to measure, and rglob makes that the same statement as "there
        # is no such file" instead of a second code path.
        for tree_csv in sorted(Path(results_dir).rglob(_TREE_FILE)):
            run_dir = tree_csv.parent
            out = run_dir / file
            if not force and out.exists() and out.stat().st_mtime >= tree_csv.stat().st_mtime:
                skipped += 1
                continue
            row = _measure(run_dir)
            if row is None:
                unmeasured.append(run_dir.name)
                continue
            arrived = _arrived(run_dir, (goal_x, goal_y), arrival_radius)
            if arrived is None:
                # No ground-truth track, so whether the robot arrived is unknown -- and the
                # scenario verdict is not a substitute for it here. Left unmeasured rather
                # than filled in.
                unmeasured.append(run_dir.name)
                continue
            row['arrived'] = int(arrived)
            # Kept beside it, never instead of it: where the two DISAGREE is the finding --
            # nav2 reporting success from a pose it does not hold.
            row['verdict_passed'] = int(bool(read_test_result(run_dir)['success']))
            with open(out, 'w', newline='', encoding='utf-8') as handle:
                writer = csv.DictWriter(handle, fieldnames=list(row))
                writer.writeheader()
                writer.writerow(row)
            written += 1

        note = f"{written} run(s) measured, {skipped} up to date"
        if unmeasured:
            # Reported, never defaulted: see _measure. A caller seeing this note and a thin
            # campaign is looking at a bag without /behavior_tree_log, not at a stack that
            # ticked nothing.
            note += (f"; {len(unmeasured)} had an empty {_TREE_FILE} "
                     f"({', '.join(unmeasured)})")
        if not written and not skipped:
            note += (f"; NO {_TREE_FILE} anywhere under {results_dir} -- check "
                     f"/behavior_tree_log is in bag_record and that nav2_bt_tree is listed "
                     f"after rosbags_nav2bt_to_csv")
        return True, note


class BtExtract(Extractor):
    """Score a cell on whether it made nav2 recover, and whether the run still arrived.

    ``params``:
        ``metrics``    per-run CSV written by :class:`BtCoverage`
        ``aggregate``  ``worst`` (default) -- which, with ``higher_is_safer=False``, is the
                       BEST draw. See the module docstring: this is an existence question.
    """

    def extract(self, config_dir: Path) -> ExtractResult:
        runs = completed_run_dirs(config_dir)
        if not runs:
            raise NoSampleError(
                f"{config_dir}: no run produced a result (no test.xml), so there is no tree "
                f"to score")

        metrics_file = self.params.get('metrics', 'bt_coverage.csv')
        how = self.params.get('aggregate', 'worst')

        scores, fractions, elicited, passes = [], [], [], []
        unmeasured = []
        for run_dir in runs:
            rows = _rows(run_dir / metrics_file)
            if not rows:
                unmeasured.append(run_dir.name)
                continue
            row = rows[0]
            total = float(row.get('nodes_total') or 0)
            if total <= 0:
                unmeasured.append(run_dir.name)
                continue
            reached = float(row.get('nodes_reached') or 0)
            kinds = float(row.get('behaviour_kinds') or 0)
            arrived = float(row.get('arrived') or 0)

            fraction = reached / total
            scores.append(fraction + (kinds / len(BEHAVIOUR_NODES)) * arrived)
            fractions.append(fraction)
            passes.append(arrived)
            # THE FIND, as funnel.vast defines it: a behaviour fired and the run still
            # arrived. Its own measure rather than inferred from the objective, whose two terms
            # are not separable after the fact.
            elicited.append(1.0 if (kinds > 0 and arrived) else 0.0)

        if not scores:
            raise NoSampleError(
                f"{config_dir}: {len(runs)} run(s) produced a verdict but no "
                f"'{metrics_file}' row (runs {', '.join(unmeasured)}), so nothing about the "
                f"behaviour tree was recorded for this cell.")
        if unmeasured:
            logger.warning(
                "%s: %d of %d run(s) had no '%s' row (runs %s); the cell is scored over the "
                "remaining %d.", config_dir, len(unmeasured), len(runs), metrics_file,
                ", ".join(unmeasured), len(scores))

        return ExtractResult(
            objectives={
                # MAXIMIZED -- declare {direction: maximize} in the .vast. `higher_is_safer`
                # False makes `worst` return the maximum, which is the best draw: see the
                # module docstring for why an existence question aggregates that way.
                'elicitation': aggregate(scores, how=how, higher_is_safer=False),
            },
            measures={
                # The rate, so a demonstration is never read as a frequency. Over every
                # MEASURED run, and the aggregation above deliberately does not touch it.
                'elicited_rate': sum(elicited) / len(elicited),
                # ARRIVAL rate, on ground truth -- not nav2's pass rate, which the aisle
                # campaign shows can be 3/3 while no run reached the goal.
                'arrival_rate': sum(passes) / len(passes),
                'nodes_fraction': aggregate(fractions, how=how, higher_is_safer=False),
            })
