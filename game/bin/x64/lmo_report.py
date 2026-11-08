#!/usr/bin/env python3
"""
lmo_report — ask the scale report why a side or brush ended at its lightmapscale.

lmoptimizer writes '<output stem>.lmo_report.json' beside the output VMF (not
on --dry-run or failure). It holds, for every side with a face extent, the
scale it came in at, the scale the solver chose, the scale it left with, and
the solver's reasoning in between; a side without an extent (a displacement
whose polygon vanished) is there too when a floor or a stage set its scale,
flagged no_geometry. This script explains those records in plain
English so an investigation does not need the pipeline rebuilt. Format and
vocabulary: docs/lmoptimizer_report.md. Standard library only.

Usage (REPORT is the .lmo_report.json path; IDs are VMF ids of the OUTPUT):
    python lmo_report.py REPORT side ID       one side, explained
    python lmo_report.py REPORT brush ID      every side of a brush, explained
    python lmo_report.py REPORT summary       run header, counts per decision and per final scale
    python lmo_report.py REPORT skipped [--top N]
        sides left at baseline because the skip-ahead pass stepped over them,
        best rank first (value/cost, or priority under --promote-order priority;
        default 20 shown, 0 = all); these are the "why was this big floor not
        promoted" candidates

A side id from the INPUT VMF that --chop carved away is not in the report
under that id; `side` then lists the pieces that replaced it.

Exit status: 0 = answered; 1 = unreadable report, or no such side/brush.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import List, Optional, Sequence


class ReportError(Exception):
    """The report cannot be read or does not contain what was asked for."""


def load_report(path: Path) -> dict:
    """Parse a report file and check it has the two top-level tables."""
    try:
        with open(path, encoding='utf-8') as f:
            report = json.load(f)
    except OSError as e:
        raise ReportError(f"cannot read {path}: {e}")
    except ValueError as e:
        raise ReportError(f"{path} is not valid JSON: {e}")
    if not isinstance(report, dict) or not isinstance(report.get('run'), dict) \
            or not isinstance(report.get('sides'), dict):
        raise ReportError(f"{path} is not an lmoptimizer scale report "
                          f"(expected 'run' and 'sides' tables)")
    return report


def _n(value, digits: int = 0) -> str:
    """A number for text, '?' when the report does not have it."""
    if value is None:
        return '?'
    if isinstance(value, float):
        return f"{value:,.{digits}f}"
    if isinstance(value, int) and not isinstance(value, bool):
        return f"{value:,}"
    return str(value)


def _plural(count: int, word: str) -> str:
    return f"{count} {word}{'' if count == 1 else 's'}"


# What a promotion unit's fate means, for the "rides along" wording.
_UNIT_FATE = {
    'promoted_prefix': 'was promoted in the prefix that fit',
    'promoted_skip_ahead': 'was promoted by the skip-ahead pass',
    'stepped_over': 'did not fit and was left at baseline',
    'not_reached': 'was never tried',
}


def _added_cells(rec: dict) -> Optional[int]:
    at, promoted = rec.get('chop_cells_at_baseline'), rec.get('chop_cells_promoted')
    return None if at is None or promoted is None else promoted - at


def _rank_basis(result: dict) -> str:
    """What the rank orders by, from the run's --promote-order: 'value/cost'
    or 'priority'; anything else (an older report) is just the promotion order."""
    return {'value': 'value/cost', 'priority': 'priority'}.get(
        result.get('promote_order'), 'promotion order')


def _unit_text(rec: dict, units: Optional[int], basis: str) -> str:
    size = rec.get('unit_size', 1)
    of = f" of {units:,}" if units else ""
    return (f"rank {_n(rec.get('rank'))}{of} by {basis}, "
            f"{_plural(size, 'side')} in its unit")


def _cost_text(rec: dict) -> str:
    """The estimate that ranked and priced the promotion."""
    base, promoted = rec.get('scale_at_baseline'), rec.get('scale_if_promoted')
    cells = rec.get('chop_cells_at_baseline')
    parts = [f"this side goes from {_n(cells)} chop "
             f"cell{'' if cells == 1 else 's'} at scale {_n(base)} to "
             f"{_n(rec.get('chop_cells_promoted'))} at scale {_n(promoted)} "
             f"(+{_n(_added_cells(rec))})"]
    if rec.get('unit_size', 1) > 1:
        parts.append(f"its unit sums to cost {_n(rec.get('unit_cost'), 1)} cells "
                     f"and value {_n(rec.get('unit_value'))}")
    else:
        parts.append(f"estimated cost {_n(rec.get('cost'), 1)} cells, "
                     f"value {_n(rec.get('value'))}")
    return "; ".join(parts)


def _why(rec: dict, run: dict) -> List[str]:
    """Plain-English lines for the solver's decision about one side."""
    result = run.get('result', {})
    budget = result.get('vertex_budget')
    baseline = result.get('baseline_scale')
    units = result.get('promotion_units')
    basis = _rank_basis(result)
    decision = rec.get('decision')
    base, promoted = rec.get('scale_at_baseline'), rec.get('scale_if_promoted')

    if decision is None:
        if run.get('solver') == 'calibration':
            return ["No per-side decision: the calibration fallback solver ran "
                    "(no VBSP), and it records only the scales it chose."]
        return ["No decision was recorded for this side."]
    if decision == 'skipped':
        return [f"Not solved ({rec.get('skip_reason', 'skipped')}): the solver never "
                f"saw it, so only the floors below can move it."]
    if decision == 'uniform':
        return [f"Pre-promoted to its max useful scale {_n(rec.get('max_useful_scale'))}"
                f" (the coarsest at which it is not chopped; minimum scale "
                f"{_n(rec.get('min_scale'))}): its baseline lightmap is perfectly "
                f"uniform and dark, so a coarse scale loses nothing."]
    if decision == 'evidence_held':
        return [f"Held at or above baseline {_n(baseline)} by an evidence floor: its "
                f"baseline lightmap is reproduced at scale "
                f"{_n(rec.get('evidence_scale'))}, so a finer scale would spend "
                f"budget for nothing. It took no promotion slot."]
    if decision == 'all_fit_scale1':
        return [f"Every side fit the budget at scale 1, so nothing was degraded and "
                f"no promotion ran. Solved at the finest scale its minimum "
                f"({_n(rec.get('min_scale'))}) and evidence floor "
                f"({_n(rec.get('evidence_scale'))}) allow."]
    if decision == 'over_budget_scale64':
        return [f"No global scale up to 64 fit the budget ({_n(budget)}): this is the "
                f"last assignment VBSP counted and no promotion ran. The map has too "
                f"many brushes for this budget."]
    if decision == 'no_gain':
        fate = _UNIT_FATE.get(rec.get('unit_decision'), 'has no recorded outcome')
        return [f"Rides along with its promotion unit but never moves: a minimum "
                f"scale {_n(rec.get('min_scale'))} (detail type, displacement extent, "
                f"texlight or a floor) holds it at {_n(base)}, at or above baseline "
                f"{_n(baseline)}, so promotion gains it nothing.",
                f"Its unit ({_unit_text(rec, units, basis)}) {fate}."]
    if decision == 'promoted_prefix':
        return [f"Promoted from baseline {_n(base)} to {_n(promoted)}: its unit "
                f"({_unit_text(rec, units, basis)}) is inside the prefix of the promotion "
                f"order that fit the budget.",
                f"Cost: {_cost_text(rec)}."]
    if decision == 'promoted_skip_ahead':
        return [f"Promoted from baseline {_n(base)} to {_n(promoted)} by the "
                f"skip-ahead pass: the prefix stopped earlier at a unit that did not "
                f"fit, and this unit ({_unit_text(rec, units, basis)}) still fit on top.",
                f"Cost: {_cost_text(rec)}."]
    if decision == 'stepped_over':
        if rec.get('stranding'):
            how = ("The promotion prefix ended here: it did not fit on top of the "
                   "units before it. "
                   + ("The skip-ahead pass then carried on after it."
                      if result.get('skip_ahead_cap') != 0 else
                      "The skip-ahead pass is off (--skip-ahead-calls 0)."))
        else:
            how = ("The skip-ahead pass stepped over it: it did not fit on top of "
                   "what was already promoted.")
        limit = rec.get('binding_limit')
        left = (f"~{_n(rec.get('headroom_at_skip'))} of the {_n(budget)} budget "
                f"was left" + (f" (binding: {limit})" if limit else ""))
        over = ""
        if rec.get('trial_worst_count') is not None and budget:
            over = (f" The trial reached {_n(rec['trial_worst_count'])}, "
                    f"{_n(rec['trial_worst_count'] - budget)} over.")
        return [f"Stayed at baseline {_n(base)}: promoting to {_n(promoted)} needed "
                f"~{_n(_added_cells(rec))} more chop cells and did not fit; {left}."
                f"{over}",
                how,
                f"Cost: {_cost_text(rec)}. Its unit is {_unit_text(rec, units, basis)}."]
    if decision == 'not_reached':
        used, cap = result.get('skip_ahead_used'), result.get('skip_ahead_cap')
        why = ("the skip-ahead pass is off (--skip-ahead-calls 0)" if cap == 0 else
               f"the skip-ahead pass ran out of calls ({_n(used)} of {_n(cap)})")
        return [f"Stayed at baseline {_n(base)}: never tried. The prefix stopped "
                f"before its unit ({_unit_text(rec, units, basis)}) and {why}.",
                f"Cost it would have had: {_cost_text(rec)}."]
    return [f"Decision '{decision}' (not known to this script; raw record follows).",
            json.dumps(rec)]


_FLAG_TEXT = {
    'never_visible': 'never visible (read as dark and flat, so it is cheap to coarsen)',
    'raw_never_visible': 'never visible by the raw visibility result',
    'culled': 'culled by VBSP (costs no BSP vertices; floored)',
    'texlight': 'texlight (emissive)',
    'carve_promoted': 'a carved dark piece promoted to its max useful scale',
}


def explain_side(report: dict, key: str) -> List[str]:
    """The lines explaining one side; `key` is its id as a string."""
    rec, run = report['sides'][key], report['run']
    flags = rec.get('flags') or {}
    no_geometry = bool(flags.get('no_geometry'))
    where = rec.get('entity') or 'worldspawn'
    disp = ', displacement' if rec.get('has_displacement') else ''
    size = ('no face extent' if no_geometry else
            f"{rec.get('extent_s', 0):g} x {rec.get('extent_t', 0):g} units")
    lines = [f"side {key}: brush {rec.get('brush_id')} ({where}), "
             f"{rec.get('material')}, {size}{disp}"]
    solver, final = rec.get('solver_scale'), rec.get('final_scale')
    lines.append(f"  scale: input {_n(rec.get('input_scale'))} -> solver "
                 f"{'(not solved)' if solver is None else solver} -> final {final}")
    if no_geometry:
        lines.append("  No face extent: the optimizer built no usable geometry for "
                     "this side (a displacement whose base quad collapsed, or a side "
                     "that degenerated under CSG), so the solver never saw it and "
                     "made no decision. Only the steps below set its scale.")
    else:
        for text in _why(rec, run):
            lines.append(f"  {text}")
    changed = rec.get('changed_by') or []
    after = [stage for stage in changed if stage != 'before_solver']
    if 'before_solver' in changed:
        lines.append(f"  Its scale differs from the input ({_n(rec.get('input_scale'))}"
                     f" -> {final}) by a step before the solver: auto-compile raises "
                     f"floors on the tree up front.")
    if after:
        lines.append(f"  After the solver it was moved "
                     f"{'' if solver is None else f'from {solver} '}to {final} by: "
                     f"{', '.join(after)}.")
    elif solver is not None and solver != final:
        lines.append(f"  It ended at {final}, not the solver's {solver}, by a stage "
                     f"the report did not attribute.")
    if rec.get('floor'):
        kinds = rec.get('floor_kinds') or []
        lines.append(f"  Floor {rec['floor']} ({', '.join(kinds) or 'unspecified'}): "
                     f"no stage may end it below that.")
        if 'disp_no_geometry' in kinds:
            lines.append("  disp_no_geometry: a displacement with no usable polygon has "
                         "an unknown extent, so VBSP's extent limit cannot be checked "
                         "and the conservative minimum applies.")
    notes = [_FLAG_TEXT[name] for name in _FLAG_TEXT if flags.get(name)]
    if no_geometry:
        notes.append('no_geometry (no face extent)')
    if 'carved_from' in flags:
        notes.append(f"carved from side {flags['carved_from']} (input scale is that "
                     f"side's)")
    if notes:
        lines.append(f"  Flags: {'; '.join(notes)}.")
    return lines


def _carved_pieces(report: dict, key: str) -> List[str]:
    return sorted((k for k, rec in report['sides'].items()
                   if str((rec.get('flags') or {}).get('carved_from')) == key),
                  key=int)


def cmd_side(report: dict, args) -> List[str]:
    key = str(args.id)
    if key in report['sides']:
        return explain_side(report, key)
    pieces = _carved_pieces(report, key)
    if pieces:
        return ([f"side {key} no longer exists: --chop carved it into side(s) "
                 f"{', '.join(pieces)}.", ""]
                + [line for p in pieces for line in explain_side(report, p) + [""]])
    raise ReportError(f"side {key} is not in the report. It lists the sides of the "
                      f"OUTPUT VMF that have a face extent, plus sides without one "
                      f"that a floor or a stage set the scale of; a side the "
                      f"optimizer neither solved nor touched, or a wrong id, looks "
                      f"like this.")


def cmd_brush(report: dict, args) -> List[str]:
    keys = sorted((k for k, rec in report['sides'].items()
                   if rec.get('brush_id') == args.id), key=int)
    if not keys:
        raise ReportError(f"brush {args.id} has no sides in the report (a brush "
                          f"--chop carved away has new ids in the output VMF).")
    lines = [f"brush {args.id}: {_plural(len(keys), 'side')}", ""]
    for key in keys:
        lines += explain_side(report, key) + [""]
    return lines


def cmd_summary(report: dict, args) -> List[str]:
    run, sides = report['run'], report['sides']
    result, flags = run.get('result', {}), run.get('args', {})
    lines = [f"report for {run.get('output')} (created {run.get('created')}, "
             f"report version {run.get('report_version')})",
             f"input {run.get('input')}",
             f"mode {run.get('mode')}, solver {run.get('solver')}"
             + (f", bsp {run['bsp']}" if run.get('bsp') else ''),
             "flags: " + ', '.join(f"{k}={v}" for k, v in flags.items())]
    if run.get('solver') == 'vbsp':
        lines += [f"baseline scale {_n(result.get('baseline_scale'))}, budget "
                  f"{_n(result.get('vertex_budget'))}, verts "
                  f"{_n(result.get('vbsp_vertex_count'))}, leaffaces "
                  f"{_n(result.get('vbsp_leafface_count'))}, faces "
                  f"{_n(result.get('vbsp_face_count'))}, binding "
                  f"{result.get('binding_limit')}, {_n(result.get('vbsp_calls'))} "
                  f"VBSP calls",
                  f"promotion: {result.get('promote_order') or '?'} order, "
                  f"{_n(result.get('promotion_units'))} units, prefix "
                  f"{_n(result.get('prefix_units'))}, skip-ahead used "
                  f"{_n(result.get('skip_ahead_used'))} of "
                  f"{_n(result.get('skip_ahead_cap'))} calls"]
    else:
        lines.append("calibration solver: " + ', '.join(
            f"{k}={_n(v)}" for k, v in result.items()))
    lines += ["", f"{len(sides):,} sides in the report", "by decision:"]
    by_decision = Counter(
        rec.get('decision')
        or ('(no geometry, not solved)' if (rec.get('flags') or {}).get('no_geometry')
            else '(none recorded)')
        for rec in sides.values())
    lines += [f"  {n:>7,}  {name}" for name, n in by_decision.most_common()]
    by_scale = Counter(rec.get('final_scale') for rec in sides.values())
    lines.append("by final scale:")
    lines += [f"  {n:>7,}  scale {scale}"
              for scale, n in sorted(by_scale.items(), key=lambda kv: (kv[0] is None, kv[0]))]
    by_stage = Counter(stage for rec in sides.values() for stage in rec.get('changed_by') or ())
    if by_stage:
        lines.append("moved after the solver:")
        lines += [f"  {n:>7,}  {stage}" for stage, n in by_stage.most_common()]
    return lines


def cmd_skipped(report: dict, args) -> List[str]:
    stepped = [(rec.get('rank', 1 << 30), int(key), rec)
               for key, rec in report['sides'].items()
               if rec.get('decision') == 'stepped_over']
    stepped.sort(key=lambda t: (t[0], t[1]))
    result = report['run'].get('result', {})
    not_reached = sum(1 for rec in report['sides'].values()
                      if rec.get('decision') == 'not_reached')
    lines = [f"{_plural(len(stepped), 'side')} stepped over (left at baseline because "
             f"promoting its unit did not fit), best {_rank_basis(result)} rank first:"]
    shown = stepped if args.top == 0 else stepped[:args.top]
    for rank, sid, rec in shown:
        limit = f" binding {rec['binding_limit']}" if rec.get('binding_limit') else ''
        lines.append(
            f"  side {sid}  rank {_n(rank if rank != 1 << 30 else None)}"
            f"/{_n(result.get('promotion_units'))}  "
            f"{_n(rec.get('scale_at_baseline'))} -> {_n(rec.get('scale_if_promoted'))}  "
            f"cost {_n(rec.get('unit_cost'), 1)} (+{_n(_added_cells(rec))} cells this side)  "
            f"headroom {_n(rec.get('headroom_at_skip'))}{limit}"
            f"{'  [prefix ended here]' if rec.get('stranding') else ''}")
    if len(shown) < len(stepped):
        lines.append(f"  ... {len(stepped) - len(shown):,} more (--top 0 shows all)")
    if not_reached:
        lines.append(f"plus {_plural(not_reached, 'side')} not_reached (skip-ahead "
                     f"stopped before their units); `side ID` explains one.")
    return lines


def _nonneg_int(text: str) -> int:
    value = int(text)
    if value < 0:
        # a negative slice would drop the best-ranked rows, not the worst
        raise argparse.ArgumentTypeError(f"must be 0 or more, got {value}")
    return value


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Explain lmoptimizer's per-side lightmapscale decisions from "
                    "its .lmo_report.json.")
    parser.add_argument('report', type=Path, help='the .lmo_report.json beside the output VMF')
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('side', help='explain one side')
    p.add_argument('id', type=int)
    p = sub.add_parser('brush', help='explain every side of a brush')
    p.add_argument('id', type=int)
    sub.add_parser('summary', help='run header, counts per decision and final scale')
    p = sub.add_parser('skipped', help='sides the skip-ahead pass stepped over')
    p.add_argument('--top', type=_nonneg_int, default=20, help='how many to show (default 20, 0 = all)')
    return parser


_COMMANDS = {'side': cmd_side, 'brush': cmd_brush,
             'summary': cmd_summary, 'skipped': cmd_skipped}


def main(argv: Optional[Sequence[str]] = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding='utf-8', errors='replace')
        except (AttributeError, ValueError, OSError):
            pass
    args = _build_parser().parse_args(argv)
    try:
        report = load_report(args.report)
        lines = _COMMANDS[args.command](report, args)
    except ReportError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 1
    print("\n".join(lines).rstrip("\n"))
    return 0


if __name__ == '__main__':
    sys.exit(main())
