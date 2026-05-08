"""
Diagnose the systematic paddle-low miss bias.

Earlier analysis found that no-contact-loss rallies have mean signed gap
(ball_y − paddle_y) = −0.143 — i.e. the ball ends ABOVE the paddle. This
script breaks that down by the (vy_sign, bounced) bias key and cross-
references the saved aim-bias state in models/agent_state.pkl.

Reports:
  1. Current aim-bias state per key:
     - _bias_dict (effective bias used by the agent)
     - long_term Bayesian aim_bias = sum_err/n_aim
     - aim_trust = min(n/80, 0.85)
  2. Per-(vy_sign, bounced) miss/contact stats from the CSV.
  3. Flags: keys where the miss direction is inconsistent with the bias.

Usage:
    python diagnose_paddle_bias.py [path-to-log.csv]
"""

import csv
import glob
import os
import pickle
import sys
from collections import defaultdict

import numpy as np


STATE_FILE = "models/agent_state.pkl"


def load_state():
    if not os.path.exists(STATE_FILE):
        return {}
    with open(STATE_FILE, "rb") as f:
        return pickle.load(f)


def parse_key(raw):
    """Bias dict keys may be tuples or stringified tuples after pickle round-trip."""
    if isinstance(raw, tuple):
        return raw
    if isinstance(raw, str):
        return eval(raw)  # e.g. "(-1, 0)"
    return raw


def report_bias_state(state):
    print("\n── Stored bias state (models/agent_state.pkl) ──")
    bd = state.get("bias_dict", {})
    lt = state.get("long_term", {})
    n_aim = lt.get("n_aim_k", {})
    sum_err = lt.get("sum_err_k", {})

    if not bd and not n_aim:
        print("  no bias data persisted")
        return

    # Collect all keys seen across the three sources
    keys = set()
    for src in (bd, n_aim, sum_err):
        for k in src.keys():
            keys.add(parse_key(k))
    keys = sorted(keys)

    print(f"{'(vy_sign, bounced)':<22} {'_bias_dict':>12} {'aim_bias':>10} "
          f"{'aim_trust':>10} {'n_aim':>8}")
    for k in keys:
        # bias_dict may be keyed by tuple or str — try both
        bias = bd.get(k, bd.get(str(k), 0.0))
        n = n_aim.get(k, n_aim.get(str(k), 0))
        s = sum_err.get(k, sum_err.get(str(k), 0.0))
        ab = (s / n) if n else 0.0
        trust = min(n / 80.0, 0.85)
        print(f"{str(k):<22} {bias:>+12.4f} {ab:>+10.4f} {trust:>10.3f} {n:>8.0f}")


def iter_rallies(path):
    """Yield rallies, annotating each frame with the *active* bias key.

    The CSV `vy_sign`/`bounced` columns are reset to (0,0) every frame after
    logging — they are non-zero only on contact frames. Reconstruct the
    persistent agent state by carrying the most-recent contact's key forward.
    """
    cur = {'frames': [], 'had_contact': False}
    prev_ep = None; prev_score = 0
    active_key = (0, 0)
    for row in csv.DictReader(open(path)):
        ep = int(row['episode']); s = float(row['score'])
        if ep != prev_ep:
            cur = {'frames': [], 'had_contact': False, 'ep': ep}
            active_key = (0, 0)
            prev_ep = ep; prev_score = s
            continue
        ds = s - prev_score
        # Update active key BEFORE attaching to the row, so the contact frame
        # itself reads the just-updated key.
        if row['contact'] in ('1', 'True'):
            try:
                active_key = (int(float(row['vy_sign'])), int(float(row['bounced'])))
            except (KeyError, ValueError):
                pass
            cur['had_contact'] = True
        row['_active_key'] = active_key
        cur['frames'].append(row)
        if abs(ds) > 0:
            cur['outcome'] = 'win' if ds > 0 else 'loss'
            yield cur
            cur = {'frames': [], 'had_contact': False, 'ep': ep}
        prev_score = s


def report_per_key_misses(path):
    """For each (vy_sign, bounced) at the moment-of-cross / contact, aggregate
       signed gap, ball_y, paddle_y, target_y."""
    bins = {
        'no-contact LOSS': defaultdict(list),
        'contact WIN':     defaultdict(list),
        'contact LOSS':    defaultdict(list),
    }

    for rally in iter_rallies(path):
        cat = ('contact' if rally['had_contact'] else 'no-contact') + ' ' + rally['outcome'].upper()
        if cat not in bins: continue
        # Pick the diagnostic frame:
        #  - no-contact: first frame with bx>0.70 (cross moment)
        #  - contact:    last contact frame
        f = None
        if rally['had_contact']:
            for fr in rally['frames']:
                if fr['contact'] in ('1', 'True'):
                    f = fr
        else:
            for fr in rally['frames']:
                try:
                    if float(fr['ball_x']) > 0.70:
                        f = fr; break
                except (KeyError, ValueError):
                    pass
        if f is None: continue
        try:
            key    = f['_active_key']
            by     = float(f['ball_y'])
            py     = float(f['player_y'])
            tgt    = float(f['tgt_chosen'])
        except (KeyError, ValueError):
            continue
        bins[cat][key].append((by, py, tgt))

    print("\n── Miss/contact stats by (vy_sign, bounced) ──")
    print("(positive signed_gap = ball BELOW paddle; negative = ball ABOVE paddle)\n")
    for cat in ('no-contact LOSS', 'contact WIN', 'contact LOSS'):
        print(f"  --- {cat} ---")
        print(f"  {'key':<14} {'n':>6} {'<by-py>':>10} {'<by>':>7} "
              f"{'<py>':>7} {'<tgt>':>7} {'frac<-0.10':>11} {'frac>+0.10':>11}")
        for k in sorted(bins[cat].keys()):
            data = np.array(bins[cat][k])
            if not len(data): continue
            by, py, tg = data.T
            gap = by - py
            print(f"  {str(k):<14} {len(gap):>6d} "
                  f"{gap.mean():>+10.4f} {by.mean():>7.3f} {py.mean():>7.3f} "
                  f"{tg.mean():>7.3f} {(gap<-0.10).mean():>10.3f} "
                  f"{(gap>+0.10).mean():>10.3f}")
        print()
    return bins


def report_diagnosis(state, bins):
    """Cross-check: for each key, is the stored bias correcting the right direction?

    target = raw_target + bias.  If actual paddle ends up too LOW (signed gap < 0:
    ball above paddle), the agent should move target UP, i.e. bias < 0.
    """
    print("── Cross-check: stored bias vs observed miss direction ──\n")

    bd = state.get("bias_dict", {})
    lt = state.get("long_term", {})
    n_aim = lt.get("n_aim_k", {})
    sum_err = lt.get("sum_err_k", {})

    keys = set()
    for src in (bd, n_aim, sum_err):
        for k in src.keys():
            keys.add(parse_key(k))

    print(f"{'key':<14} {'bias':>10} {'aim_bias':>10} {'<gap>NCL':>10} "
          f"{'NCL_n':>6} {'flag':<40}")
    for k in sorted(keys):
        bias = bd.get(k, bd.get(str(k), 0.0))
        n = n_aim.get(k, n_aim.get(str(k), 0))
        s = sum_err.get(k, sum_err.get(str(k), 0.0))
        ab = (s / n) if n else 0.0
        ncl = bins['no-contact LOSS'].get(k, [])
        if ncl:
            gap_arr = np.array([by - py for by, py, _ in ncl])
            gap = gap_arr.mean()
            ncl_n = len(gap_arr)
        else:
            gap = float('nan'); ncl_n = 0

        flag = ""
        # Signed conventions: if NCL gap < 0 → ball ABOVE paddle → need bias < 0.
        # If bias has wrong sign, flag.
        if ncl_n >= 30 and not np.isnan(gap):
            if abs(gap) > 0.10 and abs(bias) < 0.5 * abs(gap):
                flag = "under-corrected"
            if (gap < -0.05 and bias > 0.02) or (gap > 0.05 and bias < -0.02):
                flag = "WRONG-SIGN bias"
            if abs(gap) > 0.05 and n < 40:
                flag = (flag + ", " if flag else "") + "low-trust"
        print(f"{str(k):<14} {bias:>+10.4f} {ab:>+10.4f} "
              f"{gap:>+10.4f} {ncl_n:>6d} {flag:<40}")


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        cands = sorted(glob.glob("play_efe_log_*.csv"),
                       key=lambda p: os.path.getmtime(p))
        if not cands:
            print("No play_efe_log_*.csv in cwd"); sys.exit(1)
        path = cands[-1]
    print(f"Analyzing CSV: {path} ({os.path.getsize(path)/1e6:.1f} MB)")

    state = load_state()
    report_bias_state(state)
    bins = report_per_key_misses(path)
    report_diagnosis(state, bins)


if __name__ == "__main__":
    main()
