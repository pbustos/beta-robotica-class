"""
Investigate the prediction-collapse hypothesis for no-contact-loss rallies.

Hypothesis (from paddle-bias diagnostic):
    Per-key NCL stats showed paddle_y and target_y stuck at ~0.52 across
    every (vy_sign, bounced) bin while actual ball_y at miss varied from
    0.26 to 0.49. That fits `_predict_ball_landing` returning ≈ current_by
    when the velocity estimate has collapsed to ≈ 0.

This script tests it directly: at midcourt (bx≈0.5), compare
    expected vertical travel = |vy| * frames_to_paddle
    actual vertical travel   = |terminal_by - current_by|
broken out by reconstructed active bias key.

If `expected ≪ actual`, the raw EMA velocity is the bottleneck.

Usage:
    python diagnose_velocity_collapse.py [path-to-log.csv]
"""

import csv
import glob
import os
import sys
from collections import defaultdict

import numpy as np


def iter_rallies(path):
    """Yield rallies with each frame annotated with the active bias key."""
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


def collect(path):
    """Per-rally midcourt sample for NCL and contact-WIN, by active key."""
    bins = {'no-contact LOSS': defaultdict(list),
            'contact WIN':     defaultdict(list)}
    for rally in iter_rallies(path):
        if rally['had_contact'] and rally['outcome'] == 'win':
            cat = 'contact WIN'
            ref = 'contact_by'
        elif (not rally['had_contact']) and rally['outcome'] == 'loss':
            cat = 'no-contact LOSS'
            ref = 'terminal_by'
        else:
            continue

        # Pick the reference ball_y: contact frame for wins, first bx>0.70 for NCL
        ref_by = None
        if ref == 'contact_by':
            for f in rally['frames']:
                if f['contact'] in ('1', 'True'):
                    try: ref_by = float(f['ball_y'])
                    except (KeyError, ValueError): pass
        else:
            for f in rally['frames']:
                try:
                    if float(f['ball_x']) > 0.70:
                        ref_by = float(f['ball_y']); break
                except (KeyError, ValueError):
                    pass
        if ref_by is None: continue

        # Sample at first frame with vx>0 and bx in [0.45, 0.55] (midcourt)
        for f in rally['frames']:
            try:
                bx = float(f['ball_x']); vx = float(f['ball_vx'])
                if 0.45 <= bx <= 0.55 and vx > 1e-3:
                    by  = float(f['ball_y']); vy = float(f['ball_vy'])
                    tgt = float(f['target_y'])    # _last_landing_pred (paddle target)
                    frames_to_paddle = (0.74 - bx) / max(vx, 1e-3)
                    # Signed naive linear projection w/ single wall reflection
                    naive = by + vy * frames_to_paddle
                    if naive < 0: naive = -naive
                    elif naive > 1: naive = 2 - naive
                    actual_signed = ref_by - by      # >0 if ball moves DOWN (toward higher y)
                    expected_signed = vy * frames_to_paddle  # signed: +vy = moving down
                    sign_match = (actual_signed * expected_signed) > 0
                    bins[cat][f['_active_key']].append({
                        'vx': vx, 'vy': vy, 'by_now': by, 'by_ref': ref_by,
                        'pred_by_target': tgt,
                        'pred_by_naive':  naive,
                        'actual_signed': actual_signed,
                        'expected_signed': expected_signed,
                        'sign_match': sign_match,
                        'frames_to_paddle': frames_to_paddle,
                        'pred_err_target': abs(tgt - ref_by),
                        'pred_err_naive':  abs(naive - ref_by),
                    })
                    break
            except (KeyError, ValueError):
                pass
    return bins


def report(bins):
    print("\n── Midcourt (bx≈0.5) signed-velocity diagnostic ──")
    print("expected_signed = vy * frames_to_paddle  (signed; +vy = moving down)")
    print("actual_signed   = ref_by - current_by    (signed; >0 = ball moves down)")
    print("sign_match      = signs of expected and actual agree (=correct vy sign)")
    print("err_naive       = |by + vy*frames - ref_by|  (one-bounce linear projection)")
    print("err_target      = |target_y       - ref_by|  (agent's actual paddle target)\n")

    for cat in ('no-contact LOSS', 'contact WIN'):
        print(f"  --- {cat} ---")
        print(f"  {'key':<14} {'n':>5} {'<|vy|>':>7} {'<exp_signed>':>13} "
              f"{'<act_signed>':>13} {'sign_ok':>8} {'err_naive':>10} {'err_target':>11}")
        for k in sorted(bins[cat].keys()):
            recs = bins[cat][k]
            if len(recs) < 5: continue
            vy   = np.array([abs(r['vy']) for r in recs])
            exs  = np.array([r['expected_signed'] for r in recs])
            acs  = np.array([r['actual_signed']   for r in recs])
            sm   = np.array([r['sign_match']      for r in recs])
            en   = np.array([r['pred_err_naive']  for r in recs])
            et   = np.array([r['pred_err_target'] for r in recs])
            print(f"  {str(k):<14} {len(recs):>5d} "
                  f"{vy.mean():>7.4f} {exs.mean():>+13.4f} {acs.mean():>+13.4f} "
                  f"{sm.mean():>7.3f} {en.mean():>10.4f} {et.mean():>11.4f}")
        print()


def report_collapse_test(bins):
    """Distribution of |vy| at midcourt — how often does it collapse to ≈0?"""
    print("── Velocity collapse distribution: midcourt |vy| histogram ──\n")
    print(f"  {'category':<28} {'n':>5}  "
          f"{'|vy|<.005':>10} {'|vy|<.01':>10} {'|vy|<.02':>10} {'|vy|<.04':>10}")
    for cat in ('no-contact LOSS', 'contact WIN'):
        for k in sorted(bins[cat].keys()):
            recs = bins[cat][k]
            if len(recs) < 5: continue
            vy = np.array([abs(r['vy']) for r in recs])
            n = len(vy)
            print(f"  {cat:>16}  {str(k):<10} {n:>5d}  "
                  f"{(vy<0.005).mean():>9.3f} "
                  f"{(vy<0.010).mean():>9.3f} "
                  f"{(vy<0.020).mean():>9.3f} "
                  f"{(vy<0.040).mean():>9.3f}")


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        cands = sorted(glob.glob("play_efe_log_*.csv"),
                       key=lambda p: os.path.getmtime(p))
        if not cands: print("No CSV found"); sys.exit(1)
        path = cands[-1]
    print(f"Analyzing: {path} ({os.path.getsize(path)/1e6:.1f} MB)")
    bins = collect(path)
    report(bins)
    report_collapse_test(bins)


if __name__ == "__main__":
    main()
