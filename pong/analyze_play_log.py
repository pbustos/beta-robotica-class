"""
Analyze a play_efe_log_*.csv telemetry log.

Reproduces the analyses used in the no-contact-loss investigation:

  - per-episode score stats + rolling windows
  - contact-event aggregates (count, |err|, tgt_chosen, tgt_pwin)
  - per-50-ep windows: score, contacts, err, pwin, target dispersion
  - won vs lost rallies — rMM signal
  - frame-to-frame target jitter, binned by ball_x
  - rally segmentation (no-contact LOSS/WIN, contact LOSS/WIN)
  - no-contact LOSS deep-dive: paddle-ball gap, action distribution,
    target accuracy across approach phases, target-trajectory drift

Usage:
    python analyze_play_log.py <path-to-log.csv>
    python analyze_play_log.py                  # uses newest play_efe_log_*.csv
"""

import csv
import glob
import os
import sys
from collections import Counter, defaultdict

import numpy as np


# ── CSV iteration ─────────────────────────────────────────────────────────────

def _open(path):
    return csv.DictReader(open(path))


def per_episode_final_score(path):
    """Return list of (episode, final_score)."""
    out = []
    last_ep = None; last_score = None
    for row in _open(path):
        ep = int(row['episode'])
        if ep != last_ep:
            if last_ep is not None:
                out.append((last_ep, last_score))
            last_ep = ep
        last_score = float(row['score'])
    if last_ep is not None:
        out.append((last_ep, last_score))
    return out


def iter_rallies(path):
    """Yield rally dicts {ep, frames, had_contact, outcome, ds}."""
    cur = {'frames': [], 'had_contact': False}
    prev_ep = None; prev_score = 0
    for row in _open(path):
        ep = int(row['episode']); s = float(row['score'])
        if ep != prev_ep:
            cur = {'frames': [], 'had_contact': False, 'ep': ep}
            prev_ep = ep; prev_score = s
            continue
        ds = s - prev_score
        cur['frames'].append(row)
        if row['contact'] in ('1', 'True'):
            cur['had_contact'] = True
        if abs(ds) > 0:
            cur['outcome'] = 'win' if ds > 0 else 'loss'
            cur['ds'] = ds
            yield cur
            cur = {'frames': [], 'had_contact': False, 'ep': ep}
        prev_score = s


# ── analyses ──────────────────────────────────────────────────────────────────

def report_episode_scores(path, windows=(50, 100)):
    eps = per_episode_final_score(path)
    arr = np.array([s for _, s in eps])
    n = len(arr)
    print(f"\n── Episode scores (n={n}) ──")
    print(f"  overall mean={arr.mean():.2f}  median={np.median(arr):.1f}  "
          f"best={arr.max():+.0f}  worst={arr.min():+.0f}")
    print(f"  wins(>0)={(arr>0).sum()}  ties=({(arr==0).sum()})  losses={(arr<0).sum()}")
    for chunk in windows:
        print(f"\n  Window={chunk}:")
        for i in range(0, n, chunk):
            seg = arr[i:i+chunk]
            if len(seg) < chunk//2:
                continue
            print(f"    eps {i+1:3d}-{i+len(seg):3d}: "
                  f"mean={seg.mean():+6.2f}  wins={int((seg>0).sum())}/{len(seg)}  "
                  f"best={seg.max():+.0f}")


def report_contact_aggregates(path):
    contacts = 0
    errs, tgts, pwins = [], [], []
    for row in _open(path):
        if row['contact'] in ('1', 'True'):
            contacts += 1
            try:
                errs.append(float(row['contact_err']))
                tgts.append(float(row['tgt_chosen']))
                pwins.append(float(row['tgt_pwin']))
            except (KeyError, ValueError):
                pass
    if not contacts:
        print("\n── Contacts: none logged ──"); return
    e = np.array(errs); t = np.array(tgts); p = np.array(pwins)
    print(f"\n── Contact aggregates (n={contacts}) ──")
    print(f"  contact_err: mean={e.mean():+.4f}  |err| mean={np.abs(e).mean():.4f}  "
          f"median={np.median(np.abs(e)):.4f}")
    print(f"  tgt_chosen:  mean={t.mean():.3f}  std={t.std():.3f}  "
          f"range=[{t.min():.3f},{t.max():.3f}]")
    print(f"  tgt_pwin:    mean={p.mean():.3f}  std={p.std():.3f}  "
          f"range=[{p.min():.3f},{p.max():.3f}]")


def report_per_window(path, window=50):
    """Per N-episode window: score, contacts, |err|, tgt_pwin, tgt mean/std."""
    ep_data = defaultdict(lambda: {'contacts': 0, 'pwin': [], 'tgt': [],
                                    'err': [], 'last_score': 0})
    for row in _open(path):
        ep = int(row['episode'])
        ep_data[ep]['last_score'] = float(row['score'])
        if row['contact'] in ('1', 'True'):
            ep_data[ep]['contacts'] += 1
            try:
                ep_data[ep]['pwin'].append(float(row['tgt_pwin']))
                ep_data[ep]['tgt'].append(float(row['tgt_chosen']))
                ep_data[ep]['err'].append(float(row['contact_err']))
            except (KeyError, ValueError):
                pass
    eps = sorted(ep_data.keys())
    print(f"\n── Per-{window}-episode windows ──")
    print(f"{'ep_range':<14} {'score':>8} {'#contact':>8} {'|err|':>7} "
          f"{'pwin':>6} {'tgt':>6} {'tgt_std':>7}")
    for i in range(0, len(eps), window):
        seg = eps[i:i+window]
        sc, nc, er, pw, tg = [], [], [], [], []
        for e in seg:
            d = ep_data[e]
            sc.append(d['last_score']); nc.append(d['contacts'])
            er.extend(abs(x) for x in d['err'])
            pw.extend(d['pwin']); tg.extend(d['tgt'])
        print(f"{seg[0]:>3d}-{seg[-1]:<10d} {np.mean(sc):>+8.2f} "
              f"{np.mean(nc):>8.1f} {np.mean(er) if er else 0:>7.4f} "
              f"{np.mean(pw) if pw else 0:>6.3f} "
              f"{np.mean(tg) if tg else 0:>6.3f} "
              f"{np.std(tg) if tg else 0:>7.3f}")


def report_won_vs_lost(path):
    """Compare last-contact features between rallies that ended in win vs loss."""
    won_p, won_t, won_e = [], [], []
    lost_p, lost_t, lost_e = [], [], []
    for rally in iter_rallies(path):
        if not rally['had_contact']:
            continue
        last_p = last_t = last_e = None
        for f in rally['frames']:
            if f['contact'] in ('1', 'True'):
                try:
                    last_p = float(f['tgt_pwin'])
                    last_t = float(f['tgt_chosen'])
                    last_e = abs(float(f['contact_err']))
                except (KeyError, ValueError):
                    pass
        if last_p is None: continue
        if rally['outcome'] == 'win':
            won_p.append(last_p); won_t.append(last_t); won_e.append(last_e)
        else:
            lost_p.append(last_p); lost_t.append(last_t); lost_e.append(last_e)

    def stat(a, name):
        a = np.array(a)
        print(f"  {name}: n={len(a)} mean={a.mean():.4f} "
              f"median={np.median(a):.4f} std={a.std():.4f}")

    print(f"\n── Won vs Lost rallies (last-contact features) ──")
    print("WON:")
    stat(won_p, 'tgt_pwin'); stat(won_t, 'tgt_chosen'); stat(won_e, '|err|')
    print("LOST:")
    stat(lost_p, 'tgt_pwin'); stat(lost_t, 'tgt_chosen'); stat(lost_e, '|err|')
    if won_p and lost_p:
        n = len(won_p) + len(lost_p)
        print(f"\n  Win rate (rallies w/ contact): {len(won_p)}/{n} = "
              f"{len(won_p)/n:.3f}")


def report_target_jitter(path):
    """Frame-to-frame |Δtgt_chosen| within rallies, binned by ball_x."""
    bins = [(0.00, 0.30), (0.30, 0.50), (0.50, 0.70), (0.70, 0.74)]
    counts = [0] * len(bins); jumps = [0] * len(bins); big = [0] * len(bins)
    prev_ep = None; prev_tgt = None; prev_in_play = False
    for row in _open(path):
        ep = int(row['episode'])
        in_play = row['ball_in_play'] in ('1', 'True')
        try:
            tgt = float(row['tgt_chosen']); bx = float(row['ball_x'])
        except (KeyError, ValueError):
            prev_tgt = None; continue
        if not in_play:
            prev_tgt = None; continue
        if prev_ep == ep and prev_tgt is not None and prev_in_play:
            d = abs(tgt - prev_tgt)
            for i, (lo, hi) in enumerate(bins):
                if lo <= bx < hi:
                    counts[i] += 1
                    if d > 0.05: jumps[i] += 1
                    if d > 0.20: big[i] += 1
                    break
        prev_ep = ep; prev_tgt = tgt; prev_in_play = in_play

    print(f"\n── Frame-to-frame target jitter by ball_x ──")
    print(f"{'ball_x':<10} {'frames':>10} {'|Δ|>.05':>10} {'%':>5} "
          f"{'|Δ|>.20':>10} {'%':>5}")
    for (lo, hi), c, j, b in zip(bins, counts, jumps, big):
        if c == 0: continue
        print(f"{lo:.2f}-{hi:.2f}   {c:>10d} {j:>10d} {100*j/c:>4.1f}% "
              f"{b:>10d} {100*b/c:>4.1f}%")


def report_rally_breakdown(path):
    """Counts of {contact?} × {win/loss}."""
    counts = Counter()
    lens = defaultdict(list)
    for rally in iter_rallies(path):
        key = ('contact' if rally['had_contact'] else 'no-contact',
               rally['outcome'].upper())
        counts[key] += 1
        lens[key].append(len(rally['frames']))
    total = sum(counts.values())
    print(f"\n── Rally breakdown (total={total}) ──")
    print(f"{'category':<20} {'n':>7} {'%':>6} {'mean_len':>10} {'med_len':>9}")
    for key in [('contact', 'WIN'), ('contact', 'LOSS'),
                ('no-contact', 'WIN'), ('no-contact', 'LOSS')]:
        n = counts.get(key, 0)
        if n == 0:
            continue
        ll = lens[key]
        print(f"{' '.join(key):<20} {n:>7d} {100*n/total:>5.1f}% "
              f"{np.mean(ll):>10.1f} {np.median(ll):>9.1f}")


def report_no_contact_loss_diag(path):
    """Deep-dive on no-contact-loss rallies."""
    ncl = [r for r in iter_rallies(path)
           if not r['had_contact'] and r['outcome'] == 'loss']
    if not ncl:
        print("\n── No-contact LOSS rallies: none ──")
        return

    gaps, by_cross, py_cross, tgt_cross, actions = [], [], [], [], []
    for rally in ncl:
        cross = None
        for f in rally['frames']:
            try:
                if float(f['ball_x']) > 0.70:
                    cross = f; break
            except (KeyError, ValueError):
                pass
        if cross is None: continue
        try:
            py = float(cross['player_y']); by = float(cross['ball_y'])
            tgt = float(cross['tgt_chosen'])
        except (KeyError, ValueError):
            continue
        gaps.append(by - py); by_cross.append(by); py_cross.append(py)
        tgt_cross.append(tgt); actions.append(int(cross['action']))

    g = np.array(gaps)
    print(f"\n── No-contact LOSS — at moment ball crosses bx>0.70 (n={len(g)}) ──")
    print(f"  signed gap (ball_y − paddle_y): mean={g.mean():+.4f}  "
          f"median={np.median(g):+.4f}")
    print(f"  |gap|: mean={np.abs(g).mean():.4f}  "
          f"frac>0.05={np.mean(np.abs(g)>0.05):.3f}  "
          f"frac>0.10={np.mean(np.abs(g)>0.10):.3f}  "
          f"frac>0.20={np.mean(np.abs(g)>0.20):.3f}")
    print(f"  ball_y at cross: mean={np.mean(by_cross):.3f} std={np.std(by_cross):.3f}")
    print(f"  paddle_y at cross: mean={np.mean(py_cross):.3f} std={np.std(py_cross):.3f}")
    print(f"  tgt_chosen at cross: mean={np.mean(tgt_cross):.3f}")
    print(f"  action at cross (0=NOOP,2=UP,3=DOWN): {dict(Counter(actions))}")

    py = np.array(py_cross); tg = np.array(tgt_cross); by_a = np.array(by_cross)
    print(f"\n  |paddle_y − target_y| (paddle reached its target?): "
          f"mean={np.abs(py-tg).mean():.4f}")
    print(f"  |target_y − ball_y|   (target near actual landing?): "
          f"mean={np.abs(tg-by_a).mean():.4f}")
    print(f"  |paddle_y − ball_y|   (final miss distance):         "
          f"mean={np.abs(py-by_a).mean():.4f}")

    # Target alignment by approach phase
    phase_correct_05, phase_correct_10 = (defaultdict(list), defaultdict(list))
    for rally in ncl:
        terminal_by = None
        for f in rally['frames']:
            try:
                if float(f['ball_x']) > 0.70:
                    terminal_by = float(f['ball_y']); break
            except (KeyError, ValueError):
                pass
        if terminal_by is None: continue
        pairs = []
        for f in rally['frames']:
            try:
                bx = float(f['ball_x']); tgt = float(f['tgt_chosen'])
                if 0.2 <= bx <= 0.74:
                    pairs.append((bx, tgt))
            except (KeyError, ValueError):
                pass
        for thresh in (0.30, 0.45, 0.55, 0.65, 0.70):
            bin_pairs = [(b, t) for b, t in pairs if thresh <= b < thresh + 0.05]
            if not bin_pairs: continue
            ts = np.array([t for _, t in bin_pairs])
            phase_correct_05[thresh].append(np.mean(np.abs(ts - terminal_by) < 0.05))
            phase_correct_10[thresh].append(np.mean(np.abs(ts - terminal_by) < 0.10))

    print(f"\n  Per-phase fraction of frames with target near terminal_by:")
    print(f"    {'phase':<10} {'<0.05':>8} {'<0.10':>8}")
    for thresh in (0.30, 0.45, 0.55, 0.65, 0.70):
        if not phase_correct_05[thresh]: continue
        print(f"    bx≈{thresh:.2f}    "
              f"{np.mean(phase_correct_05[thresh]):>8.3f} "
              f"{np.mean(phase_correct_10[thresh]):>8.3f}")

    # Drift signature: target-trajectory range within each rally
    ever_05 = ever_10 = final_05 = static = 0
    n = 0
    for rally in ncl:
        terminal_by = None
        for f in rally['frames']:
            try:
                if float(f['ball_x']) > 0.70:
                    terminal_by = float(f['ball_y']); break
            except (KeyError, ValueError):
                pass
        if terminal_by is None: continue
        ts = []
        for f in rally['frames']:
            try: ts.append(float(f['tgt_chosen']))
            except (KeyError, ValueError): pass
        if not ts: continue
        ts = np.array(ts); n += 1
        if (ts.min() - 0.05 <= terminal_by <= ts.max() + 0.05): ever_05 += 1
        if (ts.min() - 0.10 <= terminal_by <= ts.max() + 0.10): ever_10 += 1
        if abs(ts[-1] - terminal_by) < 0.05: final_05 += 1
        if ts.std() < 0.02: static += 1
    if n:
        print(f"\n  Target drift across rally (n={n}):")
        print(f"    ever within 0.05 of terminal_by:   {ever_05} ({100*ever_05/n:.1f}%)")
        print(f"    ever within 0.10 of terminal_by:   {ever_10} ({100*ever_10/n:.1f}%)")
        print(f"    final within 0.05 of terminal_by:  {final_05} ({100*final_05/n:.1f}%)")
        print(f"    target essentially static (σ<.02): {static} ({100*static/n:.1f}%)")


# ── entry point ───────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        cands = sorted(glob.glob("play_efe_log_*.csv"),
                       key=lambda p: os.path.getmtime(p))
        if not cands:
            print("No play_efe_log_*.csv in cwd"); sys.exit(1)
        path = cands[-1]
    print(f"Analyzing: {path}  ({os.path.getsize(path)/1e6:.1f} MB)")

    report_episode_scores(path)
    report_contact_aggregates(path)
    report_per_window(path, window=50)
    report_won_vs_lost(path)
    report_target_jitter(path)
    report_rally_breakdown(path)
    report_no_contact_loss_diag(path)


if __name__ == "__main__":
    main()
