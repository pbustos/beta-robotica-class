"""
α sweep for _LATE_BOUNCE_ALPHA.  Runs three single-arm 100-ep benches
(baseline is deterministic, recorded once).  Re-uses run_arm() from
bench_lock_slack.py to keep the comparison identical.

Existing data points (from prior paired benches):
    α = 0.5: mean +0.57  wins 50  contacts/ep 59.2  NCL/ep 7.77
    α = 0.8: mean -0.03  wins 50  contacts/ep 63.5  NCL/ep 7.63
    baseline: mean +0.19  wins 52  contacts/ep 61.3  NCL/ep 8.02

Adds: α ∈ {0.3, 0.4, 0.6}.

Usage:
    python sweep_alpha.py
"""
import sys, time
import numpy as np
import bench_lock_slack as B

ALPHAS = [0.3, 0.4, 0.6]
PKL    = "models/agent_state.target_y.pkl"

print(f"Sweeping LATE_BOUNCE_ALPHA over {ALPHAS}, {B.N_EPISODES} eps each.\n")

results = {}
for alpha in ALPHAS:
    scores, contacts, ncls = B.run_arm(
        f"α={alpha}", "target_y", PKL,
        late_bounce=True, late_bounce_alpha=alpha)
    results[alpha] = (scores, contacts, ncls)

print("\n── α-sweep summary ──")
print(f"{'α':>5}  {'mean':>8}  {'std':>6}  {'wins':>5}  "
      f"{'best':>5}  {'worst':>5}  {'contacts/ep':>12}  {'NCL/ep':>7}")
# Reference rows
print(f"{'base':>5}  {'+0.19':>8}  {'5.57':>6}  {52:>5}  "
      f"{'+12':>5}  {'-14':>5}  {61.3:>12.1f}  {8.02:>7.2f}   (deterministic)")
print(f"{0.5:>5.2f}  {'+0.57':>8}  {'6.64':>6}  {50:>5}  "
      f"{'+14':>5}  {'-13':>5}  {59.2:>12.1f}  {7.77:>7.2f}   (prior run)")
for alpha in ALPHAS:
    s, c, n = results[alpha]
    s = np.array(s)
    print(f"{alpha:>5.2f}  {s.mean():>+8.2f}  {s.std():>6.2f}  "
          f"{int((s>0).sum()):>5d}  {s.max():>+5.0f}  {s.min():>+5.0f}  "
          f"{np.mean(c):>12.1f}  {np.mean(n):>7.2f}")
print(f"{0.8:>5.2f}  {'-0.03':>8}  {'5.67':>6}  {50:>5}  "
      f"{'+12':>5}  {'-12':>5}  {63.5:>12.1f}  {7.63:>7.2f}   (prior run)")
