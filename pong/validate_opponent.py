"""
Validation for OpponentBeliefTracker (Phase 2.3).

Checks:
  1. Mode probabilities vs opponent movement speed — tracking mode should
     dominate when the opponent moves fast (chasing the ball).
  2. N-step trajectory prediction accuracy (1, 3, 5 steps ahead).
  3. Opponent gap signal — does the opponent consistently lag behind the ball?
"""

import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS,
    LikelihoodModel, OpponentBeliefTracker, MixtureBeliefFilter
)

gym.register_envs(ale_py)

N_FRAMES   = 800
HORIZONS   = [1, 3, 5]   # prediction steps for rollout accuracy

# ── load models ───────────────────────────────────────────────────────────────

ball_model     = BallTransitionVBGS.from_file()
opp_model      = OpponentTransitionVBGS.from_file()
likelihood     = LikelihoodModel()
opp_tracker    = OpponentBeliefTracker(opp_model)
belief         = MixtureBeliefFilter(ball_model, opp_tracker, likelihood)

# ── helpers ───────────────────────────────────────────────────────────────────

def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])

def chase_action(ram):
    py, by = ram[51]/255., ram[54]/255.
    return 3 if py < by - 0.02 else (2 if py > by + 0.02 else 0)

# ── data collection ───────────────────────────────────────────────────────────

env    = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
obs, _ = env.reset()
o      = extract_obs(obs)
belief.initialise(o)

# ring buffer for N-step prediction: store (ball_state, action, opp_y)
HORIZON_MAX = max(HORIZONS)
history     = []   # [(ball_state_4, action, opp_y)]

opp_y_log        = []
opp_delta_log    = []    # |Δ opp_y| per frame
mode_prob_log    = []    # [P(tracking), P(lag)]
gap_log          = []    # opponent_gap
pred_errors      = {h: [] for h in HORIZONS}

prev_opp_y = o[3]

for frame in range(N_FRAMES):
    action = chase_action(obs)
    o      = extract_obs(obs)
    opp_y  = o[3]
    ball_state_4 = belief.mean[:4]

    # N-step prediction errors: compare h-step-old prediction to now
    history.append((ball_state_4.copy(), action, opp_y))
    for h in HORIZONS:
        if len(history) > h:
            past_opp_y, past_ball, past_actions = history[-h-1][2], [], []
            for i in range(-h-1, -1):
                past_ball.append(history[i][0])
                past_actions.append(history[i][1])
            traj = opp_tracker.predict_trajectory(past_opp_y, past_ball, past_actions)
            pred_errors[h].append(abs(traj[-1] - opp_y))

    # belief + tracker update
    opp_tracker.update(prev_opp_y, ball_state_4, action, opp_y)
    belief.update(o, action)

    opp_y_log.append(opp_y)
    opp_delta_log.append(abs(opp_y - prev_opp_y))
    mode_prob_log.append(opp_tracker.mode_probabilities())
    gap_log.append(opp_tracker.opponent_gap(opp_y, o[1]))

    prev_opp_y = opp_y

    obs, _, term, trunc, _ = env.step(action)
    if term or trunc:
        obs, _ = env.reset()
        o = extract_obs(obs)
        belief.initialise(o)
        prev_opp_y = o[3]
        history.clear()

env.close()

# ── plots ─────────────────────────────────────────────────────────────────────

mode_arr  = np.array(mode_prob_log)
opp_delta = np.array(opp_delta_log)
gap_arr   = np.array(gap_log)
t         = np.arange(N_FRAMES)

fig, axes = plt.subplots(4, 1, figsize=(11, 12))
fig.suptitle("OpponentBeliefTracker — Phase 2.3 validation", fontsize=13)

# 1. opponent movement vs mode probabilities
ax  = axes[0]
ax2 = ax.twinx()
ax.plot(t, opp_delta,      color="steelblue",  alpha=0.6, label="|Δ opp_y|")
ax2.plot(t, mode_arr[:, 0], color="crimson",    alpha=0.85, label="P(tracking)")
ax2.plot(t, mode_arr[:, 1], color="darkorange", alpha=0.85, label="P(lag)", linestyle="--")
ax2.set_ylim(0, 1)
ax.set_ylabel("|Δ opp_y|");  ax2.set_ylabel("Mode probability")
ax.set_title("Opponent movement speed vs mode belief")
lines1, l1 = ax.get_legend_handles_labels()
lines2, l2 = ax2.get_legend_handles_labels()
ax.legend(lines1+lines2, l1+l2, loc="upper right"); ax.grid(True, alpha=0.3)

# 2. N-step prediction MAE
ax = axes[1]
colors = ["steelblue", "darkorange", "seagreen"]
for h, col in zip(HORIZONS, colors):
    errs = np.array(pred_errors[h])
    ax.plot(errs, color=col, alpha=0.6, label=f"{h}-step MAE (mean={errs.mean():.4f})")
ax.set_ylabel("|predicted - actual| opp_y")
ax.set_title("N-step opponent trajectory prediction error")
ax.legend(); ax.grid(True, alpha=0.3)

# 3. opponent gap (opp_y - ball_y)
ax = axes[2]
ax.plot(t, gap_arr, color="seagreen", alpha=0.7, label="opp_y - ball_y")
ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax.fill_between(t, gap_arr, 0, where=gap_arr > 0, alpha=0.2, color="crimson",
                label="above ball (ball goes past below)")
ax.fill_between(t, gap_arr, 0, where=gap_arr < 0, alpha=0.2, color="steelblue",
                label="below ball (ball goes past above)")
ax.set_ylabel("Gap (normalised)"); ax.set_title("Opponent positioning error (exploitable lag)")
ax.legend(); ax.grid(True, alpha=0.3)

# 4. mode scatter: |Δ opp_y| vs P(tracking)
ax = axes[3]
sc = ax.scatter(opp_delta, mode_arr[:, 0], alpha=0.3, s=8,
                c=np.arange(N_FRAMES), cmap="viridis")
ax.set_xlabel("|Δ opp_y| (movement speed)"); ax.set_ylabel("P(tracking)")
ax.set_title("Mode belief vs opponent speed — should correlate positively")
plt.colorbar(sc, ax=ax, label="frame")
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("validation_opponent.png", dpi=120)
plt.show()
print("Saved validation_opponent.png")

# ── summary ───────────────────────────────────────────────────────────────────

print(f"\nMean P(tracking)    : {mode_arr[:, 0].mean():.3f}")
print(f"Mean P(lag)         : {mode_arr[:, 1].mean():.3f}")
print(f"Mean |opponent gap| : {np.abs(gap_arr).mean():.4f}  "
      f"({np.abs(gap_arr).mean()*255:.1f} px)")
print(f"Gap bias (mean)     : {gap_arr.mean():.4f}  "
      f"({'above' if gap_arr.mean()>0 else 'below'} ball on average)")
for h in HORIZONS:
    errs = np.array(pred_errors[h])
    print(f"{h}-step MAE          : {errs.mean():.4f}  ({errs.mean()*255:.1f} px)")
_, dominant = opp_tracker.dominant_mode()
print(f"\nFinal dominant mode : {dominant}")
