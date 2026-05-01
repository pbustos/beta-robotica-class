"""
Validation for Phase 3.3 — N-step planning horizon.

Compares:
  - Rule-based baseline
  - EFE 1-step  (λ=0)
  - EFE 3-step  (λ=0, γ=0.9)
  - EFE 5-step  (λ=0, γ=0.9)

Plots:
  1. Per-episode scores across horizons + rule-based
  2. Mean score vs horizon
  3. EFE per action over a sample episode (best horizon)
  4. Belief trajectory + chosen actions (best horizon)
"""

import time
import pathlib
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter
)
from efe_agent import EFEAgent

gym.register_envs(ale_py)

_STATE_FILE = pathlib.Path("models/agent_state.pkl")
def _load_bias():
    if _STATE_FILE.exists():
        with open(_STATE_FILE, "rb") as f:
            return pickle.load(f).get("landing_bias", 0.0)
    return 0.0

N_EPISODES    = 10
HORIZONS      = [1, 3, 5]
GAMMA         = 0.9
EPIST_WEIGHT  = 0.0          # instrumental only for the horizon sweep
ACTION_NAMES  = {0: "NOOP", 2: "UP", 3: "DOWN"}
ACTION_COLORS = {0: "gray", 2: "steelblue", 3: "darkorange"}

# ── helpers ───────────────────────────────────────────────────────────────────

def make_components():
    ball_model  = BallTransitionVBGS.from_file()
    opp_model   = OpponentTransitionVBGS.from_file()
    likelihood  = LikelihoodModel()
    opp_tracker = OpponentBeliefTracker(opp_model)
    belief      = MixtureBeliefFilter(ball_model, opp_tracker, likelihood)
    prefs       = PriorPreferences()
    agent       = EFEAgent(belief, likelihood, prefs)
    return belief, opp_tracker, agent

def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])

def rule_action(ram):
    py, by = ram[51]/255., ram[54]/255.
    return 3 if py < by - 0.02 else (2 if py > by + 0.02 else 0)

def run_episode(env, horizon=1, record=False, landing_bias=0.0):
    """Run one episode with N-step planning. Returns (score, logs, final_bias)."""
    belief, opp_tracker, agent = make_components()
    agent.landing_bias = landing_bias
    obs, _ = env.reset()
    o = extract_obs(obs)
    belief.initialise(o)

    score       = 0
    prev_opp_y  = o[3]
    prev_action = 0
    logs        = [] if record else None

    while True:
        o            = extract_obs(obs)
        ball_in_play = o[0] > 0.05
        opp_y        = o[3]

        # 1. update belief and raw velocity ────────────────────────────────────
        if ball_in_play:
            agent.update_raw_velocity(o[0], o[1])
            ball_state_4 = belief.mean[:4]
            opp_tracker.update(prev_opp_y, ball_state_4, prev_action, opp_y)
            ll = belief.update(o, prev_action)
        else:
            belief.initialise(o)
            agent.reset_raw_velocity()
            ll = 0.0

        # 2. select action ─────────────────────────────────────────────────────
        if horizon == 1:
            action, G_info = agent.select_action(epistemic_weight=EPIST_WEIGHT)
        else:
            action, G_info = agent.select_action_horizon(
                horizon=horizon, epistemic_weight=EPIST_WEIGHT, gamma=GAMMA)

        if record and ball_in_play:
            logs.append({
                "mu":     belief.mean.copy(),
                "o":      o.copy(),
                "action": action,
                "ll":     ll,
            })

        prev_opp_y  = opp_y
        prev_action = action
        obs, reward, terminated, truncated, _ = env.step(action)
        score += reward

        if reward == -1:
            agent.update_from_miss(o[1], o[2])

        if terminated or truncated:
            break

    return score, logs, agent.landing_bias

def run_rule_episode(env):
    obs, _ = env.reset()
    score  = 0
    while True:
        action = rule_action(obs)
        obs, reward, terminated, truncated, _ = env.step(action)
        score += reward
        if terminated or truncated:
            break
    return score

# ── run episodes ──────────────────────────────────────────────────────────────

env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)

horizon_scores = {}
horizon_times  = {}
for h in HORIZONS:
    label = f"H={h}"
    print(f"Running EFE {label} (3^{h}={3**h} policies/frame) ...")
    scores, t0, bias = [], time.time(), _load_bias()
    for ep in range(N_EPISODES):
        s, _, bias = run_episode(env, horizon=h, landing_bias=bias)
        scores.append(s)
        print(f"  {label} episode {ep+1:2d}: score {s:+.0f}  bias={bias:+.4f}")
    elapsed = time.time() - t0
    horizon_scores[h] = scores
    horizon_times[h]  = elapsed
    print(f"  → {elapsed:.1f}s total ({elapsed/N_EPISODES:.1f}s/ep)")

print("Running rule-based agent ...")
rule_scores = []
for ep in range(N_EPISODES):
    rule_scores.append(run_rule_episode(env))
    print(f"  Rule episode {ep+1:2d}: score {rule_scores[-1]:+.0f}")

# Detailed sample with best horizon
best_h = max(HORIZONS, key=lambda h: np.mean(horizon_scores[h]))
print(f"\nBest horizon = {best_h}  (mean {np.mean(horizon_scores[best_h]):+.1f})")
print(f"Recording sample episode with H={best_h} ...")
_, sample_logs, _ = run_episode(env, horizon=best_h, record=True,
                                landing_bias=bias)

env.close()

# ── plots ─────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(4, 1, figsize=(11, 15))
fig.suptitle(f"EFE Agent — Phase 3.3 (N-step planning, γ={GAMMA}) vs rule-based", fontsize=13)

# 1. per-episode scores
ax = axes[0]
x    = np.arange(1, N_EPISODES + 1)
cols = ["steelblue", "darkorange", "seagreen"]
for i, h in enumerate(HORIZONS):
    sc = horizon_scores[h]
    ax.plot(x, sc, "o-", color=cols[i], label=f"EFE H={h} (mean {np.mean(sc):+.1f})")
ax.plot(x, rule_scores, "s--", color="black",
        label=f"Rule (mean {np.mean(rule_scores):+.1f})")
ax.axhline(0, color="gray", linewidth=0.8, linestyle=":")
ax.set_ylabel("Episode score"); ax.set_xlabel("Episode")
ax.set_title("Score comparison across planning horizons"); ax.legend(); ax.grid(True, alpha=0.3)

# 2. mean score vs horizon
ax = axes[1]
means = [np.mean(horizon_scores[h]) for h in HORIZONS]
bars  = ax.bar([str(h) for h in HORIZONS], means, color=cols[:len(HORIZONS)])
ax.axhline(np.mean(rule_scores), color="black", linestyle="--",
           label=f"Rule mean {np.mean(rule_scores):+.1f}")
for bar, h in zip(bars, HORIZONS):
    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() - 0.4,
            f"{np.mean(horizon_scores[h]):+.1f}", ha="center", va="top",
            color="white", fontsize=9, fontweight="bold")
ax.set_xlabel("Planning horizon N"); ax.set_ylabel("Mean episode score")
ax.set_title(f"Mean score vs horizon  (γ={GAMMA}, λ={EPIST_WEIGHT})")
ax.legend(); ax.grid(True, alpha=0.3, axis="y")

# 3. belief trajectory (best horizon sample)
ax = axes[2]
if sample_logs:
    frames     = np.arange(len(sample_logs))
    mu_arr     = np.array([log["mu"] for log in sample_logs])
    obs_arr    = np.array([log["o"]  for log in sample_logs])
    action_arr = np.array([log["action"] for log in sample_logs])
    ax.plot(frames, obs_arr[:, 0],  color="steelblue", alpha=0.35, label="raw bx")
    ax.plot(frames, mu_arr[:, 0],   color="steelblue", label="μ[bx]")
    ax.plot(frames, obs_arr[:, 2],  color="seagreen",  alpha=0.35, label="raw py")
    ax.plot(frames, mu_arr[:, 4],   color="seagreen",  label="μ[py]")
    ax.plot(frames, obs_arr[:, 1],  color="tomato",    alpha=0.35, label="raw by")
    ax.plot(frames, mu_arr[:, 1],   color="tomato",    label="μ[by]")
    for f, a in enumerate(action_arr):
        ax.axvspan(f, f+1, color=ACTION_COLORS[a], alpha=0.10)
    patches = [mpatches.Patch(color=ACTION_COLORS[a], label=ACTION_NAMES[a], alpha=0.5)
               for a in [0, 2, 3]]
    ax.set_ylabel("Normalised"); ax.set_xlabel("Frame")
    ax.set_title(f"Belief trajectory — sample episode (H={best_h})")
    ax.legend(handles=ax.get_legend_handles_labels()[0] + patches, ncol=4, fontsize=8)
    ax.grid(True, alpha=0.3)

# 4. tracking error: |μ[by] − μ[py]| over episode
ax = axes[3]
if sample_logs:
    gap_belief = np.abs(mu_arr[:, 1] - mu_arr[:, 4])       # |by - py| in belief
    gap_raw    = np.abs(obs_arr[:, 1] - obs_arr[:, 2])      # |by - py| raw RAM
    ax.plot(frames, gap_belief * 255, color="steelblue", label=f"belief gap (mean {np.mean(gap_belief)*255:.1f} px)")
    ax.plot(frames, gap_raw    * 255, color="tomato",   alpha=0.5,
            label=f"raw gap (mean {np.mean(gap_raw)*255:.1f} px)")
    ax.set_ylabel("Tracking gap (pixels)"); ax.set_xlabel("Frame")
    ax.set_title(f"Player tracking gap  |by − py|  (H={best_h})")
    ax.legend(); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("validation_efe.png", dpi=120)
plt.show()
print("\nSaved validation_efe.png")

# ── summary ───────────────────────────────────────────────────────────────────

print("\n── Score summary ──")
for h in HORIZONS:
    sc = horizon_scores[h]
    print(f"EFE H={h}   mean: {np.mean(sc):+.1f}  "
          f"min: {min(sc):+.0f}  max: {max(sc):+.0f}  "
          f"({horizon_times[h]/N_EPISODES:.1f}s/ep)")
print(f"Rule       mean: {np.mean(rule_scores):+.1f}  "
      f"min: {min(rule_scores):+.0f}  max: {max(rule_scores):+.0f}")
