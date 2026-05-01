"""
Validation for MixtureBeliefFilter (Phase 2.2).

Key check: does bounce_probability rise as ball approaches top/bottom walls?

Plots:
  1. Ball y position vs bounce probability over time
  2. Component means divergence near walls
  3. Log marginal likelihood vs Phase 2.1 single-Gaussian
"""

import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS,
    LikelihoodModel, BeliefFilter, MixtureBeliefFilter
)

gym.register_envs(ale_py)

N_FRAMES   = 600
WALL_ZONE  = 0.15    # ball_y threshold for "near wall"

# ── load models ───────────────────────────────────────────────────────────────

ball_model     = BallTransitionVBGS.from_file()
opponent_model = OpponentTransitionVBGS.from_file()
likelihood     = LikelihoodModel()

single = BeliefFilter(ball_model, opponent_model, likelihood)
mixture = MixtureBeliefFilter(ball_model, opponent_model, likelihood)

# ── helpers ───────────────────────────────────────────────────────────────────

def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])

def chase_action(ram):
    py, by = ram[51]/255., ram[54]/255.
    return 3 if py < by - 0.02 else (2 if py > by + 0.02 else 0)

# ── run ───────────────────────────────────────────────────────────────────────

env    = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
obs, _ = env.reset()
o      = extract_obs(obs)
single.initialise(o)
mixture.initialise(o)

raw_by       = []
bounce_prob  = []
mu0_vy       = []   # vy in component 0 (free flight)
mu1_vy       = []   # vy in component 1 (wall bounce — should flip near wall)
ll_single    = []
ll_mixture   = []

for frame in range(N_FRAMES):
    action = chase_action(obs)
    o      = extract_obs(obs)

    ll_s = single.update(o, action)
    ll_m = mixture.update(o, action)

    raw_by.append(o[1])
    bounce_prob.append(mixture.bounce_probability)
    mu0_vy.append(mixture.mu[0, 3])   # vy of free-flight component
    mu1_vy.append(mixture.mu[1, 3])   # vy of wall-bounce component
    ll_single.append(ll_s)
    ll_mixture.append(ll_m)

    obs, _, term, trunc, _ = env.step(action)
    if term or trunc:
        obs, _ = env.reset()
        o = extract_obs(obs)
        single.initialise(o)
        mixture.initialise(o)

env.close()

# ── plots ─────────────────────────────────────────────────────────────────────

t           = np.arange(N_FRAMES)
raw_by      = np.array(raw_by)
bounce_prob = np.array(bounce_prob)
near_wall   = (raw_by < WALL_ZONE) | (raw_by > 1 - WALL_ZONE)

fig, axes = plt.subplots(3, 1, figsize=(11, 10))
fig.suptitle("MixtureBeliefFilter — Phase 2.2 validation", fontsize=13)

# 1. ball_y vs bounce probability
ax = axes[0]
ax2 = ax.twinx()
ax.plot(t, raw_by, color="steelblue", alpha=0.7, label="ball_y (RAM)")
ax.axhline(WALL_ZONE,       color="gray", linestyle="--", linewidth=0.8)
ax.axhline(1 - WALL_ZONE,   color="gray", linestyle="--", linewidth=0.8)
ax.fill_between(t, 0, 1, where=near_wall, alpha=0.08, color="red", label="near wall")
ax2.plot(t, bounce_prob, color="crimson", alpha=0.85, label="bounce prob")
ax2.set_ylim(0, 1)
ax.set_ylabel("ball_y (normalised)");  ax2.set_ylabel("P(wall bounce)")
ax.set_title("Ball y position and wall-bounce probability")
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc="upper right")
ax.grid(True, alpha=0.3)

# 2. vy divergence between components
ax = axes[1]
ax.plot(t, mu0_vy, color="steelblue",  label="μ_vy  component 0 (free flight)")
ax.plot(t, mu1_vy, color="darkorange", label="μ_vy  component 1 (wall bounce)")
ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax.fill_between(t, -0.1, 0.1, where=near_wall, alpha=0.08, color="red")
ax.set_ylabel("Inferred vy (normalised/frame)")
ax.set_title("Component vy divergence — should separate near walls")
ax.legend(); ax.grid(True, alpha=0.3)

# 3. log-likelihood comparison
ax = axes[2]
ax.plot(t, ll_single,  color="steelblue",  alpha=0.7, label="Single Gaussian (Phase 2.1)")
ax.plot(t, ll_mixture, color="darkorange", alpha=0.7, label="Mixture (Phase 2.2)")
ax.set_ylabel("log p(o)");  ax.set_xlabel("Frame")
ax.set_title("Observation log-likelihood — mixture should be ≥ single")
ax.legend(); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("validation_mixture_belief.png", dpi=120)
plt.show()
print("Saved validation_mixture_belief.png")

# ── summary ───────────────────────────────────────────────────────────────────

near  = near_wall.sum()
far   = (~near_wall).sum()
print(f"\nFrames near wall : {near}  ({100*near/N_FRAMES:.1f}%)")
print(f"Mean bounce prob near wall : {bounce_prob[near_wall].mean():.3f}")
print(f"Mean bounce prob far  wall : {bounce_prob[~near_wall].mean():.3f}")
print(f"\nMean log p(o)  single  : {np.mean(ll_single):.2f}")
print(f"Mean log p(o)  mixture : {np.mean(ll_mixture):.2f}")
