"""
Validation for BeliefFilter (Phase 2.1).

Loads pretrained VBGS models, runs the belief filter for N frames, and plots:
  1. Belief μ vs raw RAM — ball position (smoothing check)
  2. Inferred ball velocity (latent variables recovered)
  3. Belief uncertainty Σ diagonal over time
"""

import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS,
    LikelihoodModel, BeliefFilter
)

gym.register_envs(ale_py)

# ── config ────────────────────────────────────────────────────────────────────

N_FRAMES = 500

# ── load pretrained models ────────────────────────────────────────────────────

ball_model     = BallTransitionVBGS.from_file()
opponent_model = OpponentTransitionVBGS.from_file()
likelihood     = LikelihoodModel()
belief         = BeliefFilter(ball_model, opponent_model, likelihood)

# ── helpers ───────────────────────────────────────────────────────────────────

def extract_obs(ram):
    """RAM → normalised observation o = [bx, by, py, oy]."""
    return np.array([
        ram[49] / 255.0,   # ball_x
        ram[54] / 255.0,   # ball_y
        ram[51] / 255.0,   # player_y
        ram[50] / 255.0,   # opponent_y
    ])

def chase_action(ram):
    py, by = ram[51] / 255.0, ram[54] / 255.0
    return 3 if py < by - 0.02 else (2 if py > by + 0.02 else 0)

# ── run ───────────────────────────────────────────────────────────────────────

env    = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
obs, _ = env.reset()

o = extract_obs(obs)
belief.initialise(o)

raw_bx, raw_by   = [], []
bel_bx, bel_by   = [], []
bel_vx, bel_vy   = [], []
sigma_diag       = []
log_likelihoods  = []

for frame in range(N_FRAMES):
    action = chase_action(obs)
    o      = extract_obs(obs)

    ll = belief.update(o, action)

    raw_bx.append(o[0]);        raw_by.append(o[1])
    bel_bx.append(belief.mu[0]); bel_by.append(belief.mu[1])
    bel_vx.append(belief.mu[2]); bel_vy.append(belief.mu[3])
    sigma_diag.append(np.diag(belief.Sigma).copy())
    log_likelihoods.append(ll)

    obs, _, term, trunc, _ = env.step(action)
    if term or trunc:
        obs, _ = env.reset()
        o = extract_obs(obs)
        belief.initialise(o)

env.close()

# ── plots ─────────────────────────────────────────────────────────────────────

sigma_arr = np.array(sigma_diag)
t         = np.arange(N_FRAMES)

fig, axes = plt.subplots(4, 1, figsize=(11, 11))
fig.suptitle("BeliefFilter — Phase 2.1 validation", fontsize=13)

# 1. ball position: raw vs belief
ax = axes[0]
ax.plot(t, raw_bx,  color="steelblue",  alpha=0.4, label="raw bx")
ax.plot(t, bel_bx,  color="steelblue",  label="belief μ[bx]")
ax.plot(t, raw_by,  color="darkorange", alpha=0.4, label="raw by")
ax.plot(t, bel_by,  color="darkorange", label="belief μ[by]")
ax.fill_between(t,
    np.array(bel_bx) - np.sqrt(sigma_arr[:, 0]),
    np.array(bel_bx) + np.sqrt(sigma_arr[:, 0]),
    color="steelblue", alpha=0.15)
ax.fill_between(t,
    np.array(bel_by) - np.sqrt(sigma_arr[:, 1]),
    np.array(bel_by) + np.sqrt(sigma_arr[:, 1]),
    color="darkorange", alpha=0.15)
ax.set_ylabel("Normalised position")
ax.set_title("Ball position: raw RAM vs belief mean (shaded = ±1σ)")
ax.legend(ncol=2); ax.grid(True, alpha=0.3)

# 2. inferred velocity (latent — not in RAM)
ax = axes[1]
ax.plot(t, bel_vx, color="steelblue",  label="μ[vx]")
ax.plot(t, bel_vy, color="darkorange", label="μ[vy]")
ax.axhline(0, color="gray", linewidth=0.8, linestyle="--")
ax.set_ylabel("Velocity (normalised/frame)")
ax.set_title("Inferred ball velocity (latent state — not directly observed)")
ax.legend(); ax.grid(True, alpha=0.3)

# 3. uncertainty Σ diagonal
ax = axes[2]
labels = ["bx", "by", "vx", "vy", "py", "oy"]
colors = ["steelblue", "darkorange", "seagreen", "crimson", "purple", "brown"]
for i, (lbl, col) in enumerate(zip(labels, colors)):
    ax.plot(t, sigma_arr[:, i], label=f"σ²[{lbl}]", color=col, alpha=0.8)
ax.set_ylabel("Variance")
ax.set_title("Belief uncertainty Σ diagonal")
ax.legend(ncol=3); ax.grid(True, alpha=0.3)
ax.set_yscale("log")

# 4. observation log-likelihood
ax = axes[3]
ax.plot(t, log_likelihoods, color="steelblue", alpha=0.8)
ax.set_ylabel("log p(o)")
ax.set_xlabel("Frame")
ax.set_title("Observation log-likelihood (higher = belief matches RAM)")
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("validation_belief.png", dpi=120)
plt.show()
print("Saved validation_belief.png")

print(f"\nFinal belief μ : {belief.mu.round(4)}")
print(f"Final Σ diag   : {np.diag(belief.Sigma).round(6)}")
print(f"Mean log p(o)  : {np.mean(log_likelihoods):.3f}")
