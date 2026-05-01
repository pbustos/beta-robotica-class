"""
Phase 4 — Online Learning.

4.1  Online R estimation: batch EMA on Kalman innovations (every R_BATCH frames).
4.2  Online ball VBGS refinement: one CAVI step per frame on live (s_t, s_{t+1}).
4.3  Opponent model: already updated every frame by OpponentBeliefTracker.update().

Key difference from validate_efe.py:
  - Frozen baseline: fresh model load each episode (matches Phase 3.3 exactly).
  - Online condition: models shared and updated across all episodes.
  - predict() and correct() called separately so innovation is accessible.

Usage:
    python train_efe.py
"""

import time
import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter
)
from efe_agent import EFEAgent

gym.register_envs(ale_py)

N_EPISODES = 30
HORIZON    = 3
GAMMA      = 0.9
R_BATCH    = 20      # frames accumulated before updating R
R_LR       = 0.005   # EMA learning rate for R (half of default to stay stable)


# ── helpers ───────────────────────────────────────────────────────────────────

def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])

def make_models():
    """Load fresh pretrained models from disk."""
    return (BallTransitionVBGS.from_file(),
            OpponentTransitionVBGS.from_file(),
            LikelihoodModel())

def make_episode_state(ball_model, opp_model, likelihood):
    """
    Create per-episode belief + agent that reference the shared models.
    A new OpponentBeliefTracker resets responsibility history but the
    underlying opp_model VBGS continues to accumulate data.
    """
    opp_tracker = OpponentBeliefTracker(opp_model)
    belief      = MixtureBeliefFilter(ball_model, opp_tracker, likelihood)
    prefs       = PriorPreferences()
    agent       = EFEAgent(belief, likelihood, prefs)
    return opp_tracker, belief, agent


def run_episode(env, ball_model, opp_model, likelihood, learn=True, landing_bias=0.0):
    """
    Run one episode.

    learn=True:
      4.2  ball_model.update(s_t, s_{t+1}) every in-play frame.
      4.1  likelihood.update_R(innovations) every R_BATCH frames.
    learn=False:
      Matches Phase 3.3 validate_efe.py behaviour exactly.
    """
    opp_tracker, belief, agent = make_episode_state(ball_model, opp_model, likelihood)
    agent.landing_bias = landing_bias
    obs, _ = env.reset()
    o = extract_obs(obs)
    belief.initialise(o)

    score           = 0
    prev_opp_y      = o[3]
    prev_action     = 0
    prev_ball_state = None   # post-correct ball state from previous in-play frame
    innovations     = []

    while True:
        o            = extract_obs(obs)
        ball_in_play = o[0] > 0.05
        opp_y        = o[3]

        if ball_in_play:
            agent.update_raw_velocity(o[0], o[1])

            # Capture pre-predict state (matches original opp_tracker ordering)
            ball_state_4 = belief.mean[:4].copy()

            # Separate predict / correct — exposes the prediction innovation
            belief.predict(prev_action)
            inn = o - likelihood.H @ belief.mean   # predicted obs → actual obs
            belief.correct(o)

            # Opponent tracker update (also does online VBGS fit internally)
            opp_tracker.update(prev_opp_y, ball_state_4, prev_action, opp_y)

            if learn:
                # 4.2: ball VBGS one CAVI step on the just-observed transition
                curr_ball = belief.mean[:4].copy()
                if prev_ball_state is not None:
                    ball_model.update(prev_ball_state, curr_ball)
                prev_ball_state = curr_ball

                # 4.1: accumulate innovations, flush to R update in batches
                innovations.append(inn)
                if len(innovations) >= R_BATCH:
                    likelihood.update_R(np.array(innovations), lr=R_LR)
                    innovations.clear()
        else:
            belief.initialise(o)
            agent.reset_raw_velocity()
            prev_ball_state = None
            innovations.clear()

        action, _ = agent.select_action_horizon(horizon=HORIZON, gamma=GAMMA)

        prev_opp_y  = opp_y
        prev_action = action
        obs, reward, terminated, truncated, _ = env.step(action)
        score += reward

        if reward == -1:
            agent.update_from_miss(o[1], o[2])

        if terminated or truncated:
            break

    return score, agent.landing_bias


# ── run experiments ───────────────────────────────────────────────────────────

env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
print(f"Phase 4 — Online Learning  (H={HORIZON}, γ={GAMMA}, N={N_EPISODES})\n")

# ── frozen baseline: fresh load each episode (reproduces Phase 3.3) ───────────
print("── Frozen baseline (fresh models per episode) ──")
frozen_scores, t0, bias = [], time.time(), 0.0
for ep in range(N_EPISODES):
    bm, om, lk = make_models()
    s, bias = run_episode(env, bm, om, lk, learn=False, landing_bias=bias)
    frozen_scores.append(s)
    print(f"  ep {ep+1:2d}: {s:+.0f}  bias={bias:+.4f}"
          f"  (mean {np.mean(frozen_scores):+.1f})")
print(f"  → {time.time()-t0:.0f}s total\n")

# ── online: shared models accumulate updates across all episodes ───────────────
print("── Online learning (shared models across episodes) ──")
bm_on, om_on, lk_on = make_models()
online_scores, t0, bias = [], time.time(), 0.0
for ep in range(N_EPISODES):
    s, bias = run_episode(env, bm_on, om_on, lk_on, learn=True, landing_bias=bias)
    online_scores.append(s)
    print(f"  ep {ep+1:2d}: {s:+.0f}  bias={bias:+.4f}"
          f"  (mean {np.mean(online_scores):+.1f})")
print(f"  → {time.time()-t0:.0f}s total\n")

env.close()

# ── plots ─────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(2, 1, figsize=(10, 8))
fig.suptitle(f"Phase 4 — Online Learning  (H={HORIZON}, γ={GAMMA})", fontsize=13)
x = np.arange(1, N_EPISODES + 1)

ax = axes[0]
ax.plot(x, frozen_scores, "o-", color="steelblue", alpha=0.7,
        label=f"Frozen  mean={np.mean(frozen_scores):+.1f}")
ax.plot(x, online_scores, "o-", color="darkorange", alpha=0.7,
        label=f"Online  mean={np.mean(online_scores):+.1f}")
ax.axhline(0, color="gray", lw=0.8, ls=":")
ax.set_ylabel("Score"); ax.set_xlabel("Episode")
ax.set_title("Per-episode scores: frozen vs online learning")
ax.legend(); ax.grid(True, alpha=0.3)

W = 5
def rolling(a, w):
    return [np.mean(a[max(0, i - w + 1):i + 1]) for i in range(len(a))]

ax = axes[1]
ax.plot(x, rolling(frozen_scores, W), "-", color="steelblue",
        label=f"Frozen {W}-ep rolling mean")
ax.plot(x, rolling(online_scores, W), "-", color="darkorange",
        label=f"Online {W}-ep rolling mean")
ax.axhline(0, color="gray", lw=0.8, ls=":")
ax.set_ylabel("Rolling mean score"); ax.set_xlabel("Episode")
ax.set_title(f"Rolling {W}-episode mean (learning trend)")
ax.legend(); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("validation_phase4.png", dpi=120)
plt.show()
print("Saved validation_phase4.png\n")

print("── Summary ──")
print(f"Frozen  mean={np.mean(frozen_scores):+.1f}  "
      f"min={min(frozen_scores):+.0f}  max={max(frozen_scores):+.0f}")
print(f"Online  mean={np.mean(online_scores):+.1f}  "
      f"min={min(online_scores):+.0f}  max={max(online_scores):+.0f}")
