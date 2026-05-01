"""
Validation for BallTransitionVBGS (Phase 1.2).

Runs N frames of ALE/Pong, feeds transitions online into the model, then plots:
  1. Rolling prediction error (position + velocity)
  2. Component weight evolution
  3. Sample trajectory: predicted vs actual (last episode)
"""

import numpy as np
import matplotlib.pyplot as plt
import gymnasium as gym
import ale_py

from generative_model import BallTransitionVBGS

gym.register_envs(ale_py)

# ── config ────────────────────────────────────────────────────────────────────

N_FRAMES     = 4000   # total frames to collect
WARMUP       = 50     # frames before recording errors (model cold-start)
WINDOW       = 100    # rolling window for smoothed error
TRAJ_LEN     = 60     # frames to show in trajectory plot

# ── helpers ───────────────────────────────────────────────────────────────────

def extract_ball(ram):
    return np.array([ram[49] / 255.0, ram[54] / 255.0], dtype=float)


def make_state(ball, vel):
    return np.concatenate([ball, vel])


# ── data collection ───────────────────────────────────────────────────────────

env   = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
model = BallTransitionVBGS()

errors_pos  = []   # |predicted_pos - actual_pos|
errors_vel  = []   # |predicted_vel - actual_vel|
weights_log = []   # component weights per frame

# trajectory recording for the last stretch
traj_actual    = []
traj_predicted = []

obs, _ = env.reset()
prev_ball = extract_ball(obs)
prev_vel  = np.zeros(2)

for frame in range(N_FRAMES):
    ball = extract_ball(obs)
    vel  = ball - prev_ball
    s_t  = make_state(prev_ball, prev_vel)
    s_next = make_state(ball, vel)

    if frame >= WARMUP:
        # predict before updating (out-of-sample error)
        s_pred = model.predict_mean(s_t)
        err_pos = np.abs(s_pred[:2] - s_next[:2]).mean()
        err_vel = np.abs(s_pred[2:] - s_next[2:]).mean()
        errors_pos.append(err_pos)
        errors_vel.append(err_vel)
        weights_log.append(model.vbgs.component_weights().copy())

        if frame >= N_FRAMES - TRAJ_LEN:
            traj_actual.append(s_next[:2].copy())
            traj_predicted.append(s_pred[:2].copy())

    # online CAVI update
    model.update(s_t, s_next)

    # step environment (rule-based action: chase ball)
    py = obs[51] / 255.0
    by = ball[1]
    action = 3 if py < by - 0.02 else (2 if py > by + 0.02 else 0)
    obs, _, term, trunc, _ = env.step(action)
    if term or trunc:
        obs, _ = env.reset()
        prev_ball = extract_ball(obs)
        prev_vel  = np.zeros(2)
    else:
        prev_vel  = vel
        prev_ball = ball

env.close()

# ── rolling smoothing ─────────────────────────────────────────────────────────

def rolling_mean(x, w):
    return np.convolve(x, np.ones(w) / w, mode="valid")

smooth_pos = rolling_mean(errors_pos, WINDOW)
smooth_vel = rolling_mean(errors_vel, WINDOW)
weights_arr = np.array(weights_log)

# ── plots ─────────────────────────────────────────────────────────────────────

fig, axes = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle("BallTransitionVBGS — validation", fontsize=13)

# 1. prediction error over time
ax = axes[0]
x  = np.arange(len(smooth_pos))
ax.plot(x, smooth_pos, label="position MAE", color="steelblue")
ax.plot(x, smooth_vel, label="velocity MAE", color="darkorange", alpha=0.8)
ax.set_ylabel("Mean absolute error (normalised)")
ax.set_xlabel(f"Frame (rolling window={WINDOW})")
ax.set_title("Prediction error (lower = better)")
ax.legend()
ax.grid(True, alpha=0.3)

# 2. component weight evolution
ax = axes[1]
labels = ["Free flight", "Wall bounce", "Paddle bounce"]
colors = ["steelblue", "darkorange", "seagreen"]
for k in range(BallTransitionVBGS.K):
    ax.plot(weights_arr[:, k], label=labels[k], color=colors[k], alpha=0.85)
ax.set_ylabel("Mixing weight")
ax.set_xlabel("Frame")
ax.set_title("Component weight evolution")
ax.legend()
ax.grid(True, alpha=0.3)
ax.set_ylim(0, 1)

# 3. trajectory: predicted vs actual
ax = axes[2]
traj_a = np.array(traj_actual)
traj_p = np.array(traj_predicted)
t      = np.arange(len(traj_a))
ax.plot(t, traj_a[:, 0],  label="actual x",    color="steelblue")
ax.plot(t, traj_p[:, 0],  label="predicted x", color="steelblue", linestyle="--", alpha=0.8)
ax.plot(t, traj_a[:, 1],  label="actual y",    color="darkorange")
ax.plot(t, traj_p[:, 1],  label="predicted y", color="darkorange", linestyle="--", alpha=0.8)
ax.set_ylabel("Normalised position")
ax.set_xlabel("Frame")
ax.set_title(f"Trajectory comparison (last {TRAJ_LEN} frames)")
ax.legend(ncol=2)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("validation_ball_model.png", dpi=120)
plt.show()
print("Saved validation_ball_model.png")

# ── summary stats ─────────────────────────────────────────────────────────────

print(f"\nFrames collected : {N_FRAMES}  (warmup={WARMUP})")
print(f"Final pos  MAE   : {smooth_pos[-1]:.5f}  (first: {smooth_pos[0]:.5f})")
print(f"Final vel  MAE   : {smooth_vel[-1]:.5f}  (first: {smooth_vel[0]:.5f})")
print(f"Final component weights: {model.vbgs.component_weights().round(3)}")

model.save()
print("Model saved to models/ball_vbgs.pkl")
