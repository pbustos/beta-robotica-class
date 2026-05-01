"""
Pretrain BallTransitionVBGS and OpponentTransitionVBGS until convergence
or a time limit, then save to models/.

Convergence: improvement in smoothed MAE < CONV_THRESHOLD over CONV_WINDOW frames.
"""

import time
import numpy as np
import gymnasium as gym
import ale_py

from generative_model import BallTransitionVBGS, OpponentTransitionVBGS

gym.register_envs(ale_py)

# ── config ────────────────────────────────────────────────────────────────────

TIME_LIMIT      = 5 * 60        # seconds
WARMUP          = 100           # frames before tracking error
SMOOTH_WINDOW   = 500           # rolling window for MAE smoothing
CONV_WINDOW     = 2000          # frames over which to assess convergence
CONV_THRESHOLD  = 0.01          # relative improvement threshold (1%)
MAE_TARGET      = 0.008         # must reach this MAE before convergence counts
REPORT_EVERY    = 1000          # frames between progress prints

# ── helpers ───────────────────────────────────────────────────────────────────

def extract_ball(ram):
    return np.array([ram[49] / 255.0, ram[54] / 255.0], dtype=float)

def chase_action(ram):
    py, by = ram[51] / 255.0, ram[54] / 255.0
    return 3 if py < by - 0.02 else (2 if py > by + 0.02 else 0)

def rolling_mean(buf, w):
    return float(np.mean(buf[-w:])) if len(buf) >= w else float(np.mean(buf))

def is_converged(errors, window, threshold, target):
    if len(errors) < 2 * window:
        return False
    old = np.mean(errors[-2 * window:-window])
    new = np.mean(errors[-window:])
    if new > target:
        return False
    return old < 1e-12 or abs(old - new) / old < threshold

# ── init ──────────────────────────────────────────────────────────────────────

ball_model     = BallTransitionVBGS()
opponent_model = OpponentTransitionVBGS()

env    = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
obs, _ = env.reset()

prev_ball = extract_ball(obs)
prev_opp  = obs[50] / 255.0
prev_vel  = np.zeros(2)

ball_errors = []
opp_errors  = []

frame      = 0
t_start    = time.time()
converged  = False

print(f"Pretraining — time limit {TIME_LIMIT}s, convergence threshold {CONV_THRESHOLD}")
print(f"{'Frame':>8}  {'Elapsed':>7}  {'Ball MAE':>10}  {'Opp MAE':>10}  {'Ball weights'}")
print("-" * 65)

# ── main loop ─────────────────────────────────────────────────────────────────

while True:
    elapsed = time.time() - t_start
    if elapsed >= TIME_LIMIT:
        print(f"\nStopped: time limit reached ({TIME_LIMIT}s)")
        break

    ball  = extract_ball(obs)
    vel   = ball - prev_ball
    opp_y = obs[50] / 255.0

    s_t    = np.concatenate([prev_ball, prev_vel])
    s_next = np.concatenate([ball, vel])

    action = chase_action(obs)

    if frame >= WARMUP:
        # ball prediction error (before update)
        ball_pred = ball_model.predict_mean(s_t)
        ball_errors.append(np.abs(ball_pred[:2] - s_next[:2]).mean())

        # opponent prediction error (before update)
        ball_state_4 = s_t  # [bx, by, bvx, bvy]
        opp_pred = opponent_model.predict_mean(prev_opp, ball_state_4, action)
        opp_errors.append(abs(float(opp_pred[0]) - opp_y))

        if is_converged(ball_errors, CONV_WINDOW, CONV_THRESHOLD, MAE_TARGET):
            print(f"\nStopped: converged after {frame} frames ({elapsed:.1f}s)")
            converged = True
            break

    # online updates
    ball_model.update(s_t, s_next)
    opponent_model.update(prev_opp, s_t, action, opp_y)

    if frame % REPORT_EVERY == 0 and frame >= WARMUP:
        b_mae  = rolling_mean(ball_errors, SMOOTH_WINDOW)
        o_mae  = rolling_mean(opp_errors,  SMOOTH_WINDOW)
        w      = ball_model.vbgs.component_weights().round(3)
        print(f"{frame:>8}  {elapsed:>6.1f}s  {b_mae:>10.5f}  {o_mae:>10.5f}  {w}")

    # env step
    obs, _, term, trunc, _ = env.step(action)
    if term or trunc:
        obs, _ = env.reset()
        prev_ball = extract_ball(obs)
        prev_opp  = obs[50] / 255.0
        prev_vel  = np.zeros(2)
    else:
        prev_vel  = vel
        prev_ball = ball
        prev_opp  = opp_y

    frame += 1

env.close()

# ── final report ──────────────────────────────────────────────────────────────

elapsed = time.time() - t_start
print(f"\n{'─'*65}")
print(f"Frames      : {frame}")
print(f"Time        : {elapsed:.1f}s  ({frame/elapsed:.0f} fps)")
print(f"Ball MAE    : {rolling_mean(ball_errors, SMOOTH_WINDOW):.5f}")
print(f"Opp  MAE    : {rolling_mean(opp_errors,  SMOOTH_WINDOW):.5f}")
print(f"Ball weights: {ball_model.vbgs.component_weights().round(4)}")
print(f"Opp  weights: {opponent_model.vbgs.component_weights().round(4)}")

ball_model.save()
opponent_model.save()
print("\nSaved: models/ball_vbgs.pkl  models/opponent_vbgs.pkl")
