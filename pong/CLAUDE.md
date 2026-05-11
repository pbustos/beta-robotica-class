# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Active Inference agent that plays `ALE/Pong-v5` (gymnasium + ale_py, RAM observations). The agent's "physics engine" is a pair of Variational Bayesian Gaussian Sums (VBGS) fitted with closed-form CAVI; action selection is Expected Free Energy (EFE) minimisation with a deterministic horizon-3 tree search (`select_action_horizon`). There is no RL — no policy/value networks, no replay buffer, no gradients. Treat the math in `PONG.md` as the source of truth for the model.

## Working rules (from `.github/copilot-instructions.md`)

1. Don't assume. Don't hide confusion. Surface tradeoffs.
2. Minimum code that solves the problem. Nothing speculative.
3. Touch only what you must. Clean up only your own mess.
4. Define success criteria. Loop until verified.

## Commands

Pretrained VBGS models in `models/` are required before any agent script runs. If `models/ball_vbgs.pkl` or `models/opponent_vbgs.pkl` is missing, run pretraining first.

```bash
# 1. Pretrain ball + opponent VBGS (~5 min, until convergence or time limit)
python pretrain_models.py

# 2. Watch the agent play live (pygame window with game + metrics panel)
python play_efe.py                       # frozen models; only landing-bias adapts
python play_efe.py --learn               # also update VBGS and likelihood R online
python play_efe.py --reset               # wipe models/agent_state.pkl, start fresh

# 3. Headless experiments (matplotlib output)
python train_efe.py                      # Phase 4: frozen vs online over N_EPISODES
python validate_efe.py                   # Phase 3.3: horizon sweep H∈{1,3,5} vs rule-based
python validate_ball_model.py            # ball VBGS prediction error / weights
python validate_belief.py                # single-Gaussian belief filter
python validate_mixture_belief.py        # K=2 mixture filter (bounce-aware)
python validate_opponent.py              # opponent tracker accuracy

# 4. Rule-based baseline (no models needed)
python main.py
```

There is no test suite, no linter config, no `requirements.txt`. The validation scripts double as integration tests — each prints diagnostics and saves a `validation_*.png`. Runtime deps used directly: `gymnasium`, `ale_py`, `numpy`, `scipy.special.digamma`, `pygame`, `matplotlib`.

`*_v1.py` files (`efe_agent_v1.py`, `generative_model_v1.py`, `play_efe_v1.py`) are frozen earlier-phase snapshots kept for comparison. The active code is the unsuffixed version. `play_efe_log_*.csv` files are per-run telemetry logs — large and disposable.

## Architecture

### Two-file split

- **`generative_model.py`** — everything probabilistic: NIW conjugate prior, VBGS (CAVI E-step / M-step / `predict_conditional`), the two transition models (`BallTransitionVBGS`, `OpponentTransitionVBGS`), the linear-Gaussian `LikelihoodModel` (`o = H·s + ε`, R=1e-4·I), the Kalman-style filters (`BeliefFilter`, `MixtureBeliefFilter`), the opponent belief tracker, the `InteractionModel` (Dirichlet over paddle contact offsets), and `PriorPreferences` (the C-distribution over preferred observations — adaptively shrinks `σ_py` as the ball approaches the player).
- **`efe_agent.py`** — `EFEAgent` (EFE scoring + three action selectors: greedy `select_action`, tree-search `select_action_horizon` which is the one `play_efe.py` calls, and `select_action_mppi` kept for experiments) plus `LongTermStats`, a persistent cross-session sufficient-statistics accumulator (per-(vy_sign, bounced) Bayesian aim calibration + Beta win-rate grid over target_y).

`play_efe.py`, `train_efe.py`, and the `validate_*.py` scripts each instantiate the same pipeline: `make_models()` → `OpponentBeliefTracker` → `MixtureBeliefFilter` → `PriorPreferences` → `EFEAgent`. `play_efe.py` is the only entry point with a UI and persistent state.

### State / observation conventions

Module-level in `generative_model.py`:

| Name | Value | Meaning |
|------|-------|---------|
| RAM[49], RAM[54], RAM[51], RAM[50] | — | ball_x, ball_y, player_y, opp_y |
| `_PLAYER_X` | 0.740 | measured player paddle x (normalised) |
| `_OPPONENT_X` | 0.270 | measured opponent paddle x |
| `PADDLE_DY` | `{0: 0.0, 2: -0.060, 3: +0.060}` | per-step displacement under frame_skip=4 |

- 4D observation `o = [bx, by, py, oy] / 255`.
- 6D state `s = [bx, by, vx, vy, py, oy]` — velocities are latent, recovered by the filter.
- Action set: `[0=NOOP, 2=UP, 3=DOWN]`.
- The constant-velocity ball matrix `_F` in `efe_agent.py` is used for the linear-Gaussian rollout at horizon depth ≥ 1; depth 0 always uses the full VBGS `predict` (which is wall-bounce-aware via the K=2 mixture).

### Key design facts that are not obvious from the code

These came from earlier phase debugging — touching them tends to regress the score:

- Both planners use a **shared `o_star` per frame** computed once from the current belief, not per action branch. Per-action recomputation introduces VBGS ball-state noise across branches and degrades selection.
- `EFEAgent.update_raw_velocity(bx, by)` keeps an EMA frame-difference velocity (α=0.7) in parallel with the Kalman estimate. The Kalman vx spikes to ±0.3 after paddle contact, which corrupts landing prediction; the raw EMA is what feeds `_predict_ball_landing`. Reset on serve / out-of-play with `reset_raw_velocity()`.
- Bounce detection (in `update_raw_velocity`): if the new frame-diff velocity is opposite-sign to the EMA AND `|dv| > _BOUNCE_THRESH=0.003`, hard-reset the EMA instead of blending. Wall-clip detection (`_WALL_CLIP_MARGIN=0.005`) catches the case where `dvy ≈ 0` because `by` was clipped to the edge for one frame. Together these were the largest single score improvement (+4.80 mean per episode under tree-search planner, paired t-test p<0.001).
- `select_action_mppi` (stochastic sampling, longer horizons) underperforms `select_action_horizon(H=3)` by ~5 score points per episode under current code (100-ep MPPI(H=15)=−7.0 vs tree(H=3)=−1.7). MPPI's averaging washes out the bounce-detection signal; long-horizon rollouts drift past depth ~5 with no wall-bounce correction.
- `landing_bias` is a dict keyed by `(vy_sign ∈ {-1,0,1}, bounced ∈ {0,1})` — six independent EMAs, not a scalar. The public `landing_bias` property returns the active key's value for display; the setter writes a scalar to all keys (legacy state-restore path).

### Persistence

`models/agent_state.pkl` is written at every episode end and on window-close in `play_efe.py`. It carries: `landing_bias`, `bias_dict`, `placement`, `interaction_alpha`, `frames_near`, `urgency_ema`, `long_term` (LongTermStats), and — only when `--learn` was used — `ball_vbgs` and `R`. `--reset` deletes this file. The pretrained `models/ball_vbgs.pkl` and `models/opponent_vbgs.pkl` are produced by `pretrain_models.py` and are not touched by `play_efe.py` unless `--learn` is set.

## Reference

`PONG.md` is the mathematical specification (NIW prior, CAVI updates, Gaussian conditioning, EFE decomposition, MPPI). Read it before changing the inference code; update it if you change the math. Note: the production planner is now `select_action_horizon`; MPPI math in PONG.md still describes the experimental code path in `efe_agent.py`.
