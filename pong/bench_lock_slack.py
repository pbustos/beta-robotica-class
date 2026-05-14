"""
Paired-seed A/B benchmark harness (template).

Both arms run the same pipeline (tree H=3, warm rMM, identical seeds).
Re-template `run_arm` and the two `run_arm(...)` invocations at the
bottom of the script to flip whichever knob the current experiment
needs (typically a class attribute on EFEAgent or a module constant).

Usage:
    python bench_lock_slack.py [N_EPISODES]
"""

import copy
import pickle
import pathlib
import sys
import time
import numpy as np
import gymnasium as gym
import ale_py

import generative_model
from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter
)
from efe_agent import EFEAgent, LongTermStats


STATE_FILE = pathlib.Path("models/agent_state.pkl")


def load_warm_state():
    """Load the full warm state (rMM + adaptive params) from agent_state.pkl."""
    if not STATE_FILE.exists():
        return {}
    with open(STATE_FILE, "rb") as f:
        return pickle.load(f)


def apply_warm_state(agent, state):
    """Initialise a fresh per-episode agent with any saved adaptive state."""
    lt_dict = state.get("long_term")
    if lt_dict is not None:
        from efe_agent import LongTermStats
        agent.long_term = LongTermStats.from_dict(copy.deepcopy(lt_dict))

    bd = state.get("bias_dict")
    if bd is not None:
        for raw_k, v in bd.items():
            k = tuple(int(x) for x in raw_k.strip("()").split(", "))
            if k in agent._bias_dict:
                agent._bias_dict[k] = float(v)
    elif "landing_bias" in state and not isinstance(state["landing_bias"], dict):
        agent.landing_bias = float(state["landing_bias"])   # legacy scalar fan-out

    if "placement" in state:
        agent.placement = float(state["placement"])
    if "frames_near" in state and state["frames_near"] is not None:
        fn = int(state["frames_near"])
        agent.preferences.update_urgency(frames_near=fn, frames_start=fn + 15)
    if "urgency_ema" in state and state["urgency_ema"] is not None:
        agent._fell_short_frames_ema = float(state["urgency_ema"])
    if "interaction_alpha" in state and state["interaction_alpha"] is not None:
        agent.interaction_model.alpha = np.asarray(
            state["interaction_alpha"], float)

gym.register_envs(ale_py)

N_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 30
GAMMA      = 0.9
SEED_BASE  = 12345


def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])


def make_models():
    return (BallTransitionVBGS.from_file(),
            OpponentTransitionVBGS.from_file(),
            LikelihoodModel())


def run_episode(env, seed, ball_model, opp_model, likelihood, agent):
    """Reset belief filter & opp tracker; reuse the persistent agent
    (its rMM, bias_dict, placement, urgency, etc. carry across episodes)."""
    opp_tracker  = OpponentBeliefTracker(opp_model)
    belief       = MixtureBeliefFilter(ball_model, opp_tracker, likelihood)
    agent.belief = belief
    agent.reset_raw_velocity()
    agent._target_y_locked = None
    agent._locked_offset   = None

    obs, _ = env.reset(seed=seed)
    o = extract_obs(obs)
    belief.initialise(o)

    score                 = 0
    prev_opp_y            = o[3]
    prev_action           = 0
    bounced_this_approach = False
    n_contacts            = 0
    n_no_contact_losses   = 0
    rally_had_contact     = False

    while True:
        o            = extract_obs(obs)
        ball_in_play = o[0] > 0.05
        opp_y        = o[3]

        if ball_in_play:
            prev_raw_vx = agent._raw_vx
            agent.update_raw_velocity(o[0], o[1])
            if agent._last_vy_bounce:
                bounced_this_approach = True

            if (prev_raw_vx > 0.008 and agent._raw_vx < -0.008
                    and o[0] > 0.60):
                contact_offset = o[1] - o[2]
                agent.interaction_model.observe(contact_offset)
                agent.update_preferences_on_contact(
                    o[2],
                    vy_sign=int(np.sign(agent._raw_vy)),
                    bounced=int(bounced_this_approach),
                    opp_y=float(o[3]),
                )
                bounced_this_approach = False
                n_contacts += 1
                rally_had_contact = True

            ball_state_4 = belief.mean[:4].copy()
            belief.predict(prev_action)
            belief.correct(o)
            opp_tracker.update(prev_opp_y, ball_state_4, prev_action, opp_y)
        else:
            belief.initialise(o)
            agent.reset_raw_velocity()
            bounced_this_approach = False

        action, _ = agent.select_action_horizon(horizon=3, gamma=GAMMA)
        prev_opp_y  = opp_y
        prev_action = action

        obs, reward, terminated, truncated, _ = env.step(action)
        score += reward

        if reward == -1:
            agent.update_from_miss(o[1], o[2])
            if not rally_had_contact:
                n_no_contact_losses += 1
            rally_had_contact = False
        if reward == 1:
            agent.record_win()
            rally_had_contact = False

        if terminated or truncated:
            break

    return score, n_contacts, n_no_contact_losses


def run_arm(label, feature, pkl_path, adaptive_horizon=False,
            vel_alpha_hybrid=False, relock_on_bounce=False):
    """Run a single arm with the requested rMM feature mode + toggles."""
    from efe_agent import RMM
    EFEAgent._RMM_FEATURE      = feature
    EFEAgent._ADAPTIVE_HORIZON = bool(adaptive_horizon)
    EFEAgent._VEL_ALPHA_HYBRID = bool(vel_alpha_hybrid)
    EFEAgent._RELOCK_ON_BOUNCE = bool(relock_on_bounce)
    RMM.FEATURE_VERSION        = {"offset": "v2-offset",
                                   "target_y": "v2-target_y"}[feature]

    warm_state = {}
    if pathlib.Path(pkl_path).exists():
        with open(pkl_path, "rb") as f:
            warm_state = pickle.load(f)

    print(f"\n── {label}  feature={feature}  adaptive_H={adaptive_horizon}  "
          f"vel_α_hybrid={vel_alpha_hybrid}  relock_on_bounce={relock_on_bounce}  "
          f"pkl={pkl_path}  N={N_EPISODES} ──")
    env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
    bm, om, lk = make_models()
    prefs = PriorPreferences()
    belief = MixtureBeliefFilter(bm, OpponentBeliefTracker(om), lk)
    agent  = EFEAgent(belief, lk, prefs)
    apply_warm_state(agent, copy.deepcopy(warm_state))
    print(f"  loaded: rMM K={agent.long_term.n_components()}  "
          f"rallies={int(agent.long_term.evidence())}  "
          f"placement={agent.placement:.4f}")
    scores, contacts, ncls = [], [], []
    t0 = time.time()
    for ep in range(N_EPISODES):
        seed = SEED_BASE + ep
        s, nc, nl = run_episode(env, seed, bm, om, lk, agent)
        scores.append(s); contacts.append(nc); ncls.append(nl)
        agent.update_from_episode(s)
        K = agent.long_term.n_components()
        rallies = int(agent.long_term.evidence())
        print(f"  ep {ep+1:2d}: score={s:+3.0f}  contacts={nc:3d}  "
              f"no-contact-losses={nl:2d}  K={K}  rallies={rallies}  "
              f"(mean {np.mean(scores):+.2f})")
    env.close()
    arr = np.array(scores)
    print(f"  → {time.time()-t0:.0f}s  mean={arr.mean():+.2f}  std={arr.std():.2f}  "
          f"wins(>0)={int((arr>0).sum())}  best={arr.max():+.0f}  worst={arr.min():+.0f}")
    print(f"  contacts/ep={np.mean(contacts):.1f}  no-contact-losses/ep={np.mean(ncls):.2f}")
    return scores, contacts, ncls


if __name__ == "__main__":
    # Paired A/B: lock_hold (current) vs relock_on_bounce. Both arms use
    # the target_y feature and fixed H=3, same warm pkl. The relock arm
    # clears _target_y_locked on every detected vy_bounce while ball is
    # approaching, forcing _get_preferences to resample against the
    # post-bounce landing prediction. Hypothesis from analyze_failures:
    # late bounces are the dominant aim-error driver and they bypass the
    # existing LOCK_SLACK=0.10 hysteresis.
    base_scores, base_nc, base_ncl = run_arm(
        "lock_hold",   "target_y", "models/agent_state.target_y.pkl",
        relock_on_bounce=False)
    new_scores,  new_nc,  new_ncl  = run_arm(
        "lock_relock", "target_y", "models/agent_state.target_y.pkl",
        relock_on_bounce=True)

    print("\n── Summary ──")
    print(f"{'arm':<10} {'mean':>8} {'std':>6} {'wins':>5} {'best':>5} {'worst':>5} "
          f"{'contacts/ep':>12} {'NCL/ep':>7}")
    for label, scs, ncs, nls in [("lock_hold",   base_scores, base_nc, base_ncl),
                                   ("lock_relock", new_scores,  new_nc,  new_ncl)]:
        a = np.array(scs)
        print(f"{label:<10} {a.mean():>+8.2f} {a.std():>6.2f} {int((a>0).sum()):>5d} "
              f"{a.max():>+5.0f} {a.min():>+5.0f} {np.mean(ncs):>12.1f} {np.mean(nls):>7.2f}")

    delta = np.mean(new_scores) - np.mean(base_scores)
    delta_ncl = np.mean(new_ncl) - np.mean(base_ncl)
    print(f"\nΔmean = {delta:+.2f}    ΔNCL/ep = {delta_ncl:+.2f}")
