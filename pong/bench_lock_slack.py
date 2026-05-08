"""
A/B benchmark for raw-velocity bounce-detection tweaks.

Two arms, identical seeds, frozen models, warm rMM:
  - baseline: original bounce detection (_BOUNCE_THRESH=0.008, no wall-clip)
  - sensitive: lower threshold (0.003) + wall-clip detection (margin 0.005)

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

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter
)
from efe_agent import EFEAgent, LongTermStats


STATE_FILE = pathlib.Path("models/agent_state.pkl")


def load_warm_long_term():
    """Load the warm LongTermStats from agent_state.pkl so the rMM is populated."""
    if not STATE_FILE.exists():
        return None, 0.0
    with open(STATE_FILE, "rb") as f:
        s = pickle.load(f)
    lt_dict = s.get("long_term")
    bias = float(s.get("landing_bias", 0.0)) if not isinstance(s.get("landing_bias"), dict) else 0.0
    if lt_dict is None:
        return None, bias
    return lt_dict, bias

gym.register_envs(ale_py)

N_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 30
HORIZON    = 3
GAMMA      = 0.9
SEED_BASE  = 12345


def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])


def make_models():
    return (BallTransitionVBGS.from_file(),
            OpponentTransitionVBGS.from_file(),
            LikelihoodModel())


def run_episode(env, seed, ball_model, opp_model, likelihood, long_term, landing_bias):
    opp_tracker = OpponentBeliefTracker(opp_model)
    belief      = MixtureBeliefFilter(ball_model, opp_tracker, likelihood)
    prefs       = PriorPreferences()
    agent       = EFEAgent(belief, likelihood, prefs)
    agent.long_term     = long_term
    agent.landing_bias  = landing_bias

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

        action, _ = agent.select_action_horizon(horizon=HORIZON, gamma=GAMMA)
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

    return score, agent.landing_bias, n_contacts, n_no_contact_losses


def run_arm(label, bounce_thresh, wall_clip_margin, warm_lt_dict, warm_bias):
    print(f"\n── {label}  (bounce_thresh={bounce_thresh}, "
          f"wall_clip_margin={wall_clip_margin})  N={N_EPISODES} ──")
    EFEAgent._BOUNCE_THRESH = bounce_thresh
    EFEAgent._WALL_CLIP_MARGIN = wall_clip_margin
    env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
    bm, om, lk = make_models()
    if warm_lt_dict is not None:
        lt = LongTermStats.from_dict(copy.deepcopy(warm_lt_dict))
        print(f"  loaded warm rMM: K={lt.n_components()}  rallies={int(lt.evidence())}")
    else:
        lt = LongTermStats()
    scores, contacts, ncls = [], [], []
    bias = float(warm_bias) if warm_bias is not None else 0.0
    t0 = time.time()
    for ep in range(N_EPISODES):
        seed = SEED_BASE + ep
        s, bias, nc, nl = run_episode(env, seed, bm, om, lk, lt, bias)
        scores.append(s); contacts.append(nc); ncls.append(nl)
        print(f"  ep {ep+1:2d}: score={s:+3.0f}  contacts={nc:3d}  "
              f"no-contact-losses={nl:2d}  K={lt.n_components()}  "
              f"rallies={int(lt.evidence())}  (mean {np.mean(scores):+.2f})")
    env.close()
    arr = np.array(scores)
    print(f"  → {time.time()-t0:.0f}s  mean={arr.mean():+.2f}  std={arr.std():.2f}  "
          f"wins(>0)={int((arr>0).sum())}  best={arr.max():+.0f}  worst={arr.min():+.0f}")
    print(f"  contacts/ep={np.mean(contacts):.1f}  no-contact-losses/ep={np.mean(ncls):.2f}")
    return scores, contacts, ncls


if __name__ == "__main__":
    warm_lt, warm_bias = load_warm_long_term()
    if warm_lt is not None:
        print(f"Loaded warm long_term from {STATE_FILE} (bias={warm_bias:+.4f})")
    else:
        print("No warm state — starting from cold rMM")
    # margin=0 disables wall-clip (the by<0 / by>1-0 condition is never true)
    base_scores, base_nc, base_ncl = run_arm("baseline",  0.008, 0.000, warm_lt, warm_bias)
    new_scores,  new_nc,  new_ncl  = run_arm("sensitive", 0.003, 0.005, warm_lt, warm_bias)

    print("\n── Summary ──")
    print(f"{'arm':<10} {'mean':>8} {'std':>6} {'wins':>5} {'best':>5} {'worst':>5} "
          f"{'contacts/ep':>12} {'NCL/ep':>7}")
    for label, scs, ncs, nls in [("baseline",  base_scores, base_nc, base_ncl),
                                   ("sensitive", new_scores, new_nc, new_ncl)]:
        a = np.array(scs)
        print(f"{label:<10} {a.mean():>+8.2f} {a.std():>6.2f} {int((a>0).sum()):>5d} "
              f"{a.max():>+5.0f} {a.min():>+5.0f} {np.mean(ncs):>12.1f} {np.mean(nls):>7.2f}")

    delta = np.mean(new_scores) - np.mean(base_scores)
    delta_ncl = np.mean(new_ncl) - np.mean(base_ncl)
    print(f"\nΔmean = {delta:+.2f}    ΔNCL/ep = {delta_ncl:+.2f}")
