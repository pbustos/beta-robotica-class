"""
Warm up a fresh rMM by playing N episodes with a single persistent agent.

Usage:
    python warmup_rmm.py [N_EPISODES] [--feature offset|target_y]

Default feature is "offset"; "target_y" produces the legacy-feature baseline
for the paired A/B. Saves to models/agent_state.{feature}.pkl (and also
copies to models/agent_state.pkl for convenience).
"""
import sys, copy, pickle, pathlib, time
import numpy as np
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter)
from efe_agent import EFEAgent, RMM

gym.register_envs(ale_py)

# Parse args
_pos     = [a for a in sys.argv[1:] if not a.startswith("--")]
N_EPISODES = int(_pos[0]) if _pos else 50
FEATURE = "offset"
if "--feature" in sys.argv:
    FEATURE = sys.argv[sys.argv.index("--feature") + 1]
assert FEATURE in ("offset", "target_y"), f"unknown feature {FEATURE!r}"

# Wire the rMM feature mode + pkl version sentinel.
EFEAgent._RMM_FEATURE = FEATURE
RMM.FEATURE_VERSION   = {"offset": "v2-offset", "target_y": "v2-target_y"}[FEATURE]

GAMMA      = 0.9
SEED_BASE  = 24680
STATE_FILE_GENERIC = pathlib.Path("models/agent_state.pkl")
STATE_FILE_FEAT    = pathlib.Path(f"models/agent_state.{FEATURE}.pkl")


def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])


def main():
    env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
    ball_m = BallTransitionVBGS.from_file()
    opp_m  = OpponentTransitionVBGS.from_file()
    lik    = LikelihoodModel()

    # Single agent that persists across episodes — rMM, bias_dict, urgency
    # window, and placement all accumulate.
    opp_t  = OpponentBeliefTracker(opp_m)
    belief = MixtureBeliefFilter(ball_m, opp_t, lik)
    prefs  = PriorPreferences()
    agent  = EFEAgent(belief, lik, prefs)

    t0 = time.time()
    scores = []
    for ep in range(N_EPISODES):
        # New env state per episode; rebuild filter (keeps agent state).
        opp_t  = OpponentBeliefTracker(opp_m)
        belief = MixtureBeliefFilter(ball_m, opp_t, lik)
        agent.belief = belief

        obs, _ = env.reset(seed=SEED_BASE + ep)
        o = extract_obs(obs)
        belief.initialise(o)

        prev_action = 0
        prev_opp_y  = o[3]
        score                 = 0
        bounced_this_approach = False
        n_contacts            = 0
        n_ncl                 = 0
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
                    agent.interaction_model.observe(o[1] - o[2])
                    agent.update_preferences_on_contact(
                        o[2],
                        vy_sign=int(np.sign(agent._raw_vy)),
                        bounced=int(bounced_this_approach),
                        opp_y=float(o[3]))
                    bounced_this_approach = False
                    n_contacts += 1
                    rally_had_contact = True

                ball_state_4 = belief.mean[:4].copy()
                belief.predict(prev_action)
                belief.correct(o)
                opp_t.update(prev_opp_y, ball_state_4, prev_action, opp_y)
            else:
                belief.initialise(o)
                agent.reset_raw_velocity()
                bounced_this_approach = False

            action, _ = agent.select_action_horizon(horizon=3, gamma=GAMMA)
            prev_opp_y  = opp_y
            prev_action = action

            obs, reward, term, trunc, _ = env.step(action)
            score += reward

            if reward == -1:
                agent.update_from_miss(o[1], o[2])
                if not rally_had_contact:
                    n_ncl += 1
                rally_had_contact = False
            if reward == 1:
                agent.record_win()
                rally_had_contact = False

            if term or trunc:
                break

        agent.update_from_episode(score)
        scores.append(score)
        K = agent.long_term.n_components()
        rallies = int(agent.long_term.evidence())
        print(f"ep{ep+1:3d}  score={score:+3.0f}  contacts={n_contacts:3d}  "
              f"NCL={n_ncl:2d}  K={K:2d}  rallies={rallies:5d}  "
              f"placement={agent.placement:.4f}  "
              f"frames_near={agent.preferences._frames_near}  "
              f"(mean of last 10: {np.mean(scores[-10:]):+.2f})")

    env.close()
    print(f"\n{time.time()-t0:.0f}s   mean={np.mean(scores):+.2f}  "
          f"last-30 mean={np.mean(scores[-30:]):+.2f}  "
          f"wins(>0)={int((np.array(scores)>0).sum())}")

    # Save the persistent state for later benches.
    STATE_FILE_FEAT.parent.mkdir(parents=True, exist_ok=True)
    state = {
        "landing_bias": float(agent.landing_bias),
        "bias_dict":    {str(k): float(v) for k, v in agent._bias_dict.items()},
        "placement":    float(agent.placement),
        "interaction_alpha": agent.interaction_model.counts,
        "frames_near":  int(agent.preferences._frames_near),
        "urgency_ema":  float(agent._fell_short_frames_ema),
        "long_term":    agent.long_term.to_dict(),
        "rmm_feature":  FEATURE,   # for downstream consumers
    }
    with open(STATE_FILE_FEAT, "wb") as f:
        pickle.dump(state, f)
    with open(STATE_FILE_GENERIC, "wb") as f:
        pickle.dump(state, f)
    print(f"saved {STATE_FILE_FEAT}  (feature={FEATURE}  "
          f"K={agent.long_term.n_components()}  "
          f"rallies={int(agent.long_term.evidence())})")


if __name__ == "__main__":
    main()
