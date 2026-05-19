"""
Measure: at the crucial frame of each rally (paddle contact for wins /
lwc; miss frame for ncl), how long since the ball's last wall (vy)
bounce?

Hypothesis (symmetric to the opp side): our tracker also has 1-2
frames of effective lag (raw EMA velocity + bias_dict update), so an
incoming ball that bounced close to our paddle line is harder for us
to catch — for the same reason late-bounce returns are hard for opp.

If NCL rallies have systematically more recent bounces than successful
contacts, the hypothesis is confirmed.

Usage:
    python measure_bounce_vs_miss.py [N_EPISODES]   (default 30)
"""
import sys, copy, pickle, pathlib, time
import numpy as np
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter)
from efe_agent import EFEAgent, LongTermStats

gym.register_envs(ale_py)

N_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 30
GAMMA      = 0.9
SEED_BASE  = 31415   # same as analyze_failures.py


def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])


def main():
    env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
    bm = BallTransitionVBGS.from_file()
    om = OpponentTransitionVBGS.from_file()
    lk = LikelihoodModel()

    state_file = pathlib.Path("models/agent_state.pkl")
    warm = {}
    if state_file.exists():
        with open(state_file, "rb") as f:
            warm = pickle.load(f)

    opp_t  = OpponentBeliefTracker(om)
    belief = MixtureBeliefFilter(bm, opp_t, lk)
    prefs  = PriorPreferences()
    agent  = EFEAgent(belief, lk, prefs)
    if "long_term" in warm:
        agent.long_term = LongTermStats.from_dict(copy.deepcopy(warm["long_term"]))

    # Per-event records.
    #   FSLB = frames since the ball's last vy (wall) bounce.
    # All are recorded at the moment of reward==±1 (i.e., the frame
    # we either miss or score).
    contact_fslb   = []   # at every paddle contact (informational)
    miss_ncl_fslb  = []   # at miss frame, NCL rallies (no prior contact)
    miss_lwc_fslb  = []   # at miss frame, LWC rallies (we contacted earlier)
    win_fslb       = []   # at score frame, our winning rallies

    t0 = time.time()

    for ep in range(N_EPISODES):
        opp_t  = OpponentBeliefTracker(om)
        belief = MixtureBeliefFilter(bm, opp_t, lk)
        agent.belief = belief
        agent.reset_raw_velocity()
        agent._target_y_locked = None
        agent._locked_offset   = None

        obs, _ = env.reset(seed=SEED_BASE + ep)
        o = extract_obs(obs)
        belief.initialise(o)
        prev_action = 0
        prev_opp_y  = o[3]

        # Per-rally state
        rally_frame        = 0       # frame count within the current rally
        last_bounce_frame  = -999    # frame of last vy bounce (in rally_frame units)
        had_contact_this_rally = False    # True if WE've contacted the ball this rally

        while True:
            o = extract_obs(obs)
            ball_in_play = o[0] > 0.05

            if ball_in_play:
                prev_raw_vx = agent._raw_vx
                agent.update_raw_velocity(o[0], o[1])

                rally_frame += 1
                if agent._last_vy_bounce:
                    last_bounce_frame = rally_frame

                # Paddle contact detection
                if (prev_raw_vx > 0.008 and agent._raw_vx < -0.008
                        and o[0] > 0.60):
                    agent.interaction_model.observe(o[1] - o[2])
                    agent.update_preferences_on_contact(
                        o[2], vy_sign=int(np.sign(agent._raw_vy)),
                        bounced=0, opp_y=float(o[3]))
                    fslb = rally_frame - last_bounce_frame
                    contact_fslb.append(fslb)
                    had_contact_this_rally = True

                ball_state_4 = belief.mean[:4].copy()
                belief.predict(prev_action)
                belief.correct(o)
                opp_t.update(prev_opp_y, ball_state_4, prev_action, o[3])
            else:
                # Ball out of play (between rallies). Reset per-rally tracker.
                belief.initialise(o)
                agent.reset_raw_velocity()
                rally_frame        = 0
                last_bounce_frame  = -999
                had_contact_this_rally = False

            action, _ = agent.select_action_horizon(horizon=3, gamma=GAMMA)
            prev_opp_y = o[3]
            prev_action = action
            obs, reward, term, trunc, _ = env.step(action)

            # FSLB at the moment of reward (the frame we miss or score).
            # `o` still holds the state of the frame the reward fires on.
            fslb_at_event = rally_frame - last_bounce_frame
            if reward == -1:
                if had_contact_this_rally:
                    miss_lwc_fslb.append(fslb_at_event)
                else:
                    miss_ncl_fslb.append(fslb_at_event)
                agent.update_from_miss(o[1], o[2])
            if reward == 1:
                win_fslb.append(fslb_at_event)
                agent.record_win()

            if term or trunc: break

        print(f"  ep {ep+1:2d}: contacts={len(contact_fslb)}  "
              f"ncl_misses={len(miss_ncl_fslb)}  "
              f"lwc_misses={len(miss_lwc_fslb)}  wins={len(win_fslb)}")

    env.close()
    print(f"\n{time.time()-t0:.0f}s\n")

    def report(name, arr):
        a = np.array(arr)
        if len(a) == 0:
            print(f"  {name}  (no data)"); return
        print(f"  {name:<22}  n={len(a):4d}  "
              f"mean={a.mean():6.2f}  median={np.median(a):5.1f}  "
              f"P10={np.percentile(a,10):5.1f}  "
              f"P25={np.percentile(a,25):5.1f}  "
              f"P75={np.percentile(a,75):5.1f}")
        # Distribution by bucket
        buckets = [(0, 3), (3, 6), (6, 10), (10, 20), (20, 999)]
        for lo, hi in buckets:
            n = int(((a >= lo) & (a < hi)).sum())
            pct = 100 * n / len(a)
            print(f"      fslb ∈ [{lo:>2}, {hi:>3}):  {n:4d}  ({pct:5.1f}%)")

    print("Frames-since-last-bounce distributions:")
    print()
    report("at our contact (any)", contact_fslb)
    print()
    report("at NCL miss", miss_ncl_fslb)
    print()
    report("at LWC miss (post-contact)", miss_lwc_fslb)
    print()
    report("at our contact in a winning rally", win_fslb)


if __name__ == "__main__":
    main()
