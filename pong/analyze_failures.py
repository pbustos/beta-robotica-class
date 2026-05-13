"""
Per-rally failure-mode analyzer.

Runs the production agent (target_y feature, fixed H=3, warm pkl) for N
episodes and captures full per-rally diagnostics:

  - Was the ball contacted at all?
  - If yes: where on the paddle, what was the locked target, where was opp,
    did we win?
  - If no: was it fell-short (paddle didn't reach in time) or aim error
    (paddle reached but in the wrong place)?

The aim is to find the dominant failure mode among the ~30% of rallies
the agent loses, so future interventions target a specific population
instead of the whole loss bucket.

Usage:
    python analyze_failures.py [N_EPISODES]   (default 30)
"""

import sys, copy, pickle, pathlib, time
import numpy as np
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter, _PLAYER_X)
from efe_agent import EFEAgent

gym.register_envs(ale_py)

N_EPISODES = int(sys.argv[1]) if len(sys.argv) > 1 else 30
GAMMA      = 0.9
SEED_BASE  = 31415
STATE_FILE = pathlib.Path("models/agent_state.pkl")


def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])


def load_warm():
    if not STATE_FILE.exists():
        return {}
    with open(STATE_FILE, "rb") as f:
        return pickle.load(f)


def apply_warm(agent, state):
    from efe_agent import LongTermStats
    if "long_term" in state and state["long_term"] is not None:
        agent.long_term = LongTermStats.from_dict(copy.deepcopy(state["long_term"]))
    bd = state.get("bias_dict")
    if bd is not None:
        for raw_k, v in bd.items():
            k = tuple(int(x) for x in raw_k.strip("()").split(", "))
            if k in agent._bias_dict:
                agent._bias_dict[k] = float(v)
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


def main():
    env = gym.make("ALE/Pong-v5", obs_type="ram", render_mode=None)
    ball_m = BallTransitionVBGS.from_file()
    opp_m  = OpponentTransitionVBGS.from_file()
    lik    = LikelihoodModel()

    # Persistent agent (mirrors bench semantics).
    opp_t  = OpponentBeliefTracker(opp_m)
    belief = MixtureBeliefFilter(ball_m, opp_t, lik)
    prefs  = PriorPreferences()
    agent  = EFEAgent(belief, lik, prefs)
    apply_warm(agent, load_warm())
    print(f"agent loaded: rMM K={agent.long_term.n_components()}  "
          f"rallies={int(agent.long_term.evidence())}  "
          f"placement={agent.placement:.4f}  "
          f"frames_near={agent.preferences._frames_near}")

    rallies = []
    t0 = time.time()

    for ep in range(N_EPISODES):
        opp_t  = OpponentBeliefTracker(opp_m)
        belief = MixtureBeliefFilter(ball_m, opp_t, lik)
        agent.belief = belief
        agent.reset_raw_velocity()
        agent._target_y_locked = None
        agent._locked_offset   = None

        obs, _ = env.reset(seed=SEED_BASE + ep)
        o = extract_obs(obs)
        belief.initialise(o)
        prev_action = 0
        prev_opp_y  = o[3]
        bounced_this_approach = False
        rally = None    # active rally dict

        while True:
            o            = extract_obs(obs)
            ball_in_play = o[0] > 0.05
            opp_y        = o[3]

            if ball_in_play:
                prev_raw_vx = agent._raw_vx
                agent.update_raw_velocity(o[0], o[1])
                if agent._last_vy_bounce:
                    bounced_this_approach = True
                    if rally is not None:
                        rally["bounces_during_approach"] += 1

                # Open a new rally on first in-play frame after serve.
                if rally is None:
                    rally = {
                        "frames": 0, "bounces_during_approach": 0,
                        "approach_started_bx": float(o[0]),
                        "contact": False,
                        "contact_target_y_locked": None,
                        "contact_landing_pred": None,
                        "contact_ball_y_actual": None,
                        "contact_paddle_y": None,
                        "contact_offset": None,
                        "contact_opp_y_actual": None,
                        "vy_sign_at_contact": None,
                        "bounced_at_contact": False,
                        "miss_paddle_y": None,
                        "miss_ball_y": None,
                        "miss_landing_pred": None,
                        "miss_paddle_gap": None,
                        "miss_fell_short": None,
                        "outcome": None,    # 'win' | 'loss_contact' | 'loss_ncl'
                    }
                rally["frames"] += 1

                # Paddle contact detection (vx flip near player side).
                if (prev_raw_vx > 0.008 and agent._raw_vx < -0.008
                        and o[0] > 0.60):
                    agent.interaction_model.observe(o[1] - o[2])
                    agent.update_preferences_on_contact(
                        o[2],
                        vy_sign=int(np.sign(agent._raw_vy)),
                        bounced=int(bounced_this_approach),
                        opp_y=float(o[3]))
                    rally["contact"] = True
                    rally["contact_target_y_locked"] = agent._target_y_locked
                    rally["contact_landing_pred"]    = agent._last_landing_pred
                    rally["contact_ball_y_actual"]   = float(o[1])
                    rally["contact_paddle_y"]        = float(o[2])
                    rally["contact_offset"]          = float(o[1] - o[2])
                    rally["contact_opp_y_actual"]    = float(o[3])
                    rally["vy_sign_at_contact"]      = int(np.sign(agent._raw_vy))
                    rally["bounced_at_contact"]      = bool(bounced_this_approach)
                    bounced_this_approach = False

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

            if reward == -1:
                fell_short, gap = agent.update_from_miss(o[1], o[2])
                if rally is not None:
                    rally["miss_paddle_y"]   = float(o[2])
                    rally["miss_ball_y"]     = float(o[1])
                    rally["miss_landing_pred"] = float(agent._last_landing_pred)
                    rally["miss_paddle_gap"] = float(gap)
                    rally["miss_fell_short"] = bool(fell_short)
                    rally["outcome"] = ("loss_contact" if rally["contact"]
                                         else "loss_ncl")
                    rallies.append(rally)
                    rally = None
            if reward == 1:
                agent.record_win()
                if rally is not None:
                    rally["outcome"] = "win"
                    rallies.append(rally)
                    rally = None

            if term or trunc:
                break

        agent.update_from_episode(0)   # no score-trend update needed for analysis
        print(f"  ep {ep+1:2d}: rallies so far = {len(rallies)}")

    env.close()
    print(f"\n{time.time()-t0:.0f}s   total rallies = {len(rallies)}\n")
    analyze(rallies)


def analyze(rallies):
    n = len(rallies)
    if n == 0:
        print("no rallies"); return

    wins   = [r for r in rallies if r["outcome"] == "win"]
    lwc    = [r for r in rallies if r["outcome"] == "loss_contact"]
    ncl    = [r for r in rallies if r["outcome"] == "loss_ncl"]
    fell   = [r for r in ncl if r["miss_fell_short"]]
    aim    = [r for r in ncl if not r["miss_fell_short"]]

    print(f"── Rally outcomes ({n} total) ──")
    print(f"  wins                {len(wins):5d}  {100*len(wins)/n:5.1f}%")
    print(f"  loss after contact  {len(lwc):5d}  {100*len(lwc)/n:5.1f}%")
    print(f"  loss no-contact     {len(ncl):5d}  {100*len(ncl)/n:5.1f}%")
    print(f"    └ fell-short      {len(fell):5d}  {100*len(fell)/n:5.1f}%  "
          f"(paddle didn't reach landing)")
    print(f"    └ aim error       {len(aim):5d}  {100*len(aim)/n:5.1f}%  "
          f"(paddle reached, landing was elsewhere)")

    if fell:
        gaps = np.array([abs(r["miss_paddle_gap"]) for r in fell])
        print(f"\n── fell-short paddle gaps ──")
        print(f"  mean |gap|={gaps.mean():.3f}  median={np.median(gaps):.3f}  "
              f"max={gaps.max():.3f}")
        print(f"  >0.18 (>3 paddle steps): {(gaps>0.18).sum()}  "
              f">0.30: {(gaps>0.30).sum()}")

    if aim:
        # Aim errors: landing prediction was wrong. How wrong?
        err = np.array([r["miss_ball_y"] - r["miss_landing_pred"] for r in aim])
        nb  = np.array([r["bounces_during_approach"] for r in aim])
        print(f"\n── aim-error landing-prediction residuals ──")
        print(f"  mean(ball-pred)={err.mean():+.3f}  std={err.std():.3f}  "
              f"|mean|={abs(err).mean():.3f}")
        print(f"  by bounces during approach:")
        for b in sorted(np.unique(nb)):
            mask = nb == b
            n_ = int(mask.sum())
            e_ = err[mask]
            print(f"    {b} bounces: n={n_:4d}  mean={e_.mean():+.3f}  "
                  f"std={e_.std():.3f}  |mean|={abs(e_).mean():.3f}")

    no_contact_wins = [r for r in wins if not r["contact"]]
    if no_contact_wins:
        print(f"\n  (of which: {len(no_contact_wins)} wins came from "
              f"opp missing without us contacting)")

    if wins or lwc:
        # Win-rate among contacted rallies, sliced by contact location.
        contacted = [r for r in (wins + lwc) if r["contact"]]
        offs = np.array([r["contact_offset"] for r in contacted])
        won  = np.array([r["outcome"] == "win" for r in contacted])
        print(f"\n── post-contact win-rate by contact_offset ──")
        bins = np.linspace(-0.08, 0.08, 9)
        for i in range(len(bins)-1):
            lo, hi = bins[i], bins[i+1]
            mask = (offs >= lo) & (offs < hi)
            ntot = int(mask.sum())
            if ntot > 0:
                wr = won[mask].mean()
                print(f"  offset [{lo:+.3f}, {hi:+.3f}]  n={ntot:4d}  "
                      f"win_rate={wr:.3f}")

        # Win-rate by opp_y at contact.
        opps = np.array([r["contact_opp_y_actual"] for r in contacted])
        print(f"\n── post-contact win-rate by opp_y_at_contact ──")
        bins = np.linspace(0.0, 1.0, 7)
        for i in range(len(bins)-1):
            lo, hi = bins[i], bins[i+1]
            mask = (opps >= lo) & (opps < hi)
            ntot = int(mask.sum())
            if ntot > 0:
                wr = won[mask].mean()
                print(f"  opp_y [{lo:.3f}, {hi:.3f}]   n={ntot:4d}  "
                      f"win_rate={wr:.3f}")

        # Per-(vy_sign, bounced) post-contact win-rate.
        print(f"\n── post-contact win-rate by (vy_sign, bounced_during_approach) ──")
        for vy in (-1, 0, 1):
            for b in (0, 1):
                mask = np.array([
                    (r["vy_sign_at_contact"] == vy) and
                    (int(r["bounced_at_contact"]) == b)
                    for r in contacted])
                ntot = int(mask.sum())
                if ntot > 0:
                    wr = won[mask].mean()
                    print(f"  vy_sign={vy:+d}  bounced={b}  n={ntot:4d}  "
                          f"win_rate={wr:.3f}")

    if ncl:
        # Approach geometry of NCL rallies — were they bounce-heavy?
        nb = np.array([r["bounces_during_approach"] for r in ncl])
        print(f"\n── NCL rallies: approach bounces ──")
        for b in sorted(np.unique(nb)):
            n_ = int((nb == b).sum())
            print(f"  {b} bounces during approach: {n_}  ({100*n_/len(ncl):.1f}%)")


if __name__ == "__main__":
    main()
