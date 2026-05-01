"""
Watch the EFE agent play Pong with a live learning metrics panel.

Layout:  | game (500px) | metrics (360px) |

Metrics panel shows (updated every frame):
  1. Prediction error  — rolling mean of ||innovation||, drops as ball model improves
  2. Obs noise R trace — sum of diag(R), adapts to actual sensor noise
  3. Landing bias      — EMA correction from observed misses
  4. Episode scores    — last 20 scores

State is saved to models/agent_state.pkl after every episode and reloaded on
startup, so landing bias (and VBGS/R when --learn) carry across restarts.

Usage:
    python play_efe.py              # frozen models, bias learning on
    python play_efe.py --learn      # also update VBGS and R online
    python play_efe.py --horizon 1
    python play_efe.py --reset      # wipe saved state and start fresh
"""

import argparse
import pathlib
import pickle
from collections import deque
import numpy as np
import pygame
import gymnasium as gym
import ale_py

from generative_model import (
    BallTransitionVBGS, OpponentTransitionVBGS, LikelihoodModel,
    PriorPreferences, OpponentBeliefTracker, MixtureBeliefFilter
)
from efe_agent import EFEAgent

gym.register_envs(ale_py)

GAMMA   = 0.9
GAME_W  = 500
GAME_H  = 500
PANEL_W = 360
WIN_W   = GAME_W + PANEL_W
WIN_H   = GAME_H
R_BATCH = 20
R_LR    = 0.005

# colours
BG       = (20,  20,  30)
GRID     = (40,  40,  55)
WHITE    = (220, 220, 220)
DIM      = (120, 120, 140)
C_INN    = (100, 200, 255)   # cyan  — innovation / prediction error
C_R      = (255, 180,  60)   # amber — R trace
C_SCORE  = ( 80, 220, 120)   # green — scores
C_WARN   = (255,  80,  80)   # red   — score < 0 bar fill


STATE_FILE = pathlib.Path("models/agent_state.pkl")


def save_state(landing_bias, ball_model=None, likelihood=None):
    state = {"landing_bias": float(landing_bias)}
    if ball_model is not None:
        state["ball_vbgs"] = ball_model.vbgs
    if likelihood is not None:
        state["R"] = likelihood.R.copy()
    STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(STATE_FILE, "wb") as f:
        pickle.dump(state, f)


def load_state():
    if STATE_FILE.exists():
        with open(STATE_FILE, "rb") as f:
            return pickle.load(f)
    return {}


def extract_obs(ram):
    return np.array([ram[49]/255., ram[54]/255., ram[51]/255., ram[50]/255.])


def make_models(state=None):
    ball_model  = BallTransitionVBGS.from_file()
    opp_model   = OpponentTransitionVBGS.from_file()
    likelihood  = LikelihoodModel()
    if state:
        if "ball_vbgs" in state:
            ball_model.vbgs = state["ball_vbgs"]
        if "R" in state:
            likelihood.R = state["R"]
    return ball_model, opp_model, likelihood


def make_episode_state(ball_model, opp_model, likelihood):
    opp_tracker = OpponentBeliefTracker(opp_model)
    belief      = MixtureBeliefFilter(ball_model, opp_tracker, likelihood)
    prefs       = PriorPreferences()
    agent       = EFEAgent(belief, likelihood, prefs)
    return opp_tracker, belief, agent


# ── metrics panel ─────────────────────────────────────────────────────────────

class MetricsPanel:
    MAXLEN_INN   = 400   # frames stored for innovation sparkline
    MAXLEN_R     = 200   # R-trace samples
    MAXLEN_SCORE =  20   # episode scores
    MAXLEN_BIAS  =  40   # one sample per miss event

    def __init__(self):
        self.inn_buf   = deque(maxlen=self.MAXLEN_INN)
        self.r_buf     = deque(maxlen=self.MAXLEN_R)
        self.score_buf = deque(maxlen=self.MAXLEN_SCORE)
        self.bias_buf  = deque(maxlen=self.MAXLEN_BIAS)
        self._inn_acc  = []
        self._miss_total = 0
        self._miss_short = 0
        self._gap_buf     = deque(maxlen=60)   # |paddle_gap| per recent miss
        self._short_bx_buf = deque(maxlen=60)  # bx at fell-short misses

    def push_innovation(self, inn: np.ndarray):
        self.inn_buf.append(float(np.linalg.norm(inn)))
        self._inn_acc.append(inn)

    def push_r_trace(self, R: np.ndarray):
        self.r_buf.append(float(np.trace(R)))

    def push_score(self, score: float):
        self.score_buf.append(score)

    def push_bias(self, bias: float):
        self.bias_buf.append(bias)

    def push_miss(self, fell_short: bool, gap: float, bx: float):
        """Record one miss event."""
        self._miss_total   += 1
        self._miss_short   += int(fell_short)
        self._gap_buf.append(abs(gap))
        if fell_short:
            self._short_bx_buf.append(bx)

    # ── drawing helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _sparkline(surf, values, rect, color, y_min=None, y_max=None):
        """Draw a line chart inside rect = (x, y, w, h)."""
        if len(values) < 2:
            return
        x0, y0, w, h = rect
        data  = list(values)
        lo    = min(data) if y_min is None else y_min
        hi    = max(data) if y_max is None else y_max
        span  = hi - lo if hi > lo else 1e-9

        pts = []
        for i, v in enumerate(data):
            px = x0 + int(i * w / (len(data) - 1))
            py = y0 + h - int((v - lo) / span * h)
            pts.append((px, py))
        pygame.draw.lines(surf, color, False, pts, 1)

    @staticmethod
    def _bar_chart(surf, values, rect, color_pos, color_neg, y_zero=0):
        """Horizontal bar chart for episode scores."""
        if not values:
            return
        x0, y0, w, h = rect
        n   = len(values)
        bw  = max(1, w // n - 1)
        lo  = min(min(values), y_zero - 1)
        hi  = max(max(values), y_zero + 1)
        span = hi - lo

        zero_py = y0 + h - int((y_zero - lo) / span * h)
        pygame.draw.line(surf, GRID, (x0, zero_py), (x0 + w, zero_py), 1)

        for i, v in enumerate(values):
            bx    = x0 + i * (bw + 1)
            bar_h = abs(int((v - y_zero) / span * h))
            color = color_pos if v >= y_zero else color_neg
            if v >= y_zero:
                pygame.draw.rect(surf, color, (bx, zero_py - bar_h, bw, bar_h))
            else:
                pygame.draw.rect(surf, color, (bx, zero_py, bw, bar_h))

    def draw(self, surf, font_sm, font_xs, learn: bool):
        """Render the full panel onto surf (width=PANEL_W, height=WIN_H)."""
        surf.fill(BG)
        W, H = surf.get_size()
        pad  = 8
        n_panels = 4
        ph   = (H - (n_panels + 1) * pad) // n_panels

        learn_lbl = "ONLINE LEARNING" if learn else "FROZEN"
        learn_col = C_SCORE if learn else C_WARN
        lbl = font_sm.render(learn_lbl, True, learn_col)
        surf.blit(lbl, (W // 2 - lbl.get_width() // 2, 2))

        y = lbl.get_height() + 2
        if self._miss_total > 0:
            pct = 100 * self._miss_short / self._miss_total
            avg_gap = np.mean(self._gap_buf) * 255 if self._gap_buf else 0.0
            miss_lbl = font_xs.render(
                f"misses: {self._miss_short}/{self._miss_total} fell-short"
                f" ({pct:.0f}%)  gap≈{avg_gap:.1f}px", True, C_WARN)
            surf.blit(miss_lbl, (pad, y))
            y += miss_lbl.get_height() + 2
            if self._short_bx_buf:
                avg_bx = np.mean(self._short_bx_buf) * 255
                bx_lbl = font_xs.render(
                    f"  fell-short bx≈{avg_bx:.0f}px  "
                    f"(>{200}px = close-wall bounce)", True, DIM)
                surf.blit(bx_lbl, (pad, y))
                y += bx_lbl.get_height() + 2
        y += pad // 2

        def _panel(title, buf, col, y_min, y_max):
            nonlocal y
            pygame.draw.rect(surf, GRID, (pad, y, W - 2*pad, ph), 1)
            surf.blit(font_xs.render(title, True, DIM), (pad + 4, y + 3))
            if len(buf) >= 2:
                rect = (pad + 2, y + 18, W - 2*pad - 4, ph - 22)
                self._sparkline(surf, buf, rect, col, y_min, y_max)
                last = font_xs.render(f"{list(buf)[-1]:.4f}", True, col)
                surf.blit(last, (pad + 4, y + ph - last.get_height() - 3))
            y += ph + pad

        _panel("Prediction error  |ν|",  self.inn_buf,  C_INN,  None, None)
        _panel("Obs noise  tr(R)",        self.r_buf,    C_R,    0.0,  None)

        # Landing bias — one sample per miss, centred on 0
        pygame.draw.rect(surf, GRID, (pad, y, W - 2*pad, ph), 1)
        surf.blit(font_xs.render("Landing bias  (per miss)", True, DIM),
                  (pad + 4, y + 3))
        if len(self.bias_buf) >= 2:
            rect = (pad + 2, y + 18, W - 2*pad - 4, ph - 22)
            self._sparkline(surf, self.bias_buf, rect, C_WARN, -0.3, 0.3)
            # zero line
            _, ry, rw, rh = rect
            zero_py = ry + rh // 2
            pygame.draw.line(surf, GRID, (rect[0], zero_py),
                             (rect[0] + rw, zero_py), 1)
            last_b = list(self.bias_buf)[-1]
            bv = font_xs.render(f"{last_b:+.4f}", True, C_WARN)
            surf.blit(bv, (pad + 4, y + ph - bv.get_height() - 3))
        elif self.bias_buf:
            bv = font_xs.render(f"{list(self.bias_buf)[-1]:+.4f}  (waiting for misses…)",
                                True, DIM)
            surf.blit(bv, (pad + 4, y + ph // 2))
        else:
            surf.blit(font_xs.render("waiting for first miss…", True, DIM),
                      (pad + 4, y + ph // 2))
        y += ph + pad

        # Episode scores bar chart
        pygame.draw.rect(surf, GRID, (pad, y, W - 2*pad, ph), 1)
        surf.blit(font_xs.render(f"Episode scores  (last {self.MAXLEN_SCORE})", True, DIM),
                  (pad + 4, y + 3))
        if self.score_buf:
            rect = (pad + 2, y + 18, W - 2*pad - 4, ph - 22)
            self._bar_chart(surf, list(self.score_buf), rect, C_SCORE, C_WARN)
            m = font_xs.render(f"mean {np.mean(self.score_buf):+.1f}", True, C_SCORE)
            surf.blit(m, (pad + 4, y + ph - m.get_height() - 3))


# ── main loop ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--horizon", type=int, default=3)
    parser.add_argument("--learn",   action="store_true",
                        help="also update VBGS and R online (default: bias only)")
    parser.add_argument("--reset",   action="store_true",
                        help="wipe models/agent_state.pkl and start fresh")
    args  = parser.parse_args()
    learn = args.learn

    if args.reset and STATE_FILE.exists():
        STATE_FILE.unlink()
        print("Cleared saved state.")

    state = load_state()
    if state:
        print(f"Loaded state: landing_bias={state.get('landing_bias', 0.0):+.4f}"
              + ("  ball_vbgs ✓" if "ball_vbgs" in state else "")
              + ("  R ✓"        if "R"         in state else ""))
    else:
        print("No saved state — starting fresh.")

    env = gym.make("ALE/Pong-v5", render_mode="rgb_array", obs_type="ram")

    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H), pygame.RESIZABLE)
    pygame.display.set_caption(
        f"EFE agent  H={args.horizon}  {'[online]' if learn else '[frozen+bias]'}")
    clock   = pygame.time.Clock()
    font_sm = pygame.font.SysFont("monospace", 13, bold=True)
    font_xs = pygame.font.SysFont("monospace", 11)

    ball_model, opp_model, likelihood = make_models(state if learn else None)
    opp_tracker, belief, agent = make_episode_state(ball_model, opp_model, likelihood)
    agent.landing_bias = state.get("landing_bias", 0.0)

    obs, _ = env.reset()
    o = extract_obs(obs)
    belief.initialise(o)

    metrics     = MetricsPanel()
    if agent.landing_bias != 0.0:
        metrics.push_bias(agent.landing_bias)
    prev_opp_y  = o[3]
    prev_action = 0
    prev_ball   = None
    inn_acc     = []
    episode     = 1
    score       = 0

    print(f"EFE agent  H={args.horizon}  γ={GAMMA}"
          f"  {'online' if learn else 'frozen+bias'}  — close window to quit\n")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                save_state(agent.landing_bias,
                           ball_model if learn else None,
                           likelihood if learn else None)
                print(f"Saved state  bias={agent.landing_bias:+.4f}")
                env.close(); pygame.quit(); return

        o            = extract_obs(obs)
        ball_in_play = o[0] > 0.05
        opp_y        = o[3]

        if ball_in_play:
            agent.update_raw_velocity(o[0], o[1])
            ball_state_4 = belief.mean[:4].copy()

            # Separate predict/correct to expose innovation
            belief.predict(prev_action)
            inn = o - likelihood.H @ belief.mean
            belief.correct(o)

            opp_tracker.update(prev_opp_y, ball_state_4, prev_action, opp_y)

            metrics.push_innovation(inn)

            inn_acc.append(inn)
            if len(inn_acc) >= R_BATCH:
                if learn:
                    likelihood.update_R(np.array(inn_acc), lr=R_LR)
                metrics.push_r_trace(likelihood.R)
                inn_acc.clear()

            if learn:
                curr_ball = belief.mean[:4].copy()
                if prev_ball is not None:
                    ball_model.update(prev_ball, curr_ball)
                prev_ball = curr_ball
        else:
            belief.initialise(o)
            agent.reset_raw_velocity()
            prev_ball = None
            inn_acc.clear()

        action, _ = agent.select_action_horizon(horizon=args.horizon, gamma=GAMMA)
        prev_opp_y  = opp_y
        prev_action = action

        # ── render game ───────────────────────────────────────────────────────
        frame   = env.render()
        surface = pygame.surfarray.make_surface(frame.transpose(1, 0, 2))
        cur_w, cur_h = screen.get_size()
        game_w  = cur_w - PANEL_W
        game_surf = pygame.transform.scale(surface, (game_w, cur_h))
        screen.blit(game_surf, (0, 0))

        # ── render metrics panel ──────────────────────────────────────────────
        panel_surf = pygame.Surface((PANEL_W, cur_h))
        metrics.draw(panel_surf, font_sm, font_xs, learn)
        screen.blit(panel_surf, (game_w, 0))

        pygame.display.flip()
        clock.tick(60)

        # ── step env ──────────────────────────────────────────────────────────
        obs, reward, terminated, truncated, _ = env.step(action)
        score += reward

        if reward == -1:
            fell_short, gap = agent.update_from_miss(o[1], o[2])
            metrics.push_miss(fell_short, gap, o[0])
            metrics.push_bias(agent.landing_bias)

        if terminated or truncated:
            metrics.push_score(score)
            saved_bias = agent.landing_bias
            print(f"  episode {episode}: {score:+.0f}"
                  f"  bias={saved_bias:+.4f}"
                  f"  (mean {np.mean(metrics.score_buf):+.1f})")
            save_state(saved_bias,
                       ball_model if learn else None,
                       likelihood if learn else None)
            episode += 1
            score = 0
            obs, _ = env.reset()
            if not learn:
                ball_model, opp_model, likelihood = make_models()
            opp_tracker, belief, agent = make_episode_state(
                ball_model, opp_model, likelihood)
            agent.landing_bias = saved_bias
            o = extract_obs(obs)
            belief.initialise(o)
            prev_opp_y  = o[3]
            prev_action = 0
            prev_ball   = None
            inn_acc.clear()


if __name__ == "__main__":
    main()
