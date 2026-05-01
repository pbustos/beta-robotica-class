"""
EFE-based action selection — Phases 3.1 / 3.2 / 3.3.

G(a) = G_instr(a) - λ·G_epist(a)

Phase 3.3 adds N-step planning using a receding-horizon tree search:
  - Step 0: VBGS predict (wall-bounce-aware) — only 3 calls, one per action.
  - Steps 1..N-1: linear-Gaussian rollout via recursive tree search.

Prior preferences use an adaptive player-y precision:
  σ_py shrinks as ball approaches player_x (ball far → soft, ball close → tight).
  This encodes the intuition that the agent should commit to an intercept
  position more firmly as the prediction horizon shortens and the estimate
  becomes more reliable.
"""

import numpy as np
from generative_model import MixtureBeliefFilter, LikelihoodModel, PriorPreferences, PADDLE_DY

# Constant-velocity ball dynamics; paddle/opponent carry forward.
_F = np.eye(6)
_F[0, 2] = 1.0   # bx += vx
_F[1, 3] = 1.0   # by += vy


class EFEAgent:
    """
    EFE action selection with optional N-step planning horizon.

    G(π) = Σ_{t=0}^{N-1} γ^t · [G_instr(a_t|s_t) - λ · G_epist(a_t|s_t)]

    Step 0  — full VBGS predict (wall-bounce-aware).
    Step 1+ — linear-Gaussian predict (fast; large uncertainty at depth > 1).

    Action a* = first action of argmin_π G(π).
    """

    ACTIONS = [0, 2, 3]   # NOOP, UP, DOWN
    I_PY    = 4            # player-y index in 6D state

    _I_PY_OBS = 2   # player-y index in 4D observation (o_star)

    def __init__(self, belief: MixtureBeliefFilter,
                 likelihood: LikelihoodModel,
                 preferences: PriorPreferences,
                 vel_alpha: float = 0.7,
                 bias_alpha: float = 0.15):
        self.belief      = belief
        self.likelihood  = likelihood
        self.preferences = preferences
        self._vel_alpha  = vel_alpha
        self._raw_vx     = 0.0
        self._raw_vy     = 0.0
        self._prev_bx    = None
        self._prev_by    = None
        # Landing-bias: EMA correction applied to predicted landing y.
        # Updated from observed misses: bias += α·(actual_y − predicted_y).
        self._bias_alpha        = bias_alpha
        self.landing_bias       = 0.0   # public so play_efe.py can display it
        self._last_landing_pred = 0.5

    # ── raw velocity tracker ──────────────────────────────────────────────────

    # Minimum |dv| to treat a sign flip as a real bounce rather than noise.
    # Normal ball speed ≈ 0.015–0.025/frame; serve jitter is near-zero.
    _BOUNCE_THRESH = 0.008

    def update_raw_velocity(self, bx: float, by: float):
        """
        Call once per frame with the latest raw ball position (normalised).

        On normal frames: EMA blend (α=vel_alpha) to suppress quantisation noise.
        On bounce frames: hard-reset to the new frame-difference velocity so the
        landing prediction immediately reflects the reversed direction.

        Bounce detection: the new dv has the opposite sign to the current EMA
        estimate AND its magnitude exceeds _BOUNCE_THRESH (rules out noise).
        """
        if self._prev_bx is not None:
            dvx = float(np.clip(bx - self._prev_bx, -0.06, 0.06))
            dvy = float(np.clip(by - self._prev_by, -0.06, 0.06))

            vx_bounce = (dvx * self._raw_vx < 0) and abs(dvx) > self._BOUNCE_THRESH
            vy_bounce = (dvy * self._raw_vy < 0) and abs(dvy) > self._BOUNCE_THRESH

            if vx_bounce or vy_bounce:
                # Direction reversed — accept new velocity instantly
                self._raw_vx = dvx
                self._raw_vy = dvy
            else:
                a = self._vel_alpha
                self._raw_vx = a * dvx + (1.0 - a) * self._raw_vx
                self._raw_vy = a * dvy + (1.0 - a) * self._raw_vy

        self._prev_bx = bx
        self._prev_by = by

    def reset_raw_velocity(self):
        """Call when the ball goes out of play (serve / reset)."""
        self._raw_vx  = 0.0
        self._raw_vy  = 0.0
        self._prev_bx = None
        self._prev_by = None

    # ── belief state save / restore ───────────────────────────────────────────

    def _save_belief(self):
        b = self.belief
        return b.mu.copy(), b.Sigma.copy(), b.weights.copy()

    def _restore_belief(self, state):
        self.belief.mu, self.belief.Sigma, self.belief.weights = state

    # ── shared preference computation ─────────────────────────────────────────

    # Paddle gap above which the miss is classified as "fell short" rather
    # than a prediction error.  0.016/frame × ~2 frames slack = 0.032.
    _FELL_SHORT_THRESH = 0.032

    def update_from_miss(self, ball_y: float, player_y: float):
        """
        Call with the normalised ball_y and player_y at the frame a point was
        lost (reward == -1).

        Returns (fell_short, paddle_gap):
          fell_short  — True when the paddle didn't reach the target in time
          paddle_gap  — signed distance: player_y − predicted_landing
                        (positive = paddle below target, negative = above)

        Bias update is skipped on fell-short misses: the prediction may have
        been correct; the problem was timing, not aim.
        """
        paddle_gap  = float(player_y) - self._last_landing_pred
        fell_short  = abs(paddle_gap) > self._FELL_SHORT_THRESH

        if not fell_short:
            error = float(ball_y) - self._last_landing_pred
            self.landing_bias = float(np.clip(
                self._bias_alpha * error + (1.0 - self._bias_alpha) * self.landing_bias,
                -0.3, 0.3))

        return fell_short, paddle_gap

    def _get_preferences(self):
        """
        Compute (o_star, C_inv, log_norm) once per step.
        Uses EMA raw velocity for the landing prediction.
        Applies the accumulated landing_bias to correct systematic prediction errors.
        Stores _last_landing_pred (pre-bias) for update_from_miss().
        """
        raw_vel = (self._raw_vx, self._raw_vy) if self._prev_bx is not None else None
        o_star, C_inv, log_norm = self.preferences.contextual_target(
            self.belief.mean, raw_vel=raw_vel)
        self._last_landing_pred = float(o_star[self._I_PY_OBS])
        if self.landing_bias != 0.0:
            o_star = o_star.copy()
            o_star[self._I_PY_OBS] = float(
                np.clip(o_star[self._I_PY_OBS] + self.landing_bias, 0.0, 1.0))
        return o_star, C_inv, log_norm

    # ── VBGS-based single-step EFE + predicted state ──────────────────────────

    def _vbgs_efe(self, action: int, o_star, C_inv, log_norm):
        """
        Full VBGS virtual predict, then EFE with pre-computed preferences.
        Returns (G_instr, G_epist, mu_pred_6D, Sigma_pred_6x6).
        """
        saved = self._save_belief()
        self.belief.predict(action)
        mu_pred    = self.belief.mean.copy()
        Sigma_pred = self.belief.cov.copy()
        self._restore_belief(saved)

        H, R = self.likelihood.H, self.likelihood.R
        mu_o = H @ mu_pred
        S    = H @ Sigma_pred @ H.T + R

        g_instr = -self.preferences.expected_log_prior(mu_o, S, o_star,
                                                        C_inv, log_norm)
        _, logdet_S = np.linalg.slogdet(S)
        _, logdet_R = np.linalg.slogdet(R)
        g_epist     = 0.5 * (logdet_S - logdet_R)

        return g_instr, g_epist, mu_pred, Sigma_pred

    # ── linear-Gaussian single-step EFE (fast rollout) ───────────────────────

    def _linear_efe(self, mu: np.ndarray, Sigma: np.ndarray, action: int,
                    o_star, C_inv, log_norm):
        """
        Cheap one-step EFE using constant-velocity ball dynamics.
        Preferences are passed in (fixed across the whole tree for consistency).
        Returns (G_instr, G_epist, mu_next, Sigma_next).
        """
        Q = self.belief.Q
        delta = np.zeros(6)
        delta[self.I_PY] = PADDLE_DY.get(action, 0.0)

        mu_pred    = _F @ mu + delta
        mu_pred[self.I_PY] = np.clip(mu_pred[self.I_PY], 0.0, 1.0)
        Sigma_pred = _F @ Sigma @ _F.T + Q

        H, R = self.likelihood.H, self.likelihood.R
        mu_o = H @ mu_pred
        S    = H @ Sigma_pred @ H.T + R

        g_instr = -self.preferences.expected_log_prior(mu_o, S, o_star,
                                                        C_inv, log_norm)
        _, logdet_S = np.linalg.slogdet(S)
        _, logdet_R = np.linalg.slogdet(R)
        g_epist     = 0.5 * (logdet_S - logdet_R)

        return g_instr, g_epist, mu_pred, Sigma_pred

    # ── recursive tree search (linear-Gaussian, steps 1+) ────────────────────

    def _tree_min_efe(self, mu, Sigma, depth, ew, discount,
                      o_star, C_inv, log_norm):
        """Min discounted EFE over a depth-step linear-Gaussian tree."""
        best = np.inf
        for a in self.ACTIONS:
            g_i, g_e, mu_n, Sigma_n = self._linear_efe(
                mu, Sigma, a, o_star, C_inv, log_norm)
            g = discount * (g_i - ew * g_e)
            if depth > 1:
                g += self._tree_min_efe(mu_n, Sigma_n, depth - 1, ew,
                                        discount * self._gamma,
                                        o_star, C_inv, log_norm)
            if g < best:
                best = g
        return best

    # ── public action selection ───────────────────────────────────────────────

    def select_action(self, epistemic_weight: float = 0.0):
        """
        1-step EFE (Phase 3.1/3.2).
        Returns (best_action, {action: (G_total, G_instr, G_epist)}).
        """
        o_star, C_inv, log_norm = self._get_preferences()
        results = {}
        for a in self.ACTIONS:
            g_i, g_e, _, _ = self._vbgs_efe(a, o_star, C_inv, log_norm)
            results[a] = (g_i - epistemic_weight * g_e, g_i, g_e)
        best = min(results, key=lambda a: results[a][0])
        return best, results

    def select_action_horizon(self, horizon: int = 3,
                              epistemic_weight: float = 0.0,
                              gamma: float = 0.9):
        """
        N-step planning (Phase 3.3).

        Evaluates 3 VBGS step-0 calls + recursive linear-Gaussian tree.
        Preferences fixed at current belief (consistent across all branches).
        Returns (best_first_action, {action: G_total_from_that_first_action}).
        """
        self._gamma = gamma
        o_star, C_inv, log_norm = self._get_preferences()

        best_G      = np.inf
        best_action = self.ACTIONS[0]
        action_scores = {}

        for a0 in self.ACTIONS:
            g_i, g_e, mu_1, Sigma_1 = self._vbgs_efe(a0, o_star, C_inv, log_norm)
            g0 = g_i - epistemic_weight * g_e

            if horizon > 1:
                g0 += self._tree_min_efe(mu_1, Sigma_1, horizon - 1,
                                         epistemic_weight, gamma,
                                         o_star, C_inv, log_norm)

            action_scores[a0] = g0
            if g0 < best_G:
                best_G      = g0
                best_action = a0

        return best_action, action_scores
