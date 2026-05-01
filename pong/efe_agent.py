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
from generative_model import (MixtureBeliefFilter, LikelihoodModel,
                               PriorPreferences, PADDLE_DY,
                               _PLAYER_X, _OPPONENT_X, InteractionModel)

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
                 bias_alpha: float = 0.04):
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

        # Adaptive placement: updated once per episode from score trend.
        self._placement_alpha  = 0.07   # episode-level learning rate
        self._placement_min    = 0.00
        self._placement_max    = 0.07
        self.placement         = 0.02   # public; starts conservative
        self.confidence        = 1.0    # public; updated each step
        self._last_confidence  = 1.0
        self._score_history    = []     # rolling window for trend signal

        # Epistemic weight: λ_eff = λ_max × (1 - confidence).
        # High uncertainty → explore; high confidence → pure exploitation.
        self._lambda_max  = 0.2        # reduced: exploration must not override interception
        self.epistemic_w  = 0.0        # public; current effective λ

        # rMM-style interaction model (AXIOM epistemic term at step 0)
        self.interaction_model = InteractionModel()

        # Preference adaptation (Step 7): σ_py_near from win-frame deviations.
        # EMA of (py - landing_pred)² on winning points → achievable precision.
        self._pref_ema_var  = 0.03 ** 2  # start at σ=0.03 (current default)
        self._pref_ema_lr   = 0.05       # slow adaptation

        # Urgency window adaptation (Step 7b): adapt _frames_near from fell-short timing.
        # frames_needed = |paddle_gap| / PADDLE_SPEED estimates how many extra frames
        # the paddle required. EMA of these → set _frames_near so urgency kicks in early enough.
        self._PADDLE_SPEED         = 0.060  # normalised units per step (frame_skip=4, measured)
        self._FELL_SHORT_THRESH    = 0.090  # ~1.5 steps: gap bigger than this = timing failure not aim
        self._urgency_alpha        = 0.10   # EMA learning rate
        self._fell_short_frames_ema = float(preferences._frames_near)  # warm-start

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
    # than a prediction error.  0.060/step × ~1.5 steps slack = 0.090.
    _FELL_SHORT_THRESH = 0.090  # overridden in __init__; keep in sync

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
        else:
            # Adapt urgency window: estimate how many more frames were needed.
            frames_needed = abs(paddle_gap) / self._PADDLE_SPEED
            self._fell_short_frames_ema = (
                (1.0 - self._urgency_alpha) * self._fell_short_frames_ema
                + self._urgency_alpha * frames_needed)
            new_near  = int(round(self._fell_short_frames_ema)) + 2
            self.preferences.update_urgency(
                frames_near=new_near,
                frames_start=new_near + 15)

        return fell_short, paddle_gap

    def update_from_episode(self, score: float, window: int = 10):
        """
        Call once per episode with the final score.
        Compares score to the rolling mean of the last `window` episodes:
          - Improving → increase placement (be bolder)
          - Degrading  → decrease placement (be safer)
        Episode-level signal is much less noisy than per-point win/fell-short.
        """
        self._score_history.append(score)
        if len(self._score_history) < 2:
            return

        recent = self._score_history[-window:]
        if len(recent) >= 2:
            # Compare latest score to mean of previous episodes in window
            trend = score - float(np.mean(recent[:-1]))
        else:
            trend = 0.0

        if trend > 0:
            self.placement = float(np.clip(
                self.placement + self._placement_alpha * (self._placement_max - self.placement),
                self._placement_min, self._placement_max))
        else:
            self.placement = float(np.clip(
                self.placement * (1.0 - self._placement_alpha),
                self._placement_min, self._placement_max))

    def update_preferences_on_contact(self, player_y: float):
        """
        Call when the ball contacts our paddle (vx sign flip near player_x).

        Updates σ_py_near from achievable paddle precision:
          EMA of (player_y − last_landing_pred)² → σ = sqrt(EMA_var).
        Tightens when the paddle consistently hits close to the predicted target;
        loosens when placement is imprecise — adapts to actual motor capability.
        """
        py_dev = float(player_y) - self._last_landing_pred
        self._pref_ema_var = ((1.0 - self._pref_ema_lr) * self._pref_ema_var
                              + self._pref_ema_lr * py_dev ** 2)
        new_sigma = float(np.sqrt(self._pref_ema_var))
        self.preferences.update_sigma(sigma_py_near=new_sigma)

    # ── opponent position prediction ──────────────────────────────────────────

    @staticmethod
    def _bounce(by, vy):
        if by < 0.0: return -by, -vy
        if by > 1.0: return 2.0 - by, -vy
        return by, vy

    def _predict_opp_y(self, frames: int) -> float:
        """
        Predict opponent y after `frames` steps using the opponent belief tracker.
        Ball trajectory is simulated with constant velocity + wall bounces.
        Falls back to current oy when frames is out of range.
        """
        if frames <= 0 or frames > 50:
            return float(self.belief.mean[5])

        mu = self.belief.mean
        bx = float(mu[0])
        by = float(mu[1])
        vx = float(np.clip(self._raw_vx if self._prev_bx is not None else mu[2], -0.06, 0.06))
        vy = float(np.clip(self._raw_vy if self._prev_bx is not None else mu[3], -0.06, 0.06))

        ball_states = []
        for _ in range(frames):
            ball_states.append(np.array([bx, by, vx, vy]))
            bx += vx
            by += vy
            by, vy = self._bounce(by, vy)

        opp_y = float(mu[5])
        traj  = self.belief.opp_tracker.predict_trajectory(
            opp_y, ball_states, [0] * frames)
        return float(traj[-1]) if traj else opp_y

    # Sensitivity of placement confidence to ball-y belief variance.
    # confidence = 1 / (1 + _COV_K * Sigma[by,by])
    # Sigma[by,by] ≈ 0.001 (well-tracked) → confidence ≈ 0.95
    # Sigma[by,by] ≈ 0.02  (uncertain)    → confidence ≈ 0.50
    # Sigma[by,by] ≈ 0.10  (fresh/lost)   → confidence ≈ 0.17
    _COV_K = 30.0

    def _get_preferences(self):
        """
        Compute (o_star, C_inv, log_norm) once per step.

        Placement is scaled by belief confidence (inverse of ball-y variance).
        Opponent position is predicted at ball-arrival time so placement aims
        at the side the opponent will NOT be — exploiting the opponent model.
        """
        sigma_by        = float(self.belief.cov[1, 1])
        self.confidence = 1.0 / (1.0 + self._COV_K * sigma_by)
        eff_placement   = self.placement * self.confidence

        raw_vel = (self._raw_vx, self._raw_vy) if self._prev_bx is not None else None
        vx      = float(raw_vel[0]) if raw_vel is not None else float(self.belief.mean[2])
        bx      = float(self.belief.mean[0])

        # Predict where the opponent will be when ball arrives
        if vx > 1e-3:
            frames        = max(0, int(((_PLAYER_X - bx) / vx)))
            predicted_opp = self._predict_opp_y(frames)
        else:
            predicted_opp = float(self.belief.mean[5])

        o_star, C_inv, log_norm = self.preferences.contextual_target(
            self.belief.mean, raw_vel=raw_vel,
            placement=eff_placement, predicted_opp_y=predicted_opp)
        self._last_landing_pred = float(o_star[self._I_PY_OBS])
        self._last_confidence   = self.confidence
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

        # rMM epistemic term (AXIOM): expected KL update to the Dirichlet
        # count model over contact offsets — drives exploration of unexplored
        # parts of the paddle to learn the offset→angle map.
        contact_offset = self._last_landing_pred - float(mu_pred[4])
        g_epist        = self.interaction_model.epistemic_value(contact_offset)

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
        
        # Handle wall bounces in mental rollout
        if mu_pred[1] < 0.0:
            mu_pred[1] = -mu_pred[1]
            mu_pred[3] = -mu_pred[3]  # flip vy
        elif mu_pred[1] > 1.0:
            mu_pred[1] = 2.0 - mu_pred[1]
            mu_pred[3] = -mu_pred[3]  # flip vy
            
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
                              epistemic_weight: float = None,
                              gamma: float = 0.9):
        """
        N-step planning (Phase 3.3).

        epistemic_weight=None (default): use adaptive λ = λ_max×(1−confidence),
        so the agent explores when uncertain and exploits when confident.
        Pass a float to override (0.0 = pure instrumental, as before).

        Evaluates 3 VBGS step-0 calls + recursive linear-Gaussian tree.
        Preferences fixed at current belief (consistent across all branches).
        Returns (best_first_action, {action: G_total_from_that_first_action}).
        """
        self._gamma = gamma
        o_star, C_inv, log_norm = self._get_preferences()

        # Adaptive epistemic weight — zero out when ball is close (pure interception)
        bx  = float(self.belief.mean[0])
        vx  = float(self._raw_vx if self._prev_bx is not None else self.belief.mean[2])
        frames = max(0, int((_PLAYER_X - bx) / vx)) if vx > 1e-3 else 999
        if epistemic_weight is None:
            base_ew = self._lambda_max * (1.0 - self.confidence)
            self.epistemic_w = base_ew if frames > 8 else 0.0
        else:
            self.epistemic_w = float(epistemic_weight)
        ew = self.epistemic_w

        best_G      = np.inf
        best_action = self.ACTIONS[0]
        action_scores = {}

        for a0 in self.ACTIONS:
            g_i, g_e, mu_1, Sigma_1 = self._vbgs_efe(a0, o_star, C_inv, log_norm)
            g0 = g_i - ew * g_e

            if horizon > 1:
                g0 += self._tree_min_efe(mu_1, Sigma_1, horizon - 1,
                                         ew, gamma,
                                         o_star, C_inv, log_norm)

            action_scores[a0] = g0
            if g0 < best_G:
                best_G      = g0
                best_action = a0

        return best_action, action_scores
