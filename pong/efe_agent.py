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
                               _PLAYER_X, _OPPONENT_X, InteractionModel, NIW)

# Constant-velocity ball dynamics; paddle/opponent carry forward.
_F = np.eye(6)
_F[0, 2] = 1.0   # bx += vx
_F[1, 3] = 1.0   # by += vy


# ── Recurrent Mixture Model (Phase 1: continuous f → won) ─────────────────────

class RMM:
    """
    AXIOM-style rMM: continuous features → rally outcome.

    Each component k owns:
        NIW(m_k, κ_k, ν_k, W_k)   — over continuous features f ∈ R^D_F
        Beta(a_k, b_k)            — over rally outcome won ∈ {0, 1}
        n_k                        — count of hard assignments to this cluster

    Online streaming, hard assignment, deterministic CRP growth:
        - if max_k log p(f | θ_k) < LL_THRESH  AND  K < K_MAX → spawn new
        - else update argmax_k by NIW-CAVI M-step (one-sample)

    Feature v2 (current): f = (offset_scaled, opp_y_at_arrival)
        offset_scaled = (target_y - landing_y) / OFFSET_SCALE
        with OFFSET_SCALE = 0.05 (max_placement). Decouples the strategic
        return-angle choice from the absolute paddle height — the same
        absolute target_y produces opposite return angles depending on
        landing_y, so target_y alone conflates angle with geometry.
    """

    # Set at module level (warmup / bench) before any RMM is constructed.
    # "v2-target_y" → f = (target_y, opp_y)   — default; paired bench winner
    # "v2-offset"   → f = ((target_y - landing_y) / OFFSET_SCALE, opp_y)
    FEATURE_VERSION = "v2-target_y"

    D_F       = 2          # (offset_scaled, opp_y_at_arrival)
    K_MAX     = 16
    LL_THRESH = -2.0       # log p(f|θ_k) below this → spawn new cluster
                           # Also: above this no novelty bonus in UCB
    KAPPA_0   = 0.5        # NIW pseudo-count on the mean
    NU_0      = 4          # NIW dof (must be > D_F + 1 = 3)
    W_0       = 0.0025     # NIW scale; predictive 1σ ≈ 0.05 in normalised units
    A_0       = 2.0        # Beta prior wins
    B_0       = 2.0        # Beta prior losses

    def __init__(self):
        self.components: list = []   # each: {niw, a_won, b_won, n}

    # ── component lifecycle ──────────────────────────────────────────────────

    def _spawn(self, f: np.ndarray) -> dict:
        m = np.asarray(f, float).copy()
        W = np.eye(self.D_F) * self.W_0
        return {
            "niw":   NIW(m, self.KAPPA_0, self.NU_0, W),
            "a_won": self.A_0,
            "b_won": self.B_0,
            "n":     0,
        }

    def _log_lik(self, f: np.ndarray) -> np.ndarray:
        """log p(f | θ_k) for each component, using the NIW Gaussian predictive."""
        if not self.components:
            return np.array([])
        f = np.asarray(f, float)
        return np.array([
            c["niw"].marginal_log_likelihood(f, self.D_F)
            for c in self.components
        ])

    def _responsibilities(self, f: np.ndarray) -> np.ndarray:
        """Soft mixture weights r_k ∝ π_k · p(f|θ_k); π_k from sample counts."""
        if not self.components:
            return np.array([])
        log_lik = self._log_lik(f)
        n_vec   = np.array([c["n"] + 1.0 for c in self.components])
        log_pi  = np.log(n_vec / n_vec.sum())
        log_r   = log_pi + log_lik
        log_r  -= log_r.max()
        r       = np.exp(log_r)
        return r / r.sum()

    # ── learning ─────────────────────────────────────────────────────────────

    def update(self, f: np.ndarray, won: bool):
        """One streaming observation: hard-assign or spawn, then update."""
        f = np.asarray(f, float)
        log_lik = self._log_lik(f)

        if (len(self.components) == 0
                or (len(self.components) < self.K_MAX
                    and log_lik.max() < self.LL_THRESH)):
            self.components.append(self._spawn(f))
            k = len(self.components) - 1
        else:
            # Hard MAP assignment: include mixing weight prior log π_k.
            # Without this, a small under-populated cluster with wide
            # predictive cov can outscore a tight well-attested one on
            # far points (its Gaussian is flatter so log p(f|θ) decays
            # less). This matches `_responsibilities`.
            n_vec  = np.array([c["n"] + 1.0 for c in self.components])
            log_pi = np.log(n_vec / n_vec.sum())
            k      = int(np.argmax(log_pi + log_lik))

        comp = self.components[k]
        # NIW one-sample CAVI M-step: N=1, x_bar=f, S=0
        comp["niw"] = comp["niw"].update(
            N_k=1.0, x_bar=f, S_k=np.zeros((self.D_F, self.D_F)))
        if won:
            comp["a_won"] += 1.0
        else:
            comp["b_won"] += 1.0
        comp["n"] += 1

    # ── prediction ───────────────────────────────────────────────────────────

    def expected_win(self, f: np.ndarray) -> float:
        """E[won | f] under the soft component mixture."""
        if not self.components:
            return 0.5
        r     = self._responsibilities(f)
        means = np.array([c["a_won"] / (c["a_won"] + c["b_won"])
                          for c in self.components])
        return float(np.sum(r * means))

    def ucb_score(self, f: np.ndarray, kappa: float = 0.3) -> float:
        """
        Bayes-UCB on the soft mixture:
            score = E[won|f] + κ · sqrt(1 / effective_observed_mass)

        Effective mass aggregates the prior-subtracted Beta count weighted by
        responsibility.  When the query point is novel (max log p(f|θ_k) <
        LL_THRESH), the responsibilities are unreliable — return the prior
        with full κ-bonus so unexplored regions look attractive.
        """
        if not self.components:
            return 0.5 + kappa
        log_lik = self._log_lik(f)
        if log_lik.max() < self.LL_THRESH:
            return 0.5 + kappa
        r     = self._responsibilities(f)
        means = np.array([c["a_won"] / (c["a_won"] + c["b_won"])
                          for c in self.components])
        n_eff = np.array([
            (c["a_won"] - self.A_0) + (c["b_won"] - self.B_0)
            for c in self.components])
        weighted_n = float(np.sum(r * n_eff))
        bonus = kappa * np.sqrt(1.0 / max(weighted_n, 1.0))
        return float(np.sum(r * means)) + bonus

    # ── diagnostics / persistence ────────────────────────────────────────────

    def n_components(self) -> int:
        return len(self.components)

    def total_observed(self) -> float:
        return float(sum(c["n"] for c in self.components))

    def to_dict(self) -> dict:
        return {
            "feature_version": self.FEATURE_VERSION,
            "components": [
                {"m":     c["niw"].m.tolist(),
                 "kappa": float(c["niw"].kappa),
                 "nu":    float(c["niw"].nu),
                 "W":     c["niw"].W.tolist(),
                 "a_won": float(c["a_won"]),
                 "b_won": float(c["b_won"]),
                 "n":     int(c["n"])}
                for c in self.components
            ]
        }

    @classmethod
    def from_dict(cls, d: dict) -> "RMM":
        rmm = cls()
        if d.get("feature_version") != cls.FEATURE_VERSION:
            return rmm   # incompatible feature space → cold start
        for c in d.get("components", []):
            rmm.components.append({
                "niw":   NIW(np.array(c["m"], float),
                              float(c["kappa"]),
                              float(c["nu"]),
                              np.array(c["W"], float)),
                "a_won": float(c["a_won"]),
                "b_won": float(c["b_won"]),
                "n":     int(c["n"]),
            })
        return rmm


# ── Long-term cross-session learning (rMM-inspired) ───────────────────────────

class LongTermStats:
    """
    Persistent sufficient-statistics accumulator — never decays.

    Win-rate predictor: `RMM` over continuous (target_y, opp_y_at_arrival)
    features with a Beta(won) head per cluster.  Each rally outcome updates
    the rMM via one CAVI step.  Placement-choice queries the rMM via UCB.

    Per-(vy_sign, bounced) aim calibrator is retained for the existing
    landing-bias code path — orthogonal concern, kept for backward compat.
    """

    BIAS_KEYS = [(-1, 0), (-1, 1), (0, 0), (0, 1), (1, 0), (1, 1)]

    def __init__(self):
        # Per-key aim accumulators (legacy bias path)
        self._n_aim   = {k: 0   for k in self.BIAS_KEYS}
        self._sum_err = {k: 0.0 for k in self.BIAS_KEYS}
        self.n_aim = 0
        # Win-rate rMM
        self.rmm = RMM()

    # ── update interface ──────────────────────────────────────────────────────

    def record_aim_error(self, err: float, key=None):
        """Call at every non-fell-short miss with the contact key."""
        self.n_aim += 1
        if key is not None and key in self._n_aim:
            self._n_aim[key]   += 1
            self._sum_err[key] += float(err)

    def record_rally(self, offset_scaled: float, opp_y: float, won: bool):
        """Stream one rally outcome into the rMM (v2-offset feature space)."""
        f = np.array([float(offset_scaled), float(opp_y)])
        self.rmm.update(f, bool(won))

    # ── read interface ────────────────────────────────────────────────────────

    def aim_bias(self, key=None):
        if key is None or key not in self._n_aim:
            return 0.0
        n = self._n_aim[key]
        return self._sum_err[key] / max(n, 1) if n else 0.0

    def aim_trust(self, key=None):
        if key is None or key not in self._n_aim:
            return 0.0
        return min(self._n_aim[key] / 80.0, 0.85)

    def expected_win(self, offset_scaled: float, opp_y: float) -> float:
        return self.rmm.expected_win(np.array([float(offset_scaled), float(opp_y)]))

    def ucb_score(self, offset_scaled: float, opp_y: float,
                   kappa: float = 0.3) -> float:
        return self.rmm.ucb_score(np.array([float(offset_scaled), float(opp_y)]),
                                    kappa=kappa)

    def evidence(self) -> float:
        return self.rmm.total_observed()

    def n_components(self) -> int:
        return self.rmm.n_components()

    # ── persistence ───────────────────────────────────────────────────────────

    def to_dict(self):
        return {
            "n_aim":     self.n_aim,
            "n_aim_k":   {str(k): v for k, v in self._n_aim.items()},
            "sum_err_k": {str(k): v for k, v in self._sum_err.items()},
            "rmm":       self.rmm.to_dict(),
        }

    @classmethod
    def from_dict(cls, d):
        lt = cls()
        lt.n_aim = int(d.get("n_aim", 0))
        for raw_k, v in d.get("n_aim_k", {}).items():
            k = tuple(int(x) for x in raw_k.strip("()").split(", "))
            if k in lt._n_aim:
                lt._n_aim[k] = int(v)
        for raw_k, v in d.get("sum_err_k", {}).items():
            k = tuple(int(x) for x in raw_k.strip("()").split(", "))
            if k in lt._sum_err:
                lt._sum_err[k] = float(v)
        if "rmm" in d:
            lt.rmm = RMM.from_dict(d["rmm"])
        return lt


class EFEAgent:
    """
    EFE action selection with optional N-step planning horizon.

    G(pi) = sum_{t=0}^{N-1} gamma^t * [G_instr(a_t|s_t) - lambda * G_epist(a_t|s_t)]

    Step 0  - full VBGS predict (wall-bounce-aware).
    Step 1+ - linear-Gaussian predict (fast; large uncertainty at depth > 1).

    Action a* = first action of argmin_pi G(pi).
    """

    ACTIONS = [0, 2, 3]   # NOOP, UP, DOWN
    I_PY    = 4            # player-y index in 6D state

    _I_PY_OBS = 2   # player-y index in 4D observation (o_star)

    # rMM feature scaling: offset_scaled = (target_y - landing_y) / OFFSET_SCALE.
    # 0.05 = max_placement; with target ∈ [landing - 0.05, landing + 0.05]
    # after clipping in _choose_target_y, offset_scaled stays in [-1, +1].
    OFFSET_SCALE = 0.05

    # rMM feature mode: "target_y" (default, paired A/B winner) or "offset".
    # Must mirror RMM.FEATURE_VERSION so the pkl version sentinel matches.
    _RMM_FEATURE = "target_y"

    # Adaptive planning horizon: when the ball is within _MAX_HORIZON frames
    # of arrival, grow H to plan past the contact step. _tree_min_efe early-
    # stops branches once the rollout crosses _PLAYER_X so cost stays bounded.
    # When False, select_action_horizon uses the caller-passed `horizon`.
    _ADAPTIVE_HORIZON = False
    _MIN_HORIZON      = 3
    _MAX_HORIZON      = 6

    @classmethod
    def _to_rmm_feature(cls, target_y: float, landing_y: float) -> float:
        """target_y → rMM feature value, per `_RMM_FEATURE` mode."""
        if cls._RMM_FEATURE == "offset":
            return (float(target_y) - float(landing_y)) / cls.OFFSET_SCALE
        return float(target_y)

    def __init__(self, belief: MixtureBeliefFilter,
                 likelihood: LikelihoodModel,
                 preferences: PriorPreferences,
                 vel_alpha: float = 0.7,
                 vel_alpha_far: float = 0.3,
                 bias_alpha: float = 0.10):
        self.belief         = belief
        self.likelihood     = likelihood
        self.preferences    = preferences
        self._vel_alpha     = vel_alpha     # near-contact (responsive)
        self._vel_alpha_far = vel_alpha_far  # far approach (smoothed)
        self._raw_vx     = 0.0
        self._raw_vy     = 0.0
        self._prev_bx    = None
        self._prev_by    = None
        # Landing-bias: per-(vy_sign, bounced) EMA correction applied to predicted landing y.
        # Keys: (vy_sign ∈ {-1, 0, 1}, bounced ∈ {0, 1})
        # Updated from observed non-fell-short misses using the contact-frame key.
        # landing_bias property returns the currently active key's value for display.
        self._bias_alpha  = bias_alpha
        _BIAS_KEYS = [(-1,0), (-1,1), (0,0), (0,1), (1,0), (1,1)]
        self._bias_dict: dict = {k: 0.0 for k in _BIAS_KEYS}
        self._contact_vy_sign  = 0   # vy_sign at most recent paddle contact
        self._contact_bounced  = 0   # bounced flag at most recent paddle contact
        self._last_landing_pred = 0.5

        # Adaptive placement: updated once per episode from score trend.
        self._placement_alpha  = 0.05   # episode-level learning rate
        self._placement_min    = 0.025  # floor: below this strategic offset collapses
        self._placement_max    = 0.07
        self.placement         = 0.035  # public; starts at mid-range
        self.confidence        = 1.0    # public; updated each step
        self._last_confidence  = 1.0
        self._score_history    = []     # rolling window for trend signal

        # Epistemic weight: λ_eff = λ_max × (1 - confidence).
        # High uncertainty → explore; high confidence → pure exploitation.
        self._lambda_max  = 0.2        # reduced: exploration must not override interception
        self.epistemic_w  = 0.0        # public; current effective λ

        # rMM-style interaction model (AXIOM epistemic term at step 0)
        self.interaction_model = InteractionModel()

        # Urgency window adaptation (Step 7b): adapt _frames_near from fell-short timing.
        # frames_needed = |paddle_gap| / PADDLE_SPEED estimates how many extra frames
        # the paddle required. EMA of these → set _frames_near so urgency kicks in early enough.
        self._PADDLE_SPEED         = 0.060  # normalised units per step (frame_skip=4, measured)
        self._FELL_SHORT_THRESH    = 0.090  # ~1.5 steps: gap bigger than this = timing failure not aim
        self._urgency_alpha        = 0.10   # EMA learning rate
        self._fell_short_frames_ema = float(preferences._frames_near)  # warm-start

        # Long-term cross-session learning (LongTermStats / rMM-inspired)
        self.long_term       = LongTermStats()
        # Snapshotted at each paddle contact for win attribution:
        #   _contact_offset = (target_y_locked - landing_y) / OFFSET_SCALE at
        #                     the moment of contact (in v2-offset feature units).
        #   _contact_opp_y  = predicted opp_y at our arrival, locked at choice
        #                     time (matches the feature passed to ucb_score).
        self._contact_offset = None
        self._contact_opp_y  = None
        # RNG kept for Thompson fallback / future use.
        self._tgt_rng = np.random.default_rng()
        # Bayes-UCB exploration constant. Larger → more exploration of low-n
        # cells; small → fast greedy convergence. 0.3 keeps fresh-cell UCB
        # (≈0.5 + 0.15) below known-good cells (≈0.7+) so well-attested wins
        # are exploited and unvisited cells get tried only when reachable
        # neighbours have similar means.
        self._ucb_kappa = 0.3
        # Locked target_y for the current incoming approach (None when idle),
        # and the offset_scaled feature it represents.
        self._target_y_locked = None
        self._locked_offset   = None
        # Per-frame diagnostics (instrumented in CSV by play_efe.py)
        self._tgt_chosen   = 0.5    # last placement-grid pick (raw, pre-bias)
        self._tgt_opp_pred = 0.5    # predicted opp_y used at last placement choice
        self._tgt_pwin     = 0.5    # E[win] at chosen cell

        # Set to True for exactly one frame after a wall (vy) bounce is detected.
        # Read by play_efe.py to track bounced_this_approach for contact logging.
        self._last_vy_bounce = False

    # ── landing_bias public property (display / legacy compatibility) ─────────

    @property
    def landing_bias(self) -> float:
        """Current bias for the active contact key — used by play_efe.py for display."""
        k = (self._contact_vy_sign, self._contact_bounced)
        return self._bias_dict.get(k, 0.0)

    @landing_bias.setter
    def landing_bias(self, value):
        """Legacy setter used during state restore: write scalar to all keys."""
        for k in self._bias_dict:
            self._bias_dict[k] = float(value)

    # ── raw velocity tracker ──────────────────────────────────────────────────

    # Minimum |dv| to treat a sign flip as a real bounce rather than noise.
    # Normal ball speed ≈ 0.015–0.025/frame; serve jitter is near-zero.
    _BOUNCE_THRESH = 0.003

    # Wall-clip detection: if y is within this distance of an edge and the
    # current vy is heading further into that edge, force a vy reversal.
    # Catches bounces where the ball is clipped to the wall for a frame and
    # dvy = 0 → original threshold-based detector misses the sign flip.
    _WALL_CLIP_MARGIN = 0.005

    # Hybrid α for the raw velocity EMA. When the ball is more than
    # `_VEL_NEAR_FRAMES` frames from arrival, use `_vel_alpha_far`
    # (low → heavy smoothing) so quantisation jitter doesn't drift the
    # landing prediction. Within the urgency window, fall back to
    # `_vel_alpha` (high → responsive) so wall-bounces and last-frame
    # corrections aren't lagged. Class toggle for A/B.
    _VEL_ALPHA_HYBRID = False
    _VEL_NEAR_FRAMES  = 6

    # Force a target re-sample on every detected vy bounce, so the paddle
    # re-aims against the post-bounce landing prediction instead of holding
    # a stale lock. The existing LOCK_SLACK=0.10 hysteresis lets late bounces
    # (which change landing by 0.05–0.08) slip through without re-sampling;
    # the failure analyzer shows ~0.03 of aim error accumulates per bounce.
    _RELOCK_ON_BOUNCE = False

    def update_raw_velocity(self, bx: float, by: float):
        """
        Call once per frame with the latest raw ball position (normalised).

        On normal frames: EMA blend (α=vel_alpha) to suppress quantisation noise.
        On bounce frames: hard-reset to the new frame-difference velocity so the
        landing prediction immediately reflects the reversed direction.

        Bounce detection — vy is reversed when EITHER:
          (a) dvy and current EMA have opposite sign and |dvy| > _BOUNCE_THRESH
              (the original sign-flip rule), OR
          (b) `by` is at a wall edge and the EMA still points further into that
              wall (wall-clip case where dvy ≈ 0 hides the sign flip).
        """
        if self._prev_bx is not None:
            dvx = float(np.clip(bx - self._prev_bx, -0.06, 0.06))
            dvy = float(np.clip(by - self._prev_by, -0.06, 0.06))

            vx_bounce = (dvx * self._raw_vx < 0) and abs(dvx) > self._BOUNCE_THRESH
            vy_bounce = (dvy * self._raw_vy < 0) and abs(dvy) > self._BOUNCE_THRESH

            # Wall-clip: by clipped to top (≈0) while raw_vy still negative,
            # or to bottom (≈1) while raw_vy still positive.
            margin = self._WALL_CLIP_MARGIN
            if (by < margin and self._raw_vy < -self._BOUNCE_THRESH) \
                    or (by > 1.0 - margin and self._raw_vy > self._BOUNCE_THRESH):
                vy_bounce = True
                # Estimate post-bounce dvy by reflecting current raw_vy
                dvy = -self._raw_vy

            if vx_bounce or vy_bounce:
                # Direction reversed — accept new velocity instantly
                self._raw_vx = dvx
                self._raw_vy = dvy
                if vy_bounce:
                    # Ball bounced off top/bottom wall: landing prediction changes
                    # immediately but EMA target would lag 3 frames → wrong paddle
                    # direction after each bounce.  Reset so next frame reinitialises.
                    self._smooth_tgt = None
                    if self._RELOCK_ON_BOUNCE and self._raw_vx > 1e-3:
                        # Force _get_preferences to resample target against the
                        # post-bounce landing prediction.
                        self._target_y_locked = None
                        self._locked_offset   = None
                self._last_vy_bounce = vy_bounce   # True only when vy reversed
            else:
                a = self._vel_alpha
                if self._VEL_ALPHA_HYBRID and self._raw_vx > 1e-3:
                    frames_left = (_PLAYER_X - bx) / self._raw_vx
                    if frames_left >= self._VEL_NEAR_FRAMES:
                        a = self._vel_alpha_far
                self._raw_vx = a * dvx + (1.0 - a) * self._raw_vx
                self._raw_vy = a * dvy + (1.0 - a) * self._raw_vy
                self._last_vy_bounce = False

        self._prev_bx = bx
        self._prev_by = by

    def reset_raw_velocity(self):
        """Call when the ball goes out of play (serve / reset)."""
        self._raw_vx        = 0.0
        self._raw_vy        = 0.0
        self._prev_bx       = None
        self._prev_by       = None
        self._smooth_tgt    = None   # reinit EMA on next approach
        self._last_vy_bounce = False

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
            # Update the per-key EMA — use the key recorded at ball-paddle contact.
            key = (self._contact_vy_sign, self._contact_bounced)
            cur_bias = self._bias_dict.get(key, 0.0)
            ema_bias = float(np.clip(
                self._bias_alpha * error + (1.0 - self._bias_alpha) * cur_bias,
                -0.3, 0.3))
            # Bayesian long-term accumulator: blends in and takes over as evidence grows
            self.long_term.record_aim_error(error, key=key)
            trust = self.long_term.aim_trust(key)
            self._bias_dict[key] = float(np.clip(
                trust * self.long_term.aim_bias(key) + (1.0 - trust) * ema_bias,
                -0.3, 0.3))
            # Record rally loss for the rMM — attribute to the most recent
            # contact's (offset_scaled, opp_y). Misses without a prior contact
            # this rally are not strategically attributable; skip.
            if self._contact_offset is not None and self._contact_opp_y is not None:
                self.long_term.record_rally(self._contact_offset,
                                             self._contact_opp_y, won=False)
                self._contact_offset = None
                self._contact_opp_y  = None
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

        # Magnitude-weighted update: tanh(trend/10) in (-1,+1) so small noisy
        # episodes barely move placement while strong signals have full effect.
        # Episode scores span ~±15; tanh(15/10)=0.91 gives ~full weight.
        weight = float(np.tanh(trend / 10.0))
        if weight > 0:
            self.placement = float(np.clip(
                self.placement + self._placement_alpha * weight * (self._placement_max - self.placement),
                self._placement_min, self._placement_max))
        else:
            self.placement = float(np.clip(
                self.placement + self._placement_alpha * weight * (self.placement - self._placement_min),
                self._placement_min, self._placement_max))

        # NOTE: preferred_placement() floor was removed — the Beta grid is binned
        # by ball landing y (wrong quantity; should be placement magnitude), causing
        # argmax to always land at an edge bin → constant 0.070 cap.  Restore once
        # the grid is rebinned by eff_placement.

    def update_preferences_on_contact(self, player_y: float,
                                       vy_sign: int = 0, bounced: int = 0,
                                       opp_y: float = None):
        """
        Call when the ball contacts our paddle (vx sign flip near player_x).

        Records contact key (vy_sign, bounced) for per-trajectory bias lookup,
        the offset_scaled chosen at lock time for rMM win attribution, and
        opp_y at contact for the second rMM feature.
        """
        self._contact_vy_sign = int(vy_sign)
        self._contact_bounced = int(bounced)
        self._contact_offset  = self._locked_offset
        if opp_y is not None:
            self._contact_opp_y = float(opp_y)

    def record_win(self):
        """Call when reward==+1 (player just scored). Attributes the win to
        the (offset_scaled, opp_y) feature at the most recent paddle contact."""
        if self._contact_offset is not None and self._contact_opp_y is not None:
            self.long_term.record_rally(self._contact_offset,
                                         self._contact_opp_y, won=True)
            self._contact_offset = None
            self._contact_opp_y  = None

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

    # Number of candidate target_y points scanned within the reach window.
    _N_TARGET_CANDIDATES = 9

    # Hysteresis slack on the relock criterion. Must exceed typical
    # frame-to-frame landing_y jitter so the target stays locked through
    # prediction noise; otherwise the lock thrashes once eff_max collapses
    # in the final approach.
    _LOCK_SLACK = 0.10

    # rMM trust threshold (rally count). Below this we default to aiming at
    # landing_y directly — the cold rMM has too few components for UCB to
    # discriminate, and its "explore novel regions" preference systematically
    # pushes the target to the edge of the reach window, killing contact rate.
    _RMM_TRUST_RALLIES = 30

    def _choose_target_y(self, landing_y: float, predicted_opp_y: float,
                          eff_max: float) -> float:
        """
        Pick target_y by Bayes-UCB argmax over a discrete sweep of candidates
        within the reach window [landing_y − eff_max, landing_y + eff_max].
        The rMM is scored on (offset_scaled, opp_y) where
        offset_scaled = (candidate − landing_y) / OFFSET_SCALE, so the rMM
        learns return-angle preferences independent of where the ball lands.
        """
        if eff_max < 1e-3:
            return float(np.clip(landing_y, 0.0, 1.0))

        if self.long_term.evidence() < self._RMM_TRUST_RALLIES:
            return float(np.clip(landing_y, 0.0, 1.0))

        candidates = np.linspace(landing_y - eff_max, landing_y + eff_max,
                                  self._N_TARGET_CANDIDATES)
        candidates = np.clip(candidates, 0.0, 1.0)
        feats      = np.array([self._to_rmm_feature(t, landing_y)
                                for t in candidates])
        scores     = np.array([
            self.long_term.ucb_score(f, predicted_opp_y, kappa=self._ucb_kappa)
            for f in feats
        ])
        # Uninformative rMM (cold start or novel feature region) → all scores
        # collapse to 0.5 + κ. Default to the centre candidate (= landing_y)
        # rather than letting argmax pick index 0 (= landing - eff_max), which
        # systematically aims the paddle below the ball and tanks the contact
        # rate until the rMM has populated this region.
        if scores.max() - scores.min() < 1e-9:
            return float(np.clip(landing_y, 0.0, 1.0))
        return float(candidates[int(np.argmax(scores))])

    def _get_preferences(self):
        """
        Compute (o_star, C_inv, log_norm) once per step.

        Placement is scaled by belief confidence (inverse of ball-y variance).
        Opponent position is predicted at ball-arrival time. Target_y is
        chosen by Thompson sampling the 2D Beta win-rate grid at the predicted
        opp_y row, restricted to bins reachable within paddle-reach window.
        Locked per approach to keep the paddle from thrashing.
        """
        sigma_by        = float(self.belief.cov[1, 1])
        self.confidence = 1.0 / (1.0 + self._COV_K * sigma_by)
        eff_placement   = self.placement * self.confidence

        raw_vel = (self._raw_vx, self._raw_vy) if self._prev_bx is not None else None
        vx      = float(raw_vel[0]) if raw_vel is not None else float(self.belief.mean[2])
        bx      = float(self.belief.mean[0])

        target_y_override = None
        if vx > 1e-3:
            frames        = max(0, int(((_PLAYER_X - bx) / vx)))
            predicted_opp = self._predict_opp_y(frames)
            # Lock a Thompson-sampled target per approach; resample if a bounce
            # moved the predicted landing outside the locked target's reach.
            from generative_model import _predict_ball_landing
            landing_y = _predict_ball_landing(self.belief.mean, raw_vel=raw_vel)
            placement_scale = float(np.clip(
                (frames - 10) / max(1, self.preferences._frames_start - 10),
                0.0, 1.0))
            eff_max = eff_placement * placement_scale
            need_sample = (
                self._target_y_locked is None
                or abs(self._target_y_locked - landing_y) > eff_max + self._LOCK_SLACK)
            if need_sample:
                self._target_y_locked = self._choose_target_y(
                    landing_y, predicted_opp, eff_max)
                self._locked_offset   = self._to_rmm_feature(
                    self._target_y_locked, landing_y)
                self._tgt_chosen   = self._target_y_locked
                self._tgt_opp_pred = predicted_opp
                self._tgt_pwin     = self.long_term.expected_win(
                    self._locked_offset, predicted_opp)
            target_y_override = self._target_y_locked
        else:
            predicted_opp         = float(self.belief.mean[5])
            self._target_y_locked = None   # outgoing/idle: clear lock
            self._locked_offset   = None

        # Pass the current frame's eff_max (with placement_scale fade) to the
        # preferences module so it re-clips the locked target into the
        # current reach window — prevents the stale lock drifting beyond the
        # placement budget as landing_y evolves.
        em_for_clip = eff_max if vx > 1e-3 else None
        o_star, C_inv, log_norm = self.preferences.contextual_target(
            self.belief.mean, raw_vel=raw_vel,
            eff_max=em_for_clip,
            target_y_override=target_y_override)
        self._last_confidence = self.confidence

        # Compute final biased target using per-(vy_sign, bounced) correction.
        raw_tgt    = float(o_star[self._I_PY_OBS])
        bias_key   = (self._contact_vy_sign, self._contact_bounced)
        cur_bias   = self._bias_dict.get(bias_key, 0.0)
        biased_tgt = float(np.clip(raw_tgt + cur_bias, 0.0, 1.0))

        # EMA smoothing: dampens frame-to-frame landing-prediction noise that
        # causes the paddle to reverse direction every few frames.
        # Alpha=0.35  →  τ ≈ 3 frames.  Reset whenever ball is not approaching
        # so the filter reinitialises cleanly on the next serve.
        _EMA = 0.35
        if vx > 1e-3:
            if not hasattr(self, '_smooth_tgt') or self._smooth_tgt is None:
                self._smooth_tgt = biased_tgt
            else:
                self._smooth_tgt = _EMA * biased_tgt + (1.0 - _EMA) * self._smooth_tgt
            final_tgt = float(self._smooth_tgt)
        else:
            self._smooth_tgt = None   # force reinit on next approach
            final_tgt = biased_tgt

        o_star = o_star.copy()
        o_star[self._I_PY_OBS]  = np.clip(final_tgt, 0.0, 1.0)
        self._last_landing_pred = float(o_star[self._I_PY_OBS])
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
        """
        Min discounted EFE over a depth-step linear-Gaussian tree.

        Branches whose rollout has already crossed _PLAYER_X are not expanded
        further — past contact, the action-dependent component of the cost is
        irrelevant (paddle can't affect this rally), so deeper expansion is
        pure compute waste. Keeps adaptive horizon bounded.
        """
        best = np.inf
        for a in self.ACTIONS:
            g_i, g_e, mu_n, Sigma_n = self._linear_efe(
                mu, Sigma, a, o_star, C_inv, log_norm)
            g = discount * (g_i - ew * g_e)
            if depth > 1 and mu_n[0] < _PLAYER_X:
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

        # Adaptive horizon: when ball is within _MAX_HORIZON frames of arrival
        # plan all the way to contact (+1 for the contact step itself). Beyond
        # _MAX_HORIZON, fall back to the caller's `horizon`. _tree_min_efe will
        # early-stop branches that cross _PLAYER_X so the tree stays bounded.
        if self._ADAPTIVE_HORIZON and frames < self._MAX_HORIZON:
            H = max(self._MIN_HORIZON, frames + 1)
        else:
            H = horizon

        best_G      = np.inf
        best_action = self.ACTIONS[0]
        action_scores = {}

        for a0 in self.ACTIONS:
            g_i, g_e, mu_1, Sigma_1 = self._vbgs_efe(a0, o_star, C_inv, log_norm)
            g0 = g_i - ew * g_e

            if H > 1:
                g0 += self._tree_min_efe(mu_1, Sigma_1, H - 1,
                                         ew, gamma,
                                         o_star, C_inv, log_norm)

            action_scores[a0] = g0
            if g0 < best_G:
                best_G      = g0
                best_action = a0

        return best_action, action_scores

