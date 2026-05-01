import pickle
import pathlib
import numpy as np
from scipy.special import digamma

# Paddle displacement per STEP (normalised).  Action 2=UP, 3=DOWN, 0=NOOP.
# Atari uses frame_skip=4: real displacement ≈ 0.060/step (measured from game data).
PADDLE_DY = {0: 0.0, 2: -0.060, 3: +0.060}


# ── Normal-Inverse-Wishart conjugate prior ────────────────────────────────────

class NIW:
    """Conjugate prior/posterior for a single Gaussian component."""

    def __init__(self, m, kappa, nu, W):
        self.m     = np.asarray(m, float)
        self.kappa = float(kappa)
        self.nu    = float(nu)
        self.W     = np.asarray(W, float)
        self.d     = self.m.shape[0]

    def expected_log_det_lambda(self):
        """E[log|Λ|] under q(Λ) = Wishart(ν, W)."""
        _, logdet = np.linalg.slogdet(self.W)
        return (
            sum(digamma((self.nu - i) / 2.0) for i in range(self.d))
            + self.d * np.log(2.0)
            + logdet
        )

    def expected_mahal(self, X):
        """E[(x-μ)^T Λ (x-μ)] for rows of X (N,d) → (N,)."""
        diff      = X - self.m
        Winv_diff = np.linalg.solve(self.W, diff.T).T
        return self.d / self.kappa + self.nu * (diff * Winv_diff).sum(axis=1)

    def update(self, N_k, x_bar, S_k):
        """CAVI M-step: return updated NIW from sufficient statistics."""
        kappa_n = self.kappa + N_k
        m_n     = (self.kappa * self.m + N_k * x_bar) / kappa_n
        nu_n    = self.nu + N_k
        diff    = x_bar - self.m
        W_n     = self.W + S_k + (self.kappa * N_k / kappa_n) * np.outer(diff, diff)
        return NIW(m_n, kappa_n, nu_n, W_n)

    def predictive_mean_cov(self):
        mean = self.m.copy()
        cov  = self.W / (self.nu - self.d - 1) if self.nu > self.d + 1 else self.W.copy()
        return mean, cov

    def marginal_log_likelihood(self, x_obs, d_x):
        """
        Log p(x_obs) under the marginal N(μ_x, Σ_xx) of the predictive.
        Used to weight components in predict_conditional.
        """
        mu, Sigma = self.predictive_mean_cov()
        mu_x = mu[:d_x]
        Sxx  = Sigma[:d_x, :d_x]
        diff = x_obs - mu_x
        sign, logdet = np.linalg.slogdet(Sxx)
        mahal = diff @ np.linalg.solve(Sxx, diff)
        return -0.5 * (d_x * np.log(2 * np.pi) + logdet + mahal)

    def conditional(self, x_obs, d_x):
        """
        Gaussian conditioning: given x_obs (d_x,) return (mean_y, cov_y)
        for the remaining d_y = d - d_x dimensions.
        """
        mu, Sigma = self.predictive_mean_cov()
        mu_x, mu_y = mu[:d_x], mu[d_x:]
        Sxx = Sigma[:d_x, :d_x]
        Sxy = Sigma[:d_x, d_x:]
        Syy = Sigma[d_x:, d_x:]
        gain   = np.linalg.solve(Sxx, Sxy).T        # (d_y, d_x)
        mean_y = mu_y + gain @ (x_obs - mu_x)
        cov_y  = Syy - gain @ Sxy
        return mean_y, cov_y


# ── VBGS core ─────────────────────────────────────────────────────────────────

class VBGS:
    """
    Variational Bayesian Gaussian Sum.
    Models a joint GMM over (x, y) with NIW priors and closed-form CAVI.
    Supports conditional prediction p(y|x) via Gaussian conditioning.
    """

    def __init__(self, K, d, priors, alpha0=1.0):
        self.K      = K
        self.d      = d
        self.alpha0 = alpha0
        self.niw    = list(priors)
        self.alpha  = np.full(K, alpha0, dtype=float)

    # ── CAVI E-step ──────────────────────────────────────────────────────────

    def responsibilities(self, X):
        """Soft assignments r[n,k] for data matrix X (N,d)."""
        psi_sum = digamma(self.alpha.sum())
        log_rho = np.column_stack([
            digamma(self.alpha[k]) - psi_sum
            + 0.5 * self.niw[k].expected_log_det_lambda()
            - 0.5 * self.d * np.log(2 * np.pi)
            - 0.5 * self.niw[k].expected_mahal(X)
            for k in range(self.K)
        ])
        log_rho -= log_rho.max(axis=1, keepdims=True)
        rho = np.exp(log_rho)
        return rho / rho.sum(axis=1, keepdims=True)

    # ── CAVI M-step ──────────────────────────────────────────────────────────

    def _update_params(self, X, r):
        N_k        = r.sum(axis=0)
        self.alpha = self.alpha0 + N_k
        for k in range(self.K):
            if N_k[k] < 1e-10:
                continue
            x_bar        = (r[:, k] @ X) / N_k[k]
            diff         = X - x_bar
            S_k          = (r[:, k, None] * diff).T @ diff
            self.niw[k]  = self.niw[k].update(N_k[k], x_bar, S_k)

    # ── public interface ──────────────────────────────────────────────────────

    def fit_step(self, X, n_iter=1):
        """One or more CAVI iterations on data X (N,d)."""
        for _ in range(n_iter):
            r = self.responsibilities(X)
            self._update_params(X, r)
        return r

    def predict_conditional(self, x_obs, d_x):
        """
        Given x_obs (d_x,) return [(weight_k, mean_y_k, cov_y_k), ...].
        Weights come from the marginal p(x|k) to avoid dimension mismatch.
        """
        log_w = np.array([
            np.log(self.alpha[k] / self.alpha.sum())
            + self.niw[k].marginal_log_likelihood(x_obs, d_x)
            for k in range(self.K)
        ])
        log_w -= log_w.max()
        w = np.exp(log_w)
        w /= w.sum()
        return [(w[k], *self.niw[k].conditional(x_obs, d_x)) for k in range(self.K)]

    def predict_mean(self, x_obs, d_x):
        """Posterior-weighted mean prediction of y given x_obs."""
        components = self.predict_conditional(x_obs, d_x)
        return sum(w * mu for w, mu, _ in components)

    def component_weights(self):
        """Normalised mixing proportions from the Dirichlet posterior."""
        return self.alpha / self.alpha.sum()


# ── Interaction epistemic model (rMM-style, AXIOM) ───────────────────────────

class InteractionModel:
    """
    Dirichlet count model over discretised paddle-offset bins.

    Implements the rMM epistemic term from AXIOM:
      G_epist(a) = ψ(α[b]+1) − ψ(α[b])   (expected KL update to Dirichlet)

    where b is the bin index of the predicted contact offset
    (ball_y − paddle_y at the moment of contact).

    Rarely-visited offsets have high epistemic value → agent is driven to
    explore different parts of the paddle, learning the angle-change map.
    As counts grow, epistemic value decays and exploitation takes over.
    """

    N_BINS    = 9
    BIN_EDGES = np.linspace(-0.08, 0.08, N_BINS + 1)

    def __init__(self, alpha0: float = 2.0):
        self.alpha = np.full(self.N_BINS, float(alpha0))

    def bin_idx(self, offset: float) -> int:
        return int(np.clip(
            np.searchsorted(self.BIN_EDGES[1:], offset), 0, self.N_BINS - 1))

    def epistemic_value(self, offset: float) -> float:
        """Expected KL gain from observing one contact at this offset."""
        b = self.bin_idx(offset)
        return float(digamma(self.alpha[b] + 1.0) - digamma(self.alpha[b]))

    def observe(self, offset: float):
        """Update counts after observing a contact at this offset."""
        self.alpha[self.bin_idx(offset)] += 1.0

    @property
    def counts(self) -> np.ndarray:
        return self.alpha.copy()


# ── Prior preferences ────────────────────────────────────────────────────────

_PLAYER_X   = 0.740   # measured player paddle x (RAM normalised)
_OPPONENT_X = 0.270   # measured opponent paddle x

def _frames_to_arrival(bx: float, vx: float,
                        player_x: float = _PLAYER_X) -> int:
    """Frames until ball reaches player_x. Returns 999 if ball not coming."""
    if vx <= 1e-3:
        return 999
    return max(0, int((player_x - bx) / vx))


def _wall_bounce(by, vy):
    """Reflect position and velocity off top/bottom walls."""
    if by < 0.0:
        return -by, -vy
    if by > 1.0:
        return 2.0 - by, -vy
    return by, vy


def _predict_ball_landing(mu6: np.ndarray,
                           raw_vel=None,
                           player_x: float = _PLAYER_X,
                           max_steps: int = 300) -> float:
    """
    Predict where the ball will arrive at the player's paddle line.

    Velocity source (in priority order):
      1. raw_vel = (vx, vy) — EMA of frame-differenced RAM observations.
         More accurate than the Kalman estimate: no filter lag, no VBGS spikes.
      2. mu6[2:4] — Kalman belief velocity (fallback when raw_vel is None).

    Both are clamped to ±0.06 before integration.

    Ball moving away (vx ≤ 1e-3): return current by — the ball is heading
    toward the opponent; the return will land near the current by.

    mu6: [bx, by, vx, vy, py, oy] (normalised [0,1]).
    Returns predicted landing y in [0, 1].
    """
    bx, by = float(mu6[0]), float(mu6[1])

    if raw_vel is not None:
        vx = float(np.clip(raw_vel[0], -0.06, 0.06))
        vy = float(np.clip(raw_vel[1], -0.06, 0.06))
    else:
        vx = float(np.clip(mu6[2], -0.06, 0.06))
        vy = float(np.clip(mu6[3], -0.06, 0.06))

    if vx <= 1e-3:
        return 0.5  # RL STRATEGY: Return to center when ball is moving away

    for _ in range(max_steps):
        # Precise linear interpolation at the exact plane of the paddle.
        # This prevents the final 'by' from jumping past the paddle when
        # the final discrete velocity step bridges the intersection point.
        if bx + vx >= player_x:
            time_to_intersect = (player_x - bx) / vx
            by += time_to_intersect * vy
            if by < 0.0 or by > 1.0:
                by, _ = _wall_bounce(by, vy)  # apply one final bounce if needed
            break
        
        bx += vx
        by += vy
        by, vy = _wall_bounce(by, vy)

    return float(np.clip(by, 0.0, 1.0))


class PriorPreferences:
    """
    p̃(o) = N(o ; o*, C)  — preferred observation distribution.

    Observation o = [ball_x, ball_y, player_y, opponent_y].

    Three goals encoded in o* and C:
      G1  ball_x low       → ball on opponent's side (left goal)
      G2  player_y ≈ landing_y → paddle intercepts ball at player's line
      G3  |ball_y - opponent_y| large → ball far from opponent paddle
    """

    D_OBS = 4

    # Observation indices
    IDX_BX, IDX_BY, IDX_PY, IDX_OY = 0, 1, 2, 3

    def __init__(self):
        # Baseline target — player_y is overridden contextually each step
        self.o_star = np.array([
            0.15,   # G1: ball_x near opponent's goal (left side, low value)
            0.50,   # G2: ball_y neutral baseline
            0.50,   # G2: player_y neutral baseline (set from belief at runtime)
            0.10,   # G3: opponent_y pulled away from ball (near top wall)
        ])

        # Diagonal covariance — smaller σ² = stronger preference
        # NOT ACTING LAZY: Reduced far sigma from 0.10 to 0.04 to force early alignment
        self._sigma_py_near = 0.02   # tight — ball about to arrive; PADDLE_DY=0.06 so 1σ=PADDLE_DY/3
        self._sigma_py_far  = 0.04   # loose — ball far or going away
        self._sigma_oy      = 0.15   # opponent tracking precision (learnable)
        self._sigma_bx      = 0.05   # fixed — scoring objective
        self._sigma_by      = 0.20   # fixed — ball_y not a direct goal
        self._rebuild_C()

        # Urgency: tighten σ_py over this frame window before arrival
        # NOT ACTING LAZY: Increased start window from 20 to 50 frames to account for max paddle speed
        self._frames_start = 50   # begin tightening at 50 frames out
        self._frames_near  =  5   # full urgency at 5 frames out

        # Aggressive placement: scale offset by opponent distance from centre.
        # Paddle half-height ≈ 0.030; 0.05 hits near edge — steep angle.
        # Offset fades to zero below _frames_placement_cutoff so interception
        # is never sacrificed for placement when time is short.
        self._max_placement = 0.05
        self._frames_placement_cutoff = 10

    def _rebuild_C(self):
        sigma = np.array([self._sigma_bx, self._sigma_by,
                          self._sigma_py_near, self._sigma_oy])
        self.C = np.diag(sigma ** 2)
        self.C_inv = np.diag(1.0 / sigma ** 2)
        self._log_norm = (self.D_OBS * np.log(2 * np.pi)
                          + np.linalg.slogdet(self.C)[1])

    def update_sigma(self, sigma_py_near=None, sigma_oy=None):
        if sigma_py_near is not None:
            self._sigma_py_near = float(np.clip(sigma_py_near, 0.015, 0.08))
        if sigma_oy is not None:
            self._sigma_oy = float(np.clip(sigma_oy, 0.05, 0.30))
        self._rebuild_C()

    def update_urgency(self, frames_near=None, frames_start=None):
        """Adapt the urgency tightening window from observed fell-short timing."""
        if frames_near is not None:
            self._frames_near = int(np.clip(frames_near, 3, 20))
        if frames_start is not None:
            self._frames_start = int(np.clip(frames_start,
                                             self._frames_near + 5, 40))

    def contextual_target(self, belief_mu, raw_vel=None, placement=None,
                          predicted_opp_y=None):
        """
        Return (o_star, C_inv, log_norm) with player_y target set to the
        predicted ball landing y, adjusted for intentional placement.

        Urgency scaling: σ_py loosens when ball is far (paddle idles),
        tightens linearly from _frames_start to _frames_near before arrival
        (paddle commits aggressively). Prevents premature oscillation while
        ensuring decisive movement when the ball is close.

        Placement (ball heading toward us only):
          offset = clip((oy - 0.5) * 2 * _max_placement, ±_max_placement)
          Scales with opponent distance from centre — zero at centre,
          maximum at oy=0.25 / oy=0.75. Aims return to opponent's weak side.

        Coordinate convention: smaller py = higher on screen.
        """
        o_star = self.o_star.copy()
        landing_y = _predict_ball_landing(belief_mu, raw_vel=raw_vel)

        bx = float(belief_mu[0])
        vx = float(raw_vel[0]) if raw_vel is not None else float(belief_mu[2])

        # ── urgency-scaled σ_py ───────────────────────────────────────────────
        frames = _frames_to_arrival(bx, vx)
        t = float(np.clip(
            (frames - self._frames_near) / (self._frames_start - self._frames_near),
            0.0, 1.0))
        sigma_py = self._sigma_py_near + t * (self._sigma_py_far - self._sigma_py_near)

        C_inv_dyn = self.C_inv.copy()
        C_inv_dyn[self.IDX_PY, self.IDX_PY] = 1.0 / sigma_py ** 2
        log_norm_dyn = (self._log_norm
                        + 2.0 * np.log(sigma_py)
                        - 2.0 * np.log(self._sigma_py_near))

        # ── landing target + placement ────────────────────────────────────────
        # Placement only when we have enough frames to cover the extra distance.
        # Below _frames_placement_cutoff, intercept cleanly (offset → 0).
        if vx > 1e-3:
            # Use predicted opponent position if available, else current oy
            oy = float(predicted_opp_y) if predicted_opp_y is not None \
                 else float(belief_mu[5])
            max_p = float(placement) if placement is not None else self._max_placement
            placement_scale = float(np.clip(
                (frames - self._frames_placement_cutoff) /
                (self._frames_start - self._frames_placement_cutoff),
                0.0, 1.0))
            offset = float(np.clip(
                (oy - 0.5) * 2.0 * max_p * placement_scale,
                -max_p, max_p))
            o_star[self.IDX_PY] = float(np.clip(landing_y + offset, 0.0, 1.0))
        else:
            o_star[self.IDX_PY] = landing_y

        return o_star, C_inv_dyn, log_norm_dyn

    # ── log p̃(o) ─────────────────────────────────────────────────────────────

    def log_prior(self, o, o_star=None):
        """log p̃(o) = log N(o ; o*, C).  Uses fixed (non-adaptive) C."""
        if o_star is None:
            o_star = self.o_star
        diff = o - o_star
        return -0.5 * (diff @ self.C_inv @ diff + self._log_norm)

    # ── analytic expectation (used in EFE Phase 3) ────────────────────────────

    def expected_log_prior(self, mu_o, Sigma_o, o_star,
                           C_inv=None, log_norm=None):
        """
        E_{N(o ; mu_o, Sigma_o)}[log p̃(o)]

        Closed form:
          = -½ [ tr(C⁻¹ Σ_o) + (μ_o - o*)ᵀ C⁻¹ (μ_o - o*) + log|C| + d log 2π ]

        C_inv / log_norm: adaptive values from contextual_target; falls back
        to fixed self.C_inv / self._log_norm if not provided.
        """
        if C_inv    is None: C_inv    = self.C_inv
        if log_norm is None: log_norm = self._log_norm
        diff = mu_o - o_star
        return -0.5 * (
            np.trace(C_inv @ Sigma_o)
            + diff @ C_inv @ diff
            + log_norm
        )


# ── Likelihood model ─────────────────────────────────────────────────────────

class LikelihoodModel:
    """
    p(o | s) = N(o | H s, R)

    Full state  s = [ball_x, ball_y, ball_vx, ball_vy, player_y, opponent_y]  (6D)
    Observation o = [ball_x, ball_y, player_y, opponent_y]                    (4D)

    H is a fixed selection matrix; R is a diagonal noise covariance.
    Velocities are latent — not directly observed.
    """

    D_STATE  = 6
    D_OBS    = 4
    OBS_IDX  = [0, 1, 4, 5]   # which state dims are observed: bx, by, py, oy

    def __init__(self, r=1e-4):
        self.H = np.zeros((self.D_OBS, self.D_STATE))
        for row, col in enumerate(self.OBS_IDX):
            self.H[row, col] = 1.0
        self.R = r * np.eye(self.D_OBS)

    # ── point-state interface ─────────────────────────────────────────────────

    def predict_obs(self, s):
        """Expected observation E[o | s] = H s."""
        return self.H @ s

    def innovation(self, s, o):
        """Observation residual: o - H s."""
        return o - self.predict_obs(s)

    def log_likelihood(self, s, o):
        """log p(o | s) = log N(o ; H s, R)."""
        inn = self.innovation(s, o)
        _, logdet = np.linalg.slogdet(self.R)
        mahal = inn @ np.linalg.solve(self.R, inn)
        return -0.5 * (self.D_OBS * np.log(2 * np.pi) + logdet + mahal)

    # ── Gaussian-belief interface (for Phase 2 belief update) ─────────────────

    def log_likelihood_belief(self, mu, Sigma, o):
        """
        log p(o | q(s) = N(mu, Sigma)).
        Marginalises over the state belief:
            p(o | q) = N(o ; H mu, H Sigma H^T + R)
        """
        mu_o = self.H @ mu
        S    = self.H @ Sigma @ self.H.T + self.R
        inn  = o - mu_o
        _, logdet = np.linalg.slogdet(S)
        mahal = inn @ np.linalg.solve(S, inn)
        return -0.5 * (self.D_OBS * np.log(2 * np.pi) + logdet + mahal)

    def kalman_update(self, mu, Sigma, o):
        """
        Bayesian update of Gaussian belief given observation o.
        Returns (mu_updated, Sigma_updated, log_likelihood).

        Kalman gain:  K = Sigma H^T (H Sigma H^T + R)^{-1}
        mu*     = mu + K (o - H mu)
        Sigma*  = (I - K H) Sigma
        """
        S   = self.H @ Sigma @ self.H.T + self.R
        K   = np.linalg.solve(S, self.H @ Sigma).T    # (D_STATE, D_OBS)
        inn = o - self.H @ mu
        mu_new    = mu + K @ inn
        Sigma_new = (np.eye(self.D_STATE) - K @ self.H) @ Sigma
        ll        = -0.5 * (self.D_OBS * np.log(2 * np.pi)
                            + np.linalg.slogdet(S)[1]
                            + inn @ np.linalg.solve(S, inn))
        return mu_new, Sigma_new, ll

    # ── online noise estimation ───────────────────────────────────────────────

    def update_R(self, innovations, lr=0.01):
        """
        Exponential moving average update of R from a batch of innovations.
        innovations: (N, D_OBS) array
        """
        R_emp = (innovations.T @ innovations) / len(innovations)
        self.R = (1 - lr) * self.R + lr * R_emp

    def save(self, path="models/likelihood.pkl"):
        import pickle, pathlib
        pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump(self, f)

    @classmethod
    def from_file(cls, path="models/likelihood.pkl"):
        import pickle
        with open(path, "rb") as f:
            return pickle.load(f)


# ── Weak physics priors ───────────────────────────────────────────────────────

def _niw(m, kappa=1.0, nu_extra=2.0, scale=0.01):
    d  = len(m)
    nu = d + nu_extra           # just above minimum (d-1) so data dominates fast
    W  = scale * np.eye(d)
    return NIW(m, kappa, nu, W)


def make_ball_priors():
    """
    K=3 NIW priors for the 8D joint [bx,by,bvx,bvy | bx',by',bvx',bvy'].
    All values normalised to [0,1] (RAM byte / 255).
    Typical speed ≈ 3-4 px/frame → v ≈ 0.015 at unit scale.
    """
    v = 0.015

    # Component 0 — free flight: position advances by velocity, velocity constant
    m0 = np.array([0.50,   0.50,  v,  v,   0.50+v, 0.50+v,  v,  v])

    # Component 1 — wall bounce (top/bottom): vy flips sign
    m1 = np.array([0.50,   0.10,  v,  v,   0.50+v, 0.10-v,  v, -v])

    # Component 2 — paddle bounce: vx flips, vy slightly perturbed
    m2 = np.array([0.85,   0.50,  v,  v,   0.85-v, 0.50+v, -v,  v])

    return [_niw(m0), _niw(m1), _niw(m2)]


def make_opponent_priors():
    """
    K=2 NIW priors for the 7D joint
    [opp_y, bx, by, bvx, bvy, action | opp_y'].
    action is normalised: UP→+1, NOOP→0, DOWN→-1.

    Component 0 — tracking: tight prior (model is confident opp chases ball).
    Component 1 — stationary/lag: looser prior (more uncertain movement).
    """
    v = 0.015
    # Both components centred at neutral state; scale differentiates them
    m_base = np.array([0.5, 0.5, 0.5, v, v, 0.0,  0.5])

    return [
        _niw(m_base, scale=0.005),   # Component 0: tight  → tracking
        _niw(m_base, scale=0.02),    # Component 1: loose  → stationary / lag
    ]


# ── Transition model wrappers ─────────────────────────────────────────────────

class BallTransitionVBGS:
    """
    p(ball' | ball) as a conditional GMM. K=3 components.
    ball state: [ball_x, ball_y, ball_vx, ball_vy]  (normalised)
    """
    D_X = 4   # conditioning dimensions
    D_Y = 4   # predicted dimensions
    K   = 3

    def __init__(self):
        self.vbgs = VBGS(self.K, self.D_X + self.D_Y, make_ball_priors())

    def update(self, s_t, s_next):
        """Online CAVI step with one (s_t, s_next) transition."""
        joint = np.concatenate([s_t, s_next])[None]
        self.vbgs.fit_step(joint)

    def predict(self, s_t):
        """Returns [(weight, mean_next, cov_next), ...] for K components."""
        return self.vbgs.predict_conditional(s_t, self.D_X)

    def predict_mean(self, s_t):
        return self.vbgs.predict_mean(s_t, self.D_X)

    def save(self, path="models/ball_vbgs.pkl"):
        pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump(self.vbgs, f)

    def load(self, path="models/ball_vbgs.pkl"):
        with open(path, "rb") as f:
            self.vbgs = pickle.load(f)

    @classmethod
    def from_file(cls, path="models/ball_vbgs.pkl"):
        obj = cls.__new__(cls)
        obj.load(path)
        return obj


class OpponentTransitionVBGS:
    """
    p(opp_y' | opp_y, ball_state, action) as a conditional GMM. K=2 components.
    Input:  [opp_y, ball_x, ball_y, ball_vx, ball_vy, action_norm]
    Output: [opp_y']
    """
    D_X = 6
    D_Y = 1
    K   = 2

    # action encoding: ALE action index → normalised scalar
    _ACTION_NORM = {0: 0.0, 2: 1.0, 3: -1.0}

    def __init__(self):
        self.vbgs = VBGS(self.K, self.D_X + self.D_Y, make_opponent_priors())

    def _input(self, opp_y, ball_state, action):
        a = self._ACTION_NORM.get(int(action), 0.0)
        return np.array([opp_y, *ball_state, a], dtype=float)

    def update(self, opp_y, ball_state, action, opp_y_next):
        """Online CAVI step with one observed opponent transition."""
        x     = self._input(opp_y, ball_state, action)
        joint = np.concatenate([x, [opp_y_next]])[None]
        self.vbgs.fit_step(joint)

    def predict(self, opp_y, ball_state, action):
        """Returns [(weight, mean_next, cov_next), ...] for K components."""
        x = self._input(opp_y, ball_state, action)
        return self.vbgs.predict_conditional(x, self.D_X)

    def predict_mean(self, opp_y, ball_state, action):
        x = self._input(opp_y, ball_state, action)
        return self.vbgs.predict_mean(x, self.D_X)

    def save(self, path="models/opponent_vbgs.pkl"):
        pathlib.Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            pickle.dump(self.vbgs, f)

    def load(self, path="models/opponent_vbgs.pkl"):
        with open(path, "rb") as f:
            self.vbgs = pickle.load(f)

    @classmethod
    def from_file(cls, path="models/opponent_vbgs.pkl"):
        obj = cls.__new__(cls)
        obj.load(path)
        return obj


# ── Opponent belief tracker (Phase 2.3) ──────────────────────────────────────

class OpponentBeliefTracker:
    """
    Online belief over the opponent's behavioral mode.

    Wraps OpponentTransitionVBGS and exposes:
      - mode_probabilities()    VBGS responsibilities → P(tracking), P(lag)
      - update()                online CAVI + responsibility recording
      - predict_trajectory()    N-step rollout using current mode belief
    """

    MODES = ["tracking", "stationary/lag"]

    def __init__(self, opponent_model):
        self.model = opponent_model
        K = opponent_model.K
        self._responsibilities = np.full(K, 1.0 / K)

    # ── online update ─────────────────────────────────────────────────────────

    def update(self, opp_y, ball_state, action, opp_y_next):
        """
        Record responsibilities for the current transition, then do CAVI update.
        Responsibilities are taken BEFORE the update so they reflect the prior
        belief — i.e. which mode the model thought was active this frame.
        """
        x     = self.model._input(opp_y, ball_state, action)
        joint = np.concatenate([x, [opp_y_next]])[None]
        self._responsibilities = self.model.vbgs.responsibilities(joint)[0]
        self.model.update(opp_y, ball_state, action, opp_y_next)

    # ── belief accessors ──────────────────────────────────────────────────────

    def mode_probabilities(self):
        """Soft assignment to each behavioral mode for the last observed frame."""
        return self._responsibilities.copy()

    def dominant_mode(self):
        """Index and name of the currently dominant mode."""
        idx = int(np.argmax(self._responsibilities))
        return idx, self.MODES[idx]

    def is_tracking(self, threshold=0.6):
        """True when tracking mode dominates."""
        return self._responsibilities[0] > threshold

    # ── N-step trajectory prediction ─────────────────────────────────────────

    def predict_trajectory(self, opp_y, ball_states, actions):
        """
        Roll out opponent position N steps ahead.

        ball_states : list of N arrays [bx, by, bvx, bvy]
        actions     : list of N player ALE actions
        Returns     : list of N predicted opp_y values
        """
        trajectory  = []
        current_opp = opp_y
        for bs, a in zip(ball_states, actions):
            pred        = float(self.model.predict_mean(current_opp, bs, a)[0])
            trajectory.append(pred)
            current_opp = pred
        return trajectory

    # ── exploitation signal ───────────────────────────────────────────────────

    def opponent_gap(self, opp_y, ball_y):
        """
        Signed distance from opponent paddle to ball (normalised).
        Positive → opponent is above ball (ball will go past below).
        Negative → opponent is below ball.
        Used to identify exploitable positioning errors.
        """
        return float(opp_y - ball_y)


# ── Belief filter ─────────────────────────────────────────────────────────────

class BeliefFilter:
    """
    Single Gaussian belief q(s_t) = N(μ_t, Σ_t) over the 6D state.

    Each frame:
      Predict : VBGS mixture → moment-matched Gaussian  (ball + opponent)
      Correct : Kalman measurement update via LikelihoodModel

    State layout: s = [ball_x, ball_y, ball_vx, ball_vy, player_y, opponent_y]
    """

    D = 6
    BALL  = slice(0, 4)
    I_PY  = 4
    I_OY  = 5

    def __init__(self, ball_model, opponent_model, likelihood):
        self.ball_model     = ball_model
        self.opponent_model = opponent_model
        self.likelihood     = likelihood
        self.mu    = np.array([0.5, 0.5, 0.0, 0.0, 0.5, 0.5])
        self.Sigma = np.diag([0.1, 0.1, 0.01, 0.01, 0.1, 0.1])
        # Process noise floor — prevents Σ_pred collapsing to zero
        # after the VBGS is well-trained. Values reflect actual
        # residual prediction error (~0.009 pos, ~0.009 vel from validation).
        q_pos, q_vel, q_paddle = 1e-4, 1e-4, 1e-3
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel, q_paddle, q_pos])

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _moment_match(components):
        """Collapse [(w_k, μ_k, Σ_k)] into a single (μ, Σ)."""
        mu    = sum(w * m           for w, m, _ in components)
        Sigma = sum(w * (S + np.outer(m, m)) for w, m, S in components) \
                - np.outer(mu, mu)
        return mu, Sigma

    # ── predict step ──────────────────────────────────────────────────────────

    def predict(self, action):
        """
        Roll belief forward one step via VBGS transition models.
        action: player ALE action (used to condition opponent model).
        """
        s_ball = self.mu[self.BALL]

        # ball: 4D moment-matched prediction
        mu_ball, S_ball = self._moment_match(self.ball_model.predict(s_ball))

        # opponent: 1D moment-matched prediction
        mu_opp, S_opp = self._moment_match(
            self.opponent_model.predict(self.mu[self.I_OY], s_ball, action)
        )

        mu_pred = np.empty(self.D)
        mu_pred[self.BALL] = mu_ball
        mu_pred[self.I_PY] = np.clip(
            self.mu[self.I_PY] + PADDLE_DY.get(action, 0.0), 0.0, 1.0)
        mu_pred[self.I_OY] = float(mu_opp[0])

        Sigma_pred = np.zeros((self.D, self.D))
        Sigma_pred[np.ix_(range(4), range(4))] = S_ball
        Sigma_pred[self.I_PY, self.I_PY]       = self.Sigma[self.I_PY, self.I_PY]
        Sigma_pred[self.I_OY, self.I_OY]       = float(S_opp[0, 0])

        self.mu, self.Sigma = mu_pred, Sigma_pred + self.Q

    # ── correct step ──────────────────────────────────────────────────────────

    def correct(self, o):
        """
        Kalman update given o = [bx, by, py, oy].
        Returns log-likelihood of the observation.
        """
        self.mu, self.Sigma, ll = self.likelihood.kalman_update(
            self.mu, self.Sigma, o
        )
        return ll

    # ── public interface ──────────────────────────────────────────────────────

    def update(self, o, action):
        """Full predict-correct cycle. Returns observation log-likelihood."""
        self.predict(action)
        return self.correct(o)

    def initialise(self, o):
        """
        Warm-start from first observation.
        Sets positions from o, leaves velocities uncertain.
        """
        self.mu    = np.array([o[0], o[1], 0.0, 0.0, o[2], o[3]])
        self.Sigma = np.diag([1e-4, 1e-4, 0.01, 0.01, 1e-4, 1e-4])


# ── Mixture belief filter (Phase 2.2) ────────────────────────────────────────

class MixtureBeliefFilter:
    """
    Gaussian Sum Filter — belief as K=2 mixture of Gaussians.

    Component 0: no-wall-bounce regime  (VBGS components: free flight + paddle)
    Component 1: wall-bounce regime     (VBGS component:  wall bounce)

    Near walls component 1 gains weight from the VBGS; the observation then
    disambiguates by reweighting according to p(o | k).
    """

    K    = 2
    D    = 6
    BALL = slice(0, 4)
    I_PY = 4
    I_OY = 5

    # Ball VBGS indices assigned to each belief component:
    # 0 = free flight, 1 = wall bounce, 2 = paddle bounce
    _REGIME = [[0, 2], [1]]

    def __init__(self, ball_model, opponent_tracker, likelihood):
        self.ball_model      = ball_model
        self.opp_tracker     = opponent_tracker
        self.likelihood      = likelihood

        q_pos, q_vel, q_paddle = 1e-4, 1e-4, 1e-3
        self.Q = np.diag([q_pos, q_pos, q_vel, q_vel, q_paddle, q_pos])

        mu0 = np.array([0.5, 0.5, 0.0, 0.0, 0.5, 0.5])
        S0  = np.diag([0.1, 0.1, 0.01, 0.01, 0.1, 0.1])
        self.weights = np.full(self.K, 1.0 / self.K)
        self.mu      = np.stack([mu0.copy() for _ in range(self.K)])   # (K, D)
        self.Sigma   = np.stack([S0.copy()  for _ in range(self.K)])   # (K, D, D)

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _moment_match(components):
        """Collapse [(w, μ, Σ)] into (μ, Σ, total_weight)."""
        w_sum = sum(w for w, _, _ in components)
        if w_sum < 1e-12:
            _, m0, S0 = components[0]
            return m0, S0, 0.0
        mu    = sum(w * m           for w, m, _ in components) / w_sum
        Sigma = sum(w * (S + np.outer(m, m)) for w, m, S in components) / w_sum \
                - np.outer(mu, mu)
        return mu, Sigma, w_sum

    # ── predict step ──────────────────────────────────────────────────────────

    def predict(self, action):
        """
        Each belief component is propagated through its own VBGS regime.
        Inter-component weights updated by the VBGS mass in each regime.
        """
        new_mu    = np.empty_like(self.mu)
        new_Sigma = np.empty_like(self.Sigma)
        new_w     = np.empty(self.K)

        for k in range(self.K):
            s_ball    = self.mu[k, self.BALL]
            all_comps = self.ball_model.predict(s_ball)       # K_vbgs components

            # ball: moment-match only the VBGS components for this regime
            regime_comps      = [all_comps[j] for j in self._REGIME[k]]
            mu_ball, S_ball, w_regime = self._moment_match(regime_comps)

            # opponent: full moment-match (regime-independent)
            opp_comps         = self.opp_tracker.model.predict(
                                    self.mu[k, self.I_OY], s_ball, action)
            mu_opp, S_opp, _  = self._moment_match(opp_comps)

            mu_pred = np.empty(self.D)
            mu_pred[self.BALL] = mu_ball
            mu_pred[self.I_PY] = np.clip(
                self.mu[k, self.I_PY] + PADDLE_DY.get(action, 0.0), 0.0, 1.0)
            mu_pred[self.I_OY] = float(mu_opp[0])

            Sigma_pred = np.zeros((self.D, self.D))
            Sigma_pred[np.ix_(range(4), range(4))] = S_ball
            Sigma_pred[self.I_PY, self.I_PY]       = self.Sigma[k, self.I_PY, self.I_PY]
            Sigma_pred[self.I_OY, self.I_OY]       = float(S_opp[0, 0])

            new_mu[k]    = mu_pred
            new_Sigma[k] = Sigma_pred + self.Q
            new_w[k]     = self.weights[k] * w_regime

        w_sum = new_w.sum()
        self.weights = new_w / w_sum if w_sum > 1e-12 else np.full(self.K, 1.0 / self.K)
        self.mu      = new_mu
        self.Sigma   = new_Sigma

    # ── correct step ──────────────────────────────────────────────────────────

    def correct(self, o):
        """
        Kalman update per component; reweight by π_k · p(o|k).
        Returns log marginal likelihood log Σ_k π_k p(o|k).
        """
        new_mu    = np.empty_like(self.mu)
        new_Sigma = np.empty_like(self.Sigma)
        log_liks  = np.empty(self.K)

        for k in range(self.K):
            mu_k, Sigma_k, ll = self.likelihood.kalman_update(
                self.mu[k], self.Sigma[k], o)
            new_mu[k]    = mu_k
            new_Sigma[k] = Sigma_k
            log_liks[k]  = ll

        # log p(o) = logsumexp(log π_k + log p(o|k))
        log_joint = np.log(np.clip(self.weights, 1e-300, None)) + log_liks
        log_marg  = log_joint.max() + np.log(np.exp(log_joint - log_joint.max()).sum())

        self.weights = np.exp(log_joint - log_marg)
        self.mu      = new_mu
        self.Sigma   = new_Sigma
        return float(log_marg)

    # ── public interface ──────────────────────────────────────────────────────

    def update(self, o, action):
        """Full predict-correct cycle. Returns log marginal likelihood."""
        self.predict(action)
        return self.correct(o)

    def initialise(self, o):
        """Warm-start from first observation."""
        mu0 = np.array([o[0], o[1], 0.0, 0.0, o[2], o[3]])
        S0  = np.diag([1e-4, 1e-4, 0.01, 0.01, 1e-4, 1e-4])
        self.weights = np.full(self.K, 1.0 / self.K)
        self.mu      = np.stack([mu0.copy() for _ in range(self.K)])
        self.Sigma   = np.stack([S0.copy()  for _ in range(self.K)])

    @property
    def mean(self):
        """Mixture mean."""
        return self.weights @ self.mu

    @property
    def cov(self):
        """Mixture covariance."""
        mu = self.mean
        return sum(
            self.weights[k] * (self.Sigma[k] + np.outer(
                self.mu[k] - mu, self.mu[k] - mu))
            for k in range(self.K)
        )

    @property
    def bounce_probability(self):
        """Weight of the wall-bounce component."""
        return float(self.weights[1])
