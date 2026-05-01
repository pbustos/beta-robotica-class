# Active Inference Pong — Mathematics

## 1. State and observations

All RAM bytes are normalised to $[0,1]$ by dividing by 255.

### 1.1 Ball state

$$
s^b_t = \begin{bmatrix} b_x \\ b_y \\ \dot b_x \\ \dot b_y \end{bmatrix} \in \mathbb{R}^4
$$

Velocities are computed by frame differencing:

$$
\dot b_t = b_t - b_{t-1}
$$

### 1.2 Opponent state

$$
s^o_t = \begin{bmatrix} o_y \end{bmatrix} \in \mathbb{R}^1
$$

### 1.3 RAM addresses used

| Symbol | RAM index | Meaning |
|--------|-----------|---------|
| $b_x$  | 49        | Ball x position |
| $b_y$  | 54        | Ball y position |
| $p_y$  | 51        | Player paddle y (right) |
| $o_y$  | 50        | Opponent paddle y (left) |

---

## 2. Normal-Inverse-Wishart prior

Each mixture component $k$ has a Gaussian likelihood with unknown mean $\mu_k$ and precision $\Lambda_k$.  
The conjugate prior is the Normal-Inverse-Wishart (NIW):

$$
p(\mu_k, \Lambda_k) = \mathcal{NIW}(\mathbf{m}_k,\, \kappa_k,\, \nu_k,\, \mathbf{W}_k)
$$

with hyperparameters:

| Symbol | Meaning |
|--------|---------|
| $\mathbf{m}_k \in \mathbb{R}^d$ | prior mean |
| $\kappa_k > 0$ | mean pseudo-count |
| $\nu_k > d - 1$ | degrees of freedom |
| $\mathbf{W}_k \in \mathbb{R}^{d \times d}$ | scale matrix |

### 2.1 Variational expectations

Under the variational posterior $q(\Lambda_k) = \mathcal{W}(\nu_k, \mathbf{W}_k)$:

$$
\mathbb{E}\!\left[\log|\Lambda_k|\right]
= \sum_{i=0}^{d-1} \psi\!\left(\frac{\nu_k - i}{2}\right) + d\ln 2 + \ln|\mathbf{W}_k|
$$

where $\psi$ is the digamma function.

Under $q(\mu_k, \Lambda_k) = \mathcal{NIW}(\mathbf{m}_k, \kappa_k, \nu_k, \mathbf{W}_k)$:

$$
\mathbb{E}\!\left[(\mathbf{x} - \mu_k)^\top \Lambda_k\, (\mathbf{x} - \mu_k)\right]
= \frac{d}{\kappa_k} + \nu_k\,(\mathbf{x} - \mathbf{m}_k)^\top \mathbf{W}_k^{-1}(\mathbf{x} - \mathbf{m}_k)
$$

### 2.2 Posterior update (CAVI M-step)

Given effective count $N_k$, weighted mean $\bar{\mathbf{x}}_k$, and scatter $\mathbf{S}_k$:

$$
\kappa_k^* = \kappa_k + N_k
\qquad
\mathbf{m}_k^* = \frac{\kappa_k \mathbf{m}_k + N_k \bar{\mathbf{x}}_k}{\kappa_k^*}
$$

$$
\nu_k^* = \nu_k + N_k
$$

$$
\mathbf{W}_k^* = \mathbf{W}_k + \mathbf{S}_k
+ \frac{\kappa_k N_k}{\kappa_k^*}\,(\bar{\mathbf{x}}_k - \mathbf{m}_k)(\bar{\mathbf{x}}_k - \mathbf{m}_k)^\top
$$

### 2.3 Posterior predictive

The posterior predictive is a Student-t. For prediction we use the Gaussian approximation at the posterior mean:

$$
\hat{\Sigma}_k = \frac{\mathbf{W}_k}{\nu_k - d - 1} \qquad (\nu_k > d+1)
$$

---

## 3. Variational Bayesian Gaussian Sum (VBGS)

A joint GMM over $(\mathbf{x}, \mathbf{y})$ with $K$ components.  
The full variational family is:

$$
q(\mathbf{Z}, \boldsymbol{\pi}, \{\mu_k, \Lambda_k\})
= q(\mathbf{Z})\,q(\boldsymbol{\pi})\prod_k q(\mu_k, \Lambda_k)
$$

with $q(\boldsymbol{\pi}) = \mathrm{Dir}(\boldsymbol{\alpha})$ and each $q(\mu_k,\Lambda_k) = \mathcal{NIW}(\mathbf{m}_k,\kappa_k,\nu_k,\mathbf{W}_k)$.

### 3.1 CAVI E-step — responsibilities

For data point $\mathbf{x}_n$ and component $k$:

$$
\ln \tilde{r}_{nk}
= \psi(\alpha_k) - \psi\!\left(\sum_j \alpha_j\right)
+ \frac{1}{2}\,\mathbb{E}[\ln|\Lambda_k|]
- \frac{d}{2}\ln(2\pi)
- \frac{1}{2}\,\mathbb{E}\!\left[(\mathbf{x}_n - \mu_k)^\top\Lambda_k(\mathbf{x}_n - \mu_k)\right]
$$

$$
r_{nk} = \frac{\tilde{r}_{nk}}{\sum_j \tilde{r}_{nj}}
$$

### 3.2 CAVI M-step — sufficient statistics

$$
N_k = \sum_n r_{nk}
\qquad
\bar{\mathbf{x}}_k = \frac{1}{N_k}\sum_n r_{nk}\,\mathbf{x}_n
\qquad
\mathbf{S}_k = \sum_n r_{nk}\,(\mathbf{x}_n - \bar{\mathbf{x}}_k)(\mathbf{x}_n - \bar{\mathbf{x}}_k)^\top
$$

$$
\alpha_k^* = \alpha_0 + N_k
$$

Then apply the NIW update from §2.2 for each component.

### 3.3 Mixing weights

The posterior mixing proportions are:

$$
\hat\pi_k = \frac{\alpha_k}{\sum_j \alpha_j}
$$

---

## 4. Conditional prediction

The joint NIW for component $k$ defines a joint Gaussian $(\mathbf{x}, \mathbf{y}) \sim \mathcal{N}(\boldsymbol{\mu}_k, \hat\Sigma_k)$.  
Partitioned as:

$$
\boldsymbol{\mu}_k = \begin{bmatrix}\boldsymbol{\mu}_x \\ \boldsymbol{\mu}_y\end{bmatrix}
\qquad
\hat\Sigma_k = \begin{bmatrix}\Sigma_{xx} & \Sigma_{xy} \\ \Sigma_{yx} & \Sigma_{yy}\end{bmatrix}
$$

The conditional $p(\mathbf{y}\mid\mathbf{x}, k)$ is Gaussian:

$$
\mathbf{y} \mid \mathbf{x},\, k \;\sim\; \mathcal{N}\!\left(\,\boldsymbol{\mu}_y + \Sigma_{yx}\Sigma_{xx}^{-1}(\mathbf{x} - \boldsymbol{\mu}_x),\;\; \Sigma_{yy} - \Sigma_{yx}\Sigma_{xx}^{-1}\Sigma_{xy}\,\right)
$$

### 4.1 Component weights for prediction

When conditioning on $\mathbf{x}$, component weights are re-scored using the marginal likelihood:

$$
w_k \propto \hat\pi_k \cdot p(\mathbf{x} \mid k)
\qquad
p(\mathbf{x}\mid k) = \mathcal{N}(\mathbf{x};\;\boldsymbol{\mu}_x^k,\;\Sigma_{xx}^k)
$$

### 4.2 Mixture predictive mean

$$
\hat{\mathbf{y}} = \sum_k w_k\,\boldsymbol{\mu}_{y|x}^k
$$

---

## 5. Transition models

### 5.1 Ball transition model

Models $p(s^b_{t+1} \mid s^b_t)$ as a conditional GMM with $K=3$ components.

**Joint data:** $\mathbf{z}_t = [s^b_t;\; s^b_{t+1}] \in \mathbb{R}^8$, $d_x = 4$, $d_y = 4$.

**Components and weak physics priors** (all values in $[0,1]$, $v = 0.015$):

| $k$ | Regime | Prior mean $\mathbf{m}_k$ |
|-----|--------|--------------------------|
| 0 | Free flight | $[0.5,\;0.5,\;v,\;v \;\|\; 0.5{+}v,\;0.5{+}v,\;v,\;v]$ |
| 1 | Wall bounce ($\dot b_y$ flips) | $[0.5,\;0.1,\;v,\;v \;\|\; 0.5{+}v,\;0.1{-}v,\;v,\;{-}v]$ |
| 2 | Paddle bounce ($\dot b_x$ flips) | $[0.85,\;0.5,\;v,\;v \;\|\; 0.85{-}v,\;0.5{+}v,\;{-}v,\;v]$ |

Prior hyperparameters: $\kappa_0 = 1$, $\nu_0 = d + 2 = 10$, $\mathbf{W}_0 = 0.01\,\mathbf{I}$.  
Low pseudo-counts ensure the prior is overridden after a few dozen observations.

### 5.2 Opponent transition model

Models $p(o_{t+1} \mid o_t, s^b_t, a_t)$ as a conditional GMM with $K=2$ components.

**Input:** $\mathbf{x} = [o_y,\; b_x,\; b_y,\; \dot b_x,\; \dot b_y,\; a] \in \mathbb{R}^6$

**Action encoding:**

$$
a = \begin{cases} 0 & \text{NOOP} \\ +1 & \text{UP} \\ -1 & \text{DOWN} \end{cases}
$$

**Joint data:** $\mathbf{z} = [\mathbf{x};\; o_{t+1}] \in \mathbb{R}^7$, $d_x = 6$, $d_y = 1$.

| $k$ | Regime | Prior scale |
|-----|--------|-------------|
| 0 | Tracking (confident) | $\mathbf{W}_0 = 0.005\,\mathbf{I}$ |
| 1 | Stationary / lag | $\mathbf{W}_0 = 0.02\,\mathbf{I}$ |

---

## 6. Online learning

Both models are updated online after each observed transition using a single CAVI iteration (one E-step + one M-step). No replay buffer — the NIW sufficient statistics accumulate incrementally.

Each frame:

1. Observe $(s_t, s_{t+1})$
2. Form joint $\mathbf{z} = [s_t;\; s_{t+1}]$
3. E-step: compute $r_{k}$ for $\mathbf{z}$
4. M-step: update $\alpha_k$, $\mathbf{m}_k$, $\kappa_k$, $\nu_k$, $\mathbf{W}_k$

---

## 7. Prior preferences

The agent's goals are encoded as a preferred observation distribution:

$$
\tilde{p}(o) = \mathcal{N}(o \;;\; o^*,\; C)
$$

with $o = [b_x,\; b_y,\; p_y,\; o_y]$.

### 7.1 Goals

| Goal | Mechanism |
|------|-----------|
| **G1** Ball on opponent's side | $o^*_{b_x} = 0.15$ (low = left = opponent's goal), tight $\sigma_{b_x} = 0.05$ |
| **G2** Player paddle tracks ball | $o^*_{p_y} = \mu^{b_y}$ (set from current ball belief), tight $\sigma_{p_y} = 0.03$ |
| **G3** Ball far from opponent paddle | $o^*_{o_y} = 0.10$ (pulls opponent toward wall), moderate $\sigma_{o_y} = 0.15$ |

$o^*_{b_y}$ is left loose ($\sigma = 0.20$) — the agent doesn't care where vertically the ball is, only that it reaches the opponent's side.

### 7.2 Contextual target

Goal G2 is relational: the player should track wherever the ball is. At each step the target is updated from the current state belief $\mu$:

$$
o^*_{p_y} \leftarrow \mu_{b_y}
$$

### 7.3 Analytic expectation (used in EFE)

When the predictive observation distribution is Gaussian $p(o \mid q) = \mathcal{N}(\mu_o, \Sigma_o)$, the expected log-preference has a closed form:

$$
\mathbb{E}[\log\tilde{p}(o)] = -\frac{1}{2}\left[\operatorname{tr}(C^{-1}\Sigma_o) + (\mu_o - o^*)^\top C^{-1}(\mu_o - o^*) + \log|C| + d_o\log 2\pi\right]
$$

This is the **instrumental value** term of the Expected Free Energy (Phase 3).

---

## 8. Likelihood model

### 7.1 Full state vector

The generative model operates on a 6D state that merges the ball and paddle sub-states:

$$
s = \begin{bmatrix} b_x,\; b_y,\; \dot b_x,\; \dot b_y,\; p_y,\; o_y \end{bmatrix}^\top \in \mathbb{R}^6
$$

Velocities $(\dot b_x, \dot b_y)$ are latent — they cannot be read from a single RAM frame.

### 7.2 Observation model

The 4D observation vector contains only the directly readable positions:

$$
o = \begin{bmatrix} b_x,\; b_y,\; p_y,\; o_y \end{bmatrix}^\top \in \mathbb{R}^4
$$

The likelihood is a linear Gaussian:

$$
p(o \mid s) = \mathcal{N}(o \;;\; Hs,\; R)
$$

The observation matrix $H \in \mathbb{R}^{4 \times 6}$ selects positions from the state (indices 0, 1, 4, 5):

$$
H = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1
\end{bmatrix}
$$

The noise covariance $R = r\,\mathbf{I}_4$ with $r = 10^{-4}$, reflecting RAM quantisation noise of $\pm 1$ pixel $\approx \pm 0.004$ in normalised units.

### 7.3 Marginalised likelihood over a Gaussian belief

When the agent holds a Gaussian belief $q(s) = \mathcal{N}(\mu, \Sigma)$, the expected likelihood is obtained by marginalising over $s$:

$$
p(o \mid q) = \int p(o \mid s)\, q(s)\, ds = \mathcal{N}\!\left(o \;;\; H\mu,\; H\Sigma H^\top + R\right)
$$

### 7.4 Kalman measurement update

The observation $o_t$ updates the Gaussian belief via the standard Kalman correction:

$$
S = H\Sigma H^\top + R \qquad \text{(innovation covariance)}
$$

$$
K = \Sigma H^\top S^{-1} \qquad \text{(Kalman gain)}
$$

$$
\mu^* = \mu + K\,(o - H\mu)
$$

$$
\Sigma^* = (I - KH)\,\Sigma
$$

The log-likelihood of the innovation is:

$$
\log p(o) = -\frac{1}{2}\left(d_o \ln(2\pi) + \ln|S| + (o - H\mu)^\top S^{-1}(o - H\mu)\right)
$$

### 7.5 Online noise estimation

$R$ can be refined online from observed innovations $\nu_t = o_t - H\mu_t$ via an exponential moving average:

$$
R \leftarrow (1-\eta)\,R + \eta\,\frac{1}{N}\sum_n \nu_n \nu_n^\top
$$

with learning rate $\eta = 0.01$ (Phase 4.1).

---

## 9. Belief filter (Phase 2.1)

The agent maintains a single Gaussian belief over the full 6D state:

$$q(s_t) = \mathcal{N}(\mu_t,\, \Sigma_t)$$

Each frame the belief is updated in two sequential steps.

### 9.1 Predict step

The VBGS transition models produce a mixture of Gaussians for $p(s_{t+1} \mid s_t)$.  
This is collapsed to a single Gaussian by **moment matching**:

$$\mu^- = \sum_k w_k\, \mu^k_{y|x}$$

$$\Sigma^- = \sum_k w_k\!\left(\Sigma^k_{y|x} + \mu^k_{y|x}{\mu^k_{y|x}}^\top\right) - \mu^-{\mu^-}^\top$$

Ball (4D) and opponent (1D) are predicted independently; player_y is agent-controlled and carried forward unchanged.  
A **process noise floor** $Q$ is added to prevent $\Sigma^-$ from collapsing to zero when the VBGS is well-trained:

$$\Sigma^-_{\text{pred}} \leftarrow \Sigma^- + Q, \qquad Q = \mathrm{diag}(10^{-4},\, 10^{-4},\, 10^{-4},\, 10^{-4},\, 10^{-5},\, 10^{-4})$$

### 9.2 Correct step

The new RAM observation $o_t$ is used to update the belief via the standard Kalman measurement equations (derived in §8.4):

$$S = H\Sigma^- H^\top + R \qquad K = \Sigma^- H^\top S^{-1}$$

$$\mu_t = \mu^- + K(o_t - H\mu^-) \qquad \Sigma_t = (I - KH)\,\Sigma^-$$

### 9.3 Why this is CAVI

Minimising the variational free energy $F = \mathbb{E}_q[\log q(s) - \log p(s,o)]$ over the Gaussian family yields exactly these predict-correct equations. CAVI = Kalman for linear-Gaussian models — no approximation is made.

### 9.4 What the belief recovers

| Quantity | Source |
|----------|--------|
| $\mu_{b_x}, \mu_{b_y}$ | Smoothed ball position (less noisy than raw RAM) |
| $\mu_{\dot b_x}, \mu_{\dot b_y}$ | Ball velocity — **latent**, not in any single RAM frame |
| $\Sigma$ diagonal | Per-dimension uncertainty — feeds epistemic EFE term in Phase 3 |

---

## 10. Validation results

After pretraining on 30,053 frames (~15 s at 2,000 fps):

| Model | MAE | Pixels ($\times 255$) |
|-------|-----|----------------------|
| Ball position | 0.0089 | ≈ 2.3 px |
| Opponent position | 0.0193 | ≈ 4.9 px |

Final ball component weights: $[0.29,\; 0.46,\; 0.25]$.  
Component 1 (wall bounce) dominates, consistent with the ball spending most time bouncing off the top and bottom walls.

---

## 11. Expected Free Energy — Phase 3

### 11.1 Full EFE decomposition

For action $a$ the agent evaluates:

$$
G(a) = \underbrace{-\mathbb{E}_{q(o|a)}[\log \tilde{p}(o)]}_{\text{instrumental}} \;-\; \lambda\;\underbrace{\tfrac{1}{2}\log\frac{|S(a)|}{|R|}}_{\text{epistemic}}
$$

with $\lambda \ge 0$ controlling how much the agent values information gain.

### 11.2 Predictive observation distribution

The agent first computes a virtual one-step-ahead belief without side effects:

$$
(\mu^-_a,\;\Sigma^-_a) = \text{predict}(\mu, \Sigma, a)
$$

The predictive observation distribution is then:

$$
q(o \mid a) = \mathcal{N}(o\,;\; H\mu^-_a,\; S(a)), \qquad S(a) = H\Sigma^-_a H^\top + R
$$

### 11.3 Instrumental term

Using the analytic expectation from §7.3 with $\Sigma_o = S(a)$ and $\mu_o = H\mu^-_a$:

$$
G_\text{instr}(a) = \tfrac{1}{2}\!\left[\operatorname{tr}(C^{-1} S(a)) + (H\mu^-_a - o^*)^\top C^{-1}(H\mu^-_a - o^*) + \log|C| + d_o\log 2\pi\right]
$$

where $o^*$ is the contextual target (§7.2) and $C$ is the preference covariance.

### 11.4 Epistemic term — mutual information

The epistemic term equals the mutual information $I(s'; o \mid a)$ between the next state $s'$ and the next observation $o$, evaluated under the current belief:

$$
G_\text{epist}(a) = I(s';\,o\mid a) = \tfrac{1}{2}\!\left(\log|S(a)| - \log|R|\right)
$$

**Derivation:**  
$I(s';o) = H(o) - H(o \mid s')$.  
$H(o) = \tfrac{1}{2}\log|2\pi e\, S|$ and $H(o\mid s') = \tfrac{1}{2}\log|2\pi e\, R|$.  
Their difference gives the formula above.  

When $\Sigma^-_a \to 0$ (belief is certain), $S \to R$ and $G_\text{epist} \to 0$.  
When uncertainty is large, $G_\text{epist} > 0$ — the agent gains information.

Subtracting $\lambda G_\text{epist}$ from $G$ therefore rewards actions that resolve uncertainty.

### 11.5 Action selection (1-step)

$$
a^* = \arg\min_a\; G(a) = \arg\min_a\;\left[G_\text{instr}(a) - \lambda\, G_\text{epist}(a)\right]
$$

$\lambda = 0$ recovers Phase 3.1 (instrumental only).  
$\lambda > 0$ adds curiosity — the agent prefers actions that lead to more informative observations.

---

## 12. N-step planning horizon (Phase 3.3)

### 12.1 Motivation

Single-step EFE sees only one frame ahead: the paddle moves $\pm 4$ px per step.  
If the ball is 40 px away, 1-step lookahead cannot prefer UP over NOOP because the gap remains large regardless. Planning $N$ steps ahead lets the agent "see" that $N$ consecutive UPs will close the gap, while $N$ NOOPs will not.

### 12.2 Policy tree

A **policy** is a fixed action sequence $\pi = (a_0, a_1, \ldots, a_{N-1})$ with $a_t \in \{0, 2, 3\}$.  
The number of policies is $3^N$ (27 for $N=3$, 243 for $N=5$).

### 12.3 Discounted EFE for a policy

For each policy $\pi$, the agent unrolls the belief forward $N$ steps **in simulation** (predict steps only, no observation updates):

$$
s_0^- = (\mu_t, \Sigma_t) \quad \text{(current belief)}
$$

$$
s_{k+1}^- = \text{predict}(s_k^-, a_k), \quad k = 0, \ldots, N-1
$$

At each step $k$ the per-step EFE is evaluated under $s_k^-$:

$$
G_k(a_k) = G_\text{instr}(a_k \mid s_k^-) - \lambda\, G_\text{epist}(a_k \mid s_k^-)
$$

The policy score is the **discounted sum**:

$$
G(\pi) = \sum_{k=0}^{N-1} \gamma^k\, G_k(a_k), \qquad \gamma \in (0,1]
$$

$\gamma < 1$ discounts future uncertainty — later steps are based on predictions of predictions, so they are noisier.  Default: $\gamma = 0.9$.

### 12.4 Action selection

$$
\pi^* = \arg\min_\pi\; G(\pi)
$$

The agent executes only the **first action** $a^* = \pi^*_0$ and replans at the next frame.  
This is the "receding horizon" or Model Predictive Control (MPC) strategy.

### 12.5 Hybrid two-model rollout

Naively evaluating all $3^N$ policies with full VBGS at every step costs $\mathcal{O}(3^N \cdot N \cdot T_\text{VBGS})$, where $T_\text{VBGS} \approx 0.7\,\text{ms}$ — 677 ms per frame at $N=5$ (intractable).

**Optimisation:** Step 0 uses the VBGS (wall-bounce-aware); steps $1 \ldots N-1$ use a cheap linear-Gaussian rollout with constant-velocity ball dynamics and Q noise:

$$
\mu_{t+1} = F\mu_t + \delta_a, \quad \Sigma_{t+1} = F\Sigma_t F^\top + Q
$$

where $F$ integrates velocity ($b_x \mathrel{+}= v_x$, $b_y \mathrel{+}= v_y$) and $\delta_a$ applies the paddle kinematics.

Tree structure: only 3 VBGS calls (one per first action), then $3^2 + \cdots + 3^N$ linear calls. At $N=5$: 3 VBGS + 360 linear = **6.7 ms/frame** (100× speedup).

### 12.6 Consistent preference target

The intercept target $o^*_{p_y}$ is computed **once per frame** from the current belief mean and shared across all branches of the tree. Computing it separately inside each branch introduces action-dependent noise (the VBGS slightly changes the ball estimate per action), which can corrupt the UP vs DOWN comparison. The single shared target eliminates this.

---

## 13. Ball-intercept prior preference (Phase 3.3+)

### 13.1 Motivation

The naive preference $o^*_{p_y} = \mu_{b_y}$ (track current ball y) is purely reactive.  
The agent should instead target **where the ball will arrive** at the player's paddle, computed analytically from the current belief velocity.

### 13.2 Landing prediction

Given the current belief mean $\mu = (b_x, b_y, v_x, v_y, p_y, o_y)^\top$:

1. Clamp velocity to $|v_x|, |v_y| \le 0.06$ (suppress VBGS noise spikes from bounces).
2. If $v_x \le 10^{-3}$ (ball moving away): return current $b_y$.
3. Simulate forward: $b_x \mathrel{+}= v_x$, $b_y \mathrel{+}= v_y$ each step; reflect $v_y$ when $b_y \notin [0,1]$.
4. Stop when $b_x \ge x_\text{player} \approx 0.73$ (measured contact point). Return $b_y$.

This converts the tracking problem into an **interception problem**: the EFE minimisation aligns the paddle with the future ball position from the moment the ball is served, giving the agent the full travel time to reposition.

### 13.3 Why this helps

With reactive tracking $o^*_{p_y} = \mu_{b_y}$:
- EFE only penalises current ball–paddle gap
- Agent learns to track, not to anticipate
- Ball arrives at $b_y \ne p_y$ if it curves during flight

With intercept targeting $o^*_{p_y} = y_\text{land}$:
- EFE penalises gap to the **final** position from frame 1
- Paddle starts moving immediately after serve
- Each new frame refines the velocity estimate and updates the target

### 13.4 Validated results (Phase 3.3)

| Agent | Mean score | Best episode |
|-------|-----------|------|
| EFE H=3 (intercept) | **−10.8** | **−2** |
| EFE H=1 (intercept) | −11.7 | −6 |
| EFE H=5 (intercept) | −14.0 | −8 |
| Rule-based baseline | −14.8 | −12 |

EFE H=3 with intercept targeting beats rule-based by **+4 points** (mean).  
H=5 underperforms because the linear rollout's constant-velocity approximation drifts at depth 4–5 without wall-bounce corrections.

---

## 14. Online learning — Phase 4

Three sub-phases of online refinement run continuously while the agent plays.

### 14.1 Online R estimation (observation noise)

The initial likelihood noise $R = 10^{-4} I$ is a fixed guess.  
After each predict step, the prediction innovation is:

$$
\nu_t = o_t - H \hat{\mu}_{t|t-1}
$$

where $\hat{\mu}_{t|t-1}$ is the mixture mean after `predict()` but before `correct()`.  
Innovations are accumulated in a buffer of $B=20$ frames, then used to update $R$ via EMA:

$$
R \leftarrow (1 - \eta_R)\,R + \eta_R \cdot \frac{1}{B}\sum_{i} \nu_i \nu_i^\top, \qquad \eta_R = 0.005
$$

The small learning rate prevents the noisy single-frame innovations from destabilising the filter.

### 14.2 Online ball VBGS refinement

Each in-play frame produces a transition $(s_t, s_{t+1})$ where both states are approximated by the post-correct belief mean $\mu_{t|t}[:4]$.  
One CAVI E+M step is applied to the joint $\mathbf{z} = [s_t;\; s_{t+1}]$:

$$
r_k \leftarrow \text{E-step}(\mathbf{z}), \qquad
\theta_k \leftarrow \text{M-step}(r_k, \mathbf{z})
$$

Because the pretrained NIW sufficient statistics encode $\sim 30\,000$ data points, each new frame shifts the posterior by $\approx 1/30\,000$ — slow, noise-resistant refinement.

### 14.3 Online opponent model

Already active since Phase 2.3: `OpponentBeliefTracker.update()` calls `OpponentTransitionVBGS.update()` every in-play frame.  
No additional wiring required.

### Episode structure change

`validate_efe.py` creates fresh models from disk each episode — no cross-episode learning.  
`train_efe.py` loads models **once** and shares them across all episodes so updates accumulate:

```
Frozen (validate_efe.py)      Online (train_efe.py)
  ep 1: load → run → discard     ep 1: load → run │
  ep 2: load → run → discard     ep 2:       run  │ ← models persist
  ep 3: load → run → discard     ep 3:       run  │
```

---

## 8. Roadmap

| Phase | Description | Status |
|-------|-------------|--------|
| 0 | Environment, RAM observations, rule-based baseline | ✅ |
| 1.1 | State representation | ✅ |
| 1.2 | Transition models (VBGS) | ✅ |
| 1.3 | Likelihood model $p(o\mid s)$ | ✅ |
| 1.4 | Prior preferences | ✅ |
| 2.1 | Single Gaussian belief (CAVI / Kalman) | ✅ |
| 2.2 | Mixture of Gaussians belief near walls | ✅ |
| 2.3 | Online opponent model | ✅ |
| 3.1 | EFE — instrumental value only | ✅ |
| 3.2 | EFE — full with epistemic term | ✅ |
| 3.3 | Planning horizon $N$ steps | ✅ |
| 4.1 | Online R estimation from innovations | ✅ |
| 4.2 | Online ball VBGS from live transitions | ✅ |
| 4.3 | Online opponent model (active since 2.3) | ✅ |
| 5 | 100-episode evaluation vs rule-based and RL baseline | — |
