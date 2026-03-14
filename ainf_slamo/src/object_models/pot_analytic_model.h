#pragma once

#include <cstdint>

#ifdef slots
  #define RC_QT_SLOTS_WAS_DEFINED
  #undef slots
#endif
#ifdef signals
  #define RC_QT_SIGNALS_WAS_DEFINED
  #undef signals
#endif
#ifdef emit
  #define RC_QT_EMIT_WAS_DEFINED
  #undef emit
#endif

#include <torch/torch.h>

#ifdef RC_QT_SLOTS_WAS_DEFINED
  #define slots Q_SLOTS
  #undef RC_QT_SLOTS_WAS_DEFINED
#endif
#ifdef RC_QT_SIGNALS_WAS_DEFINED
  #define signals Q_SIGNALS
  #undef RC_QT_SIGNALS_WAS_DEFINED
#endif
#ifdef RC_QT_EMIT_WAS_DEFINED
  #define emit Q_EMIT
  #undef RC_QT_EMIT_WAS_DEFINED
#endif

namespace rc::object_models
{
/**
 * PotAnalyticModel — 5-DOF analytic shape model for a plant pot (truncated cone).
 *
 * Params layout: [tx, ty, yaw, radius, height].
 *   - (tx, ty, yaw): pose (yaw has no geometric effect for a body of
 *                    revolution but is kept for API consistency).
 *   - radius:        top (larger) radius of the truncated cone.
 *   - height:        total height from the floor.
 *
 * SDF: exact signed distance to a capped cone (frustum) centred at (tx, ty)
 * with bottom radius = 0.65 * radius at z = 0 and top radius = radius at z = h.
 *
 * Reference: Inigo Quilez, "distance functions",
 *   https://iquilezles.org/articles/distfunctions/
 *   Function: sdCappedCone(p, h, r1, r2)
 */
class PotAnalyticModel
{
public:
    static constexpr std::int64_t kParamSize = 5;

    struct FitResult
    {
        bool ok = false;
        torch::Tensor params;   // [5], CPU float32
        float initial_loss = 0.f;
        float final_loss   = 0.f;
        int   iterations   = 0;
    };

    explicit PotAnalyticModel(const torch::Tensor& initial_params);

    torch::Tensor forward(const torch::Tensor& points_xyz) const;
    static torch::Tensor forward_sdf(const torch::Tensor& points_xyz,
                                     const torch::Tensor& params_5);
    torch::Tensor score_points(const torch::Tensor& points_xyz, float beta = 10.f) const;
    FitResult     fit_autograd(const torch::Tensor& points_xyz,
                               int max_iterations = 80, float learning_rate = 0.03f);
    torch::Tensor params() const;

private:
    torch::Tensor state_;  // [tx, ty, yaw, log_r, log_h]
};

} // namespace rc::object_models
