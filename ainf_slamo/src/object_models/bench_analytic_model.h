#pragma once

#include <cstdint>
#include <vector>

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
 * BenchAnalyticModel — 6-DOF analytic shape model for a park bench.
 *
 * Params layout: [tx, ty, yaw, width, depth, height].
 *   - (tx, ty, yaw): pose in the room XY plane.
 *   - width:         long dimension (local X).
 *   - depth:         short dimension (local Y).
 *   - height:        total height from floor to top of backrest.
 *
 * SDF composition (union via min):
 *   1. Seat slab: full width × depth, thin horizontal box at seat height.
 *   2. Backrest: full width × thin slab at rear (-Y local) edge, from seat
 *      to total height.
 *   3. Two stout side supports at ±X ends, spanning most of the depth,
 *      from floor to seat.
 */
class BenchAnalyticModel
{
public:
    static constexpr std::int64_t kParamSize = 6;

    struct FitResult
    {
        bool ok = false;
        torch::Tensor params;
        float initial_loss = 0.f;
        float final_loss   = 0.f;
        int   iterations   = 0;
    };

    explicit BenchAnalyticModel(const torch::Tensor& initial_params);

    torch::Tensor forward(const torch::Tensor& points_xyz) const;
    static torch::Tensor forward_sdf(const torch::Tensor& points_xyz,
                                     const torch::Tensor& params_6);
    torch::Tensor score_points(const torch::Tensor& points_xyz, float beta = 10.f) const;
    FitResult     fit_autograd(const torch::Tensor& points_xyz,
                               int max_iterations = 80, float learning_rate = 0.03f);
    torch::Tensor params() const;

private:
    torch::Tensor state_;  // [tx, ty, yaw, log_w, log_d, log_h]
};

} // namespace rc::object_models
