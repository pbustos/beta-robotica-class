#pragma once

#include <cstdint>
#include <vector>

// ---- PyTorch vs Qt macros (slots/signals/emit) ----
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
 * ChairAnalyticModel — 6-DOF analytic shape model for a chair.
 *
 * Params layout: [tx, ty, yaw, width, depth, seat_height].
 *   - (tx, ty, yaw): pose in the room XY plane (Z is up).
 *   - width:        lateral extent (local X).
 *   - depth:        front-to-back extent (local Y); the backrest is at +Y.
 *   - seat_height:  height of the seat surface from the floor.
 *
 * SDF composition (smooth union via min):
 *   1. Seat slab:  full width × depth, thin horizontal box at the seat level.
 *   2. Backrest:   thin vertical panel along the rear edge, seat_height tall.
 *   3. Four legs:  boxes at the four corners, height = seat_height − seat slab.
 */
class ChairAnalyticModel
{
public:
    static constexpr std::int64_t kParamSize = 6;

    struct FitResult
    {
        bool ok = false;
        torch::Tensor params;   // [6], CPU float32
        float initial_loss = 0.f;
        float final_loss   = 0.f;
        int   iterations   = 0;
    };

    explicit ChairAnalyticModel(const torch::Tensor& initial_params);

    /// Returns signed distances for each point in points_xyz [N,3].
    torch::Tensor forward(const torch::Tensor& points_xyz) const;

    /// Static version operating on an explicit params tensor [6].
    static torch::Tensor forward_sdf(const torch::Tensor& points_xyz,
                                     const torch::Tensor& params_6);

    /// E-step helper: converts distances into a likelihood-like score in (0,1].
    torch::Tensor score_points(const torch::Tensor& points_xyz, float beta = 10.f) const;

    /// M-step helper: optimise the 6D analytic state with autograd.
    FitResult fit_autograd(const torch::Tensor& points_xyz,
                           int   max_iterations = 80,
                           float learning_rate  = 0.03f);

    torch::Tensor params() const;

private:
    torch::Tensor state_;  // [tx, ty, yaw, log_w, log_d, log_h]
};

} // namespace rc::object_models
