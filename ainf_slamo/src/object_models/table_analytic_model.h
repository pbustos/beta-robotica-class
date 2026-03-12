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
class TableAnalyticModel
{
public:
    // Params layout: [tx, ty, yaw, width, depth, height].
    static constexpr std::int64_t kParamSize = 6;

    struct FitResult
    {
        bool ok = false;
        torch::Tensor params;  // [6], CPU float32
        float initial_loss = 0.f;
        float final_loss = 0.f;
        int iterations = 0;
    };

    explicit TableAnalyticModel(const torch::Tensor& initial_params);

    // Returns signed distances for each point in points_xyz [N,3].
    // Negative values are inside the analytic table cuboid model.
    torch::Tensor forward(const torch::Tensor& points_xyz) const;

    // Static version operating on an explicit params tensor [6].
    static torch::Tensor forward_sdf(const torch::Tensor& points_xyz,
                                     const torch::Tensor& params_6);

    // E-step helper: converts distances into a likelihood-like score in (0,1].
    torch::Tensor score_points(const torch::Tensor& points_xyz, float beta = 10.f) const;

    // M-step helper: optimize the 6D analytic state with autograd.
    FitResult fit_autograd(const torch::Tensor& points_xyz,
                           int max_iterations = 80,
                           float learning_rate = 0.03f);

    torch::Tensor params() const;

private:
    // Internal learnable state. Width/depth/height are stored in log-space
    // and mapped through exp() in forward_sdf to guarantee positivity.
    torch::Tensor state_;  // [tx, ty, yaw, log_w, log_d, log_h]
};
}