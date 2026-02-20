/*
 * Localizer - Monte Carlo Localization (AMCL) for 2D robot localization
 *
 * Based on MRPT's CMonteCarloLocalization2D approach:
 * - Particle filter with adaptive sample size (KLD-sampling)
 * - Likelihood field model using ESDF
 * - Motion model with configurable noise
 *
 * Copyright 2024
 */

#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <Eigen/Dense>
#include <vector>
#include <random>
#include <optional>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include "grid_esdf.h"

class Localizer
{
public:
    //////////////////////////////////////////////////////////////////////////
    // Types
    //////////////////////////////////////////////////////////////////////////

    // 2D Pose: x, y (mm), theta (rad)
    struct Pose2D
    {
        float x = 0.f;
        float y = 0.f;
        float theta = 0.f;

        Pose2D() = default;
        Pose2D(float x_, float y_, float theta_) : x(x_), y(y_), theta(theta_) {}

        // Convert to Eigen for convenience
        Eigen::Vector2f position() const { return {x, y}; }
        Eigen::Affine2f toAffine() const
        {
            Eigen::Affine2f t;
            t.translation() = Eigen::Vector2f(x, y);
            t.linear() = Eigen::Rotation2Df(theta).toRotationMatrix();
            return t;
        }

        // Compose two poses: this (+) other
        Pose2D compose(const Pose2D& delta) const
        {
            float c = std::cos(theta);
            float s = std::sin(theta);
            return {
                x + delta.x * c - delta.y * s,
                y + delta.x * s + delta.y * c,
                normalizeAngle(theta + delta.theta)
            };
        }

        static float normalizeAngle(float a)
        {
            while (a > M_PI) a -= 2 * M_PI;
            while (a < -M_PI) a += 2 * M_PI;
            return a;
        }
    };

    // Particle: pose + weight
    struct Particle
    {
        Pose2D pose;
        double log_weight = 0.0;  // Log-weight for numerical stability

        // Default constructor
        Particle() = default;

        // Get weight efficiently (intentionally inline for speed)
        inline double getWeight() const {
            return std::exp(log_weight);
        }
    };

    // Odometry increment (can come from real odometry or simulated from absolute poses)
    struct OdometryDelta
    {
        float delta_x = 0.f;      // Forward displacement (robot frame)
        float delta_y = 0.f;      // Lateral displacement (robot frame)
        float delta_theta = 0.f;  // Rotation
    };

    //////////////////////////////////////////////////////////////////////////
    // Configuration
    //////////////////////////////////////////////////////////////////////////

    struct Params
    {
        // Particle count (balanced for precision and performance)
        size_t min_particles = 100;
        size_t max_particles = 1000;      // Increased back from 600
        size_t initial_particles = 500;   // Increased back from 300

        // KLD-sampling parameters (adaptive sample size)
        float kld_bin_size_xy = 200.f;    // mm - spatial binning
        float kld_bin_size_theta = 0.1f;  // rad - angular binning
        float kld_delta = 0.01f;          // Confidence (1 - delta)
        float kld_epsilon = 0.02f;        // KL-distance bound

        // Motion model noise (proportional to movement) - defaults for WITHOUT ODOMETRY
        float alpha1 = 0.25f;  // Rotation noise from rotation
        float alpha2 = 0.10f;  // Rotation noise from translation
        float alpha3 = 0.25f;  // Translation noise from translation
        float alpha4 = 0.10f;  // Translation noise from rotation

        // Minimum diffusion noise - defaults for WITHOUT ODOMETRY
        float min_trans_diffusion = 50.f;   // mm per cycle (~1000mm/s at 20Hz)
        float min_rot_diffusion = 0.05f;    // rad per cycle (~3 degrees)

        // Observation model
        float sigma_hit = 100.f;     // mm - std dev for hit model
        float z_hit = 0.95f;         // Weight for hit model
        float z_rand = 0.05f;        // Weight for random model
        float max_range = 15000.f;   // mm - max LiDAR range
        int lidar_subsample = 15;    // Reverted from 25 to 15 for better precision

        // Resampling
        float resample_threshold = 0.5f;  // Resample when ESS < threshold * N

        // Convergence detection
        float position_stddev_threshold = 200.f;  // mm - consider converged below this
        float angle_stddev_threshold = 0.1f;      // rad

        // Visualization
        bool draw_particles = false;
        float particle_size = 50.f;  // mm
    };

    //////////////////////////////////////////////////////////////////////////
    // Public Interface
    //////////////////////////////////////////////////////////////////////////

    Localizer() = default;
    ~Localizer();

    // Initialize with map and scene for visualization
    void initialize(GridESDF* map, QGraphicsScene* scene = nullptr);

    // Set parameters
    void setParams(const Params& p) { params_ = p; }
    Params& params() { return params_; }
    const Params& params() const { return params_; }

    // Reset particles uniformly in free space
    void resetUniform(float x_min, float x_max,
                      float y_min, float y_max,
                      float theta_min = -M_PI, float theta_max = M_PI);

    // Reset particles around a known pose (e.g., from GPS or manual placement)
    void resetGaussian(const Pose2D& mean, float std_xy, float std_theta);

    // Main update: prediction + correction
    // Returns estimated pose if localization is confident, nullopt otherwise
    std::optional<Pose2D> update(const OdometryDelta& odometry,
                                  const std::vector<Eigen::Vector2f>& lidar_points_local);

    // Get current estimate (mean pose weighted by particles)
    Pose2D getMeanPose() const;

    // Get pose covariance (2x2 for position, separate for theta)
    void getCovariance(Eigen::Matrix2f& pos_cov, float& theta_var) const;

    // Get full 3x3 covariance matrix as vector [xx, xy, xθ, yy, yθ, θθ] (upper triangular)
    std::vector<float> getCovarianceVector() const;

    // Check if localization has converged
    bool isConverged() const;

    // Get number of effective particles (ESS)
    double getEffectiveSampleSize() const;

    // Get current particle count
    size_t getParticleCount() const { return particles_.size(); }

    // Get copy of particles for visualization (thread-safe, call from main thread)
    std::vector<Particle> getParticlesCopy() const { return particles_; }

    // Visualization (must be called from Qt main thread!)
    void drawParticles();
    void clearParticleVisualization();  // Clear all particle graphics from scene
    void clearVisualization();

    //////////////////////////////////////////////////////////////////////////
    // Utility: Generate odometry delta from two absolute poses
    //////////////////////////////////////////////////////////////////////////
    static OdometryDelta computeOdometryDelta(const Pose2D& prev, const Pose2D& curr);

    // Add noise to odometry (to simulate from ground truth)
    OdometryDelta addNoiseToOdometry(const OdometryDelta& odom);

private:
    //////////////////////////////////////////////////////////////////////////
    // Internal Methods
    //////////////////////////////////////////////////////////////////////////

    // Prediction step: propagate particles through motion model
    void predict(const OdometryDelta& odometry);

    // Correction step: weight particles by observation likelihood
    void correct(const std::vector<Eigen::Vector2f>& lidar_points_local);

    // Compute likelihood of a LiDAR scan given a particle pose
    double computeLikelihood(const Pose2D& pose,
                             const std::vector<Eigen::Vector2f>& lidar_points_local);

    // Resample particles using low-variance resampling
    void resample();

    // KLD-adaptive resampling
    void resampleKLD();

    // Fused operation: normalize weights + compute ESS + compute mean pose (single pass)
    void normalizeAndComputeStats();

    // Normalize weights (convert from log-weights) - kept for compatibility
    void normalizeWeights();

    // Compute required number of samples for KLD
    size_t computeKLDSampleCount(size_t num_bins) const;

    // Trim particles when filter is stable (high ESS) to save computation
    void trimParticlesWhenStable();

    // Sample from motion model
    Pose2D sampleMotionModel(const Pose2D& pose, const OdometryDelta& odometry);

    //////////////////////////////////////////////////////////////////////////
    // Member Variables
    //////////////////////////////////////////////////////////////////////////

    Params params_;
    std::vector<Particle> particles_;
    GridESDF* map_ = nullptr;
    QGraphicsScene* scene_ = nullptr;

    // Visualization items
    std::vector<QGraphicsEllipseItem*> particle_items_;

    // Random number generator
    std::mt19937 rng_{std::random_device{}()};
    std::normal_distribution<float> normal_dist_{0.f, 1.f};
    std::uniform_real_distribution<float> uniform_dist_{0.f, 1.f};

    // State
    bool initialized_ = false;
    Pose2D last_mean_pose_;

    // Cached statistics (computed in normalizeAndComputeStats)
    double cached_ess_ = 0.0;
    Pose2D cached_mean_pose_;
};

#endif // LOCALIZER_H
