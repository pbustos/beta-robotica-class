/*
 * Localizer - Monte Carlo Localization (AMCL) Implementation
 *
 * Based on MRPT's approach and Thrun's Probabilistic Robotics book.
 */

#include "localizer.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <set>
#include <unordered_set>
#include <QBrush>
#include <QPen>

// Hash function for std::tuple<int, int, int> (for KLD binning)
namespace std {
    template <>
    struct hash<std::tuple<int, int, int>> {
        size_t operator()(const std::tuple<int, int, int>& t) const {
            size_t h1 = std::hash<int>()(std::get<0>(t));
            size_t h2 = std::hash<int>()(std::get<1>(t));
            size_t h3 = std::hash<int>()(std::get<2>(t));
            return h1 ^ (h2 << 10) ^ (h3 << 20);
        }
    };
}

Localizer::~Localizer()
{
    clearVisualization();
}

void Localizer::initialize(GridESDF* map, QGraphicsScene* scene)
{
    map_ = map;
    scene_ = scene;
    initialized_ = false;
    particles_.clear();
}

//////////////////////////////////////////////////////////////////////////////
// Reset Methods
//////////////////////////////////////////////////////////////////////////////

void Localizer::resetUniform(float x_min, float x_max,
                              float y_min, float y_max,
                              float theta_min, float theta_max)
{
    particles_.clear();
    particles_.reserve(params_.initial_particles);

    std::uniform_real_distribution<float> x_dist(x_min, x_max);
    std::uniform_real_distribution<float> y_dist(y_min, y_max);
    std::uniform_real_distribution<float> theta_dist(theta_min, theta_max);

    // Generate particles in free space
    size_t attempts = 0;
    const size_t max_attempts = params_.initial_particles * 100;

    while (particles_.size() < params_.initial_particles && attempts < max_attempts)
    {
        attempts++;
        Pose2D pose{x_dist(rng_), y_dist(rng_), theta_dist(rng_)};

        // Check if position is in free space using the map
        if (map_)
        {
            auto key = map_->point_to_key(pose.x, pose.y);
            if (map_->is_obstacle(key))
                continue;  // Skip occupied cells
        }

        Particle p;
        p.pose = pose;
        p.log_weight = 0.0;  // Uniform weight initially
        particles_.push_back(p);
    }

    // If we couldn't find enough free cells, just fill with random poses
    while (particles_.size() < params_.initial_particles)
    {
        Particle p;
        p.pose = {x_dist(rng_), y_dist(rng_), theta_dist(rng_)};
        p.log_weight = 0.0;
        particles_.push_back(p);
    }

    initialized_ = true;
    qDebug() << "[Localizer] Reset uniform with" << particles_.size() << "particles";
}

void Localizer::resetGaussian(const Pose2D& mean, float std_xy, float std_theta)
{
    particles_.clear();
    particles_.reserve(params_.initial_particles);

    std::normal_distribution<float> xy_dist(0.f, std_xy);
    std::normal_distribution<float> theta_dist(0.f, std_theta);

    for (size_t i = 0; i < params_.initial_particles; ++i)
    {
        Particle p;
        p.pose.x = mean.x + xy_dist(rng_);
        p.pose.y = mean.y + xy_dist(rng_);
        p.pose.theta = Pose2D::normalizeAngle(mean.theta + theta_dist(rng_));
        p.log_weight = 0.0;
        particles_.push_back(p);
    }

    initialized_ = true;
    last_mean_pose_ = mean;
    qDebug() << "[Localizer] Reset Gaussian around" << mean.x << mean.y
             << "with" << particles_.size() << "particles";
}

//////////////////////////////////////////////////////////////////////////////
// Main Update
//////////////////////////////////////////////////////////////////////////////

std::optional<Localizer::Pose2D> Localizer::update(
    const OdometryDelta& odometry,
    const std::vector<Eigen::Vector2f>& lidar_points_local)
{
    if (!initialized_ || particles_.empty())
    {
        qWarning() << "[Localizer] Not initialized!";
        return std::nullopt;
    }

    // 1. Prediction step: propagate particles through motion model
    predict(odometry);

    // 2. Correction step: weight particles by observation likelihood
    if (!lidar_points_local.empty())
    {
        correct(lidar_points_local);
    }

    // 3. Fused operation: normalize weights + compute ESS + compute mean pose (single pass)
    normalizeAndComputeStats();

    // 4. Resample if needed (based on ESS from cached value)
    const double ess_ratio = cached_ess_ / static_cast<double>(particles_.size());

    if (cached_ess_ < params_.resample_threshold * particles_.size())
    {
        // Standard resampling when ESS is low
        resampleKLD();
        static int resample_log_counter = 0;
        if (++resample_log_counter % 50 == 0)  // Log less frequently
        {
            qDebug() << "[Localizer] Resampled, ESS was" << cached_ess_
                     << ", new particle count:" << particles_.size();
        }
    }
    else if (ess_ratio > 0.5 && particles_.size() > params_.min_particles * 2)
    {
        // When ESS is reasonably high (filter stable) and we have many particles,
        // periodically reduce particle count to save computation.
        // Lower threshold (0.5 instead of 0.85) allows earlier reduction.
        static int stable_counter = 0;
        if (++stable_counter >= 20)  // Every ~1 second at 20Hz
        {
            stable_counter = 0;
            // Trim particles while maintaining distribution
            trimParticlesWhenStable();
        }
    }

    // 5. Update mean pose from cache
    last_mean_pose_ = cached_mean_pose_;

    // NOTE: Visualization is now handled by main thread calling drawParticles()
    // to avoid Qt thread-safety issues

    // Return pose only if converged
    if (isConverged())
        return last_mean_pose_;
    else
        return std::nullopt;
}

//////////////////////////////////////////////////////////////////////////////
// Prediction Step
//////////////////////////////////////////////////////////////////////////////

void Localizer::predict(const OdometryDelta& odometry)
{
    for (auto& particle : particles_)
    {
        particle.pose = sampleMotionModel(particle.pose, odometry);
    }
}

Localizer::Pose2D Localizer::sampleMotionModel(const Pose2D& pose, const OdometryDelta& odometry)
{
    // Simplified odometry motion model
    // Odometry comes in robot frame, apply directly with noise

    const float delta_trans = std::hypot(odometry.delta_x, odometry.delta_y);
    const float delta_rot = std::abs(odometry.delta_theta);

    // Adaptive diffusion: when commanded velocity is ~0 (robot should be stationary),
    // reduce diffusion to allow covariance to converge. When moving, use full diffusion.
    // This allows the filter to be precise when stopped while still tracking external motion.

    // Threshold for "stopped" (in mm displacement per cycle at 20Hz)
    // ~5mm/cycle = ~100mm/s, below this we consider the robot commanded to stop
    constexpr float STOPPED_TRANS_THRESHOLD = 5.0f;   // mm
    constexpr float STOPPED_ROT_THRESHOLD = 0.01f;    // rad (~0.6 degrees)

    // Calculate effective diffusion based on commanded motion
    float effective_trans_diffusion = params_.min_trans_diffusion;
    float effective_rot_diffusion = params_.min_rot_diffusion;

    if (delta_trans < STOPPED_TRANS_THRESHOLD && delta_rot < STOPPED_ROT_THRESHOLD)
    {
        // Robot is commanded to be stationary - use reduced diffusion
        // This allows covariance to converge when the robot is stopped
        // Still maintain some minimum for sensor noise and small vibrations
        effective_trans_diffusion = params_.min_trans_diffusion * 0.2f;  // 20% of normal
        effective_rot_diffusion = params_.min_rot_diffusion * 0.2f;
    }

    // Add noise proportional to motion PLUS adaptive minimum diffusion
    const float trans_noise_std = params_.alpha3 * delta_trans + params_.alpha4 * delta_rot * 100.f + effective_trans_diffusion;
    const float rot_noise_std = params_.alpha1 * delta_rot + params_.alpha2 * delta_trans * 0.001f + effective_rot_diffusion;

    // Sample noise
    const float noise_x = normal_dist_(rng_) * trans_noise_std;
    const float noise_y = normal_dist_(rng_) * trans_noise_std;
    const float noise_theta = normal_dist_(rng_) * rot_noise_std;

    // Apply odometry in robot frame, then rotate to world frame
    const float c = std::cos(pose.theta);
    const float s = std::sin(pose.theta);

    // Odometry delta (with noise) transformed to world frame
    const float odom_x_noisy = odometry.delta_x + noise_x;
    const float odom_y_noisy = odometry.delta_y + noise_y;
    const float dx_world = odom_x_noisy * c - odom_y_noisy * s;
    const float dy_world = odom_x_noisy * s + odom_y_noisy * c;

    Pose2D new_pose;
    new_pose.x = pose.x + dx_world;
    new_pose.y = pose.y + dy_world;
    new_pose.theta = Pose2D::normalizeAngle(pose.theta + odometry.delta_theta + noise_theta);

    // Sanity check - prevent NaN/Inf propagation
    if (!std::isfinite(new_pose.x) || !std::isfinite(new_pose.y) || !std::isfinite(new_pose.theta))
    {
        return pose;  // Return unchanged pose if something went wrong
    }

    return new_pose;
}

//////////////////////////////////////////////////////////////////////////////
// Correction Step
//////////////////////////////////////////////////////////////////////////////

void Localizer::correct(const std::vector<Eigen::Vector2f>& lidar_points_local)
{
    if (!map_) return;

    for (auto& particle : particles_)
    {
        double likelihood = computeLikelihood(particle.pose, lidar_points_local);

        // Clamp likelihood to valid range
        if (!std::isfinite(likelihood) || likelihood <= 0.0)
            likelihood = 1e-300;

        particle.log_weight += std::log(likelihood);
        particle.log_weight = std::max(particle.log_weight, -100.0);
    }
}

double Localizer::computeLikelihood(const Pose2D& pose,
                                     const std::vector<Eigen::Vector2f>& lidar_points_local)
{
    if (!map_ || lidar_points_local.empty())
        return 1.0;

    // Precalculate constants for likelihood model
    const float c = std::cos(pose.theta);
    const float s = std::sin(pose.theta);
    const float inv_sigma_sq_2 = 1.0f / (2.0f * params_.sigma_hit * params_.sigma_hit);
    const float p_rand = params_.z_rand / params_.max_range;
    const float z_hit_norm = params_.z_hit / (params_.sigma_hit * 2.506628274631f);

    float total_log_likelihood = 0.0f;
    int count = 0;

    // Subsample LiDAR points for efficiency
    const int step = std::max(1, params_.lidar_subsample);
    const size_t n_points = lidar_points_local.size();

    for (size_t i = 0; i < n_points; i += step)
    {
        const float lx = lidar_points_local[i].x();
        const float ly = lidar_points_local[i].y();

        // Transform to world frame
        const float p_world_x = pose.x + lx * c - ly * s;
        const float p_world_y = pose.y + lx * s + ly * c;

        // Get distance to nearest obstacle - use precomputed if available (O(1))
        float dist;
        if (map_->has_precomputed_distances())
            dist = map_->get_distance_precomputed(p_world_x, p_world_y);
        else
            dist = map_->get_distance(map_->point_to_key(p_world_x, p_world_y));

        // Likelihood: Gaussian hit + uniform random
        const float p_hit = z_hit_norm * std::exp(-dist * dist * inv_sigma_sq_2);
        const float p = p_hit + p_rand;

        total_log_likelihood += std::log(p + 1e-30f);
        count++;
    }

    if (count > 0)
    {
        float avg_log_likelihood = total_log_likelihood / static_cast<float>(count);
        return static_cast<double>(std::exp(std::max(avg_log_likelihood, -20.0f)));
    }
    return 1.0;
}

//////////////////////////////////////////////////////////////////////////////
// Resampling
//////////////////////////////////////////////////////////////////////////////

void Localizer::normalizeAndComputeStats()
{
    // FUSED OPERATION: normalize weights + compute ESS + compute mean pose
    // This combines what was 3 separate passes into 1 pass

    if (particles_.empty())
    {
        cached_ess_ = 0.0;
        cached_mean_pose_ = {};
        return;
    }

    // Pass 1: Find max log-weight for numerical stability
    double max_log_w = particles_[0].log_weight;
    for (const auto& p : particles_)
        max_log_w = std::max(max_log_w, p.log_weight);

    // Pass 2: Normalize, compute ESS, and compute weighted mean - ALL IN ONE PASS
    double sum_weights = 0.0;
    double sum_weights_sq = 0.0;
    float sum_x = 0.0f, sum_y = 0.0f;
    float sum_cos = 0.0f, sum_sin = 0.0f;

    for (auto& p : particles_)
    {
        // Normalize weight
        p.log_weight -= max_log_w;
        const float w = static_cast<float>(std::exp(p.log_weight));
        sum_weights += w;

        // Accumulate for ESS
        sum_weights_sq += w * w;

        // Accumulate for mean pose (using unnormalized weights, will divide later)
        sum_x += w * p.pose.x;
        sum_y += w * p.pose.y;
        sum_cos += w * std::cos(p.pose.theta);
        sum_sin += w * std::sin(p.pose.theta);
    }

    // Finalize normalization (divide by sum)
    const double log_sum = std::log(sum_weights);
    for (auto& p : particles_)
        p.log_weight -= log_sum;

    // Compute ESS from normalized weights
    const double normalized_sum_sq = sum_weights_sq / (sum_weights * sum_weights);
    cached_ess_ = (normalized_sum_sq > 0) ? 1.0 / normalized_sum_sq : 0.0;

    // Compute mean pose (divide by sum_weights to get weighted average)
    const float inv_sum = 1.0f / static_cast<float>(sum_weights);
    cached_mean_pose_ = {
        sum_x * inv_sum,
        sum_y * inv_sum,
        std::atan2(sum_sin, sum_cos)
    };
}

void Localizer::normalizeWeights()
{
    if (particles_.empty()) return;

    // Find max log-weight for numerical stability
    double max_log_w = particles_[0].log_weight;
    for (const auto& p : particles_)
        max_log_w = std::max(max_log_w, p.log_weight);

    // Normalize: subtract max, exp, sum, divide
    double sum = 0.0;
    for (auto& p : particles_)
    {
        p.log_weight -= max_log_w;
        sum += std::exp(p.log_weight);
    }

    const double log_sum = std::log(sum);
    for (auto& p : particles_)
        p.log_weight -= log_sum;
}

double Localizer::getEffectiveSampleSize() const
{
    // Return cached value if available (from normalizeAndComputeStats)
    return cached_ess_;
}

void Localizer::resample()
{
    // Low-variance resampling (Thrun's book, Table 4.4)
    const size_t N = particles_.size();
    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    // Build cumulative weight distribution using cached weights
    std::vector<double> cumulative(N);
    cumulative[0] = particles_[0].getWeight();
    for (size_t i = 1; i < N; ++i)
        cumulative[i] = cumulative[i-1] + particles_[i].getWeight();

    // Low-variance resampling
    const double step = 1.0 / N;
    double r = uniform_dist_(rng_) * step;
    size_t idx = 0;

    for (size_t i = 0; i < N; ++i)
    {
        while (idx < N - 1 && r > cumulative[idx])
            idx++;

        Particle p = particles_[idx];
        p.log_weight = 0.0;  // Reset weight after resampling
        new_particles.push_back(p);
        r += step;
    }

    particles_ = std::move(new_particles);
}

void Localizer::resampleKLD()
{
    // KLD-sampling: adaptive number of particles based on distribution complexity
    // See Fox, D. "KLD-Sampling: Adaptive Particle Filters"

    using Bin = std::tuple<int, int, int>;  // (x_bin, y_bin, theta_bin)
    std::unordered_set<Bin> occupied_bins;  // O(1) insertion instead of O(log N)

    const size_t N = particles_.size();
    std::vector<Particle> new_particles;
    new_particles.reserve(params_.max_particles);

    // Build cumulative weight distribution using cached weights
    std::vector<double> cumulative(N);
    cumulative[0] = particles_[0].getWeight();
    for (size_t i = 1; i < N; ++i)
        cumulative[i] = cumulative[i-1] + particles_[i].getWeight();

    // Low-variance resampling with KLD stopping criterion
    const double step = 1.0 / params_.max_particles;
    double r = uniform_dist_(rng_) * step;
    size_t idx = 0;

    while (new_particles.size() < params_.max_particles)
    {
        // Find particle to resample
        while (idx < N - 1 && r > cumulative[idx])
            idx++;

        Particle p = particles_[idx];
        p.log_weight = 0.0;
        new_particles.push_back(p);
        r += step;
        if (r > 1.0) r -= 1.0;  // Wrap around

        // Update bin occupancy
        Bin bin{
            static_cast<int>(std::floor(p.pose.x / params_.kld_bin_size_xy)),
            static_cast<int>(std::floor(p.pose.y / params_.kld_bin_size_xy)),
            static_cast<int>(std::floor(p.pose.theta / params_.kld_bin_size_theta))
        };
        occupied_bins.insert(bin);

        // Check KLD stopping criterion
        if (new_particles.size() >= params_.min_particles)
        {
            size_t required = computeKLDSampleCount(occupied_bins.size());
            if (new_particles.size() >= required)
                break;
        }
    }

    particles_ = std::move(new_particles);
}

size_t Localizer::computeKLDSampleCount(size_t num_bins) const
{
    // Wilson-Hilferty approximation (Fox, 2003)
    if (num_bins <= 1)
        return params_.min_particles;

    const double k = static_cast<double>(num_bins);
    const double eps = params_.kld_epsilon;

    // z_{1-delta} for confidence level
    // Approximation: z_0.99 ≈ 2.33, z_0.95 ≈ 1.645
    const double z = 2.33;  // For delta = 0.01

    // KLD formula
    const double term = 1.0 - 2.0 / (9.0 * (k - 1)) +
                        std::sqrt(2.0 / (9.0 * (k - 1))) * z;

    double n = (k - 1) / (2.0 * eps) * term * term * term;

    // Adaptive reduction based on ESS ratio: when ESS is very high (weights uniform),
    // we can use fewer particles. This prevents keeping many particles when converged.
    // ESS ratio close to 1.0 means particles are well-distributed and we can reduce count.
    const double ess_ratio = cached_ess_ / static_cast<double>(particles_.size());
    if (ess_ratio > 0.8)  // Very uniform weights - filter is stable
    {
        // Scale down required particles when ESS is high
        // At ess_ratio=1.0 → factor=0.5, at ess_ratio=0.8 → factor=1.0
        const double reduction_factor = 1.0 - 0.5 * (ess_ratio - 0.8) / 0.2;
        n *= reduction_factor;
    }

    return static_cast<size_t>(std::clamp(n,
                                           static_cast<double>(params_.min_particles),
                                           static_cast<double>(params_.max_particles)));
}

void Localizer::trimParticlesWhenStable()
{
    // When the filter is stable (high ESS), we can reduce particle count
    // while maintaining the pose distribution. This saves computation.

    const size_t current_size = particles_.size();
    const size_t target_size = std::max(params_.min_particles, current_size * 2 / 3);  // Reduce by ~33%

    if (target_size >= current_size)
        return;

    // Uniform subsampling since weights are already nearly uniform (high ESS)
    std::vector<Particle> new_particles;
    new_particles.reserve(target_size);

    // Use stride-based selection to maintain spatial coverage
    const double stride = static_cast<double>(current_size) / static_cast<double>(target_size);
    double idx_float = 0.0;

    while (new_particles.size() < target_size && idx_float < current_size)
    {
        const size_t idx = static_cast<size_t>(idx_float);
        if (idx < particles_.size())
        {
            Particle p = particles_[idx];
            p.log_weight = 0.0;  // Reset to uniform weights
            new_particles.push_back(p);
        }
        idx_float += stride;
    }

    particles_ = std::move(new_particles);

    qDebug() << "[Localizer] Trimmed particles from" << current_size
             << "to" << particles_.size() << "(filter stable)";
}

//////////////////////////////////////////////////////////////////////////////
// Pose Estimation
//////////////////////////////////////////////////////////////////////////////

Localizer::Pose2D Localizer::getMeanPose() const
{
    // Return cached value (computed in normalizeAndComputeStats)
    return cached_mean_pose_;
}

void Localizer::getCovariance(Eigen::Matrix2f& pos_cov, float& theta_var) const
{
    Pose2D mean = getMeanPose();

    float sum_xx = 0.f, sum_yy = 0.f, sum_xy = 0.f;
    float sum_theta_sq = 0.f;

    for (const auto& p : particles_)
    {
        const float w = static_cast<float>(p.getWeight());
        const float dx = p.pose.x - mean.x;
        const float dy = p.pose.y - mean.y;
        const float dtheta = Pose2D::normalizeAngle(p.pose.theta - mean.theta);

        sum_xx += w * dx * dx;
        sum_yy += w * dy * dy;
        sum_xy += w * dx * dy;
        sum_theta_sq += w * dtheta * dtheta;
    }

    pos_cov << sum_xx, sum_xy,
               sum_xy, sum_yy;
    theta_var = sum_theta_sq;
}

std::vector<float> Localizer::getCovarianceVector() const
{
    Pose2D mean = getMeanPose();

    // Compute full 3x3 covariance matrix weighted by particle weights
    float sum_xx = 0.f, sum_yy = 0.f, sum_tt = 0.f;
    float sum_xy = 0.f, sum_xt = 0.f, sum_yt = 0.f;
    float total_weight = 0.f;

    for (const auto& p : particles_)
    {
        const float w = static_cast<float>(p.getWeight());
        const float dx = p.pose.x - mean.x;
        const float dy = p.pose.y - mean.y;
        const float dt = Pose2D::normalizeAngle(p.pose.theta - mean.theta);

        sum_xx += w * dx * dx;
        sum_yy += w * dy * dy;
        sum_tt += w * dt * dt;
        sum_xy += w * dx * dy;
        sum_xt += w * dx * dt;
        sum_yt += w * dy * dt;
        total_weight += w;
    }

    // Normalize if weights don't sum to 1
    if (total_weight > 1e-10f)
    {
        float inv_w = 1.0f / total_weight;
        sum_xx *= inv_w;
        sum_yy *= inv_w;
        sum_tt *= inv_w;
        sum_xy *= inv_w;
        sum_xt *= inv_w;
        sum_yt *= inv_w;
    }

    // Return upper triangular: [xx, xy, xθ, yy, yθ, θθ]
    // Debug output
    static int cov_debug_count = 0;
    if (++cov_debug_count % 100 == 0)
        qDebug() << "[Localizer] Covariance: σx=" << std::sqrt(sum_xx)
                 << "mm, σy=" << std::sqrt(sum_yy) << "mm, σθ=" << std::sqrt(sum_tt) << "rad";

    return {sum_xx, sum_xy, sum_xt, sum_yy, sum_yt, sum_tt};
}

bool Localizer::isConverged() const
{
    Eigen::Matrix2f pos_cov;
    float theta_var;
    getCovariance(pos_cov, theta_var);

    // Check if standard deviations are below thresholds
    float pos_stddev = std::sqrt(pos_cov.trace());
    float theta_stddev = std::sqrt(theta_var);

    return pos_stddev < params_.position_stddev_threshold &&
           theta_stddev < params_.angle_stddev_threshold;
}

//////////////////////////////////////////////////////////////////////////////
// Odometry Utility
//////////////////////////////////////////////////////////////////////////////

Localizer::OdometryDelta Localizer::computeOdometryDelta(const Pose2D& prev, const Pose2D& curr)
{
    // Compute delta in robot frame
    float dx_world = curr.x - prev.x;
    float dy_world = curr.y - prev.y;
    float dtheta = Pose2D::normalizeAngle(curr.theta - prev.theta);

    // Rotate to robot frame (prev orientation)
    float c = std::cos(-prev.theta);
    float s = std::sin(-prev.theta);

    return {
        dx_world * c - dy_world * s,  // delta_x in robot frame
        dx_world * s + dy_world * c,  // delta_y in robot frame
        dtheta
    };
}

Localizer::OdometryDelta Localizer::addNoiseToOdometry(const OdometryDelta& odom)
{
    // Add Gaussian noise proportional to motion magnitude
    float trans = std::hypot(odom.delta_x, odom.delta_y);
    float rot = std::abs(odom.delta_theta);

    float noise_trans = normal_dist_(rng_) * (params_.alpha3 * trans + params_.alpha4 * rot);
    float noise_rot = normal_dist_(rng_) * (params_.alpha1 * rot + params_.alpha2 * trans);

    // Apply noise in the direction of motion
    float angle = std::atan2(odom.delta_y, odom.delta_x);

    return {
        odom.delta_x + noise_trans * std::cos(angle),
        odom.delta_y + noise_trans * std::sin(angle),
        odom.delta_theta + noise_rot
    };
}

//////////////////////////////////////////////////////////////////////////////
// Visualization
//////////////////////////////////////////////////////////////////////////////

void Localizer::drawParticles()
{
    if (!scene_)
    {
        qDebug() << "[Localizer::drawParticles] No scene!";
        return;
    }
    if (!params_.draw_particles)
    {
        qDebug() << "[Localizer::drawParticles] draw_particles is false";
        return;
    }
    if (particles_.empty())
    {
        qDebug() << "[Localizer::drawParticles] No particles!";
        return;
    }

    // Debug: show particle count and first particle position
    static int dbg_count = 0;
    if (++dbg_count % 50 == 0)
        qDebug() << "[Localizer::drawParticles] Drawing" << particles_.size()
                 << "particles, first at:" << particles_[0].pose.x << particles_[0].pose.y;

    // Clear all existing items to recreate with correct parameters
    for (auto* item : particle_items_)
    {
        scene_->removeItem(item);
        delete item;
    }
    particle_items_.clear();

    const float s = 150.f;  // Larger size for visibility
    static const QPen pen(QColor("black"), 2);  // Black border for visibility

    // Only draw a subset of particles if there are too many
    const size_t max_draw = 500;
    const size_t step = std::max(size_t(1), particles_.size() / max_draw);

    for (size_t i = 0; i < particles_.size(); i += step)
    {
        const auto& p = particles_[i];

        // Color based on weight (red = low, green = high)
        double w = p.getWeight();
        double w_normalized = std::clamp(w * particles_.size(), 0.0, 1.0);
        int r = static_cast<int>(255 * (1.0 - w_normalized));
        int g = static_cast<int>(255 * w_normalized);
        QBrush brush(QColor(r, g, 0, 200));

        auto* item = scene_->addEllipse(-s/2, -s/2, s, s, pen, brush);
        item->setPos(p.pose.x, p.pose.y);
        item->setZValue(60);  // Above robot (robot is 55)
        particle_items_.push_back(item);
    }
}

void Localizer::clearVisualization()
{
    if (!scene_) return;

    for (auto* item : particle_items_)
    {
        scene_->removeItem(item);
        delete item;
    }
    particle_items_.clear();
}

void Localizer::clearParticleVisualization()
{
    clearVisualization();
}

