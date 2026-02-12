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
#include <QBrush>
#include <QPen>
#ifdef _OPENMP
#include <omp.h>
#endif

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

    // 3. Normalize weights
    normalizeWeights();

    // 4. Resample if needed (based on ESS)
    double ess = getEffectiveSampleSize();
    if (ess < params_.resample_threshold * particles_.size())
    {
        resampleKLD();
        static int resample_log_counter = 0;
        if (++resample_log_counter % 20 == 0)  // Log every ~20 resamples
        {
            qDebug() << "[Localizer] Resampled, ESS was" << ess
                     << ", new particle count:" << particles_.size();
        }
    }

    // 5. Compute and return mean pose
    last_mean_pose_ = getMeanPose();

    // 6. Update visualization (every 5 frames to save CPU)
    static int viz_counter = 0;
    if (params_.draw_particles && ++viz_counter % 5 == 0)
        drawParticles();

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

    // Add small noise proportional to motion
    const float trans_noise_std = params_.alpha3 * delta_trans + params_.alpha4 * delta_rot + 1.f;  // 1mm minimum
    const float rot_noise_std = params_.alpha1 * delta_rot + params_.alpha2 * delta_trans + 0.001f;  // ~0.06 deg minimum

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

    const size_t n_particles = particles_.size();

    // OpenMP disabled for stability
    // #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < n_particles; ++i)
    {
        double likelihood = computeLikelihood(particles_[i].pose, lidar_points_local);
        particles_[i].log_weight += std::log(likelihood + 1e-300);
        particles_[i].log_weight = std::max(particles_[i].log_weight, -100.0);
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
    const float sigma_sq_2 = 2.0f * params_.sigma_hit * params_.sigma_hit;
    const float inv_sigma_sq_2 = 1.0f / sigma_sq_2;
    const float p_rand = params_.z_rand / params_.max_range;
    const float z_hit_norm = params_.z_hit / (params_.sigma_hit * 2.506628274631f); // sqrt(2*PI) ≈ 2.5066

    double total_log_likelihood = 0.0;
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

        // Get distance to nearest obstacle
        const float dist = map_->get_distance(map_->point_to_key(p_world_x, p_world_y));

        // Likelihood: Gaussian hit + uniform random
        const float p_hit = z_hit_norm * std::exp(-dist * dist * inv_sigma_sq_2);
        const float p = p_hit + p_rand;

        total_log_likelihood += std::log(p + 1e-30f);
        count++;
    }

    if (count > 0)
    {
        double avg_log_likelihood = total_log_likelihood / count;
        return std::exp(std::max(avg_log_likelihood, -20.0));
    }
    return 1.0;
}

//////////////////////////////////////////////////////////////////////////////
// Resampling
//////////////////////////////////////////////////////////////////////////////

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
    // ESS = 1 / sum(w_i^2)  where w_i are normalized weights
    double sum_sq = 0.0;
    for (const auto& p : particles_)
    {
        double w = std::exp(p.log_weight);
        sum_sq += w * w;
    }
    return (sum_sq > 0) ? 1.0 / sum_sq : 0.0;
}

void Localizer::resample()
{
    // Low-variance resampling (Thrun's book, Table 4.4)
    const size_t N = particles_.size();
    std::vector<Particle> new_particles;
    new_particles.reserve(N);

    // Build cumulative weight distribution
    std::vector<double> cumulative(N);
    cumulative[0] = std::exp(particles_[0].log_weight);
    for (size_t i = 1; i < N; ++i)
        cumulative[i] = cumulative[i-1] + std::exp(particles_[i].log_weight);

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
    std::set<Bin> occupied_bins;

    const size_t N = particles_.size();
    std::vector<Particle> new_particles;
    new_particles.reserve(params_.max_particles);

    // Build cumulative weight distribution
    std::vector<double> cumulative(N);
    cumulative[0] = std::exp(particles_[0].log_weight);
    for (size_t i = 1; i < N; ++i)
        cumulative[i] = cumulative[i-1] + std::exp(particles_[i].log_weight);

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

    const double n = (k - 1) / (2.0 * eps) * term * term * term;

    return static_cast<size_t>(std::clamp(n,
                                           static_cast<double>(params_.min_particles),
                                           static_cast<double>(params_.max_particles)));
}

//////////////////////////////////////////////////////////////////////////////
// Pose Estimation
//////////////////////////////////////////////////////////////////////////////

Localizer::Pose2D Localizer::getMeanPose() const
{
    if (particles_.empty())
        return {};

    // Weighted mean (circular mean for angle)
    double sum_x = 0, sum_y = 0;
    double sum_cos = 0, sum_sin = 0;

    for (const auto& p : particles_)
    {
        double w = std::exp(p.log_weight);
        sum_x += w * p.pose.x;
        sum_y += w * p.pose.y;
        sum_cos += w * std::cos(p.pose.theta);
        sum_sin += w * std::sin(p.pose.theta);
    }

    return {
        static_cast<float>(sum_x),
        static_cast<float>(sum_y),
        static_cast<float>(std::atan2(sum_sin, sum_cos))
    };
}

void Localizer::getCovariance(Eigen::Matrix2f& pos_cov, float& theta_var) const
{
    Pose2D mean = getMeanPose();

    double sum_xx = 0, sum_yy = 0, sum_xy = 0;
    double sum_theta_sq = 0;

    for (const auto& p : particles_)
    {
        double w = std::exp(p.log_weight);
        double dx = p.pose.x - mean.x;
        double dy = p.pose.y - mean.y;
        double dtheta = Pose2D::normalizeAngle(p.pose.theta - mean.theta);

        sum_xx += w * dx * dx;
        sum_yy += w * dy * dy;
        sum_xy += w * dx * dy;
        sum_theta_sq += w * dtheta * dtheta;
    }

    pos_cov << sum_xx, sum_xy,
               sum_xy, sum_yy;
    theta_var = static_cast<float>(sum_theta_sq);
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
    if (!scene_) return;

    // Resize items vector if needed
    while (particle_items_.size() > particles_.size())
    {
        scene_->removeItem(particle_items_.back());
        delete particle_items_.back();
        particle_items_.pop_back();
    }

    const float s = params_.particle_size;  // Original size
    static const QPen pen(Qt::NoPen);

    // Only draw a subset of particles if there are too many
    const size_t max_draw = 500;
    const size_t step = std::max(size_t(1), particles_.size() / max_draw);

    size_t item_idx = 0;
    for (size_t i = 0; i < particles_.size(); i += step)
    {
        const auto& p = particles_[i];

        // Color based on weight (red = low, green = high)
        double w = std::exp(p.log_weight);
        double w_normalized = std::clamp(w * particles_.size(), 0.0, 1.0);
        int r = static_cast<int>(255 * (1.0 - w_normalized));
        int g = static_cast<int>(255 * w_normalized);
        QBrush brush(QColor(r, g, 0, 150));  // Original transparency

        if (item_idx < particle_items_.size())
        {
            // Update existing item
            particle_items_[item_idx]->setPos(p.pose.x, p.pose.y);
            particle_items_[item_idx]->setBrush(brush);
        }
        else
        {
            // Create new item
            auto* item = scene_->addEllipse(-s/2, -s/2, s, s, pen, brush);
            item->setPos(p.pose.x, p.pose.y);
            item->setZValue(2);  // Below obstacles, below robot
            particle_items_.push_back(item);
        }
        item_idx++;
    }

    // Remove excess items
    while (particle_items_.size() > item_idx)
    {
        scene_->removeItem(particle_items_.back());
        delete particle_items_.back();
        particle_items_.pop_back();
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
