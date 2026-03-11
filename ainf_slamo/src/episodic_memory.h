#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace rc
{
    class EpisodicMemory
    {
        public:
            struct MissionTarget
            {
                std::string target_mode = "none";
                std::optional<float> target_x;
                std::optional<float> target_y;
                std::string target_object_id;
            };

            struct MissionContext
            {
                std::string mission_type = "unknown";
                std::string map_layout_id;
                std::string furniture_hash;
                std::string robot_context;
                std::string controller_version;
            };

            struct TrajectorySummary
            {
                int n_cycles = 0;
                float distance_traveled_m = 0.f;
                float path_efficiency = 0.f;
                float mean_speed = 0.f;
                float p95_speed = 0.f;
                float mean_rot = 0.f;
                float p95_rot = 0.f;
                float mean_ess_ratio = 0.f;
                float p05_ess_ratio = 0.f;
                float mean_cpu_pct = 0.f;
                float p95_mppi_ms = 0.f;
            };

            struct SafetySummary
            {
                float min_esdf_m = 0.f;
                int n_near_collision = 0;
                int n_collision = 0;
                int n_blocked_events = 0;
                float blocked_time_s = 0.f;
                int n_replans = 0;
            };

            struct OutcomeSummary
            {
                float time_to_goal_s = 0.f;
                int success_binary = 0;
                float comfort_jerk_score = 0.f;
                float safety_score = 0.f;
                float efficiency_score = 0.f;
                float composite_score = 0.f;
            };

            struct ReplayTrial
            {
                std::string param_delta_id;
                float score = 0.f;
                bool safety_violated = false;
            };

            struct ReplaySummary
            {
                bool replayed = false;
                std::string replay_set_id;
                std::vector<ReplayTrial> counterfactual_trials;
                std::unordered_map<std::string, float> local_sensitivity;
            };

            struct EpisodeRecord
            {
                std::string episode_id;
                std::int64_t start_ts_ms = 0;
                std::int64_t end_ts_ms = 0;
                float duration_s = 0.f;
                std::string status = "unknown";

                MissionContext mission;
                MissionTarget target;

                std::unordered_map<std::string, float> params_snapshot;
                std::unordered_map<std::string, float> mood_snapshot;

                TrajectorySummary trajectory;
                SafetySummary safety;
                OutcomeSummary outcome;
                ReplaySummary replay;
            };

            explicit EpisodicMemory(std::string root_dir = "./episodic_memory");

            bool save_episode(const EpisodeRecord& episode) const;
            std::optional<EpisodeRecord> load_episode(const std::string& episode_id) const;
            std::vector<std::string> list_episode_ids(std::size_t max_count = 0) const;
            std::vector<EpisodeRecord> load_recent(std::size_t max_count) const;

            const std::string& root_dir() const { return root_dir_; }
            static std::string make_episode_id();

        private:
            std::string root_dir_;
    };

} // namespace rc
