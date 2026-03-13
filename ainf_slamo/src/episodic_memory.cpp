#include "episodic_memory.h"

#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <algorithm>
#include <chrono>
#include <random>

namespace rc
{
namespace
{
QJsonObject map_to_json(const std::unordered_map<std::string, float>& values)
{
    QJsonObject object;
    for (const auto& [key, value] : values)
        object[QString::fromStdString(key)] = value;
    return object;
}

std::unordered_map<std::string, float> json_to_map(const QJsonObject& object)
{
    std::unordered_map<std::string, float> values;
    values.reserve(static_cast<std::size_t>(object.size()));
    for (auto it = object.begin(); it != object.end(); ++it)
    {
        if (it->isDouble())
            values.emplace(it.key().toStdString(), static_cast<float>(it->toDouble()));
    }
    return values;
}

QJsonObject trajectory_to_json(const EpisodicMemory::TrajectorySummary& trajectory)
{
    return QJsonObject{
        {"n_cycles", trajectory.n_cycles},
        {"distance_traveled_m", trajectory.distance_traveled_m},
        {"path_efficiency", trajectory.path_efficiency},
        {"mean_speed", trajectory.mean_speed},
        {"p95_speed", trajectory.p95_speed},
        {"mean_rot", trajectory.mean_rot},
        {"p95_rot", trajectory.p95_rot},
        {"mean_ess_ratio", trajectory.mean_ess_ratio},
        {"p05_ess_ratio", trajectory.p05_ess_ratio},
        {"mean_cpu_pct", trajectory.mean_cpu_pct},
        {"p95_mppi_ms", trajectory.p95_mppi_ms},
    };
}

EpisodicMemory::TrajectorySummary json_to_trajectory(const QJsonObject& object)
{
    EpisodicMemory::TrajectorySummary trajectory;
    trajectory.n_cycles = object.value("n_cycles").toInt();
    trajectory.distance_traveled_m = static_cast<float>(object.value("distance_traveled_m").toDouble());
    trajectory.path_efficiency = static_cast<float>(object.value("path_efficiency").toDouble());
    trajectory.mean_speed = static_cast<float>(object.value("mean_speed").toDouble());
    trajectory.p95_speed = static_cast<float>(object.value("p95_speed").toDouble());
    trajectory.mean_rot = static_cast<float>(object.value("mean_rot").toDouble());
    trajectory.p95_rot = static_cast<float>(object.value("p95_rot").toDouble());
    trajectory.mean_ess_ratio = static_cast<float>(object.value("mean_ess_ratio").toDouble());
    trajectory.p05_ess_ratio = static_cast<float>(object.value("p05_ess_ratio").toDouble());
    trajectory.mean_cpu_pct = static_cast<float>(object.value("mean_cpu_pct").toDouble());
    trajectory.p95_mppi_ms = static_cast<float>(object.value("p95_mppi_ms").toDouble());
    return trajectory;
}

QJsonObject safety_to_json(const EpisodicMemory::SafetySummary& safety)
{
    return QJsonObject{
        {"min_esdf_m", safety.min_esdf_m},
        {"n_near_collision", safety.n_near_collision},
        {"n_collision", safety.n_collision},
        {"n_blocked_events", safety.n_blocked_events},
        {"blocked_time_s", safety.blocked_time_s},
        {"n_replans", safety.n_replans},
    };
}

EpisodicMemory::SafetySummary json_to_safety(const QJsonObject& object)
{
    EpisodicMemory::SafetySummary safety;
    safety.min_esdf_m = static_cast<float>(object.value("min_esdf_m").toDouble());
    safety.n_near_collision = object.value("n_near_collision").toInt();
    safety.n_collision = object.value("n_collision").toInt();
    safety.n_blocked_events = object.value("n_blocked_events").toInt();
    safety.blocked_time_s = static_cast<float>(object.value("blocked_time_s").toDouble());
    safety.n_replans = object.value("n_replans").toInt();
    return safety;
}

QJsonObject outcome_to_json(const EpisodicMemory::OutcomeSummary& outcome)
{
    return QJsonObject{
        {"time_to_goal_s", outcome.time_to_goal_s},
        {"success_binary", outcome.success_binary},
        {"comfort_jerk_score", outcome.comfort_jerk_score},
        {"safety_score", outcome.safety_score},
        {"efficiency_score", outcome.efficiency_score},
        {"composite_score", outcome.composite_score},
    };
}

EpisodicMemory::OutcomeSummary json_to_outcome(const QJsonObject& object)
{
    EpisodicMemory::OutcomeSummary outcome;
    outcome.time_to_goal_s = static_cast<float>(object.value("time_to_goal_s").toDouble());
    outcome.success_binary = object.value("success_binary").toInt();
    outcome.comfort_jerk_score = static_cast<float>(object.value("comfort_jerk_score").toDouble());
    outcome.safety_score = static_cast<float>(object.value("safety_score").toDouble());
    outcome.efficiency_score = static_cast<float>(object.value("efficiency_score").toDouble());
    outcome.composite_score = static_cast<float>(object.value("composite_score").toDouble());
    return outcome;
}

QJsonObject replay_to_json(const EpisodicMemory::ReplaySummary& replay)
{
    QJsonArray trials;
    for (const auto& trial : replay.counterfactual_trials)
    {
        trials.push_back(QJsonObject{
            {"param_delta_id", QString::fromStdString(trial.param_delta_id)},
            {"score", trial.score},
            {"safety_violated", trial.safety_violated},
        });
    }

    return QJsonObject{
        {"replayed", replay.replayed},
        {"replay_set_id", QString::fromStdString(replay.replay_set_id)},
        {"counterfactual_trials", trials},
        {"local_sensitivity", map_to_json(replay.local_sensitivity)},
    };
}

EpisodicMemory::ReplaySummary json_to_replay(const QJsonObject& object)
{
    EpisodicMemory::ReplaySummary replay;
    replay.replayed = object.value("replayed").toBool();
    replay.replay_set_id = object.value("replay_set_id").toString().toStdString();

    const QJsonArray trials = object.value("counterfactual_trials").toArray();
    replay.counterfactual_trials.reserve(static_cast<std::size_t>(trials.size()));
    for (const auto& trial_value : trials)
    {
        const QJsonObject trial_object = trial_value.toObject();
        EpisodicMemory::ReplayTrial trial;
        trial.param_delta_id = trial_object.value("param_delta_id").toString().toStdString();
        trial.score = static_cast<float>(trial_object.value("score").toDouble());
        trial.safety_violated = trial_object.value("safety_violated").toBool();
        replay.counterfactual_trials.push_back(std::move(trial));
    }

    replay.local_sensitivity = json_to_map(object.value("local_sensitivity").toObject());
    return replay;
}

QJsonObject mission_context_to_json(const EpisodicMemory::MissionContext& mission)
{
    return QJsonObject{
        {"mission_type", QString::fromStdString(mission.mission_type)},
        {"map_layout_id", QString::fromStdString(mission.map_layout_id)},
        {"furniture_hash", QString::fromStdString(mission.furniture_hash)},
        {"robot_context", QString::fromStdString(mission.robot_context)},
        {"controller_version", QString::fromStdString(mission.controller_version)},
    };
}

EpisodicMemory::MissionContext json_to_mission_context(const QJsonObject& object)
{
    EpisodicMemory::MissionContext mission;
    mission.mission_type = object.value("mission_type").toString().toStdString();
    mission.map_layout_id = object.value("map_layout_id").toString().toStdString();
    mission.furniture_hash = object.value("furniture_hash").toString().toStdString();
    mission.robot_context = object.value("robot_context").toString().toStdString();
    mission.controller_version = object.value("controller_version").toString().toStdString();
    return mission;
}

QJsonObject mission_target_to_json(const EpisodicMemory::MissionTarget& target)
{
    QJsonObject object{
        {"target_mode", QString::fromStdString(target.target_mode)},
        {"target_object_id", QString::fromStdString(target.target_object_id)},
    };
    if (target.target_x.has_value()) object["target_x"] = target.target_x.value();
    if (target.target_y.has_value()) object["target_y"] = target.target_y.value();
    return object;
}

EpisodicMemory::MissionTarget json_to_mission_target(const QJsonObject& object)
{
    EpisodicMemory::MissionTarget target;
    target.target_mode = object.value("target_mode").toString().toStdString();
    target.target_object_id = object.value("target_object_id").toString().toStdString();
    if (object.contains("target_x")) target.target_x = static_cast<float>(object.value("target_x").toDouble());
    if (object.contains("target_y")) target.target_y = static_cast<float>(object.value("target_y").toDouble());
    return target;
}

QJsonObject mission_source_to_json(const EpisodicMemory::MissionSource& source)
{
    QJsonObject object;
    if (source.source_x.has_value()) object["x"] = source.source_x.value();
    if (source.source_y.has_value()) object["y"] = source.source_y.value();
    if (source.obstacle_density.has_value()) object["obstacle_density"] = source.obstacle_density.value();
    return object;
}

EpisodicMemory::MissionSource json_to_mission_source(const QJsonObject& object)
{
    EpisodicMemory::MissionSource source;
    if (object.contains("x")) source.source_x = static_cast<float>(object.value("x").toDouble());
    if (object.contains("y")) source.source_y = static_cast<float>(object.value("y").toDouble());
    if (object.contains("obstacle_density")) source.obstacle_density = static_cast<float>(object.value("obstacle_density").toDouble());
    return source;
}

QJsonObject episode_to_json(const EpisodicMemory::EpisodeRecord& episode)
{
    return QJsonObject{
        {"episode_id", QString::fromStdString(episode.episode_id)},
        {"start_ts_ms", QString::number(episode.start_ts_ms)},
        {"end_ts_ms", QString::number(episode.end_ts_ms)},
        {"duration_s", episode.duration_s},
        {"status", QString::fromStdString(episode.status)},
        {"mission", mission_context_to_json(episode.mission)},
        {"source", mission_source_to_json(episode.source)},
        {"target", mission_target_to_json(episode.target)},
        {"params_snapshot", map_to_json(episode.params_snapshot)},
        {"mood_snapshot", map_to_json(episode.mood_snapshot)},
        {"trajectory", trajectory_to_json(episode.trajectory)},
        {"safety", safety_to_json(episode.safety)},
        {"outcome", outcome_to_json(episode.outcome)},
        {"replay", replay_to_json(episode.replay)},
    };
}

std::optional<EpisodicMemory::EpisodeRecord> json_to_episode(const QJsonObject& object)
{
    EpisodicMemory::EpisodeRecord episode;
    episode.episode_id = object.value("episode_id").toString().toStdString();
    if (episode.episode_id.empty())
        return std::nullopt;

    episode.start_ts_ms = object.value("start_ts_ms").toString().toLongLong();
    episode.end_ts_ms = object.value("end_ts_ms").toString().toLongLong();
    episode.duration_s = static_cast<float>(object.value("duration_s").toDouble());
    episode.status = object.value("status").toString().toStdString();

    episode.mission = json_to_mission_context(object.value("mission").toObject());
    episode.source = json_to_mission_source(object.value("source").toObject());
    episode.target = json_to_mission_target(object.value("target").toObject());

    episode.params_snapshot = json_to_map(object.value("params_snapshot").toObject());
    episode.mood_snapshot = json_to_map(object.value("mood_snapshot").toObject());

    episode.trajectory = json_to_trajectory(object.value("trajectory").toObject());
    episode.safety = json_to_safety(object.value("safety").toObject());
    episode.outcome = json_to_outcome(object.value("outcome").toObject());
    episode.replay = json_to_replay(object.value("replay").toObject());

    return episode;
}

QString make_episode_file_path(const std::string& root_dir, const std::string& episode_id)
{
    return QString::fromStdString(root_dir + "/" + episode_id + ".json");
}

} // namespace

EpisodicMemory::EpisodicMemory(std::string root_dir)
    : root_dir_(std::move(root_dir))
{
    QDir dir(QString::fromStdString(root_dir_));
    if (!dir.exists())
        dir.mkpath(".");
}

bool EpisodicMemory::save_episode(const EpisodeRecord& episode) const
{
    if (episode.episode_id.empty())
        return false;

    QDir dir(QString::fromStdString(root_dir_));
    if (!dir.exists() && !dir.mkpath("."))
        return false;

    QFile file(make_episode_file_path(root_dir_, episode.episode_id));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
        return false;

    const QJsonDocument document(episode_to_json(episode));
    file.write(document.toJson(QJsonDocument::Indented));
    file.close();
    return true;
}

std::optional<EpisodicMemory::EpisodeRecord> EpisodicMemory::load_episode(const std::string& episode_id) const
{
    QFile file(make_episode_file_path(root_dir_, episode_id));
    if (!file.exists() || !file.open(QIODevice::ReadOnly | QIODevice::Text))
        return std::nullopt;

    const QByteArray data = file.readAll();
    file.close();

    const QJsonParseError parse_error{};
    Q_UNUSED(parse_error)
    const QJsonDocument document = QJsonDocument::fromJson(data);
    if (!document.isObject())
        return std::nullopt;

    return json_to_episode(document.object());
}

std::vector<std::string> EpisodicMemory::list_episode_ids(std::size_t max_count) const
{
    QDir dir(QString::fromStdString(root_dir_));
    if (!dir.exists())
        return {};

    QStringList files = dir.entryList(QStringList() << "*.json", QDir::Files, QDir::Time);
    std::vector<std::string> ids;
    ids.reserve(static_cast<std::size_t>(files.size()));

    for (const auto& file_name : files)
    {
        const QString base_name = QFileInfo(file_name).baseName();
        ids.push_back(base_name.toStdString());
        if (max_count > 0 && ids.size() >= max_count)
            break;
    }

    return ids;
}

std::vector<EpisodicMemory::EpisodeRecord> EpisodicMemory::load_recent(std::size_t max_count) const
{
    std::vector<EpisodeRecord> episodes;
    const auto ids = list_episode_ids(max_count);
    episodes.reserve(ids.size());

    for (const auto& id : ids)
    {
        if (auto episode = load_episode(id); episode.has_value())
            episodes.push_back(std::move(episode.value()));
    }

    return episodes;
}

std::string EpisodicMemory::make_episode_id()
{
    static thread_local std::mt19937_64 rng{std::random_device{}()};
    const auto now = std::chrono::system_clock::now();
    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    const std::uint64_t random_bits = rng();

    return "ep_" + std::to_string(now_ms) + "_" + std::to_string(random_bits & 0xFFFFF);
}

} // namespace rc
