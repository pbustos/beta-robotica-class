#ifndef SCENE_GRAPH_MODEL_H
#define SCENE_GRAPH_MODEL_H

#include <Eigen/Eigen>
#include <QJsonObject>

#include <optional>
#include <string>
#include <vector>

namespace rc
{
struct SceneGraphObject
{
    std::string id;
    std::string label;
    std::vector<Eigen::Vector2f> vertices;
    std::optional<float> last_fit_sdf;
    float frame_yaw_inward_rad = 0.f;
    float height = 0.8f;
};

class SceneGraphModel
{
public:
    struct Node
    {
        std::string id;
        std::string label;
        std::string type; // room | wall | floor | object
        Eigen::Vector3f translation = Eigen::Vector3f::Zero();
        float yaw_rad = 0.f;
        Eigen::Vector3f extents = Eigen::Vector3f::Zero();
        std::vector<Eigen::Vector3f> local_vertices;
        std::optional<float> last_fit_sdf;
        std::vector<Node> children;
    };

    void clear();
    void rebuild(const std::vector<Eigen::Vector2f>& room_polygon,
                 const std::vector<SceneGraphObject>& objects,
                 float room_height = 2.6f);

    bool upsert_object(const SceneGraphObject& object);
    bool remove_object(const std::string& id, const std::string& label);

    bool try_update_object_fit(const std::string& id,
                               const std::string& label,
                               const std::vector<Eigen::Vector2f>& world_vertices,
                               float new_sdf,
                               SceneGraphObject* updated_object = nullptr);

    void apply_to_objects(std::vector<SceneGraphObject>& objects) const;

    std::vector<SceneGraphObject> objects() const;
    std::vector<Eigen::Vector2f> room_polygon_xy() const;

    QJsonObject to_json() const;
    bool from_json(const QJsonObject& json);
    std::string to_usda() const;
    bool from_usda(const std::string& usda_text);

    const Node& root() const { return root_; }

private:
    Node root_;

    static bool is_finite_positive(float v);
    static Eigen::Vector2f compute_polygon_centroid(const std::vector<Eigen::Vector2f>& poly);
    static const Node* find_floor_const(const Node& root);
    static Node* find_floor(Node& root);
    static Node* find_object(Node& floor, const std::string& id, const std::string& label);
    static void world_to_local_polygon(const std::vector<Eigen::Vector2f>& world_poly,
                                       const Eigen::Vector2f& center,
                                       float y_yaw,
                                       std::vector<Eigen::Vector3f>& local_poly,
                                       Eigen::Vector3f& extents);
    static std::vector<Eigen::Vector2f> local_to_world_polygon(const std::vector<Eigen::Vector3f>& local_poly,
                                                               const Eigen::Vector3f& translation,
                                                               float y_yaw);
};

} // namespace rc

#endif // SCENE_GRAPH_MODEL_H
