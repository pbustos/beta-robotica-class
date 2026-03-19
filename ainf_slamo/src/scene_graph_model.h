#ifndef SCENE_GRAPH_MODEL_H
#define SCENE_GRAPH_MODEL_H

#include <Eigen/Eigen>
#include <QJsonObject>
#include <QObject>
#include <QString>

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

class SceneGraphModel : public QObject
{
    Q_OBJECT
public:
    struct Node
    {
        std::string id;
        std::string label;
        // type: "room" | "wall" | "floor" | "object"
        std::string type;
        // object_type (for objects): "table" | "chair" | "bench" | "pot" | "object"
        std::string object_type;
        Eigen::Vector3f translation = Eigen::Vector3f::Zero();
        float yaw_rad = 0.f;
        Eigen::Vector3f extents = Eigen::Vector3f::Zero();
        std::vector<Eigen::Vector3f> local_vertices;
        std::optional<float> last_fit_sdf;
        std::vector<Node> children;
    };

    explicit SceneGraphModel(QObject* parent = nullptr);

    // ---- Core rebuild / mutate ----
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

    // ---- Silent sync from external polygon data (no signal emitted) ----
    /// Called after translate/rotate operations where views are already updated.
    /// Keeps the model node in sync without triggering signal-driven view updates.
    void sync_object_silent(const std::string& label,
                            const std::vector<Eigen::Vector2f>& world_vertices,
                            float height,
                            std::optional<float> yaw_override = std::nullopt);

    // ---- Authoritative setters — emit objectChanged ----
    /// Teleport object to a new pose; polygon is reconstructed as a W×D rectangle.
    bool set_object_pose(const std::string& label, float tx, float ty, float yaw_rad);
    /// Set the vertical translation of an object.
    bool set_object_tz(const std::string& label, float tz);
    /// Resize object; polygon is reconstructed as a W×D rectangle.
    bool set_object_extents(const std::string& label, float width, float depth, float height);

    // ---- Accessors ----
    std::vector<SceneGraphObject> objects() const;
    std::vector<Eigen::Vector2f>  room_polygon_xy() const;
    const Node& root() const { return root_; }
    std::optional<Node>           get_object_node(const std::string& label) const;
    std::optional<Eigen::Vector3f> get_object_translation(const std::string& label) const;
    std::optional<float>           get_object_yaw(const std::string& label) const;
    std::optional<Eigen::Vector3f> get_object_extents(const std::string& label) const;

    // ---- Serialisation ----
    QJsonObject to_json() const;
    bool from_json(const QJsonObject& json);
    std::string to_usda() const;
    bool from_usda(const std::string& usda_text);

    // ---- Static helper ----
    /// Build a 4-vertex world-space rectangle from 6D params (for polygon reconstruction).
    static std::vector<Eigen::Vector2f> make_world_rect_polygon(
        float tx, float ty, float yaw_rad, float width, float depth);

Q_SIGNALS:
    /// Emitted after clear() or rebuild() — full hierarchy changed.
    void modelRebuilt();
    /// Emitted after a single object node changes (pose or extents).
    void objectChanged(const QString& label);

private:
    Node root_;

    static bool is_finite_positive(float v);
    static Eigen::Vector2f compute_polygon_centroid(const std::vector<Eigen::Vector2f>& poly);
    static Eigen::Vector2f compute_obb_center(const std::vector<Eigen::Vector2f>& poly, float yaw);
    static const Node* find_floor_const(const Node& root);
    static Node* find_floor(Node& root);
    static Node* find_object(Node& floor, const std::string& id, const std::string& label);
    static Node* find_object_by_label(Node& floor, const std::string& label);
    static void world_to_local_polygon(const std::vector<Eigen::Vector2f>& world_poly,
                                       const Eigen::Vector2f& center,
                                       float y_yaw,
                                       std::vector<Eigen::Vector3f>& local_poly,
                                       Eigen::Vector3f& extents);
    static std::vector<Eigen::Vector2f> local_to_world_polygon(const std::vector<Eigen::Vector3f>& local_poly,
                                                               const Eigen::Vector3f& translation,
                                                               float y_yaw);
    static std::string infer_object_type(const std::string& label);
    static std::vector<Eigen::Vector3f> make_rect_local(float w, float d);
};

} // namespace rc

#endif // SCENE_GRAPH_MODEL_H
