#include "scene_graph_model.h"

#include <QJsonArray>
#include <QJsonDocument>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace rc
{
namespace
{
QJsonArray vec3_to_json(const Eigen::Vector3f& v)
{
    return QJsonArray{static_cast<double>(v.x()), static_cast<double>(v.y()), static_cast<double>(v.z())};
}

Eigen::Vector3f json_to_vec3(const QJsonArray& a)
{
    if (a.size() < 3)
        return Eigen::Vector3f::Zero();
    return Eigen::Vector3f(static_cast<float>(a[0].toDouble()),
                           static_cast<float>(a[1].toDouble()),
                           static_cast<float>(a[2].toDouble()));
}

} // namespace

SceneGraphModel::SceneGraphModel(QObject* parent)
    : QObject(parent)
{}

bool SceneGraphModel::is_finite_positive(float v)
{
    return std::isfinite(v) && v > 0.f;
}

Eigen::Vector2f SceneGraphModel::compute_polygon_centroid(const std::vector<Eigen::Vector2f>& poly)
{
    if (poly.empty())
        return Eigen::Vector2f::Zero();
    Eigen::Vector2f c = Eigen::Vector2f::Zero();
    for (const auto& p : poly)
        c += p;
    return c / static_cast<float>(poly.size());
}

const SceneGraphModel::Node* SceneGraphModel::find_floor_const(const Node& root)
{
    for (const auto& child : root.children)
        if (child.type == "floor")
            return &child;
    return nullptr;
}

SceneGraphModel::Node* SceneGraphModel::find_floor(Node& root)
{
    for (auto& child : root.children)
        if (child.type == "floor")
            return &child;
    return nullptr;
}

SceneGraphModel::Node* SceneGraphModel::find_object(Node& floor, const std::string& id, const std::string& label)
{
    for (auto& child : floor.children)
    {
        if (child.type != "object")
            continue;
        if (!id.empty() && child.id == id)
            return &child;
        if (!label.empty() && child.label == label)
            return &child;
    }
    return nullptr;
}

SceneGraphModel::Node* SceneGraphModel::find_object_by_label(Node& floor, const std::string& label)
{
    if (label.empty()) return nullptr;
    for (auto& child : floor.children)
        if (child.type == "object" && child.label == label)
            return &child;
    return nullptr;
}

std::string SceneGraphModel::infer_object_type(const std::string& label)
{
    const QString q = QString::fromStdString(label).toLower();
    if (q.contains("table")  || q.contains("mesa"))   return "table";
    if (q.contains("chair")  || q.contains("silla"))  return "chair";
    if (q.contains("bench")  || q.contains("banco"))  return "bench";
    if (q.contains("pot")    || q.contains("maceta") || q.contains("plant")) return "pot";
    return "object";
}

std::vector<Eigen::Vector3f> SceneGraphModel::make_rect_local(float w, float d)
{
    const float hw = w * 0.5f, hd = d * 0.5f;
    return {{-hw, -hd, 0.f}, {hw, -hd, 0.f}, {hw, hd, 0.f}, {-hw, hd, 0.f}};
}

// Standard 2D rotation convention: width = local X, depth = local Y.
//   xdir_world = R(yaw) * (1,0) = ( cos yaw,  sin yaw)   local X axis in world
//   ydir_world = R(yaw) * (0,1) = (-sin yaw,  cos yaw)   local Y axis in world
// This matches object_footprints.h and viewer_3d.cpp so extents.x == width, extents.y == depth.
void SceneGraphModel::world_to_local_polygon(const std::vector<Eigen::Vector2f>& world_poly,
                                             const Eigen::Vector2f& center,
                                             float y_yaw,
                                             std::vector<Eigen::Vector3f>& local_poly,
                                             Eigen::Vector3f& extents)
{
    const Eigen::Vector2f xdir( std::cos(y_yaw),  std::sin(y_yaw));  // local X in world
    const Eigen::Vector2f ydir(-std::sin(y_yaw),  std::cos(y_yaw));  // local Y in world

    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();

    local_poly.clear();
    local_poly.reserve(world_poly.size());
    for (const auto& vw : world_poly)
    {
        const Eigen::Vector2f d = vw - center;
        const float lx = d.dot(xdir);  // width component
        const float ly = d.dot(ydir);  // depth component
        local_poly.emplace_back(lx, ly, 0.f);
        min_x = std::min(min_x, lx);
        max_x = std::max(max_x, lx);
        min_y = std::min(min_y, ly);
        max_y = std::max(max_y, ly);
    }

    extents = Eigen::Vector3f(std::max(0.08f, max_x - min_x),   // width  (X)
                              std::max(0.08f, max_y - min_y),   // depth  (Y)
                              0.8f);
}

std::vector<Eigen::Vector2f> SceneGraphModel::local_to_world_polygon(const std::vector<Eigen::Vector3f>& local_poly,
                                                                     const Eigen::Vector3f& translation,
                                                                     float y_yaw)
{
    const Eigen::Vector2f center(translation.x(), translation.y());
    const Eigen::Vector2f xdir( std::cos(y_yaw),  std::sin(y_yaw));  // local X in world
    const Eigen::Vector2f ydir(-std::sin(y_yaw),  std::cos(y_yaw));  // local Y in world

    std::vector<Eigen::Vector2f> world_poly;
    world_poly.reserve(local_poly.size());
    for (const auto& lv : local_poly)
        world_poly.emplace_back(center + xdir * lv.x() + ydir * lv.y());
    return world_poly;
}

void SceneGraphModel::clear()
{
    root_ = Node{};
    emit modelRebuilt();
}

void SceneGraphModel::rebuild(const std::vector<Eigen::Vector2f>& room_polygon,
                              const std::vector<SceneGraphObject>& objects,
                              float room_height)
{
    root_ = Node{};
    root_.id = "room";
    root_.label = "room";
    root_.type = "room";

    root_.local_vertices.reserve(room_polygon.size());
    for (const auto& p : room_polygon)
        root_.local_vertices.emplace_back(p.x(), p.y(), 0.f);

    if (room_polygon.size() >= 2)
    {
        for (std::size_t i = 0; i < room_polygon.size(); ++i)
        {
            const Eigen::Vector2f a = room_polygon[i];
            const Eigen::Vector2f b = room_polygon[(i + 1) % room_polygon.size()];
            const Eigen::Vector2f d = b - a;
            const float len = d.norm();
            if (len < 1e-4f)
                continue;

            Node wall;
            wall.id = "wall_" + std::to_string(i);
            wall.label = wall.id;
            wall.type = "wall";
            wall.translation = Eigen::Vector3f(0.5f * (a.x() + b.x()), 0.5f * (a.y() + b.y()), 0.5f * room_height);
            wall.yaw_rad = std::atan2(d.y(), d.x());
            wall.extents = Eigen::Vector3f(len, room_height, 0.05f);
            root_.children.emplace_back(std::move(wall));
        }
    }

    Node floor;
    floor.id = "floor";
    floor.label = "floor";
    floor.type = "floor";
    floor.local_vertices.reserve(room_polygon.size());
    for (const auto& p : room_polygon)
        floor.local_vertices.emplace_back(p.x(), p.y(), 0.f);

    for (const auto& obj_src : objects)
    {
        if (obj_src.vertices.size() < 3)
            continue;

        const Eigen::Vector2f c = compute_polygon_centroid(obj_src.vertices);

        // Always use the stored yaw from the furniture data (the old
        // room-centre fallback is no longer needed — every object now
        // carries an explicit yaw_rad, including the valid value 0).
        const float y_yaw = obj_src.frame_yaw_inward_rad;

        Node obj;
        obj.id = obj_src.id;
        obj.label = obj_src.label;
        obj.type = "object";
        obj.object_type = infer_object_type(obj_src.label);
        obj.translation = Eigen::Vector3f(c.x(), c.y(), 0.f);
        obj.yaw_rad = y_yaw;
        obj.last_fit_sdf = obj_src.last_fit_sdf;

        world_to_local_polygon(obj_src.vertices, c, y_yaw, obj.local_vertices, obj.extents);
        obj.extents.z() = std::max(0.2f, obj_src.height);

        // Normalize to a clean rectangle so the 2D polygon drawn from
        // local_to_world_polygon() always matches the extents+yaw used
        // by the 3D viewer and the tree panel.
        obj.local_vertices = make_rect_local(obj.extents.x(), obj.extents.y());

        floor.children.emplace_back(std::move(obj));
    }

    root_.children.emplace_back(std::move(floor));
    emit modelRebuilt();
}

bool SceneGraphModel::upsert_object(const SceneGraphObject& object)
{
    if (object.vertices.size() < 3)
        return false;

    Node* floor = find_floor(root_);
    if (floor == nullptr)
    {
        Node f;
        f.id = "floor";
        f.label = "floor";
        f.type = "floor";
        root_.children.emplace_back(std::move(f));
        floor = find_floor(root_);
        if (floor == nullptr)
            return false;
    }

    Node* obj = find_object(*floor, object.id, object.label);
    if (obj == nullptr)
    {
        Node n;
        n.id = object.id;
        n.label = object.label;
        n.type = "object";
        floor->children.emplace_back(std::move(n));
        obj = &floor->children.back();
    }

    const Eigen::Vector2f c = compute_polygon_centroid(object.vertices);
    const float y_yaw = object.frame_yaw_inward_rad;
    obj->translation = Eigen::Vector3f(c.x(), c.y(), 0.f);
    obj->yaw_rad = y_yaw;
    obj->last_fit_sdf = object.last_fit_sdf;
    obj->object_type = infer_object_type(object.label);
    world_to_local_polygon(object.vertices, c, y_yaw, obj->local_vertices, obj->extents);
    obj->extents.z() = std::max(0.2f, object.height);
    // Normalize to a clean rectangle derived from extents
    obj->local_vertices = make_rect_local(obj->extents.x(), obj->extents.y());
    emit objectChanged(QString::fromStdString(object.label));
    return true;
}

bool SceneGraphModel::remove_object(const std::string& id, const std::string& label)
{
    Node* floor = find_floor(root_);
    if (floor == nullptr)
        return false;

    const auto old_size = floor->children.size();
    floor->children.erase(std::remove_if(floor->children.begin(), floor->children.end(),
                        [&](const Node& n)
                        {
                            if (n.type != "object")
                                return false;
                            if (!id.empty() && n.id == id)
                                return true;
                            if (!label.empty() && n.label == label)
                                return true;
                            return false;
                        }),
                        floor->children.end());

    return floor->children.size() != old_size;
}

bool SceneGraphModel::try_update_object_fit(const std::string& id,
                                            const std::string& label,
                                            const std::vector<Eigen::Vector2f>& world_vertices,
                                            float new_sdf,
                                            SceneGraphObject* updated_object)
{
    if (world_vertices.size() < 3 || !is_finite_positive(new_sdf))
        return false;

    Node* floor = find_floor(root_);
    if (floor == nullptr)
    {
        Node f;
        f.id = "floor";
        f.label = "floor";
        f.type = "floor";
        root_.children.emplace_back(std::move(f));
        floor = find_floor(root_);
        if (floor == nullptr)
            return false;
    }

    Node* obj = find_object(*floor, id, label);
    if (obj == nullptr)
    {
        Node n;
        n.id = id;
        n.label = label;
        n.type = "object";
        floor->children.emplace_back(std::move(n));
        obj = &floor->children.back();
    }

    const bool improved = !obj->last_fit_sdf.has_value() || (new_sdf + 1e-4f < obj->last_fit_sdf.value());
    if (!improved)
        return false;

    const Eigen::Vector2f c = compute_polygon_centroid(world_vertices);

    Eigen::Vector2f room_center = Eigen::Vector2f::Zero();
    if (!root_.local_vertices.empty())
    {
        room_center = Eigen::Vector2f::Zero();
        for (const auto& p : root_.local_vertices)
            room_center += Eigen::Vector2f(p.x(), p.y());
        room_center /= static_cast<float>(root_.local_vertices.size());
    }

    Eigen::Vector2f ydir = room_center - c;
    if (ydir.norm() < 1e-4f)
        ydir = Eigen::Vector2f(0.f, 1.f);
    ydir.normalize();
    const float y_yaw = std::atan2(ydir.y(), ydir.x());

    obj->translation = Eigen::Vector3f(c.x(), c.y(), 0.f);
    obj->yaw_rad = y_yaw;
    obj->last_fit_sdf = new_sdf;
    world_to_local_polygon(world_vertices, c, y_yaw, obj->local_vertices, obj->extents);

    if (updated_object != nullptr)
    {
        updated_object->id = obj->id;
        updated_object->label = obj->label;
        updated_object->vertices = local_to_world_polygon(obj->local_vertices, obj->translation, obj->yaw_rad);
        updated_object->last_fit_sdf = obj->last_fit_sdf;
        updated_object->frame_yaw_inward_rad = obj->yaw_rad;
    }

    return true;
}

void SceneGraphModel::apply_to_objects(std::vector<SceneGraphObject>& objects) const
{
    const Node* floor = find_floor_const(root_);
    if (floor == nullptr)
        return;

    std::unordered_map<std::string, const Node*> by_id;
    std::unordered_map<std::string, const Node*> by_label;
    by_id.reserve(floor->children.size() * 2);
    by_label.reserve(floor->children.size() * 2);

    for (const auto& child : floor->children)
    {
        if (child.type != "object")
            continue;
        if (!child.id.empty())
            by_id[child.id] = &child;
        if (!child.label.empty())
            by_label[child.label] = &child;
    }

    for (auto& obj : objects)
    {
        const Node* node = nullptr;
        if (!obj.id.empty())
        {
            const auto it = by_id.find(obj.id);
            if (it != by_id.end())
                node = it->second;
        }
        if (node == nullptr && !obj.label.empty())
        {
            const auto it = by_label.find(obj.label);
            if (it != by_label.end())
                node = it->second;
        }
        if (node == nullptr || node->local_vertices.size() < 3)
            continue;

        obj.vertices = local_to_world_polygon(node->local_vertices, node->translation, node->yaw_rad);
        obj.last_fit_sdf = node->last_fit_sdf;
        obj.frame_yaw_inward_rad = node->yaw_rad;
    }
}

QJsonObject SceneGraphModel::to_json() const
{
    std::function<QJsonObject(const Node&)> node_to_json;
    node_to_json = [&](const Node& n) -> QJsonObject
    {
        QJsonObject obj;
        obj["id"] = QString::fromStdString(n.id);
        obj["label"] = QString::fromStdString(n.label);
        obj["type"] = QString::fromStdString(n.type);
        obj["translation"] = vec3_to_json(n.translation);
        obj["yaw_rad"] = static_cast<double>(n.yaw_rad);
        obj["extents"] = vec3_to_json(n.extents);
        if (n.last_fit_sdf.has_value())
            obj["last_fit_sdf"] = static_cast<double>(n.last_fit_sdf.value());

        // Only serialize local_vertices for non-object nodes (room/floor have arbitrary polygons);
        // object nodes recompute theirs from extents on load.
        if (n.type != "object")
        {
            QJsonArray verts;
            for (const auto& v : n.local_vertices)
                verts.append(vec3_to_json(v));
            obj["local_vertices"] = verts;
        }

        QJsonArray children;
        for (const auto& c : n.children)
            children.append(node_to_json(c));
        obj["children"] = children;
        return obj;
    };

    return node_to_json(root_);
}

bool SceneGraphModel::from_json(const QJsonObject& json)
{
    std::function<Node(const QJsonObject&)> parse_node;
    parse_node = [&](const QJsonObject& obj) -> Node
    {
        Node n;
        n.id = obj.value("id").toString().toStdString();
        n.label = obj.value("label").toString().toStdString();
        n.type = obj.value("type").toString().toStdString();
        n.translation = json_to_vec3(obj.value("translation").toArray());
        n.yaw_rad = static_cast<float>(obj.value("yaw_rad").toDouble(0.0));
        n.extents = json_to_vec3(obj.value("extents").toArray());
        if (obj.contains("last_fit_sdf"))
            n.last_fit_sdf = static_cast<float>(obj.value("last_fit_sdf").toDouble());

        const auto verts = obj.value("local_vertices").toArray();
        if (!verts.isEmpty())
        {
            n.local_vertices.reserve(verts.size());
            for (const auto& vv : verts)
                n.local_vertices.emplace_back(json_to_vec3(vv.toArray()));
        }
        // For object nodes with missing/empty local_vertices, recompute from extents
        if (n.type == "object" && n.local_vertices.empty()
            && n.extents.x() > 1e-4f && n.extents.y() > 1e-4f)
            n.local_vertices = make_rect_local(n.extents.x(), n.extents.y());

        const auto children = obj.value("children").toArray();
        n.children.reserve(children.size());
        for (const auto& cc : children)
            n.children.emplace_back(parse_node(cc.toObject()));
        return n;
    };

    if (json.isEmpty())
        return false;

    root_ = parse_node(json);
    return true;
}

std::vector<SceneGraphObject> SceneGraphModel::objects() const
{
    std::vector<SceneGraphObject> out;
    const Node* floor = find_floor_const(root_);
    if (floor == nullptr)
        return out;

    out.reserve(floor->children.size());
    for (const auto& n : floor->children)
    {
        if (n.type != "object")
            continue;
        SceneGraphObject o;
        o.id = n.id;
        o.label = n.label;
        o.vertices = local_to_world_polygon(n.local_vertices, n.translation, n.yaw_rad);
        o.last_fit_sdf = n.last_fit_sdf;
        o.frame_yaw_inward_rad = n.yaw_rad;
        o.height = std::max(0.2f, n.extents.z());
        out.emplace_back(std::move(o));
    }
    return out;
}

std::vector<Eigen::Vector2f> SceneGraphModel::room_polygon_xy() const
{
    std::vector<Eigen::Vector2f> out;
    out.reserve(root_.local_vertices.size());
    for (const auto& p : root_.local_vertices)
        out.emplace_back(p.x(), p.y());
    return out;
}

std::string SceneGraphModel::to_usda() const
{
    const auto json_obj = to_json();
    std::string json = QJsonDocument(json_obj).toJson(QJsonDocument::Indented).toStdString();

    std::ostringstream out;
    out << "#usda 1.0\n";
    out << "(\n";
    out << "    customLayerData = {\n";
    out << "        string schema = \"ainf_scene_graph_v1\"\n";
    out << "    }\n";
    out << ")\n\n";
    out << "def Xform \"Room\"\n";
    out << "{\n";
    out << "    custom string scene_graph_json = \"\"\"\n";
    out << json;
    if (json.empty() || json.back() != '\n')
        out << "\n";
    out << "\"\"\"\n";
    out << "}\n";
    return out.str();
}

bool SceneGraphModel::from_usda(const std::string& usda_text)
{
    const std::string key = "scene_graph_json";
    const std::size_t kpos = usda_text.find(key);
    if (kpos == std::string::npos)
        return false;

    const std::size_t eq = usda_text.find('=', kpos + key.size());
    if (eq == std::string::npos)
        return false;

    std::size_t p = eq + 1;
    while (p < usda_text.size() && std::isspace(static_cast<unsigned char>(usda_text[p])))
        ++p;
    if (p >= usda_text.size())
        return false;

    std::string json_payload;

    // Preferred format: multiline triple-quoted JSON block.
    if (p + 2 < usda_text.size() && usda_text[p] == '"' && usda_text[p + 1] == '"' && usda_text[p + 2] == '"')
    {
        const std::size_t b = p + 3;
        const std::size_t e = usda_text.find("\"\"\"", b);
        if (e == std::string::npos)
            return false;
        json_payload = usda_text.substr(b, e - b);
    }
    // Legacy format: escaped single-line JSON string.
    else if (usda_text[p] == '"')
    {
        std::string esc;
        bool escaped = false;
        for (std::size_t i = p + 1; i < usda_text.size(); ++i)
        {
            const char ch = usda_text[i];
            if (!escaped)
            {
                if (ch == '\\')
                {
                    escaped = true;
                    continue;
                }
                if (ch == '"')
                    break;
                esc.push_back(ch);
            }
            else
            {
                if (ch == 'n') esc.push_back('\n');
                else if (ch == 'r') esc.push_back('\r');
                else if (ch == 't') esc.push_back('\t');
                else esc.push_back(ch);
                escaped = false;
            }
        }
        json_payload = std::move(esc);
    }
    else
        return false;

    QJsonParseError parse_error;
    const auto doc = QJsonDocument::fromJson(QByteArray::fromStdString(json_payload), &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !doc.isObject())
        return false;
    return from_json(doc.object());
}

// ---------------------------------------------------------------------------
// New accessor / mutation methods
// ---------------------------------------------------------------------------

void SceneGraphModel::sync_object_silent(const std::string& label,
                                         const std::vector<Eigen::Vector2f>& world_vertices,
                                         float height,
                                         std::optional<float> yaw_override)
{
    Node* floor = find_floor(root_);
    if (!floor) return;
    Node* obj = find_object_by_label(*floor, label);
    if (!obj || world_vertices.size() < 3) return;

    const Eigen::Vector2f c   = compute_polygon_centroid(world_vertices);
    const float           yaw = yaw_override.value_or(obj->yaw_rad);

    obj->translation.x() = c.x();
    obj->translation.y() = c.y();
    obj->yaw_rad          = yaw;
    world_to_local_polygon(world_vertices, c, yaw, obj->local_vertices, obj->extents);
    obj->extents.z() = std::max(0.2f, height);
    // Normalize to a clean rectangle derived from extents
    obj->local_vertices = make_rect_local(obj->extents.x(), obj->extents.y());
    // No signal — caller already updated views.
}

bool SceneGraphModel::set_object_pose(const std::string& label,
                                      float tx, float ty, float yaw_rad_new)
{
    Node* floor = find_floor(root_);
    if (!floor) return false;
    Node* obj = find_object_by_label(*floor, label);
    if (!obj) return false;

    obj->translation.x() = tx;
    obj->translation.y() = ty;
    obj->yaw_rad          = yaw_rad_new;
    // Rebuild the footprint polygon as a W×D rectangle in local coords.
    if (obj->extents.x() > 1e-4f && obj->extents.y() > 1e-4f)
        obj->local_vertices = make_rect_local(obj->extents.x(), obj->extents.y());

    emit objectChanged(QString::fromStdString(label));
    return true;
}

bool SceneGraphModel::set_object_extents(const std::string& label,
                                          float width, float depth, float height)
{
    Node* floor = find_floor(root_);
    if (!floor) return false;
    Node* obj = find_object_by_label(*floor, label);
    if (!obj) return false;

    obj->extents = Eigen::Vector3f(std::max(0.05f, width),
                                   std::max(0.05f, depth),
                                   std::max(0.05f, height));
    // Rebuild footprint polygon to match new W×D.
    obj->local_vertices = make_rect_local(obj->extents.x(), obj->extents.y());

    emit objectChanged(QString::fromStdString(label));
    return true;
}

std::optional<SceneGraphModel::Node> SceneGraphModel::get_object_node(const std::string& label) const
{
    const Node* floor = find_floor_const(root_);
    if (!floor) return std::nullopt;
    for (const auto& c : floor->children)
        if (c.type == "object" && c.label == label)
            return c;
    return std::nullopt;
}

std::optional<Eigen::Vector3f> SceneGraphModel::get_object_translation(const std::string& label) const
{
    const Node* floor = find_floor_const(root_);
    if (!floor) return std::nullopt;
    for (const auto& c : floor->children)
        if (c.type == "object" && c.label == label)
            return c.translation;
    return std::nullopt;
}

std::optional<float> SceneGraphModel::get_object_yaw(const std::string& label) const
{
    const Node* floor = find_floor_const(root_);
    if (!floor) return std::nullopt;
    for (const auto& c : floor->children)
        if (c.type == "object" && c.label == label)
            return c.yaw_rad;
    return std::nullopt;
}

std::optional<Eigen::Vector3f> SceneGraphModel::get_object_extents(const std::string& label) const
{
    const Node* floor = find_floor_const(root_);
    if (!floor) return std::nullopt;
    for (const auto& c : floor->children)
        if (c.type == "object" && c.label == label)
            return c.extents;
    return std::nullopt;
}

std::vector<Eigen::Vector2f> SceneGraphModel::make_world_rect_polygon(
    float tx, float ty, float yaw_rad, float width, float depth)
{
    const auto local = make_rect_local(width, depth);
    const Eigen::Vector3f trans(tx, ty, 0.f);
    return local_to_world_polygon(local, trans, yaw_rad);
}

} // namespace rc
