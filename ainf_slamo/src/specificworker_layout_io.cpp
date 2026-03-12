#include "specificworker.h"

#include <QDomDocument>
#include <QFileDialog>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>

void SpecificWorker::slot_save_layout()
{
    if (room_polygon_.empty())
    {
        qWarning() << "No polygon to save - capture a room first";
        return;
    }

    // Use native dialog with explicit options to avoid freezing
    QString filename = QFileDialog::getSaveFileName(this,
        "Save Room Layout",
        "./room_layout",
        "SVG Files (*.svg)",
        nullptr,
        QFileDialog::DontUseNativeDialog);

    if (filename.isEmpty())
        return;

    if (!filename.endsWith(".svg", Qt::CaseInsensitive))
        filename += ".svg";

    save_layout_to_svg(filename.toStdString());

    // Remove the old polygon from the UI now that the new one is saved
    if (polygon_item_backup_)
    {
        viewer->scene.removeItem(polygon_item_backup_);
        delete polygon_item_backup_;
        polygon_item_backup_ = nullptr;
    }
    room_polygon_backup_.clear();
}

void SpecificWorker::slot_load_layout()
{
    QString filename = QFileDialog::getOpenFileName(this,
        "Load Room Layout",
        "./",
        "SVG Files (*.svg)",
        nullptr,
        QFileDialog::DontUseNativeDialog);

    if (filename.isEmpty())
        return;

    load_layout_from_file(filename.toStdString());
}

void SpecificWorker::slot_flip_x()
{
    if (room_polygon_.empty())
    {
        qWarning() << "Cannot flip: no room polygon defined";
        return;
    }

    // Flip all polygon vertices on X axis
    for (auto& vertex : room_polygon_)
    {
        vertex.x() = -vertex.x();
    }

    // Toggle flip state
    flip_x_applied_ = !flip_x_applied_;

    // Update the room model with flipped polygon (thread-safe)
    push_loc_command(LocCmdSetPolygon{room_polygon_});
    path_planner_.set_polygon(room_polygon_);

    // Redraw the polygon
    draw_room_polygon();

    qInfo() << "Room polygon flipped on X axis (flip_x=" << flip_x_applied_ << ")";
}

void SpecificWorker::slot_flip_y()
{
    if (room_polygon_.empty())
    {
        qWarning() << "Cannot flip: no room polygon defined";
        return;
    }

    // Flip all polygon vertices on Y axis
    for (auto& vertex : room_polygon_)
    {
        vertex.y() = -vertex.y();
    }

    // Toggle flip state
    flip_y_applied_ = !flip_y_applied_;

    // Update the room model with flipped polygon (thread-safe)
    push_loc_command(LocCmdSetPolygon{room_polygon_});
    path_planner_.set_polygon(room_polygon_);

    // Redraw the polygon
    draw_room_polygon();

    qInfo() << "Room polygon flipped on Y axis (flip_y=" << flip_y_applied_ << ")";
}

void SpecificWorker::save_layout_to_svg(const std::string& filename)
{
    if (room_polygon_.empty())
    {
        qWarning() << "No polygon to save";
        return;
    }

    // Calculate bounding box of the polygon
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();

    for (const auto& v : room_polygon_)
    {
        min_x = std::min(min_x, v.x());
        min_y = std::min(min_y, v.y());
        max_x = std::max(max_x, v.x());
        max_y = std::max(max_y, v.y());
    }

    // Add margin around the polygon (10% of size)
    const float margin_x = (max_x - min_x) * 0.1f;
    const float margin_y = (max_y - min_y) * 0.1f;
    min_x -= margin_x;
    min_y -= margin_y;
    max_x += margin_x;
    max_y += margin_y;

    const float width = max_x - min_x;
    const float height = max_y - min_y;

    // Scale factor: 100 pixels per meter for good resolution in Inkscape
    constexpr float px_per_meter = 100.0f;
    const float svg_width = width * px_per_meter;
    const float svg_height = height * px_per_meter;

    QFile file(QString::fromStdString(filename));
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qWarning() << "Failed to save SVG to" << QString::fromStdString(filename);
        return;
    }

    QTextStream out(&file);
    out.setRealNumberPrecision(4);

    // SVG header with viewBox for proper scaling
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\"\n";
    out << "     xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\"\n";
    out << "     width=\"" << svg_width << "\" height=\"" << svg_height << "\"\n";
    out << "     viewBox=\"" << min_x << " " << -max_y << " " << width << " " << height << "\">\n";
    out << "  <!-- Room layout polygon - editable in Inkscape -->\n";
    out << "  <!-- Coordinates are in meters. Scale: " << px_per_meter << " pixels/meter -->\n";
    out << "  <!-- Note: Y-axis is flipped (SVG Y increases downward) -->\n";
    out << "\n";

    // Add a grid layer for reference (optional, helps with editing)
    out << "  <g inkscape:groupmode=\"layer\" inkscape:label=\"Grid\" style=\"opacity:0.3\">\n";
    const int grid_start_x = static_cast<int>(std::floor(min_x));
    const int grid_end_x = static_cast<int>(std::ceil(max_x));
    const int grid_start_y = static_cast<int>(std::floor(min_y));
    const int grid_end_y = static_cast<int>(std::ceil(max_y));
    for (int x = grid_start_x; x <= grid_end_x; ++x)
    {
        out << "    <line x1=\"" << x << "\" y1=\"" << -max_y << "\" x2=\"" << x << "\" y2=\"" << -min_y
            << "\" stroke=\"#cccccc\" stroke-width=\"0.01\"/>\n";
    }
    for (int y = grid_start_y; y <= grid_end_y; ++y)
    {
        out << "    <line x1=\"" << min_x << "\" y1=\"" << -y << "\" x2=\"" << max_x << "\" y2=\"" << -y
            << "\" stroke=\"#cccccc\" stroke-width=\"0.01\"/>\n";
    }
    out << "  </g>\n\n";

    // Origin marker
    out << "  <g inkscape:groupmode=\"layer\" inkscape:label=\"Origin\">\n";
    out << "    <circle cx=\"0\" cy=\"0\" r=\"0.1\" fill=\"red\" opacity=\"0.7\"/>\n";
    out << "    <line x1=\"-0.3\" y1=\"0\" x2=\"0.3\" y2=\"0\" stroke=\"red\" stroke-width=\"0.02\"/>\n";
    out << "    <line x1=\"0\" y1=\"-0.3\" x2=\"0\" y2=\"0.3\" stroke=\"red\" stroke-width=\"0.02\"/>\n";
    out << "  </g>\n\n";

    // Room polygon layer
    out << "  <g inkscape:groupmode=\"layer\" inkscape:label=\"Room Polygon\">\n";
    out << "    <polygon\n";
    out << "      id=\"room_contour\"\n";
    out << "      inkscape:label=\"Room Contour\"\n";
    out << "      points=\"";

    // Write polygon points (flip Y for SVG coordinate system)
    for (size_t i = 0; i < room_polygon_.size(); ++i)
    {
        if (i > 0) out << " ";
        out << room_polygon_[i].x() << "," << -room_polygon_[i].y();
    }

    out << "\"\n";
    out << "      style=\"fill:none;stroke:#ff00ff;stroke-width:0.05;stroke-linejoin:round\"/>\n";

    // Add vertex circles for easier editing
    out << "    <!-- Vertex markers -->\n";
    for (size_t i = 0; i < room_polygon_.size(); ++i)
    {
        out << "    <circle cx=\"" << room_polygon_[i].x() << "\" cy=\"" << -room_polygon_[i].y()
            << "\" r=\"0.08\" fill=\"#ffff00\" stroke=\"#000000\" stroke-width=\"0.01\""
            << " inkscape:label=\"Vertex " << i << "\"/>\n";
    }
    out << "  </g>\n";

    out << "</svg>\n";

    file.close();
    qInfo() << "SVG layout saved to" << QString::fromStdString(filename)
            << "(" << room_polygon_.size() << " vertices)";
}

void SpecificWorker::load_layout_from_file(const std::string& filename)
{
    load_polygon_from_file(filename);

    // If polygon was loaded, initialize room_ai
    if (room_polygon_.size() >= 3)
    {
        push_loc_command(LocCmdSetPolygon{room_polygon_});
        path_planner_.set_polygon(room_polygon_);
        trajectory_controller_.set_room_boundary(room_polygon_);
        if (!furniture_polygons_.empty())
        {
            std::vector<std::vector<Eigen::Vector2f>> obs;
            for (const auto& fp : furniture_polygons_) obs.push_back(fp.vertices);
            path_planner_.set_obstacles(obs);
            trajectory_controller_.set_static_obstacles(obs);
        }
        draw_room_polygon();
        draw_furniture();
        qInfo() << "Layout loaded and room_ai initialized with" << room_polygon_.size() << "vertices,"
                << furniture_polygons_.size() << "furniture polygons";
    }
}

void SpecificWorker::save_scene_graph_to_usd()
{
    rc::SceneGraphAdapter::rebuild_graph(
        scene_graph_,
        room_polygon_,
        furniture_polygons_,
        [this](const std::string& label) { return model_height_from_label(label); },
        2.6f);

    const std::string usda = scene_graph_.to_usda();

    QFile file(PERSISTED_SCENE_GRAPH_FILE);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        qWarning() << "Cannot write scene graph to" << PERSISTED_SCENE_GRAPH_FILE;
        return;
    }
    file.write(QByteArray::fromStdString(usda));
    file.close();
    qInfo() << "Saved scene graph to" << PERSISTED_SCENE_GRAPH_FILE << "objects=" << furniture_polygons_.size();
}

bool SpecificWorker::load_scene_graph_from_usd()
{
    QFile file(PERSISTED_SCENE_GRAPH_FILE);
    if (!file.open(QIODevice::ReadOnly))
    {
        qInfo() << "No persisted scene graph file found at" << PERSISTED_SCENE_GRAPH_FILE;
        return false;
    }
    const std::string usda = file.readAll().toStdString();
    file.close();
    if (!scene_graph_.from_usda(usda))
    {
        qWarning() << "Failed parsing USD scene graph. Falling back to legacy JSON if available.";

        // Legacy compatibility path: old JSON persistence file.
        QFile legacy_file("./fitted_meshes.json");
        if (!legacy_file.open(QIODevice::ReadOnly))
            return false;
        QJsonParseError parseError;
        const auto doc = QJsonDocument::fromJson(legacy_file.readAll(), &parseError);
        legacy_file.close();
        if (parseError.error != QJsonParseError::NoError || !doc.isObject())
            return false;

        const auto root_obj = doc.object();
        if (root_obj.contains("scene_tree") && root_obj.value("scene_tree").isObject())
            scene_graph_.from_json(root_obj.value("scene_tree").toObject());
    }

    room_polygon_ = scene_graph_.room_polygon_xy();
    furniture_polygons_ = rc::SceneGraphAdapter::to_furniture(scene_graph_.objects());
    qInfo() << "Loaded scene graph from" << PERSISTED_SCENE_GRAPH_FILE << "objects=" << furniture_polygons_.size();
    return !room_polygon_.empty();
}

void SpecificWorker::load_polygon_from_file(const std::string& filename)
{
    if (!filename.empty())
        current_layout_file_ = filename;

    // USD-first: once persisted, this is authoritative world state.
    if (load_scene_graph_from_usd())
    {
        qInfo() << "Loaded persisted USD scene graph as primary layout source";
        return;
    }

    QFile file(QString::fromStdString(filename));
    if (!file.open(QIODevice::ReadOnly))
    {
        qInfo() << "No default layout file found at" << QString::fromStdString(filename);
        return;
    }
    qInfo() << "Loading layout from" << QString::fromStdString(filename);

    QByteArray data = file.readAll();
    file.close();

    // Clear previous polygon
    room_polygon_.clear();
    for (auto* item : polygon_vertex_items)
    {
        viewer->scene.removeItem(item);
        delete item;
    }
    polygon_vertex_items.clear();

    // Parse SVG file
    QString content = QString::fromUtf8(data);
    load_polygon_from_svg(content);
    // Bootstrap from SVG only when USD persistence is not present yet.
    save_scene_graph_to_usd();

    qInfo() << "Polygon loaded from" << QString::fromStdString(filename)
            << "with" << room_polygon_.size() << "vertices";
}

void SpecificWorker::load_polygon_from_svg(const QString& svg_content)
{
    // -----------------------------------------------------------------------
    // Helper: parse an SVG path 'd' attribute into a list of absolute points.
    // Returns empty vector if the path is NOT closed (no Z/z command).
    // -----------------------------------------------------------------------
    auto parsePath = [](const QString& pathData, bool requireClosed) -> std::vector<Eigen::Vector2f>
    {
        std::vector<Eigen::Vector2f> pts;
        bool isClosed = false;

        QRegularExpression cmdRegex(R"(([MmLlHhVvCcSsQqTtAaZz])\s*([-\d\.\s,eE+]*))");
        QRegularExpressionMatchIterator it = cmdRegex.globalMatch(pathData);

        float cx = 0, cy = 0, sx = 0, sy = 0;
        bool firstPoint = true;

        auto parseNums = [](const QString& s) -> std::vector<float> {
            std::vector<float> nums;
            QRegularExpression numRe(R"([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)");
            auto nit = numRe.globalMatch(s);
            while (nit.hasNext()) nums.push_back(nit.next().captured(0).toFloat());
            return nums;
        };

        while (it.hasNext())
        {
            auto m = it.next();
            QString cmd = m.captured(1);
            auto nums = parseNums(m.captured(2).trimmed());

            if (cmd == "M")
            {
                if (nums.size() >= 2) { cx = nums[0]; cy = nums[1]; sx = cx; sy = cy;
                    pts.emplace_back(cx, -cy); firstPoint = false;
                    for (size_t i = 2; i + 1 < nums.size(); i += 2)
                    { cx = nums[i]; cy = nums[i+1]; pts.emplace_back(cx, -cy); } }
            }
            else if (cmd == "m")
            {
                if (nums.size() >= 2) {
                    if (firstPoint) { cx = nums[0]; cy = nums[1]; }
                    else            { cx += nums[0]; cy += nums[1]; }
                    sx = cx; sy = cy; pts.emplace_back(cx, -cy); firstPoint = false;
                    for (size_t i = 2; i + 1 < nums.size(); i += 2)
                    { cx += nums[i]; cy += nums[i+1]; pts.emplace_back(cx, -cy); } }
            }
            else if (cmd == "L")
            { for (size_t i = 0; i + 1 < nums.size(); i += 2)
                { cx = nums[i]; cy = nums[i+1]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "l")
            { for (size_t i = 0; i + 1 < nums.size(); i += 2)
                { cx += nums[i]; cy += nums[i+1]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "H") { for (auto v : nums) { cx = v;  pts.emplace_back(cx, -cy); } }
            else if (cmd == "h") { for (auto v : nums) { cx += v; pts.emplace_back(cx, -cy); } }
            else if (cmd == "V") { for (auto v : nums) { cy = v;  pts.emplace_back(cx, -cy); } }
            else if (cmd == "v") { for (auto v : nums) { cy += v; pts.emplace_back(cx, -cy); } }
            else if (cmd == "C")
            { for (size_t i = 0; i + 5 < nums.size(); i += 6)
                { cx = nums[i+4]; cy = nums[i+5]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "c")
            { for (size_t i = 0; i + 5 < nums.size(); i += 6)
                { cx += nums[i+4]; cy += nums[i+5]; pts.emplace_back(cx, -cy); } }
            else if (cmd == "Z" || cmd == "z")
            { cx = sx; cy = sy; isClosed = true; }
        }

        if (requireClosed && !isClosed) return {};

        // Remove duplicate closing point
        if (pts.size() > 2)
            if (std::fabs(pts.front().x() - pts.back().x()) < 0.01f &&
                std::fabs(pts.front().y() - pts.back().y()) < 0.01f)
                pts.pop_back();

        return pts;
    };

    // -----------------------------------------------------------------------
    // Helper: apply SVG matrix(a,b,c,d,e,f) transform to a list of points.
    // parsePath already stores Y-flipped (-cy), so we must undo+redo the flip.
    // -----------------------------------------------------------------------
    auto applyMatrix = [](std::vector<Eigen::Vector2f>& pts, const std::array<float,6>& mat)
    {
        float a=mat[0], b=mat[1], c=mat[2], d=mat[3], e=mat[4], f=mat[5];
        for (auto& p : pts)
        {
            float px = p.x(), py = -p.y();   // undo Y-flip
            float tx = a*px + c*py + e;
            float ty = b*px + d*py + f;
            p = Eigen::Vector2f(tx, -ty);    // re-apply Y-flip
        }
    };

    // -----------------------------------------------------------------------
    // Helper: extract matrix from a transform="matrix(...)" attribute string.
    // -----------------------------------------------------------------------
    auto extractMatrix = [](const QString& attr, std::array<float,6>& mat) -> bool
    {
        QRegularExpression re(R"(matrix\(\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*,\s*([-\d\.eE+]+)\s*\))");
        auto m = re.match(attr);
        if (!m.hasMatch()) return false;
        for (int i = 0; i < 6; ++i) mat[i] = m.captured(i+1).toFloat();
        return true;
    };

    // -----------------------------------------------------------------------
    // Struct to hold a parsed path with its metadata
    // -----------------------------------------------------------------------
    struct ParsedPath {
        std::vector<Eigen::Vector2f> pts;
        QString id;
        QString label;         // inkscape:label of the path itself
        QString layer_label;   // inkscape:label of the parent <g> layer
    };
    std::vector<ParsedPath> allPaths;

    // -----------------------------------------------------------------------
    // Parse <path> elements using Qt's XML parser for robustness.
    // For paths inside a <g transform="matrix(...)"> apply the group transform.
    // -----------------------------------------------------------------------
    QDomDocument doc;
    QString parseErr;
    int errLine, errCol;
    if (!doc.setContent(svg_content, false, &parseErr, &errLine, &errCol))
    {
        qWarning() << "[SVG] XML parse error at line" << errLine << "col" << errCol << ":" << parseErr;
    }
    else
    {
        // Recursive lambda to walk the DOM tree, propagating layer label
        std::function<void(const QDomElement&, std::array<float,6>, QString)> walkElement;
        walkElement = [&](const QDomElement& elem, std::array<float,6> parentMat, QString layerLabel)
        {
            QString tag = elem.tagName();

            // Accumulate transform from this element
            std::array<float,6> currentMat = parentMat;
            if (elem.hasAttribute("transform"))
            {
                std::array<float,6> localMat = {1,0,0,1,0,0};
                if (extractMatrix(elem.attribute("transform"), localMat))
                {
                    float a1=parentMat[0],b1=parentMat[1],c1=parentMat[2],
                          d1=parentMat[3],e1=parentMat[4],f1=parentMat[5];
                    float a2=localMat[0], b2=localMat[1], c2=localMat[2],
                          d2=localMat[3], e2=localMat[4], f2=localMat[5];
                    currentMat[0] = a1*a2 + c1*b2;
                    currentMat[1] = b1*a2 + d1*b2;
                    currentMat[2] = a1*c2 + c1*d2;
                    currentMat[3] = b1*c2 + d1*d2;
                    currentMat[4] = a1*e2 + c1*f2 + e1;
                    currentMat[5] = b1*e2 + d1*f2 + f1;
                }
            }

            // Track Inkscape layer label: <g inkscape:groupmode="layer" inkscape:label="XXX">
            QString currentLayerLabel = layerLabel;
            if (tag == "g" && elem.attribute("inkscape:groupmode") == "layer")
            {
                currentLayerLabel = elem.attribute("inkscape:label");
            }

            if (tag == "path")
            {
                QString d = elem.attribute("d");
                if (!d.isEmpty())
                {
                    auto pts = parsePath(d, /*requireClosed=*/true);
                    if (pts.size() >= 3)
                    {
                        // Apply accumulated transform
                        bool isIdentity = (currentMat[0]==1 && currentMat[1]==0 && currentMat[2]==0 &&
                                           currentMat[3]==1 && currentMat[4]==0 && currentMat[5]==0);
                        if (!isIdentity) applyMatrix(pts, currentMat);

                        ParsedPath pp;
                        pp.pts = std::move(pts);
                        pp.id  = elem.attribute("id");
                        pp.label = elem.attribute("inkscape:label");
                        pp.layer_label = currentLayerLabel;
                        allPaths.push_back(std::move(pp));

                        qInfo() << "[SVG] Found closed path id=" << allPaths.back().id
                                << "label=" << allPaths.back().label
                                << "layer=" << allPaths.back().layer_label
                                << "vertices=" << allPaths.back().pts.size();
                    }
                }
            }
            else if (tag == "rect")
            {
                // Inkscape draws rectangles as <rect> elements; convert to 4-point polygon.
                const float rx = elem.attribute("x", "0").toFloat();
                const float ry = elem.attribute("y", "0").toFloat();
                const float rw = elem.attribute("width",  "0").toFloat();
                const float rh = elem.attribute("height", "0").toFloat();
                if (rw > 0.f && rh > 0.f)
                {
                    // Points stored Y-flipped to match parsePath convention (-cy)
                    std::vector<Eigen::Vector2f> pts = {
                        { rx,      -ry       },
                        { rx + rw, -ry       },
                        { rx + rw, -(ry + rh)},
                        { rx,      -(ry + rh)}
                    };
                    bool isIdentity = (currentMat[0]==1 && currentMat[1]==0 && currentMat[2]==0 &&
                                       currentMat[3]==1 && currentMat[4]==0 && currentMat[5]==0);
                    if (!isIdentity) applyMatrix(pts, currentMat);

                    ParsedPath pp;
                    pp.pts = std::move(pts);
                    pp.id  = elem.attribute("id");
                    pp.label = elem.attribute("inkscape:label");
                    if (pp.label.isEmpty()) pp.label = pp.id;
                    pp.layer_label = currentLayerLabel;
                    allPaths.push_back(std::move(pp));

                    qInfo() << "[SVG] Found rect id=" << allPaths.back().id
                            << "label=" << allPaths.back().label
                            << "layer=" << allPaths.back().layer_label;
                }
            }
            else
            {
                QDomNode child = elem.firstChild();
                while (!child.isNull())
                {
                    if (child.isElement())
                        walkElement(child.toElement(), currentMat, currentLayerLabel);
                    child = child.nextSibling();
                }
            }
        };

        std::array<float,6> identity = {1,0,0,1,0,0};
        walkElement(doc.documentElement(), identity, QString());
    }

    if (allPaths.empty())
    {
        qWarning() << "[SVG] No closed paths found.";
        return;
    }

    qInfo() << "[SVG] Total closed paths found:" << allPaths.size();

    // Classify paths by layer:
    //   - Paths in a layer whose label contains "Furniture" (case-insensitive) -> furniture obstacles
    //   - First path NOT in a "Furniture" layer -> room contour
    //   - Remaining non-furniture paths are ignored (or could be alternate room contours)
    furniture_polygons_.clear();
    bool room_found = false;

    for (const auto& pp : allPaths)
    {
        const bool is_furniture = pp.layer_label.contains("furniture", Qt::CaseInsensitive) ||
                                  pp.layer_label.contains("obstacle", Qt::CaseInsensitive);
        if (is_furniture)
        {
            rc::FurniturePolygonData fp;
            fp.id = pp.id.toStdString();
            fp.label = pp.label.isEmpty() ? pp.id.toStdString() : pp.label.toStdString();
            fp.vertices = pp.pts;
            furniture_polygons_.push_back(std::move(fp));
            qInfo() << "[SVG] Furniture:" << QString::fromStdString(furniture_polygons_.back().label)
                    << "vertices=" << furniture_polygons_.back().vertices.size();
        }
        else if (!room_found)
        {
            room_polygon_ = pp.pts;
            room_found = true;
            qInfo() << "[SVG] room_polygon_ set to path id='" << pp.id
                    << "' (layer=" << pp.layer_label << ") with" << room_polygon_.size() << "vertices";
        }
        else
        {
            qInfo() << "[SVG] Skipping extra non-furniture path id='" << pp.id
                    << "' (layer=" << pp.layer_label << ")";
        }
    }

    if (!room_found && !allPaths.empty())
    {
        // Fallback: no layer classification, use first path as room contour
        room_polygon_ = allPaths[0].pts;
        qInfo() << "[SVG] No layer classification found. Using first path as room_polygon_: id='" << allPaths[0].id
                << "' with" << room_polygon_.size() << "vertices";
    }

    qInfo() << "[SVG] Room polygon:" << room_polygon_.size() << "vertices,"
            << furniture_polygons_.size() << "furniture polygons";
}
