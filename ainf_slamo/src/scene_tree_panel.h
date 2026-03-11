#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <Eigen/Core>
#include <vector>
#include <string>

/**
 * SceneTreePanel – foldable tree view of the loaded room layout.
 *
 * Tree structure:
 *   Layout  (N elements)
 *   ├─ Room
 *   │   ├─ Bbox X / Y / Size
 *   │   └─ Vertices [...]
 *   ├─ <furniture label>
 *   │   ├─ Label / ID / Centroid / Size / Bbox X / Bbox Y
 *   │   └─ Vertices [...]
 *   └─ ...
 */
class SceneTreePanel : public QWidget
{
    Q_OBJECT
public:
    struct FurnitureEntry
    {
        std::string id;
        std::string label;
        std::vector<Eigen::Vector2f> vertices;
    };

    explicit SceneTreePanel(QWidget* parent = nullptr);

    /**
     * Rebuild the tree from the current room polygon and furniture list.
     * Safe to call any time data changes.
     */
    void refresh(const std::vector<Eigen::Vector2f>& room_polygon,
                 const std::vector<FurnitureEntry>&  furniture);

    /// Select and reveal the furniture item node with the given name/label.
    /// Returns true if an item was found and selected.
    bool select_item_by_name(const QString& name);

    /// Toggle selection of an item by name: if already selected, unselect it.
    /// Returns true if an item was selected or unselected.
    bool toggle_item_by_name(const QString& name);

Q_SIGNALS:
    /// Emitted when a furniture top-level item is clicked/toggled by user.
    void furnitureClicked(const QString& name, bool selected);

private:
    QTreeWidget* tree_ = nullptr;
    QTreeWidgetItem* pressed_selected_item_ = nullptr;

    static QTreeWidgetItem* make_kv(const QString& key, const QString& value = {});
    static QString          fmt(float v, int decimals = 2);

    static void add_bbox_items(QTreeWidgetItem* parent,
                               const std::vector<Eigen::Vector2f>& pts);
    static void add_vertex_items(QTreeWidgetItem* parent,
                                 const std::vector<Eigen::Vector2f>& pts);
    static bool is_furniture_top_item(QTreeWidgetItem* item);
};
