#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <Eigen/Core>
#include <vector>
#include <string>

// Forward declaration — full include not needed in header.
namespace rc { class SceneGraphModel; }

/**
 * SceneTreePanel — live, editable tree view of the scene graph model.
 *
 * Tree structure (mirrors SceneGraphModel hierarchy):
 *
 *   Room
 *   ├── Floor  (N objects)
 *   │   ├── [Table_1]  (table)
 *   │   │   ├── tx      [editable]
 *   │   │   ├── ty      [editable]
 *   │   │   ├── yaw     [editable, degrees]
 *   │   │   ├── width   [editable]
 *   │   │   ├── depth   [editable]
 *   │   │   └── height  [editable]
 *   │   └── ...
 *   └── Wall Perimeter  (N walls)
 *       ├── wall_0  — length, pos_x, pos_y, angle
 *       └── ...
 *
 * Editing a value cell emits objectPropertyEdited(), which callers should route
 * through SceneGraphModel's setter methods so all views stay consistent.
 */
class SceneTreePanel : public QWidget
{
    Q_OBJECT
public:
    explicit SceneTreePanel(QWidget* parent = nullptr);

    /**
     * Attach to a model.  From this point, the panel listens to
     * SceneGraphModel::modelRebuilt  → rebuild_from_model()
     * SceneGraphModel::objectChanged → update_object_display(label)
     * The model pointer is stored as a non-owning raw pointer.
     */
    void set_model(rc::SceneGraphModel* model);

    /// Rebuild the entire tree from the current model state.
    void rebuild_from_model();

    /**
     * Refresh a single object's value cells from the model without
     * rebuilding the whole tree.  Call after silent (no-signal) mutations.
     */
    void update_object_display(const QString& label);

    bool select_item_by_name(const QString& name);
    bool toggle_item_by_name(const QString& name);

Q_SIGNALS:
    /// Emitted when a furniture object item is clicked / toggled by the user.
    void furnitureClicked(const QString& name, bool selected);

    /**
     * Emitted when the user edits a numeric property cell of an object node.
     * @param label    Object label string.
     * @param property One of: "tx", "ty", "yaw_deg", "width", "depth", "height"
     * @param value    New value (yaw in degrees; distances in metres).
     */
    void objectPropertyEdited(const QString& label, const QString& property, float value);

private:
    QTreeWidget*         tree_      = nullptr;
    rc::SceneGraphModel* model_     = nullptr;
    bool                 updating_  = false;          // guard re-entrant itemChanged
    QTreeWidgetItem*     pressed_selected_item_ = nullptr;

    // ---- Tree-build helpers ----
    QTreeWidgetItem* make_object_item(const rc::SceneGraphModel* m,
                                      const QString& label,
                                      const QString& type_str,
                                      float tx, float ty, float yaw_rad,
                                      float width, float depth, float height) const;

    QTreeWidgetItem* make_wall_item(const QString& label,
                                    float pos_x, float pos_y,
                                    float length_m, float angle_deg) const;

    QTreeWidgetItem* find_object_item(const QString& label) const;

    // ---- Click-handling helpers ----
    static bool is_object_item(QTreeWidgetItem* item);

    static QTreeWidgetItem* make_kv(const QString& key,
                                    const QString& value  = {},
                                    bool           editable = false,
                                    const QString& owner_label = {},
                                    const QString& prop_name   = {});

    static QString fmt(float v, int decimals = 3);
};

