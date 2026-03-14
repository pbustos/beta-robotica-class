#include "scene_tree_panel.h"
#include "scene_graph_model.h"

#include <QHeaderView>
#include <QFont>
#include <QVBoxLayout>
#include <QMenu>
#include <QAction>
#include <QtMath>

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
namespace
{
constexpr int kLabelRole = Qt::UserRole;
constexpr int kPropRole  = Qt::UserRole + 1;
} // namespace

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
SceneTreePanel::SceneTreePanel(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(2, 2, 2, 2);
    layout->setSpacing(0);

    tree_ = new QTreeWidget(this);
    tree_->setColumnCount(2);
    tree_->setHeaderLabels({"Property", "Value"});
    tree_->header()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    tree_->header()->setSectionResizeMode(1, QHeaderView::Stretch);
    tree_->setAlternatingRowColors(true);
    tree_->setAnimated(true);
    tree_->setIndentation(14);
    tree_->setSelectionMode(QAbstractItemView::SingleSelection);
    tree_->setFont(QFont("Monospace", 8));
    tree_->setMinimumWidth(140);
    tree_->setContextMenuPolicy(Qt::CustomContextMenu);

    // ---- Right-click context menu ----
    connect(tree_, &QTreeWidget::customContextMenuRequested,
            this, [this](const QPoint& pos)
    {
        QTreeWidgetItem* item = tree_->itemAt(pos);
        // Walk up to the object-level item (top-level children of Floor)
        while (item && !is_object_item(item))
            item = item->parent();
        if (!item)
            return;
        const QString label = item->text(0);
        QMenu menu(tree_);
        QAction* removeAct = menu.addAction(QIcon::fromTheme("edit-delete"), tr("Remove \"%1\"").arg(label));
        if (menu.exec(tree_->viewport()->mapToGlobal(pos)) == removeAct)
            emit removeObjectRequested(label);
    });

    // ---- Object-selection via click ----
    connect(tree_, &QTreeWidget::itemPressed, this, [this](QTreeWidgetItem* item, int)
    {
        if (item && is_object_item(item) && item->isSelected())
            pressed_selected_item_ = item;
        else
            pressed_selected_item_ = nullptr;
    });

    connect(tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem* item, int)
    {
        if (!item || !is_object_item(item))
        {
            pressed_selected_item_ = nullptr;
            return;
        }
        if (item == pressed_selected_item_)
        {
            tree_->clearSelection();
            tree_->setCurrentItem(nullptr);
            item->setExpanded(false);
            emit furnitureClicked(item->text(0), false);
            pressed_selected_item_ = nullptr;
            return;
        }
        emit furnitureClicked(item->text(0), true);
        pressed_selected_item_ = nullptr;
    });

    // ---- Editable property cells ----
    connect(tree_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int col)
    {
        if (updating_ || col != 1) return;

        const QString label = item->data(0, kLabelRole).toString();
        const QString prop  = item->data(1, kPropRole).toString();
        if (label.isEmpty() || prop.isEmpty()) return;

        // Strip units suffix, then parse.
        QString raw = item->text(1).trimmed();
        raw.remove(QRegularExpression(R"(\s*(m|°)\s*$)"));
        bool ok = false;
        const float value = raw.toFloat(&ok);
        if (!ok)
        {
            if (model_) update_object_display(label);
            return;
        }
        emit objectPropertyEdited(label, prop, value);
    });

    layout->addWidget(tree_);
}

// ---------------------------------------------------------------------------
// set_model
// ---------------------------------------------------------------------------
void SceneTreePanel::set_model(rc::SceneGraphModel* model)
{
    if (model_ == model) return;
    if (model_) model_->disconnect(this);
    model_ = model;
    if (model_)
    {
        connect(model_, &rc::SceneGraphModel::modelRebuilt,
                this, &SceneTreePanel::rebuild_from_model, Qt::UniqueConnection);
        connect(model_, &rc::SceneGraphModel::objectChanged,
                this, &SceneTreePanel::update_object_display, Qt::UniqueConnection);
        rebuild_from_model();
    }
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------
QString SceneTreePanel::fmt(float v, int decimals)
{
    return QString::number(static_cast<double>(v), 'f', decimals);
}

QTreeWidgetItem* SceneTreePanel::make_kv(const QString& key, const QString& value,
                                          bool editable,
                                          const QString& owner_label,
                                          const QString& prop_name)
{
    auto* it = new QTreeWidgetItem();
    it->setText(0, key);
    it->setText(1, value);
    if (editable)
    {
        it->setFlags(it->flags() | Qt::ItemIsEditable);
        it->setData(0, kLabelRole, owner_label);
        it->setData(1, kPropRole,  prop_name);
    }
    return it;
}

bool SceneTreePanel::is_object_item(QTreeWidgetItem* item)
{
    if (!item || !item->parent()) return false;
    return item->parent()->text(0) == "Floor";
}

// ---------------------------------------------------------------------------
// make_object_item
// ---------------------------------------------------------------------------
QTreeWidgetItem* SceneTreePanel::make_object_item(
    const rc::SceneGraphModel* /*m*/,
    const QString& label, const QString& type_str,
    float tx, float ty, float yaw_rad,
    float width, float depth, float height) const
{
    auto* item = new QTreeWidgetItem();
    item->setText(0, label);
    item->setText(1, type_str);
    item->setData(0, kLabelRole, label);
    QFont bold = item->font(0);
    bold.setBold(true);
    item->setFont(0, bold);

    const float yd = static_cast<float>(qRadiansToDegrees(static_cast<double>(yaw_rad)));
    item->addChild(make_kv("tx",     fmt(tx)     + " m", true, label, "tx"));
    item->addChild(make_kv("ty",     fmt(ty)     + " m", true, label, "ty"));
    item->addChild(make_kv("yaw",    fmt(yd)     + " °", true, label, "yaw_deg"));
    item->addChild(make_kv("width",  fmt(width)  + " m", true, label, "width"));
    item->addChild(make_kv("depth",  fmt(depth)  + " m", true, label, "depth"));
    item->addChild(make_kv("height", fmt(height) + " m", true, label, "height"));
    return item;
}

// ---------------------------------------------------------------------------
// make_wall_item
// ---------------------------------------------------------------------------
QTreeWidgetItem* SceneTreePanel::make_wall_item(const QString& label,
                                                 float pos_x, float pos_y,
                                                 float length_m, float angle_deg) const
{
    auto* item = new QTreeWidgetItem();
    item->setText(0, label);
    item->setText(1, fmt(length_m, 2) + " m");
    item->addChild(make_kv("pos_x",  fmt(pos_x)        + " m"));
    item->addChild(make_kv("pos_y",  fmt(pos_y)        + " m"));
    item->addChild(make_kv("length", fmt(length_m, 2)  + " m"));
    item->addChild(make_kv("angle",  fmt(angle_deg, 1) + " °"));
    return item;
}

// ---------------------------------------------------------------------------
// find_object_item
// ---------------------------------------------------------------------------
QTreeWidgetItem* SceneTreePanel::find_object_item(const QString& label) const
{
    const QString target = label.trimmed().toLower();
    for (int i = 0; i < tree_->topLevelItemCount(); ++i)
    {
        auto* room = tree_->topLevelItem(i);
        for (int j = 0; j < room->childCount(); ++j)
        {
            auto* mid = room->child(j);
            if (!mid || mid->text(0) != "Floor") continue;
            for (int k = 0; k < mid->childCount(); ++k)
            {
                auto* obj = mid->child(k);
                if (!obj) continue;
                const QString cur = obj->data(0, kLabelRole).toString().trimmed().toLower();
                if (cur == target || cur.contains(target) || target.contains(cur))
                    return obj;
            }
        }
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// rebuild_from_model
// ---------------------------------------------------------------------------
void SceneTreePanel::rebuild_from_model()
{
    if (!model_) return;

    // Preserve expansion and selection.
    QSet<QString> expanded_set;
    QString       selected_label;
    for (int i = 0; i < tree_->topLevelItemCount(); ++i)
    {
        auto* room = tree_->topLevelItem(i);
        if (!room) continue;
        for (int j = 0; j < room->childCount(); ++j)
        {
            auto* mid = room->child(j);
            if (!mid) continue;
            if (mid->isExpanded()) expanded_set.insert(mid->text(0));
            for (int k = 0; k < mid->childCount(); ++k)
            {
                auto* obj = mid->child(k);
                if (!obj) continue;
                if (obj->isExpanded()) expanded_set.insert(obj->text(0));
                if (obj->isSelected())
                    selected_label = obj->data(0, kLabelRole).toString();
            }
        }
    }

    updating_ = true;
    pressed_selected_item_ = nullptr;
    tree_->clear();

    const auto& root = model_->root();

    // ---- Room ----
    auto* room_item = new QTreeWidgetItem(tree_);
    QFont bold = room_item->font(0);
    bold.setBold(true);
    room_item->setFont(0, bold);
    room_item->setText(0, "Room");
    room_item->setExpanded(true);

    // ---- Floor ----
    const rc::SceneGraphModel::Node* floor_node = nullptr;
    for (const auto& c : root.children)
        if (c.type == "floor") { floor_node = &c; break; }

    if (floor_node)
    {
        const int obj_count = static_cast<int>(
            std::count_if(floor_node->children.begin(), floor_node->children.end(),
                          [](const rc::SceneGraphModel::Node& n){ return n.type == "object"; }));
        auto* floor_item = new QTreeWidgetItem();
        floor_item->setFont(0, bold);
        floor_item->setText(0, "Floor");
        floor_item->setText(1, QString::number(obj_count) + " objects");
        room_item->addChild(floor_item);
        floor_item->setExpanded(expanded_set.contains("Floor") || true);

        for (const auto& obj : floor_node->children)
        {
            if (obj.type != "object") continue;
            const QString qlabel = QString::fromStdString(obj.label);
            auto* oi = make_object_item(model_,
                qlabel,
                QString::fromStdString(obj.object_type),
                obj.translation.x(), obj.translation.y(), obj.yaw_rad,
                obj.extents.x(), obj.extents.y(), obj.extents.z());
            floor_item->addChild(oi);
            if (expanded_set.contains(qlabel)) oi->setExpanded(true);
        }
    }

    // ---- Wall Perimeter ----
    int wall_count = 0;
    for (const auto& c : root.children)
        if (c.type == "wall") ++wall_count;

    if (wall_count > 0)
    {
        auto* walls_item = new QTreeWidgetItem();
        walls_item->setFont(0, bold);
        walls_item->setText(0, "Wall Perimeter");
        walls_item->setText(1, QString::number(wall_count) + " walls");
        room_item->addChild(walls_item);
        if (expanded_set.contains("Wall Perimeter")) walls_item->setExpanded(true);

        for (const auto& c : root.children)
        {
            if (c.type != "wall") continue;
            const float angle_deg = static_cast<float>(
                qRadiansToDegrees(static_cast<double>(c.yaw_rad)));
            walls_item->addChild(make_wall_item(
                QString::fromStdString(c.label),
                c.translation.x(), c.translation.y(),
                c.extents.x(), angle_deg));
        }
    }

    tree_->resizeColumnToContents(0);
    updating_ = false;
    if (!selected_label.isEmpty())
        select_item_by_name(selected_label);
}

// ---------------------------------------------------------------------------
// update_object_display — in-place refresh of one object's value cells
// ---------------------------------------------------------------------------
void SceneTreePanel::update_object_display(const QString& label)
{
    if (!model_ || updating_) return;
    auto maybe_node = model_->get_object_node(label.toStdString());
    if (!maybe_node) return;
    const auto& node = *maybe_node;

    QTreeWidgetItem* oi = find_object_item(label);
    if (!oi) return;

    const float yd = static_cast<float>(
        qRadiansToDegrees(static_cast<double>(node.yaw_rad)));

    updating_ = true;
    for (int i = 0; i < oi->childCount(); ++i)
    {
        auto* child = oi->child(i);
        const QString prop = child->data(1, kPropRole).toString();
        if      (prop == "tx")      child->setText(1, fmt(node.translation.x()) + " m");
        else if (prop == "ty")      child->setText(1, fmt(node.translation.y()) + " m");
        else if (prop == "yaw_deg") child->setText(1, fmt(yd)                   + " °");
        else if (prop == "width")   child->setText(1, fmt(node.extents.x())     + " m");
        else if (prop == "depth")   child->setText(1, fmt(node.extents.y())     + " m");
        else if (prop == "height")  child->setText(1, fmt(node.extents.z())     + " m");
    }
    updating_ = false;
}

// ---------------------------------------------------------------------------

// select_item_by_name
// ---------------------------------------------------------------------------
bool SceneTreePanel::select_item_by_name(const QString& name)
{
    if (!tree_ || tree_->topLevelItemCount() == 0) return false;
    tree_->clearSelection();
    tree_->setCurrentItem(nullptr);
    QTreeWidgetItem* best = find_object_item(name);
    if (!best) return false;
    best->setSelected(true);
    tree_->setCurrentItem(best);
    if (best->parent()) best->parent()->setExpanded(true);
    best->setExpanded(true);
    tree_->scrollToItem(best, QAbstractItemView::PositionAtCenter);
    return true;
}

// ---------------------------------------------------------------------------
// toggle_item_by_name
// ---------------------------------------------------------------------------
bool SceneTreePanel::toggle_item_by_name(const QString& name)
{
    if (!tree_ || tree_->topLevelItemCount() == 0) return false;
    const QString target = name.trimmed().toLower();
    for (auto* item : tree_->selectedItems())
    {
        if (!item) continue;
        const QString cur = item->data(0, kLabelRole).toString().trimmed().toLower();
        if (cur == target || cur.contains(target) || target.contains(cur))
        {
            tree_->clearSelection();
            tree_->setCurrentItem(nullptr);
            item->setExpanded(false);
            return true;
        }
    }
    return select_item_by_name(name);
}


