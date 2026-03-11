#include "scene_tree_panel.h"

#include <QHeaderView>
#include <QFont>
#include <QVBoxLayout>
#include <limits>
#include <cmath>

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
    tree_->setMinimumWidth(120);

    connect(tree_, &QTreeWidget::itemPressed, this, [this](QTreeWidgetItem* item, int)
    {
        if (item != nullptr && is_furniture_top_item(item) && item->isSelected())
            pressed_selected_item_ = item;
        else
            pressed_selected_item_ = nullptr;
    });

    connect(tree_, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem* item, int)
    {
        if (item == nullptr || !is_furniture_top_item(item))
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

    layout->addWidget(tree_);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
QTreeWidgetItem* SceneTreePanel::make_kv(const QString& key, const QString& value)
{
    auto* it = new QTreeWidgetItem();
    it->setText(0, key);
    it->setText(1, value);
    return it;
}

QString SceneTreePanel::fmt(float v, int decimals)
{
    return QString::number(static_cast<double>(v), 'f', decimals);
}

void SceneTreePanel::add_bbox_items(QTreeWidgetItem* parent,
                                    const std::vector<Eigen::Vector2f>& pts)
{
    if (pts.empty()) return;
    float min_x =  std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y =  std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    for (const auto& v : pts)
    {
        min_x = std::min(min_x, v.x()); max_x = std::max(max_x, v.x());
        min_y = std::min(min_y, v.y()); max_y = std::max(max_y, v.y());
    }
    parent->addChild(make_kv("Bbox X",
        fmt(min_x) + " → " + fmt(max_x) + " m"));
    parent->addChild(make_kv("Bbox Y",
        fmt(min_y) + " → " + fmt(max_y) + " m"));
    parent->addChild(make_kv("Size",
        fmt(max_x - min_x) + " × " + fmt(max_y - min_y) + " m"));
}

void SceneTreePanel::add_vertex_items(QTreeWidgetItem* parent,
                                      const std::vector<Eigen::Vector2f>& pts)
{
    auto* vnode = make_kv("Vertices", QString::number(pts.size()));
    for (std::size_t i = 0; i < pts.size(); ++i)
    {
        vnode->addChild(make_kv(
            QString("[%1]").arg(i),
            "(" + fmt(pts[i].x()) + ", " + fmt(pts[i].y()) + ")"));
    }
    parent->addChild(vnode);
}

bool SceneTreePanel::is_furniture_top_item(QTreeWidgetItem* item)
{
    if (item == nullptr || item->parent() == nullptr)
        return false;
    if (item->parent()->text(0) != "Layout")
        return false;

    const QString n = item->text(0).trimmed().toLower();
    return n != "room";
}

// ---------------------------------------------------------------------------
// refresh — rebuilds tree from current data
// ---------------------------------------------------------------------------
void SceneTreePanel::refresh(const std::vector<Eigen::Vector2f>& room_polygon,
                              const std::vector<FurnitureEntry>&  furniture)
{
    pressed_selected_item_ = nullptr;

    // Remember which top-level items were expanded so we can restore them
    QSet<QString> expanded_labels;
    for (int i = 0; i < tree_->topLevelItemCount(); ++i)
    {
        auto* top = tree_->topLevelItem(i);
        for (int j = 0; j < top->childCount(); ++j)
        {
            auto* child = top->child(j);
            if (child->isExpanded())
                expanded_labels.insert(child->text(0));
        }
    }

    tree_->clear();

    // ---- Root: Layout -------------------------------------------------------
    auto* root = new QTreeWidgetItem(tree_);
    root->setText(0, "Layout");
    root->setText(1, QString("%1 element(s)").arg(furniture.size()));
    QFont bold_font = root->font(0);
    bold_font.setBold(true);
    root->setFont(0, bold_font);
    root->setExpanded(true);

    // ---- Room polygon node --------------------------------------------------
    {
        auto* room_node = make_kv("Room",
            QString("%1 vertices").arg(room_polygon.size()));
        room_node->setFont(0, bold_font);
        add_bbox_items(room_node, room_polygon);
        add_vertex_items(room_node, room_polygon);
        if (expanded_labels.contains("Room"))
            room_node->setExpanded(true);
        root->addChild(room_node);
    }

    // ---- Furniture nodes ----------------------------------------------------
    for (const auto& fi : furniture)
    {
        const QString name = fi.label.empty()
            ? QString::fromStdString(fi.id)
            : QString::fromStdString(fi.label);

        auto* node = make_kv(name,
            QString("%1 vertices").arg(fi.vertices.size()));
        node->setFont(0, bold_font);

        node->addChild(make_kv("Label", QString::fromStdString(fi.label)));
        node->addChild(make_kv("ID",    QString::fromStdString(fi.id)));

        // Centroid
        if (!fi.vertices.empty())
        {
            Eigen::Vector2f cen = Eigen::Vector2f::Zero();
            for (const auto& v : fi.vertices) cen += v;
            cen /= static_cast<float>(fi.vertices.size());
            node->addChild(make_kv("Centroid",
                "(" + fmt(cen.x()) + ", " + fmt(cen.y()) + ") m"));
        }

        add_bbox_items(node, fi.vertices);
        add_vertex_items(node, fi.vertices);

        if (expanded_labels.contains(name))
            node->setExpanded(true);

        root->addChild(node);
    }

    tree_->resizeColumnToContents(0);
}

bool SceneTreePanel::select_item_by_name(const QString& name)
{
    if (!tree_ || tree_->topLevelItemCount() == 0)
        return false;

    tree_->clearSelection();
    tree_->setCurrentItem(nullptr);

    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    if (target.isEmpty())
        return false;

    QTreeWidgetItem* best = nullptr;
    QTreeWidgetItem* contains_match = nullptr;

    for (int i = 0; i < tree_->topLevelItemCount(); ++i)
    {
        auto* root = tree_->topLevelItem(i);
        for (int j = 0; j < root->childCount(); ++j)
        {
            auto* child = root->child(j);
            const QString cur = normalize(child->text(0));
            if (cur == target)
            {
                best = child;
                break;
            }
            if (cur.contains(target) || target.contains(cur))
                contains_match = child;
        }
        if (best) break;
    }

    if (!best)
        best = contains_match;
    if (!best)
        return false;

    best->setSelected(true);
    tree_->setCurrentItem(best);
    if (best->parent())
        best->parent()->setExpanded(true);
    best->setExpanded(true);
    tree_->scrollToItem(best, QAbstractItemView::PositionAtCenter);
    return true;
}

bool SceneTreePanel::toggle_item_by_name(const QString& name)
{
    if (!tree_ || tree_->topLevelItemCount() == 0)
        return false;

    auto normalize = [](QString s)
    {
        s = s.trimmed();
        const int idx = s.indexOf(" (");
        if (idx > 0)
            s = s.left(idx);
        return s.toLower();
    };

    const QString target = normalize(name);
    if (target.isEmpty())
        return false;

    const auto selected = tree_->selectedItems();
    for (auto* item : selected)
    {
        if (item != nullptr && normalize(item->text(0)) == target)
        {
            tree_->clearSelection();
            tree_->setCurrentItem(nullptr);
            item->setExpanded(false);
            return true;
        }
    }

    return select_item_by_name(name);
}
