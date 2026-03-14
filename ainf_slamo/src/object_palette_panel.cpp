#include "object_palette_panel.h"
#include "object_geometry.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

// ---------------------------------------------------------------------------
// Static type catalogue
// ---------------------------------------------------------------------------
const std::vector<ObjectPalettePanel::TypeDef>& ObjectPalettePanel::type_defs()
{
    static const std::vector<TypeDef> defs = {
        { "chair", rc::geometry::nominal_chair.width, rc::geometry::nominal_chair.depth, rc::geometry::nominal_chair.height },
        { "table", rc::geometry::nominal_table.width, rc::geometry::nominal_table.depth, rc::geometry::nominal_table.height },
        { "bench", rc::geometry::nominal_bench.width, rc::geometry::nominal_bench.depth, rc::geometry::nominal_bench.height },
        { "pot",   rc::geometry::nominal_pot.width,   rc::geometry::nominal_pot.depth,   rc::geometry::nominal_pot.height },
    };
    return defs;
}

// ---------------------------------------------------------------------------
ObjectPalettePanel::ObjectPalettePanel(QWidget* parent)
    : QWidget(parent)
{
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(6, 4, 6, 4);
    root->setSpacing(4);

    // Title
    auto* title = new QLabel("Add Object", this);
    title->setStyleSheet("font-weight: bold; font-size: 9pt;");
    root->addWidget(title);

    // Object-type list
    type_list_ = new QListWidget(this);
    type_list_->setAlternatingRowColors(true);
    type_list_->setFixedHeight(96);
    for (const auto& td : type_defs())
        type_list_->addItem(td.label);
    root->addWidget(type_list_);

    // Attribute form (hidden until a type is picked)
    attrs_box_ = new QGroupBox("Attributes", this);
    attrs_box_->setVisible(false);
    auto* form = new QFormLayout(attrs_box_);
    form->setContentsMargins(4, 4, 4, 4);
    form->setSpacing(3);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    auto make_spin = [&](double lo, double hi, double step, double val,
                         const QString& suffix) -> QDoubleSpinBox*
    {
        auto* s = new QDoubleSpinBox(attrs_box_);
        s->setRange(lo, hi);
        s->setSingleStep(step);
        s->setValue(val);
        s->setSuffix(suffix);
        s->setDecimals(2);
        return s;
    };

    spin_x_   = make_spin(-20.0, 20.0,  0.10, 0.0, " m");
    spin_y_   = make_spin(-20.0, 20.0,  0.10, 0.0, " m");
    spin_yaw_ = make_spin(-180.0, 180.0, 5.0, 0.0, " °");
    spin_w_   = make_spin(  0.1,  5.0,  0.05, 0.5, " m");
    spin_d_   = make_spin(  0.1,  5.0,  0.05, 0.5, " m");
    spin_h_   = make_spin(  0.1,  3.0,  0.05, 0.8, " m");

    form->addRow("X:",      spin_x_);
    form->addRow("Y:",      spin_y_);
    form->addRow("Yaw:",    spin_yaw_);
    form->addRow("Width:",  spin_w_);
    form->addRow("Depth:",  spin_d_);
    form->addRow("Height:", spin_h_);

    root->addWidget(attrs_box_);

    // Add button
    add_btn_ = new QPushButton("Add to Scene", this);
    add_btn_->setEnabled(false);
    add_btn_->setStyleSheet("font-weight: bold;");
    root->addWidget(add_btn_);

    root->addStretch();

    // Connections
    connect(type_list_, &QListWidget::currentRowChanged,
            this, &ObjectPalettePanel::on_type_selected);
    connect(add_btn_, &QPushButton::clicked,
            this, &ObjectPalettePanel::on_add_clicked);
}

// ---------------------------------------------------------------------------
void ObjectPalettePanel::on_type_selected(int row)
{
    const auto& defs = type_defs();
    if (row < 0 || row >= static_cast<int>(defs.size()))
    {
        attrs_box_->setVisible(false);
        add_btn_->setEnabled(false);
        return;
    }

    const auto& td = defs[static_cast<std::size_t>(row)];
    spin_w_->setValue(static_cast<double>(td.width_m));
    spin_d_->setValue(static_cast<double>(td.depth_m));
    spin_h_->setValue(static_cast<double>(td.height_m));

    attrs_box_->setVisible(true);
    add_btn_->setEnabled(true);
}

// ---------------------------------------------------------------------------
void ObjectPalettePanel::on_add_clicked()
{
    const int row = type_list_->currentRow();
    const auto& defs = type_defs();
    if (row < 0 || row >= static_cast<int>(defs.size()))
        return;

    Q_EMIT addObjectRequested(
        defs[static_cast<std::size_t>(row)].label,
        static_cast<float>(spin_x_->value()),
        static_cast<float>(spin_y_->value()),
        static_cast<float>(spin_yaw_->value()),
        static_cast<float>(spin_w_->value()),
        static_cast<float>(spin_d_->value()),
        static_cast<float>(spin_h_->value()));
}
