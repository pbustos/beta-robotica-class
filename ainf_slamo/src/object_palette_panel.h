#pragma once

#include <QWidget>
#include <vector>
#include <QString>

class QListWidget;
class QGroupBox;
class QDoubleSpinBox;
class QPushButton;

/**
 * @brief Lower-right panel: pick an object type from a palette, configure its
 *        initial attributes, and emit addObjectRequested to insert it into the
 *        live scene (2D, 3D and tree views).
 */
class ObjectPalettePanel : public QWidget
{
    Q_OBJECT

public:
    struct TypeDef
    {
        QString label;    ///< display name and type-prefix used for labelling
        float   width_m;  ///< default footprint width (metres)
        float   depth_m;  ///< default footprint depth (metres)
        float   height_m; ///< default object height (metres)
    };

    explicit ObjectPalettePanel(QWidget* parent = nullptr);

Q_SIGNALS:
    /**
     * Emitted when the user clicks "Add to Scene".
     * @param type_label  Type string (e.g. "chair")
     * @param tx, ty      World-frame position (metres)
     * @param yaw_deg     Orientation (degrees)
     * @param width_m     Footprint width (metres)
     * @param depth_m     Footprint depth (metres)
     * @param height_m    Object height (metres)
     */
    void addObjectRequested(const QString& type_label,
                            float tx, float ty, float yaw_deg,
                            float width_m, float depth_m, float height_m);

private slots:
    void on_type_selected(int row);
    void on_add_clicked();

private:
    static const std::vector<TypeDef>& type_defs();

    QListWidget*    type_list_  = nullptr;
    QGroupBox*      attrs_box_  = nullptr;
    QDoubleSpinBox* spin_x_     = nullptr;
    QDoubleSpinBox* spin_y_     = nullptr;
    QDoubleSpinBox* spin_yaw_   = nullptr;
    QDoubleSpinBox* spin_w_     = nullptr;
    QDoubleSpinBox* spin_d_     = nullptr;
    QDoubleSpinBox* spin_h_     = nullptr;
    QPushButton*    add_btn_    = nullptr;
};
