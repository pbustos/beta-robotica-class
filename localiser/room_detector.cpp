//
// Created by pbustos on 2/12/22.
//

#include "room_detector.h"
#include <cppitertools/enumerate.hpp>
#include <cppitertools/combinations.hpp>
#include <opencv2/imgproc.hpp>
#include <cppitertools/zip.hpp>
#include <chrono>
#include "specificworker.h"

namespace rc
{
    // par lines and rooms are commented. Lines have to be at least of half-room size
    std::tuple<Corners, Lines> Room_Detector::compute_corners(const std::vector<Eigen::Vector2d> &line, QGraphicsScene *scene)
    {
        const std::vector<Eigen::Vector2d> floor_line_cart = line;

        // compute lines
        auto  lines = RansacLineDetector::detect_lines(floor_line_cart);
        if (scene != nullptr) draw_lines_on_2D_tab(lines, scene);

        //lines = filter_lines_by_length(lines, static_cast<float>(estimated_size.head(2).minCoeff()/2.f));

        // compute corners
        Corners corners = get_corners(lines);

        // select the set of corners that complies with a rectangle and is minimal
        corners = select_minimal_rectangle(corners);

        if (scene != nullptr) draw_corners_on_2D_tab(corners, {Eigen::Vector2d{0,0}}, scene);
        current_walls = lines;  // update cached walls
        return {corners, lines};  // TODO return only lines that form corners
    }
    std::tuple<Corners, Lines> Room_Detector::compute_corners(const std::vector<Eigen::Vector3d> &line, QGraphicsScene *scene)
    {
        std::vector<Eigen::Vector2d> line2d;
        std::ranges::transform(line, std::back_inserter(line2d), [](const auto &p){return p.head(2);});
        return compute_corners(line2d, scene);
    }
    std::tuple<Corners, Lines> Room_Detector::compute_corners(const RoboCompLidar3D::TPoints &points,  QGraphicsScene *scene)
    {
        std::vector<Eigen::Vector2d> line2d;
        std::ranges::transform(points, std::back_inserter(line2d), [](const auto &p){return Eigen::Vector2d{p.x, p.y};});
        return compute_corners(line2d,  scene);
    }

     ////////////////////////////////////////////////
    Eigen::Vector3d Room_Detector::estimate_room_sizes(const Eigen::Vector2d &room_center, std::vector<Eigen::Vector2d> &floor_line_cart)
    {
        Eigen::MatrixX2d zero_mean_points(floor_line_cart.size(), 2);
        for(const auto &[i, p] : iter::enumerate(floor_line_cart))
            zero_mean_points.row(i) = p - room_center;

        Eigen::Matrix2d cov = (zero_mean_points.adjoint() * zero_mean_points) / double(zero_mean_points.rows() - 1);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver;
        eigensolver.compute(cov);
        Eigen::Vector2d values = eigensolver.eigenvalues().real().cwiseSqrt()*2.f;
        Eigen::Index i;
        values.maxCoeff(&i);
        Eigen::Vector2d max_vector = eigensolver.eigenvectors().real().col(i);
        return Eigen::Vector3d(values.x(), values.y(), atan2(max_vector.x(), max_vector.y()));
    }

    Corners Room_Detector::get_corners(Lines &lines)
    {
        Corners corners;
        for(auto &&comb: iter::combinations(lines, 2))
        {
            const auto& line1 = comb[0];
            const auto& line2 = comb[1];
            double angle = fabs(qDegreesToRadians(line1.toQLineF().angleTo(line2.toQLineF())));
            angle = fmod(angle + M_PI, 2 * M_PI);
            if (angle < 0) angle += 2 * M_PI;
            angle -= M_PI;
            constexpr double delta = 0.2;
            QPointF intersection;
            const bool angle_condition = (angle < M_PI/2+delta and angle > M_PI/2-delta) or (angle < -M_PI/2+delta and angle > -M_PI/2-delta );
            long now = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            if(angle_condition and line1.toQLineF().intersects(line2.toQLineF(), &intersection) == QLineF::UnboundedIntersection)
                corners.emplace_back(intersection, 0.0, now );
        }

        // NM suppression
        Corners filtered_corners;
        for (const auto &corner : corners)
        {
            const auto &p1 = std::get<0>(corner); // QPointF
            const bool too_close = std::ranges::any_of(filtered_corners, [&](const Corner &c2){
                const auto &p2 = std::get<0>(c2);
                return euc_distance_between_points(p1, p2) < 400.0;
            });
            if (not too_close)
                filtered_corners.push_back(corner);
        }
        return filtered_corners;
    }

     ////////// DRAW  /////////////////////////////////////////////////////////////
    Lines Room_Detector::filter_lines_by_length(const Lines &lines, float threshold )
    {
        Lines filtered_lines;
        std::ranges::copy_if(lines, std::back_inserter(filtered_lines),
                     [threshold](const LineSegment &line) {
                         const double length = (line.end - line.start).norm();
                         return length >= threshold;});
        return filtered_lines;
    }
    void Room_Detector::draw_lines_on_2D_tab(const Lines &lines, QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> lines_vec;
        for (const auto l: lines_vec)
        {
            scene->removeItem(l);
            delete l;
        }
        lines_vec.clear();

        const QPen pen(QColor("orange"), 20);
        for(const auto &l : lines)
        {
            auto ql = l.toQLineF();
            const auto p = scene->addLine(ql, pen);
            lines_vec.push_back(p);
        }
    }
    void Room_Detector::draw_corners_on_2D_tab(const Corners &corners, const std::vector<Eigen::Vector2d> &model_corners,
                                               QGraphicsScene *scene)
    {
        static std::vector<QGraphicsItem*> items;
        for (const auto &i: items)
        {
            scene->removeItem(i);
            delete i;
        }
        items.clear();

        const QColor color("darkMagenta");
        for(const auto &[p, _, __] : corners)
        {
            const auto i = scene->addEllipse(-100, -100, 200, 200, QPen(color), QBrush(color));
            i->setPos(p.x(), p.y());
            items.push_back(i);
        }
        //model corners
        QColor ccolor("cyan");
        for(const auto &[m, c] : iter::zip(model_corners, corners))
        {
            auto p = scene->addEllipse(-100, -100, 200, 200, QPen(ccolor), QBrush(ccolor));
            p->setPos(m.x(), m.y());
            items.push_back(p);
        }
    }

    ////////// AUX ///////////////////////////////////////////////////////////////////////////////////////
    QPointF Room_Detector::get_most_distant_point(const QPointF &p, const QPointF &p1, const QPointF &p2)
    {
        if( (p-p1).manhattanLength() < (p-p2).manhattanLength()) return p2; else return p1;
    }
    Eigen::Vector2d Room_Detector::to_eigen(const QPointF &p)
    {
        return Eigen::Vector2d{p.x(), p.y()};
    }
    Eigen::Vector2d Room_Detector::to_eigen(const cv::Point2d  &p)
    {
        return Eigen::Vector2d{p.x, p.y};
    }
    QPointF Room_Detector::to_qpointf(const cv::Point2d  &p)
    {
        return QPointF{p.x, p.y};
    }
    double Room_Detector::euc_distance_between_points(const QPointF &p1, const QPointF &p2)
    {
        return sqrt((p1.x()-p2.x())*(p1.x()-p2.x())+(p1.y()-p2.y())*(p1.y()-p2.y()));
    }

    Eigen::Vector3d Room_Detector::normalizeLineABC_(const Eigen::Vector3d &abc)
    {
        double a = abc.x();
        double b = abc.y();
        double c = abc.z();

        const double n = std::hypot(a, b);
        if (n < 1e-9)
            return abc; // degenerate; leave as-is to be filtered by grouping

        // Force a consistent orientation of the normal to allow c-based sorting
        const double s = (a > 0.0 || (a == 0.0 && b > 0.0)) ? 1.0 : -1.0;
        return Eigen::Vector3d(s * a / n, s * b / n, s * c / n);
    }

    std::optional<std::pair<Eigen::Vector3d, Eigen::Vector3d>>

    Room_Detector::pickOuterParallelPairByC_(std::vector<Eigen::Vector3d> &grp)
    {
        if (grp.size() < 2)
            return std::nullopt;

        for (auto &L : grp)
            L = normalizeLineABC_(L);

        std::sort(grp.begin(), grp.end(),
                  [](const Eigen::Vector3d &u, const Eigen::Vector3d &v) { return u.z() < v.z(); });

        return std::make_pair(grp.front(), grp.back());
    }

    Eigen::Vector3d Room_Detector::midline_(const Eigen::Vector3d &L1, const Eigen::Vector3d &L2)
    {
        // L1 = (a,b,c1), L2 = (a,b,c2) assumed parallel and normalized w/ consistent sign
        return Eigen::Vector3d(L1.x(), L1.y(), 0.5 * (L1.z() + L2.z()));
    }

    Eigen::Vector2d Room_Detector::intersect_(const Eigen::Vector3d &L1, const Eigen::Vector3d &L2)
    {
        // Solve:
        // a1 x + b1 y + c1 = 0
        // a2 x + b2 y + c2 = 0
        Eigen::Matrix2d A;
        A << L1.x(), L1.y(),
             L2.x(), L2.y();

        Eigen::Vector2d c(-L1.z(), -L2.z());
        return A.colPivHouseholderQr().solve(c);
    }

    std::optional<Eigen::Vector2d>

    Room_Detector::estimate_center_from_walls(const Lines &lines) const
    {
        if (lines.size() < 3)
            return std::nullopt;

        // 1) Normalize lines and cluster into two orthogonal groups (~0° vs ~90°)
        std::vector<Eigen::Vector3d> grp0;
        std::vector<Eigen::Vector3d> grp90;
        grp0.reserve(lines.size());
        grp90.reserve(lines.size());

        for (const auto &Lraw : lines)
        {
            const Eigen::Vector3d L = normalizeLineABC_(Lraw.to_general_form());
            const double a = L.x(), b = L.y();
            const double ang = std::fabs(std::fmod(std::fabs(std::atan2(b, a)), M_PI));

            // Assign to whichever cluster is closer: 0 vs pi/2
            if (std::fabs(ang - 0.0) <= std::fabs(ang - M_PI / 2.0))
                grp0.push_back(L);
            else
                grp90.push_back(L);
        }

        if (grp0.empty() || grp90.empty())
            return std::nullopt;

        // 2) In each group, pick the two outermost parallel lines (by c)
        auto pair0  = pickOuterParallelPairByC_(grp0);
        auto pair90 = pickOuterParallelPairByC_(grp90);
        if (!pair0 || !pair90)
            return std::nullopt;

        // 3) Compute midlines and intersect them
        const Eigen::Vector3d mid0  = midline_(pair0->first,  pair0->second);
        const Eigen::Vector3d mid90 = midline_(pair90->first, pair90->second);

        const double det = std::fabs(mid0.x() * mid90.y() - mid0.y() * mid90.x());
        if (det < 1e-6)
            return std::nullopt; // nearly parallel; unexpected in rectangles

        const Eigen::Vector2d center = intersect_(mid0, mid90);
        return center;
    }
    std::optional<Eigen::Vector2d> Room_Detector::estimate_center_from_walls() const
    {
        if (current_walls.empty())
            return {};
        return estimate_center_from_walls(current_walls);
    }

/* c++ */
Corners Room_Detector::select_minimal_rectangle(const Corners &corners)
{
    if (corners.size() < 4)
        return {};

    const double angle_tol = 0.2;         // radians tolerance around 90deg (≈11.5°)
    const double side_ratio_tol = 0.2;    // allowed relative difference between opposite sides (20%)
    Corners best_rect;
    double best_area = std::numeric_limits<double>::infinity();

    auto point_from_corner = [](const Corner &c) -> QPointF { return std::get<0>(c); };

    auto area_polygon = [](const std::vector<QPointF> &pts) {
        double a = 0.0;
        for (size_t i = 0, n = pts.size(); i < n; ++i)
        {
            const auto &p1 = pts[i];
            const auto &p2 = pts[(i+1) % n];
            a += (p1.x() * p2.y() - p2.x() * p1.y());
        }
        return std::abs(a) * 0.5;
    };

    auto length = [](const QPointF &a, const QPointF &b) {
        const double dx = a.x() - b.x();
        const double dy = a.y() - b.y();
        return std::hypot(dx, dy);
    };

    auto dot = [](const QPointF &u, const QPointF &v) {
        return u.x()*v.x() + u.y()*v.y();
    };

    auto sub = [](const QPointF &a, const QPointF &b) {
        return QPointF{a.x()-b.x(), a.y()-b.y()};
    };

    // point-in-polygon: returns true if p is inside poly or on its boundary
    auto point_in_poly = [](const std::vector<QPointF> &poly, const QPointF &p) {
        bool inside = false;
        const double eps = 1e-9;
        if (poly.empty()) return false;
        for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
        {
            const double xi = poly[i].x(), yi = poly[i].y();
            const double xj = poly[j].x(), yj = poly[j].y();

            // Check if point is exactly on the segment [j,i]
            const double minx = std::min(xi, xj), maxx = std::max(xi, xj);
            const double miny = std::min(yi, yj), maxy = std::max(yi, yj);
            const double cross = (p.x() - xi) * (yj - yi) - (p.y() - yi) * (xj - xi);
            if (std::abs(cross) < eps && p.x() + eps >= minx && p.x() - eps <= maxx && p.y() + eps >= miny && p.y() - eps <= maxy)
                return true; // on boundary -> treat as inside

            const bool intersect = ((yi > p.y()) != (yj > p.y())) &&
                                   (p.x() < (xj - xi) * (p.y() - yi) / (yj - yi) + xi);
            if (intersect) inside = !inside;
        }
        return inside;
    };

    const QPointF origin(0.0, 0.0);

    for (auto &&comb : iter::combinations(corners, 4))
    {
        // Extract original Corner objects into a vector to preserve associated data
        std::vector<Corner> rect_corners{ comb[0], comb[1], comb[2], comb[3] };

        // Compute centroid
        QPointF centroid(0,0);
        for (const auto &c : rect_corners)
            centroid += point_from_corner(c);
        centroid /= 4.0;

        // Order CCW by angle around centroid
        std::sort(rect_corners.begin(), rect_corners.end(),
                  [&](const Corner &a, const Corner &b){
                      const QPointF pa = point_from_corner(a);
                      const QPointF pb = point_from_corner(b);
                      double anga = std::atan2(pa.y()-centroid.y(), pa.x()-centroid.x());
                      double angb = std::atan2(pb.y()-centroid.y(), pb.x()-centroid.x());
                      return anga < angb;
                  });

        // Build points in order
        std::vector<QPointF> pts;
        pts.reserve(4);
        for (const auto &c : rect_corners) pts.push_back(point_from_corner(c));

        // Quick convexity check: signed area must be positive (CCW) and no crossing (4 points assumed)
        double poly_area = area_polygon(pts);
        if (poly_area <= 1e-6) continue;

        // Require that the rectangle contains the origin
        if (!point_in_poly(pts, origin)) continue;

        // Edge vectors and lengths
        std::array<QPointF,4> edges;
        std::array<double,4> lens;
        for (int i = 0; i < 4; ++i)
        {
            const QPointF &p0 = pts[i];
            const QPointF &p1 = pts[(i+1)%4];
            edges[i] = sub(p1, p0);
            lens[i] = length(p0, p1);
            if (lens[i] < 1e-6) { poly_area = -1; break; } // degenerate
        }
        if (poly_area < 0) continue;

        // Check right angles (each adjacent pair should be ~90deg)
        bool angles_ok = true;
        for (int i = 0; i < 4; ++i)
        {
            const QPointF &v1 = edges[i];
            const QPointF &v2 = edges[(i+1)%4];
            const double denom = std::max(1e-9, length(QPointF{0,0}, v1) * length(QPointF{0,0}, v2));
            double cosang = dot(v1, v2) / denom;
            if (cosang > 1.0) cosang = 1.0;
            if (cosang < -1.0) cosang = -1.0;
            double ang = std::acos(cosang);
            if (std::abs(ang - M_PI/2.0) > angle_tol) { angles_ok = false; break; }
        }
        if (!angles_ok) continue;

        // Check opposite sides similar length
        const double r0 = std::abs(lens[0] - lens[2]) / std::max(1e-9, std::max(lens[0], lens[2]));
        const double r1 = std::abs(lens[1] - lens[3]) / std::max(1e-9, std::max(lens[1], lens[3]));
        if (r0 > side_ratio_tol || r1 > side_ratio_tol) continue;

        // Accept candidate: choose minimal area
        if (poly_area < best_area)
        {
            best_area = poly_area;
            best_rect.clear();
            for (const auto &c : rect_corners) best_rect.push_back(c);
        }
    }

     return best_rect;
    }

} // rc
