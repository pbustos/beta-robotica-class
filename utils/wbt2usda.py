import re
import math
import sys
import os
import json
import argparse
import importlib
import matplotlib.pyplot as plt

def axis_angle_to_quaternion(ax, ay, az, angle):
    mag = math.sqrt(ax**2 + ay**2 + az**2)
    if mag < 1e-6: return (1, 0, 0, 0)
    ax /= mag; ay /= mag; az /= mag
    s = math.sin(angle / 2)
    return (math.cos(angle / 2), ax * s, ay * s, az * s)

def parse_wbt(file_path):
    if not os.path.exists(file_path):
        sys.exit(f"❌ No se encontró el archivo: {file_path}")

    with open(file_path, 'r') as f:
        content = f.read()

    # Patrón para capturar nodos PROTO
    object_pattern = re.compile(r'(\w+)\s+\{(.*?)\}', re.DOTALL)
    blocks = object_pattern.findall(content)
    
    results = []
    # Diccionario de etiquetas permitidas (PROTO o Nombre)
    # Incluye variaciones de Webots: WoodenChair, Table, PottedPlant, Bench...
    allowed_keys = ['wall', 'muro', 'mesa', 'table', 'silla', 'chair', 'maceta', 'plant', 'banco', 'bench', 'potted', 'plant', 'solid', 'Pecera']

    print(f"--- Iniciando lectura de {file_path} ---")
    for proto_name, block in blocks:
        name_match = re.search(r'name\s+"(.*?)"', block)
        name = name_match.group(1) if name_match else proto_name
        
        # Filtro por etiqueta
        is_wall = 'wall' in proto_name.lower() or 'wall' in name.lower()
        is_furniture = any(k in proto_name.lower() or k in name.lower() for k in allowed_keys)

        if is_wall or is_furniture:
            t_match = re.search(r'translation\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)', block)
            if t_match:
                trans = [float(x) for x in t_match.groups()]
                
                # Ignorar ruido de coordenadas gigantes
                if any(abs(c) > 500 for c in trans): continue

                s_match = re.search(r'size\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)', block)
                # Defaults si el mueble no tiene size definido en el bloque
                if s_match:
                    size = [float(x) for x in s_match.groups()]
                else:
                    size = [0.8, 0.8, 0.8] if is_furniture else [0.2, 2.0, 2.0]

                r_match = re.search(r'rotation\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)\s+([-0-9.eE+]+)', block)
                quat = axis_angle_to_quaternion(*[float(x) for x in r_match.groups()]) if r_match else (1, 0, 0, 0)
                
                results.append({
                    'proto': proto_name,
                    'name': name,
                    'trans': trans,
                    'size': size,
                    'quat': quat,
                    'is_wall': is_wall
                })

    print(f"✅ Se encontraron {len(results)} objetos válidos.")
    return results

def filter_by_layout(data):
    walls = [d for d in data if d['is_wall']]
    if not walls:
        return data, False # No hay muros, no filtramos por área

    # Calculamos el área de los muros para el Bounding Box
    all_x = [w['trans'][0] for w in walls]
    all_y = [w['trans'][1] for w in walls]
    min_x, max_x = min(all_x), max(all_x)
    min_y, max_y = min(all_y), max(all_y)

    filtered = []
    margin = 2.0 # Margen de 2 metros alrededor de los muros
    for d in data:
        tx, ty = d['trans'][0], d['trans'][1]
        if min_x - margin <= tx <= max_x + margin and min_y - margin <= ty <= max_y + margin:
            filtered.append(d)
    
    # Simple connected check (distancia entre muros < 8m)
    is_connected = True
    for i, w1 in enumerate(walls):
        dists = [math.hypot(w1['trans'][0]-w2['trans'][0], w1['trans'][1]-w2['trans'][1]) 
                 for j, w2 in enumerate(walls) if i != j]
        if dists and min(dists) > 8.0:
            is_connected = False
            break

    return filtered, is_connected

def detect_wall_components(data, touch_tol=0.05):
    walls = [d for d in data if d['is_wall']]
    if not walls:
        return []

    try:
        shapely_geometry = importlib.import_module('shapely.geometry')
        Polygon = shapely_geometry.Polygon

        polygons = []
        for w in walls:
            corners = wall_corners_2d(w)[:-1]
            polygons.append(Polygon(corners).buffer(touch_tol))

        adjacency = [[] for _ in walls]
        for i in range(len(walls)):
            for j in range(i + 1, len(walls)):
                if polygons[i].intersects(polygons[j]):
                    adjacency[i].append(j)
                    adjacency[j].append(i)
    except Exception:
        # Fallback geométrico sin shapely: adyacencia por proximidad de centros.
        adjacency = [[] for _ in walls]
        for i in range(len(walls)):
            for j in range(i + 1, len(walls)):
                wi, wj = walls[i], walls[j]
                d = math.hypot(wi['trans'][0] - wj['trans'][0], wi['trans'][1] - wj['trans'][1])
                ri = 0.5 * math.hypot(wi['size'][0], wi['size'][1])
                rj = 0.5 * math.hypot(wj['size'][0], wj['size'][1])
                if d <= (ri + rj + touch_tol):
                    adjacency[i].append(j)
                    adjacency[j].append(i)

    visited = [False] * len(walls)
    components = []
    for i in range(len(walls)):
        if visited[i]:
            continue
        stack = [i]
        visited[i] = True
        comp = []
        while stack:
            cur = stack.pop()
            comp.append(walls[cur])
            for nxt in adjacency[cur]:
                if not visited[nxt]:
                    visited[nxt] = True
                    stack.append(nxt)
        components.append(comp)

    return components

def component_points(component):
    pts = []
    for w in component:
        pts.extend(wall_corners_2d(w)[:-1])
    return pts

def closest_points_between_components(comp_a, comp_b):
    pts_a = component_points(comp_a)
    pts_b = component_points(comp_b)
    if not pts_a or not pts_b:
        return None

    best = None
    for pa in pts_a:
        for pb in pts_b:
            d = math.hypot(pa[0] - pb[0], pa[1] - pb[1])
            if best is None or d < best['distance']:
                best = {'a': pa, 'b': pb, 'distance': d}
    return best

def disconnection_links(components):
    if not components or len(components) <= 1:
        return []

    # Construimos un arbol de enlaces minimos entre componentes para mostrar
    # los puntos de desconexion principales (N-1 enlaces para N componentes).
    remaining = set(range(1, len(components)))
    connected = {0}
    links = []

    while remaining:
        best = None
        best_pair = None
        for ci in connected:
            for cj in remaining:
                link = closest_points_between_components(components[ci], components[cj])
                if link is None:
                    continue
                if best is None or link['distance'] < best['distance']:
                    best = link
                    best_pair = (ci, cj)

        if best is None or best_pair is None:
            break

        best['comp_from'] = best_pair[0]
        best['comp_to'] = best_pair[1]
        links.append(best)
        connected.add(best_pair[1])
        remaining.remove(best_pair[1])

    return links

def save_usda(data, filename):
    with open(filename, 'w') as f:
        f.write("#usda 1.0\n( defaultPrim = \"World\"; metersPerUnit = 1; upAxis = \"Z\" )\ndef Scope \"World\"\n{\n")
        for i, d in enumerate(data):
            safe_name = re.sub(r'\W+', '_', d['name']) + f"_{i}"
            t, q, s = d['trans'], d['quat'], d['size']
            f.write(f'    def Cube "{safe_name}"\n    {{\n')
            f.write(f'        double3 xformOp:translate = ({t[0]}, {t[1]}, {t[2]})\n')
            f.write(f'        quatd xformOp:orient = ({q[0]}, {q[1]}, {q[2]}, {q[3]})\n')
            f.write(f'        double3 xformOp:scale = ({s[0]/2}, {s[1]/2}, {s[2]/2})\n')
            f.write(f'        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]\n    }}\n')
        f.write("}\n")
    print(f"💾 Archivo USDA guardado: {filename}")

def wall_corners_2d(obj):
    tx, ty, sx, sy = obj['trans'][0], obj['trans'][1], obj['size'][0], obj['size'][1]
    w, _, _, z = obj['quat']
    angle = 2 * math.atan2(z, w)

    hw, hl = sx / 2, sy / 2
    local_corners = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl)]

    corners = []
    for cx, cy in local_corners:
        rx = cx * math.cos(angle) - cy * math.sin(angle) + tx
        ry = cx * math.sin(angle) + cy * math.cos(angle) + ty
        corners.append([rx, ry])

    # Cerramos el polígono repitiendo el primer punto
    corners.append(corners[0])
    return corners

def contour_from_wall_union(walls, base_connect_tol_m=0.05):
    try:
        shapely_geometry = importlib.import_module('shapely.geometry')
        shapely_ops = importlib.import_module('shapely.ops')
        Polygon = shapely_geometry.Polygon
        unary_union = shapely_ops.unary_union
    except Exception as exc:
        raise RuntimeError(
            "Para exportar .json con contorno detallado se requiere el paquete 'shapely'."
        ) from exc

    polygons = []
    for w in walls:
        corners = wall_corners_2d(w)[:-1]
        if len(corners) >= 3:
            polygons.append(Polygon(corners))

    if not polygons:
        return []

    merged_raw = unary_union(polygons)

    # Cierra micro-grietas numéricas entre segmentos de muro. Se usa una
    # tolerancia progresiva para evitar falsos desconectados por ruido.
    tolerances = [
        base_connect_tol_m,
        base_connect_tol_m * 2.0,
        base_connect_tol_m * 4.0,
        base_connect_tol_m * 8.0,
    ]

    candidate = None
    for tol in tolerances:
        merged = merged_raw.buffer(tol).buffer(-tol)
        if merged.is_empty:
            continue
        if merged.geom_type == 'Polygon':
            candidate = merged
            break

    if candidate is None:
        # Sin buffer, también puede ser ya una sola geometría válida.
        if not merged_raw.is_empty and merged_raw.geom_type == 'Polygon':
            candidate = merged_raw

    if candidate is None:
        n_comp = len(list(getattr(merged_raw, 'geoms', []))) if hasattr(merged_raw, 'geoms') else 0
        raise RuntimeError(
            f"El contorno no es simplemente conexo: se detectaron {n_comp or 'varios'} componentes desconectados."
        )

    return [[float(x), float(y)] for x, y in list(candidate.exterior.coords)]

def save_json_contour(data, filename):
    walls = [d for d in data if d['is_wall']]
    contour = contour_from_wall_union(walls)

    payload = {'layout': [contour]}

    with open(filename, 'w') as f:
        json.dump(payload, f, indent=4)
    print(f"💾 Archivo JSON (contorno) guardado: {filename}")
    return contour

def parse_args():
    parser = argparse.ArgumentParser(
        description="Convierte un layout de Webots (.wbt) a USDA o JSON y permite exportar solo el contorno."
    )
    parser.add_argument('input', help='Archivo de entrada .wbt')
    parser.add_argument(
        '-o', '--output',
        required=True,
        help='Archivo de salida obligatorio (.usda o .json)'
    )
    parser.add_argument(
        '--contour-only',
        action='store_true',
        help='Genera el layout sin objetos interiores (solo muros/contorno)'
    )

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    args = parser.parse_args()
    ext = os.path.splitext(args.output)[1].lower()
    if ext not in ('.usda', '.json'):
        parser.error("El archivo de salida (-o) debe terminar en .usda o .json")
    if ext == '.json' and not args.contour_only:
        parser.error("Para salida .json es obligatorio usar --contour-only")
    return args

def visualize(data, connected, contour_points=None, wall_components=None):
    if not data:
        print("❌ Nada que visualizar. Verifica las etiquetas de los objetos.")
        return

    plt.figure(figsize=(12, 12))
    ax = plt.gca()
    all_pts_x, all_pts_y = [], []

    for d in data:
        tx, ty, sx, sy = d['trans'][0], d['trans'][1], d['size'][0], d['size'][1]
        w, _, _, z = d['quat']
        angle = 2 * math.atan2(z, w)
        
        hw, hl = sx/2, sy/2
        corners = [(-hw, -hl), (hw, -hl), (hw, hl), (-hw, hl), (-hw, -hl)]
        rx, ry = [], []
        for cx, cy in corners:
            rx.append(cx * math.cos(angle) - cy * math.sin(angle) + tx)
            ry.append(cx * math.sin(angle) + cy * math.cos(angle) + ty)
        
        all_pts_x.extend(rx); all_pts_y.extend(ry)
        color = 'royalblue' if d['is_wall'] else 'orange'
        ax.fill(rx, ry, color=color, alpha=0.5, edgecolor='black', label=d['proto'] if d['proto'] not in ax.get_legend_handles_labels()[1] else "")

    if contour_points:
        contour_x = [p[0] for p in contour_points]
        contour_y = [p[1] for p in contour_points]
        all_pts_x.extend(contour_x)
        all_pts_y.extend(contour_y)
        ax.plot(
            contour_x,
            contour_y,
            color='crimson',
            linewidth=2.2,
            marker='o',
            markersize=2.8,
            label='JSON contour'
        )

    if wall_components and len(wall_components) > 1:
        comp_colors = ['crimson', 'darkorange', 'darkviolet', 'teal', 'goldenrod', 'magenta', 'black']
        for ci, comp in enumerate(wall_components):
            c = comp_colors[ci % len(comp_colors)]
            cx_vals, cy_vals = [], []
            for w in comp:
                corners = wall_corners_2d(w)
                xs = [p[0] for p in corners]
                ys = [p[1] for p in corners]
                cx_vals.append(w['trans'][0])
                cy_vals.append(w['trans'][1])
                ax.plot(xs, ys, color=c, linewidth=2.0, alpha=0.9)

            if cx_vals and cy_vals:
                ax.text(
                    sum(cx_vals) / len(cx_vals),
                    sum(cy_vals) / len(cy_vals),
                    f"C{ci+1}",
                    color=c,
                    fontsize=11,
                    fontweight='bold',
                    ha='center',
                    va='center',
                    bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8, edgecolor=c)
                )

        ax.text(
            0.02,
            0.98,
            f"Componentes desconectados: {len(wall_components)}",
            transform=ax.transAxes,
            ha='left',
            va='top',
            fontsize=10,
            color='crimson',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.85, edgecolor='crimson')
        )

        # Marca explicita de puntos de desconexion (pares mas cercanos).
        for li, link in enumerate(disconnection_links(wall_components), start=1):
            ax.plot(
                [link['a'][0], link['b'][0]],
                [link['a'][1], link['b'][1]],
                color='red',
                linestyle='--',
                linewidth=1.6,
                alpha=0.9,
                zorder=8
            )
            ax.scatter(
                [link['a'][0], link['b'][0]],
                [link['a'][1], link['b'][1]],
                color='red',
                marker='x',
                s=70,
                linewidths=2,
                zorder=9
            )
            mx = 0.5 * (link['a'][0] + link['b'][0])
            my = 0.5 * (link['a'][1] + link['b'][1])
            ax.text(
                mx,
                my,
                f"Gap {li}: {link['distance']:.2f}m",
                color='red',
                fontsize=9,
                ha='center',
                va='bottom',
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.85, edgecolor='red')
            )

    # Forzar límites del gráfico
    ax.set_xlim(min(all_pts_x)-1, max(all_pts_x)+1)
    ax.set_ylim(min(all_pts_y)-1, max(all_pts_y)+1)
    ax.set_aspect('equal', adjustable='box')
    
    title = "SIMPLY CONNECTED" if connected else "COMPLEX / DISCONNECTED"
    plt.title(f"Layout Final - {title}", fontsize=14, fontweight='bold', color='green' if connected else 'red')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.xlabel("X+ (Derecha) [m]"); plt.ylabel("Y+ (Adelante) [m]")
    plt.show()

if __name__ == "__main__":
    args = parse_args()
    objects = parse_wbt(args.input)
    filtered, conn = filter_by_layout(objects)
    contour_points = None

    if args.contour_only:
        filtered = [d for d in filtered if d['is_wall']]

    wall_components = detect_wall_components(filtered)
    conn = len(wall_components) <= 1 if wall_components else conn

    output_ext = os.path.splitext(args.output)[1].lower()
    if output_ext == '.usda':
        save_usda(filtered, args.output)
    else:
        try:
            contour_points = save_json_contour(filtered, args.output)
        except RuntimeError as exc:
            print(f"⚠️ {exc}")
            contour_points = None

    visualize(filtered, conn, contour_points=contour_points, wall_components=wall_components)
