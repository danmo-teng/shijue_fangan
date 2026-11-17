"""User-drawn closed sweep region; retain legacy left/right file support."""
from pathlib import Path
import json
from functools import lru_cache
import numpy as np
from datetime import datetime
import shutil

DEFAULT_PATH = Path(__file__).resolve().parents[1] / 'vision/config/sweep_boundaries.json'


def obstacle_reference_point(class_name, bbox):
    """Sweep-region membership only; carried cargo and safe-zone tests keep their rules."""
    x, y, width, height = bbox
    return (float(x + width * .5),
            float(y + height * (.5 if class_name == 'danger_cyan' else 1.)))


def validated_region(region):
    points = region.get('polygon')
    if not isinstance(points, (list, tuple)) or len(points) < 3:
        raise ValueError('请沿区域轮廓至少标三个点，再保存闭合区域')
    result = []
    for point in points:
        if (not isinstance(point, (list, tuple)) or len(point) != 2 or
                not all(isinstance(v, (int, float)) and np.isfinite(v) for v in point) or
                not (0 <= point[0] < 1280 and 0 <= point[1] < 1024)):
            raise ValueError('扫障区域坐标须在1280×1024画面内')
        p = list(map(float, point))
        if not result or p != result[-1]:
            result.append(p)
    if len(result) > 1 and result[0] == result[-1]:
        result.pop()
    if len(result) < 3:
        raise ValueError('闭合区域至少需要三个不同的点')

    def cross(a, b, c):
        return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])

    def intersects(a, b, c, d):
        if (max(a[0],b[0]) < min(c[0],d[0]) or max(c[0],d[0]) < min(a[0],b[0]) or
                max(a[1],b[1]) < min(c[1],d[1]) or max(c[1],d[1]) < min(a[1],b[1])):
            return False
        return cross(a,b,c)*cross(a,b,d) <= 0 and cross(c,d,a)*cross(c,d,b) <= 0

    n = len(result)
    area = sum(result[i][0]*result[(i+1)%n][1]-result[(i+1)%n][0]*result[i][1] for i in range(n))
    if abs(area) < 1:
        raise ValueError('区域没有面积，请沿外轮廓画一圈')
    for i in range(n):
        for j in range(i+1, n):
            if j == i+1 or (i == 0 and j == n-1):
                continue
            if intersects(result[i], result[(i+1)%n], result[j], result[(j+1)%n]):
                raise ValueError('轮廓不能交叉；请按边界顺序描一圈')
    return {'polygon': result}


def validated_boundary(value):
    return validated_region(value) if 'polygon' in value else validated_pair(value)


def validated_pair(pair):
    result = {}
    for side in ('left', 'right'):
        points = sorted(pair[side], key=lambda p: p[1])
        if len(points) < 2 or any(len(p) != 2 or not (0 <= p[0] < 1280 and 0 <= p[1] < 1024) for p in points):
            raise ValueError('每条边界至少两个点，坐标须在画面内')
        if len({p[1] for p in points}) != len(points):
            raise ValueError('同一条边界不能有两个相同高度的点')
        result[side] = [list(map(float, p)) for p in points]
    top = max(result['left'][0][1], result['right'][0][1])
    bottom = min(result['left'][-1][1], result['right'][-1][1])
    if top >= bottom:
        raise ValueError('左右边界必须有共同的高度范围')
    ys = sorted({top, bottom} | {p[1] for ps in result.values() for p in ps if top <= p[1] <= bottom})
    left, right = (_xs(result[side], ys) for side in ('left', 'right'))
    if np.any(left >= right):
        raise ValueError('左右边界不能交叉或重合')
    return result


def _xs(points, ys):
    return np.interp(ys, [p[1] for p in points], [p[0] for p in points])


def load_boundaries(path=DEFAULT_PATH):
    try:
        data = json.loads(Path(path).read_text())
        return {key: validated_boundary(pair) for key, pair in data.get('angles', {}).items()
                if key in {'11500', '12000'}}
    except FileNotFoundError:
        return {}


@lru_cache(maxsize=1)
def runtime_boundaries():
    try:
        return load_boundaries()
    except (ValueError, KeyError, TypeError, OSError) as error:
        print(f'扫障边界文件无效，沿用原矩形：{error}')
        return {}


def save_boundaries(path, data):
    checked = {str(k): validated_boundary(v) for k, v in data.items()}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(json.dumps({'schema_version': 2, 'width': 1280, 'height': 1024,
        'interpolation': 'piecewise_linear', 'object_reference': 'bbox_bottom_center',
        'object_reference_overrides': {'danger_cyan': 'bbox_center'},
        'angles': checked}, ensure_ascii=False, indent=2)+'\n')
    if path.exists():
        backup = path.with_name(path.name + '.' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.bak')
        shutil.copy2(path, backup)
    tmp.replace(path)


def corridor_polygon(pair, safe_bottom):
    if 'polygon' in pair:
        # A complete user outline owns every edge. No bounding rectangle or
        # interpolation/extrapolation from two independently drawn sides.
        return tuple((round(x), round(y)) for x,y in pair['polygon'])
    top = max(float(safe_bottom), pair['left'][0][1], pair['right'][0][1])
    bottom = min(pair['left'][-1][1], pair['right'][-1][1])
    if top >= bottom:
        # Preserve the existing zero-length corridor behavior when the zone
        # edge reaches the near boundary; do not create a new waiting state.
        left, right = (_xs(pair[side], [bottom])[0] for side in ('left', 'right'))
        return ((round(left), round(bottom)), (round(right), round(bottom)),
                (round(right), round(bottom)), (round(left), round(bottom)))
    ys = sorted({top, bottom} | {p[1] for ps in pair.values() for p in ps if top < p[1] < bottom})
    left, right = (_xs(pair[side], ys) for side in ('left', 'right'))
    return tuple([(round(x), round(y)) for x, y in zip(left[::-1], ys[::-1])] +
                 [(round(x), round(y)) for x, y in zip(right, ys)])
