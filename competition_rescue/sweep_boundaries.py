"""User-drawn left/right sweep boundaries, independent of claw capture ROI."""
from pathlib import Path
import json
from functools import lru_cache
import numpy as np

DEFAULT_PATH = Path(__file__).resolve().parents[1] / 'vision/config/sweep_boundaries.json'


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
        return {key: validated_pair(pair) for key, pair in data.get('angles', {}).items()
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
    checked = {str(k): validated_pair(v) for k, v in data.items()}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix('.json.tmp')
    tmp.write_text(json.dumps({'schema_version': 1, 'width': 1280, 'height': 1024,
        'interpolation': 'piecewise_linear', 'angles': checked}, ensure_ascii=False, indent=2)+'\n')
    tmp.replace(path)


def corridor_polygon(pair, safe_bottom):
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
