# CL-4.2
"""
Animator core (CL-4.2)  -  Pose Editor(CL-3.17) 위에 올리는 애니메이션 코어. Qt 비의존.

결정 사항
  - 키프레임 = 전체 포즈 스냅샷(노드 위치, 현재 링크 길이, F 각도)
  - 보간 = 뼈 각도 보간(FK) + solver로 제약 복원
  - 고정(pin)은 편집 보조 기능: 키프레임에 저장/보간하지 않는다(문서 단위로만 유지)
  - 애니메이션은 파일로 저장/불러오기 한다(JSON)

구성
  Keyframe / Animation        : 데이터와 evaluate(t) (키 사이 포즈 계산)
  AnimationEditor             : 시간 커서, 자동 키, 키 편집, 문서 단위 Undo/Redo, 저장/불러오기
  TimelineModel / advance_time: 타임라인 컴포넌트의 좌표·히트테스트·재생 계산(화면 비의존)

보간 개요
  1) 뼈대 트리(루트에서 BFS)를 만들고, 각 뼈의 길이와 '부모 뼈 기준 상대 각도'를 최단 경로로 보간
  2) FK로 위치를 재구성(트리 링크 길이는 정확히 유지)
  3) 루프·R·F·A·삼각형 방향 제약은 기존 solver(PoseEditor._attempt)로 복원
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass

from pose_editor_cl317 import EPS, Pose, PoseEditor, Rule, Session, dist, key, wrap

FILE_VERSION = 1
INTERP_MODES = ("linear", "ease", "hold")


# ----------------------------------------------------------------------
# 보간 도구
# ----------------------------------------------------------------------
def lerp(a, b, t):
    return a + (b - a) * t


def alerp(a, b, t):
    """각도 보간(최단 경로)"""
    return a + wrap(b - a) * t


def apply_curve(mode, u):
    if mode == "hold":
        return 0.0
    if mode == "ease":
        return u * u * (3 - 2 * u)
    return u


def choose_root(skel):
    """연결(링크) 수가 가장 많은 노드(동률이면 번호가 작은 노드)"""
    deg = {n: 0 for n in skel.nodes}
    for (a, b) in skel.max_len:
        deg[a] += 1
        deg[b] += 1
    return max(sorted(deg), key=lambda n: (deg[n], -n)) if deg else None


def build_tree(skel, root=None):
    """[(노드, 부모 또는 None)] BFS 순서. 연결되지 않은 덩어리는 각자 루트를 가진다."""
    adj = {n: [] for n in skel.nodes}
    for (a, b) in skel.max_len:
        adj[a].append(b)
        adj[b].append(a)
    for n in adj:
        adj[n].sort()
    order, seen = [], set()

    def grow(r):
        seen.add(r)
        order.append((r, None))
        q = [r]
        while q:
            u = q.pop(0)
            for v in adj[u]:
                if v not in seen:
                    seen.add(v)
                    order.append((v, u))
                    q.append(v)

    if root is not None and root in adj:
        grow(root)
    remaining = [n for n in sorted(adj) if n not in seen]
    while remaining:
        sub = {n: len([m for m in adj[n] if m not in seen]) for n in remaining}
        grow(max(remaining, key=lambda n: (sub[n], -n)))
        remaining = [n for n in sorted(adj) if n not in seen]
    return order


def _bone_data(pose, tree):
    parent = dict(tree)
    ang, length = {}, {}
    for n, p in tree:
        if p is None:
            continue
        vx, vy = pose.pos[n][0] - pose.pos[p][0], pose.pos[n][1] - pose.pos[p][1]
        ang[n] = math.atan2(vy, vx)
        length[n] = math.hypot(vx, vy)
    rel = {}
    for n, p in tree:
        if p is None:
            continue
        ref = ang[p] if parent[p] is not None else 0.0
        rel[n] = wrap(ang[n] - ref)
    return length, rel


def _triangle_signs(skel, pose):
    scratch = PoseEditor()
    scratch.skel = skel
    scratch.pose = pose
    return {(a, b, c): sg for (a, b, c, sg) in scratch._triangles()}


def interpolate_pose(skel, tree, pa, pb, u):
    """pa → pb 를 u(0~1)만큼 보간한 포즈. 키에서(u=0,1)는 호출하지 않는 것을 권장."""
    lenA, relA = _bone_data(pa, tree)
    lenB, relB = _bone_data(pb, tree)
    parent = dict(tree)
    pos, ang = {}, {}
    for n, p in tree:
        if p is None:
            pos[n] = (lerp(pa.pos[n][0], pb.pos[n][0], u), lerp(pa.pos[n][1], pb.pos[n][1], u))
            continue
        L = lerp(lenA[n], lenB[n], u)
        a = (ang[p] if parent[p] is not None else 0.0) + alerp(relA[n], relB[n], u)
        ang[n] = a
        pos[n] = (pos[p][0] + L * math.cos(a), pos[p][1] + L * math.sin(a))

    # 제약 복원: 루프·R·F·A·삼각형 방향(양쪽 키에서 방향이 같은 삼각형만)
    cur_len = {k: lerp(dist(pa.pos[k[0]], pa.pos[k[1]]), dist(pb.pos[k[0]], pb.pos[k[1]]), u)
               for k in skel.max_len}
    angles = {i: alerp(pa.angles[i], pb.angles[i], u) for i in pa.angles if i in pb.angles}
    sa, sb = _triangle_signs(skel, pa), _triangle_signs(skel, pb)
    tris = [(a, b, c, sg) for (a, b, c), sg in sa.items() if sb.get((a, b, c)) == sg]

    scratch = PoseEditor()
    scratch.skel = skel
    sp = Pose()
    sp.pos = dict(pos)
    sp.cur_len = dict(cur_len)
    sp.angles = angles
    scratch.pose = sp
    sess = Session(kind="node", nodes=[], link=None, alt=False, press=(0.0, 0.0), last=(0.0, 0.0),
                   base={}, snapshot=sp.copy())
    sess.keep = dict(cur_len)
    sess.r_passive = [(r.n1, r.n2, r.n3, r.ratio) for r in skel.rules if r.kind == "R"]
    sess.f_active = [i for i, r in enumerate(skel.rules) if r.kind == "F" and i in angles]
    sess.a_rules = [(r.n1, r.n2, r.n3, None) for r in skel.rules if r.kind == "A"]
    sess.tris = tris
    scratch.sess = sess
    ok, solved, _fd = scratch._attempt({})
    if ok:
        pos = {n: (p[0], p[1]) for n, p in solved.items()}

    out = Pose()
    out.pos = pos
    out.cur_len = {k: min(dist(pos[k[0]], pos[k[1]]), skel.max_len[k]) for k in skel.max_len}
    out.angles = angles
    return out


# ----------------------------------------------------------------------
# 데이터
# ----------------------------------------------------------------------
@dataclass
class Keyframe:
    frame: int
    pose: Pose                  # pinned는 항상 비어 있음(고정은 키에 저장하지 않는다)
    interp: str = "linear"      # 이 키에서 다음 키까지의 보간: linear | ease | hold

    def copy(self):
        return Keyframe(self.frame, self.pose.copy(), self.interp)


class Animation:
    def __init__(self, skel, rest, fps=24, length=48, loop=False, root=None):
        self.skel = skel
        self.rest = rest                    # 키가 없을 때 보여줄 기본 포즈
        self.keys = {}                      # frame(int) -> Keyframe
        self.fps = fps
        self.length = length
        self.loop = loop
        self.root = root

    def frames(self):
        return sorted(self.keys)

    def tree(self):
        return build_tree(self.skel, self.root if self.root is not None else choose_root(self.skel))

    def evaluate(self, t):
        """시간 t(프레임, 소수 가능)의 포즈를 계산(pinned는 비어 있음)"""
        fr = self.frames()
        if not fr:
            p = self.rest.copy()
            p.pinned = set()
            return p
        T = self.length if (self.loop and self.length > 0) else None
        if T is not None:
            t = t % T
            if t in self.keys:
                return self._exact(t)
            if t < fr[0]:
                a, b = fr[-1] - T, fr[0]
                ka, kb = self.keys[fr[-1]], self.keys[fr[0]]
            elif t > fr[-1]:
                a, b = fr[-1], fr[0] + T
                ka, kb = self.keys[fr[-1]], self.keys[fr[0]]
            else:
                ka, kb, a, b = self._bracket(t, fr)
        else:
            if t <= fr[0]:
                return self._exact(fr[0])
            if t >= fr[-1]:
                return self._exact(fr[-1])
            if t in self.keys:
                return self._exact(t)
            ka, kb, a, b = self._bracket(t, fr)
        if ka is kb or b == a:
            return self._exact(ka.frame)
        u = apply_curve(ka.interp, (t - a) / (b - a))
        if u <= 0.0:
            return self._exact(ka.frame)
        return interpolate_pose(self.skel, self.tree(), ka.pose, kb.pose, u)

    def _exact(self, frame):
        p = self.keys[frame].pose.copy()
        p.pinned = set()
        return p

    def _bracket(self, t, fr):
        i = max(j for j, f in enumerate(fr) if f <= t)
        return self.keys[fr[i]], self.keys[fr[i + 1]], fr[i], fr[i + 1]


# ----------------------------------------------------------------------
# 에디터(컨트롤러)
# ----------------------------------------------------------------------
class AnimationEditor:
    def __init__(self, editor=None, fps=24, length=48):
        self.editor = editor if editor is not None else PoseEditor()
        rest = self.editor.pose.copy()
        rest.pinned = set()
        self.anim = Animation(self.editor.skel, rest, fps=fps, length=length)
        self.t = 0.0
        self.auto_key = True
        self._undo, self._redo = [], []

    # ---- 시간 ----------------------------------------------------
    @property
    def frame(self):
        return int(round(self.t))

    def set_time(self, t):
        """시간 커서를 옮기고 그 시간의 포즈를 화면(에디터)에 적용"""
        if self.editor.sess is not None:
            return
        self.t = max(0.0, float(t))
        self._show(self.anim.evaluate(self.t))

    def _show(self, pose):
        pose.pinned = set(self.editor.pose.pinned)      # 고정은 시간과 무관한 편집 보조
        self.editor.pose = pose

    def is_key(self, frame=None):
        return (self.frame if frame is None else frame) in self.anim.keys

    def key_frames(self):
        return self.anim.frames()

    # ---- 드래그(포즈 편집) ----------------------------------------
    def begin_drag_node(self, n, alt=False):
        return self.editor.begin_drag_node(n, alt)

    def begin_drag_link(self, a, b, press, alt=False):
        return self.editor.begin_drag_link(a, b, press, alt)

    def drag_move(self, target, alt=False):
        return self.editor.drag_move(target, alt)

    def end_drag(self):
        """드래그가 끝나면(자동 키 켜짐) 현재 프레임이 키프레임이 된다."""
        e, s = self.editor, self.editor.sess
        if s is None:
            return
        before = s.snapshot.copy()
        before.pinned = set()
        e.end_drag()
        e._undo.clear()
        e._redo.clear()
        changed = not e.pose.same_as(s.snapshot)
        if changed and self.auto_key:
            self._push_undo()
            f = self.frame
            if not self.anim.keys and f > 0:            # 첫 키가 0이 아니면 드래그 전 포즈를 0프레임에 자동 저장
                self._store(0, before)
            self._store(f, e.pose)

    def toggle_pin(self, n):
        if self.editor.sess is not None:
            return
        self._push_undo()
        pins = self.editor.pose.pinned
        pins.discard(n) if n in pins else pins.add(n)

    # ---- 키 편집 -------------------------------------------------
    def _store(self, frame, pose):
        p = pose.copy()
        p.pinned = set()
        old = self.anim.keys.get(frame)
        self.anim.keys[frame] = Keyframe(frame, p, old.interp if old else "linear")
        self.anim.length = max(self.anim.length, frame)

    def add_key(self, frame=None):
        """현재(화면) 포즈를 키로 저장"""
        self._push_undo()
        self._store(self.frame if frame is None else int(frame), self.editor.pose)

    def delete_key(self, frame):
        if frame not in self.anim.keys:
            return False
        self._push_undo()
        del self.anim.keys[frame]
        self.set_time(self.t)
        return True

    def move_key(self, src, dst, overwrite=False):
        if src not in self.anim.keys or src == dst or (dst in self.anim.keys and not overwrite) or dst < 0:
            return False
        self._push_undo()
        k = self.anim.keys.pop(src)
        self.anim.keys[dst] = Keyframe(dst, k.pose, k.interp)
        self.anim.length = max(self.anim.length, dst)
        self.set_time(self.t)
        return True

    def copy_key(self, src, dst, overwrite=False):
        if src not in self.anim.keys or src == dst or (dst in self.anim.keys and not overwrite) or dst < 0:
            return False
        self._push_undo()
        k = self.anim.keys[src]
        self.anim.keys[dst] = Keyframe(dst, k.pose.copy(), k.interp)
        self.anim.length = max(self.anim.length, dst)
        self.set_time(self.t)
        return True

    def set_interp(self, frame, mode):
        if frame not in self.anim.keys or mode not in INTERP_MODES:
            return False
        self._push_undo()
        self.anim.keys[frame].interp = mode
        self.set_time(self.t)
        return True

    # ---- Undo / Redo (문서 단위) ----------------------------------
    def _snapshot(self):
        return ({f: k.copy() for f, k in self.anim.keys.items()}, set(self.editor.pose.pinned), self.anim.length)

    def _restore(self, snap):
        keys, pins, length = snap
        self.anim.keys = {f: k.copy() for f, k in keys.items()}
        self.anim.length = length
        self.editor.pose.pinned = set(pins)
        self.set_time(self.t)

    def _push_undo(self):
        self._undo.append(self._snapshot())
        self._redo.clear()

    def undo(self):
        if self.editor.sess is not None or not self._undo:
            return False
        self._redo.append(self._snapshot())
        self._restore(self._undo.pop())
        return True

    def redo(self):
        if self.editor.sess is not None or not self._redo:
            return False
        self._undo.append(self._snapshot())
        self._restore(self._redo.pop())
        return True

    # ---- 저장 / 불러오기 ------------------------------------------
    def to_dict(self):
        sk, a = self.anim.skel, self.anim
        return {
            "version": FILE_VERSION,
            "skeleton": {
                "nodes": list(sk.nodes),
                "links": [[x, y, L] for (x, y), L in sk.max_len.items()],
                "rules": [[r.kind, r.n1, r.n2, r.n3, r.ratio] for r in sk.rules],
            },
            "fps": a.fps, "length": a.length, "loop": a.loop, "root": a.root,
            "rest": _pose_to_list(a.rest),
            "keys": [{"frame": f, "interp": k.interp, "pose": _pose_to_list(k.pose)}
                     for f, k in sorted(a.keys.items())],
        }

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=1)

    @staticmethod
    def from_dict(d):
        if d.get("version") != FILE_VERSION:
            raise ValueError(f"지원하지 않는 파일 버전: {d.get('version')}")
        ed = PoseEditor()
        sk = ed.skel
        sk.nodes = list(d["skeleton"]["nodes"])
        sk.max_len = {key(x, y): L for x, y, L in d["skeleton"]["links"]}
        sk.rules = [Rule(k, n1, n2, n3, ratio) for k, n1, n2, n3, ratio in d["skeleton"]["rules"]]
        rest = _pose_from_list(d["rest"])
        ed.pose = rest.copy()
        ae = AnimationEditor(ed, fps=d["fps"], length=d["length"])
        ae.anim.rest = rest
        ae.anim.loop = d["loop"]
        ae.anim.root = d["root"]
        for k in d["keys"]:
            ae.anim.keys[k["frame"]] = Keyframe(k["frame"], _pose_from_list(k["pose"]), k["interp"])
        ae._undo, ae._redo = [], []
        ae.set_time(0)
        return ae

    @staticmethod
    def load(path):
        with open(path, encoding="utf-8") as f:
            return AnimationEditor.from_dict(json.load(f))


def _pose_to_list(p):
    return {"pos": [[n, x, y] for n, (x, y) in sorted(p.pos.items())],
            "len": [[a, b, L] for (a, b), L in sorted(p.cur_len.items())],
            "angles": [[i, a] for i, a in sorted(p.angles.items())]}


def _pose_from_list(d):
    p = Pose()
    p.pos = {n: (x, y) for n, x, y in d["pos"]}
    p.cur_len = {key(a, b): L for a, b, L in d["len"]}
    p.angles = {i: a for i, a in d["angles"]}
    return p


# ----------------------------------------------------------------------
# 타임라인 계산 (화면 비의존: PyQt 위젯은 이것만 호출해서 그린다)
# ----------------------------------------------------------------------
def advance_time(t, dt_seconds, fps, length, loop):
    """재생: 반환 (새 시간, 끝났는지). 루프면 length로 순환, 아니면 length에서 멈춘다."""
    nt = t + dt_seconds * fps
    if loop and length > 0:
        return nt % length, False
    if nt >= length:
        return float(length), True
    return nt, False


class TimelineModel:
    """프레임 <-> 픽셀 변환, 키 마커 히트테스트, 눈금, 확대/이동"""

    def __init__(self, width=800, view_start=0.0, view_span=48.0, margin=12):
        self.width = width
        self.view_start = view_start
        self.view_span = view_span
        self.margin = margin

    @property
    def px_per_frame(self):
        return (self.width - 2 * self.margin) / self.view_span

    def frame_to_x(self, f):
        return self.margin + (f - self.view_start) * self.px_per_frame

    def x_to_frame(self, x, snap=True, min_frame=0):
        f = self.view_start + (x - self.margin) / self.px_per_frame
        if snap:
            f = round(f)
        return max(min_frame, f)

    def hit_key(self, x, frames, radius=7):
        """x 근처의 키 프레임(가장 가까운 것). 없으면 None"""
        best, bd = None, radius + 1e-9
        for f in frames:
            d = abs(self.frame_to_x(f) - x)
            if d <= bd:
                best, bd = f, d
        return best

    def ticks(self, min_px=40):
        """[(프레임, 주요 눈금 여부)]  눈금 간격은 화면에서 min_px 이상이 되도록 1,2,5,10,... 중에서 선택"""
        step = 1
        for s in (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000):
            step = s
            if s * self.px_per_frame >= min_px:
                break
        first = int(math.ceil(self.view_start / step) * step)
        last = int(math.floor((self.view_start + self.view_span) / step) * step)
        return [(f, (f // step) % 5 == 0) for f in range(first, last + 1, step) if f >= 0]

    def zoom(self, factor, anchor_x):
        """anchor_x 위치의 프레임을 고정한 채 확대(factor>1)/축소"""
        anchor = self.view_start + (anchor_x - self.margin) / self.px_per_frame
        self.view_span = max(4.0, min(5000.0, self.view_span / factor))
        self.view_start = max(0.0, anchor - (anchor_x - self.margin) / self.px_per_frame)

    def pan(self, dx_px):
        self.view_start = max(0.0, self.view_start - dx_px / self.px_per_frame)

    def ensure_visible(self, f):
        if f < self.view_start:
            self.view_start = max(0.0, f)
        elif f > self.view_start + self.view_span:
            self.view_start = f - self.view_span


class TimelineController:
    """타임라인 마우스 동작(화면 비의존). PyQt 위젯은 이벤트를 이 컨트롤러에 전달하고 결과를 그리기만 한다.
       - 빈 곳 누르고 끌기: 시간 이동(스크럽)
       - 키 마커 누르기: 선택 + 그 시간으로 이동, 끌어서 놓으면 키 이동(Alt: 복제)
       - 휠: 이동, Ctrl+휠: 확대/축소"""

    def __init__(self, ae, model):
        self.ae = ae
        self.m = model
        self.selected = None
        self.preview = None            # 키를 끄는 중일 때 놓을 위치(프레임)
        self._mode = None
        self._src = None

    def press(self, x):
        f = self.m.hit_key(x, self.ae.key_frames())
        if f is not None:
            self.selected, self._mode, self._src, self.preview = f, "key", f, f
            self.ae.set_time(f)
        else:
            self.selected, self._mode, self._src, self.preview = None, "scrub", None, None
            self.ae.set_time(self.m.x_to_frame(x))

    def move(self, x):
        if self._mode == "scrub":
            self.ae.set_time(self.m.x_to_frame(x))
        elif self._mode == "key":
            self.preview = self.m.x_to_frame(x)

    def release(self, x, copy=False):
        if self._mode == "key":
            dst = self.m.x_to_frame(x)
            if dst != self._src:
                ok = self.ae.copy_key(self._src, dst) if copy else self.ae.move_key(self._src, dst)
                if ok:
                    self.selected = dst
                    self.ae.set_time(dst)
        self._mode, self._src, self.preview = None, None, None

    def wheel(self, x, delta, ctrl=False):
        if ctrl:
            self.m.zoom(1.25 if delta > 0 else 0.8, x)
        else:
            self.m.pan(delta)

    def delete_selected(self):
        if self.selected is not None and self.ae.delete_key(self.selected):
            self.selected = None
            return True
        return False
