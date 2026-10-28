# CL-3.13
"""
Pose Editor  (CL-3.13)

구성
  1) 코어(Qt 비의존): Skeleton(Object) / Pose / PoseEditor(solver, Undo/Redo)
  2) UI(PyQt6): 맨 아래 run_ui()  - 코어만 쓸 때는 PyQt6가 없어도 import 가능

solver: 위치 기반 제약 투영(PBD, Gauss-Seidel) + 규칙 우선순위
  드래그 노드는 '가장 약한 당김'으로 목표를 따라가고(1단계), 이어서 제약만 다듬는다(2단계)
  (목표에 못 닿으면 제약을 만족하는 가장 가까운 위치로 이동)
  우선순위  고정 > 최대 길이 > R > 현재 길이 유지 > F
  - 투영 순서는 낮은 우선순위부터 높은 순서(F, 길이유지, R, 최대길이, 고정)
  - A(1,2,3): 노드 1은 선분 2-3 위에서만 이동. 노드 1이 고정(pin)이면 선 위 비율 t가 고정되어 선과 함께 움직인다
  - R은 노드 1·3을 직접 끌 때만 켜지며, 그동안 중심 노드(2)는 위치 고정이다
  - 삼각형(서로 연결된 노드 3개)은 방향(부호 있는 면적)을 유지: 뒤집힘 방지 = 최대 길이와 같은 하드 제약
  - F의 노드 1이 직접 드래그되거나 R에 의해 움직이면 그 F는 해제되고, 이동 후 각도를 새로 저장
  - 해가 없으면 F를 먼저 버려 보고, 그래도 안 되면 이동을 막는다
  - 목표 위치까지 못 가면 갈 수 있는 곳까지만 이동(이분 탐색)

데이터 분리
  Object(Skeleton): 노드/링크 구조, 최초 길이(max_len), 규칙(F, R)
  Pose            : 노드 위치, 현재 링크 길이, F 각도, 고정(pin) 노드
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

TOL = 0.02        # 제약 만족 허용 오차(px)
PULL_ITER = 40    # 1단계: 드래그 노드를 목표로 당기며 제약 투영
POLISH_ITER = 80  # 2단계: 당김을 끄고 제약만 다듬어 허용 오차 이내로 수렴
EPS_H = 0.5       # 삼각형 높이 최소값(px): 이보다 납작해지면(=뒤집히려 하면) 막는다
SOFT = 0.1        # 1단계에서 드래그 노드의 역질량(작을수록 목표를 강하게 따라감)
EPS = 1e-9


def key(a, b):
    return (a, b) if a < b else (b, a)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


# ----------------------------------------------------------------------
# 데이터
# ----------------------------------------------------------------------
@dataclass
class Rule:
    kind: str            # 'F' 또는 'R'
    n1: int
    n2: int              # 꼭짓점 / 대칭 중심
    n3: int
    ratio: float = 1.0   # R 전용: 초기 |3-2| / |1-2|


class Skeleton:
    """Object: 구조와 최초 길이, 규칙"""

    def __init__(self):
        self.nodes = []
        self.max_len = {}    # key(a,b) -> 최초 길이
        self.rules = []


class Pose:
    """Pose: 포즈마다 달라지는 값"""

    def __init__(self):
        self.pos = {}        # node -> (x, y)
        self.cur_len = {}    # key(a,b) -> 현재 길이
        self.angles = {}     # rule index -> F 각도(rad)
        self.pinned = set()

    def copy(self):
        p = Pose()
        p.pos = dict(self.pos)
        p.cur_len = dict(self.cur_len)
        p.angles = dict(self.angles)
        p.pinned = set(self.pinned)
        return p

    def same_as(self, o):
        return (self.pos == o.pos and self.cur_len == o.cur_len
                and self.angles == o.angles and self.pinned == o.pinned)


@dataclass
class Session:
    kind: str                    # 'node' | 'link'
    nodes: list                  # 드래그로 직접 움직이는 노드(운동학적 노드)
    link: tuple | None
    alt: bool
    press: tuple
    last: tuple
    base: dict                   # 세션 시작 시 노드 위치
    snapshot: Pose               # Undo용
    r_active: list = field(default_factory=list)   # (시작, 상대, 중심, k)
    keep: dict = field(default_factory=dict)       # 현재 길이 유지 대상
    f_active: list = field(default_factory=list)   # 유지할 F 규칙 index
    f_release: set = field(default_factory=set)    # 해제 후 재저장할 F index
    a_rules: list = field(default_factory=list)    # (n1, n2, n3, 고정 비율 또는 None)
    tris: list = field(default_factory=list)       # (a, b, c, 방향 부호) 뒤집힘 방지 대상
    f_dropped: bool = False


# ----------------------------------------------------------------------
# 에디터(코어)
# ----------------------------------------------------------------------
class PoseEditor:
    def __init__(self):
        self.skel = Skeleton()
        self.pose = Pose()
        self._undo = []
        self._redo = []
        self.sess = None

    # ---- 구성 API -------------------------------------------------
    def add_node(self, nid, x, y):
        self.skel.nodes.append(nid)
        self.pose.pos[nid] = (float(x), float(y))

    def add_line(self, n1, n2):
        L = dist(self.pose.pos[n1], self.pose.pos[n2])
        k = key(n1, n2)
        self.skel.max_len[k] = L
        self.pose.cur_len[k] = L

    def add_rule(self, kind, n1, n2, n3):
        kind = kind.upper()
        if kind not in ("F", "R", "A"):
            raise ValueError("kind must be 'F', 'R' or 'A'")
        r = Rule(kind, n1, n2, n3)
        if kind == "R":
            d1 = dist(self.pose.pos[n1], self.pose.pos[n2])
            d3 = dist(self.pose.pos[n3], self.pose.pos[n2])
            r.ratio = d3 / d1 if d1 > EPS else 1.0
        self.skel.rules.append(r)
        if kind == "A":                        # 노드 1을 선분 2-3 위로 스냅
            self.pose.pos[n1] = self._seg_point(self.pose.pos, r)
        if kind == "F":
            self.pose.angles[len(self.skel.rules) - 1] = self._angle(self.pose.pos, r)

    @staticmethod
    def _seg_t(pos, n1, n2, n3):
        """노드 1을 선분 2-3에 사영한 비율 t (0~1로 제한)"""
        (x1, y1), (x2, y2), (x3, y3) = pos[n1], pos[n2], pos[n3]
        dx, dy = x3 - x2, y3 - y2
        L2 = dx * dx + dy * dy
        if L2 < EPS:
            return 0.0
        return max(0.0, min(1.0, ((x1 - x2) * dx + (y1 - y2) * dy) / L2))

    def _seg_point(self, pos, r):
        t = self._seg_t(pos, r.n1, r.n2, r.n3)
        return (pos[r.n2][0] + t * (pos[r.n3][0] - pos[r.n2][0]),
                pos[r.n2][1] + t * (pos[r.n3][1] - pos[r.n2][1]))

    @staticmethod
    def _angle(pos, r):
        p1, p2, p3 = pos[r.n1], pos[r.n2], pos[r.n3]
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v3 = (p3[0] - p2[0], p3[1] - p2[1])
        return math.atan2(v1[0] * v3[1] - v1[1] * v3[0], v1[0] * v3[0] + v1[1] * v3[1])

    # ---- 고정(pin) / Undo ----------------------------------------
    def toggle_pin(self, n):
        if self.sess is not None:
            return
        self._push(self.pose.copy())
        if n in self.pose.pinned:
            self.pose.pinned.discard(n)
        else:
            self.pose.pinned.add(n)

    def _push(self, snap):
        self._undo.append(snap)
        self._redo.clear()

    def undo(self):
        if self.sess is not None or not self._undo:
            return False
        self._redo.append(self.pose)
        self.pose = self._undo.pop()
        return True

    def redo(self):
        if self.sess is not None or not self._redo:
            return False
        self._undo.append(self.pose)
        self.pose = self._redo.pop()
        return True

    # ---- 드래그 API -----------------------------------------------
    def begin_drag_node(self, n, alt=False):
        if self.sess is not None:
            return False
        if n in self.pose.pinned:
            return False
        pos = self.pose.pos[n]
        self.sess = self._build_session("node", [n], None, alt, pos, self.pose.copy())
        return True

    def begin_drag_link(self, a, b, press, alt=False):
        if self.sess is not None:
            return False
        if a in self.pose.pinned and b in self.pose.pinned:
            return False
        self.sess = self._build_session("link", [a, b], key(a, b), alt, press, self.pose.copy())
        return True

    def drag_move(self, target, alt=False):
        """반환: 'ok'(목표까지 이동) | 'partial'(가장 가까운 가능 위치까지) | 'blocked'(이동 불가)"""
        s = self.sess
        if s is None:
            return "blocked"
        if alt != s.alt:                       # Alt 토글: 그 순간 상태를 새 기준으로
            self._rebase(alt)
            s = self.sess
        s.last = target
        kin = self._kin_target(target)
        if kin is None:
            return "blocked"
        cur = {n: self.pose.pos[n] for n in kin}
        ok, pos, fd = self._attempt(kin)
        if not ok:                             # 하드 제약(고정·길이·R·삼각형) 충돌: 이동 거부
            return "blocked"
        if s.f_active and not fd:              # F 때문에 목표에 못 닿으면 F(가장 낮은 우선순위)를 버려 본다
            err = max(dist(pos[n], kin[n]) for n in kin)
            if err > 1.0:
                pos2, rep2 = self._solve(kin, False)
                if max(rep2.values()) <= TOL and max(dist(pos2[n], kin[n]) for n in kin) < err - 0.5:
                    pos, fd = pos2, True
        return self._finish(pos, fd, kin, cur)

    def _finish(self, pos, fd, kin, cur):
        """해를 적용하고 결과 상태를 판정(목표에 못 닿으면 가장 가까운 가능 위치로 이동)"""
        err = max(dist(pos[n], kin[n]) for n in kin)
        prog = max(dist(pos[n], cur[n]) for n in kin)
        if prog < 0.5 and err > 1.0:
            return "blocked"
        self._apply(pos, fd)
        return "ok" if err <= 1.0 else "partial"

    def end_drag(self):
        s = self.sess
        if s is None:
            return
        self._commit(s)
        if not self.pose.same_as(s.snapshot):
            self._push(s.snapshot)
        self.sess = None

    # ---- 세션 구성 ------------------------------------------------
    def _build_session(self, kind, nodes, link, alt, press, snapshot):
        S = set(nodes)
        r_active, exempt = [], set()
        if kind == "node":
            s = nodes[0]
            for r in self.skel.rules:
                if r.kind != "R":
                    continue
                if s == r.n1:
                    o, k = r.n3, r.ratio
                elif s == r.n3:
                    o, k = r.n1, (1.0 / r.ratio if r.ratio > EPS else 1.0)
                else:
                    continue
                r_active.append((s, o, r.n2, k))
                exempt |= {key(s, r.n2), key(o, r.n2)}
        if alt:                                # Alt: 시작 노드의 링크는 최대 길이까지 자유
            for k in self.skel.max_len:
                if k[0] in S or k[1] in S:
                    exempt.add(k)
        keep = {k: L for k, L in self.pose.cur_len.items() if k not in exempt}
        driven = {o for (_s, o, _c, _k) in r_active}        # R이 구동하는 노드도 '움직이는 노드'
        f_release = {i for i, r in enumerate(self.skel.rules)
                     if r.kind == "F" and (alt or r.n1 in S or r.n1 in driven)}
        f_active = [i for i, r in enumerate(self.skel.rules)
                    if r.kind == "F" and i not in f_release]
        base = {n: self.pose.pos[n] for n in nodes}
        sess = Session(kind, list(nodes), link, alt, press, press, base, snapshot,
                       r_active, keep, f_active, f_release)
        sess.tris = self._triangles()
        for r in self.skel.rules:
            if r.kind == "A":                  # 고정된 A 노드: 선 위 비율 t를 고정
                tf = self._seg_t(self.pose.pos, r.n1, r.n2, r.n3) if r.n1 in self.pose.pinned else None
                sess.a_rules.append((r.n1, r.n2, r.n3, tf))
        return sess

    def _triangles(self):
        """서로 연결된 노드 3개(삼각형)와 현재 방향 부호. 이미 납작한 삼각형은 제외."""
        adj = {}
        for (a, b) in self.skel.max_len:
            adj.setdefault(a, set()).add(b)
            adj.setdefault(b, set()).add(a)
        out, pos = [], self.pose.pos
        for a in sorted(adj):
            for b in sorted(x for x in adj[a] if x > a):
                for c in sorted(x for x in adj[b] if x > b and x in adj[a]):
                    d = dist(pos[a], pos[b])
                    if d < EPS:
                        continue
                    cr = ((pos[b][0] - pos[a][0]) * (pos[c][1] - pos[a][1])
                          - (pos[b][1] - pos[a][1]) * (pos[c][0] - pos[a][0]))
                    if abs(cr) / d >= 2 * EPS_H:
                        out.append((a, b, c, 1.0 if cr > 0 else -1.0))
        return out

    def _rebase(self, alt):
        s = self.sess
        self._commit(s)
        ns = self._build_session(s.kind, s.nodes, s.link, alt, s.last, s.snapshot)
        self.sess = ns

    def _commit(self, s):
        """이동이 끝났을 때(또는 Alt 전환 시) 현재 길이와 F 각도를 새로 확정"""
        pose = self.pose
        for k, M in self.skel.max_len.items():
            if k not in s.keep:
                pose.cur_len[k] = min(dist(pose.pos[k[0]], pose.pos[k[1]]), M)
        idxs = set(s.f_release)
        if s.f_dropped:
            idxs |= {i for i, r in enumerate(self.skel.rules) if r.kind == "F"}
        for i in idxs:
            pose.angles[i] = self._angle(pose.pos, self.skel.rules[i])
        s.f_dropped = False

    # ---- 운동학적 목표 --------------------------------------------
    def _kin_target(self, target):
        s, pose = self.sess, self.pose
        if s.kind == "node":
            return {s.nodes[0]: target}
        a, b = s.nodes
        pa, pb = a in pose.pinned, b in pose.pinned
        if pa and pb:
            return None
        if not pa and not pb:
            dx, dy = target[0] - s.press[0], target[1] - s.press[1]
            return {a: (s.base[a][0] + dx, s.base[a][1] + dy),
                    b: (s.base[b][0] + dx, s.base[b][1] + dy)}
        p, f = (a, b) if pa else (b, a)        # 한쪽 고정: 고정 노드 중심 회전
        pp = pose.pos[p]
        vx, vy = target[0] - pp[0], target[1] - pp[1]
        d = math.hypot(vx, vy)
        if d < EPS:
            vx, vy, d = 1.0, 0.0, 1.0
        k = key(a, b)
        L = pose.cur_len[k] if not s.alt else min(d, self.skel.max_len[k])
        return {f: (pp[0] + vx / d * L, pp[1] + vy / d * L)}

    # ---- solver ---------------------------------------------------
    def _attempt(self, kinpos):
        s = self.sess
        for use_f in ([True, False] if s.f_active else [False]):
            pos, rep = self._solve(kinpos, use_f)
            if max(rep.values()) <= TOL:
                return True, pos, (not use_f and bool(s.f_active))
        return False, None, False

    def _apply(self, pos, f_dropped):
        self.pose.pos = {n: (p[0], p[1]) for n, p in pos.items()}
        if f_dropped:
            self.sess.f_dropped = True

    def _solve(self, kinpos, use_f):
        s, pose, sk = self.sess, self.pose, self.skel
        pos = {n: [p[0], p[1]] for n, p in pose.pos.items()}
        for n, p in kinpos.items():
            pos[n] = [p[0], p[1]]
        w = {n: (0.0 if n in pose.pinned else (SOFT if n in kinpos else 1.0)) for n in pos}
        fr = [(sk.rules[i], pose.angles[i]) for i in s.f_active] if use_f else []
        for (_src, _o, c, _k) in s.r_active:                   # R 활성 중에는 중심 노드를 위치 고정
            w[c] = 0.0
        for (n1, _n2, _n3, tf) in s.a_rules:                   # 고정된 A 노드는 위치가 아니라 선 위 비율만 고정
            if tf is not None:
                w[n1] = 1.0
        for phase, iters in (("pull", PULL_ITER), ("polish", POLISH_ITER)):
            if phase == "polish":                              # 당김을 끄고 제약만 다듬는다
                for n in kinpos:
                    if n not in pose.pinned:
                        w[n] = 1.0
            for _ in range(iters):
                if phase == "pull":
                    for n, p in kinpos.items():                # 드래그 목표(가장 약한 힘)
                        pos[n][0], pos[n][1] = p[0], p[1]
                for r, th in fr:                               # 가장 낮은 우선순위
                    self._proj_angle(pos, w, r, th)
                for (a, b), L in s.keep.items():               # 현재 길이 유지
                    self._proj_eq(pos, w, a, b, L)
                for (src, o, c, k) in s.r_active:              # R
                    self._proj_r(pos, w, src, o, c, k)
                for (a, b, c, sg) in s.tris:                   # 삼각형 뒤집힘 방지
                    self._proj_tri(pos, w, a, b, c, sg)
                for (n1, n2, n3, tf) in s.a_rules:             # A: 노드 1은 선분 2-3 위
                    self._proj_a(pos, w, n1, n2, n3, tf)
                for (a, b), M in sk.max_len.items():           # 최대 길이
                    self._proj_max(pos, w, a, b, M)
                # 고정(pin): w=0 이므로 어떤 투영에서도 움직이지 않음
                if phase == "polish" and max(self._report(pos, fr).values()) < 1e-3:
                    break
        return pos, self._report(pos, fr)

    @staticmethod
    def _proj_eq(pos, w, a, b, L):
        dx, dy = pos[b][0] - pos[a][0], pos[b][1] - pos[a][1]
        d = math.hypot(dx, dy)
        sw = w[a] + w[b]
        if sw == 0:
            return
        if d < EPS:
            dx, dy, d = 1e-6, 0.0, 1e-6
        c = (d - L) / d
        pos[a][0] += w[a] / sw * c * dx
        pos[a][1] += w[a] / sw * c * dy
        pos[b][0] -= w[b] / sw * c * dx
        pos[b][1] -= w[b] / sw * c * dy

    @staticmethod
    def _a_residual(pos, n1, n2, n3, tf):
        (x1, y1), (x2, y2), (x3, y3) = pos[n1], pos[n2], pos[n3]
        dx, dy = x3 - x2, y3 - y2
        L2 = dx * dx + dy * dy
        if tf is None:
            t = 0.0 if L2 < EPS else max(0.0, min(1.0, ((x1 - x2) * dx + (y1 - y2) * dy) / L2))
        else:
            t = tf
        return t, (x1 - ((1 - t) * x2 + t * x3), y1 - ((1 - t) * y2 + t * y3))

    def _proj_a(self, pos, w, n1, n2, n3, tf):
        """일방향 제약: 노드 1만 선분 2-3 위(고정 시 고정 비율 t 위)로 옮긴다. 노드 2·3은 움직이지 않는다."""
        if w[n1] <= 0:
            return
        _t, r = self._a_residual(pos, n1, n2, n3, tf)
        pos[n1][0] -= r[0]
        pos[n1][1] -= r[1]

    @staticmethod
    def _proj_tri(pos, w, a, b, c, sg):
        """부호 있는 면적(2배)이 sg*cross >= |ab|*EPS_H 가 되도록 투영(PBD 면적 제약)"""
        ax, ay = pos[a]
        bx, by = pos[b]
        cx, cy = pos[c]
        d = math.hypot(bx - ax, by - ay)
        if d < EPS:
            return
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        C = sg * cross - d * EPS_H
        if C >= 0:
            return
        g = {a: (by - cy, cx - bx), b: (cy - ay, -(cx - ax)), c: (-(by - ay), bx - ax)}
        denom = sum(w[n] * (g[n][0] ** 2 + g[n][1] ** 2) for n in g)
        if denom < 1e-12:
            return
        lam = -C / denom
        for n in g:
            pos[n][0] += lam * w[n] * sg * g[n][0]
            pos[n][1] += lam * w[n] * sg * g[n][1]

    def _proj_max(self, pos, w, a, b, M):
        if dist(pos[a], pos[b]) > M:
            self._proj_eq(pos, w, a, b, M)

    @staticmethod
    def _proj_r(pos, w, src, o, c, k):
        if w[o] == 0:
            return
        pos[o][0] = pos[c][0] + k * (pos[c][0] - pos[src][0])   # 점대칭
        pos[o][1] = pos[c][1] + k * (pos[c][1] - pos[src][1])

    @staticmethod
    def _proj_angle(pos, w, r, th):
        n1, n2, n3 = r.n1, r.n2, r.n3
        v1 = (pos[n1][0] - pos[n2][0], pos[n1][1] - pos[n2][1])
        v3 = (pos[n3][0] - pos[n2][0], pos[n3][1] - pos[n2][1])
        cur = math.atan2(v1[0] * v3[1] - v1[1] * v3[0], v1[0] * v3[0] + v1[1] * v3[1])
        diff = wrap(th - cur)
        tw = w[n1] + w[n3]
        if tw == 0 or abs(diff) < 1e-9:
            return

        def rot(v, a):
            c, s_ = math.cos(a), math.sin(a)
            return (v[0] * c - v[1] * s_, v[0] * s_ + v[1] * c)

        r3 = rot(v3, diff * w[n3] / tw)
        r1 = rot(v1, -diff * w[n1] / tw)
        pos[n3][0], pos[n3][1] = pos[n2][0] + r3[0], pos[n2][1] + r3[1]
        pos[n1][0], pos[n1][1] = pos[n2][0] + r1[0], pos[n2][1] + r1[1]

    def _report(self, pos, fr):
        s, sk = self.sess, self.skel
        rep = {"max": 0.0, "keep": 0.0, "R": 0.0, "F": 0.0, "tri": 0.0, "A": 0.0}
        for (n1, n2, n3, tf) in s.a_rules:
            rep["A"] = max(rep["A"], math.hypot(*self._a_residual(pos, n1, n2, n3, tf)[1]))
        for (a, b, c, sg) in s.tris:
            d = dist(pos[a], pos[b])
            if d > EPS:
                cr = ((pos[b][0] - pos[a][0]) * (pos[c][1] - pos[a][1])
                      - (pos[b][1] - pos[a][1]) * (pos[c][0] - pos[a][0]))
                rep["tri"] = max(rep["tri"], EPS_H - sg * cr / d)
        for (a, b), M in sk.max_len.items():
            rep["max"] = max(rep["max"], dist(pos[a], pos[b]) - M)
        for (a, b), L in s.keep.items():
            rep["keep"] = max(rep["keep"], abs(dist(pos[a], pos[b]) - L))
        for (src, o, c, k) in s.r_active:
            want = (pos[c][0] + k * (pos[c][0] - pos[src][0]),
                    pos[c][1] + k * (pos[c][1] - pos[src][1]))
            rep["R"] = max(rep["R"], dist(pos[o], want))
        for r, th in fr:
            v1 = (pos[r.n1][0] - pos[r.n2][0], pos[r.n1][1] - pos[r.n2][1])
            v3 = (pos[r.n3][0] - pos[r.n2][0], pos[r.n3][1] - pos[r.n2][1])
            cur = math.atan2(v1[0] * v3[1] - v1[1] * v3[0], v1[0] * v3[0] + v1[1] * v3[1])
            rep["F"] = max(rep["F"], abs(wrap(th - cur)) * min(math.hypot(*v1), math.hypot(*v3)))
        return rep


# ----------------------------------------------------------------------
# 데모 구성 (어깨 R + 몸통 각도 F)
# ----------------------------------------------------------------------
def build_demo():
    e = PoseEditor()
    for nid, x, y in [(1, 240, 200), (2, 300, 200), (3, 360, 200), (4, 300, 300), (5, 300, 140)]:
        e.add_node(nid, x, y)
    for a, b in [(1, 2), (2, 3), (2, 4), (2, 5)]:
        e.add_line(a, b)
    e.add_rule("R", 1, 2, 3)      # 어깨: 노드 2 중심 점대칭
    e.add_rule("F", 1, 2, 4)      # 왼쪽 어깨-몸통 각도 고정
    e.add_rule("F", 3, 2, 4)      # 오른쪽 어깨-몸통 각도 고정
    return e


# ----------------------------------------------------------------------
# UI (PyQt6)  - 좌클릭 드래그: 노드/선 이동, Alt: 자유 이동, 우클릭: 고정 토글,
#               Ctrl+Z / Ctrl+Y: Undo / Redo
# ----------------------------------------------------------------------
def run_ui():
    import sys
    from PyQt6.QtCore import Qt
    from PyQt6.QtGui import QBrush, QColor, QPainter, QPen
    from PyQt6.QtWidgets import QApplication, QWidget

    class Canvas(QWidget):
        def __init__(self, editor):
            super().__init__()
            self.e = editor
            self.dragging = False
            self.setMinimumSize(640, 480)
            self.setWindowTitle("Pose Editor CL-3.13")

        def _hit_node(self, x, y):
            for n, p in self.e.pose.pos.items():
                if math.hypot(p[0] - x, p[1] - y) <= 10:
                    return n
            return None

        def _hit_link(self, x, y):
            for (a, b) in self.e.skel.max_len:
                pa, pb = self.e.pose.pos[a], self.e.pose.pos[b]
                dx, dy = pb[0] - pa[0], pb[1] - pa[1]
                L2 = dx * dx + dy * dy
                if L2 < EPS:
                    continue
                t = max(0.0, min(1.0, ((x - pa[0]) * dx + (y - pa[1]) * dy) / L2))
                if math.hypot(pa[0] + t * dx - x, pa[1] + t * dy - y) <= 6:
                    return (a, b)
            return None

        def paintEvent(self, _ev):
            qp = QPainter(self)
            qp.setRenderHint(QPainter.RenderHint.Antialiasing)
            qp.fillRect(self.rect(), QColor("white"))
            qp.setPen(QPen(QColor("#555"), 3))
            for (a, b) in self.e.skel.max_len:
                pa, pb = self.e.pose.pos[a], self.e.pose.pos[b]
                qp.drawLine(int(pa[0]), int(pa[1]), int(pb[0]), int(pb[1]))
            for n, p in self.e.pose.pos.items():
                pinned = n in self.e.pose.pinned
                qp.setPen(QPen(QColor("#222"), 2))
                qp.setBrush(QBrush(QColor("#e55" if pinned else "#6ab")))
                qp.drawEllipse(int(p[0]) - 8, int(p[1]) - 8, 16, 16)
            qp.setPen(QColor("#333"))
            qp.drawText(10, 20, "좌클릭 드래그: 이동 | Alt: 자유 이동 | 우클릭: 고정 | Ctrl+Z/Y: Undo/Redo")

        def mousePressEvent(self, ev):
            x, y = ev.position().x(), ev.position().y()
            alt = bool(ev.modifiers() & Qt.KeyboardModifier.AltModifier)
            n = self._hit_node(x, y)
            if ev.button() == Qt.MouseButton.RightButton:
                if n is not None:
                    self.e.toggle_pin(n)
            elif ev.button() == Qt.MouseButton.LeftButton:
                if n is not None:
                    self.dragging = self.e.begin_drag_node(n, alt)
                else:
                    lk = self._hit_link(x, y)
                    if lk:
                        self.dragging = self.e.begin_drag_link(lk[0], lk[1], (x, y), alt)
            self.update()

        def mouseMoveEvent(self, ev):
            if self.dragging:
                alt = bool(ev.modifiers() & Qt.KeyboardModifier.AltModifier)
                self.e.drag_move((ev.position().x(), ev.position().y()), alt)
                self.update()

        def mouseReleaseEvent(self, _ev):
            if self.dragging:
                self.e.end_drag()
                self.dragging = False
                self.update()

        def keyPressEvent(self, ev):
            if ev.modifiers() & Qt.KeyboardModifier.ControlModifier:
                if ev.key() == Qt.Key.Key_Z:
                    self.e.undo()
                elif ev.key() == Qt.Key.Key_Y:
                    self.e.redo()
                self.update()

    app = QApplication(sys.argv)
    w = Canvas(build_demo())
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run_ui()
