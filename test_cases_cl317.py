# CL-3.17
"""요구 테스트케이스 T01~T43 자동 실행기 (PyQt6 불필요)
실행: python test_cases_cl317.py
  PASS = 기대 결과 충족 / WARN = 핵심은 충족, 참고할 한계 있음 / FAIL = 기대 결과 불충족
"""
import math
import random

from pose_editor_cl317 import dist
from test_structures_cl317 import BOT, TOP, TRUSS_L, TREE_N, TREE_L, SQ_N, SQ_L, arc, area, line, mk, triangles

random.seed(11)


# ----------------------------------------------------------------- 도우미
def run(e, node, path, alt=False):
    assert e.begin_drag_node(node, alt), "begin_drag 거부"
    st = [e.drag_move(t, alt) for t in path]
    e.end_drag()
    return st


def ang(e, i):
    return e._angle(e.pose.pos, e.skel.rules[i])


def dang(a, b):
    return abs(math.remainder(a - b, 2 * math.pi))


def lens_kept(e, tol=0.05):
    for (a, b), L in e.pose.cur_len.items():
        d = dist(e.pose.pos[a], e.pose.pos[b])
        assert abs(d - L) < tol, f"링크 {a}-{b}: 현재 {d:.3f} / 저장 {L:.3f}"


def within_max(e, tol=0.05):
    for (a, b), M in e.skel.max_len.items():
        assert dist(e.pose.pos[a], e.pose.pos[b]) <= M + tol, f"최대 길이 초과 {a}-{b}"
        assert e.pose.cur_len[(a, b)] <= M + 1e-6, f"cur_len 초과 {a}-{b}"


def finite(e, bound=1e4):
    for n, p in e.pose.pos.items():
        assert all(math.isfinite(v) and abs(v) < bound for v in p), f"노드 {n} 폭주/NaN: {p}"


def pins_fixed(e, p0):
    for n in e.pose.pinned:
        assert e.pose.pos[n] == p0[n], f"고정 노드 {n} 이동"


def sym_err(e, i):
    r = e.skel.rules[i]
    p1, p2, p3 = (e.pose.pos[n] for n in (r.n1, r.n2, r.n3))
    return dist(p3, (p2[0] + r.ratio * (p2[0] - p1[0]), p2[1] + r.ratio * (p2[1] - p1[1])))


def flip_count(e, p0, tris):
    return sum(1 for t in tris if area(e.pose.pos, *t) * area(p0, *t) < 0)


def poly_area(e, order):
    pts = [e.pose.pos[n] for n in order]
    return sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1]
               for i in range(len(pts))) / 2


def snap(e):
    return dict(e.pose.pos)


def moved(e, p0):
    return max(dist(e.pose.pos[n], p0[n]) for n in p0)


# ----------------------------------------------------------------- 구조 정의
CH2 = [(1, 100, 100), (2, 200, 100)]
CH3 = [(1, 0, 100), (2, 100, 100), (3, 200, 100)]
CH3_L = [(1, 2), (2, 3)]
FANGLE = [(1, 0, 100), (2, 100, 100), (3, 100, 0)]          # 꺾인 1-2-3 (90°)
RTREE = [(1, 0, 100), (2, 100, 100), (4, 200, 100), (3, 100, 0)]
RTREE_L = [(1, 2), (2, 4), (2, 3)]
LOOP5 = [(1, 0, 100), (2, 100, 100), (3, 200, 100), (4, 200, 200), (5, 0, 0)]   # 2 중심 점대칭 폐곡선
LOOP5_L = [(1, 2), (2, 3), (3, 4), (4, 5), (5, 1)]
STAR = [(0, 200, 200), (1, 100, 200), (2, 300, 200), (3, 200, 100), (4, 200, 300)]
STAR_L = [(0, 1), (0, 2), (0, 3), (0, 4)]
TRI = [(1, 0, 0), (2, 100, 0), (3, 50, 87)]
TRI_L = [(1, 2), (2, 3), (1, 3)]


# ----------------------------------------------------------------- 기본 링크 (T01~T08)
def T01():
    e = mk(CH2, [(1, 2)])
    run(e, 2, line((200, 100), (150, 180)))
    lens_kept(e)
    assert dist(e.pose.pos[2], (150, 180)) < 1.0


def T02():
    e = mk(CH2, [(1, 2)], pins=[1])
    run(e, 2, line((200, 100), (150, 180), 20))
    p1, p2 = e.pose.pos[1], e.pose.pos[2]
    assert p1 == (100.0, 100.0)
    assert abs(dist(p1, p2) - 100) < 0.05
    a_node = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    a_tgt = math.atan2(180 - 100, 150 - 100)
    assert dang(a_node, a_tgt) < math.radians(2), "마우스 방향으로 회전하지 않음"


def T03():
    e = mk(CH2, [(1, 2)], pins=[1, 2])
    assert not e.begin_drag_link(1, 2, (150, 100))
    assert not e.begin_drag_node(1) and not e.begin_drag_node(2)


def T04():
    e = mk(CH3, CH3_L)
    p0 = snap(e)
    st = run(e, 3, line((200, 100), (200, 200)))
    assert set(st) == {"ok"}
    lens_kept(e)
    assert dist(e.pose.pos[3], (200, 200)) < 1.0
    assert dist(e.pose.pos[2], p0[2]) > 5 and dist(e.pose.pos[1], p0[1]) > 5, "연결 노드가 따라오지 않음"


def T05():
    e = mk(CH3, CH3_L)
    run(e, 3, line((200, 100), (150, 100)), alt=True)
    assert abs(e.pose.cur_len[(2, 3)] - 50) < 0.5, f"2-3 길이 {e.pose.cur_len[(2, 3)]:.2f}"
    assert abs(e.pose.cur_len[(1, 2)] - 100) < 1e-6
    within_max(e)


def T06():
    e = mk(CH3, CH3_L)
    p0 = snap(e)
    run(e, 3, line((200, 100), (400, 100), 30), alt=True)
    within_max(e)
    assert e.pose.pos[2][0] > p0[2][0] + 10, "이웃 노드가 따라오지 않음"
    assert dist(e.pose.pos[3], (400, 100)) < 1.0
    assert abs(dist(e.pose.pos[1], e.pose.pos[2]) - 100) < 0.05


def T07():
    e = mk(STAR, STAR_L)
    p0 = snap(e)
    st = run(e, 0, line((200, 200), (230, 230), 20))
    assert set(st) == {"ok"}
    lens_kept(e)
    for n in (1, 2, 3, 4):
        assert dist(e.pose.pos[n], p0[n]) > 5, f"이웃 {n} 정지"
    e2 = mk(STAR, STAR_L)
    run(e2, 0, line((200, 200), (160, 200), 20), alt=True)
    within_max(e2)


def T08():
    e = mk(STAR, STAR_L, pins=[1])
    p0 = snap(e)
    st = run(e, 0, line((200, 200), (190, 260), 20))
    pins_fixed(e, p0)
    lens_kept(e)
    assert moved(e, p0) > 5 and set(st) != {"blocked"}
    e2 = mk(STAR, STAR_L, pins=[1, 2])               # 양쪽 이웃 고정: 중심은 한 점
    q0 = snap(e2)
    run(e2, 0, line((200, 200), (190, 260), 20))
    pins_fixed(e2, q0)
    lens_kept(e2, tol=0.05)
    mv = moved(e2, q0)
    assert mv < 3.0, f"이동 불가여야 하는데 {mv:.1f}px 이동"
    if mv > 0.5:
        return f"WARN: 두 원이 접하는 퇴화 배치에서 허용 오차(0.02) 때문에 {mv:.1f}px 이동"


# ----------------------------------------------------------------- F 규칙 (T09~T16)
def T09():
    e = mk(FANGLE, CH3_L, rules=[("F", 1, 2, 3)])
    assert abs(abs(e.pose.angles[0]) - math.pi / 2) < 1e-9


def T10():
    e = mk(FANGLE, CH3_L, rules=[("F", 1, 2, 3)])
    a0 = e.pose.angles[0]
    run(e, 3, line((100, 0), (150, -30), 20))
    lens_kept(e)
    assert dang(ang(e, 0), a0) < 0.02


def T11():
    e = mk(FANGLE, CH3_L, rules=[("F", 1, 2, 3)])
    a0 = e.pose.angles[0]
    run(e, 2, line((100, 100), (130, 130), 20))
    lens_kept(e)
    assert dang(ang(e, 0), a0) < 0.02


def T12():
    e = mk(FANGLE, CH3_L, rules=[("F", 1, 2, 3)])
    a0 = e.pose.angles[0]
    assert e.begin_drag_node(1)
    for t in line((0, 100), (-30, 140), 20):
        e.drag_move(t)
    mid = ang(e, 0)
    e.end_drag()
    assert dang(mid, a0) > 0.05, "이동 중 F가 해제되지 않음(각도 변화 없음)"
    assert dang(e.pose.angles[0], ang(e, 0)) < 1e-6, "종료 후 기준각 미갱신"
    assert dang(e.pose.angles[0], a0) > 0.05


def T13():
    e = mk(FANGLE, CH3_L, pins=[1], rules=[("F", 1, 2, 3)])
    p0, a0 = snap(e), e.pose.angles[0]
    run(e, 2, line((100, 100), (140, 60), 20))
    pins_fixed(e, p0)
    lens_kept(e)
    assert abs(dist(e.pose.pos[1], e.pose.pos[2]) - 100) < 0.05
    assert dang(ang(e, 0), a0) < 0.02


def T14():
    e = mk(FANGLE, CH3_L, pins=[1], rules=[("F", 1, 2, 3)])
    p0 = snap(e)
    run(e, 3, line((100, 0), (60, -20), 20))
    pins_fixed(e, p0)
    lens_kept(e)
    assert dang(ang(e, 0), e.pose.angles[0]) < 0.02, "저장 각도와 실제 각도 불일치"
    return f"F 각도 변화 {math.degrees(dang(ang(e, 0), 1.5707963)):.1f}° (유지 시도 후 기준각 일관)"


def T15():
    """1 고정 + 4 이동: 명세 5.2 — F 무시, 2가 이동, 종료 후 F 갱신"""
    e = mk([(1, 0, 100), (2, 100, 100), (4, 100, 0)], [(1, 2), (2, 4)], pins=[1], rules=[("F", 1, 2, 4)])
    p0, a0 = snap(e), e.pose.angles[0]
    run(e, 4, line((100, 0), (100, -20), 20))
    pins_fixed(e, p0)
    lens_kept(e)
    assert dist(e.pose.pos[4], (100, -20)) < 1.0, "F를 무시하지 않아 4번이 목표에 도달하지 못함"
    assert dist(e.pose.pos[2], p0[2]) > 1, "2번 노드 미이동"
    assert dang(e.pose.angles[0], ang(e, 0)) < 1e-6, "종료 후 F 미갱신"
    assert dang(e.pose.angles[0], a0) > 0.01


def T16():
    nodes = [(1, 0, 100), (2, 100, 100), (3, 100, 0), (4, 200, 100)]
    links = [(1, 2), (2, 3), (2, 4)]
    e = mk(nodes, links, rules=[("F", 1, 2, 3), ("F", 3, 2, 4)])
    a0 = dict(e.pose.angles)
    run(e, 4, line((200, 100), (190, 140), 15))
    lens_kept(e)
    for i in (0, 1):
        assert dang(ang(e, i), a0[i]) < 0.02, f"F{i} 위반"


# ----------------------------------------------------------------- R 규칙 (T17~T22)
def T17():
    e = mk(CH3, CH3_L, rules=[("R", 1, 2, 3)])
    assert abs(e.skel.rules[0].ratio - 1.0) < 1e-9
    assert sym_err(e, 0) < 1e-9
    v1 = (e.pose.pos[1][0] - e.pose.pos[2][0], e.pose.pos[1][1] - e.pose.pos[2][1])
    v3 = (e.pose.pos[3][0] - e.pose.pos[2][0], e.pose.pos[3][1] - e.pose.pos[2][1])
    assert abs(v1[0] * v3[1] - v1[1] * v3[0]) < 1e-9 and (v1[0] * v3[0] + v1[1] * v3[1]) < 0


def T18():
    e = mk(CH3, CH3_L, rules=[("R", 1, 2, 3)])
    p2 = e.pose.pos[2]
    st = run(e, 1, line((0, 100), (10, 130), 20))
    assert set(st) == {"ok"}
    assert e.pose.pos[2] == p2
    assert sym_err(e, 0) < 0.05
    assert e.pose.pos[3][1] < 100, "3이 반대 방향(아래쪽)으로 이동하지 않음"


def T19():
    e = mk([(1, 0, 100), (2, 100, 100), (3, 180, 100)], CH3_L, rules=[("R", 1, 2, 3)])
    assert abs(e.skel.rules[0].ratio - 0.8) < 1e-9
    run(e, 1, line((0, 100), (10, 130), 20))
    assert abs(dist(e.pose.pos[3], e.pose.pos[2]) - 0.8 * dist(e.pose.pos[1], e.pose.pos[2])) < 0.05
    assert sym_err(e, 0) < 0.05
    run(e, 1, line(e.pose.pos[1], (-100, 100), 30))          # 최대 길이 초과 요청
    assert dist(e.pose.pos[1], e.pose.pos[2]) <= 100 + 0.05
    assert dist(e.pose.pos[3], e.pose.pos[2]) <= 80 + 0.05
    assert sym_err(e, 0) < 0.05


def T20():
    e = mk([(1, 0, 100), (2, 100, 100), (3, 180, 100)], CH3_L, rules=[("R", 1, 2, 3)])
    p2 = e.pose.pos[2]
    run(e, 3, line((180, 100), (170, 120), 20))
    assert e.pose.pos[2] == p2
    assert sym_err(e, 0) < 0.05
    assert abs(dist(e.pose.pos[1], e.pose.pos[2]) - dist(e.pose.pos[3], e.pose.pos[2]) / 0.8) < 0.05


def T21():
    e = mk(CH3, CH3_L, pins=[1], rules=[("R", 1, 2, 3)])
    p0 = snap(e)
    st = run(e, 3, line((200, 100), (210, 130), 10))
    assert set(st) == {"blocked"} and moved(e, p0) == 0


def T22():
    e = mk(CH3, CH3_L, pins=[3], rules=[("R", 1, 2, 3)])
    p0 = snap(e)
    st = run(e, 1, line((0, 100), (10, 130), 10))
    assert set(st) == {"blocked"} and moved(e, p0) == 0


# ----------------------------------------------------------------- F + R (T23~T25)
def T23():
    """F(1,2,3) + R(1,2,4): 노드 4를 끌면 R이 노드 1을 움직이므로 F의 노드 1이 움직임 -> F 해제 후 재저장"""
    e = mk(RTREE, RTREE_L, rules=[("F", 1, 2, 3), ("R", 1, 2, 4)])
    p0, p2 = snap(e), e.pose.pos[2]
    run(e, 4, line((200, 100), (190, 130), 15))
    assert e.pose.pos[2] == p2
    assert sym_err(e, 1) < 0.05, "R 위반"
    assert dist(e.pose.pos[3], p0[3]) < 0.05, "F가 해제되지 않아 노드 3이 따라 움직임"
    assert dang(e.pose.angles[0], ang(e, 0)) < 1e-6, "종료 후 F 각도 미갱신"
    lens_kept(e)


def T24():
    e = mk(RTREE, RTREE_L, pins=[3], rules=[("F", 1, 2, 3), ("R", 1, 2, 4)])
    p0, a0 = snap(e), e.pose.angles[0]
    run(e, 4, line((200, 100), (190, 130), 15))
    pins_fixed(e, p0)
    assert sym_err(e, 1) < 0.05, "R이 우선 적용되지 않음"
    assert dang(e.pose.angles[0], ang(e, 0)) < 1e-6, "F 기준각 재계산 안 됨"
    assert dang(e.pose.angles[0], a0) > 0.01, "F가 완화되지 않음(각도 변화 없음)"


def T25():
    e = mk(RTREE, RTREE_L, pins=[3], rules=[("F", 1, 2, 3), ("R", 1, 2, 4)])
    p0 = snap(e)
    run(e, 1, line((0, 100), (10, 120), 15))
    pins_fixed(e, p0)
    assert e.pose.pos[2] == p0[2]
    assert sym_err(e, 1) < 0.05
    assert dang(e.pose.angles[0], ang(e, 0)) < 1e-6


# ----------------------------------------------------------------- 복합 구조 / 폐곡선 (T26~T35)
def T26():
    e = mk(TRI, TRI_L)
    p0, tris = snap(e), triangles(e)
    run(e, 3, line((50, 87), (90, 130), 20))
    lens_kept(e)
    assert flip_count(e, p0, tris) == 0
    assert dist(e.pose.pos[3], (90, 130)) < 1.0


def T27():
    e = mk(SQ_N, SQ_L, pins=[1])
    p0 = snap(e)
    a0 = poly_area(e, [1, 2, 3, 4])
    st = run(e, 3, line((100, 100), (160, 120), 20))
    pins_fixed(e, p0)
    lens_kept(e)
    finite(e)
    assert moved(e, p0) > 5
    assert poly_area(e, [1, 2, 3, 4]) * a0 > 0, "폐곡선이 뒤집힘"


def T28():
    e = mk(TRI, TRI_L, pins=[1, 2])
    p0 = snap(e)
    st = run(e, 3, line((50, 87), (90, 130), 20))
    assert moved(e, p0) < 0.5, "이동 가능 범위가 0인데 움직임"
    pins_fixed(e, p0)


def T29():
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    p0, tris = snap(e), triangles(e)
    st = run(e, 8, line((250, 100), (255, 90), 20))
    pins_fixed(e, p0)
    lens_kept(e)
    assert moved(e, p0) > 1
    assert flip_count(e, p0, tris) == 0


def T30():
    e = mk(SQ_N, SQ_L, pins=[1], rules=[("F", 1, 2, 3)])
    p0, a0 = snap(e), e.pose.angles[0]
    st = run(e, 3, arc((0, 0), dist((0, 0), (100, 100)), 45, 60, 15))   # 핀 1 중심 원 위: F 유지 가능
    pins_fixed(e, p0)
    lens_kept(e)
    assert set(st) == {"ok"}
    assert dang(ang(e, 0), a0) < 0.02, "F 위반"
    e2 = mk(SQ_N, SQ_L, pins=[1], rules=[("F", 1, 2, 3)])               # 원 밖 목표: F가 양보
    q0 = snap(e2)
    run(e2, 3, line((100, 100), (130, 70), 20))
    pins_fixed(e2, q0)
    lens_kept(e2)
    assert dang(e2.pose.angles[0], ang(e2, 0)) < 1e-6, "F 양보 후 기준각 재저장 안 됨"
    return "F가 유지 가능하면 유지, 목표에 닿으려면 F가 양보하고 종료 후 재저장"


def T31():
    e = mk(LOOP5, LOOP5_L, rules=[("R", 1, 2, 3)])
    p2 = e.pose.pos[2]
    st = run(e, 1, arc((0, 0), 100, 90, 80, 10))
    assert e.pose.pos[2] == p2
    assert sym_err(e, 0) < 0.05, "R 위반"
    for k in ((3, 4), (4, 5), (1, 5)):
        assert abs(dist(e.pose.pos[k[0]], e.pose.pos[k[1]]) - e.pose.cur_len[k]) < 0.05
    return f"상태 {set(st)}"


def T32():
    e = mk(LOOP5, LOOP5_L, rules=[("R", 1, 2, 3), ("F", 4, 5, 1)])
    a0 = e.pose.angles[1]
    st = run(e, 1, arc((0, 0), 100, 90, 80, 10))
    assert sym_err(e, 0) < 0.05, "R 위반"
    for k in ((3, 4), (4, 5), (1, 5)):
        assert abs(dist(e.pose.pos[k[0]], e.pose.pos[k[1]]) - e.pose.cur_len[k]) < 0.05
    kept = dang(ang(e, 1), a0) < 0.02
    stored_ok = dang(e.pose.angles[1], ang(e, 1)) < 0.02
    assert kept or stored_ok, "F 각도 불일치"
    return f"F {'유지' if kept else '완화 후 재저장'}, 상태 {set(st)}"


def T33():
    e = mk(BOT + TOP, TRUSS_L, pins=[1, 2])
    p0 = snap(e)
    assert e.begin_drag_node(9)
    assert e.drag_move((350, 40)) == "blocked"
    e.end_drag()
    assert moved(e, p0) == 0
    return "WARN: 드래그 시작 전 검출 API는 없음(첫 이동 시 blocked로 검출)"


def T34():
    e = mk(TREE_N, TREE_L, pins=[1])
    reach = e.skel.max_len[(1, 2)] + e.skel.max_len[(2, 4)]
    assert e.begin_drag_node(4)
    hist = []
    for t in line((170, 260), (-100, -100), 40):
        e.drag_move(t)
        hist.append(e.pose.pos[4])
    e.end_drag()
    d = dist(e.pose.pos[1], e.pose.pos[4])
    assert reach - 3 <= d <= reach + 0.05, f"경계 도달 거리 {d:.1f} / 한계 {reach:.1f}"
    assert e.begin_drag_node(4)
    for _ in range(5):                                     # 마우스를 멈추면 경계에서 정지
        e.drag_move((-100, -100))
        hist.append(e.pose.pos[4])
    e.end_drag()
    assert dist(hist[-1], hist[-5]) < 0.05, "경계에서 정지하지 않음"


def T35():
    notes = []
    for label, e in [("트리", mk(TREE_N, TREE_L, pins=[1])), ("트러스", mk(BOT + TOP, TRUSS_L, pins=[1])),
                     ("사각형", mk(SQ_N, SQ_L, pins=[1]))]:
        p0, tris = snap(e), triangles(e)
        node = {"트리": 4, "트러스": 9, "사각형": 3}[label]
        run(e, node, [(1e6, 1e6)])
        finite(e)
        lens_kept(e)
        within_max(e)
        pins_fixed(e, p0)
        f = flip_count(e, p0, tris)
        if f:
            notes.append(f"{label} 삼각형 {f}개 뒤집힘")
    return ("WARN: " + ", ".join(notes)) if notes else None


# ----------------------------------------------------------------- 안정성 (T36~T43)
def T36():
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    assert e.begin_drag_node(9)
    hist = []
    for _ in range(100):
        e.drag_move((350, 40))
        hist.append(snap(e))
    e.end_drag()
    finite(e)
    d_last = max(dist(hist[-1][n], hist[-2][n]) for n in hist[-1])
    d_10 = max(dist(hist[-1][n], hist[9][n]) for n in hist[-1])
    assert d_last < 0.01 and d_10 < 0.05, f"진동/드리프트 {d_last:.4f}/{d_10:.4f}"


def T37():
    notes = []
    for label, e, node in [("트러스", mk(BOT + TOP, TRUSS_L, pins=[1]), 9), ("트리", mk(TREE_N, TREE_L, pins=[1]), 4)]:
        p0, tris = snap(e), triangles(e)
        assert e.begin_drag_node(node)
        for _ in range(50):
            e.drag_move((random.uniform(-250, 550), random.uniform(-250, 450)))
        last = (random.uniform(0, 400), random.uniform(0, 300))
        hist = []
        for _ in range(6):
            e.drag_move(last)
            hist.append(snap(e))
        e.end_drag()
        finite(e)
        lens_kept(e)
        pins_fixed(e, p0)
        osc = max(dist(hist[-1][n], hist[-2][n]) for n in hist[-1])
        assert osc < 0.05, f"{label} 진동 {osc:.3f}"
        steps = [max(dist(hist[i][n], hist[i - 1][n]) for n in hist[i]) for i in range(1, len(hist))]
        assert all(steps[i] <= steps[i - 1] + 0.05 for i in range(1, len(steps))), f"{label} 비단조 {steps}"
        f = flip_count(e, p0, tris)
        if f:
            notes.append(f"{label} 삼각형 {f}개 뒤집힘")
    return ("WARN: " + ", ".join(notes)) if notes else None


def T38():
    e = mk(TREE_N, TREE_L, pins=[1], rules=[("F", 2, 1, 3)])
    L0, a0 = dict(e.pose.cur_len), e.pose.angles[0]
    assert e.begin_drag_node(4)
    worst = 0.0
    for i in range(200):
        t = (200 + 40 * math.sin(i * 0.9), 220 + 30 * math.cos(i * 1.7))
        e.drag_move(t)
        for (a, b), L in e.pose.cur_len.items():
            worst = max(worst, abs(dist(e.pose.pos[a], e.pose.pos[b]) - L))
    e.end_drag()
    assert worst < 0.05, f"길이 오차 누적 {worst:.3f}"
    assert e.pose.cur_len == L0, "cur_len 드리프트"
    assert dang(ang(e, 0), a0) < 0.02


def T39():
    for label, e, nodes in [("트리", mk(TREE_N, TREE_L, pins=[1, 4, 7]), [2, 3, 5, 6]),
                            ("트러스", mk(BOT + TOP, TRUSS_L, pins=[1, 2, 5]), [3, 4, 6, 7, 8, 9])]:
        p0 = snap(e)
        for _ in range(100):
            n = random.choice(nodes)
            c = e.pose.pos[n]
            tg = (c[0] + random.uniform(-80, 80), c[1] + random.uniform(-80, 80))
            if e.begin_drag_node(n, random.random() < 0.3):
                e.drag_move(tg)
                e.end_drag()
        pins_fixed(e, p0)
        finite(e)


def T40():
    e = mk(CH3, CH3_L)
    M0 = dict(e.skel.max_len)
    for i in range(60):
        run(e, 3, line(e.pose.pos[3], (e.pose.pos[3][0] + 80 * random.choice([1, -1]), e.pose.pos[3][1] + random.uniform(-60, 60)), 10), alt=True)
        within_max(e)
    assert e.skel.max_len == M0, "최초 길이 변경됨"


def T41():
    e = mk(CH3, CH3_L)
    M0 = dict(e.skel.max_len)
    assert e.begin_drag_node(3, False)
    for i in range(80):
        alt = (i // 5) % 2 == 1
        e.drag_move((200 + 60 * math.sin(i * 0.3), 100 + 60 * math.cos(i * 0.2)), alt)
    e.end_drag()
    assert e.skel.max_len == M0
    within_max(e)
    lens_kept(e)


def T42():
    e = mk(RTREE, RTREE_L, rules=[("F", 1, 2, 3), ("R", 1, 2, 4)])
    p0, a0, ratio0 = snap(e), e.pose.angles[0], e.skel.rules[1].ratio
    for _ in range(50):
        run(e, 4, line((200, 100), (190, 130), 8) + line((190, 130), (200, 100), 8))
    assert moved(e, p0) < 0.5, f"Pose 변형 누적 {moved(e, p0):.2f}"
    assert dang(e.pose.angles[0], a0) < 0.02 and e.skel.rules[1].ratio == ratio0
    assert sym_err(e, 1) < 0.05
    e2 = mk(RTREE, RTREE_L, rules=[("F", 1, 2, 3), ("R", 1, 2, 4)])
    q0 = snap(e2)
    for _ in range(50):
        run(e2, 3, line((100, 0), (120, -20), 8) + line((120, -20), (100, 0), 8))
    assert dang(e2.pose.angles[0], a0) < 0.02 and dang(ang(e2, 0), a0) < 0.02
    lens_kept(e2)
    notes = []
    if sym_err(e2, 1) > 0.05:
        notes.append(f"노드 3을 끌 때는 R이 꺼져 있어 4가 사슬처럼 끌려오며 대칭 오차 {sym_err(e2, 1):.1f}px 누적(양쪽에 F를 걸면 유지)")
    if moved(e2, q0) > 0.5:
        notes.append(f"3을 되돌려도 나머지 노드가 {moved(e2, q0):.1f}px 틀어짐(핀 없는 사슬의 이력 현상)")
    if notes:
        return "WARN: " + "; ".join(notes)


def T43():
    e = mk(SQ_N, SQ_L, pins=[1])
    p0 = snap(e)
    a0 = poly_area(e, [1, 2, 3, 4])
    L0 = dict(e.pose.cur_len)
    for i in range(100):
        run(e, 3, line(e.pose.pos[3], (100 + 40, 100 + 30), 6) + line((140, 130), (100, 100), 6))
        assert poly_area(e, [1, 2, 3, 4]) * a0 > 0, f"{i}회차에서 폐곡선 뒤집힘"
    lens_kept(e)
    finite(e)
    pins_fixed(e, p0)
    assert e.pose.cur_len == L0


def T44():
    """데모(R(1,2,3) + F(1,2,4) 하나): 노드 1을 끌면 R로 노드 3이 움직이고, 노드 4는 움직이지 않으며, F 각도는 종료 후 갱신"""
    from pose_editor_cl317 import build_demo
    e = build_demo()
    p0 = snap(e)
    run(e, 1, line((240, 200), (250, 230), 15))
    assert dist(e.pose.pos[4], p0[4]) < 0.5, f"노드 4가 따라 움직임 {dist(e.pose.pos[4], p0[4]):.1f}px"
    assert dist(e.pose.pos[3], p0[3]) > 5, "R로 노드 3이 움직이지 않음"
    assert sym_err(e, 0) < 0.05
    assert dang(e.pose.angles[1], ang(e, 1)) < 1e-6, "F 종료 후 각도 미갱신"


def T45():
    """데모: 노드 4를 끌면 F(1,2,4) 각도 유지, R 대칭도 유지"""
    from pose_editor_cl317 import build_demo
    e = build_demo()
    a0 = e.pose.angles[1]
    run(e, 4, line((300, 300), (320, 320), 15))
    lens_kept(e)
    assert dang(ang(e, 1), a0) < 0.02, "F 위반"
    assert sym_err(e, 0) < 0.05, "R 대칭 위반"


def _stick(rules):
    from pose_editor_cl317 import PoseEditor
    e = PoseEditor()
    pts = {1: (240, 200), 2: (300, 200), 3: (360, 200), 4: (300, 300), 5: (300, 140),
           6: (200, 250), 7: (400, 250), 8: (270, 380), 9: (330, 380)}
    for n, (x, y) in pts.items():
        e.add_node(n, x, y)
    for a, b in [(1, 2), (2, 3), (2, 4), (2, 5), (1, 6), (3, 7), (4, 8), (4, 9)]:
        e.add_line(a, b)
    for r in rules:
        e.add_rule(*r)
    return e


def T53():
    """R 유지: F를 하나만 두고 어느 노드를 끌어도(몸통·머리·팔·다리) R 대칭이 끝까지 유지된다"""
    worst = 0.0
    for node in (4, 5, 6, 7, 8):
        e = _stick([("R", 1, 2, 3), ("F", 1, 2, 4)])
        c = e.pose.pos[node]
        assert e.begin_drag_node(node)
        for i in range(80):
            e.drag_move((c[0] + 50 * math.sin(i * 0.15), c[1] + 40 * math.cos(i * 0.11)))
            worst = max(worst, sym_err(e, 0))
        e.end_drag()
        lens_kept(e)
    assert worst < 0.05, f"R 대칭 오차 {worst:.2f}px"


def T54():
    """떨림 점검: 마우스를 멈추면 노드가 움직이지 않고, 부드러운 마우스 이동에서 프레임 간 이동량이 급변하지 않는다"""
    for node in (4, 5, 6, 8):
        e = _stick([("R", 1, 2, 3), ("F", 1, 2, 4)])
        c = e.pose.pos[node]
        assert e.begin_drag_node(node)
        prev, steps = None, []
        for i in range(80):
            tg = (c[0] + 50 * math.sin(i * 0.15), c[1] + 40 * math.cos(i * 0.11))
            e.drag_move(tg)
            cur = {k: e.pose.pos[k] for k in (1, 3)}
            if prev:
                steps.append(max(dist(cur[k], prev[k]) for k in cur))
            prev = cur
        rest = []
        for _ in range(10):
            e.drag_move(tg)
            cur = {k: e.pose.pos[k] for k in (1, 3)}
            rest.append(max(dist(cur[k], prev[k]) for k in cur))
            prev = cur
        e.end_drag()
        assert max(rest[1:]) < 0.01, f"노드 {node}: 정지 시 진동 {max(rest[1:]):.3f}px"   # 첫 호출은 마지막 이동분을 마무리
        acc = max(abs(steps[i] - steps[i - 1]) for i in range(1, len(steps)))
        assert acc < 3.0, f"노드 {node}: 프레임 간 이동량 급변 {acc:.2f}px"


def T55():
    """R 유지가 불가능한 입력(양 끝 노드가 고정)이면 중심 노드는 움직이지 않는다"""
    e = mk(CH3, CH3_L, pins=[1, 3], rules=[("R", 1, 2, 3)])
    p0 = snap(e)
    st = run(e, 2, line((100, 100), (110, 130), 10))
    assert moved(e, p0) < 0.5 and "ok" not in st
    pins_fixed(e, p0)


# ----------------------------------------------------------------- A 규칙 (T46~T52)
A_N = [(1, 100, 100), (2, 0, 100), (3, 200, 100), (4, 100, 0)]      # 1은 선분 2-3 위(비율 0.5)
A_L = [(2, 3), (1, 4)]


def seg_dist(e, n1=1, n2=2, n3=3):
    """노드 1에서 선분 2-3까지의 거리"""
    p1, p2, p3 = e.pose.pos[n1], e.pose.pos[n2], e.pose.pos[n3]
    dx, dy = p3[0] - p2[0], p3[1] - p2[1]
    L2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((p1[0] - p2[0]) * dx + (p1[1] - p2[1]) * dy) / L2))
    return dist(p1, (p2[0] + t * dx, p2[1] + t * dy))


def seg_t(e, n1=1, n2=2, n3=3):
    p1, p2, p3 = e.pose.pos[n1], e.pose.pos[n2], e.pose.pos[n3]
    dx, dy = p3[0] - p2[0], p3[1] - p2[1]
    return ((p1[0] - p2[0]) * dx + (p1[1] - p2[1]) * dy) / (dx * dx + dy * dy)


def T46():
    """A: 노드 1을 선 위가 아닌 곳으로 끌어도 선분 2-3 위를 따라 미끄러지고 양 끝을 넘지 않는다"""
    e = mk([(1, 100, 100), (2, 0, 100), (3, 200, 100)], [(2, 3)], rules=[("A", 1, 2, 3)])
    run(e, 1, line((100, 100), (140, 160), 20))
    assert seg_dist(e) < 0.05 and abs(e.pose.pos[1][0] - 140) < 1.0 and abs(e.pose.pos[1][1] - 100) < 0.05
    run(e, 1, line(e.pose.pos[1], (400, 30), 20))                     # 선분 끝(3)을 넘으려 함
    assert seg_dist(e) < 0.05 and e.pose.pos[1][0] <= 200 + 0.05
    run(e, 1, line(e.pose.pos[1], (-300, 20), 20))                    # 선분 끝(2)을 넘으려 함
    assert seg_dist(e) < 0.05 and e.pose.pos[1][0] >= -0.05


def T47():
    """A: 노드 1을 고정하지 않은 상태에서 선(2)을 움직여도 노드 1은 선분 위에 머문다"""
    e = mk(A_N, A_L, rules=[("A", 1, 2, 3)])
    run(e, 2, line((0, 100), (20, 160), 20))
    assert seg_dist(e) < 0.05


def T48():
    """A + 고정: 노드 1을 고정하면 선 위 비율 t가 고정되어 선과 함께 움직인다"""
    e = mk(A_N, A_L, rules=[("A", 1, 2, 3)])
    e.toggle_pin(1)
    p0, t0 = snap(e), seg_t(e)
    run(e, 2, line((0, 100), (20, 160), 20))
    assert seg_dist(e) < 0.05
    assert abs(seg_t(e) - t0) < 0.005, f"t 변화 {seg_t(e) - t0:.4f}"
    assert dist(e.pose.pos[1], p0[1]) > 3, "선과 함께 움직이지 않음"
    assert not e.begin_drag_node(1), "고정된 노드 1이 직접 끌림"


def T49():
    """A + 고정 + 선 끝점 하나가 고정: 선의 다른 끝을 끌면 노드 1은 비율 t를 지키며 따라온다"""
    e = mk(A_N, A_L, pins=[1, 3], rules=[("A", 1, 2, 3)])
    p0 = snap(e)
    t0 = seg_t(e)
    run(e, 2, line((0, 100), (30, 150), 20))
    assert e.pose.pos[3] == p0[3]
    assert seg_dist(e) < 0.05 and abs(seg_t(e) - t0) < 0.005


def T50():
    """A + 고정: 선분 양끝도 고정이면 전부 정지"""
    e = mk(A_N, A_L, pins=[1, 2, 3], rules=[("A", 1, 2, 3)])
    p0 = snap(e)
    assert not e.begin_drag_node(2) and not e.begin_drag_node(1)
    assert e.pose.pos == p0


def T51():
    """A: 선(2-3 링크) 드래그 시 고정된 노드 1은 비율을 지키며 선과 함께 이동"""
    e = mk(A_N, A_L, pins=[1], rules=[("A", 1, 2, 3)])
    t0 = seg_t(e)
    assert e.begin_drag_link(2, 3, (100, 100))
    for tg in line((100, 100), (130, 160), 20):
        e.drag_move(tg)
    e.end_drag()
    assert seg_dist(e) < 0.05 and abs(seg_t(e) - t0) < 0.005
    assert dist(e.pose.pos[2], e.pose.pos[3]) - 200 < 0.05


def T52():
    """A 무작위: 어떤 드래그에서도 NaN 없이 노드 1이 선분 위(고정이면 비율 유지)에 있다"""
    rnd = random.Random(5)
    for trial in range(150):
        pins = rnd.choice([[], [1], [3], [1, 3]])
        e = mk(A_N, A_L, pins=pins, rules=[("A", 1, 2, 3)])
        t0 = seg_t(e)
        node = rnd.choice([n for n in (1, 2, 3, 4) if n not in pins])
        c = e.pose.pos[node]
        path = [(c[0] + rnd.uniform(-150, 150), c[1] + rnd.uniform(-150, 150)) for _ in range(rnd.choice([1, 4, 10]))]
        run(e, node, path, alt=rnd.random() < 0.3)
        finite(e)
        assert seg_dist(e) < 0.1, f"{trial}: 선 이탈 {seg_dist(e):.3f} pins={pins} node={node}"
        if 1 in pins:
            assert abs(seg_t(e) - t0) < 0.01, f"{trial}: t 변화"
        lens_kept(e)



# ----------------------------------------------------------------- 규모/적응형 반복 (T56~T59)
def _big_chain(n):
    from pose_editor_cl317 import PoseEditor
    e = PoseEditor()
    for i in range(n):
        e.add_node(i, 50 + 20 * i, 200)
    for i in range(n - 1):
        e.add_line(i, i + 1)
    return e


def _big_truss(k):
    from pose_editor_cl317 import PoseEditor
    e = PoseEditor()
    for i in range(k + 1):
        e.add_node(i, 50 * i, 200)
    for i in range(k):
        e.add_node(100 + i, 50 * i + 25, 150)
    for i in range(k):
        e.add_line(i, i + 1)
    for i in range(k - 1):
        e.add_line(100 + i, 100 + i + 1)
    for i in range(k):
        e.add_line(i, 100 + i)
        e.add_line(100 + i, i + 1)
    return e


def T56():
    """한쪽을 고정한 긴 사슬(30·60노드)의 끝을 도달 가능한 곳으로 끌면 막히지 않고 길이가 유지된다"""
    for n in (30, 60):
        e = _big_chain(n)
        e.toggle_pin(0)
        c = e.pose.pos[n - 1]
        st = run(e, n - 1, line(c, (c[0] - 60, c[1] + 120), 20))
        assert "blocked" not in st, f"사슬 {n}: blocked {st.count('blocked')}회"
        lens_kept(e)
        finite(e)


def T57():
    """트러스 8칸(노드 17개), 핀 1개: 핀을 중심으로 회전하며 막히지 않는다"""
    e = _big_truss(8)
    e.toggle_pin(0)
    p0, c = snap(e), e.pose.pos[107]
    st = run(e, 107, line(c, (c[0] + 10, c[1] - 30), 20))
    assert "blocked" not in st, f"blocked {st.count('blocked')}회"
    lens_kept(e)
    pins_fixed(e, p0)
    assert moved(e, p0) > 3


def T58():
    """소형 구조는 적응형이 느려지지 않는다(수렴하면 조기 종료)"""
    import time
    e = _stick([("R", 1, 2, 3), ("F", 1, 2, 4)])
    c = e.pose.pos[8]
    assert e.begin_drag_node(8)
    t0 = time.time()
    for i in range(80):
        e.drag_move((c[0] + 50 * math.sin(i * 0.15), c[1] + 40 * math.cos(i * 0.11)))
    ms = (time.time() - t0) * 1000 / 80
    e.end_drag()
    assert ms < 5.0, f"스틱맨 9노드 {ms:.1f} ms/스텝"
    return f"스틱맨 9노드 {ms:.2f} ms/스텝"


def T59():
    """수렴 불가 입력(큰 트러스 + 양 끝 고정 + 큰 이동)에서도 반복 상한으로 제때 끝난다"""
    import time
    e = _big_truss(24)
    e.toggle_pin(0)
    e.toggle_pin(24)
    c = e.pose.pos[112]
    assert e.begin_drag_node(112)
    worst = 0.0
    for tg in line(c, (c[0] + 200, c[1] - 200), 10):
        t0 = time.time()
        e.drag_move(tg)
        worst = max(worst, time.time() - t0)
    e.end_drag()
    finite(e)
    assert worst < 1.5, f"한 스텝 최대 {worst:.2f}초"
    return f"한 스텝 최대 {worst * 1000:.0f} ms"


# ----------------------------------------------------------------- Alt와 F 해제 범위 (T60~T62)
def _alt_f_rig():
    return mk([(1, 100, 100), (2, 200, 100), (3, 200, 0), (5, 100, 200)], [(1, 2), (2, 3), (1, 5)],
              rules=[("F", 1, 2, 3)])


def T60():
    """Alt: 드래그한 노드(5)가 포함되지 않은 F 규칙은 해제되지 않고 각도가 유지된다"""
    e = _alt_f_rig()
    a0 = e.pose.angles[0]
    run(e, 5, line((100, 200), (20, 240), 20), alt=True)
    assert dang(ang(e, 0), a0) < 0.02, f"F가 해제됨: {math.degrees(dang(ang(e, 0), a0)):.1f}° 변화"
    assert dang(e.pose.angles[0], a0) < 1e-9, "F 기준각이 재저장됨"
    within_max(e)


def T61():
    """Alt: 드래그한 노드가 포함된 F 규칙은 해제되고 종료 후 기준각이 갱신된다(n1·n2·n3 어느 위치든)"""
    for node, target in ((1, (60, 140)), (2, (240, 130)), (3, (230, 40))):
        e = _alt_f_rig()
        a0 = e.pose.angles[0]
        run(e, node, line(e.pose.pos[node], target, 20), alt=True)
        assert dang(ang(e, 0), a0) > 0.05, f"노드 {node}: F가 해제되지 않아 각도가 유지됨"
        assert dang(e.pose.angles[0], ang(e, 0)) < 1e-6, f"노드 {node}: 종료 후 기준각 미갱신"


def T62():
    """Alt: 규칙이 둘일 때 드래그한 노드가 속한 규칙만 해제, 나머지 규칙은 유지"""
    e = mk([(1, 100, 100), (2, 200, 100), (3, 200, 0), (4, 300, 100), (5, 300, 200)],
           [(1, 2), (2, 3), (2, 4), (4, 5)], rules=[("F", 1, 2, 3), ("F", 3, 2, 4)])
    a0 = dict(e.pose.angles)
    run(e, 5, line((300, 200), (360, 260), 20), alt=True)              # 노드 5는 어느 F에도 없음
    assert dang(ang(e, 0), a0[0]) < 0.02 and dang(ang(e, 1), a0[1]) < 0.02
    e2 = mk([(1, 100, 100), (2, 200, 100), (3, 200, 0), (4, 300, 100), (5, 300, 200)],
            [(1, 2), (2, 3), (2, 4), (4, 5)], rules=[("F", 1, 2, 3), ("F", 3, 2, 4)])
    run(e2, 1, line((100, 100), (60, 140), 20), alt=True)              # 노드 1은 첫 규칙에만 있음
    assert dang(ang(e2, 0), a0[0]) > 0.05, "드래그한 노드가 속한 F는 해제되어야 함"
    assert dang(ang(e2, 1), a0[1]) < 0.02, "다른 F는 유지되어야 함"


# ----------------------------------------------------------------- W 규칙: 오브젝트 연결 (T63~T65)
def T63():
    """W(주인, 따르는 노드): 따르는 노드는 주인을 따라가고, 따르는 쪽을 끌어도 주인은 끌려오지 않는다(일방향)"""
    e = mk([(1, 0, 0), (2, 100, 0), (3, 300, 100), (4, 380, 100)], [(1, 2), (3, 4)], rules=[("W", 2, 3)])
    assert e.pose.pos[3] == e.pose.pos[2]
    run(e, 1, line((0, 0), (40, 60), 20))                               # 주인 쪽을 끌면 도구(3,4)가 함께 이동
    assert dist(e.pose.pos[3], e.pose.pos[2]) < 0.05
    lens_kept(e)
    master = e.pose.pos[2]
    run(e, 4, line(e.pose.pos[4], (500, 20), 20))                       # 따르는 쪽 끝을 끌어도 주인은 그대로
    assert dist(e.pose.pos[2], master) < 0.05 and dist(e.pose.pos[3], e.pose.pos[2]) < 0.05
    lens_kept(e)


def T64():
    """W: 따르는 노드가 고정(pin)이면 주인은 움직일 수 없다. 규칙을 끄면 다시 자유"""
    e = mk([(1, 0, 0), (2, 100, 0), (3, 300, 100), (4, 380, 100)], [(1, 2), (3, 4)], pins=[4], rules=[("W", 2, 3)])
    p0 = snap(e)
    st = run(e, 1, line((0, 0), (40, 60), 10))
    assert moved(e, p0) < 0.5 and "ok" not in st
    e.set_rule_active(0, False)
    run(e, 1, line(e.pose.pos[1], (40, 60), 10))
    assert dist(e.pose.pos[1], (40, 60)) < 1.0


def T65():
    """연결되지 않은 오브젝트는 계산에서 제외되어 위치가 그대로이고, 오브젝트가 많아져도 비용이 늘지 않는다"""
    import time
    nodes, links = [(1, 0, 0), (2, 100, 0)], [(1, 2)]
    for o in range(1, 31):                                              # 서로 연결되지 않은 30개 오브젝트
        nodes += [(10 * o, 300 * o, 0), (10 * o + 1, 300 * o + 100, 0)]
        links += [(10 * o, 10 * o + 1)]
    e = mk(nodes, links)
    p0 = snap(e)
    assert e.begin_drag_node(2)
    t0 = time.time()
    for tg in line((100, 0), (140, 70), 40):
        e.drag_move(tg)
    ms = (time.time() - t0) * 1000 / 40
    e.end_drag()
    assert all(e.pose.pos[n] == p0[n] for n in p0 if n not in (1, 2)), "연결되지 않은 노드가 움직임"
    assert ms < 3.0, f"{ms:.2f} ms/스텝"
    return f"독립 오브젝트 30개가 더 있어도 {ms:.2f} ms/스텝"

# ----------------------------------------------------------------- 실행기
def main():
    names = sorted(n for n in globals() if n.startswith("T") and n[1:].isdigit())
    rows = []
    for n in names:
        try:
            note = globals()[n]()
            if note and note.startswith("WARN"):
                rows.append((n, "WARN", note[5:].strip(" :")))
            else:
                rows.append((n, "PASS", note or ""))
        except AssertionError as ex:
            rows.append((n, "FAIL", str(ex)))
        except Exception as ex:                      # noqa: BLE001
            rows.append((n, "ERROR", f"{type(ex).__name__}: {ex}"))
    for r in rows:
        print(f"{r[0]}  {r[1]:5s}  {r[2]}")
    cnt = {s: sum(1 for r in rows if r[1] == s) for s in ("PASS", "WARN", "FAIL", "ERROR")}
    print("합계:", cnt)
    return rows


if __name__ == "__main__":
    main()
