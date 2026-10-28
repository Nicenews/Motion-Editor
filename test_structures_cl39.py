# CL-3.9
"""트리 / 트러스(삼각형 연결) / 폐곡선 구조에서 고정점이 있을 때의 동작 테스트
실행: python test_structures_cl39.py   (PyQt6 불필요)

공통 불변식(모든 시나리오에서 검사)
  - 고정 노드는 움직이지 않는다
  - 현재 길이 유지 링크의 길이 오차 < 0.05, 최대 길이 초과 < 0.05
  - 삼각형이 뒤집히지 않는다(부호 있는 면적의 부호 유지)
  - NaN 없음
"""
import math
import time

from pose_editor_cl39 import PoseEditor, dist, key


# ----------------------------------------------------------------- 도우미
def mk(nodes, links, pins=(), rules=()):
    e = PoseEditor()
    for n, x, y in nodes:
        e.add_node(n, x, y)
    for a, b in links:
        e.add_line(a, b)
    for r in rules:
        e.add_rule(*r)
    for p in pins:
        e.toggle_pin(p)
    return e


def area(p, a, b, c):
    return (p[b][0] - p[a][0]) * (p[c][1] - p[a][1]) - (p[b][1] - p[a][1]) * (p[c][0] - p[a][0])


def triangles(e):
    L, ns, out = set(e.skel.max_len), sorted(e.pose.pos), []
    for i in ns:
        for j in ns:
            for k in ns:
                if i < j < k and key(i, j) in L and key(j, k) in L and key(i, k) in L:
                    out.append((i, j, k))
    return out


def line(a, b, n=30):
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]


def arc(c, r, a0, a1, n=40):
    return [(c[0] + r * math.cos(math.radians(a0 + (a1 - a0) * i / n)),
             c[1] + r * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(1, n + 1)]


def drag(e, node, path, alt=False):
    """path를 따라 드래그하고 결과 요약을 돌려준다. 공통 불변식도 여기서 검사."""
    p0 = dict(e.pose.pos)
    tris = triangles(e)
    a0 = {t: area(p0, *t) for t in tris}
    assert e.begin_drag_node(node, alt)
    states, t0 = {}, time.time()
    for tg in path:
        r = e.drag_move(tg, alt)
        states[r] = states.get(r, 0) + 1
    e.end_drag()
    ms = (time.time() - t0) * 1000 / len(path)
    p = e.pose.pos
    assert all(not math.isnan(v) for q in p.values() for v in q), "NaN"
    for n in e.pose.pinned:
        assert dist(p[n], p0[n]) < 1e-9, f"고정 노드 {n} 이동"
    for (a, b), L in e.pose.cur_len.items():
        assert abs(dist(p[a], p[b]) - L) < 0.05, f"길이 {a}-{b}"
    for (a, b), M in e.skel.max_len.items():
        assert dist(p[a], p[b]) <= M + 0.05, f"최대 길이 {a}-{b}"
    for t in tris:
        assert area(p, *t) * a0[t] > 0, f"삼각형 {t} 뒤집힘"
    moved = max(dist(p[n], p0[n]) for n in p)
    return {"states": states, "moved": moved, "ms": ms, "p0": p0}


# ----------------------------------------------------------------- 구조 정의
TREE_N = [(1, 300, 100), (2, 220, 180), (3, 380, 180), (4, 170, 260), (5, 260, 260),
          (6, 340, 260), (7, 430, 260)]
TREE_L = [(1, 2), (1, 3), (2, 4), (2, 5), (3, 6), (3, 7)]

BOT = [(i + 1, 100 * i, 200) for i in range(5)]            # 1..5 하현재
TOP = [(6 + i, 50 + 100 * i, 100) for i in range(4)]       # 6..9 상현재
TRUSS_L = ([(1, 2), (2, 3), (3, 4), (4, 5), (6, 7), (7, 8), (8, 9)]
           + [(1, 6), (6, 2), (2, 7), (7, 3), (3, 8), (8, 4), (4, 9), (9, 5)])

SQ_N = [(1, 0, 0), (2, 100, 0), (3, 100, 100), (4, 0, 100)]
SQ_L = [(1, 2), (2, 3), (3, 4), (4, 1)]
PENT_N = [(i + 1, 200 + 120 * math.cos(math.radians(90 + 72 * i)),
           200 + 120 * math.sin(math.radians(90 + 72 * i))) for i in range(5)]
PENT_L = [(1, 2), (2, 3), (3, 4), (4, 5), (5, 1)]


# ----------------------------------------------------------------- 트리
def test_tree_root_pinned_leaf_in_reach():
    e = mk(TREE_N, TREE_L, pins=[1])
    r = drag(e, 4, arc((300, 100), dist((300, 100), (170, 260)), 130, 100))
    assert r["states"] == {"ok": 40}
    assert dist(e.pose.pos[4], (300 + dist((300, 100), (170, 260)) * math.cos(math.radians(100)),
                                100 + dist((300, 100), (170, 260)) * math.sin(math.radians(100)))) < 1.0


def test_tree_pinned_parent_node_rotates_toward_mouse():
    """부모(1)가 고정이면 노드 2는 1을 중심으로 마우스 쪽으로 회전해야 한다(달라붙으면 실패)."""
    e = mk(TREE_N, TREE_L, pins=[1])
    r = drag(e, 2, line((220, 180), (160, 150)))
    assert r["moved"] > 10, "노드 2가 마우스를 따라 회전하지 않음"
    assert abs(dist(e.pose.pos[1], e.pose.pos[2]) - e.skel.max_len[(1, 2)]) < 0.05


def test_tree_out_of_reach_stops_at_limit():
    e = mk(TREE_N, TREE_L, pins=[1])
    drag(e, 4, line((170, 260), (60, 60)))
    reach = e.skel.max_len[(1, 2)] + e.skel.max_len[(2, 4)]
    assert dist(e.pose.pos[1], e.pose.pos[4]) <= reach + 0.05


def test_tree_two_pinned_leaves():
    e = mk(TREE_N, TREE_L, pins=[4, 7])
    r = drag(e, 5, line((260, 260), (260, 330)))
    assert r["states"].get("ok", 0) == 30


def test_tree_with_f_rule_keeps_angle():
    e = mk(TREE_N, TREE_L, pins=[1], rules=[("F", 2, 1, 3)])
    a0 = e.pose.angles[0]
    drag(e, 4, arc((300, 100), dist((300, 100), (170, 260)), 130, 110, n=20))
    now = e._angle(e.pose.pos, e.skel.rules[0])
    assert abs(math.remainder(now - a0, 2 * math.pi)) < 0.02


# ----------------------------------------------------------------- 트러스
def test_truss_single_pin_rotates_rigidly():
    """핀 1개: 트러스 전체가 핀을 중심으로 강체 회전해야 한다."""
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    r = drag(e, 9, line((350, 100), (350, 40)))
    assert r["moved"] > 10, "트러스가 회전하지 않음"
    p0, p = r["p0"], e.pose.pos
    for n in p:
        assert abs(dist(p[n], p[1]) - dist(p0[n], p0[1])) < 0.1, f"핀까지 거리 변화 {n}"


def test_truss_single_pin_on_arc_reaches_target():
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    rr = dist((0, 200), (350, 100))
    r = drag(e, 9, arc((0, 200), rr, -16, -40))
    assert r["states"] == {"ok": 40}


def test_truss_two_pins_is_rigid():
    for pins, node, path in [([1, 2], 9, line((350, 100), (350, 40))),
                             ([1, 5], 7, line((150, 100), (150, 60)))]:
        e = mk(BOT + TOP, TRUSS_L, pins=pins)
        r = drag(e, node, path)
        assert r["moved"] < 0.5, f"핀 {pins}: 움직이면 안 됨"
        assert "ok" not in r["states"]


def test_truss_alt_with_pin():
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    r = drag(e, 9, line((350, 100), (350, 40)), alt=True)
    assert r["moved"] > 10


def test_truss_speed():
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    r = drag(e, 9, line((350, 100), (350, 40)))
    assert r["ms"] < 50, f"{r['ms']:.1f} ms/step"


# ----------------------------------------------------------------- 폐곡선
def test_fourbar_crank_follows_circle():
    e = mk(SQ_N, SQ_L, pins=[1, 2])
    r = drag(e, 4, arc((0, 0), 100, 90, 60))
    assert r["states"] == {"ok": 40}


def test_fourbar_crank_slides_when_mouse_off_circle():
    e = mk(SQ_N, SQ_L, pins=[1, 2])
    r = drag(e, 4, line((0, 100), (30, 95)))
    assert r["moved"] > 5, "4절 링크에서 마우스가 원을 벗어나면 달라붙음"


def test_loop_with_diagonal_and_pins_is_rigid():
    e = mk(SQ_N, SQ_L + [(1, 3)], pins=[1, 2])
    r = drag(e, 3, line((100, 100), (140, 100)))
    assert r["moved"] < 0.5


def test_pentagon_one_pin_free_motion():
    e = mk(PENT_N, PENT_L, pins=[1])
    r = drag(e, 3, line(PENT_N[2][1:], (PENT_N[2][1] + 30, PENT_N[2][2] + 30)))
    assert r["states"] == {"ok": 30}


def test_pentagon_two_pins_one_dof():
    e = mk(PENT_N, PENT_L, pins=[1, 3])
    r = drag(e, 4, line(PENT_N[3][1:], (PENT_N[3][1] + 10, PENT_N[3][2] + 10)))
    assert r["moved"] > 1


def test_undo_after_structure_drag():
    e = mk(BOT + TOP, TRUSS_L, pins=[1])
    start = dict(e.pose.pos)
    drag(e, 9, line((350, 100), (350, 40)))
    assert e.pose.pos != start
    assert e.undo() and e.pose.pos == start

def test_no_flip_random_stress():
    """무작위 고정/드래그/Alt/큰 점프 600회: 고정·길이·최대 길이·NaN·삼각형 뒤집힘 위반이 없어야 한다"""
    import random
    rnd = random.Random(7)
    structs = {"tree": (TREE_N, TREE_L), "truss": (BOT + TOP, TRUSS_L), "square": (SQ_N, SQ_L),
               "square+diag": (SQ_N, SQ_L + [(1, 3)]), "pentagon": (PENT_N, PENT_L)}
    for name, (N, L) in structs.items():
        ids = [n for n, _, _ in N]
        for _ in range(120):
            pins = rnd.sample(ids, rnd.choice([0, 1, 1, 2]))
            e = mk(N, L, pins=pins)
            node = rnd.choice([n for n in ids if n not in pins])
            alt = rnd.random() < 0.3
            cx, cy = e.pose.pos[node]
            jump = rnd.choice([5, 30, 150, 400])
            path = [(cx + rnd.uniform(-jump, jump), cy + rnd.uniform(-jump, jump))
                    for _ in range(rnd.choice([1, 3, 10]))]
            drag(e, node, path, alt)          # drag() 안에서 모든 불변식(뒤집힘 포함)을 검사


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("ALL PASSED")
