# CL-3.9
"""코어 solver 헤드리스 테스트 (PyQt6 불필요):  python test_pose_editor_cl39.py"""
import math

from pose_editor_cl39 import build_demo, dist, key

TOL = 0.1


def drag(e, node, target, alt=False):
    assert e.begin_drag_node(node, alt)
    r = e.drag_move(target, alt)
    e.end_drag()
    return r


def lens_ok(e, skip=()):
    for k, L in e.pose.cur_len.items():
        if k in skip:
            continue
        d = dist(e.pose.pos[k[0]], e.pose.pos[k[1]])
        assert abs(d - L) < TOL, (k, d, L)


def test_drag_spine_keeps_lengths_and_angles():
    e = build_demo()
    ang0 = dict(e.pose.angles)
    assert drag(e, 4, (350, 330)) == "ok"
    lens_ok(e)
    for i, a in ang0.items():
        now = e._angle(e.pose.pos, e.skel.rules[i])
        assert abs(math.remainder(now - a, 2 * math.pi)) < 0.01
    assert e.pose.pos[2] != (300, 200)          # 노드 2가 딸려옴


def test_r_point_symmetry_and_limit():
    e = build_demo()
    r = drag(e, 1, (250, 230))
    assert r in ("ok", "partial")
    p1, p2, p3 = (e.pose.pos[n] for n in (1, 2, 3))
    k = e.skel.rules[0].ratio
    assert dist(p3, (p2[0] + k * (p2[0] - p1[0]), p2[1] + k * (p2[1] - p1[1]))) < TOL
    drag(e, 1, (0, 0))                          # 최초 길이를 넘는 요청
    for k in ((1, 2), (2, 3)):
        assert dist(e.pose.pos[k[0]], e.pose.pos[k[1]]) <= e.skel.max_len[k] + TOL


def test_pinned_partner_blocks_r():
    e = build_demo()
    e.toggle_pin(1)
    before = dict(e.pose.pos)
    assert e.begin_drag_node(3, False)
    assert e.drag_move((380, 230)) == "blocked"
    e.end_drag()
    assert e.pose.pos == before
    assert not e.begin_drag_node(1, False)       # 고정 노드는 직접 이동 불가


def test_pinned_rotation_keeps_pin_and_length():
    e = build_demo()
    e.toggle_pin(1)
    p1 = e.pose.pos[1]
    drag(e, 2, (300, 150))
    assert e.pose.pos[1] == p1
    assert abs(dist(e.pose.pos[1], e.pose.pos[2]) - e.skel.max_len[(1, 2)]) < TOL


def test_alt_pulls_neighbors_and_respects_max():
    e = build_demo()
    drag(e, 5, (300, -200), alt=True)
    assert e.pose.pos[2][1] < 150               # 최대 길이에 닿아 이웃(2)이 딸려옴
    for k, M in e.skel.max_len.items():
        assert dist(e.pose.pos[k[0]], e.pose.pos[k[1]]) <= M + TOL
        assert e.pose.cur_len[k] <= M + 1e-6


def test_link_drag_pinned_cases():
    e = build_demo()
    e.toggle_pin(2)
    assert e.begin_drag_link(2, 4, (300, 250))
    e.drag_move((400, 200))
    e.end_drag()
    assert abs(dist(e.pose.pos[2], e.pose.pos[4]) - e.skel.max_len[(2, 4)]) < TOL
    e2 = build_demo()
    e2.toggle_pin(2)
    e2.toggle_pin(4)
    assert not e2.begin_drag_link(2, 4, (300, 250))


def test_undo_redo_includes_pin():
    e = build_demo()
    start = dict(e.pose.pos)
    drag(e, 4, (350, 330))
    e.toggle_pin(5)
    assert 5 in e.pose.pinned
    assert e.undo() and 5 not in e.pose.pinned
    assert e.undo() and e.pose.pos == start
    assert e.redo() and e.redo() and 5 in e.pose.pinned


def _r_truss():
    """1·2·4가 일직선인 R(1,2,4) + F(1,2,3), 삼각형 두 개"""
    from test_structures_cl39 import mk
    N = [(1, 0, 100), (2, 100, 100), (3, 100, 0), (4, 200, 100)]
    L = [(1, 2), (2, 4), (1, 3), (3, 4), (2, 3)]
    return mk, N, L, [("F", 1, 2, 3), ("R", 1, 2, 4)]


def test_r_center_fixed_while_dragging_1_or_3():
    """R 활성(노드 1·3 드래그) 중에는 노드 2가 위치 고정"""
    mk, N, L, rules = _r_truss()
    e = mk(N, L, rules=rules)
    c0 = e.pose.pos[2]
    assert e.begin_drag_node(1)
    for tg in [(-10, 110), (-20, 120), (-30, 125)]:
        e.drag_move(tg)
        assert e.pose.pos[2] == c0
    e.end_drag()


def test_r_partner_pinned_blocks_even_if_center_could_move():
    """상대 노드(4)가 고정이면 노드 1은 R로 이동 불가(중심이 끌려가서 풀리면 안 됨)"""
    mk, N, L, rules = _r_truss()
    e = mk(N, L, pins=[4], rules=rules)
    before = dict(e.pose.pos)
    assert e.begin_drag_node(1)
    assert e.drag_move((-20, 130)) == "blocked"
    e.end_drag()
    assert e.pose.pos == before


def test_r_inactive_when_other_node_dragged_center_moves_lengths_kept():
    """노드 3을 끌 때는 R이 꺼져 노드 2가 이동 가능, 1-2·2-3 길이는 현재 길이 유지"""
    mk, N, L, rules = _r_truss()
    e = mk(N, L, rules=rules)
    assert e.begin_drag_node(3)
    e.drag_move((130, -30))
    e.end_drag()
    assert e.pose.pos[2] != (100, 100)
    for k in ((1, 2), (2, 4)):
        assert abs(dist(e.pose.pos[k[0]], e.pose.pos[k[1]]) - e.pose.cur_len[k]) < 0.05
        assert abs(e.pose.cur_len[k] - 100.0) < 1e-6


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("ALL PASSED")
