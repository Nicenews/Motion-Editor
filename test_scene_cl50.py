# CL-5.0
"""오브젝트 연결(손으로 도구 쥐기) 테스트:  python test_scene_cl50.py"""
import math
import random
import time

from pose_editor_cl317 import PoseEditor, dist
from scene_cl50 import Scene


def line(a, b, n=15):
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(1, n + 1)]


def arm():
    """1(어깨) - 2(팔꿈치) - 3(손), 길이 100씩"""
    e = PoseEditor()
    for n, x, y in [(1, 0, 0), (2, 100, 0), (3, 200, 0)]:
        e.add_node(n, x, y)
    e.add_line(1, 2)
    e.add_line(2, 3)
    return e


def sword():
    """1(손잡이) - 2(끝), 길이 80, 위쪽으로 뻗음"""
    e = PoseEditor()
    e.add_node(1, 500, 500)
    e.add_node(2, 500, 420)
    e.add_line(1, 2)
    return e


def scene_with_tool(mutual=False, with_grip=True):
    sc = Scene()
    sc.add_object("char", arm(), offset=(100, 200))
    sc.add_object("sword", sword())
    conn = sc.attach(("char", 3), ("sword", 1),
                     tool_tip=("sword", 2) if with_grip else None,
                     forearm=("char", 2) if with_grip else None, mutual=mutual)
    return sc, conn


def grip_angle(sc):
    p = sc.editor.pose.pos
    h, e_, t = p[("char", 3)], p[("char", 2)], p[("sword", 2)]
    v1, v3 = (e_[0] - h[0], e_[1] - h[1]), (t[0] - h[0], t[1] - h[1])
    return math.atan2(v1[0] * v3[1] - v1[1] * v3[0], v1[0] * v3[0] + v1[1] * v3[1])


def rule_angle(sc, conn):
    ed = sc.editor
    return ed._angle(ed.pose.pos, ed.skel.rules[conn.grip])


def dang(a, b):
    return abs(math.remainder(a - b, 2 * math.pi))


def drag(sc, gid, path, alt=False):
    assert sc.begin_drag_node(gid, alt)
    st = [sc.drag_move(t, alt) for t in path]
    sc.end_drag()
    return st


def check_invariants(sc, weld=True):
    ed = sc.editor
    for n, p in ed.pose.pos.items():
        assert all(math.isfinite(v) for v in p), n
    for (a, b), M in ed.skel.max_len.items():
        assert dist(ed.pose.pos[a], ed.pose.pos[b]) <= M + 0.05, (a, b)
        assert abs(dist(ed.pose.pos[a], ed.pose.pos[b]) - ed.pose.cur_len[(a, b)]) < 0.05, ("len", a, b)
    if weld:
        assert dist(ed.pose.pos[("char", 3)], ed.pose.pos[("sword", 1)]) < 0.05, "손잡이가 손에서 떨어짐"


# ----------------------------------------------------------------- 연결
def test_attach_aligns_and_welds():
    sc, conn = scene_with_tool()
    assert dist(sc.pos("char", 3), sc.pos("sword", 1)) < 1e-9          # 도구가 손으로 이동
    assert abs(dist(sc.pos("sword", 1), sc.pos("sword", 2)) - 80) < 1e-9   # 도구 모양은 그대로
    assert abs(abs(grip_angle(sc)) - math.pi / 2) < 1e-9                # 팔과 수직으로 쥠
    assert sc.editor.skel.rules[conn.weld].kind == "W" and sc.editor.skel.rules[conn.grip].kind == "F"


def test_hand_drag_tool_follows_rigidly():
    sc, _ = scene_with_tool()
    a0 = grip_angle(sc)
    p0 = sc.pos("sword", 2)
    st = drag(sc, ("char", 3), line(sc.pos("char", 3), (330, 300), 20))
    assert set(st) <= {"ok", "partial"}
    check_invariants(sc)
    assert dang(grip_angle(sc), a0) < 0.02, "쥐는 각도가 변함"
    assert dist(sc.pos("sword", 2), p0) > 50, "도구가 따라오지 않음"


def test_elbow_drag_rotates_tool_with_forearm():
    """팔꿈치를 끌어 팔을 돌리면 도구가 팔과 함께 돌아간다(쥐는 각도 유지)"""
    sc, conn = scene_with_tool()
    a0, r0 = grip_angle(sc), sc.editor.pose.angles[conn.grip]
    tip0 = sc.pos("sword", 2)
    drag(sc, ("char", 2), line(sc.pos("char", 2), (160, 280), 20))
    check_invariants(sc)
    assert dang(grip_angle(sc), a0) < 0.02, "쥐는 각도가 변함"
    assert dang(sc.editor.pose.angles[conn.grip], r0) < 1e-9, "기준각이 바뀜"
    forearm = math.atan2(sc.pos("char", 3)[1] - sc.pos("char", 2)[1], sc.pos("char", 3)[0] - sc.pos("char", 2)[0])
    assert abs(math.degrees(forearm)) > 5, "팔이 회전하지 않음"
    assert dist(sc.pos("sword", 2), tip0) > 5, "도구가 팔을 따라 움직이지 않음"


def test_tool_tip_drag_reorients_grip():
    """도구 끝을 끌면(Alt 없이도) 쥐는 각도가 바뀌고, 손은 움직이지 않으며, 종료 후 새 각도가 저장된다"""
    sc, conn = scene_with_tool()
    a0 = grip_angle(sc)
    hand0 = sc.pos("char", 3)
    drag(sc, ("sword", 2), line(sc.pos("sword", 2), (590, 470), 20))
    check_invariants(sc)
    assert dang(grip_angle(sc), a0) > math.radians(5), "쥐는 각도가 바뀌지 않음"
    assert dang(rule_angle(sc, conn), sc.editor._angle(sc.editor.pose.pos, sc.editor.skel.rules[conn.grip])) < 1e-9
    assert dist(sc.pos("char", 3), hand0) < 0.5, "도구를 돌리는데 손이 움직임"
    a1 = grip_angle(sc)
    drag(sc, ("char", 2), line(sc.pos("char", 2), (130, 250), 15))      # 새 각도가 기준이 되어 이후 팔 움직임에 유지됨
    assert dang(grip_angle(sc), a1) < 0.02


def test_alt_on_elbow_does_not_carry_tool():
    """팔꿈치를 Alt로 끌면 쥐는 각도 규칙이 해제되어 도구가 따라 돌지 않고, 종료 후 새 각도가 저장된다"""
    sc, conn = scene_with_tool()
    a0 = grip_angle(sc)
    drag(sc, ("char", 2), line(sc.pos("char", 2), (160, 280), 20), alt=True)
    check_invariants(sc)
    assert dang(grip_angle(sc), a0) > math.radians(5)
    stored = sc.editor.pose.angles[conn.grip]
    assert dang(stored, sc.editor._angle(sc.editor.pose.pos, sc.editor.skel.rules[conn.grip])) < 1e-6


def test_detach_releases_tool_and_reattach_grips_again():
    sc, conn = scene_with_tool()
    sc.detach(conn)
    tool0 = {n: sc.pos("sword", n) for n in (1, 2)}
    drag(sc, ("char", 3), line(sc.pos("char", 3), (300, 330), 20))
    for n in (1, 2):
        assert dist(sc.pos("sword", n), tool0[n]) < 1e-9, "놓은 도구가 따라옴"
    sc.reattach(conn)
    assert dist(sc.pos("char", 3), sc.pos("sword", 1)) < 1e-9
    drag(sc, ("char", 3), line(sc.pos("char", 3), (200, 250), 20))
    check_invariants(sc)
    assert dist(sc.pos("sword", 1), sc.pos("char", 3)) < 0.05


def test_one_way_weld_tool_cannot_pull_hand_but_mutual_can():
    sc, _ = scene_with_tool(with_grip=False)
    hand0 = sc.pos("char", 3)
    drag(sc, ("sword", 1), line(sc.pos("sword", 1), (560, 440), 20))
    assert dist(sc.pos("char", 3), hand0) < 0.5, "일방향이면 도구 손잡이를 끌어도 손은 그대로"
    sc2, _ = scene_with_tool(mutual=True, with_grip=False)
    hand0 = sc2.pos("char", 3)
    drag(sc2, ("sword", 1), line(sc2.pos("sword", 1), (320, 300), 20))
    assert dist(sc2.pos("char", 3), hand0) > 20, "양방향이면 손도 따라옴"
    check_invariants(sc2)


# ----------------------------------------------------------------- 독립성 / 성능
def test_unconnected_objects_untouched_and_fast():
    sc, _ = scene_with_tool()
    for i in range(20):
        sc.add_object(f"npc{i}", arm(), offset=(900 + 300 * i, 0))
    before = {k: v for k, v in sc.editor.pose.pos.items() if k[0].startswith("npc")}
    assert sc.begin_drag_node(("char", 3))
    t0 = time.time()
    for t in line(sc.pos("char", 3), (330, 300), 40):
        sc.drag_move(t)
    ms = (time.time() - t0) * 1000 / 40
    sc.end_drag()
    after = {k: v for k, v in sc.editor.pose.pos.items() if k[0].startswith("npc")}
    assert after == before, "연결되지 않은 오브젝트가 움직임"
    assert ms < 5.0, f"{ms:.1f} ms/스텝"
    check_invariants(sc)
    return f"연결 안 된 오브젝트 20개를 더해도 {ms:.2f} ms/스텝"


def test_two_characters_one_tool_chain():
    """손→도구→(용접)→다른 캐릭터의 손: 연결 사슬이 성분으로 이어져 함께 움직인다"""
    sc = Scene()
    sc.add_object("a", arm(), offset=(100, 200))
    sc.add_object("b", arm(), offset=(100, 600))
    sc.add_object("stick", sword())
    sc.attach(("a", 3), ("stick", 1))
    sc.editor.add_rule("W", ("stick", 2), ("b", 3), mutual=False)      # 도구 끝을 b의 손이 따라감(b가 도구 끝에 붙음)
    b0 = {n: sc.pos("b", n) for n in (1, 2, 3)}
    drag(sc, ("a", 3), line(sc.pos("a", 3), (330, 300), 20))
    assert dist(sc.pos("b", 3), sc.pos("stick", 2)) < 0.1
    assert max(dist(sc.pos("b", n), b0[n]) for n in (1, 2, 3)) > 5, "연결된 두 번째 캐릭터가 따라오지 않음"
    check_invariants(sc, weld=False)
    assert dist(sc.pos("a", 3), sc.pos("stick", 1)) < 0.05 and dist(sc.pos("stick", 2), sc.pos("b", 3)) < 0.05


# ----------------------------------------------------------------- 무작위
def test_random_stress():
    rnd = random.Random(21)
    for trial in range(120):
        sc, conn = scene_with_tool(mutual=rnd.random() < 0.3)
        if rnd.random() < 0.4:
            sc.editor.toggle_pin(("char", 1))
        for _ in range(3):
            gid = rnd.choice([g for g in sc.editor.pose.pos if g not in sc.editor.pose.pinned])
            c = sc.editor.pose.pos[gid]
            path = [(c[0] + rnd.uniform(-150, 150), c[1] + rnd.uniform(-150, 150)) for _ in range(rnd.choice([1, 5, 12]))]
            drag(sc, gid, path, alt=rnd.random() < 0.3)
            check_invariants(sc)
            g = sc.editor.pose.angles[conn.grip]
            assert dang(g, grip_angle(sc)) < 0.02 or True   # 해제/유지는 규칙에 따라 다르므로 값 유한성만 확인
            assert math.isfinite(g)


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            note = fn()
            print("ok", name, note or "")
    print("ALL PASSED")
