# CL-5.0
"""
Scene (CL-5.0)  -  여러 오브젝트를 하나의 장면으로 묶고, 오브젝트 사이를 연결한다(예: 손으로 도구 쥐기).
Qt 비의존. 계산은 pose_editor_cl317의 solver를 그대로 쓴다.

구조
  - 장면은 노드 ID를 (오브젝트 이름, 노드 번호)로 네임스페이스해서 하나의 PoseEditor에 담는다.
  - 오브젝트 사이 연결은 용접 규칙 W(손, 도구 손잡이)로 표현한다.
      W는 일방향: 도구가 손을 따라가고, 도구가 손(캐릭터)을 끌고 가지는 않는다. (mutual=True면 양방향)
  - '쥐는 각도'는 F(도구 끝, 손, 팔꿈치): 팔을 움직이면 도구가 함께 회전하고(F 유지),
    도구 끝을 끌면 F가 해제되어(노드 1) 쥐는 각도를 바꿀 수 있으며 이동 후 새 각도로 갱신된다.
  - 연결 해제는 규칙을 끄는 것(detach): 도구를 놓는다. 다시 attach하면 다시 쥔다.
  - solver는 드래그한 노드와 구속으로 이어진 묶음(성분)만 계산하므로, 연결되지 않은 오브젝트는 비용이 들지 않는다.
"""
from dataclasses import dataclass

from pose_editor_cl317 import PoseEditor, Rule, dist, key


@dataclass
class Connection:
    weld: int                 # 용접 규칙 인덱스
    grip: int | None = None   # 쥐는 각도 F 규칙 인덱스(없으면 None)


class Scene:
    def __init__(self):
        self.editor = PoseEditor()
        self.objects = {}          # oid -> {"nodes": [전역 ID], "rules": [규칙 인덱스]}

    # ---- 오브젝트 --------------------------------------------------
    def add_object(self, oid, source, offset=(0.0, 0.0)):
        """source(PoseEditor)의 구조와 포즈를 복사해 장면에 넣는다. 노드는 (oid, 번호)로 이름이 바뀐다."""
        if oid in self.objects:
            raise ValueError(f"이미 있는 오브젝트: {oid}")
        ed, ox, oy = self.editor, offset[0], offset[1]
        g = lambda n: (oid, n)                                   # noqa: E731
        info = {"nodes": [], "rules": []}
        for n, (x, y) in source.pose.pos.items():
            ed.add_node(g(n), x + ox, y + oy)
            info["nodes"].append(g(n))
        for (a, b), L in source.skel.max_len.items():
            k = key(g(a), g(b))
            ed.skel.max_len[k] = L
            ed.pose.cur_len[k] = source.pose.cur_len.get((a, b), L)
        for i, r in enumerate(source.skel.rules):
            ed.skel.rules.append(Rule(r.kind, g(r.n1), g(r.n2), g(r.n3), r.ratio, r.mutual, r.active))
            idx = len(ed.skel.rules) - 1
            info["rules"].append(idx)
            if r.kind == "F":
                ed.pose.angles[idx] = source.pose.angles[i]
        ed.pose.pinned |= {g(n) for n in source.pose.pinned}
        self.objects[oid] = info

    def pos(self, oid, node):
        return self.editor.pose.pos[(oid, node)]

    def object_pose(self, oid):
        """한 오브젝트의 {노드 번호: 위치}"""
        return {n: self.editor.pose.pos[(o, n)] for (o, n) in self.objects[oid]["nodes"]}

    # ---- 연결 ------------------------------------------------------
    def attach(self, hand, tool_node, tool_tip=None, forearm=None, mutual=False):
        """도구를 손에 쥐게 한다.
           hand=(오브젝트, 노드) 쥘 노드, tool_node=(오브젝트, 노드) 도구 손잡이,
           tool_tip(도구 끝)과 forearm(손 바로 앞 노드)을 주면 쥐는 각도 F(tool_tip, hand, forearm)도 건다.
           1) 도구 오브젝트 전체를 평행이동해 손잡이를 손에 맞추고  2) 용접 W 추가  3) 쥐는 각도 F 추가"""
        ed = self.editor
        toid = tool_node[0]
        hx, hy = ed.pose.pos[hand]
        tx, ty = ed.pose.pos[tool_node]
        dx, dy = hx - tx, hy - ty
        for n in self.objects[toid]["nodes"]:
            x, y = ed.pose.pos[n]
            ed.pose.pos[n] = (x + dx, y + dy)
        w = ed.add_rule("W", hand, tool_node, mutual=mutual)
        f = ed.add_rule("F", tool_tip, hand, forearm) if (tool_tip is not None and forearm is not None) else None
        return Connection(w, f)

    def detach(self, conn):
        """도구를 놓는다(연결 규칙을 끈다)."""
        self.editor.set_rule_active(conn.weld, False)
        if conn.grip is not None:
            self.editor.set_rule_active(conn.grip, False)

    def reattach(self, conn):
        """놓았던 도구를 다시 쥔다. 도구를 손 위치로 옮기고(손잡이→손) 쥐는 각도는 현재 모양으로 새로 잡는다."""
        ed = self.editor
        w = ed.skel.rules[conn.weld]
        hx, hy = ed.pose.pos[w.n1]
        tx, ty = ed.pose.pos[w.n2]
        for n in self.objects[w.n2[0]]["nodes"]:
            x, y = ed.pose.pos[n]
            ed.pose.pos[n] = (x + hx - tx, y + hy - ty)
        ed.set_rule_active(conn.weld, True)
        if conn.grip is not None:
            ed.set_rule_active(conn.grip, True)
            ed.pose.angles[conn.grip] = ed._angle(ed.pose.pos, ed.skel.rules[conn.grip])

    # ---- 편집(드래그) 전달 ------------------------------------------
    def begin_drag_node(self, gid, alt=False):
        return self.editor.begin_drag_node(gid, alt)

    def drag_move(self, target, alt=False):
        return self.editor.drag_move(target, alt)

    def end_drag(self):
        self.editor.end_drag()
