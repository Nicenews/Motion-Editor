# Ch-3.5
import sys
import math
from dataclasses import dataclass
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QBrush, QPen, QColor, QPainter
from PyQt6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsScene, QGraphicsView, QMainWindow


# ---------- Geometry ----------
def distance(a, b):
    return math.hypot(a.x() - b.x(), a.y() - b.y())


def sub_point(a, b):
    return QPointF(a.x() - b.x(), a.y() - b.y())


def add_point(a, b):
    return QPointF(a.x() + b.x(), a.y() + b.y())


def mul_point(a, value):
    return QPointF(a.x() * value, a.y() * value)


def vector_length(v):
    return math.hypot(v.x(), v.y())


def normalized(v, fallback=QPointF(1.0, 0.0)):
    length = vector_length(v)
    if length < 1e-9:
        return QPointF(fallback)
    return QPointF(v.x() / length, v.y() / length)


def rotate_vector(v, angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return QPointF(v.x() * c - v.y() * s, v.x() * s + v.y() * c)


def cross(v1, v2):
    return v1.x() * v2.y() - v1.y() * v2.x()


def angle_between(center, a, b):
    v1 = sub_point(a, center)
    v2 = sub_point(b, center)
    l1 = vector_length(v1)
    l2 = vector_length(v2)
    if l1 < 1e-9 or l2 < 1e-9:
        return 0.0
    value = (v1.x() * v2.x() + v1.y() * v2.y()) / (l1 * l2)
    value = max(-1.0, min(1.0, value))
    return math.acos(value)


def orientation_sign(center, a, b):
    v1 = sub_point(a, center)
    v2 = sub_point(b, center)
    value = cross(v1, v2)
    if value >= 0.0:
        return 1.0
    return -1.0


# ---------- Model ----------
@dataclass
class Node:
    id: int
    position: QPointF
    fixed: bool = False


@dataclass
class Link:
    id: int
    node1: int
    node2: int
    max_length: float


class FRule:
    def __init__(self, node1, node2, node3, angle):
        self.node1 = node1
        self.node2 = node2
        self.node3 = node3
        self.angle = angle
        self.active = True

    def nodes(self):
        return self.node1, self.node2, self.node3


class RRule:
    def __init__(self, node1, node2, node3, ratio):
        self.node1 = node1
        self.node2 = node2
        self.node3 = node3
        self.ratio = ratio
        self.active = True

    def nodes(self):
        return self.node1, self.node2, self.node3


class Constraint:
    def __init__(self):
        self.f_rules = []
        self.r_rules = []

    def add_f_rule(self, rule):
        self.f_rules.append(rule)

    def add_r_rule(self, rule):
        self.r_rules.append(rule)


# ---------- Object ----------
class Object:
    def __init__(self):
        self.nodes = {}
        self.links = {}
        self.constraints = Constraint()
        self.next_link_id = 1
        self.drag = None

    # ---------- Node ----------
    def add_node(self, node_id, x, y, fixed=False):
        self.nodes[node_id] = Node(node_id, QPointF(x, y), fixed)

    def get_node(self, node_id):
        return self.nodes.get(node_id)

    # ---------- Link ----------
    def add_link(self, node1, node2, max_length=None):
        if node1 not in self.nodes or node2 not in self.nodes:
            return None
        if max_length is None:
            max_length = distance(self.nodes[node1].position, self.nodes[node2].position)
        link = Link(self.next_link_id, node1, node2, max_length)
        self.links[link.id] = link
        self.next_link_id += 1
        return link

    def get_link_between(self, node1, node2):
        for link in self.links.values():
            if (link.node1 == node1 and link.node2 == node2) or (link.node1 == node2 and link.node2 == node1):
                return link
        return None

    def get_link_id(self, node1, node2):
        link = self.get_link_between(node1, node2)
        return link.id if link else None

    # ---------- Constraint ----------
    def add_f_rule(self, node1, node2, node3):
        angle = angle_between(self.nodes[node2].position, self.nodes[node1].position, self.nodes[node3].position)
        rule = FRule(node1, node2, node3, angle)
        self.constraints.add_f_rule(rule)
        return rule

    def add_r_rule(self, node1, node2, node3):
        l1 = distance(self.nodes[node1].position, self.nodes[node2].position)
        l2 = distance(self.nodes[node3].position, self.nodes[node2].position)
        ratio = l2 / l1 if l1 > 1e-9 else 1.0
        rule = RRule(node1, node2, node3, ratio)
        self.constraints.add_r_rule(rule)
        return rule

    # ---------- Rule Search ----------
    def f_rules_for_node(self, node_id):
        return [rule for rule in self.constraints.f_rules if rule.active and node_id in rule.nodes()]

    def r_rules_for_node(self, node_id):
        return [rule for rule in self.constraints.r_rules if rule.active and node_id in rule.nodes()]

    def related_r_rule(self, node_id):
        rules = self.r_rules_for_node(node_id)
        return rules[0] if rules else None

    def related_f_rule(self, node_id):
        rules = self.f_rules_for_node(node_id)
        return rules[0] if rules else None

    # ---------- Mobility ----------
    def is_node_movable(self, node_id):
        node = self.nodes.get(node_id)
        if node is None or node.fixed:
            return False

        for rule in self.constraints.r_rules:
            if not rule.active:
                continue
            if node_id == rule.node1 and self.nodes[rule.node3].fixed:
                return False
            if node_id == rule.node3 and self.nodes[rule.node1].fixed:
                return False
            if node_id == rule.node2 and (self.nodes[rule.node1].fixed or self.nodes[rule.node3].fixed):
                return False

        for rule in self.constraints.f_rules:
            if not rule.active:
                continue
            if node_id == rule.node2 and self.nodes[rule.node1].fixed and self.nodes[rule.node3].fixed:
                return False

        return True

    # ---------- Drag Start ----------
    def begin_node_drag(self, node_id, mouse_position):
        if not self.is_node_movable(node_id):
            return False

        self.drag = {
            "type": "node",
            "node_id": node_id,
            "mouse_start": QPointF(mouse_position),
            "node_start": QPointF(self.nodes[node_id].position),
            "positions": {node.id: QPointF(node.position) for node in self.nodes.values()},
            "f_states": {},
            "r_states": {}
        }

        for rule in self.f_rules_for_node(node_id):
            self.capture_f_state(rule)

        for rule in self.r_rules_for_node(node_id):
            self.capture_r_state(rule)

        return True

    def begin_link_drag(self, link_id, mouse_position):
        link = self.links.get(link_id)
        if link is None:
            return False

        n1 = self.nodes[link.node1]
        n2 = self.nodes[link.node2]

        if n1.fixed and n2.fixed:
            return False

        self.drag = {
            "type": "link",
            "link_id": link_id,
            "mouse_start": QPointF(mouse_position),
            "positions": {node.id: QPointF(node.position) for node in self.nodes.values()},
            "f_states": {},
            "r_states": {}
        }

        for rule in self.constraints.r_rules:
            if rule.node1 in (link.node1, link.node2) or rule.node2 in (link.node1, link.node2) or rule.node3 in (link.node1, link.node2):
                self.capture_r_state(rule)

        for rule in self.constraints.f_rules:
            if rule.node1 in (link.node1, link.node2) or rule.node2 in (link.node1, link.node2) or rule.node3 in (link.node1, link.node2):
                self.capture_f_state(rule)

        return True

    def capture_f_state(self, rule):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        self.drag["f_states"][id(rule)] = {
            "length12": distance(n1.position, n2.position),
            "length23": distance(n2.position, n3.position),
            "angle": rule.angle,
            "sign": orientation_sign(n2.position, n1.position, n3.position),
            "positions": {
                rule.node1: QPointF(n1.position),
                rule.node2: QPointF(n2.position),
                rule.node3: QPointF(n3.position)
            }
        }

    def capture_r_state(self, rule):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        self.drag["r_states"][id(rule)] = {
            "length12": distance(n1.position, n2.position),
            "length23": distance(n2.position, n3.position),
            "ratio": rule.ratio,
            "positions": {
                rule.node1: QPointF(n1.position),
                rule.node2: QPointF(n2.position),
                rule.node3: QPointF(n3.position)
            }
        }

    # ---------- Node Drag ----------
    def update_node_drag(self, mouse_position):
        if not self.drag or self.drag["type"] != "node":
            return

        node_id = self.drag["node_id"]
        mouse_delta = sub_point(mouse_position, self.drag["mouse_start"])
        target = add_point(self.drag["node_start"], mouse_delta)

        r_rule = self.related_r_rule(node_id)

        # R이 존재하면 F보다 R을 먼저 처리한다.
        if r_rule is not None:
            self.apply_r_rule_drag(r_rule, node_id, target)
            self.recalculate_f_after_r(r_rule)
            self.enforce_max_lengths()
            return

        f_rule = self.related_f_rule(node_id)

        if f_rule is not None:
            self.apply_f_rule_drag(f_rule, node_id, target)
        else:
            self.nodes[node_id].position = QPointF(target)

        self.enforce_max_lengths()

    # ---------- F Drag ----------
    def apply_f_rule_drag(self, rule, moved_id, target):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]
        state = self.drag["f_states"].get(id(rule))

        if state is None:
            self.capture_f_state(rule)
            state = self.drag["f_states"][id(rule)]

        length12 = state["length12"]
        length23 = state["length23"]
        angle = state["angle"]
        sign = state["sign"]

        # Node 1은 F를 무시하고 자유 이동한다.
        if moved_id == rule.node1:
            n1.position = QPointF(target)
            return

        # Node 3도 Node 1이 움직일 수 없는 특수 상황에서는 F를 무시한다.
        if moved_id == rule.node3 and n1.fixed:
            n3.position = QPointF(target)
            return

        # Node 2 이동
        if moved_id == rule.node2:
            self.solve_f_from_node2(rule, target, length12, length23, angle, sign)
            return

        # Node 3 이동
        if moved_id == rule.node3:
            self.solve_f_from_node3(rule, target, length12, length23, angle, sign)
            return

    def solve_f_from_node2(self, rule, target, length12, length23, angle, sign):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        if n1.fixed and n3.fixed:
            return

        # 양쪽 Node가 자유로우면 F 전체를 평행이동한다.
        if not n1.fixed and not n3.fixed:
            delta = sub_point(target, n2.position)
            n1.position = add_point(n1.position, delta)
            n2.position = QPointF(target)
            n3.position = add_point(n3.position, delta)
            return

        # Node 1이 고정이면 Node 2를 1-2 길이 원 위에 배치한다.
        if n1.fixed:
            direction = normalized(sub_point(target, n1.position), normalized(sub_point(n2.position, n1.position)))
            n2.position = add_point(n1.position, mul_point(direction, length12))
            self.set_f_node3_from_node2(rule, length23, angle, sign)
            return

        # Node 3이 고정이면 Node 2를 2-3 길이 원 위에 배치한다.
        if n3.fixed:
            direction = normalized(sub_point(target, n3.position), normalized(sub_point(n2.position, n3.position)))
            n2.position = add_point(n3.position, mul_point(direction, length23))
            self.set_f_node1_from_node2(rule, length12, angle, sign)
            return

    def solve_f_from_node3(self, rule, target, length12, length23, angle, sign):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        # Node 2가 고정되어 있으면 Node 3의 위치를 기준으로 Node 1을 계산한다.
        if n2.fixed:
            direction3 = normalized(sub_point(target, n2.position), normalized(sub_point(n3.position, n2.position)))
            n3.position = add_point(n2.position, mul_point(direction3, length23))
            self.set_f_node1_from_node3(rule, length12, angle, sign)
            return

        # Node 2가 자유로우면 전체 F 구조를 이동한다.
        if not n1.fixed and not n2.fixed:
            delta = sub_point(target, n3.position)
            n1.position = add_point(n1.position, delta)
            n2.position = add_point(n2.position, delta)
            n3.position = QPointF(target)
            return

        # Node 1이 고정된 특수 경우는 호출 전에 처리되므로 여기까지 오지 않는다.
        if n1.fixed:
            n3.position = QPointF(target)

    def set_f_node3_from_node2(self, rule, length23, angle, sign):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        v1 = normalized(sub_point(n1.position, n2.position))
        v3 = rotate_vector(v1, sign * angle)
        n3.position = add_point(n2.position, mul_point(v3, length23))

    def set_f_node1_from_node2(self, rule, length12, angle, sign):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        v3 = normalized(sub_point(n3.position, n2.position))
        v1 = rotate_vector(v3, -sign * angle)
        n1.position = add_point(n2.position, mul_point(v1, length12))

    def set_f_node1_from_node3(self, rule, length12, angle, sign):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        v3 = normalized(sub_point(n3.position, n2.position))
        v1 = rotate_vector(v3, -sign * angle)
        n1.position = add_point(n2.position, mul_point(v1, length12))

    def set_f_node3_from_node1(self, rule, length23, angle, sign):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]

        v1 = normalized(sub_point(n1.position, n2.position))
        v3 = rotate_vector(v1, sign * angle)
        n3.position = add_point(n2.position, mul_point(v3, length23))

    # ---------- R Drag ----------
    def apply_r_rule_drag(self, rule, moved_id, target):
        n1 = self.nodes[rule.node1]
        n2 = self.nodes[rule.node2]
        n3 = self.nodes[rule.node3]
        state = self.drag["r_states"].get(id(rule))

        if state is None:
            self.capture_r_state(rule)
            state = self.drag["r_states"][id(rule)]

        ratio = state["ratio"]

        # R의 양쪽 중 하나라도 고정이면 양쪽을 움직이지 않는다.
        if n1.fixed or n3.fixed:
            return

        # 중심 Node 이동은 전체 구조를 평행이동한다.
        if moved_id == rule.node2:
            delta = sub_point(target, n2.position)
            n1.position = add_point(n1.position, delta)
            n2.position = QPointF(target)
            n3.position = add_point(n3.position, delta)
            return

        # Node 1 이동
        if moved_id == rule.node1:
            n1.position = QPointF(target)
            v = sub_point(n1.position, n2.position)
            length12 = vector_length(v)

            if length12 < 1e-9:
                return

            direction = normalized(v)
            length23 = length12 * ratio
            n3.position = sub_point(n2.position, mul_point(direction, length23))
            return

        # Node 3 이동
        if moved_id == rule.node3:
            n3.position = QPointF(target)
            v = sub_point(n3.position, n2.position)
            length23 = vector_length(v)

            if length23 < 1e-9:
                return

            direction = normalized(v)
            length12 = length23 / ratio if ratio > 1e-9 else length23
            n1.position = sub_point(n2.position, mul_point(direction, length12))

    # ---------- R -> F ----------
    def recalculate_f_after_r(self, r_rule):
        r_nodes = set(r_rule.nodes())

        for f_rule in self.constraints.f_rules:
            if not f_rule.active:
                continue

            if not r_nodes.intersection(f_rule.nodes()):
                continue

            n1 = self.nodes[f_rule.node1]
            n2 = self.nodes[f_rule.node2]
            n3 = self.nodes[f_rule.node3]

            f_rule.angle = angle_between(n2.position, n1.position, n3.position)

    # ---------- Max Link Length ----------
    def enforce_max_lengths(self):
        for _ in range(3):
            changed = False

            for link in self.links.values():
                n1 = self.nodes[link.node1]
                n2 = self.nodes[link.node2]

                current = distance(n1.position, n2.position)

                if current <= link.max_length + 1e-7:
                    continue

                if n1.fixed and n2.fixed:
                    continue

                direction = normalized(sub_point(n2.position, n1.position))

                if n1.fixed:
                    n2.position = add_point(n1.position, mul_point(direction, link.max_length))
                    changed = True
                    continue

                if n2.fixed:
                    direction = normalized(sub_point(n1.position, n2.position))
                    n1.position = add_point(n2.position, mul_point(direction, link.max_length))
                    changed = True
                    continue

                midpoint = QPointF((n1.position.x() + n2.position.x()) * 0.5, (n1.position.y() + n2.position.y()) * 0.5)
                half = link.max_length * 0.5
                n1.position = sub_point(midpoint, mul_point(direction, half))
                n2.position = add_point(midpoint, mul_point(direction, half))
                changed = True

            if not changed:
                break

    # ---------- Drag End ----------
    def end_drag(self):
        if not self.drag:
            return

        if self.drag["type"] == "node":
            node_id = self.drag["node_id"]

            # R이 적용된 경우 F는 현재 결과 각도로 다시 시작한다.
            r_rule = self.related_r_rule(node_id)

            if r_rule is not None:
                self.recalculate_f_after_r(r_rule)
            else:
                for rule in self.f_rules_for_node(node_id):
                    n1 = self.nodes[rule.node1]
                    n2 = self.nodes[rule.node2]
                    n3 = self.nodes[rule.node3]

                    if node_id == rule.node1:
                        rule.angle = angle_between(n2.position, n1.position, n3.position)

                    elif node_id == rule.node3 and n1.fixed:
                        rule.angle = angle_between(n2.position, n1.position, n3.position)

        self.drag = None

    # ---------- Fixed ----------
    def toggle_fixed(self, node_id):
        node = self.nodes.get(node_id)

        if node is None:
            return

        node.fixed = not node.fixed


# ---------- Node Item ----------
class NodeItem(QGraphicsEllipseItem):
    RADIUS = 6.0

    def __init__(self, model, node_id, editor):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.model = model
        self.node_id = node_id
        self.editor = editor
        self.setAcceptHoverEvents(True)
        self.setZValue(1)
        self.refresh()

    def refresh(self):
        node = self.model.nodes[self.node_id]
        self.setPos(node.position)

        if node.fixed:
            self.setBrush(QBrush(QColor(220, 50, 50)))
        else:
            self.setBrush(QBrush(QColor(245, 245, 245)))

        self.setPen(QPen(QColor(30, 30, 30), 1.5))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.model.begin_node_drag(self.node_id, event.scenePos()):
                event.accept()
                return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        drag = self.model.drag

        if drag and drag["type"] == "node" and drag["node_id"] == self.node_id:
            self.model.update_node_drag(event.scenePos())
            self.editor.refresh_items()
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            drag = self.model.drag

            if drag and drag["type"] == "node" and drag["node_id"] == self.node_id:
                self.model.end_drag()
                self.editor.refresh_items()
                event.accept()
                return

        super().mouseReleaseEvent(event)

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.model.end_drag()
            self.model.toggle_fixed(self.node_id)
            self.editor.refresh_items()
            event.accept()
            return

        super().mouseDoubleClickEvent(event)


# ---------- Link Item ----------
class LinkItem(QGraphicsLineItem):
    def __init__(self, model, link_id, editor):
        super().__init__()
        self.model = model
        self.link_id = link_id
        self.editor = editor
        self.setPen(QPen(QColor(70, 70, 70), 3.0))
        self.setAcceptHoverEvents(True)
        self.setZValue(-1)

    def refresh(self):
        link = self.model.links[self.link_id]
        p1 = self.model.nodes[link.node1].position
        p2 = self.model.nodes[link.node2].position
        self.setLine(p1.x(), p1.y(), p2.x(), p2.y())

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.model.begin_link_drag(self.link_id, event.scenePos()):
                event.accept()
                return

        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        drag = self.model.drag

        if drag and drag["type"] == "link" and drag["link_id"] == self.link_id:
            self.editor.update_link_drag(event.scenePos())
            event.accept()
            return

        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            drag = self.model.drag

            if drag and drag["type"] == "link" and drag["link_id"] == self.link_id:
                self.model.end_drag()
                self.editor.refresh_items()
                event.accept()
                return

        super().mouseReleaseEvent(event)


# ---------- Editor ----------
class PoseEditor(QGraphicsView):
    def __init__(self):
        super().__init__()

        self.model = Object()
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setSceneRect(-500, -400, 1000, 800)
        self.setMinimumSize(900, 700)

        self.node_items = {}
        self.link_items = {}

        self.create_test_model()
        self.create_items()

    # ---------- Test Model ----------
    def create_test_model(self):
        # F 테스트용 Node
        self.model.add_node(1, 100, 150)
        self.model.add_node(2, 220, 150)
        self.model.add_node(3, 320, 210)

        # R 테스트용 Node
        self.model.add_node(4, 100, 400)
        self.model.add_node(5, 220, 400)
        self.model.add_node(6, 340, 400)

        # F Link
        self.model.add_link(1, 2)
        self.model.add_link(2, 3)

        # R Link
        self.model.add_link(4, 5)
        self.model.add_link(5, 6)

        # F(1,2,3)
        self.model.add_f_rule(1, 2, 3)

        # R(4,5,6)
        self.model.add_r_rule(4, 5, 6)

    # ---------- Graphics ----------
    def create_items(self):
        self.scene.clear()
        self.node_items.clear()
        self.link_items.clear()

        for link_id in self.model.links:
            item = LinkItem(self.model, link_id, self)
            self.link_items[link_id] = item
            self.scene.addItem(item)

        for node_id in self.model.nodes:
            item = NodeItem(self.model, node_id, self)
            self.node_items[node_id] = item
            self.scene.addItem(item)

        self.refresh_items()

    def refresh_items(self):
        for item in self.link_items.values():
            item.refresh()

        for item in self.node_items.values():
            item.refresh()

    # ---------- Link Drag ----------
    def update_link_drag(self, mouse_position):
        drag = self.model.drag

        if not drag or drag["type"] != "link":
            return

        link = self.model.links[drag["link_id"]]
        delta = sub_point(mouse_position, drag["mouse_start"])

        node_ids = [link.node1, link.node2]

        for node_id in node_ids:
            node = self.model.nodes[node_id]

            if not node.fixed:
                start = drag["positions"][node_id]
                node.position = add_point(start, delta)

        # R이 포함된 Link는 R을 우선한다.
        r_rules = []

        for rule in self.model.constraints.r_rules:
            if not rule.active:
                continue

            if link.node1 in rule.nodes() or link.node2 in rule.nodes():
                r_rules.append(rule)

        for rule in r_rules:
            movable_id = link.node1 if not self.model.nodes[link.node1].fixed else link.node2

            if self.model.is_node_movable(movable_id):
                self.model.apply_r_rule_drag(rule, movable_id, self.model.nodes[movable_id].position)
                self.model.recalculate_f_after_r(rule)

        self.model.enforce_max_lengths()
        self.refresh_items()


# ---------- Main Window ----------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Pose Editor - Ch-3.5")
        self.resize(1000, 800)

        self.editor = PoseEditor()
        self.setCentralWidget(self.editor)


# ---------- Main ----------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())