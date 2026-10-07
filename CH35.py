# Ch-3.5
import sys
import math
from dataclasses import dataclass
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QBrush, QPen, QColor
from PyQt6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsScene, QGraphicsView, QMainWindow


# ---------- Geometry ----------
def distance(a, b): return math.hypot(a.x() - b.x(), a.y() - b.y())

def angle_between(center, a, b):
    v1x, v1y = a.x() - center.x(), a.y() - center.y()
    v2x, v2y = b.x() - center.x(), b.y() - center.y()
    l1, l2 = math.hypot(v1x, v1y), math.hypot(v2x, v2y)
    if l1 < 1e-9 or l2 < 1e-9: return 0.0
    value = max(-1.0, min(1.0, (v1x * v2x + v1y * v2y) / (l1 * l2)))
    return math.acos(value)

def rotate_vector(vx, vy, angle):
    c, s = math.cos(angle), math.sin(angle)
    return QPointF(vx * c - vy * s, vx * s + vy * c)

def vector_length(v):
    return math.hypot(v.x(), v.y())

def normalized(v, fallback=QPointF(1.0, 0.0)):
    length = vector_length(v)
    if length < 1e-9: return fallback
    return QPointF(v.x() / length, v.y() / length)

def add_point(a, b): return QPointF(a.x() + b.x(), a.y() + b.y())

def sub_point(a, b): return QPointF(a.x() - b.x(), a.y() - b.y())

def mul_point(a, value): return QPointF(a.x() * value, a.y() * value)


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

    def other(self, node_id):
        if self.node1 == node_id: return self.node2
        if self.node2 == node_id: return self.node1
        return None


class FRule:
    def __init__(self, node1, node2, node3, angle=0.0):
        self.node1 = node1
        self.node2 = node2
        self.node3 = node3
        self.angle = angle
        self.active = True

    def nodes(self):
        return self.node1, self.node2, self.node3


class RRule:
    def __init__(self, node1, node2, node3, ratio=1.0):
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

    # ----- Node -----
    def add_node(self, node_id, x, y, fixed=False):
        self.nodes[node_id] = Node(node_id, QPointF(x, y), fixed)

    def get_node(self, node_id):
        return self.nodes.get(node_id)

    def set_node_position(self, node_id, position):
        node = self.get_node(node_id)
        if node is None or node.fixed: return False
        node.position = QPointF(position)
        return True

    # ----- Link -----
    def add_link(self, node1, node2, max_length=None):
        if node1 not in self.nodes or node2 not in self.nodes: return None
        if max_length is None: max_length = distance(self.nodes[node1].position, self.nodes[node2].position)
        link = Link(self.next_link_id, node1, node2, max_length)
        self.links[link.id] = link
        self.next_link_id += 1
        return link

    def get_link_between(self, node1, node2):
        for link in self.links.values():
            if (link.node1 == node1 and link.node2 == node2) or (link.node1 == node2 and link.node2 == node1): return link
        return None

    def connected_links(self, node_id):
        return [link for link in self.links.values() if link.node1 == node_id or link.node2 == node_id]

    # ----- Rule -----
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

    # ----- Mobility -----
    def is_r_side_locked(self, node_id):
        for rule in self.constraints.r_rules:
            if not rule.active: continue
            if node_id == rule.node1 and self.nodes[rule.node1].fixed: return True
            if node_id == rule.node3 and self.nodes[rule.node3].fixed: return True
            if node_id == rule.node1 and self.nodes[rule.node3].fixed: return True
            if node_id == rule.node3 and self.nodes[rule.node1].fixed: return True
        return False

    def is_node_movable(self, node_id):
        node = self.nodes.get(node_id)
        if node is None or node.fixed: return False
        if self.is_r_side_locked(node_id): return False
        return True

    # ----- Drag start -----
    def begin_node_drag(self, node_id, mouse_position):
        if not self.is_node_movable(node_id): return False
        self.drag = {
            "type": "node",
            "node_id": node_id,
            "mouse_start": QPointF(mouse_position),
            "node_start": QPointF(self.nodes[node_id].position),
            "f_rules": [],
            "r_rules": []
        }
        for rule in self.constraints.f_rules:
            if rule.active and node_id in rule.nodes(): self.drag["f_rules"].append(rule)
        for rule in self.constraints.r_rules:
            if rule.active and node_id in rule.nodes(): self.drag["r_rules"].append(rule)
        self.capture_drag_lengths()
        return True

    def begin_link_drag(self, link_id, mouse_position):
        link = self.links.get(link_id)
        if link is None: return False
        n1, n2 = self.nodes[link.node1], self.nodes[link.node2]
        if n1.fixed and n2.fixed: return False
        self.drag = {
            "type": "link",
            "link_id": link_id,
            "mouse_start": QPointF(mouse_position),
            "node_positions": {n1.id: QPointF(n1.position), n2.id: QPointF(n2.position)}
        }
        self.capture_drag_lengths()
        return True

    def capture_drag_lengths(self):
        self.drag["lengths"] = {}
        for link in self.links.values():
            self.drag["lengths"][link.id] = distance(self.nodes[link.node1].position, self.nodes[link.node2].position)

    # ----- Node drag -----
    def update_node_drag(self, mouse_position):
        if not self.drag or self.drag["type"] != "node": return
        node_id = self.drag["node_id"]
        mouse_delta = sub_point(mouse_position, self.drag["mouse_start"])
        target = add_point(self.drag["node_start"], mouse_delta)

        node = self.nodes[node_id]
        if node.fixed: return

        # R이 관련되면 R을 최우선으로 처리한다.
        active_r = self.find_active_r_rule(node_id)
        if active_r is not None:
            self.apply_r_drag(active_r, node_id, target)
            self.update_f_angles_after_r(active_r)
            self.enforce_max_lengths()
            return

        # F의 일반적인 처리
        active_f = self.find_active_f_rule(node_id)
        if active_f is not None:
            self.apply_f_drag(active_f, node_id, target)
        else:
            node.position = QPointF(target)

        self.enforce_max_lengths()

    def find_active_r_rule(self, node_id):
        for rule in self.drag["r_rules"]:
            if rule.active and node_id in rule.nodes(): return rule
        return None

    def find_active_f_rule(self, node_id):
        for rule in self.drag["f_rules"]:
            if rule.active and node_id in rule.nodes(): return rule
        return None

    # ----- F Rule -----
    def apply_f_drag(self, rule, moved_id, target):
        n1, n2, n3 = self.nodes[rule.node1], self.nodes[rule.node2], self.nodes[rule.node3]

        # Node 1이 먼저 움직이는 경우에는 F를 잠시 무시한다.
        if moved_id == rule.node1:
            n1.position = QPointF(target)
            return

        # Node 3가 먼저 움직이고 Node 1이 고정이면 F를 무시하고 각도를 갱신한다.
        if moved_id == rule.node3 and n1.fixed:
            n3.position = QPointF(target)
            return

        if moved_id == rule.node2:
            n2.position = QPointF(target)
            self.solve_f_from_center(rule)
            return

        if moved_id == rule.node3:
            n3.position = QPointF(target)
            self.solve_f_from_node3(rule)
            return

    def solve_f_from_center(self, rule):
        n1, n2, n3 = self.nodes[rule.node1], self.nodes[rule.node2], self.nodes[rule.node3]
        length12 = self.drag["lengths"].get(self.get_link_id(rule.node1, rule.node2), distance(n1.position, n2.position))
        length23 = self.drag["lengths"].get(self.get_link_id(rule.node2, rule.node3), distance(n2.position, n3.position))

        if not n1.fixed:
            old = normalized(sub_point(n1.position, n2.position))
            n1.position = add_point(n2.position, mul_point(old, length12))

        if not n3.fixed:
            direction = normalized(sub_point(n1.position, n2.position), QPointF(1, 0))
            direction = rotate_vector(direction.x(), direction.y(), rule.angle)
            n3.position = add_point(n2.position, mul_point(direction, length23))

    def solve_f_from_node3(self, rule):
        n1, n2, n3 = self.nodes[rule.node1], self.nodes[rule.node2], self.nodes[rule.node3]
        if n2.fixed:
            center = n2.position
            v3 = normalized(sub_point(n3.position, center))
            v1 = rotate_vector(v3.x(), v3.y(), -rule.angle)
            length12 = self.drag["lengths"].get(self.get_link_id(rule.node1, rule.node2), distance(n1.position, center))
            if not n1.fixed: n1.position = add_point(center, mul_point(v1, length12))
        else:
            n3.position = QPointF(n3.position)

    # ----- R Rule -----
    def apply_r_drag(self, rule, moved_id, target):
        n1, n2, n3 = self.nodes[rule.node1], self.nodes[rule.node2], self.nodes[rule.node3]

        # R에서는 Node 1과 Node 3 중 한쪽이 고정이면 양쪽 모두 이동하지 않는다.
        if n1.fixed or n3.fixed: return

        if moved_id == rule.node1:
            n1.position = QPointF(target)
            v = sub_point(n1.position, n2.position)
            length12 = vector_length(v)
            if length12 < 1e-9: return
            direction = normalized(v)
            length23 = length12 * rule.ratio
            n3.position = sub_point(n2.position, mul_point(direction, length23))
            return

        if moved_id == rule.node3:
            n3.position = QPointF(target)
            v = sub_point(n3.position, n2.position)
            length23 = vector_length(v)
            if length23 < 1e-9: return
            direction = normalized(v)
            length12 = length23 / rule.ratio if rule.ratio > 1e-9 else length23
            n1.position = sub_point(n2.position, mul_point(direction, length12))
            return

        if moved_id == rule.node2:
            delta = sub_point(target, n2.position)
            n2.position = QPointF(target)
            n1.position = add_point(n1.position, delta)
            n3.position = add_point(n3.position, delta)

    def update_f_angles_after_r(self, r_rule):
        affected = set()
        for f_rule in self.constraints.f_rules:
            if any(node_id in f_rule.nodes() for node_id in r_rule.nodes()):
                affected.add(f_rule)
        for rule in affected:
            n1, n2, n3 = self.nodes[rule.node1], self.nodes[rule.node2], self.nodes[rule.node3]
            rule.angle = angle_between(n2.position, n1.position, n3.position)

    # ----- Link length -----
    def get_link_id(self, node1, node2):
        link = self.get_link_between(node1, node2)
        return link.id if link else None

    def enforce_max_lengths(self):
        for _ in range(4):
            changed = False
            for link in self.links.values():
                n1, n2 = self.nodes[link.node1], self.nodes[link.node2]
                current = distance(n1.position, n2.position)
                if current <= link.max_length + 1e-7: continue
                if n1.fixed and n2.fixed: continue
                if n1.fixed:
                    direction = normalized(sub_point(n2.position, n1.position))
                    n2.position = add_point(n1.position, mul_point(direction, link.max_length))
                    changed = True
                elif n2.fixed:
                    direction = normalized(sub_point(n1.position, n2.position))
                    n1.position = add_point(n2.position, mul_point(direction, link.max_length))
                    changed = True
                else:
                    direction = normalized(sub_point(n2.position, n1.position))
                    midpoint = QPointF((n1.position.x() + n2.position.x()) * 0.5, (n1.position.y() + n2.position.y()) * 0.5)
                    half = link.max_length * 0.5
                    n1.position = sub_point(midpoint, mul_point(direction, half))
                    n2.position = add_point(midpoint, mul_point(direction, half))
                    changed = True
            if not changed: break

    # ----- Drag end -----
    def end_drag(self):
        if not self.drag: return

        if self.drag["type"] == "node":
            node_id = self.drag["node_id"]
            for rule in self.drag["f_rules"]:
                # Node 1의 자유 이동 또는 Node 3의 특수 이동 종료 시 현재 각도를 새 기준각으로 저장한다.
                if node_id == rule.node1 or (node_id == rule.node3 and self.nodes[rule.node1].fixed):
                    n1, n2, n3 = self.nodes[rule.node1], self.nodes[rule.node2], self.nodes[rule.node3]
                    rule.angle = angle_between(n2.position, n1.position, n3.position)

        self.drag = None

    # ----- Toggle fixed -----
    def toggle_fixed(self, node_id):
        node = self.nodes.get(node_id)
        if node is None: return
        node.fixed = not node.fixed


# ---------- Graphics ----------
class NodeItem(QGraphicsEllipseItem):
    RADIUS = 6.0

    def __init__(self, model, node_id, editor):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.model = model
        self.node_id = node_id
        self.editor = editor
        self.setFlag(QGraphicsEllipseItem.GraphicsItemFlag.ItemIsMovable, False)
        self.setAcceptHoverEvents(True)
        self.refresh()

    def refresh(self):
        node = self.model.nodes[self.node_id]
        self.setPos(node.position)
        self.setBrush(QBrush(QColor(220, 50, 50) if node.fixed else QColor(245, 245, 245)))
        self.setPen(QPen(QColor(30, 30, 30), 1.5))

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            if self.model.begin_node_drag(self.node_id, event.scenePos()):
                self.editor.active_item = self
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.model.drag and self.model.drag["type"] == "node" and self.model.drag["node_id"] == self.node_id:
            self.model.update_node_drag(event.scenePos())
            self.editor.refresh_items()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.model.drag and self.model.drag["type"] == "node":
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
                self.editor.active_item = self
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.model.drag and self.model.drag["type"] == "link" and self.model.drag["link_id"] == self.link_id:
            self.editor.update_link_drag(event.scenePos())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.model.drag and self.model.drag["type"] == "link":
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
        self.setRenderHint(self.renderHints())
        self.setSceneRect(-500, -400, 1000, 800)
        self.node_items = {}
        self.link_items = {}
        self.active_item = None
        self.create_test_model()
        self.create_items()

    def create_test_model(self):
        # F 테스트
        self.model.add_node(1, 100, 100)
        self.model.add_node(2, 220, 100)
        self.model.add_node(3, 320, 170)

        # R 테스트
        self.model.add_node(4, 150, 350)
        self.model.add_node(5, 270, 350)
        self.model.add_node(6, 390, 350)

        self.model.add_link(1, 2)
        self.model.add_link(2, 3)
        self.model.add_link(4, 5)
        self.model.add_link(5, 6)

        self.model.add_f_rule(1, 2, 3)
        self.model.add_r_rule(4, 5, 6)

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
        for item in self.link_items.values(): item.refresh()
        for item in self.node_items.values(): item.refresh()

    def update_link_drag(self, mouse_position):
        drag = self.model.drag
        if not drag or drag["type"] != "link": return

        link = self.model.links[drag["link_id"]]
        delta = sub_point(mouse_position, drag["mouse_start"])

        n1 = self.model.nodes[link.node1]
        n2 = self.model.nodes[link.node2]

        p1 = add_point(drag["node_positions"][n1.id], delta)
        p2 = add_point(drag["node_positions"][n2.id], delta)

        if not n1.fixed: n1.position = QPointF(p1)
        if not n2.fixed: n2.position = QPointF(p2)

        # 링크 이동도 R을 최우선으로 처리한다.
        for rule in self.model.constraints.r_rules:
            if not rule.active: continue
            if link.node1 in rule.nodes() or link.node2 in rule.nodes():
                moved = link.node1 if not self.model.nodes[link.node1].fixed else link.node2
                if self.model.is_node_movable(moved):
                    self.model.apply_r_drag(rule, moved, self.model.nodes[moved].position)
                    self.model.update_f_angles_after_r(rule)

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())