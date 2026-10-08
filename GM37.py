import sys, math
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QBrush, QPen, QColor, QPainterPath, QPainterPathStroker
from PyQt6.QtWidgets import (QApplication, QGraphicsEllipseItem, QGraphicsLineItem, 
                             QGraphicsScene, QGraphicsView, QMainWindow, QGraphicsItem)

# ============================================================
# Math Utils
# ============================================================
def distance(a, b): 
    return math.hypot(b.x() - a.x(), b.y() - a.y())

def normalize_angle(angle):
    while angle > math.pi: angle -= math.tau
    while angle < -math.pi: angle += math.tau
    return angle

def angle_between(center, a, b):
    # center -> a 벡터 기준으로 center -> b 벡터까지의 회전 각도
    ang_a = math.atan2(a.y() - center.y(), a.x() - center.x())
    ang_b = math.atan2(b.y() - center.y(), b.x() - center.x())
    return normalize_angle(ang_b - ang_a)

def rotate_point(point, center, angle):
    dx, dy = point.x() - center.x(), point.y() - center.y()
    c, s = math.cos(angle), math.sin(angle)
    return QPointF(center.x() + dx * c - dy * s, center.y() + dx * s + dy * c)

# ============================================================
# Model Classes
# ============================================================
class Node:
    def __init__(self, node_id, x, y, fixed=False):
        self.id = node_id
        self.position = QPointF(x, y)
        self.fixed = fixed
        self.lines = []

    @property
    def inv_mass(self): 
        return 0.0 if self.fixed else 1.0

    def set_position(self, position):
        if not self.fixed: 
            self.position = QPointF(position)

    def set_fixed(self, fixed): 
        self.fixed = fixed

    def add_line(self, line):
        if line not in self.lines: 
            self.lines.append(line)

class Line:
    _next_id = 1
    def __init__(self, node_a, node_b):
        self.id = Line._next_id
        Line._next_id += 1
        self.node_a = node_a
        self.node_b = node_b
        self.rest_length = distance(node_a.position, node_b.position)
        node_a.add_line(self)
        node_b.add_line(self)

class AngleConstraint:
    _next_id = 1
    def __init__(self, center, node_a, node_b):
        self.id = AngleConstraint._next_id
        AngleConstraint._next_id += 1
        self.center = center      # 중심 노드 (2번)
        self.node_a = node_a      # 기준 노드 (1번 - 자유 이동)
        self.node_b = node_b      # 종속 노드 (4번 - 각도 추종)
        self.target_angle = angle_between(center.position, node_a.position, node_b.position)

class ReflectionConstraint:
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id = ReflectionConstraint._next_id
        ReflectionConstraint._next_id += 1
        self.center = center
        self.node1 = node1
        self.node3 = node3

class ShapeModel:
    def __init__(self):
        self.nodes_by_id = {}
        self.lines = []
        self.angle_constraints = []
        self.reflection_constraints = []

    @property
    def nodes(self): 
        return list(self.nodes_by_id.values())

    def add_node(self, node_id, x, y, fixed=False):
        node = Node(node_id, x, y, fixed)
        self.nodes_by_id[node_id] = node
        return node

    def add_line(self, node_id1, node_id2):
        node_a, node_b = self.nodes_by_id[node_id1], self.nodes_by_id[node_id2]
        line = Line(node_a, node_b)
        self.lines.append(line)
        return line

    def add_rule(self, rule_type, node_id1, node_id2, node_id3):
        rule_type = str(rule_type).upper()
        if rule_type == 'F':
            c, a, b = self.nodes_by_id[node_id2], self.nodes_by_id[node_id1], self.nodes_by_id[node_id3]
            constraint = AngleConstraint(c, a, b)
            self.angle_constraints.append(constraint)
            return constraint
        elif rule_type == 'R':
            c, n1, n3 = self.nodes_by_id[node_id2], self.nodes_by_id[node_id1], self.nodes_by_id[node_id3]
            constraint = ReflectionConstraint(c, n1, n3)
            self.reflection_constraints.append(constraint)
            return constraint

# ============================================================
# Constraint Solver (v3.4 - R/F 규칙 완전 정상화)
# ============================================================
class ConstraintSolver:
    def __init__(self, model): 
        self.model = model
        self.active_drag_node = None
        self.last_drag_pos = None

    def on_drag_start(self, node, start_pos):
        self.active_drag_node = node
        self.last_drag_pos = QPointF(start_pos)

    def handle_drag_move(self, new_pos):
        if not self.active_drag_node or self.last_drag_pos is None:
            return

        delta = new_pos - self.last_drag_pos
        node = self.active_drag_node

        # 2번(중심) 노드 드래그 시 모든 연결 노드 평행 이동
        r_centers = [r.center for r in self.model.reflection_constraints]
        if node in r_centers:
            node.set_position(new_pos)
            moved_nodes = {node}
            for r in self.model.reflection_constraints:
                if r.center is node:
                    r.node1.set_position(r.node1.position + delta)
                    r.node3.set_position(r.node3.position + delta)
                    moved_nodes.update([r.node1, r.node3])

            for f in self.model.angle_constraints:
                if f.center is node:
                    if f.node_a not in moved_nodes:
                        f.node_a.set_position(f.node_a.position + delta)
                        moved_nodes.add(f.node_a)
                    if f.node_b not in moved_nodes:
                        f.node_b.set_position(f.node_b.position + delta)
                        moved_nodes.add(f.node_b)
        else:
            node.set_position(new_pos)

        self.last_drag_pos = QPointF(new_pos)

    def solve(self, iterations=10):
        for _ in range(iterations):
            # 1. R 대칭 규칙 강제
            self.solve_reflection_constraints()
            # 2. F 각도 고정 규칙 강제
            self.solve_angle_constraints()
            # 3. 선분 길이 유지
            self.solve_line_lengths()

    def solve_reflection_constraints(self):
        for r in self.model.reflection_constraints:
            c, n1, n3 = r.center, r.node1, r.node3

            # 3번 노드를 직접 드래그 중인 경우 -> 1번이 대칭 위치로 이동
            if self.active_drag_node is n3:
                v3 = n3.position - c.position
                n1.set_position(c.position - v3)
            # 그 외 모든 경우(1번 드래그, 2번 드래그 등) -> 3번이 1번의 대칭 위치 추종
            else:
                v1 = n1.position - c.position
                n3.set_position(c.position - v1)

    def solve_angle_constraints(self):
        for f in self.model.angle_constraints:
            c, a, b = f.center, f.node_a, f.node_b

            # 1번(a)은 자유롭게 움직임. 
            # 1번(a)과 2번(c)이 이루는 기준 방위각을 구함
            base_angle = math.atan2(a.position.y() - c.position.y(), a.position.x() - c.position.x())
            
            # 4번(b) 노드가 위치해야 할 목표 절대 각도
            target_abs_angle = base_angle + f.target_angle
            
            # 2번(c)과 4번(b) 사이의 현재 거리 유지
            dist = distance(c.position, b.position)
            if dist < 1e-6:
                dist = 1.0

            # 4번(b) 노드의 위치를 기준 각도에 맞춰 강제 설정 (4번이 1번을 추종)
            if self.active_drag_node is not b:
                new_b_x = c.position.x() + dist * math.cos(target_abs_angle)
                new_b_y = c.position.y() + dist * math.sin(target_abs_angle)
                b.set_position(QPointF(new_b_x, new_b_y))

    def solve_line_lengths(self):
        for line in self.model.lines:
            a, b = line.node_a, line.node_b
            if a.fixed and b.fixed: continue

            dx, dy = b.position.x() - a.position.x(), b.position.y() - a.position.y()
            dist = math.hypot(dx, dy)
            if dist < 1e-6: continue

            err = dist - line.rest_length
            if abs(err) < 1e-4: continue

            nx, ny = dx / dist, dy / dist
            
            if self.active_drag_node is a:
                b.set_position(QPointF(b.position.x() - nx * err, b.position.y() - ny * err))
            elif self.active_drag_node is b:
                a.set_position(QPointF(a.position.x() + nx * err, a.position.y() + ny * err))
            else:
                w_a, w_b = a.inv_mass, b.inv_mass
                w_sum = w_a + w_b
                if w_sum == 0: continue
                a.set_position(QPointF(a.position.x() + nx * err * (w_a / w_sum), a.position.y() + ny * err * (w_a / w_sum)))
                b.set_position(QPointF(b.position.x() - nx * err * (w_b / w_sum), b.position.y() - ny * err * (w_b / w_sum)))

# ============================================================
# View Components
# ============================================================
class NodeItem(QGraphicsEllipseItem):
    RADIUS = 6
    def __init__(self, editor, node):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.editor, self.node, self.dragging = editor, node, False
        self.setBrush(QBrush(QColor("red") if node.fixed else QColor("white")))
        self.setPen(QPen(QColor("black"), 1.5))
        self.setZValue(1)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setPos(node.position)

    def update_from_model(self):
        self.setPos(self.node.position)
        self.setBrush(QBrush(QColor("red") if self.node.fixed else QColor("white")))

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or self.node.fixed: return
        self.dragging = True
        self.editor.solver.on_drag_start(self.node, event.scenePos())
        self.setSelected(True)
        event.accept()

    def mouseMoveEvent(self, event):
        if not self.dragging or self.node.fixed: return
        self.editor.solver.handle_drag_move(event.scenePos())
        self.editor.solve_from_interaction()
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or not self.dragging: return
        self.dragging = False
        self.editor.solver.active_drag_node = None
        self.editor.solver.last_drag_pos = None
        self.editor.solve_from_interaction()
        event.accept()

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.editor.set_node_fixed(self.node, not self.node.fixed)
            event.accept()

class LineItem(QGraphicsLineItem):
    def __init__(self, editor, line):
        super().__init__()
        self.editor, self.line_model = editor, line
        self.setPen(QPen(QColor("black"), 2))
        self.setZValue(-1)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.update_from_model()

    def update_from_model(self):
        a, b = self.line_model.node_a.position, self.line_model.node_b.position
        self.setLine(a.x(), a.y(), b.x(), b.y())
        self.setPos(0, 0)

    def shape(self):
        path = QPainterPath()
        path.moveTo(self.line().p1())
        path.lineTo(self.line().p2())
        stroker = QPainterPathStroker()
        stroker.setWidth(10)
        return stroker.createStroke(path)

class EditorScene(QGraphicsScene):
    def __init__(self, editor):
        super().__init__()
        self.editor = editor
        self.setSceneRect(0, 0, 1000, 700)

class ShapeEditor(QGraphicsView):
    def __init__(self):
        super().__init__()
        self.model = ShapeModel()
        self.solver = ConstraintSolver(self.model)
        self.scene_obj = EditorScene(self)
        self.setScene(self.scene_obj)
        self.node_items = {}
        self.line_items = {}
        self.solver_updating = False
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.DragMode.NoDrag)
        self.setSceneRect(0, 0, 1000, 700)

    def add_node(self, node_id, x, y, fixed=False):
        node = self.model.add_node(node_id, x, y, fixed)
        item = NodeItem(self, node)
        self.node_items[node.id] = item
        self.scene_obj.addItem(item)
        return node

    def add_line(self, node_id1, node_id2):
        line = self.model.add_line(node_id1, node_id2)
        item = LineItem(self, line)
        self.line_items[line.id] = item
        self.scene_obj.addItem(item)
        return line

    def add_rule(self, rule_type, node_id1, node_id2, node_id3):
        return self.model.add_rule(rule_type, node_id1, node_id2, node_id3)

    def set_node_fixed(self, node, fixed):
        node.set_fixed(fixed)
        self.update_graphics()

    def solve_from_interaction(self):
        if self.solver_updating: return
        self.solver_updating = True
        try:
            self.solver.solve()
            self.update_graphics()
        finally:
            self.solver_updating = False

    def update_graphics(self):
        for node in self.model.nodes:
            if item := self.node_items.get(node.id): item.update_from_model()
        for line in self.model.lines:
            if item := self.line_items.get(line.id): item.update_from_model()

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1 / 1.15
        self.scale(factor, factor)

# ============================================================
# Main Application
# ============================================================
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 Node Line Constraint Editor v3.4")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 1. 노드 생성 (1: 기준/자유, 2: 중심, 3: 대칭, 4: 각도 추종)
        ed.add_node(1, 200, 200)       # 1번 노드
        ed.add_node(2, 350, 300)       # 2번 중심 노드
        ed.add_node(3, 500, 400)       # 3번 대칭 노드
        ed.add_node(4, 250, 450)       # 4번 각도 종속 노드

        # 2. 선분 생성
        ed.add_line(1, 2)
        ed.add_line(2, 3)
        ed.add_line(2, 4)

        # 3. Rule 적용 (F 규칙: 2번 중심, 1번 기준 -> 4번 각도 고정 / R 규칙: 2번 중심, 1-3번 대칭)
        ed.add_rule('F', 1, 2, 4)
        ed.add_rule('R', 1, 2, 3)

        ed.update_graphics()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()