import sys, math
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QBrush, QPen, QColor, QPainterPath, QPainterPathStroker
from PyQt6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsScene, QGraphicsView, QMainWindow, QGraphicsItem

# ============================================================
# Math Utils
# ============================================================
def distance(a, b): return math.hypot(b.x() - a.x(), b.y() - a.y())

def normalize_angle(angle):
    while angle > math.pi: angle -= math.tau
    while angle < -math.pi: angle += math.tau
    return angle

def angle_between(center, a, b):
    return normalize_angle(math.atan2(b.y() - center.y(), b.x() - center.x()) - math.atan2(a.y() - center.y(), a.x() - center.x()))

def rotate_point(point, center, angle):
    dx, dy, c, s = point.x() - center.x(), point.y() - center.y(), math.cos(angle), math.sin(angle)
    return QPointF(center.x() + dx * c - dy * s, center.y() + dx * s + dy * c)

# ============================================================
# Model Classes
# ============================================================
class Node:
    def __init__(self, node_id, x, y, fixed=False):
        self.id = node_id
        self.position, self.fixed, self.lines = QPointF(x, y), fixed, []

    @property
    def inv_mass(self): return 0.0 if self.fixed else 1.0

    def set_position(self, position):
        if not self.fixed: self.position = QPointF(position)

    def set_fixed(self, fixed): self.fixed = fixed

    def add_line(self, line):
        if line not in self.lines: self.lines.append(line)

class Line:
    _next_id = 1
    def __init__(self, node_a, node_b):
        self.id, Line._next_id = Line._next_id, Line._next_id + 1
        self.node_a, self.node_b = node_a, node_b
        self.rest_length = distance(node_a.position, node_b.position)
        self.flexible = False
        node_a.add_line(self), node_b.add_line(self)

class AngleConstraint:
    _next_id = 1
    def __init__(self, center, node_a, node_b):
        self.id, AngleConstraint._next_id = AngleConstraint._next_id, AngleConstraint._next_id + 1
        self.center, self.node_a, self.node_b = center, node_a, node_b
        self.target_angle = angle_between(center.position, node_a.position, node_b.position)
        self.enabled = True

    def update_target(self):
        self.target_angle = angle_between(self.center.position, self.node_a.position, self.node_b.position)

class ReflectionConstraint:
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id, ReflectionConstraint._next_id = ReflectionConstraint._next_id, ReflectionConstraint._next_id + 1
        self.center, self.node1, self.node3 = center, node1, node3
        self.enabled = True

class ShapeModel:
    def __init__(self):
        self.nodes_by_id = {}
        self.lines = []
        self.angle_constraints = []
        self.reflection_constraints = []

    @property
    def nodes(self): return list(self.nodes_by_id.values())

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
# Constraint Solver (v3.1 - 대칭 및 F 규칙 완벽 연동)
# ============================================================
class ConstraintSolver:
    def __init__(self, model): 
        self.model = model
        self.active_drag_node = None
        self.last_drag_pos = None

    def handle_drag_move(self, node, new_pos):
        if self.last_drag_pos is None:
            self.last_drag_pos = QPointF(node.position)
        
        delta = new_pos - self.last_drag_pos
        
        # [핵심 1] 2번(중심) 노드를 드래그할 경우: 대칭 노드(1,3번) 및 구동 노드를 함께 평행 이동
        r_centers = [r.center for r in self.model.reflection_constraints]
        if node in r_centers:
            node.position = QPointF(new_pos)
            for r in self.model.reflection_constraints:
                if r.center is node:
                    r.node1.position = r.node1.position + delta
                    r.node3.position = r.node3.position + delta
            for f in self.model.angle_constraints:
                if f.center is node:
                    # 중심이 이동할 때 다른 피연산 노드가 대칭 노드가 아닌 경우에도 평행 이동
                    if f.node_a not in [r.node1, r.node3 for r in self.model.reflection_constraints]:
                        f.node_a.position = f.node_a.position + delta
                    if f.node_b not in [r.node1, r.node3 for r in self.model.reflection_constraints]:
                        f.node_b.position = f.node_b.position + delta
        else:
            node.position = QPointF(new_pos)

        self.last_drag_pos = QPointF(new_pos)

    def solve(self, iterations=30):
        # 1. R 규칙 처리 (1번/3번 노드 드래그 시 대칭 동기화)
        r_changed = False
        for constraint in self.model.reflection_constraints:
            if constraint.enabled and self.solve_reflection_constraint(constraint):
                r_changed = True

        # 2. 1번/3번 노드 드래그로 각도가 변했을 때만 F 규칙 target_angle 업데이트
        if r_changed and self.active_drag_node not in [r.center for r in self.model.reflection_constraints]:
            for f_constraint in self.model.angle_constraints:
                f_constraint.update_target()

        # 3. F 규칙 (각도 보정) 및 일반 Line 길이 보정
        for _ in range(iterations):
            changed = False
            for constraint in self.model.angle_constraints:
                if constraint.enabled and self.solve_angle_constraint(constraint): 
                    changed = True
            for line in self.model.lines:
                if not line.flexible and self.solve_line_length(line): 
                    changed = True
            if not changed: 
                break

    def solve_reflection_constraint(self, constraint):
        c, n1, n3 = constraint.center, constraint.node1, constraint.node3
        changed = False

        # Node 3 드래그 시 -> Node 1 대칭 이동 (현재 2-3 거리 및 반대 방향 유지)
        if self.active_drag_node is n3:
            v3 = n3.position - c.position
            dist3 = math.hypot(v3.x(), v3.y())
            if dist3 > 1e-6:
                target_pos1 = c.position - v3
                if distance(n1.position, target_pos1) > 1e-4:
                    n1.position = target_pos1
                    changed = True

        # Node 1 드래그 시 -> Node 3 대칭 이동 (현재 2-1 거리 및 반대 방향 유지)
        elif self.active_drag_node is n1:
            v1 = n1.position - c.position
            dist1 = math.hypot(v1.x(), v1.y())
            if dist1 > 1e-6:
                target_pos3 = c.position - v1
                if distance(n3.position, target_pos3) > 1e-4:
                    n3.position = target_pos3
                    changed = True

        return changed

    def solve_angle_constraint(self, constraint):
        c, a, b = constraint.center, constraint.node_a, constraint.node_b
        
        # 현재 각도와 목표 각도 차이
        current_angle = angle_between(c.position, a.position, b.position)
        error = normalize_angle(current_angle - constraint.target_angle)
        if abs(error) < 1e-4: return False

        # 중심 노드가 드래그 중이 아니면 a, b 노드 회전 보정
        w_a, w_b = a.inv_mass, b.inv_mass
        w_sum = w_a + w_b
        if w_sum == 0: return False

        corr_a = error * (w_a / w_sum)
        corr_b = error * (w_b / w_sum)

        a.position = rotate_point(a.position, c.position, -corr_a)
        b.position = rotate_point(b.position, c.position, corr_b)
        return True

    def solve_line_length(self, line):
        a, b = line.node_a, line.node_b
        w_a, w_b = a.inv_mass, b.inv_mass
        w_sum = w_a + w_b
        if w_sum == 0: return False

        dx, dy = b.position.x() - a.position.x(), b.position.y() - a.position.y()
        current_length = math.hypot(dx, dy)
        if current_length < 1e-6: return False

        error = current_length - line.rest_length
        if abs(error) < 1e-4: return False

        nx, ny = dx / current_length, dy / current_length
        corr_a, corr_b = error * (w_a / w_sum), error * (w_b / w_sum)

        a.set_position(QPointF(a.position.x() + nx * corr_a, a.position.y() + ny * corr_a))
        b.set_position(QPointF(b.position.x() - nx * corr_b, b.position.y() - ny * corr_b))
        return True

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
        if event.button() != Qt.MouseButton.LeftButton: return
        self.dragging = True
        self.editor.solver.active_drag_node = self.node
        self.editor.solver.last_drag_pos = event.scenePos()
        self.setSelected(True)
        event.accept()

    def mouseMoveEvent(self, event):
        if not self.dragging: return
        self.editor.solver.handle_drag_move(self.node, event.scenePos())
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
        self.editor, self.line_model, self.dragging = editor, line, False
        self.previous_scene_position = QPointF()
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
        path.moveTo(self.line().p1()), path.lineTo(self.line().p2())
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
        self.solver, self.scene_obj = ConstraintSolver(self.model), EditorScene(self)
        self.setScene(self.scene_obj)
        self.node_items, self.line_items, self.solver_updating = {}, {}, False
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
        self.setWindowTitle("PyQt6 Node Line Constraint Editor v3.1")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 1. 노드 생성 (2번 노드 기본 fixed=False)
        ed.add_node(1, 200, 200)       # 조작 노드 1
        ed.add_node(2, 350, 300)       # 중심 노드 2 (이동 가능)
        ed.add_node(3, 500, 400)       # 조작 노드 3
        ed.add_node(4, 250, 450)       # F 룰 대상 노드 4

        # 2. 선분 생성
        ed.add_line(1, 2)
        ed.add_line(2, 3)
        ed.add_line(2, 4)

        # 3. Rule 적용
        ed.add_rule('F', 1, 2, 4)      # 2번을 중심으로 1번과 4번 간 각도 제약 (F)
        ed.add_rule('R', 1, 2, 3)      # 2번을 중심으로 1번과 3번 간 대칭 제약 (R)

        ed.update_graphics()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()