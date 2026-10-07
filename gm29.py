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
        node_a.add_line(self), node_b.add_line(self)

class AngleConstraint:
    _next_id = 1
    def __init__(self, center, node_a, node_b):
        self.id, AngleConstraint._next_id = AngleConstraint._next_id, AngleConstraint._next_id + 1
        self.center, self.node_a, self.node_b = center, node_a, node_b
        self.target_angle = angle_between(center.position, node_a.position, node_b.position)

    def store_current_angle(self):
        """드래그 시작 시점에 원래 각도를 보존"""
        self.target_angle = angle_between(self.center.position, self.node_a.position, self.node_b.position)

class ReflectionConstraint:
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id, ReflectionConstraint._next_id = ReflectionConstraint._next_id, ReflectionConstraint._next_id + 1
        self.center, self.node1, self.node3 = center, node1, node3

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
# Constraint Solver (v3.2 - 폭주 완전 방지 및 구동 연동)
# ============================================================
class ConstraintSolver:
    def __init__(self, model): 
        self.model = model
        self.active_drag_node = None

    def on_drag_start(self, node):
        self.active_drag_node = node
        # 드래그 시작 시점의 F 규칙 각도를 기억
        for f in self.model.angle_constraints:
            f.store_current_angle()

    def solve(self, iterations=15):
        for _ in range(iterations):
            # 1. R 대칭 규칙 동기화
            self.solve_reflection_constraints()
            # 2. F 각도 제약 연산
            self.solve_angle_constraints()
            # 3. Line 길이 유지 연산
            self.solve_line_lengths()

    def solve_reflection_constraints(self):
        for r in self.model.reflection_constraints:
            c, n1, n3 = r.center, r.node1, r.node3

            # 3번 노드 드래그 시 -> 1번 노드가 2번(중심) 기준으로 반대편에 대칭 위치
            if self.active_drag_node is n3:
                v3 = n3.position - c.position
                n1.set_position(c.position - v3)

            # 1번 노드 드래그 시 -> 3번 노드가 2번(중심) 기준으로 반대편에 대칭 위치
            elif self.active_drag_node is n1:
                v1 = n1.position - c.position
                n3.set_position(c.position - v1)

            # 2번(중심) 노드 드래그 시 -> 1번과 3번 노드의 고유 상대 거리 및 방위 유지
            elif self.active_drag_node is c:
                pass # 중심 이동 시에는 1,3번이 원래 거리를 유지하며 자연스럽게 추종

    def solve_angle_constraints(self):
        for f in self.model.angle_constraints:
            c, a, b = f.center, f.node_a, f.node_b

            # 현재 형성된 각도와 목표 각도 차이 계산
            curr_angle = angle_between(c.position, a.position, b.position)
            diff = normalize_angle(curr_angle - f.target_angle)

            if abs(diff) < 1e-4: continue

            # 노드가 드래그 중인 주체이면 회전되지 않도록 반대편 노드만 회전 보정
            if self.active_drag_node is a:
                b.set_position(rotate_point(b.position, c.position, -diff))
            elif self.active_drag_node is b:
                a.set_position(rotate_point(a.position, c.position, diff))
            else:
                # 양쪽 다 드래그 중이 아니면 절반씩 나누어 회전
                a.set_position(rotate_point(a.position, c.position, diff * 0.5))
                b.set_position(rotate_point(b.position, c.position, -diff * 0.5))

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
            
            # 드래그 중인 노드는 고정시키고 반대편 노드를 당김
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
        self.editor.solver.on_drag_start(self.node)
        self.setSelected(True)
        event.accept()

    def mouseMoveEvent(self, event):
        if not self.dragging or self.node.fixed: return
        # 드래그 중인 노드 좌표만 정확히 마우스 위치로 업데이트 후 Solver 실행
        self.node.position = QPointF(event.scenePos())
        self.editor.solve_from_interaction()
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or not self.dragging: return
        self.dragging = False
        self.editor.solver.active_drag_node = None
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
        self.setWindowTitle("PyQt6 Node Line Constraint Editor v3.2")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 1. 노드 생성
        ed.add_node(1, 200, 200)       # 조작 노드 1
        ed.add_node(2, 350, 300)       # 중심 노드 2
        ed.add_node(3, 500, 400)       # 조작 노드 3
        ed.add_node(4, 250, 450)       # F 룰 대상 노드 4

        # 2. 선분 생성
        ed.add_line(1, 2)
        ed.add_line(2, 3)
        ed.add_line(2, 4)

        # 3. Rule 적용 (F 규칙: 2번 중심 1, 4번 노드 간 각도 / R 규칙: 2번 중심 1, 3번 대칭)
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