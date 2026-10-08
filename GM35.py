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
    ang_a = math.atan2(a.y() - center.y(), a.x() - center.x())
    ang_b = math.atan2(b.y() - center.y(), b.x() - center.x())
    return normalize_angle(ang_b - ang_a)

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
        self.max_length = self.rest_length  # 최대 길이 제한
        node_a.add_line(self)
        node_b.add_line(self)

class AngleConstraint:
    _next_id = 1
    def __init__(self, center, node_a, node_b):
        self.id = AngleConstraint._next_id
        AngleConstraint._next_id += 1
        self.center = center      # 2번 중심 노드
        self.node_a = node_a      # 1번 기준 노드
        self.node_b = node_b      # 4번 각도 종속 노드
        self.update_target_angle()

    def update_target_angle(self):
        """1번 노드 이동 완료 시 호출되어 각도를 새로 Update"""
        self.target_angle = angle_between(self.center.position, self.node_a.position, self.node_b.position)

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

    def add_rule(self, rule_type, *node_ids):
        rule_type = str(rule_type).upper()
        if rule_type == 'F':
            if len(node_ids) != 3: raise ValueError("F Rule은 3개의 노드 ID가 필요합니다.")
            a, c, b = self.nodes_by_id[node_ids[0]], self.nodes_by_id[node_ids[1]], self.nodes_by_id[node_ids[2]]
            constraint = AngleConstraint(c, a, b)
            self.angle_constraints.append(constraint)
            return constraint
        elif rule_type == 'R':
            if len(node_ids) != 3: raise ValueError("R Rule은 3개의 노드 ID가 필요합니다.")
            a, c, b = self.nodes_by_id[node_ids[0]], self.nodes_by_id[node_ids[1]], self.nodes_by_id[node_ids[2]]
            constraint = ReflectionConstraint(c, a, b)
            self.reflection_constraints.append(constraint)
            return constraint

# ============================================================
# Constraint Solver
# ============================================================
class ConstraintSolver:
    def __init__(self, model): 
        self.model = model
        self.active_drag_node = None
        self.last_drag_pos = None

    def on_drag_start(self, node, start_pos):
        self.active_drag_node = node
        self.last_drag_pos = QPointF(start_pos)

    def on_drag_end(self):
        """드래그 종료 시 1번 노드가 이동한 경우 F 규칙의 각도를 Update"""
        if self.active_drag_node:
            for f in self.model.angle_constraints:
                if self.active_drag_node is f.node_a:
                    f.update_target_angle()
        self.active_drag_node = None
        self.last_drag_pos = None

    def handle_drag_move(self, new_pos):
        if not self.active_drag_node or self.last_drag_pos is None:
            return

        delta = new_pos - self.last_drag_pos
        node = self.active_drag_node

        # 2번(중심) 노드를 드래그할 경우: 모든 연결 노드를 함께 평행 이동
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
            # 1) R 규칙 (대칭 및 최대 길이 구속 - 우선 적용)
            self.solve_reflection_constraints()
            # 2) F 규칙 (고정 각도 연동)
            self.solve_angle_constraints()
            # 3) 선분 길이 유지
            self.solve_line_lengths()

    def solve_reflection_constraints(self):
        """R 규칙: 대칭 이동 및 최대 길이 제한"""
        for r in self.model.reflection_constraints:
            c, n1, n3 = r.center, r.node1, r.node3

            # 최대 길이 범위 검사
            for n in (n1, n3):
                if self.active_drag_node is n:
                    v = n.position - c.position
                    dist = math.hypot(v.x(), v.y())
                    max_len = None
                    for line in self.model.lines:
                        if (line.node_a is c and line.node_b is n) or (line.node_a is n and line.node_b is c):
                            max_len = line.max_length
                            break
                    if max_len and dist > max_len:
                        scale = max_len / dist
                        n.set_position(c.position + v * scale)

            # 2번 중심 노드 기준 대칭 위치 적용
            if self.active_drag_node is n3:
                v3 = n3.position - c.position
                n1.set_position(c.position - v3)
            else:
                v1 = n1.position - c.position
                n3.set_position(c.position - v1)

    def solve_angle_constraints(self):
        """F 규칙: 각도 고정 및 유지"""
        for f in self.model.angle_constraints:
            c, a, b = f.center, f.node_a, f.node_b

            # 1번 노드 이동 중에는 각도 제약 무시
            if self.active_drag_node is a:
                continue

            base_angle = math.atan2(a.position.y() - c.position.y(), a.position.x() - c.position.x())
            target_abs_angle = base_angle + f.target_angle
            
            line_len = 1.0
            for l in self.model.lines:
                if (l.node_a is c and l.node_b is b) or (l.node_a is b and l.node_b is c):
                    line_len = l.rest_length
                    break

            if self.active_drag_node is not b:
                new_b_x = c.position.x() + line_len * math.cos(target_abs_angle)
                new_b_y = c.position.y() + line_len * math.sin(target_abs_angle)
                b.set_position(QPointF(new_b_x, new_b_y))

    def solve_line_lengths(self):
        """기본 선분 길이 복원"""
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
        self.editor.solver.on_drag_end()
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

    def add_rule(self, rule_type, *node_ids):
        return self.model.add_rule(rule_type, *node_ids)

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
        self.setWindowTitle("PyQt6 Node Line Constraint Editor")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 노드 생성 (1: 기준 노드, 2: 중심 노드, 3: 대칭 노드, 4: 각도 추종 노드)
        ed.add_node(1, 200, 200)
        ed.add_node(2, 350, 300)
        ed.add_node(3, 500, 400)
        ed.add_node(4, 250, 450)

        # 선분 생성
        ed.add_line(1, 2)
        ed.add_line(2, 3)
        ed.add_line(2, 4)

        # 규칙 적용 (R: 1-2-3 대칭 고정 / F: 1-2-4 각도 고정)
        ed.add_rule('R', 1, 2, 3)
        ed.add_rule('F', 1, 2, 4)

        ed.update_graphics()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()