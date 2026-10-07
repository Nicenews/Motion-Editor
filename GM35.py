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
        self.target_angle, self.enabled = angle_between(center.position, node_a.position, node_b.position), True

    def update_target(self):
        self.target_angle = angle_between(self.center.position, self.node_a.position, self.node_b.position)

class ReflectionConstraint:
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id, ReflectionConstraint._next_id = ReflectionConstraint._next_id, ReflectionConstraint._next_id + 1
        self.center, self.node1, self.node3 = center, node1, node3
        self.enabled = True
        
        self.max_dist1 = distance(center.position, node1.position)
        self.max_dist3 = distance(center.position, node3.position)
        self.dist_ratio = self.max_dist3 / self.max_dist1 if self.max_dist1 > 1e-6 else 1.0

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

    def add_rule(self, rule_type, *node_ids):
        rule_type = str(rule_type).upper()
        if rule_type == 'F':
            if len(node_ids) != 3: raise ValueError("F Rule은 3개의 노드 ID가 필요합니다.")
            c, a, b = self.nodes_by_id[node_ids[1]], self.nodes_by_id[node_ids[0]], self.nodes_by_id[node_ids[2]]
            constraint = AngleConstraint(c, a, b)
            self.angle_constraints.append(constraint)
            return constraint
        elif rule_type == 'R':
            if len(node_ids) != 3: raise ValueError("R Rule은 3개의 노드 ID가 필요합니다.")
            c, n1, n3 = self.nodes_by_id[node_ids[1]], self.nodes_by_id[node_ids[0]], self.nodes_by_id[node_ids[2]]
            constraint = ReflectionConstraint(c, n1, n3)
            self.reflection_constraints.append(constraint)
            return constraint
        else:
            raise ValueError(f"지원하지 않는 Rule 타입입니다: {rule_type}")

# ============================================================
# Constraint Solver
# ============================================================
class ConstraintSolver:
    def __init__(self, model):
        self.model = model
        self.active_drag_nodes = set()

    def is_dragging(self, node):
        return node in self.active_drag_nodes

    def weight(self, node):
        if node.fixed or self.is_dragging(node): return 0.0
        return 1.0

    def solve(self, iterations=20):
        # 1. 고정 상태 제약 처리 (1번 고정 시 3번 이동 완전 금지)
        for rc in self.model.reflection_constraints:
            if rc.node1.fixed or rc.node3.fixed:
                if rc.node1.fixed: rc.node3.set_position(rc.node3.position) # 고정 유지
                if rc.node3.fixed: rc.node1.set_position(rc.node1.position)

        for _ in range(iterations):
            changed = False
            
            # [규칙] 2번 노드 드래그 시: 1번 기준 회전 (1-2 길이 유지)
            for rc in self.model.reflection_constraints:
                c, n1 = rc.center, rc.node1
                if self.is_dragging(c) and n1.fixed:
                    # 2번 위치를 1번 기준 고정 거리(1-2 원형 궤적)로 강제 정렬
                    line_12 = next((l for l in self.model.lines if (l.node_a == n1 and l.node_b == c) or (l.node_a == c and l.node_b == n1)), None)
                    if line_12:
                        dx, dy = c.position.x() - n1.position.x(), c.position.y() - n1.position.y()
                        curr_dist = math.hypot(dx, dy)
                        if curr_dist > 1e-6:
                            c.set_position(QPointF(n1.position.x() + (dx / curr_dist) * line_12.rest_length,
                                                   n1.position.y() + (dy / curr_dist) * line_12.rest_length))

            # 선분 길이 해소
            for line in self.model.lines:
                if self.solve_line_length(line): changed = True

            # F 규칙 각도 해소
            for constraint in self.model.angle_constraints:
                if constraint.enabled and self.solve_angle_constraint(constraint): changed = True

            # R 규칙 해소 (상대 노드 고정 시 건너뜀)
            for constraint in self.reflection_constraints_solve():
                if constraint.enabled and self.solve_reflection_constraint(constraint): changed = True

            if not changed: break

    def reflection_constraints_solve(self):
        return [rc for rc in self.model.reflection_constraints if not (rc.node1.fixed or rc.node3.fixed)]

    def solve_line_length(self, line):
        a, b = line.node_a, line.node_b
        w_a, w_b = self.weight(a), self.weight(b)
        if w_a + w_b == 0: return False

        dx, dy = b.position.x() - a.position.x(), b.position.y() - a.position.y()
        current_length = math.hypot(dx, dy)
        if current_length < 1e-6: return False

        error = current_length - line.rest_length
        if abs(error) < 1e-4: return False

        nx, ny = dx / current_length, dy / current_length
        stiffness = 0.8
        corr_a, corr_b = error * (w_a / (w_a + w_b)) * stiffness, error * (w_b / (w_a + w_b)) * stiffness

        a.set_position(QPointF(a.position.x() + nx * corr_a, a.position.y() + ny * corr_a))
        b.set_position(QPointF(b.position.x() - nx * corr_b, b.position.y() - ny * corr_b))
        return True

    def solve_angle_constraint(self, constraint):
        c, a, b = constraint.center, constraint.node_a, constraint.node_b
        
        w_a, w_b = self.weight(a), self.weight(b)
        # 2번(c) 드래그 시: 1번(a) 기준 4번(b) 위치 연쇄 보정
        if self.is_dragging(c):
            w_a, w_b = 0.0, 1.0  # 4번 노드가 F 규칙을 따라 회전하도록 보정

        if w_a + w_b == 0: return False

        curr_angle = angle_between(c.position, a.position, b.position)
        error = normalize_angle(curr_angle - constraint.target_angle)
        if abs(error) < 1e-4: return False

        stiffness = 0.8
        corr_a = error * (w_a / (w_a + w_b)) * stiffness
        corr_b = error * (w_b / (w_a + w_b)) * stiffness

        if w_a > 0: a.set_position(rotate_point(a.position, c.position, corr_a))
        if w_b > 0: b.set_position(rotate_point(b.position, c.position, -corr_b))
        return True

    def solve_reflection_constraint(self, constraint):
        c, n1, n3 = constraint.center, constraint.node1, constraint.node3
        target_pos = QPointF(c.position.x() - (n1.position.x() - c.position.x()) * constraint.dist_ratio,
                             c.position.y() - (n1.position.y() - c.position.y()) * constraint.dist_ratio)
        if distance(n3.position, target_pos) > 1e-4:
            n3.set_position(target_pos)
            return True
        return False

# ============================================================
# View Components
# ============================================================
class NodeItem(QGraphicsEllipseItem):
    RADIUS = 5
    def __init__(self, editor, node):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.editor, self.node, self.dragging = editor, node, False
        self.drag_start_scene, self.drag_start_position = QPointF(), QPointF()
        self.disabled_constraints = []
        self.setBrush(QBrush(QColor("red") if node.fixed else QColor("white")))
        self.setPen(QPen(QColor("black"), 1))
        self.setZValue(1)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setPos(node.position)

    def update_from_model(self):
        self.setPos(self.node.position)
        self.setBrush(QBrush(QColor("red") if self.node.fixed else QColor("white")))

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton: return

        # [규칙 1] 1번 노드가 고정이면 3번 노드 이동 불가
        for rc in self.editor.model.reflection_constraints:
            if self.node == rc.node3 and rc.node1.fixed:
                event.ignore()
                return

        if self.node.fixed: return

        self.dragging = True
        self.editor.solver.active_drag_nodes.add(self.node)
        self.drag_start_scene, self.drag_start_position = event.scenePos(), QPointF(self.node.position)
        self.disabled_constraints.clear()

        # [규칙 3] 4번 노드가 먼저 움직일 시 F 규칙 무시 (종료 후 Update)
        if self.node.id == 4:
            for constraint in self.editor.model.angle_constraints:
                if constraint.enabled:
                    constraint.enabled = False
                    self.disabled_constraints.append(constraint)

        self.setSelected(True)
        event.accept()

    def mouseMoveEvent(self, event):
        if not self.dragging or self.node.fixed: return
        self.node.position = QPointF(self.drag_start_position + (event.scenePos() - self.drag_start_scene))
        self.editor.solve_from_interaction()
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton or not self.dragging: return
        self.dragging = False
        self.editor.solver.active_drag_nodes.discard(self.node)

        # [규칙 3] 4번 노드 이동 후 F 규칙 각도 Update 및 재활성화
        for constraint in self.disabled_constraints:
            constraint.update_target()
            constraint.enabled = True
        self.disabled_constraints.clear()

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

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton: return
        self.dragging, self.previous_scene_position = True, event.scenePos()
        self.editor.solver.active_drag_nodes.add(self.line_model.node_a)
        self.editor.solver.active_drag_nodes.add(self.line_model.node_b)
        self.setSelected(True)
        event.accept()

    def mouseMoveEvent(self, event):
        if not self.dragging: return
        delta = event.scenePos() - self.previous_scene_position
        if abs(delta.x()) < 1e-6 and abs(delta.y()) < 1e-6: return

        a, b = self.line_model.node_a, self.line_model.node_b
        a.set_position(a.position + delta), b.set_position(b.position + delta)
        self.previous_scene_position = QPointF(event.scenePos())
        self.editor.solve_from_interaction()
        event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton: return
        self.dragging = False
        self.editor.solver.active_drag_nodes.discard(self.line_model.node_a)
        self.editor.solver.active_drag_nodes.discard(self.line_model.node_b)
        self.editor.update_graphics()
        self.editor.solve_from_interaction()
        event.accept()

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
        self.setWindowTitle("PyQt6 Node Line Constraint Editor v3.5")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 1. 노드 생성 (1번 노드 기본 고정 설정 테스트 가능)
        ed.add_node(1, 200, 200, fixed=True)  # 1번 노드 고정
        ed.add_node(2, 350, 300)             # 중심 노드 2
        ed.add_node(3, 500, 400)             # 조작 노드 3
        ed.add_node(4, 250, 450)             # F 룰용 노드 4

        # 2. 선분 생성
        ed.add_line(1, 2)
        ed.add_line(2, 3)
        ed.add_line(2, 4)

        # 3. Rule 적용
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