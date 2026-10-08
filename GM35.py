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

# ============================================================
# Model Classes
# ============================================================
class Node:
    def __init__(self, node_id, x, y, fixed=False):
        self.id = node_id
        self.position = QPointF(x, y)
        self.fixed = fixed
        self.lines = []

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
        self.center = center      # 2번 노드
        self.node_a = node_a      # 1번 노드
        self.node_b = node_b      # 3번 노드
        self.update_target_state()

    def update_target_state(self):
        v_cb = self.node_b.position - self.center.position
        ang_cb = math.atan2(v_cb.y(), v_cb.x())
        v_ca = self.node_a.position - self.center.position
        ang_ca = math.atan2(v_ca.y(), v_ca.x())
        
        self.rel_angle = normalize_angle(ang_ca - ang_cb)
        self.target_dist = distance(self.center.position, self.node_a.position)

class ReflectionConstraint:
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id = ReflectionConstraint._next_id
        ReflectionConstraint._next_id += 1
        self.center = center    # 2번 노드
        self.node1 = node1      # 1번 노드
        self.node3 = node3      # 3번 노드
        
        # 최초 1-2번 대비 2-3번 라인의 길이 비율 계산 (Len(2-3) / Len(1-2))
        len_12 = distance(center.position, node1.position)
        len_23 = distance(center.position, node3.position)
        self.ratio_3_over_1 = len_23 / len_12 if len_12 > 1e-6 else 1.0

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
        if self.active_drag_node:
            for f in self.model.angle_constraints:
                f.update_target_state()
        self.active_drag_node = None
        self.last_drag_pos = None

    def handle_drag_move(self, new_pos):
        if not self.active_drag_node or self.last_drag_pos is None:
            return

        node = self.active_drag_node
        node.set_position(new_pos)
        self.last_drag_pos = QPointF(new_pos)

    def solve(self, iterations=5):
        for _ in range(iterations):
            self.solve_reflection_constraints()
            self.solve_angle_constraints()

    def solve_reflection_constraints(self):
        """R 규칙: 최초 길이 비율(ratio_3_over_1)에 따른 각도 및 비율 대칭 반영"""
        for r in self.model.reflection_constraints:
            c, n1, n3 = r.center, r.node1, r.node3

            if self.active_drag_node is n1:
                # 1번 노드가 이동: 1-2의 현재 방향과 길이에 초기 비율 반영하여 3번 위치 결정
                v1 = n1.position - c.position
                len1 = math.hypot(v1.x(), v1.y())
                if len1 < 1e-6: continue
                
                # 반대 방향 벡터 (-v1/len1) * (len1 * ratio)
                dir_x, dir_y = -v1.x() / len1, -v1.y() / len1
                target_len3 = len1 * r.ratio_3_over_1
                
                n3.set_position(QPointF(c.position.x() + dir_x * target_len3, 
                                        c.position.y() + dir_y * target_len3))

            elif self.active_drag_node is n3:
                # 3번 노드가 이동: 2-3의 현재 방향과 길이에 역비율(1 / ratio) 반영하여 1번 위치 결정
                v3 = n3.position - c.position
                len3 = math.hypot(v3.x(), v3.y())
                if len3 < 1e-6: continue

                dir_x, dir_y = -v3.x() / len3, -v3.y() / len3
                target_len1 = len3 / r.ratio_3_over_1 if r.ratio_3_over_1 > 1e-6 else len3

                n1.set_position(QPointF(c.position.x() + dir_x * target_len1, 
                                        c.position.y() + dir_y * target_len1))

    def solve_angle_constraints(self):
        """F 규칙: 2번 또는 3번 노드가 움직일 때 1번 노드를 지정된 위치/길이로 추종"""
        for f in self.model.angle_constraints:
            c, a, b = f.center, f.node_a, f.node_b

            if self.active_drag_node is a:
                continue

            if self.active_drag_node in (c, b):
                v_cb = b.position - c.position
                ang_cb = math.atan2(v_cb.y(), v_cb.x())
                target_ang_a = ang_cb + f.rel_angle

                new_a_x = c.position.x() + f.target_dist * math.cos(target_ang_a)
                new_a_y = c.position.y() + f.target_dist * math.sin(target_ang_a)
                a.set_position(QPointF(new_a_x, new_a_y))

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

        # 초기 배치 (1-2 거리: 약 180.28, 2-3 거리: 약 212.13 -> 비율 자동 계산 및 유지)
        ed.add_node(1, 200, 200)
        ed.add_node(2, 350, 300)
        ed.add_node(3, 500, 450)

        # 선분 생성
        ed.add_line(1, 2)
        ed.add_line(2, 3)

        # 규칙 적용
        ed.add_rule('R', 1, 2, 3)
        ed.add_rule('F', 1, 2, 3)

        ed.update_graphics()

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
