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
    """
    R Rule: center(node2)를 중심으로 node1과 node3이 180도 반대 방향 대칭 이동
    - 초기 거리를 최대 거리(max_dist)로 설정 (초과 불가)
    - 양쪽 모두 길이 수축 가능 및 상대편 노드도 비례 수축
    """
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id, ReflectionConstraint._next_id = ReflectionConstraint._next_id, ReflectionConstraint._next_id + 1
        self.center, self.node1, self.node3 = center, node1, node3
        self.enabled = True
        
        # 최초 최대 거리 저장
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

    def reflection_followers(self, node):
        """node를 드래그할 때 R 규칙에 의해 함께 움직이게 되는 노드 목록 (node가 1 또는 3번일 때만)"""
        followers = []
        for rc in self.reflection_constraints:
            if not rc.enabled: continue
            if node is rc.node1 and rc.node3 not in followers: followers.append(rc.node3)
            elif node is rc.node3 and rc.node1 not in followers: followers.append(rc.node1)
        return followers

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
        else:
            raise ValueError(f"지원하지 않는 Rule 타입입니다: {rule_type}")

# ============================================================
# Constraint Solver
# ============================================================
class ConstraintSolver:
    """
    위치 기반(PBD) 제약 솔버.
    - 선 길이: 항상 '현재 rest_length'를 유지 (R 드래그 중에는 중심-1/3 선만 예외, 끝나면 현재 길이로 갱신)
    - F 규칙: 저장된 각도 유지
    - R 규칙: 1·3번이 이동 시작점이면 반대편이 대칭 이동(거리 축소 가능, 최초 거리 초과 불가),
              그 외에는 3번이 1번의 대칭 위치를 유지
    """
    def __init__(self, model):
        self.model = model
        self.active_drag_node = None
        self._followers = []       # R 규칙에 의해 종속적으로 움직이는 노드
        self._released_lines = []  # 이번 solve에서 길이 제약을 걸지 않는 선

    def weight(self, node):
        """0이면 다른 제약(선/각도)에 의해 밀려나지 않는 노드: 고정 / 드래그 중 / R 종속 노드"""
        if node.fixed or node is self.active_drag_node or node in self._followers: return 0.0
        return 1.0

    def prepare(self):
        drag = self.active_drag_node
        self._followers, self._released_lines = [], []
        for rc in self.model.reflection_constraints:
            if not rc.enabled: continue
            start_is_r = drag is rc.node1 or drag is rc.node3
            follower = rc.node1 if drag is rc.node3 else rc.node3
            if follower not in self._followers: self._followers.append(follower)
            for line in self.model.lines:
                ends = (line.node_a, line.node_b)
                if rc.center not in ends: continue
                # R 드래그 중: (중심-1), (중심-3) 선 길이 가변 / 그 외: 종속 노드(3)의 선은 R이 대신 결정
                if (start_is_r and (rc.node1 in ends or rc.node3 in ends)) or follower in ends:
                    if line not in self._released_lines: self._released_lines.append(line)

    def solve(self, iterations=60):
        self.prepare()
        for _ in range(iterations):
            changed = False
            for line in self.model.lines:
                if line not in self._released_lines and self.solve_line_length(line): changed = True
            for constraint in self.model.angle_constraints:
                if constraint.enabled and self.solve_angle_constraint(constraint): changed = True
            for constraint in self.model.reflection_constraints:
                if constraint.enabled and self.solve_reflection_constraint(constraint): changed = True
            if not changed: break

        # 가변이었던 선은 '현재 길이'를 새 기준 길이로 저장 (이후 이 길이가 유지됨)
        for line in self._released_lines:
            line.rest_length = distance(line.node_a.position, line.node_b.position)

    def solve_line_length(self, line):
        a, b = line.node_a, line.node_b
        w_a, w_b = self.weight(a), self.weight(b)
        if w_a + w_b == 0:
            # 드래그 노드가 고정 노드와 직접 연결된 경우: 드래그 노드를 고정 노드 기준 원 위로 끌어당김
            if a is self.active_drag_node and b.fixed: w_a = 1.0
            elif b is self.active_drag_node and a.fixed: w_b = 1.0
            else: return False
        w_sum = w_a + w_b

        dx, dy = b.position.x() - a.position.x(), b.position.y() - a.position.y()
        current_length = math.hypot(dx, dy)
        if current_length < 1e-6: return False

        error = current_length - line.rest_length
        if abs(error) < 1e-5: return False

        nx, ny = dx / current_length, dy / current_length
        stiffness = 0.8
        corr_a, corr_b = error * (w_a / w_sum) * stiffness, error * (w_b / w_sum) * stiffness

        a.set_position(QPointF(a.position.x() + nx * corr_a, a.position.y() + ny * corr_a))
        b.set_position(QPointF(b.position.x() - nx * corr_b, b.position.y() - ny * corr_b))
        return True

    def solve_angle_constraint(self, constraint):
        c, a, b = constraint.center, constraint.node_a, constraint.node_b
        w_a, w_b = self.weight(a), self.weight(b)
        w_sum = w_a + w_b
        if w_sum == 0: return False

        error = normalize_angle(angle_between(c.position, a.position, b.position) - constraint.target_angle)
        if abs(error) < 1e-5: return False

        stiffness = 0.5
        corr_a, corr_b = error * (w_a / w_sum) * stiffness, error * (w_b / w_sum) * stiffness

        a.set_position(rotate_point(a.position, c.position, corr_a))
        b.set_position(rotate_point(b.position, c.position, -corr_b))
        return True

    def solve_reflection_constraint(self, constraint):
        c, n1, n3 = constraint.center, constraint.node1, constraint.node3
        changed = False

        # 노드 3을 움직이고 있는 경우 -> Node 3 기준으로 Node 1 반응
        if self.active_drag_node is n3:
            v3_x = n3.position.x() - c.position.x()
            v3_y = n3.position.y() - c.position.y()
            curr_dist3 = math.hypot(v3_x, v3_y)
            if curr_dist3 < 1e-6: return False

            # Max Distance Clamping
            if curr_dist3 > constraint.max_dist3:
                v3_x = (v3_x / curr_dist3) * constraint.max_dist3
                v3_y = (v3_y / curr_dist3) * constraint.max_dist3
                n3.set_position(QPointF(c.position.x() + v3_x, c.position.y() + v3_y))
                changed = True

            # Node 1 위치 계산 (Node 3의 반대 방향 180도 + 비례 거리 적용)
            target_pos = QPointF(c.position.x() - v3_x / constraint.dist_ratio,
                                 c.position.y() - v3_y / constraint.dist_ratio)
            if distance(n1.position, target_pos) > 1e-5:
                n1.set_position(target_pos)
                changed = True

        # 노드 1을 움직이고 있는 경우 -> Node 1 기준으로 Node 3 반응
        elif self.active_drag_node is n1:
            v1_x = n1.position.x() - c.position.x()
            v1_y = n1.position.y() - c.position.y()
            curr_dist1 = math.hypot(v1_x, v1_y)
            if curr_dist1 < 1e-6: return False

            # Max Distance Clamping
            if curr_dist1 > constraint.max_dist1:
                v1_x = (v1_x / curr_dist1) * constraint.max_dist1
                v1_y = (v1_y / curr_dist1) * constraint.max_dist1
                n1.set_position(QPointF(c.position.x() + v1_x, c.position.y() + v1_y))
                changed = True

            # Node 3 위치 계산 (Node 1의 반대 방향 180도 + 비례 거리 적용)
            target_pos = QPointF(c.position.x() - v1_x * constraint.dist_ratio,
                                 c.position.y() - v1_y * constraint.dist_ratio)
            if distance(n3.position, target_pos) > 1e-5:
                n3.set_position(target_pos)
                changed = True

        # 이동 시작점이 1·3번이 아닌 경우 -> 길이는 현재 기준 유지, Node 3은 Node 1의 대칭 위치만 유지
        else:
            target_pos = QPointF(c.position.x() - (n1.position.x() - c.position.x()) * constraint.dist_ratio,
                                 c.position.y() - (n1.position.y() - c.position.y()) * constraint.dist_ratio)
            if distance(n3.position, target_pos) > 1e-5:
                n3.set_position(target_pos)
                changed = True

        return changed

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
        if event.button() != Qt.MouseButton.LeftButton or self.node.fixed: return
        self.dragging = True
        self.editor.solver.active_drag_node = self.node
        self.drag_start_scene, self.drag_start_position = event.scenePos(), QPointF(self.node.position)
        self.disabled_constraints.clear()

        # 1·3번(R 이동 시작점)을 끌 때만, 함께 움직이는 노드와 겹치는 F 규칙을 끄고 놓을 때 각도를 새로 저장.
        # 그 외 노드(예: 4번)를 끌 때는 F 규칙이 계속 적용되어 현재 각도를 유지한다.
        followers = self.editor.model.reflection_followers(self.node)
        if followers:
            moved_nodes = [self.node] + followers
            for constraint in self.editor.model.angle_constraints:
                involved = (constraint.node_a, constraint.node_b, constraint.center)
                if constraint.enabled and any(n in involved for n in moved_nodes):
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
        self.editor.solver.active_drag_node = None

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
        self.editor.update_graphics(), self.editor.solve_from_interaction()
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
        self.setWindowTitle("PyQt6 Node Line Constraint Editor v3.0")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 1. 노드 생성
        ed.add_node(1, 200, 200)       # 조작 노드 1
        ed.add_node(2, 350, 300)       # 중심 노드 2 (4번을 끌면 딸려옴, 더블클릭으로 고정 가능)
        ed.add_node(3, 500, 400)       # 조작 노드 3 (2번 기준 1번의 180도 대칭 위치)
        ed.add_node(4, 250, 450)       # F 룰용 노드 4

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