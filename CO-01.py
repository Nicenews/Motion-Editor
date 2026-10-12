
import math
import sys
from dataclasses import dataclass, field
from PyQt6.QtCore import QLineF, QPointF, QRectF, Qt
from PyQt6.QtGui import QBrush, QColor, QPainter, QPen
from PyQt6.QtWidgets import QApplication, QGraphicsEllipseItem, QGraphicsItem, QGraphicsLineItem, QGraphicsScene, QGraphicsSceneMouseEvent, QGraphicsSimpleTextItem, QGraphicsView, QMainWindow
@dataclass
class Node:
    """Qt와 독립적인 노드 데이터."""
    node_id: int
    x: float
    y: float
    old_x: float = field(init=False)
    old_y: float = field(init=False)
    fixed: bool = False
    def __post_init__(self):
        self.x = float(self.x)
        self.y = float(self.y)
        self.old_x = self.x
        self.old_y = self.y
    def set_position(self, x: float, y: float, reset_velocity: bool = False):
        # 직접 이동하는 경우 이전 위치도 함께 변경하여 불필요한 관성을 제거한다.
        if reset_velocity:
            self.old_x = float(x)
            self.old_y = float(y)
        self.x = float(x)
        self.y = float(y)
    def reset_velocity(self):
        # 현재 위치와 이전 위치를 같게 만들어 Verlet 속도를 제거한다.
        self.old_x = self.x
        self.old_y = self.y
@dataclass
class Line:
    """Qt와 독립적인 라인 및 거리 제약조건 데이터."""
    line_id: int
    node1_id: int
    node2_id: int
    length: float
class Object:
    """Node와 Line 데이터를 보관하고 Verlet 및 Distance Constraint Solver를 실행한다."""
    EPSILON = 1.0e-8
    def __init__(self, solver_iterations: int = 60, damping: float = 0.98, tolerance: float = 0.0001):
        self.nodes: dict[int, Node] = {}
        self.lines: dict[int, Line] = {}
        self.solver_iterations = solver_iterations
        self.damping = damping
        self.tolerance = tolerance
        self.next_line_id = 1
    def add_node(self, node_id: int, x: float, y: float) -> Node:
        # 노드 번호는 중복될 수 없다.
        if node_id in self.nodes:
            raise ValueError(f"이미 존재하는 노드 번호입니다: {node_id}")
        node = Node(node_id, x, y)
        self.nodes[node_id] = node
        return node
    def add_line(self, node1_id: int, node2_id: int) -> Line:
        # 자기 자신과 연결하거나 존재하지 않는 노드를 연결할 수 없다.
        if node1_id == node2_id:
            raise ValueError("동일한 노드를 서로 연결할 수 없습니다.")
        if node1_id not in self.nodes or node2_id not in self.nodes:
            raise ValueError(f"존재하지 않는 노드입니다: {node1_id}, {node2_id}")
        if self.find_line(node1_id, node2_id) is not None:
            raise ValueError(f"이미 존재하는 라인입니다: {node1_id}-{node2_id}")
        node1 = self.nodes[node1_id]
        node2 = self.nodes[node2_id]
        length = math.hypot(node2.x - node1.x, node2.y - node1.y)
        if length <= self.EPSILON:
            raise ValueError("동일한 위치에 있는 두 노드는 연결할 수 없습니다.")
        line = Line(self.next_line_id, node1_id, node2_id, length)
        self.lines[line.line_id] = line
        self.next_line_id += 1
        return line
    def find_line(self, node1_id: int, node2_id: int) -> Line | None:
        # 노드 순서와 관계없이 동일한 연결선을 찾는다.
        for line in self.lines.values():
            if {line.node1_id, line.node2_id} == {node1_id, node2_id}:
                return line
        return None
    def set_fixed(self, node_id: int, fixed: bool):
        # 고정 상태를 변경하면서 남아 있는 속도를 제거한다.
        node = self.nodes[node_id]
        node.fixed = fixed
        node.reset_velocity()
    def toggle_fixed(self, node_id: int) -> bool:
        self.set_fixed(node_id, not self.nodes[node_id].fixed)
        return self.nodes[node_id].fixed
    def move_node(self, node_id: int, x: float, y: float, solve: bool = True):
        # 영구 고정된 노드는 사용자 입력으로도 이동할 수 없다.
        node = self.nodes[node_id]
        if node.fixed:
            return
        node.set_position(x, y, True)
        if solve:
            # 드래그 중인 노드는 Solver 관점에서 임시 고정점으로 처리한다.
            self.solve(dragged_node_id=node_id)
    def integrate(self, dragged_node_id: int | None = None):
        """현재 위치와 이전 위치의 차이를 이용해 Verlet 적분을 수행한다."""
        for node in self.nodes.values():
            if node.fixed or node.node_id == dragged_node_id:
                node.reset_velocity()
                continue
            velocity_x = (node.x - node.old_x) * self.damping
            velocity_y = (node.y - node.old_y) * self.damping
            node.old_x = node.x
            node.old_y = node.y
            node.x += velocity_x
            node.y += velocity_y
    def inverse_mass(self, node: Node, dragged_node_id: int | None) -> float:
        # 고정 노드와 드래그 중인 노드는 질량이 무한대인 고정점으로 취급한다.
        return 0.0 if node.fixed or node.node_id == dragged_node_id else 1.0
    def solve_line(self, line: Line, dragged_node_id: int | None) -> float:
        """한 개 라인의 길이 제약조건을 계산한다."""
        node1 = self.nodes[line.node1_id]
        node2 = self.nodes[line.node2_id]
        dx = node2.x - node1.x
        dy = node2.y - node1.y
        current_length = math.hypot(dx, dy)
        # 노드가 같은 위치에 겹쳤다면 안정적인 임시 방향을 사용한다.
        if current_length <= self.EPSILON:
            dx = line.length
            dy = 0.0
            current_length = line.length
        error = current_length - line.length
        weight1 = self.inverse_mass(node1, dragged_node_id)
        weight2 = self.inverse_mass(node2, dragged_node_id)
        total_weight = weight1 + weight2
        # 두 노드가 모두 고정이면 제약조건을 보정할 수 없다.
        if total_weight <= self.EPSILON:
            return abs(error)
        correction_x = dx * error / current_length
        correction_y = dy * error / current_length
        if weight1 > 0.0:
            node1.x += correction_x * weight1 / total_weight
            node1.y += correction_y * weight1 / total_weight
        if weight2 > 0.0:
            node2.x -= correction_x * weight2 / total_weight
            node2.y -= correction_y * weight2 / total_weight
        return abs(error)
    def solve_constraints(self, dragged_node_id: int | None = None) -> float:
        """모든 라인의 거리 제약조건을 반복 적용하여 루프 구조까지 수렴시킨다."""
        maximum_error = 0.0
        for _ in range(self.solver_iterations):
            maximum_error = 0.0
            for line in self.lines.values():
                maximum_error = max(maximum_error, self.solve_line(line, dragged_node_id))
            if maximum_error <= self.tolerance:
                break
        return maximum_error
    def solve(self, dragged_node_id: int | None = None, use_verlet: bool = False) -> float:
        """Verlet 적분과 거리 제약조건을 실행한다."""
        if use_verlet:
            self.integrate(dragged_node_id)
        error = self.solve_constraints(dragged_node_id)
        # 직접 조작할 때는 Solver 처리 후 관성을 제거한다.
        if not use_verlet:
            for node in self.nodes.values():
                node.reset_velocity()
        return error
    def maximum_constraint_error(self) -> float:
        """전체 라인 중 가장 큰 길이 오차를 반환한다."""
        maximum_error = 0.0
        for line in self.lines.values():
            node1 = self.nodes[line.node1_id]
            node2 = self.nodes[line.node2_id]
            current_length = math.hypot(node2.x - node1.x, node2.y - node1.y)
            maximum_error = max(maximum_error, abs(current_length - line.length))
        return maximum_error
class NodeItem(QGraphicsEllipseItem):
    """Node 모델을 표시하고 마우스 입력을 모델에 전달한다."""
    RADIUS = 5.0
    NORMAL_COLOR = QColor("#3399FF")
    FIXED_COLOR = QColor("#FF3030")
    OUTLINE_COLOR = QColor("#003F70")
    def __init__(self, node_id: int, graph_scene: "GraphScene"):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2.0, self.RADIUS * 2.0)
        self.node_id = node_id
        self.graph_scene = graph_scene
        self.dragging = False
        self.drag_offset = QPointF()
        self.setBrush(QBrush(self.NORMAL_COLOR))
        self.setPen(QPen(self.OUTLINE_COLOR, 1.0))
        self.setZValue(10.0)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setAcceptedMouseButtons(Qt.MouseButton.LeftButton)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        # 노드 번호는 노드 오른쪽에 표시한다.
        self.label = QGraphicsSimpleTextItem(str(node_id), self)
        self.label.setBrush(QBrush(QColor("#202020")))
        self.label.setPos(self.RADIUS + 3.0, -10.0)
    def sync_from_model(self):
        # 모델 좌표와 상태를 화면에 반영한다.
        node = self.graph_scene.graph_object.nodes[self.node_id]
        self.setPos(node.x, node.y)
        self.setBrush(QBrush(self.FIXED_COLOR if node.fixed else self.NORMAL_COLOR))
        self.setCursor(Qt.CursorShape.ForbiddenCursor if node.fixed else Qt.CursorShape.ClosedHandCursor if self.dragging else Qt.CursorShape.OpenHandCursor)
        self.setToolTip(f"Node {node.node_id} | X={node.x:.2f}, Y={node.y:.2f} | {'고정' if node.fixed else '이동 가능'}")
    def mouseDoubleClickEvent(self, event: QGraphicsSceneMouseEvent):
        # 노드를 더블클릭하면 고정 상태를 반전시킨다.
        self.dragging = False
        self.graph_scene.graph_object.toggle_fixed(self.node_id)
        self.graph_scene.sync_from_model()
        event.accept()
    def mousePressEvent(self, event: QGraphicsSceneMouseEvent):
        node = self.graph_scene.graph_object.nodes[self.node_id]
        # 고정 노드는 마우스로 이동할 수 없다.
        if event.button() != Qt.MouseButton.LeftButton or node.fixed:
            event.accept()
            return
        self.dragging = True
        self.drag_offset = event.scenePos() - QPointF(node.x, node.y)
        self.setCursor(Qt.CursorShape.ClosedHandCursor)
        event.accept()
    def mouseMoveEvent(self, event: QGraphicsSceneMouseEvent):
        if not self.dragging:
            event.ignore()
            return
        target = event.scenePos() - self.drag_offset
        self.graph_scene.graph_object.move_node(self.node_id, target.x(), target.y(), True)
        self.graph_scene.sync_from_model()
        event.accept()
    def mouseReleaseEvent(self, event: QGraphicsSceneMouseEvent):
        if not self.dragging:
            event.accept()
            return
        self.dragging = False
        # 마지막 마우스 위치를 기준으로 한 번 더 제약조건을 계산한다.
        self.graph_scene.graph_object.solve(dragged_node_id=self.node_id)
        self.graph_scene.graph_object.nodes[self.node_id].reset_velocity()
        self.graph_scene.sync_from_model()
        event.accept()
class LineItem(QGraphicsLineItem):
    """Line 모델을 화면에 표시한다."""
    def __init__(self, line_id: int, graph_scene: "GraphScene"):
        super().__init__()
        self.line_id = line_id
        self.graph_scene = graph_scene
        self.setPen(QPen(QColor("#404040"), 1.5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        self.setZValue(1.0)
        self.setAcceptedMouseButtons(Qt.MouseButton.NoButton)
    def sync_from_model(self):
        # 모델의 두 노드 좌표를 이용해 화면의 라인을 갱신한다.
        line = self.graph_scene.graph_object.lines[self.line_id]
        node1 = self.graph_scene.graph_object.nodes[line.node1_id]
        node2 = self.graph_scene.graph_object.nodes[line.node2_id]
        self.setLine(QLineF(node1.x, node1.y, node2.x, node2.y))
        current_length = math.hypot(node2.x - node1.x, node2.y - node1.y)
        self.setToolTip(f"Line {line.line_id}: {line.node1_id}-{line.node2_id} | 기준={line.length:.3f}, 현재={current_length:.3f}")
class GraphScene(QGraphicsScene):
    """Object 모델을 QGraphicsItem으로 표현하고 동기화한다."""
    def __init__(self, graph_object: Object):
        super().__init__()
        self.graph_object = graph_object
        self.node_items: dict[int, NodeItem] = {}
        self.line_items: dict[int, LineItem] = {}
        self.setSceneRect(QRectF(-100.0, -100.0, 600.0, 500.0))
        self.setBackgroundBrush(QBrush(QColor("#F7F7F7")))
        self.rebuild_view()
    def rebuild_view(self):
        # 모델 전체를 기준으로 View 객체를 다시 생성한다.
        self.clear()
        self.node_items.clear()
        self.line_items.clear()
        # 라인을 먼저 추가하여 노드보다 뒤에 표시한다.
        for line_id in self.graph_object.lines:
            item = LineItem(line_id, self)
            self.line_items[line_id] = item
            self.addItem(item)
        for node_id in self.graph_object.nodes:
            item = NodeItem(node_id, self)
            self.node_items[node_id] = item
            self.addItem(item)
        self.sync_from_model()
    def sync_from_model(self):
        # 라인을 먼저 갱신하고 노드를 그 위에 표시한다.
        for item in self.line_items.values():
            item.sync_from_model()
        for item in self.node_items.values():
            item.sync_from_model()
        self.update()
class GraphView(QGraphicsView):
    """화면 이동과 확대 및 축소를 담당한다."""
    def __init__(self, scene: GraphScene):
        super().__init__(scene)
        self.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        self.setViewportUpdateMode(QGraphicsView.ViewportUpdateMode.FullViewportUpdate)
        self.setDragMode(QGraphicsView.DragMode.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)
    def wheelEvent(self, event):
        # 마우스 휠로 화면을 확대하거나 축소한다.
        scale_factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.scale(scale_factor, scale_factor)
        event.accept()
class MainWindow(QMainWindow):
    def __init__(self, graph_object: Object):
        super().__init__()
        self.graph_object = graph_object
        self.graph_scene = GraphScene(graph_object)
        self.graph_view = GraphView(self.graph_scene)
        self.setCentralWidget(self.graph_view)
        self.setWindowTitle("Verlet + Distance Constraint Solver")
        self.resize(900, 700)
        self.statusBar().showMessage("노드 드래그: 이동 | 더블클릭: 고정/해제 | 빨간색: 고정 노드 | 휠: 확대/축소")
def create_test_object() -> Object:
    """테스트 데이터를 생성한다."""
    graph_object = Object(solver_iterations=60, damping=0.98, tolerance=0.0001)
    # 테스트 노드 데이터
    graph_object.add_node(1, 100, 100)
    graph_object.add_node(2, 200, 100)
    graph_object.add_node(3, 300, 100)
    graph_object.add_node(4, 200, 200)
    graph_object.add_node(5, 200, 300)
    # 기존 테스트 라인
    graph_object.add_line(1, 2)
    graph_object.add_line(2, 3)
    graph_object.add_line(2, 4)
    graph_object.add_line(4, 5)
    # 2-3-5-4-2 루프를 만들기 위한 추가 라인
    graph_object.add_line(3, 5)
    return graph_object
def main():
    app = QApplication(sys.argv)
    graph_object = create_test_object()
    window = MainWindow(graph_object)
    window.show()
    sys.exit(app.exec())
if __name__ == "__main__":




    main()