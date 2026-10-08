import sys, math
from PyQt6.QtCore import Qt, QPointF, QLineF
from PyQt6.QtGui import QBrush, QPen, QColor
from PyQt6.QtWidgets import (QApplication, QGraphicsEllipseItem, 
                             QGraphicsLineItem, QGraphicsScene, QGraphicsView)


# ==============================================================================
# 1. DATA MODEL LAYER
# ==============================================================================

class Node:
    """순수 노드 데이터 클래스"""
    def __init__(self, node_id: int, x: float, y: float):
        self.node_id = node_id
        self.x = x
        self.y = y
        self.is_pinned = False  # 더블클릭으로 고정/해제 상태 관리

    def set_position(self, x: float, y: float):
        self.x = x
        self.y = y


class Line:
    """순수 라인 제약조건 데이터 클래스"""
    def __init__(self, node_a: Node, node_b: Node):
        self.node_a = node_a
        self.node_b = node_b
        
        dx = node_b.x - node_a.x
        dy = node_b.y - node_a.y
        self.rest_length = math.hypot(dx, dy)


class Object:
    """Node와 Line을 소유하고 물리 제약조건 연산을 총괄하는 컨테이너 클래스"""
    def __init__(self, obj_id: int):
        self.obj_id = obj_id
        self.nodes = []
        self.lines = []

    def add_node(self, node_id: int, x: float, y: float) -> Node:
        node = Node(node_id, x, y)
        self.nodes.append(node)
        return node

    def add_line(self, node_a: Node, node_b: Node) -> Line:
        line = Line(node_a, node_b)
        self.lines.append(line)
        return line

    def solve_constraints(self, moved_node: Node, iterations: int = 20):
        """고정(is_pinned) 노드를 최우선으로 보호하며 제약조건 수렴"""
        for it in range(iterations):
            line_iterable = self.lines if (it % 2 == 0) else reversed(self.lines)

            for line in line_iterable:
                nA, nB = line.node_a, line.node_b

                # 규칙 1: 두 노드가 모두 고정되어 있으면 연산 건너뜀
                if nA.is_pinned and nB.is_pinned:
                    continue

                dx = nB.x - nA.x
                dy = nB.y - nA.y
                dist = math.hypot(dx, dy)

                if dist < 1e-5:
                    continue

                delta = (dist - line.rest_length) / dist
                ox = dx * 0.5 * delta
                oy = dy * 0.5 * delta

                # 규칙 2: 어느 한 노드만 고정(pinned)되어 있는 경우
                if nA.is_pinned and not nB.is_pinned:
                    nB.x -= ox * 2
                    nB.y -= oy * 2
                elif nB.is_pinned and not nA.is_pinned:
                    nA.x += ox * 2
                    nA.y += oy * 2

                # 규칙 3: 두 노드 모두 고정되어 있지 않은 경우
                elif not nA.is_pinned and not nB.is_pinned:
                    # 마우스로 잡고 이동 중인 노드는 고정 노드처럼 다룸
                    if nA == moved_node and nB != moved_node:
                        nB.x -= ox * 2
                        nB.y -= oy * 2
                    elif nB == moved_node and nA != moved_node:
                        nA.x += ox * 2
                        nA.y += oy * 2
                    else:
                        nA.x += ox
                        nA.y += oy
                        nB.x -= ox
                        nB.y -= oy


# ==============================================================================
# 2. GRAPHICS / VIEW LAYER
# ==============================================================================

class NodeItem(QGraphicsEllipseItem):
    """Data Model(Node)을 시각화하는 그래픽 아이템"""
    RADIUS = 10

    # 색상 정의
    COLOR_NORMAL = QColor("#4A90E2")  # 기본 파란색
    COLOR_PINNED = QColor("#E74C3C")  # 고정 상태 빨간색
    BORDER_COLOR = QColor("#1C3D5A")

    def __init__(self, node_data: Node, on_drag_callback):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.node_data = node_data
        self.on_drag_callback = on_drag_callback
        self.is_dragging = False

        self.setPen(QPen(self.BORDER_COLOR, 2))
        self.setZValue(1)
        self.sync_from_model()

    def sync_from_model(self):
        """데이터 모델 상태(위치, 고정 여부) -> 그래픽 반영"""
        self.setPos(self.node_data.x, self.node_data.y)
        
        # 고정 상태에 따라 브러시 색상 전환
        if self.node_data.is_pinned:
            self.setBrush(QBrush(self.COLOR_PINNED))
        else:
            self.setBrush(QBrush(self.COLOR_NORMAL))

    def mouseDoubleClickEvent(self, event):
        """더블클릭 시 고정 상태 토글 (Red/Blue 및 is_pinned 전환)"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.node_data.is_pinned = not self.node_data.is_pinned
            self.sync_from_model()
            event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            # 고정 노드는 드래그 불가능
            if not self.node_data.is_pinned:
                self.is_dragging = True
            event.accept()

    def mouseMoveEvent(self, event):
        # 고정되어 있지 않고, 드래그 상태일 때만 이동
        if self.is_dragging and not self.node_data.is_pinned:
            pos = event.scenePos()
            self.node_data.set_position(pos.x(), pos.y())
            if self.on_drag_callback:
                self.on_drag_callback(self.node_data)
            event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_dragging = False
            event.accept()


class LineItem(QGraphicsLineItem):
    """Data Model(Line)을 시각화하는 그래픽 아이템"""
    def __init__(self, line_data: Line):
        super().__init__()
        self.line_data = line_data

        self.setPen(QPen(QColor("#333333"), 2))
        self.setZValue(-1)
        self.sync_from_model()

    def sync_from_model(self):
        nA = self.line_data.node_a
        nB = self.line_data.node_b
        self.setLine(QLineF(nA.x, nA.y, nB.x, nB.y))


class NodeScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setSceneRect(0, 0, 800, 600)
        self.objects = []
        self.node_item_map = {}
        self.line_item_map = {}

    def add_object(self, obj: Object):
        self.objects.append(obj)
        
        for node in obj.nodes:
            item = NodeItem(node, on_drag_callback=self.on_node_dragged)
            self.node_item_map[node] = item
            self.addItem(item)

        for line in obj.lines:
            item = LineItem(line)
            self.line_item_map[line] = item
            self.addItem(item)

    def on_node_dragged(self, moved_node: Node):
        for obj in self.objects:
            if moved_node in obj.nodes:
                obj.solve_constraints(moved_node)
                break

        for node_item in self.node_item_map.values():
            node_item.sync_from_model()
        for line_item in self.line_item_map.values():
            line_item.sync_from_model()


def main():
    app = QApplication(sys.argv)
    scene = NodeScene()

    box_object = Object(obj_id=1)
    
    n1 = box_object.add_node(1, 250, 200)
    n2 = box_object.add_node(2, 450, 200)
    n3 = box_object.add_node(3, 450, 400)
    n4 = box_object.add_node(4, 250, 400)

    box_object.add_line(n1, n2)
    box_object.add_line(n2, n3)
    box_object.add_line(n3, n4)
    box_object.add_line(n4, n1)

    scene.add_object(box_object)

    view = QGraphicsView(scene)
    view.setWindowTitle("Pinned Node Priority Physics")
    view.resize(800, 600)
    view.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()