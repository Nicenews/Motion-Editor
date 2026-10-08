import sys, math
from PyQt6.QtCore import Qt, QPointF, QLineF
from PyQt6.QtGui import QBrush, QPen, QColor
from PyQt6.QtWidgets import (QApplication, QGraphicsEllipseItem, 
                             QGraphicsLineItem, QGraphicsScene, QGraphicsView)


class NodeItem(QGraphicsEllipseItem):
    RADIUS = 10

    def __init__(self, node_id, x, y):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.node_id = node_id
        self.pos = QPointF(x, y)
        self.is_pinned = False

        self.setBrush(QBrush(QColor("#4A90E2")))
        self.setPen(QPen(QColor("#1C3D5A"), 2))
        self.setPos(x, y)
        self.setZValue(1)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_pinned = True
            event.accept()

    def mouseMoveEvent(self, event):
        if self.is_pinned:
            self.pos = event.scenePos()
            self.setPos(self.pos)
            scene = self.scene()
            if scene:
                scene.solve_constraints(moved_node=self)
            event.accept()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.is_pinned = False
            event.accept()


class LineItem(QGraphicsLineItem):
    def __init__(self, node_a, node_b):
        super().__init__()
        self.node_a = node_a
        self.node_b = node_b
        
        dx = node_b.pos.x() - node_a.pos.x()
        dy = node_b.pos.y() - node_a.pos.y()
        self.rest_length = math.hypot(dx, dy)

        self.setPen(QPen(QColor("#333333"), 2))
        self.setZValue(-1)
        self.update_position()

    def update_position(self):
        self.setLine(QLineF(self.node_a.pos, self.node_b.pos))


class NodeScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setSceneRect(0, 0, 800, 600)
        self.nodes = []
        self.lines = []

    def add_node(self, node_id, x, y):
        node = NodeItem(node_id, x, y)
        self.nodes.append(node)
        self.addItem(node)
        return node

    def add_line(self, node_a, node_b):
        line = LineItem(node_a, node_b)
        self.lines.append(line)
        self.addItem(line)
        return line

    def solve_constraints(self, moved_node):
        """한쪽으로 힘이 쏠리지 않도록 대칭(Forward-Backward)으로 제약조건 수렴"""
        iterations = 20  # 반복 횟수가 높을수록 연결부가 단단해집니다.

        for it in range(iterations):
            # 짝수 프레임: 정방향(0->N), 홀수 프레임: 역방향(N->0) 순회
            line_iterable = self.lines if (it % 2 == 0) else reversed(self.lines)

            for line in line_iterable:
                nA, nB = line.node_a, line.node_b

                dx = nB.pos.x() - nA.pos.x()
                dy = nB.pos.y() - nA.pos.y()
                dist = math.hypot(dx, dy)

                if dist < 1e-5:
                    continue

                delta = (dist - line.rest_length) / dist
                ox = dx * 0.5 * delta
                oy = dy * 0.5 * delta

                # 선택되어 이동 중인 노드는 고정하고 나머지 노드들의 위치 조정
                if nA == moved_node and nB != moved_node:
                    nB.pos = QPointF(nB.pos.x() - ox * 2, nB.pos.y() - oy * 2)
                elif nB == moved_node and nA != moved_node:
                    nA.pos = QPointF(nA.pos.x() + ox * 2, nA.pos.y() + oy * 2)
                else:
                    nA.pos = QPointF(nA.pos.x() + ox, nA.pos.y() + oy)
                    nB.pos = QPointF(nB.pos.x() - ox, nB.pos.y() - oy)

        # 연산 결과 그래픽 렌더링 반영
        for node in self.nodes:
            node.setPos(node.pos)
        for line in self.lines:
            line.update_position()


def main():
    app = QApplication(sys.argv)
    scene = NodeScene()

    # 사각형 순환 구조 (1-2-3-4-1)
    n1 = scene.add_node(1, 250, 200)
    n2 = scene.add_node(2, 450, 200)
    n3 = scene.add_node(3, 450, 400)
    n4 = scene.add_node(4, 250, 400)

    scene.add_line(n1, n2)
    scene.add_line(n2, n3)
    scene.add_line(n3, n4)
    scene.add_line(n4, n1)

    view = QGraphicsView(scene)
    view.setWindowTitle("Pure Position-Based Cyclic Structure")
    view.resize(800, 600)
    view.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()