import sys, math
from PyQt6.QtCore import Qt, QPointF, QLineF, QTimer
from PyQt6.QtGui import QBrush, QPen, QColor
from PyQt6.QtWidgets import (QApplication, QGraphicsEllipseItem, 
                             QGraphicsLineItem, QGraphicsScene, QGraphicsView)


class NodeItem(QGraphicsEllipseItem):
    RADIUS = 10

    def __init__(self, node_id, x, y):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.node_id = node_id
        self.pos = QPointF(x, y)
        self.old_pos = QPointF(x, y)
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
            new_pos = event.scenePos()
            self.pos = new_pos
            self.old_pos = new_pos
            self.setPos(new_pos)
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

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_physics)
        self.timer.start(16)

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

    def update_physics(self):
        friction = 0.88
        for node in self.nodes:
            if not node.is_pinned:
                vx = (node.pos.x() - node.old_pos.x()) * friction
                vy = (node.pos.y() - node.old_pos.y()) * friction
                node.old_pos = QPointF(node.pos)
                node.pos = QPointF(node.pos.x() + vx, node.pos.y() + vy)

        # ----------------------------------------------------
        # 핵심: 정방향/역방향 대칭 연산 (Symmetric Relaxation)
        # ----------------------------------------------------
        iterations = 16
        for it in range(iterations):
            # 짝수번째 반복은 정방향, 홀수번째 반복은 역방향 순회
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

                if nA.is_pinned and not nB.is_pinned:
                    nB.pos = QPointF(nB.pos.x() - ox * 2, nB.pos.y() - oy * 2)
                elif nB.is_pinned and not nA.is_pinned:
                    nA.pos = QPointF(nA.pos.x() + ox * 2, nA.pos.y() + oy * 2)
                elif not nA.is_pinned and not nB.is_pinned:
                    nA.pos = QPointF(nA.pos.x() + ox, nA.pos.y() + oy)
                    nB.pos = QPointF(nB.pos.x() - ox, nB.pos.y() - oy)

        for node in self.nodes:
            node.setPos(node.pos)
        for line in self.lines:
            line.update_position()


def main():
    app = QApplication(sys.argv)
    scene = NodeScene()

    # 사각형 대칭 구조 테스트
    n1 = scene.add_node(1, 250, 200)
    n2 = scene.add_node(2, 450, 200)
    n3 = scene.add_node(3, 450, 400)
    n4 = scene.add_node(4, 250, 400)

    scene.add_line(n1, n2)
    scene.add_line(n2, n3)
    scene.add_line(n3, n4)
    scene.add_line(n4, n1)

    view = QGraphicsView(scene)
    view.setWindowTitle("Symmetric Constraint Physics")
    view.resize(800, 600)
    view.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()