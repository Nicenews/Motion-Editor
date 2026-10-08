import sys, math
from PyQt6.QtCore import Qt, QPointF, QLineF
from PyQt6.QtGui import QBrush, QPen, QColor
from PyQt6.QtWidgets import (QApplication, QGraphicsEllipseItem, 
                             QGraphicsLineItem, QGraphicsScene, QGraphicsView)

# ============================================================
# Graphics Items
# ============================================================
class NodeItem(QGraphicsEllipseItem):
    RADIUS = 10

    def __init__(self, node_id, x, y):
        super().__init__(-self.RADIUS, -self.RADIUS, self.RADIUS * 2, self.RADIUS * 2)
        self.node_id = node_id
        self.connected_lines = []
        
        self.setBrush(QBrush(QColor("#4A90E2")))
        self.setPen(QPen(QColor("#1C3D5A"), 2))
        self.setPos(x, y)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsScenePositionChanges, True)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemScenePositionHasChanged:
            scene = self.scene()
            if scene and getattr(scene, 'is_dragging', False):
                scene.on_node_moved(self)
        return super().itemChange(change, value)


class LineItem(QGraphicsLineItem):
    def __init__(self, node_a, node_b):
        super().__init__()
        self.node_a = node_a
        self.node_b = node_b
        
        # 초기 고유 길이 저장
        pos_a, pos_b = node_a.scenePos(), node_b.scenePos()
        self.rest_length = math.hypot(pos_b.x() - pos_a.x(), pos_b.y() - pos_a.y())

        node_a.connected_lines.append(self)
        node_b.connected_lines.append(self)

        self.setPen(QPen(QColor("#333333"), 2))
        self.setZValue(-1)
        self.update_position()

    def update_position(self):
        self.setLine(QLineF(self.node_a.scenePos(), self.node_b.scenePos()))


# ============================================================
# Scene & View
# ============================================================
class NodeScene(QGraphicsScene):
    def __init__(self):
        super().__init__()
        self.setSceneRect(0, 0, 800, 600)
        self.is_dragging = False

    def mousePressEvent(self, event):
        self.is_dragging = True
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        self.is_dragging = False
        super().mouseReleaseEvent(event)

    def on_node_moved(self, moved_node):
        """PBD (Position Based Dynamics) 방식을 활용한 다중 제약조건 수렴 연산"""
        all_lines = [item for item in self.items() if isinstance(item, LineItem)]

        # iterations 횟수가 높을수록 단단한(Rigid) 변형체가 됩니다. (10~20회 권장)
        iterations = 15
        
        for _ in range(iterations):
            for line in all_lines:
                node_a, node_b = line.node_a, line.node_b
                
                pos_a = node_a.scenePos()
                pos_b = node_b.scenePos()

                dx = pos_b.x() - pos_a.x()
                dy = pos_b.y() - pos_a.y()
                dist = math.hypot(dx, dy)

                if dist < 1e-5:
                    continue

                # 목표 길이 대비 길이 오차 계산
                diff = (dist - line.rest_length) / dist
                offset_x = dx * 0.5 * diff
                offset_y = dy * 0.5 * diff

                # 이동중인 노드는 위치를 고정하고, 나머지 노드들의 위치를 분배 조정
                if node_a == moved_node:
                    node_b.setPos(pos_b.x() - offset_x * 2, pos_b.y() - offset_y * 2)
                elif node_b == moved_node:
                    node_a.setPos(pos_a.x() + offset_x * 2, pos_a.y() + offset_y * 2)
                else:
                    node_a.setPos(pos_a.x() + offset_x, pos_a.y() + offset_y)
                    node_b.setPos(pos_b.x() - offset_x, pos_b.y() - offset_y)

        # 라인 그래픽 업데이트
        for line in all_lines:
            line.update_position()


# ============================================================
# Main Execution
# ============================================================
def main():
    app = QApplication(sys.argv)
    scene = NodeScene()

    # 사각형 형태의 순환 구조 예시 노드 배치 (1-2-3-4-1)
    n1 = NodeItem(1, 200, 200)
    n2 = NodeItem(2, 400, 200)
    n3 = NodeItem(3, 400, 400)
    n4 = NodeItem(4, 200, 400)

    for n in (n1, n2, n3, n4):
        scene.addItem(n)

    # 4개 노드를 연결하여 닫힌 사각형 구조(순환 루프) 생성
    scene.addItem(LineItem(n1, n2))
    scene.addItem(LineItem(n2, n3))
    scene.addItem(LineItem(n3, n4))
    scene.addItem(LineItem(n4, n1))

    # 대각선 서포트 라인을 추가하면 사각형의 형태가 찌그러지지 않고 강체처럼 유지됩니다.
    # scene.addItem(LineItem(n1, n3)) 

    view = QGraphicsView(scene)
    view.setWindowTitle("Cyclic Structure Node Constraint")
    view.resize(800, 600)
    view.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()