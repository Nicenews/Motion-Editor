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
        # 노드의 위치가 변경될 때마다 이벤트 수신
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
        
        # 생성 시점의 라인 고유 길이 저장
        pos_a, pos_b = node_a.scenePos(), node_b.scenePos()
        self.rest_length = math.hypot(pos_b.x() - pos_a.x(), pos_b.y() - pos_a.y())

        node_a.connected_lines.append(self)
        node_b.connected_lines.append(self)

        self.setPen(QPen(QColor("#333333"), 2))
        self.setZValue(-1)
        self.update_position()

    def update_position(self):
        """두 노드의 위치에 맞게 라인 시각화 업데이트"""
        self.setLine(QLineF(self.node_a.scenePos(), self.node_b.scenePos()))

    def get_other_node(self, node):
        return self.node_b if node == self.node_a else self.node_a


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
        """드래그된 노드를 시작점으로 연쇄적인 거리 제약조건(Distance Constraint) 적용"""
        visited = {moved_node}
        queue = [moved_node]

        # BFS(너비 우선 탐색) 형태로 연결된 모든 노드를 순회하며 길이 유지를 적용
        while queue:
            curr_node = queue.pop(0)

            for line in curr_node.connected_lines:
                next_node = line.get_other_node(curr_node)
                
                # 이미 이번 위치 변경 연산에서 처리된 노드는 건너뜀
                if next_node in visited:
                    continue

                # curr_node 위치 기준으로 next_node의 위치를 rest_length 방향으로 재조정
                p_curr = curr_node.scenePos()
                p_next = next_node.scenePos()

                dx = p_next.x() - p_curr.x()
                dy = p_next.y() - p_curr.y()
                current_dist = math.hypot(dx, dy)

                if current_dist > 1e-5:
                    # 방향 단위 벡터 * 유지할 길이(rest_length)
                    target_x = p_curr.x() + (dx / current_dist) * line.rest_length
                    target_y = p_curr.y() + (dy / current_dist) * line.rest_length
                    next_node.setPos(target_x, target_y)

                visited.add(next_node)
                queue.append(next_node)

        # 모든 라인의 화면 위치 갱신
        for item in self.items():
            if isinstance(item, LineItem):
                item.update_position()


# ============================================================
# Main Execution
# ============================================================
def main():
    app = QApplication(sys.argv)
    scene = NodeScene()

    # 샘플 노드 생성
    n1 = NodeItem(1, 100, 200)
    n2 = NodeItem(2, 250, 200)
    n3 = NodeItem(3, 400, 200)
    n4 = NodeItem(4, 250, 350)

    for n in (n1, n2, n3, n4):
        scene.addItem(n)

    # 샘플 라인 연결 (1-2, 2-3, 2-4 연결)
    scene.addItem(LineItem(n1, n2))
    scene.addItem(LineItem(n2, n3))
    scene.addItem(LineItem(n2, n4))

    view = QGraphicsView(scene)
    view.setWindowTitle("Node & Line Distance Constraint")
    view.resize(800, 600)
    view.show()

    sys.exit(app.exec())

if __name__ == "__main__":
    main()