import sys
import math
from dataclasses import dataclass
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel

TOL = 0.02
PULL_ITER = 30
POLISH_ITER = 50
MAX_PULL = 300
MAX_POLISH = 1000
PULL_MIN = 10
CONV_PULL = 1e-4
EPS = 1e-9
SOFT = 0.1

def key(a, b):
    return (a, b) if a < b else (b, a)

def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])

@dataclass
class Rule:
    kind: str          # 'F', 'R', 'A'
    n1: int
    n2: int
    n3: int
    ratio: float = 1.0

class Skeleton:
    def __init__(self):
        self.nodes = []
        self.max_len = {}   
        self.rules = []

class Pose:
    def __init__(self):
        self.pos = {}       
        self.cur_len = {}   
        self.angles = {}    
        self.pinned = set() 

class GameObject:
    def __init__(self, obj_id: str):
        self.obj_id = obj_id
        self.skel = Skeleton()
        self.pose = Pose()

    def add_node(self, nid, x, y):
        self.skel.nodes.append(nid)
        self.pose.pos[nid] = [float(x), float(y)]

    def add_line(self, n1, n2):
        L = dist(self.pose.pos[n1], self.pose.pos[n2])
        k = key(n1, n2)
        self.skel.max_len[k] = L
        self.pose.cur_len[k] = L

    def add_rule(self, kind, n1, n2, n3):
        kind = kind.upper()
        r = Rule(kind, n1, n2, n3)
        if kind == "R":
            d1 = dist(self.pose.pos[n1], self.pose.pos[n2])
            d3 = dist(self.pose.pos[n3], self.pose.pos[n2])
            r.ratio = d3 / d1 if d1 > EPS else 1.0
        self.skel.rules.append(r)
        if kind == "F":
            self.pose.angles[len(self.skel.rules) - 1] = self._angle(self.pose.pos, r)

    @staticmethod
    def _angle(pos, r):
        p1, p2, p3 = pos[r.n1], pos[r.n2], pos[r.n3]
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v3 = (p3[0] - p2[0], p3[1] - p2[1])
        return math.atan2(v1[0] * v3[1] - v1[1] * v3[0], v1[0] * v3[0] + v1[1] * v3[1])


class Scene:
    def __init__(self):
        self.objects = {}
        self.welds = []     

    def add_object(self, obj: GameObject):
        self.objects[obj.obj_id] = obj

    def attach(self, p_obj, p_node, c_obj, c_node):
        w = ((p_obj, p_node), (c_obj, c_node))
        if w not in self.welds:
            self.welds.append(w)

    def detach(self, c_obj):
        self.welds = [w for w in self.welds if w[1][0] != c_obj]

    def update_angles(self):
        """이동이 끝났을 때 현재 포즈 기준으로 F 규칙의 각도를 새로 계산하여 갱신"""
        for obj_id, obj in self.objects.items():
            for i, r in enumerate(obj.skel.rules):
                if r.kind == "F":
                    obj.pose.angles[i] = GameObject._angle(obj.pose.pos, r)

    def solve_unified(self, kinpos=None):
        if kinpos is None:
            kinpos = {}

        pos = {}
        w = {}
        for obj_id, obj in self.objects.items():
            for nid, p in obj.pose.pos.items():
                gkey = (obj_id, nid)
                pos[gkey] = [p[0], p[1]]
                w[gkey] = 0.0 if nid in obj.pose.pinned else 1.0

        for gkey, p in kinpos.items():
            if gkey in pos:
                pos[gkey] = [p[0], p[1]]
                w[gkey] = 0.0

        n_nodes = len(pos)
        caps = (
            ("pull", min(MAX_PULL, max(PULL_ITER, 4 * n_nodes))),
            ("polish", min(MAX_POLISH, max(POLISH_ITER, 10 * n_nodes)))
        )

        for phase, iters in caps:
            if phase == "polish":
                for gkey in kinpos:
                    if gkey[1] not in self.objects[gkey[0]].pose.pinned:
                        w[gkey] = 1.0

            for it in range(iters):
                if phase == "pull":
                    prev = {k: (v[0], v[1]) for k, v in pos.items()}
                    for gkey, p in kinpos.items():
                        if gkey in pos:
                            pos[gkey][0], pos[gkey][1] = p[0], p[1]

                for obj_id, obj in self.objects.items():
                    for i, r in enumerate(obj.skel.rules):
                        if r.kind == "F" and i in obj.pose.angles:
                            self._proj_angle(pos, w, obj_id, r, obj.pose.angles[i])
                    for (a, b), L in obj.pose.cur_len.items():
                        self._proj_eq(pos, w, (obj_id, a), (obj_id, b), L)
                    for (a, b), M in obj.skel.max_len.items():
                        self._proj_max(pos, w, (obj_id, a), (obj_id, b), M)

                for parent_key, child_key in self.welds:
                    self._proj_weld(pos, w, parent_key, child_key)

                if phase == "pull":
                    if it >= PULL_MIN and max(abs(pos[k][0] - prev[k][0]) + abs(pos[k][1] - prev[k][1]) for k in pos) < CONV_PULL:
                        break

        for gkey, p in pos.items():
            obj_id, nid = gkey
            if obj_id in self.objects and nid in self.objects[obj_id].pose.pos:
                self.objects[obj_id].pose.pos[nid] = [p[0], p[1]]

    @staticmethod
    def _proj_eq(pos, w, a_key, b_key, L):
        if a_key not in pos or b_key not in pos:
            return
        pa, pb = pos[a_key], pos[b_key]
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        d = math.hypot(dx, dy)
        sw = w[a_key] + w[b_key]
        if sw == 0 or d < EPS:
            return
        c = (d - L) / d
        pa[0] += w[a_key] / sw * c * dx
        pa[1] += w[a_key] / sw * c * dy
        pb[0] -= w[b_key] / sw * c * dx
        pb[1] -= w[b_key] / sw * c * dy

    @staticmethod
    def _proj_max(pos, w, a_key, b_key, M):
        if a_key not in pos or b_key not in pos:
            return
        if dist(pos[a_key], pos[b_key]) > M:
            Scene._proj_eq(pos, w, a_key, b_key, M)

    @staticmethod
    def _proj_weld(pos, w, a_key, b_key):
        if a_key not in pos or b_key not in pos:
            return
        pa, pb = pos[a_key], pos[b_key]
        dx, dy = pb[0] - pa[0], pb[1] - pa[1]
        d = math.hypot(dx, dy)
        sw = w[a_key] + w[b_key]
        if sw == 0 or d < EPS:
            return
        pa[0] += (w[a_key] / sw) * dx
        pa[1] += (w[a_key] / sw) * dy
        pb[0] -= (w[b_key] / sw) * dx
        pb[1] -= (w[b_key] / sw) * dy

    @staticmethod
    def _proj_angle(pos, w, obj_id, r, th):
        n1, n2, n3 = (obj_id, r.n1), (obj_id, r.n2), (obj_id, r.n3)
        if n1 not in pos or n2 not in pos or n3 not in pos:
            return
        p1, p2, p3 = pos[n1], pos[n2], pos[n3]
        v1 = (p1[0] - p2[0], p1[1] - p2[1])
        v3 = (p3[0] - p2[0], p3[1] - p2[1])
        cur = math.atan2(v1[0] * v3[1] - v1[1] * v3[0], v1[0] * v3[0] + v1[1] * v3[1])
        diff = wrap(th - cur)
        tw = w[n1] + w[n3]
        if tw == 0 or abs(diff) < 1e-9:
            return
        def rot(v, a):
            c, s_ = math.cos(a), math.sin(a)
            return (v[0] * c - v[1] * s_, v[0] * s_ + v[1] * c)
        r3 = rot(v3, diff * w[n3] / tw)
        r1 = rot(v1, -diff * w[n1] / tw)
        p3[0], p3[1] = p2[0] + r3[0], p2[1] + r3[1]
        p1[0], p1[1] = p2[0] + r1[0], p2[1] + r1[1]


class SceneCanvas(QWidget):
    def __init__(self, scene: Scene):
        super().__init__()
        self.scene = scene
        self.drag_gkey = None
        self.setMinimumSize(900, 700)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        pen_line = QPen(QColor(70, 130, 180), 4)
        painter.setPen(pen_line)
        for obj_id, obj in self.scene.objects.items():
            for (a, b) in obj.pose.cur_len:
                p1 = QPointF(*obj.pose.pos[a])
                p2 = QPointF(*obj.pose.pos[b])
                painter.drawLine(p1, p2)

        pen_weld = QPen(QColor(255, 69, 0), 2, Qt.PenStyle.DashLine)
        painter.setPen(pen_weld)
        for (p_key, c_key) in self.scene.welds:
            p_obj, p_nid = p_key
            c_obj, c_nid = c_key
            if p_obj in self.scene.objects and c_obj in self.scene.objects:
                pt1 = QPointF(*self.scene.objects[p_obj].pose.pos[p_nid])
                pt2 = QPointF(*self.scene.objects[c_obj].pose.pos[c_nid])
                painter.drawLine(pt1, pt2)

        for obj_id, obj in self.scene.objects.items():
            for nid, (x, y) in obj.pose.pos.items():
                if nid in obj.pose.pinned:
                    painter.setBrush(QBrush(QColor(220, 20, 60))) 
                else:
                    painter.setBrush(QBrush(QColor(255, 255, 255)))
                
                painter.setPen(QPen(QColor(0, 0, 0), 2))
                painter.drawEllipse(QPointF(x, y), 8, 8)

    def mousePressEvent(self, event):
        mx, my = event.position().x(), event.position().y()
        
        clicked = None
        min_dist = 18.0
        for obj_id, obj in self.scene.objects.items():
            for nid, (x, y) in obj.pose.pos.items():
                d = math.hypot(x - mx, y - my)
                if d < min_dist:
                    min_dist = d
                    clicked = (obj_id, nid)

        if clicked:
            if event.button() == Qt.MouseButton.LeftButton:
                self.drag_gkey = clicked
            elif event.button() == Qt.MouseButton.RightButton:
                obj_id, nid = clicked
                pinned = self.scene.objects[obj_id].pose.pinned
                if nid in pinned:
                    pinned.remove(nid)
                else:
                    pinned.add(nid)
                self.update()

    def mouseMoveEvent(self, event):
        if self.drag_gkey:
            mx, my = event.position().x(), event.position().y()
            kinpos = {self.drag_gkey: [mx, my]}
            self.scene.solve_unified(kinpos)
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_gkey = None
            # 이동(드래그)이 끝나는 시점에 F 규칙의 각도를 현재 포즈 기준으로 재계산하여 갱신
            self.scene.update_angles()
            self.update()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Multi-Object Pose Editor (Dynamic F-Angle Update)")
        
        self.scene = Scene()
        
        char = GameObject("character")
        char.add_node(1, 350, 200) # 어깨 (우클릭으로 고정/해제 토글 가능)
        char.add_node(2, 350, 320) # 팔꿈치
        char.add_node(3, 350, 440) # 손
        char.add_line(1, 2)
        char.add_line(2, 3)
        char.add_rule("F", 1, 2, 3)
        # char.pose.pinned.add(1) # 필요시 1번 노드 고정 해제하여 자유롭게 이동 가능
        self.scene.add_object(char)

        tool = GameObject("tool")
        tool.add_node(1, 500, 440) 
        tool.add_node(2, 620, 440) 
        tool.add_line(1, 2)
        self.scene.add_object(tool)

        self.scene.attach("character", 3, "tool", 1)

        self.canvas = SceneCanvas(self.scene)
        
        layout = QVBoxLayout()
        layout.addWidget(QLabel("좌클릭 드래그: 이동 후 마우스를 떼면 현재 각도가 F 규칙에 새롭게 저장됨 / 우클릭: 노드 고정(Pin) 토글"))
        layout.addWidget(self.canvas)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
