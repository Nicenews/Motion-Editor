import sys
import math
from PyQt6.QtCore import Qt, QPointF
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel

# ==========================================
# 1. 기하학 및 FABRIK 코어 엔진
# ==========================================
EPS = 1e-9

def dist(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def fabrik_solve_chain(joint_positions, target_length, target_pos, iterations=10):
    """
    단일 체인(예: [어깨, 팔꿈치, 손])에 대해 FABRIK IK 알고리즘을 수행.
    joint_positions[0]은 앵커(고정점), joint_positions[-1]은 말단(드래그 대상)
    """
    pos = [list(p) for p in joint_positions]
    origin = list(pos[0]) # 고정된 앵커 위치
    target = list(target_pos) # 마우스 목표 위치
    
    total_len = sum(target_length)
    
    # 1. 대상이 도달 불가능할 때 (직선으로 펴짐)
    if dist(origin, target) > total_len:
        direction = [(target[0]-origin[0])/dist(origin,target), (target[1]-origin[1])/dist(origin,target)]
        for i in range(1, len(pos)):
            pos[i][0] = pos[i-1][0] + direction[0] * target_length[i-1]
            pos[i][1] = pos[i-1][1] + direction[1] * target_length[i-1]
        return pos
        
    # 2. 대상이 도달 가능할 때 (반복 수행)
    for _ in range(iterations):
        # Step 1: Backward Reach (말단 -> 앵커 방향으로 당기기)
        pos[-1] = target # 말단을 target에 고정
        
        for i in range(len(pos) - 2, -1, -1):
            d = dist(pos[i], pos[i+1])
            if d < EPS: d = EPS
            ratio = target_length[i] / d
            # pos[i]를 pos[i+1] 방향으로 원래 길이만큼 이동
            pos[i][0] = pos[i+1][0] + (pos[i][0] - pos[i+1][0]) * ratio
            pos[i][1] = pos[i+1][1] + (pos[i][1] - pos[i+1][1]) * ratio
            
        # Step 2: Forward Reach (앵커 -> 말단 방향으로 밀기)
        pos[0] = origin # 앵커를 원위치에 고정
        
        for i in range(len(pos) - 1):
            d = dist(pos[i], pos[i+1])
            if d < EPS: d = EPS
            ratio = target_length[i] / d
            # pos[i+1]을 pos[i] 방향으로 원래 길이만큼 이동
            pos[i+1][0] = pos[i][0] + (pos[i+1][0] - pos[i][0]) * ratio
            pos[i+1][1] = pos[i][1] + (pos[i+1][1] - pos[i][1]) * ratio
            
    return pos

# ==========================================
# 2. 스틱맨 데이터 모델
# ==========================================
class StickmanModel:
    def __init__(self):
        # 1. 관절(Node) 위치 데이터 (Dictionary for 네임스페이스)
        self.joints = {
            "pelvis": [450, 350], # Root (Pinned)
            "belly": [450, 300],
            "chest": [450, 250],
            "neck": [450, 200],
            "head": [450, 150],
            
            "r_shoulder": [480, 200], # Neck anchored Hub
            "r_elbow": [550, 200],
            "r_hand": [620, 200],
            
            "l_shoulder": [420, 200], # Neck anchored Hub
            "l_elbow": [350, 200],
            "l_hand": [280, 200],
            
            "r_hip": [470, 350],      # Pelvis anchored Hub
            "r_knee": [470, 450],
            "r_foot": [470, 550],
            
            "l_hip": [430, 350],      # Pelvis anchored Hub
            "l_knee": [430, 450],
            "l_foot": [430, 550]
        }
        
        # 2. 뼈(Link) 체인 정의 (Order: 앵커 -> 말단) 및 목표 길이 계산
        self.spine_chain_keys = ["pelvis", "belly", "chest", "neck", "head"]
        self.spine_lens = self._calc_lens(self.spine_chain_keys)
        
        self.r_arm_chain_keys = ["r_shoulder", "r_elbow", "r_hand"]
        self.r_arm_lens = self._calc_lens(self.r_arm_chain_keys)
        
        self.l_arm_chain_keys = ["l_shoulder", "l_elbow", "l_hand"]
        self.l_arm_lens = self._calc_lens(self.l_arm_chain_keys)
        
        self.r_leg_chain_keys = ["r_hip", "r_knee", "r_foot"]
        self.r_leg_lens = self._calc_lens(self.r_leg_chain_keys)
        
        self.l_leg_chain_keys = ["l_hip", "l_knee", "l_foot"]
        self.l_leg_lens = self._calc_lens(self.l_leg_chain_keys)
        
        # 고정된 허브 오프셋 데이터 (PBD soup Popping의 주범인 허브 제약을 제거하기 위해 강제 오프셋 사용)
        self._shoulder_offset = dist(self.joints["r_shoulder"], self.joints["neck"])
        self._hip_offset = dist(self.joints["r_hip"], self.joints["pelvis"])
        
        # 3. 렌더링용 라인 연결 정의 (key pairs)
        self.lines = [
            ("head", "neck"), ("neck", "chest"), ("chest", "belly"), ("belly", "pelvis"),
            ("neck", "r_shoulder"), ("r_shoulder", "r_elbow"), ("r_elbow", "r_hand"),
            ("neck", "l_shoulder"), ("l_shoulder", "l_elbow"), ("l_elbow", "l_hand"),
            ("pelvis", "r_hip"), ("r_hip", "r_knee"), ("r_knee", "r_foot"),
            ("pelvis", "l_hip"), ("l_hip", "l_knee"), ("l_knee", "l_foot")
        ]

    def _calc_lens(self, chain_keys):
        """체인 내 관절들 간의 초기 길이를 계산하여 리스트로 반환"""
        lens = []
        for i in range(len(chain_keys) - 1):
            lens.append(dist(self.joints[chain_keys[i]], self.joints[chain_keys[i+1]]))
        return lens

    def _update_hubs(self):
        """spine이 변경되었을 때, neck을 기준으로 shoulders를, pelvis를 기준으로 hips를 강제 업데이트 (Rigid FK)"""
        p = self.joints["pelvis"]
        n = self.joints["neck"]
        
        # 힙 허브: 척추 각도에 비례하여 오프셋 적용
        b = self.joints["belly"]
        dx, dy = b[0] - p[0], b[1] - p[1]
        h = math.hypot(dx, dy)
        if h < EPS: h = EPS
        nx, ny = -dy/h, dx/h # 수직 벡터
        
        self.joints["r_hip"] = [p[0] + nx * self._hip_offset, p[1] + ny * self._hip_offset]
        self.joints["l_hip"] = [p[0] - nx * self._hip_offset, p[1] - ny * self._hip_offset]
        
        # 숄더 허브: Neck 위치에 맞춤 (demo에서는 Chest-Neck 라인에 맞춤)
        c = self.joints["chest"]
        dx, dy = n[0] - c[0], n[1] - c[1]
        h = math.hypot(dx, dy)
        if h < EPS: h = EPS
        nx, ny = -dy/h, dx/h # 수직 벡터
        
        self.joints["r_shoulder"] = [n[0] + nx * self._shoulder_offset, n[1] + ny * self._shoulder_offset]
        self.joints["l_shoulder"] = [n[0] - nx * self._shoulder_offset, n[1] - ny * self._shoulder_offset]

    def solve_ik(self, drag_joint_key, target_pos):
        """마우스로 드래그하는 관절에 맞는 FABRIK 체인 솔버 실행"""
        # 1. Pelvis Drag (Root, Pin): 전신 이동
        if drag_joint_key == "pelvis":
            dx = target_pos[0] - self.joints["pelvis"][0]
            dy = target_pos[1] - self.joints["pelvis"][1]
            for j in self.joints:
                self.joints[j][0] += dx
                self.joints[j][1] += dy
            return

        # 2. 체인 말단 드래그 -> 해당 FABRIK 체인 솔브
        chain_data = None
        if drag_joint_key == "head":
            chain_data = (self.spine_chain_keys, self.spine_lens)
        elif drag_joint_key == "r_hand":
            chain_data = (self.r_arm_chain_keys, self.r_arm_lens)
        elif drag_joint_key == "l_hand":
            chain_data = (self.l_arm_chain_keys, self.l_arm_lens)
        elif drag_joint_key == "r_foot":
            chain_data = (self.r_leg_chain_keys, self.r_leg_lens)
        elif drag_joint_key == "l_foot":
            chain_data = (self.l_leg_chain_keys, self.l_leg_lens)
            
        if chain_data:
            keys, lens = chain_data
            # 체인 현재 위치 수집
            current_pos = [self.joints[k] for k in keys]
            # FABRIK 실행 (Pelvis drag가 아니므로 anchor는 고정됨)
            solved_pos = fabrik_solve_chain(current_pos, lens, target_pos)
            # 결과 업데이트
            for i, k in enumerate(keys):
                self.joints[k] = solved_pos[i]

            # 3. 척추가 변경되었다면 허브 업데이트 (다른 체인의 anchor 위치 갱신)
            if drag_joint_key == "head":
                self._update_hubs()

# ==========================================
# 3. PyQt6 Canvas UI 및 상호작용
# ==========================================
class StickmanCanvas(QWidget):
    def __init__(self, model: StickmanModel):
        super().__init__()
        self.model = model
        self.drag_key = None
        self.setMinimumSize(900, 700)
        self.setMouseTracking(True) # 마우스 움직임 트래킹 활성화

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # 1. 링크 렌더링
        pen_line = QPen(QColor(50, 50, 50), 5, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundJoin, Qt.PenJoinStyle.RoundJoin)
        painter.setPen(pen_line)
        for p_key, c_key in self.model.lines:
            pt1 = QPointF(*self.model.joints[p_key])
            pt2 = QPointF(*self.model.joints[c_key])
            painter.drawLine(pt1, pt2)

        # 2. 관절 노드 렌더링
        pen_node = QPen(QColor(0, 0, 0), 2)
        brush_normal = QBrush(QColor(255, 255, 255))
        brush_leaf = QBrush(QColor(100, 149, 237)) #IK 말단 노드 (CornflowerBlue)
        brush_root = QBrush(QColor(220, 20, 60))  #Root 노드 (Crimson)
        
        leaf_nodes = ["head", "r_hand", "l_hand", "r_foot", "l_foot"]
        
        painter.setPen(pen_node)
        for nid, (x, y) in self.model.joints.items():
            if nid == "pelvis":
                painter.setBrush(brush_root)
                painter.drawRect(int(x-7), int(y-7), 14, 14) # Pelvis는 사각형
            else:
                if nid in leaf_nodes:
                    painter.setBrush(brush_leaf)
                else:
                    painter.setBrush(brush_normal)
                painter.drawEllipse(QPointF(x, y), 8, 8)

    def mousePressEvent(self, event):
        mx, my = event.position().x(), event.position().y()
        
        # 드래그할 노드 피킹
        clicked = None
        min_dist = 18.0
        
        # 1. IK 말단 노드 우선 피킹
        leaf_nodes = ["head", "r_hand", "l_hand", "r_foot", "l_foot", "pelvis"]
        for nid in leaf_nodes:
            x, y = self.model.joints[nid]
            d = math.hypot(x - mx, y - my)
            if d < min_dist:
                min_dist = d
                clicked = nid
                
        if clicked and event.button() == Qt.MouseButton.LeftButton:
            self.drag_key = clicked
            self.update()

    def mouseMoveEvent(self, event):
        if self.drag_key:
            target_pos = [event.position().x(), event.position().y()]
            self.model.solve_ik(self.drag_key, target_pos)
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.drag_key = None
            self.update()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Stickman FABRIK Chain IK Editor (No Popping)")
        
        self.model = StickmanModel()
        self.canvas = StickmanCanvas(self.model)
        
        layout = QVBoxLayout()
        label = QLabel("빨간색(Pelvis) 드래그: 전신 이동 / 파란색(Head, Hands, Feet) 드래그: 부드러운 IK 체인 솔버 작동")
        label.setStyleSheet("font-weight: bold; color: #333;")
        layout.addWidget(label)
        layout.addWidget(self.canvas)
        
        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
