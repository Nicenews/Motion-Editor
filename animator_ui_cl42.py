# CL-4.2
"""
Animator UI (PyQt6)  -  캔버스 + 타임라인 + 재생.
계산과 마우스 판단은 animation_cl42.py(TimelineModel / TimelineController / AnimationEditor)가 맡고,
이 파일은 그리기와 이벤트 전달만 한다.

실행:  python animator_ui_cl42.py [저장한_애니메이션.json]

조작
  캔버스      좌클릭 드래그: 노드/선 이동(자동 키), Alt: 자유 이동, 우클릭: 고정 토글(편집 보조)
  타임라인    빈 곳 클릭/드래그: 시간 이동, ◆ 클릭: 선택, ◆ 드래그: 키 이동(Alt: 복제),
              우클릭 ◆: 보간(선형/이징/홀드)·삭제, 휠: 이동, Ctrl+휠: 확대/축소
  단축키      Space 재생/정지, ←/→ 한 프레임, K 현재 프레임에 키, Delete 선택 키 삭제,
              Ctrl+Z/Y Undo/Redo, Ctrl+S 저장, Ctrl+O 불러오기
"""
import math
import sys

from animation_cl42 import (AnimationEditor, TimelineController, TimelineModel, advance_time)
from pose_editor_cl317 import build_demo


def run_ui(path=None):
    from PyQt6.QtCore import QElapsedTimer, Qt, QTimer, pyqtSignal
    from PyQt6.QtGui import QBrush, QColor, QKeySequence, QPainter, QPen, QPolygonF, QShortcut
    from PyQt6.QtCore import QPointF
    from PyQt6.QtWidgets import (QApplication, QCheckBox, QFileDialog, QHBoxLayout, QLabel, QMenu,
                                 QPushButton, QVBoxLayout, QWidget)

    app = QApplication(sys.argv)
    ae = AnimationEditor.load(path) if path else AnimationEditor(build_demo())

    # ------------------------------------------------------------ 캔버스
    class Canvas(QWidget):
        edited = pyqtSignal()

        def __init__(self):
            super().__init__()
            self.dragging = False
            self.playing = False
            self.setMinimumSize(640, 420)

        def _hit_node(self, x, y):
            for n, p in ae.editor.pose.pos.items():
                if math.hypot(p[0] - x, p[1] - y) <= 10:
                    return n
            return None

        def _hit_link(self, x, y):
            for (a, b) in ae.anim.skel.max_len:
                pa, pb = ae.editor.pose.pos[a], ae.editor.pose.pos[b]
                dx, dy = pb[0] - pa[0], pb[1] - pa[1]
                L2 = dx * dx + dy * dy
                if L2 < 1e-9:
                    continue
                t = max(0.0, min(1.0, ((x - pa[0]) * dx + (y - pa[1]) * dy) / L2))
                if math.hypot(pa[0] + t * dx - x, pa[1] + t * dy - y) <= 6:
                    return (a, b)
            return None

        def paintEvent(self, _ev):
            qp = QPainter(self)
            qp.setRenderHint(QPainter.RenderHint.Antialiasing)
            qp.fillRect(self.rect(), QColor("white"))
            qp.setPen(QPen(QColor("#555"), 3))
            for (a, b) in ae.anim.skel.max_len:
                pa, pb = ae.editor.pose.pos[a], ae.editor.pose.pos[b]
                qp.drawLine(int(pa[0]), int(pa[1]), int(pb[0]), int(pb[1]))
            for n, p in ae.editor.pose.pos.items():
                pinned = n in ae.editor.pose.pinned
                qp.setPen(QPen(QColor("#222"), 2))
                qp.setBrush(QBrush(QColor("#e55" if pinned else "#6ab")))
                qp.drawEllipse(int(p[0]) - 8, int(p[1]) - 8, 16, 16)
            qp.setPen(QColor("#333"))
            kind = "키프레임" if ae.is_key() else "계산된 프레임"
            qp.drawText(10, 20, f"프레임 {ae.t:.1f}  ({kind})")

        def mousePressEvent(self, ev):
            if self.playing:
                return
            x, y = ev.position().x(), ev.position().y()
            alt = bool(ev.modifiers() & Qt.KeyboardModifier.AltModifier)
            n = self._hit_node(x, y)
            if ev.button() == Qt.MouseButton.RightButton:
                if n is not None:
                    ae.toggle_pin(n)
            elif ev.button() == Qt.MouseButton.LeftButton:
                if n is not None:
                    self.dragging = ae.begin_drag_node(n, alt)
                else:
                    lk = self._hit_link(x, y)
                    if lk:
                        self.dragging = ae.begin_drag_link(lk[0], lk[1], (x, y), alt)
            self.update()

        def mouseMoveEvent(self, ev):
            if self.dragging:
                alt = bool(ev.modifiers() & Qt.KeyboardModifier.AltModifier)
                ae.drag_move((ev.position().x(), ev.position().y()), alt)
                self.update()

        def mouseReleaseEvent(self, _ev):
            if self.dragging:
                ae.end_drag()
                self.dragging = False
                self.edited.emit()                 # 키가 생겼을 수 있으므로 타임라인도 다시 그림
            self.update()

    # ------------------------------------------------------------ 타임라인
    class Timeline(QWidget):
        changed = pyqtSignal()

        def __init__(self):
            super().__init__()
            self.setMinimumHeight(92)
            self.model = TimelineModel(width=800, view_span=max(48, ae.anim.length + 8))
            self.ctrl = TimelineController(ae, self.model)
            self.playing = False

        def paintEvent(self, _ev):
            m, qp = self.model, QPainter(self)
            m.width = self.width()
            qp.setRenderHint(QPainter.RenderHint.Antialiasing)
            qp.fillRect(self.rect(), QColor("#f4f6f8"))
            h = self.height()
            qp.setPen(QPen(QColor("#999"), 1))
            for f, major in m.ticks():
                x = int(m.frame_to_x(f))
                qp.drawLine(x, 4 if major else 12, x, 22)
                if major:
                    qp.drawText(x + 3, 16, str(f))
            row = 52
            frames = ae.key_frames()
            for a, b in zip(frames, frames[1:]):                       # 구간 막대(보간 방식별 색)
                mode = ae.anim.keys[a].interp
                color = {"linear": "#b0bec5", "ease": "#64b5f6", "hold": "#e57373"}[mode]
                qp.setPen(QPen(QColor(color), 4))
                qp.drawLine(int(m.frame_to_x(a)), row, int(m.frame_to_x(b)), row)
            qp.setPen(QPen(QColor("#888"), 1, Qt.PenStyle.DashLine))   # 애니메이션 길이 끝
            ex = int(m.frame_to_x(ae.anim.length))
            qp.drawLine(ex, 24, ex, h - 4)
            for f in frames:                                           # ◆ 키 마커
                x = m.frame_to_x(f)
                sel = (f == self.ctrl.selected)
                qp.setPen(QPen(QColor("#333"), 1))
                qp.setBrush(QBrush(QColor("#ff9800" if sel else "#37474f")))
                qp.drawPolygon(QPolygonF([QPointF(x, row - 8), QPointF(x + 7, row), QPointF(x, row + 8), QPointF(x - 7, row)]))
            if self.ctrl.preview is not None:                          # 끄는 중인 키의 놓을 위치
                px = m.frame_to_x(self.ctrl.preview)
                qp.setPen(QPen(QColor("#ff9800"), 1, Qt.PenStyle.DashLine))
                qp.drawLine(int(px), 28, int(px), h - 6)
            x = m.frame_to_x(ae.t)                                     # 재생 헤드
            qp.setPen(QPen(QColor("#d32f2f"), 2))
            qp.drawLine(int(x), 22, int(x), h - 4)
            qp.setBrush(QBrush(QColor("#d32f2f") if ae.is_key() else QColor("white")))   # 키면 채움, 계산된 프레임이면 비움
            qp.drawEllipse(QPointF(x, row + 20), 5, 5)

        def mousePressEvent(self, ev):
            if self.playing:
                return
            x = ev.position().x()
            if ev.button() == Qt.MouseButton.RightButton:
                f = self.model.hit_key(x, ae.key_frames())
                if f is not None:
                    self.ctrl.selected = f
                    self._menu(f, ev.globalPosition().toPoint())
                return
            self.ctrl.press(x)
            self.changed.emit()
            self.update()

        def mouseMoveEvent(self, ev):
            if self.playing:
                return
            self.ctrl.move(ev.position().x())
            self.changed.emit()
            self.update()

        def mouseReleaseEvent(self, ev):
            if self.playing:
                return
            self.ctrl.release(ev.position().x(), copy=bool(ev.modifiers() & Qt.KeyboardModifier.AltModifier))
            self.changed.emit()
            self.update()

        def wheelEvent(self, ev):
            self.ctrl.wheel(ev.position().x(), ev.angleDelta().y(),
                            ctrl=bool(ev.modifiers() & Qt.KeyboardModifier.ControlModifier))
            self.update()

        def _menu(self, f, pos):
            menu = QMenu(self)
            for label, mode in (("보간: 선형", "linear"), ("보간: 이징", "ease"), ("보간: 홀드", "hold")):
                menu.addAction(label, lambda mode=mode: (ae.set_interp(f, mode), self.changed.emit(), self.update()))
            menu.addSeparator()
            menu.addAction("키 삭제", lambda: (self.ctrl.delete_selected(), self.changed.emit(), self.update()))
            menu.exec(pos)

    # ------------------------------------------------------------ 메인 창
    class Main(QWidget):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("Animator CL-4.2")
            self.canvas, self.timeline = Canvas(), Timeline()
            self.play_btn = QPushButton("▶ 재생")
            self.loop_cb = QCheckBox("루프")
            self.auto_cb = QCheckBox("자동 키")
            self.auto_cb.setChecked(True)
            self.label = QLabel()
            step_l, step_r = QPushButton("◀"), QPushButton("▶|")
            key_btn, del_btn = QPushButton("키 추가 (K)"), QPushButton("키 삭제")
            save_btn, load_btn = QPushButton("저장"), QPushButton("불러오기")
            bar = QHBoxLayout()
            for w in (self.play_btn, step_l, step_r, self.loop_cb, self.auto_cb, key_btn, del_btn, save_btn, load_btn, self.label):
                bar.addWidget(w)
            lay = QVBoxLayout(self)
            lay.addWidget(self.canvas, 1)
            lay.addLayout(bar)
            lay.addWidget(self.timeline)

            self.clock, self.timer = QElapsedTimer(), QTimer(self)
            self.timer.timeout.connect(self._tick)
            self.play_btn.clicked.connect(self.toggle_play)
            step_l.clicked.connect(lambda: self.step(-1))
            step_r.clicked.connect(lambda: self.step(1))
            self.loop_cb.toggled.connect(lambda v: setattr(ae.anim, "loop", v))
            self.auto_cb.toggled.connect(lambda v: setattr(ae, "auto_key", v))
            key_btn.clicked.connect(self.add_key)
            del_btn.clicked.connect(self.delete_key)
            save_btn.clicked.connect(self.save)
            load_btn.clicked.connect(self.load)
            self.canvas.edited.connect(self.refresh)
            self.timeline.changed.connect(self.refresh)
            for seq, fn in (("Space", self.toggle_play), ("Left", lambda: self.step(-1)), ("Right", lambda: self.step(1)),
                            ("K", self.add_key), ("Delete", self.delete_key), ("Ctrl+Z", self.undo), ("Ctrl+Y", self.redo),
                            ("Ctrl+S", self.save), ("Ctrl+O", self.load)):
                QShortcut(QKeySequence(seq), self, activated=fn)
            self.loop_cb.setChecked(ae.anim.loop)
            self.refresh()

        def refresh(self):
            self.label.setText(f"프레임 {ae.frame} / {ae.anim.length}  ·  {ae.anim.fps}fps  ·  키 {len(ae.key_frames())}개")
            self.canvas.update()
            self.timeline.update()

        def toggle_play(self):
            playing = not self.timer.isActive()
            self.canvas.playing = self.timeline.playing = playing
            self.play_btn.setText("⏸ 정지" if playing else "▶ 재생")
            if playing:
                if ae.t >= ae.anim.length and not ae.anim.loop:
                    ae.set_time(0)
                self.clock.start()
                self.timer.start(16)
            else:
                self.timer.stop()

        def _tick(self):
            dt = self.clock.restart() / 1000.0
            t, done = advance_time(ae.t, dt, ae.anim.fps, ae.anim.length, ae.anim.loop)
            ae.set_time(t)
            self.timeline.model.ensure_visible(t)
            if done:
                self.toggle_play()
            self.refresh()

        def step(self, d):
            if not self.timer.isActive():
                ae.set_time(max(0, ae.frame + d))
                self.timeline.model.ensure_visible(ae.t)
                self.refresh()

        def add_key(self):
            if not self.timer.isActive():
                ae.add_key()
                self.refresh()

        def delete_key(self):
            if self.timeline.ctrl.delete_selected() or ae.delete_key(ae.frame):
                self.refresh()

        def undo(self):
            if ae.undo():
                self.refresh()

        def redo(self):
            if ae.redo():
                self.refresh()

        def save(self):
            p, _ = QFileDialog.getSaveFileName(self, "저장", "", "Animation (*.json)")
            if p:
                ae.save(p)

        def load(self):
            p, _ = QFileDialog.getOpenFileName(self, "불러오기", "", "Animation (*.json)")
            if p:
                loaded = AnimationEditor.load(p)
                ae.editor, ae.anim, ae.t = loaded.editor, loaded.anim, 0.0
                ae._undo, ae._redo = [], []
                self.timeline.ctrl.selected = None
                self.loop_cb.setChecked(ae.anim.loop)
                ae.set_time(0)
                self.refresh()

    w = Main()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    run_ui(sys.argv[1] if len(sys.argv) > 1 else None)
