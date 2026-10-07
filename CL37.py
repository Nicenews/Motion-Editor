# CL-3.7
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
    _next_id = 1
    def __init__(self, center, node1, node3):
        self.id, ReflectionConstraint._next_id = ReflectionConstraint._next_id, ReflectionConstraint._next_id + 1
        self.center, self.node1, self.node3 = center, node1, node3
        self.enabled = True

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

    def add_rule(self, rule_type, *node_ids):
        rule_type = str(rule_type).upper()
        if rule_type == 'F':
            if len(node_ids) != 3: raise ValueError("F Rule�� 3媛쒖쓽 �몃뱶 ID媛� �꾩슂�⑸땲��.")
            c, a, b = self.nodes_by_id[node_ids[1]], self.nodes_by_id[node_ids[0]], self.nodes_by_id[node_ids[2]]
            constraint = AngleConstraint(c, a, b)
            self.angle_constraints.append(constraint)
            return constraint
        elif rule_type == 'R':
            if len(node_ids) != 3: raise ValueError("R Rule�� 3媛쒖쓽 �몃뱶 ID媛� �꾩슂�⑸땲��.")
            c, n1, n3 = self.nodes_by_id[node_ids[1]], self.nodes_by_id[node_ids[0]], self.nodes_by_id[node_ids[2]]
            constraint = ReflectionConstraint(c, n1, n3)
            self.reflection_constraints.append(constraint)
            return constraint
        else:
            raise ValueError(f"吏��먰븯吏� �딅뒗 Rule ���낆엯�덈떎: {rule_type}")

# ============================================================
# Constraint Solver
# ============================================================
class ConstraintSolver:
    def __init__(self, model):
        self.model = model
        self.active_drag_nodes = set()
        self._followers = []
        self._released_lines = []

    def is_dragging(self, node):
        # �� �쒕옒洹� �� 怨좎젙 �앹젏�� active�� �ㅼ뼱�ㅻ�濡�, 怨좎젙 �몃뱶�� �쒕옒洹� 以묒쑝濡� 蹂댁� �딆쓬
        return node in self.active_drag_nodes and not node.fixed

    def weight(self, node):
        if node.fixed or self.is_dragging(node) or node in self._followers: return 0.0
        return 1.0

    def prepare(self):
        self._followers, self._released_lines = [], []
        for rc in self.model.reflection_constraints:
            if not rc.enabled: continue

            drag_1 = self.is_dragging(rc.node1)
            drag_3 = self.is_dragging(rc.node3)
            if not (drag_1 or drag_3): continue

            # �곷��� �몃뱶媛� 怨좎젙�대㈃ R �대룞 �먯껜媛� 遺덇��� �� ��移� �꾩튂濡� �섎룎由ш퀬 洹쒖튃 �곸슜 �� ��
            if (drag_3 and rc.node1.fixed) or (drag_1 and rc.node3.fixed):
                self._snap_blocked(rc, drag_3)
                continue

            if drag_3 and rc.node1 not in self._followers: self._followers.append(rc.node1)
            elif drag_1 and rc.node3 not in self._followers: self._followers.append(rc.node3)

            for line in self.model.lines:
                ends = (line.node_a, line.node_b)
                if rc.center in ends and (rc.node1 in ends or rc.node3 in ends):
                    if line not in self._released_lines: self._released_lines.append(line)

    def _snap_blocked(self, rc, drag_3):
        c, r = rc.center.position, rc.dist_ratio
        if drag_3:   # 1踰� 怨좎젙 �� 3踰덉씠 �덉뼱�� �� �먮━
            p = rc.node1.position
            rc.node3.position = QPointF(c.x() - (p.x() - c.x()) * r, c.y() - (p.y() - c.y()) * r)
        else:        # 3踰� 怨좎젙 �� 1踰덉씠 �덉뼱�� �� �먮━
            p = rc.node3.position
            rc.node1.position = QPointF(c.x() - (p.x() - c.x()) / r, c.y() - (p.y() - c.y()) / r)

    def solve(self, iterations=20):
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

        for line in self._released_lines:
            line.rest_length = distance(line.node_a.position, line.node_b.position)

    def solve_line_length(self, line):
        a, b = line.node_a, line.node_b
        w_a, w_b = self.weight(a), self.weight(b)
        if w_a + w_b == 0:
            if self.is_dragging(a) and b.fixed: w_a = 1.0
            elif self.is_dragging(b) and a.fixed: w_b = 1.0
            else: return False
        w_sum = w_a + w_b

        dx, dy = b.position.x() - a.position.x(), b.position.y() - a.position.y()
        current_length = math.hypot(dx, dy)
        if current_length < 1e-6: return False

        error = current_length - line.rest_length
        if abs(error) < 1e-4: return False

        nx, ny = dx / current_length, dy / current_length
        stiffness = 0.8
        corr_a, corr_b = error * (w_a / w_sum) * stiffness, error * (w_b / w_sum) * stiffness

        a.set_position(QPointF(a.position.x() + nx * corr_a, a.position.y() + ny * corr_a))
        b.set_position(QPointF(b.position.x() - nx * corr_b, b.position.y() - ny * corr_b))
        return True

    # ---------------- F 洹쒖튃 (媛곷룄 怨좎젙) ----------------
    def _line_between(self, n1, n2):
        for line in self.model.lines:
            if (line.node_a is n1 and line.node_b is n2) or (line.node_a is n2 and line.node_b is n1):
                return line
        return None

    def _anchor_setup(self, k):
        """F 洹쒖튃�� �쒖そ �앸쭔 怨좎젙�대㈃ (anchor, tip, sign, �듭빱-以묒떖 湲몄씠, ��-以묒떖 湲몄씠) 諛섑솚."""
        c, a, b = k.center, k.node_a, k.node_b
        if c.fixed or a.fixed == b.fixed: return None
        anchor, tip, sign = (a, b, 1.0) if a.fixed else (b, a, -1.0)
        l1, l2 = self._line_between(c, anchor), self._line_between(c, tip)
        if l1 is None or l2 is None: return None
        return anchor, tip, sign, l1.rest_length, l2.rest_length

    def _solve_anchor_center_drag(self, k, anchor, tip, sign, l_ac, l_ct):
        """[洹쒖튃 2] 以묒떖(2踰�) �쒕옒洹�: �듭빱(1踰�) 以묒떖 �� �꾨줈 �쒗븳(1-2 湲몄씠 �좎�), ��(4踰�)�� F 媛곷룄쨌湲몄씠 �좎�."""
        c = k.center
        ux, uy = c.position.x() - anchor.position.x(), c.position.y() - anchor.position.y()
        d = math.hypot(ux, uy)
        if d < 1e-6: return False
        new_c = QPointF(anchor.position.x() + ux / d * l_ac, anchor.position.y() + uy / d * l_ac)

        ang = math.atan2(anchor.position.y() - new_c.y(), anchor.position.x() - new_c.x()) + sign * k.target_angle
        new_tip = QPointF(new_c.x() + math.cos(ang) * l_ct, new_c.y() + math.sin(ang) * l_ct)

        changed = distance(c.position, new_c) > 1e-4 or distance(tip.position, new_tip) > 1e-4
        c.set_position(new_c)
        tip.set_position(new_tip)
        return changed

    def _solve_anchor_tip_drag(self, k, anchor, tip, l_ac, l_ct):
        """[洹쒖튃 3] ��(4踰�) �쒕옒洹�: F 臾댁떆, 1-2쨌2-4 湲몄씠 �좎��섎ŉ �� �먯쓽 援먯젏�쇰줈 以묒떖(2踰�) 諛곗튂."""
        c, p, t = k.center, anchor.position, tip.position
        dx, dy = t.x() - p.x(), t.y() - p.y()
        d = math.hypot(dx, dy)
        if d < 1e-6: return False
        ux, uy = dx / d, dy / d

        # �곸씠 �우쓣 �� �덈뒗 嫄곕━ 踰붿쐞濡� �쒗븳
        dc = min(max(d, abs(l_ac - l_ct)), l_ac + l_ct)
        if dc < 1e-6: return False
        new_tip = QPointF(p.x() + ux * dc, p.y() + uy * dc)

        a_ = (l_ac * l_ac - l_ct * l_ct + dc * dc) / (2 * dc)
        h = math.sqrt(max(l_ac * l_ac - a_ * a_, 0.0))
        mx, my = p.x() + ux * a_, p.y() + uy * a_
        c1, c2 = QPointF(mx - uy * h, my + ux * h), QPointF(mx + uy * h, my - ux * h)
        new_c = c1 if distance(c1, c.position) <= distance(c2, c.position) else c2

        changed = distance(c.position, new_c) > 1e-4 or distance(tip.position, new_tip) > 1e-4
        c.set_position(new_c)
        tip.set_position(new_tip)
        return changed

    def solve_angle_constraint(self, constraint):
        c, a, b = constraint.center, constraint.node_a, constraint.node_b
        if a.fixed and b.fixed: return False

        # �쒖そ ��(�듭빱)�� 怨좎젙�� 寃쎌슦 �꾩슜 泥섎━
        setup = self._anchor_setup(constraint)
        if setup:
            anchor, tip, sign, l_ac, l_ct = setup
            if self.is_dragging(c):
                return self._solve_anchor_center_drag(constraint, anchor, tip, sign, l_ac, l_ct)
            if self.is_dragging(tip):
                return self._solve_anchor_tip_drag(constraint, anchor, tip, l_ac, l_ct)

        w_a, w_b = self.weight(a), self.weight(b)
        if self.is_dragging(c):
            w_a, w_b = 1.0, 0.0

        w_sum = w_a + w_b
        if w_sum == 0: return False

        curr_angle = angle_between(c.position, a.position, b.position)
        error = normalize_angle(curr_angle - constraint.target_angle)
        if abs(error) < 1e-4: return False

        stiffness = 0.8
        corr_a = error * (w_a / w_sum) * stiffness
        corr_b = error * (w_b / w_sum) * stiffness

        if w_a > 0:
            a.set_position(rotate_point(a.position, c.position, corr_a))
        if w_b > 0:
            b.set_position(rotate_point(b.position, c.position, -corr_b))

        return True

    # ---------------- R 洹쒖튃 (��移� �대룞) ----------------
    def _move_to(self, node, pos):
        if distance(node.position, pos) > 1e-4:
            node.set_position(pos)
            return True
        return False

    def solve_reflection_constraint(self, constraint):
        c, n1, n3 = constraint.center, constraint.node1, constraint.node3
        r = constraint.dist_ratio
        changed = False

        # 1쨌3踰� 紐⑤몢 怨좎젙: 2踰� �꾩튂媛� �섎굹濡� 寃곗젙��
        if n1.fixed and n3.fixed:
            target = QPointF((n3.position.x() + r * n1.position.x()) / (1 + r),
                             (n3.position.y() + r * n1.position.y()) / (1 + r))
            if distance(c.position, target) > 1e-4:
                c.set_position(target)
                return True
            return False

        # [洹쒖튃 1] �곷��몄씠 怨좎젙�대㈃ �꾨뒗 �몃뱶�� ��移� �꾩튂�� 癒몃Ь (�대룞 湲덉�)
        if self.is_dragging(n3) and n1.fixed:
            return self._move_to(n3, QPointF(c.position.x() - (n1.position.x() - c.position.x()) * r,
                                             c.position.y() - (n1.position.y() - c.position.y()) * r))
        if self.is_dragging(n1) and n3.fixed:
            return self._move_to(n1, QPointF(c.position.x() - (n3.position.x() - c.position.x()) / r,
                                             c.position.y() - (n3.position.y() - c.position.y()) / r))

        if self.is_dragging(n3):
            v3_x = n3.position.x() - c.position.x()
            v3_y = n3.position.y() - c.position.y()
            curr_dist3 = math.hypot(v3_x, v3_y)
            if curr_dist3 < 1e-6: return False

            if curr_dist3 > constraint.max_dist3:
                v3_x = (v3_x / curr_dist3) * constraint.max_dist3
                v3_y = (v3_y / curr_dist3) * constraint.max_dist3
                n3.set_position(QPointF(c.position.x() + v3_x, c.position.y() + v3_y))
                changed = True

            target_pos = QPointF(c.position.x() - v3_x / r,
                                 c.position.y() - v3_y / r)
            if distance(n1.position, target_pos) > 1e-4:
                n1.set_position(target_pos)
                changed = True

        elif self.is_dragging(n1):
            v1_x = n1.position.x() - c.position.x()
            v1_y = n1.position.y() - c.position.y()
            curr_dist1 = math.hypot(v1_x, v1_y)
            if curr_dist1 < 1e-6: return False

            if curr_dist1 > constraint.max_dist1:
                v1_x = (v1_x / curr_dist1) * constraint.max_dist1
                v1_y = (v1_y / curr_dist1) * constraint.max_dist1
                n1.set_position(QPointF(c.position.x() + v1_x, c.position.y() + v1_y))
                changed = True

            target_pos = QPointF(c.position.x() - v1_x * r,
                                 c.position.y() - v1_y * r)
            if distance(n3.position, target_pos) > 1e-4:
                n3.set_position(target_pos)
                changed = True

        else:
            if n3.fixed and not n1.fixed:   # 3踰� 湲곗��쇰줈 1踰덉씠 ��移� 異붿쥌
                target_pos = QPointF(c.position.x() - (n3.position.x() - c.position.x()) / r,
                                     c.position.y() - (n3.position.y() - c.position.y()) / r)
                if distance(n1.position, target_pos) > 1e-4:
                    n1.set_position(target_pos)
                    changed = True
            else:                           # 1踰� 湲곗��쇰줈 3踰덉씠 ��移� 異붿쥌
                target_pos = QPointF(c.position.x() - (n1.position.x() - c.position.x()) * r,
                                     c.position.y() - (n1.position.y() - c.position.y()) * r)
                if distance(n3.position, target_pos) > 1e-4:
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
        self.retarget_constraints = []
        self.setBrush(QBrush(QColor("red") if node.fixed else QColor("white")))
        self.setPen(QPen(QColor("black"), 1))
        self.setZValue(1)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setPos(node.position)

    def update_from_model(self):
        self.setPos(self.node.position)
        self.setBrush(QBrush(QColor("red") if self.node.fixed else QColor("white")))

    def mousePressEvent(self, event):
        if event.button() != Qt.MouseButton.LeftButton: return

        # [洹쒖튃 1] �곷��몄씠 怨좎젙�대㈃ �대룞 遺덇� (ignore() �섎㈃ �꾨옒 �좎쑝濡� �대깽�멸� �섏뼱媛�誘�濡� accept �� 醫낅즺)
        if self.node.fixed or self.editor.is_move_blocked(self.node):
            event.accept()
            return

        self.dragging = True
        self.editor.solver.active_drag_nodes.add(self.node)
        self.drag_start_scene, self.drag_start_position = event.scenePos(), QPointF(self.node.position)
        self.disabled_constraints.clear()

        is_r_direct_node = False
        for rc in self.editor.model.reflection_constraints:
            if self.node is rc.node1 or self.node is rc.node3:
                is_r_direct_node = True
                break

        if is_r_direct_node:
            for constraint in self.editor.model.angle_constraints:
                if constraint.enabled:
                    constraint.enabled = False
                    self.disabled_constraints.append(constraint)

        # [洹쒖튃 3] �쒖そ �앹씠 怨좎젙�� F 洹쒖튃�� 諛섎�履� ��(1踰� 怨좎젙 �� 4踰�)�� �뚮㈃, �볦쓣 �� 媛곷룄 Update
        self.retarget_constraints = [
            k for k in self.editor.model.angle_constraints
            if k.enabled and k.center is not self.node and
            ((k.node_a.fixed and k.node_b is self.node) or (k.node_b.fixed and k.node_a is self.node))
        ]

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
        self.editor.solver.active_drag_nodes.discard(self.node)

        for constraint in self.disabled_constraints:
            constraint.update_target()
            constraint.enabled = True
        self.disabled_constraints.clear()

        for constraint in self.retarget_constraints:
            constraint.update_target()
        self.retarget_constraints.clear()

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
        self.disabled_constraints = []
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

        # [洹쒖튃 1] �대룞 湲덉� �몃뱶媛� �ы븿�� �좎� �� �� �놁쓬
        if self.editor.is_move_blocked(self.line_model.node_a) or self.editor.is_move_blocked(self.line_model.node_b):
            event.accept()
            return

        self.dragging, self.previous_scene_position = True, event.scenePos()
        self.editor.solver.active_drag_nodes.add(self.line_model.node_a)
        self.editor.solver.active_drag_nodes.add(self.line_model.node_b)
        self.disabled_constraints.clear()

        # �쇱씤�� �� �� R 洹쒖튃怨� �곌��� �몃뱶(1踰�, 3踰�)媛� �ы븿�섏뼱 �덈떎硫� F 洹쒖튃 �꾩떆 鍮꾪솢�깊솕
        line_nodes = {self.line_model.node_a, self.line_model.node_b}
        is_r_line = False
        for rc in self.editor.model.reflection_constraints:
            if (rc.node1 in line_nodes and not rc.node1.fixed) or (rc.node3 in line_nodes and not rc.node3.fixed):
                is_r_line = True
                break

        if is_r_line:
            for constraint in self.editor.model.angle_constraints:
                if constraint.enabled:
                    constraint.enabled = False
                    self.disabled_constraints.append(constraint)

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
        if event.button() != Qt.MouseButton.LeftButton or not self.dragging: return
        self.dragging = False
        self.editor.solver.active_drag_nodes.discard(self.line_model.node_a)
        self.editor.solver.active_drag_nodes.discard(self.line_model.node_b)

        # �쇱씤 �대룞 �� 理쒖떊 諛곗튂 �곹깭濡� F 洹쒖튃�� target_angle 媛깆떊 諛� �ы솢�깊솕
        for constraint in self.disabled_constraints:
            constraint.update_target()
            constraint.enabled = True
        self.disabled_constraints.clear()

        self.editor.update_graphics()
        self.editor.solve_from_interaction()
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

    def add_rule(self, rule_type, *node_ids):
        return self.model.add_rule(rule_type, *node_ids)

    def is_move_blocked(self, node):
        """R 洹쒖튃�� �대룞 遺덇��� �몃뱶: �곷���(1��3)�� 怨좎젙�대㈃ 3쨌1踰�, 1쨌3踰덉씠 紐⑤몢 怨좎젙�대㈃ 以묒떖(2踰�)."""
        for rc in self.model.reflection_constraints:
            if not rc.enabled: continue
            if node is rc.node3 and rc.node1.fixed: return True
            if node is rc.node1 and rc.node3.fixed: return True
            if node is rc.center and rc.node1.fixed and rc.node3.fixed: return True
        # �좎쑝濡� �곌껐�� �댁썐 以� 2媛� �댁긽�� 怨좎젙�대㈃ �꾩튂媛� �ъ떎�� �뺥빐�� �덉뼱(諛섎�履� 援먯젏�쇰줈 �ㅼ쭛�� �섎쭔 �덉쓬) �대룞 遺덇�
        fixed_neighbors = sum(1 for l in node.lines if (l.node_b if l.node_a is node else l.node_a).fixed)
        if not node.fixed and fixed_neighbors >= 2: return True
        return False

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
        self.setWindowTitle("PyQt6 Node Line Constraint Editor v3.7")
        self.resize(1000, 700)
        self.editor = ShapeEditor()
        self.setCentralWidget(self.editor)
        self.create_demo()

    def create_demo(self):
        ed = self.editor

        # 1. �몃뱶 �앹꽦
        ed.add_node(1, 200, 200, fixed=True)  # 議곗옉 �몃뱶 1 (湲곕낯 怨좎젙, �붾툝�대┃�쇰줈 �댁젣)
        ed.add_node(2, 350, 300)       # 以묒떖 �몃뱶 2
        ed.add_node(3, 500, 400)       # 議곗옉 �몃뱶 3 (2踰� 湲곗� 1踰덉쓽 180�� ��移� �꾩튂)
        ed.add_node(4, 250, 450)       # F 猷곗슜 �몃뱶 4

        # 2. �좊텇 �앹꽦
        ed.add_line(1, 2)
        ed.add_line(2, 3)
        ed.add_line(2, 4)

        # 3. Rule �곸슜
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