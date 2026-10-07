# ============================================================
# Constraint Solver (v2.9 - R/F Rule 연동 최적화)
# ============================================================
class ConstraintSolver:
    def __init__(self, model): 
        self.model = model
        self.active_drag_node = None

    def solve(self, iterations=60):
        # 1. R 규칙 및 Line 길이 제약조건을 우선 해결
        for _ in range(iterations):
            changed = False
            for constraint in self.model.reflection_constraints:
                if constraint.enabled and self.solve_reflection_constraint(constraint): 
                    changed = True
            for line in self.model.lines:
                if not line.flexible and self.solve_line_length(line): 
                    changed = True
            if not changed: 
                break

        # 2. R 규칙 적용으로 인해 노드 위치가 변한 경우, F 규칙의 Target Angle을 자동 업데이트
        #    (R 규칙으로 결정된 관절 각도를 새로운 기준 각도로 수용)
        if self.active_drag_node:
            for f_constraint in self.model.angle_constraints:
                # 드래그 중인 노드 혹은 R 규칙 영향권에 있는 노드와 연결된 F 규칙 업데이트
                f_constraint.update_target()

        # 3. F 규칙 (각도 제약) 해결
        for _ in range(iterations):
            changed = False
            for constraint in self.model.angle_constraints:
                if constraint.enabled and self.solve_angle_constraint(constraint): 
                    changed = True
            if not changed: 
                break

        # 4. Solver 실행 후 가변 Line들의 rest_length 최신화
        for line in self.model.lines:
            if line.flexible:
                line.rest_length = distance(line.node_a.position, line.node_b.position)

    def solve_line_length(self, line):
        a, b = line.node_a, line.node_b
        w_a, w_b = a.inv_mass, b.inv_mass
        w_sum = w_a + w_b
        if w_sum == 0: return False

        dx, dy = b.position.x() - a.position.x(), b.position.y() - a.position.y()
        current_length = math.hypot(dx, dy)
        if current_length < 1e-6: return False

        error = current_length - line.rest_length
        if abs(error) < 1e-5: return False

        nx, ny = dx / current_length, dy / current_length
        stiffness = 0.8
        corr_a, corr_b = error * (w_a / w_sum) * stiffness, error * (w_b / w_sum) * stiffness

        a.set_position(QPointF(a.position.x() + nx * corr_a, a.position.y() + ny * corr_a))
        b.set_position(QPointF(b.position.x() - nx * corr_b, b.position.y() - ny * corr_b))
        return True

    def solve_angle_constraint(self, constraint):
        c, a, b = constraint.center, constraint.node_a, constraint.node_b
        w_a, w_b = a.inv_mass, b.inv_mass
        w_sum = w_a + w_b
        if w_sum == 0: return False

        error = normalize_angle(angle_between(c.position, a.position, b.position) - constraint.target_angle)
        if abs(error) < 1e-5: return False

        stiffness = 0.5
        corr_a, corr_b = error * (w_a / w_sum) * stiffness, error * (w_b / w_sum) * stiffness

        a.set_position(rotate_point(a.position, c.position, corr_a))
        b.set_position(rotate_point(b.position, c.position, -corr_b))
        return True

    def solve_reflection_constraint(self, constraint):
        c, n1, n3 = constraint.center, constraint.node1, constraint.node3
        changed = False

        # 조건: 1번 또는 3번 노드가 직접 드래그의 '시작점(주체)'일 때만 R 규칙 계산 적용
        if self.active_drag_node is n3:
            v3_x = n3.position.x() - c.position.x()
            v3_y = n3.position.y() - c.position.y()
            curr_dist3 = math.hypot(v3_x, v3_y)
            if curr_dist3 < 1e-6: return False

            # Max Distance Clamping (3번 노드)
            if curr_dist3 > constraint.max_dist3:
                v3_x = (v3_x / curr_dist3) * constraint.max_dist3
                v3_y = (v3_y / curr_dist3) * constraint.max_dist3
                n3.set_position(QPointF(c.position.x() + v3_x, c.position.y() + v3_y))
                changed = True

            # Node 1 위치 대칭 및 비례 적용
            target_x = c.position.x() - v3_x / constraint.dist_ratio
            target_y = c.position.y() - v3_y / constraint.dist_ratio
            target_pos = QPointF(target_x, target_y)

            if distance(n1.position, target_pos) > 1e-5:
                n1.set_position(target_pos)
                changed = True

        elif self.active_drag_node is n1:
            v1_x = n1.position.x() - c.position.x()
            v1_y = n1.position.y() - c.position.y()
            curr_dist1 = math.hypot(v1_x, v1_y)
            if curr_dist1 < 1e-6: return False

            # Max Distance Clamping (1번 노드)
            if curr_dist1 > constraint.max_dist1:
                v1_x = (v1_x / curr_dist1) * constraint.max_dist1
                v1_y = (v1_y / curr_dist1) * constraint.max_dist1
                n1.set_position(QPointF(c.position.x() + v1_x, c.position.y() + v1_y))
                changed = True

            # Node 3 위치 대칭 및 비례 적용
            target_x = c.position.x() - v1_x * constraint.dist_ratio
            target_y = c.position.y() - v1_y * constraint.dist_ratio
            target_pos = QPointF(target_x, target_y)

            if distance(n3.position, target_pos) > 1e-5:
                n3.set_position(target_pos)
                changed = True

        # 1번, 3번 노드가 드래그 주체가 아닌 경우(예: 중심 노드 2번 이동 등):
        # 위치를 재계산하지 않고 기존 상대 대칭 및 거리 관계를 유지한 채 함께 이동
        return changed