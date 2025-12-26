---
sidebar_position: 3
---

# ٹریجکٹری پلیننگ اور ایگزیکیوشن

## ٹریجکٹری پلیننگ کا تعارف

ٹریجکٹری پلیننگ ہیومنوائڈ روبوٹکس کا ایک اہم جزو ہے جو یہ طے کرتا ہے کہ روبوٹ ایک کنفیگریشن سے دوسری کنفیگریشن میں کیسے منتقل ہوتا ہے۔ سادہ پوائنٹ ٹو پوائنٹ حرکات کے برعکس، ہیومنوائڈ روبوٹس کو متعدد ڈیگریز آف فریڈم کی پیچیدہ ٹریجکٹریز کو پلان کرنا چاہیے جو توازن، رکاوٹ سے بچاؤ، توانائی کی کارآمدگی، اور کام کی ضروریات کو مدنظر رکھتی ہوں۔ ٹریجکٹری پلیننگ کا عمل ہائی لیول اہداف کو کم لیول موٹر کنٹرول کمانڈز سے جوڑتا ہے۔

### ہیومنوائڈ ٹریجکٹری پلیننگ میں کلیدی چیلنج

1. **ہائی ڈائی مینش널ٹی**: ہیومنوائڈ روبوٹس کے 30+ ڈیگریز آف فریڈم ہوتے ہیں جنہیں ایک وقت میں مطابق کرنا چاہیے۔

2. **ڈائی نامک کنٹریکٹس**: توازن اور استحکام حرکت کے دوران برقرار رکھنا چاہیے۔

3. **ماحولیاتی تعامل**: پلیننگ کو ماحول کے ساتھ کنٹیکٹ کا احتساب کرنا چاہیے۔

4. **ریل ٹائم کی ضروریات**: بہت سی حرکات کو ماحولیاتی تبدیلیوں کے جواب میں دوبارہ پلین کیا جانا چاہیے۔

5. **انسان نما حرکت**: ٹریجکٹریز کو قدرتی اور انسان نما دکھنا چاہیے۔

## ریاضی کی بنیادیں

### کنفیگریشن اسپیس (C-Space)

کنفیگریشن اسپیس روبوٹ کی تمام ممکنہ جوائنٹ کنفیگریشنز کی نمائندگی کرتی ہے۔ n جوائنٹس والے ہیومنوائڈ روبوٹ کے لیے، C-اسپیس ایک n-ڈائی مینشل اسپیس ہے جہاں ہر پوائنٹ منفرد کنفیگریشن کی نمائندگی کرتا ہے۔

```python
import numpy as np
from scipy.interpolate import CubicSpline

class ConfigurationSpace:
    def __init__(self, joint_limits):
        """
        جوائنٹ لیمٹس کے ساتھ کنفیگریشن اسپیس کو شروع کریں
        joint_limits: ہر جوائنٹ کے لیے (min, max) ٹوپلز کی فہرست
        """
        self.joint_limits = np.array(joint_limits)
        self.n_dof = len(joint_limits)

    def is_valid_configuration(self, q):
        """چیک کریں کہ کیا کنفیگریشن q جوائنٹ لیمٹس کے اندر ہے"""
        return np.all(q >= self.joint_limits[:, 0]) and np.all(q <= self.joint_limits[:, 1])

    def distance(self, q1, q2):
        """دو کنفیگریشنز کے درمیان فاصلہ کا حساب لگائیں"""
        return np.linalg.norm(q2 - q1)

    def interpolate(self, q_start, q_end, t):
        """کنفیگریشنز کے درمیان لکیری انٹرپولیشن"""
        return q_start + t * (q_end - q_start)
```

### پاتھ بمقابلہ ٹریجکٹری

- **پاتھ**: کنفیگریشن اسپیس کے ذریعے جیومیٹرک راستہ (صرف پوزیشن)
- **ٹریجکٹری**: ٹائمنگ معلومات کے ساتھ پاتھ (پوزیشن، رفتار، ایکسلریشن)

## ٹریجکٹری کی نمائندگی

### پولینومیل ٹریجکٹریز

پولینومیل ٹریجکٹریز کو ان کی ہمواری اور حساب کے آسان ہونے کی وجہ سے زیادہ استعمال کیا جاتا ہے:

```python
class PolynomialTrajectory:
    def __init__(self, degree=5):
        self.degree = degree  # 5ویں آرڈر پولینومیل ہموار موشن کے لیے عام ہے

    def generate_5th_order(self, q_start, q_end, t_start, t_end,
                          qd_start=0, qd_end=0, qdd_start=0, qdd_end=0):
        """
        5ویں آرڈر پولینومیل ٹریجکٹری جنریٹ کریں
        q: پوزیشن، qd: رفتار، qdd: ایکسلریشن
        """
        dt = t_end - t_start
        a0 = q_start
        a1 = qd_start
        a2 = qdd_start / 2

        # زیادہ آرڈر کوائف کا حل
        # شروع اور ختم کے باؤنڈری کنڈیشن کا استعمال کرتے ہوئے
        A = np.array([
            [dt**3, dt**4, dt**5],
            [3*dt**2, 4*dt**3, 5*dt**4],
            [6*dt, 12*dt**2, 20*dt**3]
        ])

        b = np.array([
            q_end - a0 - a1*dt - a2*dt**2,
            qd_end - a1 - 2*a2*dt,
            qdd_end - 2*a2
        ])

        a3, a4, a5 = np.linalg.solve(A, b)

        coefficients = [a0, a1, a2, a3, a4, a5]
        return coefficients

    def evaluate(self, coefficients, t, t_start):
        """وقت t پر پوزیشن، رفتار، اور ایکسلریشن کا حساب لگائیں"""
        dt = t - t_start
        pos = sum(c * dt**i for i, c in enumerate(coefficients))

        # رفتار (پہلا ڈیریویٹو)
        vel_coeffs = [i * c for i, c in enumerate(coefficients)][1:]
        vel = sum(c * dt**i for i, c in enumerate(vel_coeffs))

        # ایکسلریشن (دوسرا ڈیریویٹو)
        acc_coeffs = [i * c for i, c in enumerate(vel_coeffs)][1:]
        acc = sum(c * dt**i for i, c in enumerate(acc_coeffs))

        return pos, vel, acc
```

### اسپلائن بیسڈ ٹریجکٹریز

اسپلائنز متعدد ویز پوائنٹس کے ذریعے ہموار انٹرپولیشن فراہم کرتے ہیں:

```python
class SplineTrajectory:
    def __init__(self):
        self.splines = {}  # ہر جوائنٹ کے لیے ایک اسپلائن

    def fit_spline(self, waypoints, times, smoothness=0.1):
        """
        ویز پوائنٹس کے ذریعے کیوبک اسپلائنز فٹ کریں
        waypoints: ہر وقت پر [q1, q2, ..., qn] کی ارے
        times: وقت کے پوائنٹس کی ارے
        """
        n_joints = len(waypoints[0])

        for j in range(n_joints):
            joint_positions = [waypoint[j] for waypoint in waypoints]
            # ہر جوائنٹ کے لیے کیوبک اسپلائن بنائیں
            self.splines[j] = CubicSpline(times, joint_positions,
                                        bc_type='natural',
                                        extrapolate=True)

    def evaluate(self, t):
        """وقت t پر ٹریجکٹری کا حساب لگائیں"""
        positions = np.array([self.splines[j](t) for j in self.splines])

        # رفتار اور ایکسلریشن کے لیے ڈیریویٹوو کا حساب لگائیں
        velocities = np.array([self.splines[j].derivative()(t) for j in self.splines])
        accelerations = np.array([self.splines[j].derivative(2)(t) for j in self.splines])

        return positions, velocities, accelerations
```

## موشن پلیننگ الگورتھم

### 1. ریپڈلی ایکسپلورنگ رینڈم ٹریز (RRT)

RRT ہائی ڈائی مینشل سپیس کے لیے مؤثر ہے اور پیچیدہ کنٹریکٹس کو سنبھال سکتا ہے:

```python
class RRTPlanner:
    def __init__(self, config_space, step_size=0.1):
        self.config_space = config_space
        self.step_size = step_size
        self.nodes = []  # کنفیگریشنز کی فہرست
        self.edges = {}  # والد-بچہ تعلقات

    def plan(self, start_config, goal_config, max_iterations=10000):
        """RRT کا استعمال کرتے ہوئے شروع سے گول تک پاتھ پلین کریں"""
        self.nodes = [start_config]
        self.edges[0] = None  # روت نوڈ کا کوئی والد نہیں

        for i in range(max_iterations):
            # بے ترتیب کنفیگریشن نمونہ
            q_rand = self.sample_configuration()

            # ٹری میں قریب ترین نوڈ تلاش کریں
            nearest_idx = self.nearest_node(q_rand)

            # رینڈم کنفیگریشن کی طرف بڑھائیں
            q_new = self.extend_towards(nearest_idx, q_rand)

            if q_new is not None:
                new_idx = len(self.nodes)
                self.nodes.append(q_new)
                self.edges[new_idx] = nearest_idx

                # چیک کریں کہ کیا گول تک پہنچ گیا
                if self.config_space.distance(q_new, goal_config) < self.step_size:
                    return self.extract_path(new_idx)

        return None  # پاتھ تلاش کرنے میں ناکام

    def sample_configuration(self):
        """جوائنٹ لیمٹس کے اندر بے ترتیب کنفیگریشن نمونہ"""
        q = np.random.uniform(
            low=self.config_space.joint_limits[:, 0],
            high=self.config_space.joint_limits[:, 1]
        )
        return q

    def nearest_node(self, q):
        """ٹری میں قریب ترین نوڈ کا اندیس تلاش کریں"""
        distances = [self.config_space.distance(q, node) for node in self.nodes]
        return np.argmin(distances)

    def extend_towards(self, nearest_idx, q_target):
        """ٹری کو ہدف کنفیگریشن کی طرف بڑھائیں"""
        q_near = self.nodes[nearest_idx]

        # ڈائریکشن ویکٹر
        direction = q_target - q_near
        distance = np.linalg.norm(direction)

        if distance < self.step_size:
            q_new = q_target
        else:
            # ہدف کی ڈائریکشن میں سٹیپ سائز منتقل کریں
            direction = direction / distance
            q_new = q_near + self.step_size * direction

        # چیک کریں کہ کیا نئی کنفیگریشن درست ہے
        if self.config_space.is_valid_configuration(q_new):
            return q_new

        return None

    def extract_path(self, goal_idx):
        """والد کے اشاروں کو فالو کرتے ہوئے گول سے شروع کا راستہ نکالیں"""
        path = []
        current_idx = goal_idx

        while current_idx is not None:
            path.append(self.nodes[current_idx])
            current_idx = self.edges[current_idx]

        return path[::-1]  # الٹ دیں تاکہ شروع سے گول کا راستہ مل جائے
```

### 2. پرو بیلٹک روڈ میپس (PRM)

PRM کنفیگریشن اسپیس کا پیش احتساب شدہ روڈ میپ بناتا ہے:

```python
class PRMPlanner:
    def __init__(self, config_space, n_samples=1000, connection_radius=0.5):
        self.config_space = config_space
        self.n_samples = n_samples
        self.connection_radius = connection_radius
        self.roadmap = {}  # گراف کی نمائندگی

    def build_roadmap(self):
        """کنفیگریشنز کا نمونہ لے کر اور قریب والوں کو جوڑ کر روڈ میپ بنائیں"""
        # بے ترتیب کنفیگریشنز کا نمونہ
        samples = []
        for _ in range(self.n_samples):
            q = self.sample_valid_configuration()
            if q is not None:
                samples.append(q)

        # قریبی کنفیگریشنز کو جوڑیں
        for i, q1 in enumerate(samples):
            for j, q2 in enumerate(samples[i+1:], i+1):
                if (self.config_space.distance(q1, q2) < self.connection_radius and
                    self.is_collision_free_path(q1, q2)):
                    # گراف میں ایج شامل کریں
                    if i not in self.roadmap:
                        self.roadmap[i] = []
                    if j not in self.roadmap:
                        self.roadmap[j] = []

                    self.roadmap[i].append(j)
                    self.roadmap[j].append(i)

    def sample_valid_configuration(self):
        """جیسے تک درست کنفیگریشن مل جائے نمونہ لیں"""
        max_attempts = 100
        for _ in range(max_attempts):
            q = np.random.uniform(
                low=self.config_space.joint_limits[:, 0],
                high=self.config_space.joint_limits[:, 1]
            )
            if self.config_space.is_valid_configuration(q):
                return q
        return None

    def is_collision_free_path(self, q1, q2):
        """چیک کریں کہ کیا q1 اور q2 کے درمیان راستہ کولیژن فری ہے"""
        # سادگی کے لیے، فرض کریں کہ کوئی رکاوٹ نہیں
        # عمل میں، یہ ماحول کے ماڈل کے خلاف چیک کرے گا
        return True
```

## ہیومنوائڈ مخصوص ٹریجکٹری پلیننگ

### 1. وہول بڈی ٹریجکٹری پلیننگ

ہیومنوائڈ روبوٹس کو ایک وقت میں متعدد کاموں کو مطابق کرنا چاہیے:

```python
class WholeBodyTrajectoryPlanner:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.tasks = []  # ترجیح دی گئی کاموں کی فہرست

    def add_balance_task(self, com_trajectory, priority=1):
        """سینٹر آف ماس توازن کا کام شامل کریں"""
        task = {
            'type': 'balance',
            'trajectory': com_trajectory,
            'priority': priority,
            'weight': 1.0
        }
        self.tasks.append(task)

    def add_end_effector_task(self, end_effector, trajectory, priority=2):
        """اینڈ ایفیکٹر پوزیشن کا کام شامل کریں"""
        task = {
            'type': 'end_effector',
            'end_effector': end_effector,
            'trajectory': trajectory,
            'priority': priority,
            'weight': 1.0
        }
        self.tasks.append(task)

    def add_posture_task(self, desired_posture, priority=3):
        """جوائنٹ پوسچر کا کام شامل کریں"""
        task = {
            'type': 'posture',
            'posture': desired_posture,
            'priority': priority,
            'weight': 0.1  # پوسچر کے لیے کم وزن
        }
        self.tasks.append(task)

    def solve_trajectory(self, time_points):
        """ترجیح دی گئی ٹاسک کنٹرول کا استعمال کرتے ہوئے وہول بڈی ٹریجکٹری حل کریں"""
        n_timesteps = len(time_points)
        n_joints = self.robot_model.n_joints

        # جوائنٹ ٹریجکٹری شروع کریں
        joint_trajectory = np.zeros((n_timesteps, n_joints))

        # کاموں کو ترجیح کے مطابق ترتیب دیں
        sorted_tasks = sorted(self.tasks, key=lambda x: x['priority'])

        for t_idx, t in enumerate(time_points):
            current_q = joint_trajectory[t_idx-1] if t_idx > 0 else self.robot_model.q_home

            # ہر کام کے لیے حل کریں
            delta_q = np.zeros(n_joints)

            for task in sorted_tasks:
                task_delta = self.compute_task_delta(task, current_q, t)
                delta_q += task_delta

            joint_trajectory[t_idx] = current_q + delta_q

        return joint_trajectory

    def compute_task_delta(self, task, current_q, time):
        """مخصوص کام کے لیے جوائنٹ اسپیس ڈیلٹا کا حساب لگائیں"""
        if task['type'] == 'balance':
            # CoM جیکوبین اور مطلوبہ CoM رفتار کا حساب لگائیں
            com_jacobian = self.robot_model.get_com_jacobian(current_q)
            desired_com_vel = self.compute_desired_com_velocity(task['trajectory'], time)

            # CoM رفتار کو جوائنٹ رفتار میں میپ کرنے کے لیے پیسیو انورس
            joint_vel = np.linalg.pinv(com_jacobian) @ desired_com_vel
            return joint_vel * 0.01  # فرض کریں 10ms ٹائم سٹیپ

        elif task['type'] == 'end_effector':
            # اینڈ ایفیکٹر کام کے لیے اسی طرح کا نقطہ نظر
            ee_jacobian = self.robot_model.get_jacobian(task['end_effector'], current_q)
            # ... حساب کریں اور ڈیلٹا لوٹائیں
            pass

        elif task['type'] == 'posture':
            # پوسچر کام - مطلوبہ جوائنٹ کنفیگریشن کی طرف جائیں
            posture_error = task['posture'] - current_q
            return task['weight'] * posture_error * 0.01

        return np.zeros(self.robot_model.n_joints)
```

### 2. واکنگ پیٹرن جنریشن

چلنے کی ٹریجکٹریز کو توازن کے لیے خصوصی غور کی ضرورت ہوتی ہے:

```python
class WalkingPatternGenerator:
    def __init__(self, step_length=0.3, step_width=0.2, step_height=0.05, step_period=0.8):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
        self.step_period = step_period

    def generate_walk_trajectory(self, n_steps, start_pos=[0, 0]):
        """n اسٹیپس کے لیے چلنے کی ٹریجکٹری جنریٹ کریں"""
        trajectory = []

        current_x, current_y = start_pos

        for step in range(n_steps):
            # یہ طے کریں کہ کون سا فوٹ سووِنگ فوٹ ہے (متبادل)
            support_foot = 'left' if step % 2 == 0 else 'right'
            swing_foot = 'right' if step % 2 == 0 else 'left'

            # سووِنگ فوٹ ہدف کی پوزیشن کا حساب لگائیں
            target_x = current_x + self.step_length
            target_y = current_y + ((-1)**step) * self.step_width / 2

            # سووِنگ ٹریجکٹری جنریٹ کریں
            swing_trajectory = self.generate_swing_trajectory(
                start_pos=[current_x, current_y + ((-1)**(step+1)) * self.step_width / 2, 0],
                end_pos=[target_x, target_y, 0],
                height=self.step_height
            )

            # کل ٹریجکٹری میں شامل کریں
            for point in swing_trajectory:
                trajectory.append({
                    'time': step * self.step_period + point['time'],
                    'support_foot': support_foot,
                    'swing_foot': swing_foot,
                    'swing_foot_pos': point['position'],
                    'com_x': current_x + point['time'] / self.step_period * self.step_length
                })

            current_x = target_x
            current_y = target_y

        return trajectory

    def generate_swing_trajectory(self, start_pos, end_pos, height, steps=20):
        """فوٹ کے لیے سووِنگ فیز ٹریجکٹری جنریٹ کریں"""
        trajectory = []
        dt = self.step_period / steps

        for i in range(steps):
            t = i * dt
            progress = t / self.step_period

            # ہاریزونٹل انٹرپولیشن
            x = start_pos[0] + progress * (end_pos[0] - start_pos[0])
            y = start_pos[1] + progress * (end_pos[1] - start_pos[1])

            # پیرابولک لفٹ کے ساتھ عمودی ٹریجکٹری
            if progress < 0.5:
                # لفٹ فیز
                z_progress = progress * 2
                z = start_pos[2] + height * np.sin(np.pi * z_progress / 2)**2
            else:
                # لوور فیز
                z_progress = (progress - 0.5) * 2
                z = start_pos[2] + height * np.sin(np.pi * (1 - z_progress) / 2)**2

            trajectory.append({
                'time': t,
                'position': [x, y, z],
                'phase': 'swing'
            })

        return trajectory
```

## آپٹیمائزیشن بیسڈ ٹریجکٹری پلیننگ

### ماڈل پریڈکٹو کنٹرول (MPC) برائے ٹریجکٹریز

MPC محدود افق پر ٹریجکٹریز کو بہتر بناتا ہے:

```python
class ModelPredictiveTrajectoryPlanner:
    def __init__(self, horizon=20, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self.robot_model = None  # بیرونی طور پر سیٹ کیا جانا چاہیے

    def solve_mpc_trajectory(self, current_state, reference_trajectory):
        """آپٹیمل ٹریجکٹری تلاش کرنے کے لیے MPC مسئلہ حل کریں"""
        import cvxpy as cp

        # آپٹیمائزیشن متغیرات کی وضاحت کریں
        # اسٹیٹ متغیرات: ہر ٹائم اسٹیپ کے لیے پوزیشن، رفتار
        X = cp.Variable((self.horizon + 1, 2))  # [x, y] پوزیشن
        U = cp.Variable((self.horizon, 2))     # [vx, vy] رفتار کمانڈز

        # قیمت فنکشن
        cost = 0

        # ٹریکنگ قیمت
        for k in range(self.horizon):
            cost += cp.sum_squares(X[k, :] - reference_trajectory[k])

        # کنٹرول کوشش قیمت
        for k in range(self.horizon):
            cost += 0.1 * cp.sum_squares(U[k, :])

        # ڈائی نامکس کنٹریکٹس (سادہ)
        constraints = []
        constraints.append(X[0, :] == current_state[:2])  # ابتدائی اسٹیٹ

        for k in range(self.horizon):
            # سادہ ڈائی نامکس: x[k+1] = x[k] + dt * u[k]
            constraints.append(X[k+1, :] == X[k, :] + self.dt * U[k, :])

        # رفتار لیمٹس
        for k in range(self.horizon):
            constraints.append(cp.norm(U[k, :], 2) <= 2.0)  # زیادہ سے زیادہ رفتار 2 میٹر/سیکنڈ

        # اسٹیٹ کنٹریکٹس (ورک سپیس لیمٹس)
        for k in range(self.horizon + 1):
            constraints.append(X[k, 0] >= -10)  # x_min
            constraints.append(X[k, 0] <= 10)   # x_max
            constraints.append(X[k, 1] >= -5)   # y_min
            constraints.append(X[k, 1] <= 5)    # y_max

        # آپٹیمائزیشن مسئلہ حل کریں
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve()

        if problem.status not in ["infeasible", "unbounded"]:
            return X.value, U.value
        else:
            return None, None
```

### ٹائم آپٹیمل ٹریجکٹری پلیننگ

کنٹریکٹس کا احترام کرتے ہوئے انجام کے وقت کو کم کرنا:

```python
class TimeOptimalTrajectoryPlanner:
    def __init__(self):
        self.max_velocity = 1.0
        self.max_acceleration = 2.0

    def generate_time_optimal_trajectory(self, start_pos, end_pos, start_vel=0, end_vel=0):
        """شروع اور ختم کے پوائنٹس کے درمیان ٹائم آپٹیمل ٹریجکٹری جنریٹ کریں"""
        distance = end_pos - start_pos

        # ٹائم آپٹیمل پروفائل کا حساب لگائیں
        # فیز 1: ایکسلریشن
        t_acc = min(self.max_velocity / self.max_acceleration,
                   abs(distance) / self.max_velocity)
        d_acc = 0.5 * self.max_acceleration * t_acc**2

        # فیز 2: مستقل رفتار (اگر ضرورت ہو)
        remaining_distance = abs(distance) - 2 * d_acc  # فرض کریں ڈی سیلریشن فیز بھی

        if remaining_distance > 0:
            # ٹریپیزoidal پروفائل
            t_const = remaining_distance / self.max_velocity
            t_dec = t_acc  # متناظر ڈی سیلریشن
        else:
            # ٹرائی اینگولر پروفائل - کوئی مستقل رفتار فیز نہیں
            new_peak_vel = np.sqrt(abs(distance) * self.max_acceleration)
            t_acc = new_peak_vel / self.max_acceleration
            t_const = 0
            t_dec = t_acc

        total_time = t_acc + t_const + t_dec

        # ٹریجکٹری پوائنٹس جنریٹ کریں
        trajectory = []
        dt = 0.01  # 100 Hz
        current_time = 0
        current_pos = start_pos
        current_vel = start_vel

        while current_time <= total_time:
            if current_time <= t_acc:
                # ایکسلریشن فیز
                current_vel = start_vel + self.max_acceleration * current_time
                current_pos = start_pos + 0.5 * self.max_acceleration * current_time**2
            elif current_time <= t_acc + t_const:
                # مستقل رفتار فیز
                current_vel = self.max_velocity
                elapsed = current_time - t_acc
                current_pos = start_pos + d_acc + self.max_velocity * elapsed
            else:
                # ڈی سیلریشن فیز
                elapsed = current_time - t_acc - t_const
                current_vel = self.max_velocity - self.max_acceleration * elapsed
                current_pos = (start_pos + d_acc + self.max_velocity * t_const +
                              self.max_velocity * elapsed -
                              0.5 * self.max_acceleration * elapsed**2)

            trajectory.append({
                'time': current_time,
                'position': current_pos,
                'velocity': current_vel,
                'acceleration': self.get_acceleration(current_time, t_acc, t_const, total_time)
            })

            current_time += dt

        return trajectory

    def get_acceleration(self, t, t_acc, t_const, total_time):
        """وقت t پر ایکسلریشن حاصل کریں"""
        if t <= t_acc:
            return self.max_acceleration
        elif t <= t_acc + t_const:
            return 0
        elif t <= total_time:
            return -self.max_acceleration
        else:
            return 0
```

## ریل ٹائم ٹریجکٹری ایگزیکیوشن

### ٹریجکٹری فالوونگ کنٹرول

```python
class TrajectoryFollower:
    def __init__(self, kp=100, kd=20):
        self.kp = kp  # م_PROPOR_IONAL گین
        self.kd = kd  # ڈیریویٹوو گین
        self.trajectory = None
        self.current_time = 0
        self.time_step = 0.01  # 100 Hz

    def set_trajectory(self, trajectory):
        """ فالو کرنے کے لیے ٹریجکٹری سیٹ کریں"""
        self.trajectory = trajectory
        self.current_time = 0

    def compute_control(self, current_state):
        """ٹریجکٹری فالو کرنے کے لیے کنٹرول کمانڈ کا حساب لگائیں"""
        if self.trajectory is None:
            return np.zeros_like(current_state['position'])

        # موجودہ حوالہ اسٹیٹ تلاش کریں
        ref_state = self.get_reference_state(self.current_time)

        # پوزیشن اور رفتار کی خامیاں کا حساب لگائیں
        pos_error = ref_state['position'] - current_state['position']
        vel_error = ref_state['velocity'] - current_state['velocity']

        # PD کنٹرول
        control = self.kp * pos_error + self.kd * vel_error

        # وقت اپ ڈیٹ کریں
        self.current_time += self.time_step

        return control

    def get_reference_state(self, time):
        """انٹرپولیشن کا استعمال کرتے ہوئے دی گئی وقت پر حوالہ اسٹیٹ حاصل کریں"""
        if len(self.trajectory) < 2:
            return self.trajectory[0] if self.trajectory else {'position': 0, 'velocity': 0}

        # ارد گرد کے ٹریجکٹری پوائنٹس تلاش کریں
        for i in range(len(self.trajectory) - 1):
            if self.trajectory[i]['time'] <= time < self.trajectory[i+1]['time']:
                # پوائنٹس کے درمیان لکیری انٹرپولیشن
                t1, t2 = self.trajectory[i]['time'], self.trajectory[i+1]['time']
                w = (time - t1) / (t2 - t1)

                ref_pos = (1 - w) * self.trajectory[i]['position'] + w * self.trajectory[i+1]['position']
                ref_vel = (1 - w) * self.trajectory[i]['velocity'] + w * self.trajectory[i+1]['velocity']

                return {'position': ref_pos, 'velocity': ref_vel}

        # اگر آخر سے گزر گیا ہو، تو آخری پوائنٹ لوٹائیں
        return self.trajectory[-1]
```

### آن لائن ٹریجکٹری مڈیفکیشن

```python
class OnlineTrajectoryModifier:
    def __init__(self):
        self.original_trajectory = None
        self.modified_trajectory = None
        self.modification_window = 5.0  # سیکنڈز ری پلین کے لیے

    def update_trajectory(self, current_state, obstacle_detected=None):
        """موجودہ اسٹیٹ اور رکاوٹس کے مطابق ٹریجکٹری اپ ڈیٹ کریں"""
        if self.original_trajectory is None:
            return None

        # ٹریجکٹری میں موجودہ پوزیشن تلاش کریں
        current_idx = self.find_current_trajectory_index(current_state['time'])

        if obstacle_detected:
            # رکاوٹ کے گرد نیا راستہ پلین کریں
            new_path = self.plan_around_obstacle(
                current_state['position'],
                self.get_future_waypoints(current_idx)
            )

            # نئے راستے کو باقی ٹریجکٹری کے ساتھ مکس کریں
            modified = self.blend_trajectories(
                current_state['time'],
                new_path,
                self.original_trajectory[current_idx:]
            )

            return modified

        return self.original_trajectory[current_idx:]

    def plan_around_obstacle(self, current_pos, future_waypoints):
        """رکاوٹ کے گرد راستہ پلین کریں"""
        # سادہ رکاوٹ سے بچاؤ
        # عمل میں، یہ مقامی پلیننگ الگورتھم کا استعمال کرے گا
        avoidance_path = []

        # رکاوٹ سے بچنے کے لیے ج横向
        lateral_offset = 0.5  # میٹر
        avoidance_point = [
            current_pos[0] + 0.5,  # تھوڑا آگے بڑھیں
            current_pos[1] + lateral_offset  # طرفین میں جائیں
        ]

        avoidance_path.append(avoidance_point)

        # پھر اصل راستے پر واپس جائیں
        if future_waypoints:
            return_to_path = future_waypoints[0]
            avoidance_path.append(return_to_path)

        return avoidance_path
```

## ٹریجکٹری پلیننگ میں اعلیٰ موضوعات

### لرننگ بیسڈ ٹریجکٹری جنریشن

```python
class LearningBasedTrajectoryPlanner:
    def __init__(self):
        self.trajectory_database = []  # کامیاب ٹریجکٹریز اسٹور کریں
        self.similarity_threshold = 0.1  # ٹریجکٹری بازیافت کے لیے

    def learn_from_demonstration(self, demonstration_trajectory):
        """کامیاب ٹریجکٹری کو ڈیٹا بیس میں شامل کریں"""
        self.trajectory_database.append(demonstration_trajectory)

    def retrieve_similar_trajectory(self, start_state, goal_state):
        """ڈیٹا بیس سے مماثل ٹریجکٹری بازیافت کریں"""
        best_similarity = float('inf')
        best_trajectory = None

        for traj in self.trajectory_database:
            similarity = self.calculate_similarity(
                start_state, goal_state,
                traj['start'], traj['goal']
            )

            if similarity < best_similarity:
                best_similarity = similarity
                best_trajectory = traj

        return best_trajectory if best_similarity < self.similarity_threshold else None

    def calculate_similarity(self, start1, goal1, start2, goal2):
        """دو اسٹیٹ جوڑوں کے درمیان مماثلت کا حساب لگائیں"""
        start_dist = np.linalg.norm(np.array(start1) - np.array(start2))
        goal_dist = np.linalg.norm(np.array(goal1) - np.array(goal2))
        return start_dist + goal_dist
```

### ملٹی موڈل ٹریجکٹری پلیننگ

مختلف قسم کی موشن کے لیے پلیننگ (چلنا، رینگنا، چڑھنا):

```python
class MultiModalTrajectoryPlanner:
    def __init__(self):
        self.motion_modes = {
            'walking': self.plan_walking_trajectory,
            'crawling': self.plan_crawling_trajectory,
            'climbing': self.plan_climbing_trajectory
        }
        self.mode_selector = MotionModeClassifier()

    def plan_for_environment(self, environment_type, start_config, goal_config):
        """مناسب موشن موڈ منتخب کریں اور ٹریجکٹری پلین کریں"""
        # درکار موشن موڈ کلاسیفائی کریں
        mode = self.mode_selector.classify_mode(environment_type, start_config, goal_config)

        # منتخب کردہ موڈ کے لیے ٹریجکٹری پلین کریں
        if mode in self.motion_modes:
            return self.motion_modes[mode](start_config, goal_config)
        else:
            # ڈیفالٹ طور پر چلنے کا
            return self.motion_modes['walking'](start_config, goal_config)

    def plan_walking_trajectory(self, start_config, goal_config):
        """کنفیگریشنز کے درمیان چلنے کی ٹریجکٹری پلین کریں"""
        # واکنگ پیٹرن جنریٹر کا استعمال کریں
        pattern_gen = WalkingPatternGenerator()
        return pattern_gen.generate_walk_trajectory(10)  # 10 اسٹیپس

    def plan_crawling_trajectory(self, start_config, goal_config):
        """رینگنے کی ٹریجکٹری (کم جگہ کے لیے)"""
        # سادہ نفاذ
        # عمل میں، یہ مختلف جوائنٹ پیٹرنز جنریٹ کرے گا
        pass

    def plan_climbing_trajectory(self, start_config, goal_config):
        """چڑھنے کی ٹریجکٹری (سیڑھیوں/پہاڑوں کے لیے)"""
        # مناسب فوٹ سٹیپ پیٹرنز کے ساتھ ٹریجکٹری جنریٹ کریں
        pass
```

## کارکردگی کا جائزہ

### ٹریجکٹری کوالٹی میٹرکس

```python
class TrajectoryEvaluator:
    def __init__(self):
        self.metrics = {}

    def evaluate_trajectory(self, trajectory, robot_model):
        """متعدد میٹرکس کا استعمال کرتے ہوئے ٹریجکٹری کوالٹی کا جائزہ لیں"""
        self.metrics = {}

        # ہمواری
        self.metrics['smoothness'] = self.calculate_smoothness(trajectory)

        # توانائی کی کارآمدگی
        self.metrics['energy'] = self.calculate_energy(trajectory, robot_model)

        # توازن برقرار رکھنا
        self.metrics['balance'] = self.calculate_balance_score(trajectory, robot_model)

        # کام مکمل ہونا
        self.metrics['task_completion'] = self.calculate_task_completion(trajectory)

        # محفوظی
        self.metrics['safety'] = self.calculate_safety_score(trajectory, robot_model)

        return self.metrics

    def calculate_smoothness(self, trajectory):
        """ٹریجکٹری ہمواری کا حساب لگائیں (کم جرک)"""
        total_jerk = 0
        dt = trajectory[1]['time'] - trajectory[0]['time'] if len(trajectory) > 1 else 1.0

        for i in range(2, len(trajectory)):
            acc1 = (trajectory[i-1]['velocity'] - trajectory[i-2]['velocity']) / dt
            acc2 = (trajectory[i]['velocity'] - trajectory[i-1]['velocity']) / dt
            jerk = (acc2 - acc1) / dt
            total_jerk += abs(jerk)

        return 1.0 / (1.0 + total_jerk)  # زیادہ بہتر ہے

    def calculate_energy(self, trajectory, robot_model):
        """ٹریجکٹری کے ساتھ توانائی کی کھپت کا تخمینہ لگائیں"""
        total_energy = 0
        for i in range(1, len(trajectory)):
            # تقریباً توانائی ٹارکس کے مربع کا مجموعہ کے طور پر
            torques = robot_model.compute_inverse_dynamics(
                trajectory[i]['position'],
                trajectory[i]['velocity'],
                trajectory[i]['acceleration']
            )
            energy = np.sum(torques**2) * (trajectory[i]['time'] - trajectory[i-1]['time'])
            total_energy += energy

        return 1.0 / (1.0 + total_energy)  # زیادہ بہتر ہے (کم توانائی استعمال کی گئی)

    def calculate_balance_score(self, trajectory, robot_model):
        """یہ حساب لگائیں کہ کیسے ٹریجکٹری توازن برقرار رکھتی ہے"""
        balance_score = 0
        n_points = len(trajectory)

        for point in trajectory:
            # چیک کریں کہ کیا ZMP سپورٹ پولی گون کے اندر ہے
            zmp = robot_model.compute_zmp(point['position'], point['velocity'], point['acceleration'])
            support_polygon = robot_model.get_support_polygon()

            if self.is_zmp_stable(zmp, support_polygon):
                balance_score += 1

        return balance_score / n_points if n_points > 0 else 0

    def is_zmp_stable(self, zmp, support_polygon):
        """چیک کریں کہ کیا ZMP سپورٹ پولی گون کے اندر ہے"""
        # سادہ پوائنٹ ان پولی گون ٹیسٹ
        # عمل میں، یہ زیادہ جامع ہوگا
        zmp_x, zmp_y = zmp
        min_x = min(p[0] for p in support_polygon)
        max_x = max(p[0] for p in support_polygon)
        min_y = min(p[1] for p in support_polygon)
        max_y = max(p[1] for p in support_polygon)

        return min_x <= zmp_x <= max_x and min_y <= zmp_y <= max_y
```

## خاتمہ

ٹریجکٹری پلیننگ ایک جامع شعبہ ہے جو ریاضی کی آپٹیمائزیشن، کنٹرول تھیوری، اور روبوٹکس کو ملانا ہے تاکہ ہیومنوائڈ روبوٹس پیچیدہ ماحول میں مؤثر طریقے سے حرکت کر سکیں۔ جدید نقطہ نظر متعدد پلیننگ لیئرز کو ایک ساتھ ملاتے ہیں، ہائی لیول پاتھ پلیننگ سے لے کر کم لیول کنٹرول تک، جبکہ بائی پیڈل لوموکوشن اور انسان نما موشن پیٹرنز کی منفرد پابندیوں کو مدنظر رکھتے ہوئے۔

اگلا باب اڈاپٹو کنٹرول حکمت عملیوں کا جائزہ لے گا جو ہیومنوائڈ روبوٹس کو ماحولیاتی فیڈ بیک اور تبدیل ہوتی حالات کے مطابق اپنی حرکات کو ریل ٹائم میں ایڈجسٹ کرنے کی اجازت دیتے ہیں۔

[اگلا: اڈاپٹو کنٹرول →](./adaptive-control)