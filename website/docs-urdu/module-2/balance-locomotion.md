---
sidebar_position: 1
---

# توازن اور لوموکوشن کنٹرول

## بائی پیڈل لوموکوشن کا تعارف

بائی پیڈل لوموکوشن ہیومنوائڈ روبوٹکس میں سب سے چیلنجنگ مسائل میں سے ایک ہے۔ وہیلڈ یا ٹریکڈ روبوٹس کے برعکس، ہیومنوائڈ روبوٹس کو چلتے، دوڑتے یا دیگر ڈائی نامک حرکات انجام دیتے وقت توازن برقرار رکھنا ہوتا ہے۔ اس کے لیے جامع کنٹرول حکمت عملیوں کی ضرورت ہوتی ہے جو بائی پیڈل گیٹ کی ا inherent عدم مستحکم کو سنبھال سکے اور مختلف زمینوں اور رکاوٹوں کے مطابق اپنے آپ کو مطابق کر سکے۔

### بائی پیڈل چلنے کا چیلنج

انسان کا چلنا ایک پیچیدہ ڈائی نامک عمل ہے جس میں شامل ہے:
- مسلسل توازن برقرار رکھنا
- متعدد جوڑوں کی حرکات کو مطابق کرنا
- زمین کی تبدیلیوں کے مطابق اپنے آپ کو ڈھالنا
- توانائی کے موثر حرکت نمونے
- رکاوٹوں کو مسترد کرنا

### لوموکوشن میں کلیدی تصورات

#### زیرو مومینٹ پوائنٹ (ZMP)
ZMP بائی پیڈل روبوٹکس میں ایک اہم تصور ہے، جو ground reaction force کے net moment کے point کی نمائندگی کرتا ہے جہاں صفر ہو۔ مستحکم چلنے کے لیے، ZMP کو feet کے ذریعے متعین کردہ support polygon کے اندر ہی رہنا چاہیے۔

#### کیپچر پوائنٹ
کیپچر پوائنٹ ظاہر کرتا ہے کہ CoM کہاں آرام پر آ جائے گا دی گئی momentum کے تحت۔ یہ توازن بازیابی کی حکمت عملیوں کے لیے اہم ہے۔

#### سپورٹ فیزز
- **ڈبل سپورٹ**: دونوں پاؤں زمین پر
- **سنگل سپورٹ**: ایک پاؤں زمین پر
- **فلائٹ فیز**: کوئی بھی پاؤں زمین پر نہیں (دوڑتے وقت)

## ریاضی کی بنیادیں

### ZMP کا حساب

ZMP کے نقاط کا حساب یوں کیا جاتا ہے:

```
ZMP_x = (Σ(m_i * (ẍ_i * h - z_i * ẍ_i)) / Σ(m_i * g + m_i * z̈_i))
ZMP_y = (Σ(m_i * (ÿ_i * h - z_i * ÿ_i)) / Σ(m_i * g + m_i * z̈_i))
```

جہاں:
- h: CoM کی زمین کے اوپر اونچائی
- g: گریوی ٹیشنل ایکسلریشن
- m_i: لنک i کا ماس
- ẍ_i, ÿ_i: لنک i کی ایکسلریشن
- z_i: زمین کے اوپر لنک i کی اونچائی

### لینیئر انورٹڈ پینڈولم ماڈل (LIPM)

LIPM بائی پیڈل ڈائی نامکس کو سادہ بناتا ہے point mass کو ایک massless leg کے ساتھ سپورٹ کیا گیا:

```
ẍ = g/h * (x - x_zmp)
```

جہاں:
- x: CoM کی پوزیشن
- x_zmp: ZMP کی پوزیشن
- h: CoM کی اونچائی
- g: گریوی ٹیشنل ایکسلریشن

## کنٹرول کی حکمت عملیاں

### 1. ZMP-مبنی کنٹرول

ZMP-مبنی کنٹرول ایک مطلوبہ ZMP ٹریجکٹری کو ٹریک کر کے استحکام برقرار رکھتا ہے:

```python
class ZMPController:
    def __init__(self):
        self.kp = 100.0  # م_PROPOR_IONAL گین
        self.kd = 20.0   # ڈیریویٹوو گین
        self.com_height = 0.8  # میٹر

    def compute_control(self, current_zmp, desired_zmp, current_com, current_com_vel):
        # ZMP خامی
        zmp_error = desired_zmp - current_zmp

        # CoM ایکسلریشن کمانڈ (انورٹڈ پینڈولم ماڈل)
        com_acc_desired = (self.gravity / self.com_height) * (current_com - desired_zmp)

        # فیڈ بیک کنٹرول
        com_acc_feedback = self.kp * zmp_error + self.kd * current_com_vel

        return com_acc_desired + com_acc_feedback
```

### 2. کیپچر پوائنٹ کنٹرول

کیپچر پوائنٹ کنٹرول CoM کو ایک محفوظ پوزیشن پر لانے پر توجہ مرکوز کرتا ہے:

```python
class CapturePointController:
    def __init__(self):
        self.com_height = 0.8
        self.gravity = 9.81

    def compute_capture_point(self, com_pos, com_vel):
        """حساب لگائیں کہ CoM کہاں آرام پر آ جائے گا"""
        omega = np.sqrt(self.gravity / self.com_height)
        return com_pos + com_vel / omega

    def is_capturable(self, capture_point, support_polygon):
        """چیک کریں کہ کیا کیپچر پوائنٹ سپورٹ پولی گون کے اندر ہے"""
        cp_x, cp_y = capture_point
        min_x, max_x, min_y, max_y = support_polygon

        return (min_x <= cp_x <= max_x) and (min_y <= cp_y <= max_y)

    def compute_safe_footstep(self, current_com, current_com_vel, support_foot):
        """محفوظ فوٹ سٹیپ لوکیشن کا حساب لگائیں"""
        capture_point = self.compute_capture_point(current_com, current_com_vel)

        # یقینی بنائیں کہ کیپچر پوائنٹ محفوظ علاقے کے اندر ہے
        safe_x = np.clip(capture_point[0], support_foot[0] - 0.1, support_foot[0] + 0.3)
        safe_y = np.clip(capture_point[1], support_foot[1] - 0.2, support_foot[1] + 0.2)

        return [safe_x, safe_y]
```

### 3. وہول بڈی کنٹرول

وہول بڈی کنٹرول متعدد کاموں کو ایک ساتھ مطابق کرتا ہے:

```python
class WholeBodyController:
    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.tasks = []

    def add_balance_task(self, desired_com_pos, weight=1.0):
        """سینٹر آف ماس توازن کا کام شامل کریں"""
        task = {
            'type': 'com',
            'desired': desired_com_pos,
            'weight': weight
        }
        self.tasks.append(task)

    def add_foot_task(self, foot_name, desired_pos, weight=1.0):
        """فوٹ پلیسمنٹ کا کام شامل کریں"""
        task = {
            'type': 'foot',
            'name': foot_name,
            'desired': desired_pos,
            'weight': weight
        }
        self.tasks.append(task)

    def compute_joint_commands(self, current_state):
        """کوائفریٹک پروگرامنگ کا استعمال کر کے جوائنٹ کمانڈز کا حساب لگائیں"""
        # QP مسئلہ کو فارمولیٹ کریں
        # مینیمائز: ||Ax - b||^2
        # م subject to: Cx <= d

        # یہ عام طور پر CVXOPT جیسے QP حل کار کا استعمال کرے گا
        pass
```

## واکنگ پیٹرن جنریشن

### 1. پری ویو کنٹرول

پری ویو کنٹرول مستقبل کے حوالہ ٹریجکٹریز کو استعمال کر کے ٹریکنگ میں بہتری لاتا ہے:

```python
class PreviewController:
    def __init__(self, preview_steps=20, dt=0.01):
        self.preview_steps = preview_steps
        self.dt = dt
        self.gravity = 9.81
        self.com_height = 0.8

        # پری کمپیوٹ پری ویو گینز
        self.compute_preview_gains()

    def compute_preview_gains(self):
        """پری ویو کنٹرول کے لیے گینز کا حساب لگائیں"""
        omega = np.sqrt(self.gravity / self.com_height)

        # اسٹیٹ اسپیس نمائندگی
        A = np.array([[0, 1], [omega**2, 0]])
        B = np.array([0, -omega**2])

        # ڈسکریٹ ٹائم سسٹم
        dt = self.dt
        self.Ad = np.array([[1, np.sinh(omega*dt)/omega],
                           [omega*np.sinh(omega*dt), np.cosh(omega*dt)]])
        self.Bd = np.array([dt + np.sinh(omega*dt)/omega - np.cosh(omega*dt)/omega,
                           omega*(np.cosh(omega*dt) - 1)])

    def compute_control(self, current_com, future_reference):
        """مستقبل کے حوالہ کے ساتھ کنٹرول کا حساب لگائیں"""
        # پری ویو کنٹرول الگورتھم کا نفاذ
        pass
```

### 2. فوٹ سٹیپ پلاننگ

فوٹ سٹیپ پلاننگ یہ طے کرتا ہے کہ کہاں اور کب فوٹس پلیس کریں:

```python
class FootstepPlanner:
    def __init__(self):
        self.step_length = 0.3  # میٹر
        self.step_width = 0.2   # میٹر
        self.step_height = 0.1  # میٹر (سووِنگ فیز کے لیے)

    def plan_footsteps(self, start_pos, goal_pos, terrain_map=None):
        """گوئل تک پہنچنے کے لیے فوٹ سٹیپس کی ترتیب منصوبہ بند کریں"""
        footsteps = []

        # ضروری اسٹیپس کی تعداد کا حساب لگائیں
        distance = np.linalg.norm(np.array(goal_pos) - np.array(start_pos))
        num_steps = int(distance / self.step_length) + 1

        # اسٹیپ پوزیشنز جنریٹ کریں
        for i in range(num_steps):
            # متبادل پاؤں
            foot_offset = (-1)**i * self.step_width / 2

            # پوزیشن کو انٹرپولیٹ کریں
            t = (i + 1) / (num_steps + 1)
            step_x = start_pos[0] + t * (goal_pos[0] - start_pos[0])
            step_y = start_pos[1] + foot_offset

            footsteps.append([step_x, step_y, 0.0])  # x, y, z

        return footsteps

    def generate_swing_trajectory(self, start_pos, end_pos, height=0.1):
        """فوٹ کے لیے ہموار سووِنگ ٹریجکٹری جنریٹ کریں"""
        # 5ویں آرڈر پولینومیل ٹریجکٹری
        duration = 0.8  # سیکنڈ
        t = np.linspace(0, duration, int(duration/0.01))

        # ہموار ٹریجکٹری کے لیے پولینومیل کوائف
        c0 = start_pos
        c1 = 0  # شروع میں صفر رفتار
        c2 = 0  # شروع میں صفر ایکسلریشن
        c3 = 0  # ختم میں صفر رفتار
        c4 = 0  # ختم میں صفر ایکسلریشن
        c5 = 0

        # 5ویں آرڈر پولینومیل کے لیے کوائف کا حساب لگائیں
        tf = duration
        c5 = 6 * (end_pos - start_pos) / tf**5
        c4 = -15 * (end_pos - start_pos) / tf**4
        c3 = 10 * (end_pos - start_pos) / tf**3

        trajectory = []
        for ti in t:
            pos = c0 + c1*ti + c2*ti**2 + c3*ti**3 + c4*ti**4 + c5*ti**5
            trajectory.append(pos)

        return trajectory
```

## اعلیٰ کنٹرول کی تکنیکیں

### 1. ماڈل پریڈکٹو کنٹرول (MPC)

MPC مستقبل کے رویے کو سسٹم کے ماڈل کی بنیاد پر محفوظ کرتا ہے:

```python
class ModelPredictiveController:
    def __init__(self, horizon=10, dt=0.1):
        self.horizon = horizon
        self.dt = dt
        self.gravity = 9.81
        self.com_height = 0.8

    def solve_mpc(self, current_state, reference_trajectory):
        """MPC کے اختیاراتی مسئلہ کو حل کریں"""
        # اختیاراتی متغیرات کی وضاحت کریں
        # مینیمائز: sum of (state_error^2 + control_effort^2)
        # م subject to: system dynamics, constraints

        # یہ عام طور پر ایک اختیاراتی حل کار کا استعمال کرے گا
        # جیسے CasADi یا ACADO

        # scipy.optimize کا استعمال کر کے سادہ نقطہ نظر
        from scipy.optimize import minimize

        def cost_function(control_sequence):
            total_cost = 0
            state = current_state.copy()

            for i in range(self.horizon):
                # کنٹرول لگائیں اور آگے سیمولیٹ کریں
                state = self.predict_next_state(state, control_sequence[i])

                # سٹیٹ ڈویئیشن کے لیے لاگت شامل کریں
                state_error = state - reference_trajectory[i]
                total_cost += np.sum(state_error**2)

                # کنٹرول کوشش کی لاگت شامل کریں
                total_cost += 0.1 * np.sum(control_sequence[i]**2)

            return total_cost

        # کنٹرول سیکوئنس کے لیے ابتدائی تصور
        initial_control = np.zeros((self.horizon, 2))  # 2D کنٹرول

        # اختیاراتی حل
        result = minimize(cost_function, initial_control.flatten())

        # پہلا کنٹرول ایکشن لوٹائیں
        optimal_control = result.x[:2]
        return optimal_control

    def predict_next_state(self, state, control_input):
        """انورٹڈ پینڈولم ماڈل کا استعمال کر کے اگلا سٹیٹ توقع کریں"""
        com_x, com_y, com_x_dot, com_y_dot = state
        zmp_x, zmp_y = control_input

        # انورٹڈ پینڈولم ڈائی نامکس
        com_x_ddot = self.gravity / self.com_height * (com_x - zmp_x)
        com_y_ddot = self.gravity / self.com_height * (com_y - zmp_y)

        # ڈائی نامکس کو ان ٹیگریٹ کریں
        dt = self.dt
        new_com_x_dot = com_x_dot + com_x_ddot * dt
        new_com_y_dot = com_y_dot + com_y_ddot * dt
        new_com_x = com_x + new_com_x_dot * dt
        new_com_y = com_y + new_com_y_dot * dt

        return np.array([new_com_x, new_com_y, new_com_x_dot, new_com_y_dot])
```

### 2. لوموکوشن کے لیے ری انفورسمنٹ لرننگ

RL جامع لوموکوشن پیٹرنز سیکھ سکتا ہے:

```python
class RLLocomotionController:
    def __init__(self):
        # نیورل نیٹ ورک پالیسی
        self.policy_network = self.build_policy_network()
        self.value_network = self.build_value_network()

    def build_policy_network(self):
        """پالیسی کے لیے نیورل نیٹ ورک بنائیں"""
        import tensorflow as tf

        model = tf.keras.Sequential([
            tf.keras.layers.Dense(256, activation='tanh', input_shape=(20,)),  # سٹیٹ ان پٹ
            tf.keras.layers.Dense(256, activation='tanh'),
            tf.keras.layers.Dense(128, activation='tanh'),
            tf.keras.layers.Dense(12, activation='tanh')  # جوائنٹ کمانڈز
        ])

        return model

    def compute_action(self, state):
        """نیورل نیٹ ورک پالیسی سے ایکشن حاصل کریں"""
        state_tensor = tf.convert_to_tensor(state.reshape(1, -1), dtype=tf.float32)
        action = self.policy_network(state_tensor)
        return action.numpy().flatten()

    def compute_reward(self, robot_state, action):
        """موجودہ سٹیٹ اور ایکشن کے لیے انعام کا حساب لگائیں"""
        # فارورڈ رفتار انعام
        forward_vel = robot_state[6]  # فرض کریں 7ویں عنصر فارورڈ رفتار ہے
        reward = forward_vel * 2.0

        # توازن انعام
        torso_angle = robot_state[3]  # فرض کریں 4تھ عنصر ٹورسو اینگل ہے
        balance_reward = np.exp(-abs(torso_angle) * 10)
        reward += balance_reward

        # زیادہ جوائنٹ ٹورکس کے لیے جرمانہ
        torque_penalty = -np.sum(np.abs(action)) * 0.01
        reward += torque_penalty

        # گرنے کے لیے جرمانہ
        if abs(torso_angle) > 0.5:  # ریڈ
            reward -= 100

        return reward
```

## استحکام کا تجزیہ

### لیاپونوف استحکام

کسی سسٹم کے مستحکم ہونے کے لیے، ہمیں ایک لیاپونوف فنکشن V(x) کی ضرورت ہے جیسے کہ:
- V(x) > 0 for all x ≠ 0
- V̇(x) ≤ 0 for all x

```python
def lyapunov_function(com_state, desired_state):
    """توازن کنٹرول کے لیے لیاپونوف فنکشن کی وضاحت کریں"""
    com_pos, com_vel = com_state
    desired_pos, desired_vel = desired_state

    # پوزیشن خامی
    pos_error = com_pos - desired_pos

    # رفتار خامی
    vel_error = com_vel - desired_vel

    # لیاپونوف فنکشن: V = 0.5 * (pos_error^2 + vel_error^2)
    V = 0.5 * (pos_error**2 + vel_error**2)

    return V

def lyapunov_derivative(com_state, com_dynamics, desired_state, desired_dynamics):
    """لیاپونوف ڈیریویٹو کا حساب لگائیں"""
    com_pos, com_vel = com_state
    desired_pos, desired_vel = desired_state

    # خامی ڈائی نامکس
    pos_error = com_pos - desired_pos
    vel_error = com_vel - desired_vel

    pos_error_dot = com_vel - desired_vel
    vel_error_dot = com_dynamics - desired_dynamics

    # V̇ = pos_error * pos_error_dot + vel_error * vel_error_dot
    V_dot = pos_error * pos_error_dot + vel_error * vel_error_dot

    return V_dot
```

### استحکام مارجنز

```python
class StabilityAnalyzer:
    def __init__(self):
        self.stability_thresholds = {
            'com_deviation': 0.1,      # max CoM deviation (میٹر)
            'torso_angle': 0.3,        # max torso angle (ریڈ)
            'angular_velocity': 0.5,   # max angular velocity (ریڈ/سیکنڈ)
            'zmp_margin': 0.05         # min ZMP margin (میٹر)
        }

    def evaluate_stability(self, robot_state):
        """موجودہ استحکام مارجنز کا جائزہ لیں"""
        metrics = {}

        # سینٹر آف ماس کا انحراف
        com_deviation = abs(robot_state['com_x'] - robot_state['zmp_x'])
        metrics['com_stability'] = com_deviation < self.stability_thresholds['com_deviation']
        metrics['com_margin'] = self.stability_thresholds['com_deviation'] - com_deviation

        # ٹورسو اینگل
        torso_angle = abs(robot_state['torso_angle'])
        metrics['angle_stability'] = torso_angle < self.stability_thresholds['torso_angle']
        metrics['angle_margin'] = self.stability_thresholds['torso_angle'] - torso_angle

        # ZMP مارجن
        support_polygon = robot_state['support_polygon']
        zmp_pos = (robot_state['zmp_x'], robot_state['zmp_y'])
        zmp_margin = self.calculate_zmp_margin(zmp_pos, support_polygon)
        metrics['zmp_stability'] = zmp_margin > self.stability_thresholds['zmp_margin']
        metrics['zmp_margin'] = zmp_margin

        return metrics

    def calculate_zmp_margin(self, zmp_pos, support_polygon):
        """ZMP سے پولی گون کے کنارے تک کم از کم فاصلہ کا حساب لگائیں"""
        zmp_x, zmp_y = zmp_pos
        min_distance = float('inf')

        # سپورٹ پولی گون کے ہر کنارے تک فاصلہ کا حساب لگائیں
        for i in range(len(support_polygon)):
            p1 = support_polygon[i]
            p2 = support_polygon[(i + 1) % len(support_polygon)]

            # پوائنٹ سے لائن سیگمینٹ تک فاصلہ
            distance = self.point_to_line_distance(zmp_pos, p1, p2)
            min_distance = min(min_distance, distance)

        return min_distance
```

## عملی نفاذ کے مسائل

### توازن کے لیے سینسر فیوژن

```python
class BalanceSensorFusion:
    def __init__(self):
        # کیلمین فلٹر کے پیرامیٹر
        self.process_noise = 0.1
        self.measurement_noise = 0.01
        self.estimate_error = 1.0
        self.estimate = 0.0

    def kalman_update(self, measurement):
        """کیلمین فلٹر کا استعمال کر کے سٹیٹ اندازہ اپ ڈیٹ کریں"""
        # پریڈکشن اسٹیپ
        prediction = self.estimate
        pred_error = self.estimate_error + self.process_noise

        # اپ ڈیٹ اسٹیپ
        kalman_gain = pred_error / (pred_error + self.measurement_noise)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * pred_error

        return self.estimate

    def estimate_com_position(self, imu_data, force_sensors, vision_data):
        """متعدد سینسرز کا استعمال کر کے CoM پوزیشن کا تخمینہ لگائیں"""
        # IMU اورینٹیشن اور اینگولر رفتار فراہم کرتا ہے
        orientation = self.integrate_gyro(imu_data['gyro'])

        # فورس سینسرز ZMP معلومات فراہم کرتے ہیں
        zmp_from_forces = self.calculate_zmp_from_forces(force_sensors)

        # وژن ایبسولیوٹ پوزیشن حوالہ فراہم کرتا ہے
        absolute_pos = vision_data['position'] if vision_data else None

        # تمام اندازے فیوژن کریں
        fused_estimate = self.fuse_sensor_data(orientation, zmp_from_forces, absolute_pos)

        return fused_estimate
```

### ریل ٹائم کنٹرول نفاذ

```python
import threading
import time

class RealTimeBalanceController:
    def __init__(self):
        self.control_frequency = 1000  # Hz
        self.dt = 1.0 / self.control_frequency
        self.running = False
        self.control_thread = None

        # کنٹرولرز کو شروع کریں
        self.zmp_controller = ZMPController()
        self.footstep_planner = FootstepPlanner()

    def start_control_loop(self):
        """الگ تھریڈ میں ریل ٹائم کنٹرول لوپ شروع کریں"""
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()

    def control_loop(self):
        """ریل ٹائم کنٹرول لوپ"""
        last_time = time.time()

        while self.running:
            current_time = time.time()
            elapsed = current_time - last_time

            if elapsed >= self.dt:
                # سینسر ڈیٹا پڑھیں
                sensor_data = self.read_sensors()

                # سٹیٹ اندازہ اپ ڈیٹ کریں
                robot_state = self.estimate_state(sensor_data)

                # کنٹرول کمانڈز کا حساب لگائیں
                control_commands = self.compute_control(robot_state)

                # ایکٹو ایٹرز کو کمانڈز بھیجیں
                self.send_commands(control_commands)

                last_time = current_time
            else:
                # ٹائم کو برقرار رکھنے کے لیے سلیپ کریں
                sleep_time = self.dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def stop_control_loop(self):
        """ریل ٹائم کنٹرول بند کریں"""
        self.running = False
        if self.control_thread:
            self.control_thread.join()
```

## اعلیٰ لوموکوشن پیٹرنز

### واکنگ گیٹس

```python
class WalkingGaitGenerator:
    def __init__(self):
        self.gait_params = {
            'walk': {'step_length': 0.3, 'step_height': 0.05, 'period': 0.8},
            'trot': {'step_length': 0.4, 'step_height': 0.08, 'period': 0.6},
            'pace': {'step_length': 0.2, 'step_height': 0.03, 'period': 1.0}
        }

    def generate_walk_pattern(self, gait_type='walk', duration=10.0):
        """مخصوص گیٹ کے لیے چلنے کا پیٹرن جنریٹ کریں"""
        params = self.gait_params[gait_type]

        # اسٹیپ ٹائمنگ جنریٹ کریں
        step_timing = np.arange(0, duration, params['period'])

        # فوٹ ٹریجکٹریز جنریٹ کریں
        trajectories = []
        for t in step_timing:
            # گیٹ فیز کے مطابق فوٹ پوزیشن کا حساب لگائیں
            phase = (t % params['period']) / params['period']

            # سووِنگ ٹریجکٹری جنریٹ کریں
            if phase < 0.5:  # سووِنگ فیز
                swing_progress = phase * 2
                foot_x = params['step_length'] * swing_progress
                foot_z = params['step_height'] * np.sin(np.pi * swing_progress)
            else:  # سٹینس فیز
                stance_progress = (phase - 0.5) * 2
                foot_x = params['step_length'] * (0.5 + 0.5 * stance_progress)
                foot_z = 0.0

            trajectories.append({
                'time': t,
                'position': [foot_x, 0, foot_z],
                'phase': 'swing' if phase < 0.5 else 'stance'
            })

        return trajectories
```

## خاتمہ

توازن اور لوموکوشن کنٹرول ہیومنوائڈ روبوٹکس کے سب سے چیلنجنگ پہلوؤں میں سے ایک کی نمائندگی کرتا ہے۔ کامیابی کے لیے ڈائی نامکس ماڈلنگ، کنٹرول تھیوری، اور ریل ٹائم نفاذ کا دیکھ بھال کی ضرورت ہوتی ہے۔ اس سیکشن میں شامل کردہ طریقوں - کلاسیکل ZMP کنٹرول سے لے کر جدید ری انفورسمنٹ لرننگ تک - مستحکم، موثر چلنے والے روبوٹس کو تیار کرنے کے لیے ایک بنیاد فراہم کرتے ہیں۔

اگلا سیکشن موٹر کنٹرول اور ایکٹو ایشن سسٹم کو ایکسپلور کرے گا جو ان جامع کنٹرول حکمت عملیوں کو فعال کرتے ہیں۔

[اگلا: موٹر کنٹرول →](./motor-control)