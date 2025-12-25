---
sidebar_position: 5
---

# جسمانی مصنوعی ذہانت پروجیکٹ: سادہ ہیومنوڈ توازن کنٹرولر

## پروجیکٹ کا جائزہ

اس پروجیکٹ میں، ہم جسمانی مصنوعی ذہانت کے اصولوں کو استعمال کرتے ہوئے ایک سادہ ہیومنوڈ توازن کنٹرولر نافذ کریں گے جو ہم نے سیکھا ہے۔ ہم ایک 2D ہموار بائپیڈ ماڈل بنائیں گے جو خارجی رکاوٹوں کے تحت توازن برقرار رکھ سکے۔ یہ پروجیکٹ جسمانی مصنوعی ذہانت کے نظام میں ڈائنامکس، کنٹرول تھیوری، اور شبیہ سازی کے انضمام کو ظاہر کرتا ہے۔

## سیکھنے کے اہداف

اس پروجیکٹ کو مکمل کر کے، آپ:

- ایک سادہ ہیومنوڈ ڈائنامکس ماڈل نافذ کریں گے
- فیڈ بیک کنٹرول کا استعمال کرتے ہوئے ایک توازن کنٹرولر ڈیزائن کریں گے
- ایک فزکس ماحول میں سسٹم کی شبیہ سازی کریں گے
- اپنے کنٹرولر کے استحکام اور کارکردگی کا تجزیہ کریں گے
- عمل میں جسمانی مصنوعی ذہانت کے چیلنجوں کو سمجھیں گے

## شرائطِ لازمہ

- پائی تھون اور NumPy کی بنیادی سمجھ
- فزکس شبیہ سازی کے تصورات سے واقفیت
- کنٹرول تھیوری کی بنیاد سے سمجھ
- ضروری لائبریریز کے ساتھ ایک پائی تھون ماحول تک رسائی

## ضروری لائبریریز

```bash
pip install numpy scipy matplotlib pygame
```

## سسٹم ڈیزائن

### روبوٹ ماڈل

ہم ایک سادہ کردہ 2D ہموار بائپیڈ بنائیں گے:

- 6 ڈگری آف فریڈم (3 فی لیگ)
- ٹورسو کے لیے پوائنٹ ماس
- لیگز کے لیے جڑیل لنکس
- کوئی باہیں نہیں (سادگی کے لیے)

### کنٹرول آرکیٹیکچر

ہمارا کنٹرول سسٹم شامل کرے گا:

- حالت کی اندراج (CoM مقام، رفتار، سمت)
- توازن کنٹرولر (ZMP-مبنی یا الٹا پینڈولم)
- جوڈ سطح کا کنٹرول (PD کنٹرول)

## نفاذ

### 1. روبوٹ ڈائنامکس ماڈل

```python
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

class SimpleBiped:
    def __init__(self):
        # روبوٹ پیرامیٹر
        self.torso_mass = 50.0  # kg
        self.leg_mass = 10.0   # kg فی لیگ
        self.torso_height = 0.8  # m
        self.leg_length = 0.9    # m
        self.gravity = 9.81      # m/s^2

        # ابتدائی حالت [x, z, theta, x_dot, z_dot, theta_dot]
        self.state = np.array([0.0, self.torso_height, 0.0, 0.0, 0.0, 0.0])

        # سپورٹ پولی گون (سادہ کردہ)
        self.foot_separation = 0.2  # m

    def dynamics(self, state, t, control_input):
        """
        سادہ کردہ الٹا پینڈولم ڈائنامکس کنٹرول ان پٹ کے ساتھ
        state = [x, z, theta, x_dot, z_dot, theta_dot]
        control_input = [مطلوبہ CoM x, مطلوبہ CoM z]
        """
        x, z, theta, x_dot, z_dot, theta_dot = state

        # سادہ کردہ حرکت کے مساوات
        # ایک 2D الٹا پینڈولم کے لیے: x_ddot = g * theta
        x_ddot = self.gravity * np.tan(theta) + control_input[0]
        z_ddot = control_input[1]  # عمودی قوت کنٹرول
        theta_ddot = control_input[2]  # ٹورک کنٹرول

        return [x_dot, z_dot, theta_dot, x_ddot, z_ddot, theta_ddot]

    def update(self, dt, control_input):
        """عددی انٹیگریشن کا استعمال کرتے ہوئے روبوٹ کی حالت کو اپ ڈیٹ کریں"""
        # ڈائنامکس کو انٹیگریٹ کریں
        k1 = np.array(self.dynamics(self.state, 0, control_input))
        self.state = self.state + k1 * dt

        # CoM کو مناسب حدود کے اندر رکھیں
        if abs(self.state[0]) > 1.0:
            self.state[0] = np.sign(self.state[0]) * 1.0

    def get_com_position(self):
        """مرکزِ کثافت کا مقام حاصل کریں"""
        return self.state[0], self.state[1]

    def get_orientation(self):
        """ٹورسو کی سمت حاصل کریں"""
        return self.state[2]

    def is_stable(self):
        """چیک کریں کہ روبوٹ مستحکم ہے (سادہ کردہ)"""
        com_x, com_z = self.get_com_position()

        # چیک کریں کہ CoM سپورٹ پولی گون کے اندر ہے
        left_foot = -self.foot_separation / 2
        right_foot = self.foot_separation / 2

        is_balanced = left_foot <= com_x <= right_foot
        is_upright = abs(self.get_orientation()) < 0.5  # ریڈینس

        return is_balanced and is_upright
```

### 2. توازن کنٹرولر

```python
class BalanceController:
    def __init__(self):
        # کنٹرول گینز
        self.kp_com = 100.0  # CoM مقام کے لیے تناسب کا گین
        self.kd_com = 20.0   # CoM رفتار کے لیے ماخوذ گین
        self.kp_theta = 50.0 # سمت کے لیے تناسب کا گین
        self.kd_theta = 10.0 # زاویہ وار رفتار کے لیے ماخوذ گین

        # مطلوبہ حالتیں
        self.desired_com_x = 0.0
        self.desired_com_z = 0.8
        self.desired_theta = 0.0

    def compute_control(self, robot_state, dt):
        """
        توازن برقرار رکھنے کے لیے کنٹرول کمانڈ کا حساب
        [مطلوبہ CoM x, مطلوبہ CoM z, مطلوبہ ٹورک] واپس کرتا ہے
        """
        # موجودہ حالت نکالیں
        com_x, com_z = robot_state[0], robot_state[1]
        com_x_dot, com_z_dot = robot_state[3], robot_state[4]
        theta = robot_state[2]
        theta_dot = robot_state[5]

        # خطا کا حساب
        com_x_error = self.desired_com_x - com_x
        com_z_error = self.desired_com_z - com_z
        theta_error = self.desired_theta - theta

        # کنٹرول آؤٹ پٹ کا حساب
        com_x_control = self.kp_com * com_x_error - self.kd_com * com_x_dot
        com_z_control = self.kp_com * com_z_error - self.kd_com * com_z_dot
        theta_control = self.kp_theta * theta_error - self.kd_theta * theta_dot

        return [com_x_control, com_z_control, theta_control]

    def set_desired_state(self, com_x, com_z, theta):
        """مطلوبہ CoM مقام اور سمت سیٹ کریں"""
        self.desired_com_x = com_x
        self.desired_com_z = com_z
        self.desired_theta = theta
```

### 3. شبیہ سازی کا ماحول

```python
class SimulationEnvironment:
    def __init__(self):
        self.robot = SimpleBiped()
        self.controller = BalanceController()
        self.time = 0.0
        self.dt = 0.01  # 100 Hz کنٹرول شرح
        self.history = {'time': [], 'com_x': [], 'com_z': [], 'theta': [], 'stable': []}

    def apply_disturbance(self, time, magnitude):
        """مخصوص وقت پر بیرونی رکاوٹ لگائیں"""
        if abs(self.time - time) < 0.1:  # 0.1 سیکنڈ کے لیے لگائیں
            return magnitude
        return 0.0

    def step(self):
        """ایک شبیہ سازی اسٹیپ انجام دیں"""
        # کنٹرول ان پٹ کا حساب
        control_input = self.controller.compute_control(self.robot.state, self.dt)

        # بیرونی رکاوٹ شامل کریں
        disturbance = self.apply_disturbance(self.time, 20.0)  # N
        control_input[0] += disturbance

        # روبوٹ کو اپ ڈیٹ کریں
        self.robot.update(self.dt, control_input)

        # حالت ریکارڈ کریں
        self.history['time'].append(self.time)
        self.history['com_x'].append(self.robot.get_com_position()[0])
        self.history['com_z'].append(self.robot.get_com_position()[1])
        self.history['theta'].append(self.robot.get_orientation())
        self.history['stable'].append(self.robot.is_stable())

        # وقت کو اپ ڈیٹ کریں
        self.time += self.dt

    def run_simulation(self, duration):
        """مخصوص مدت کے لیے شبیہ سازی چلائیں"""
        steps = int(duration / self.dt)

        for _ in range(steps):
            self.step()

            # ناکامی کے لیے چیک کریں
            if not self.robot.is_stable() and self.time > 0.5:
                print(f"روبوٹ {self.time:.2f}s وقت پر گرا")
                break

    def plot_results(self):
        """شبیہ سازی کے نتائج کو پلاٹ کریں"""
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12))

        # CoM X مقام
        ax1.plot(self.history['time'], self.history['com_x'])
        ax1.set_ylabel('CoM X مقام (m)')
        ax1.grid(True)
        ax1.set_title('وقت کے ساتھ مرکزِ کثافت X مقام')

        # CoM Z مقام
        ax2.plot(self.history['time'], self.history['com_z'])
        ax2.set_ylabel('CoM Z مقام (m)')
        ax2.grid(True)
        ax2.set_title('وقت کے ساتھ مرکزِ کثافت Z مقام')

        # سمت
        ax3.plot(self.history['time'], np.degrees(self.history['theta']))
        ax3.set_ylabel('ٹورسو کی سمت (ڈگریوں میں)')
        ax3.set_xlabel('وقت (s)')
        ax3.grid(True)
        ax3.set_title('وقت کے ساتھ ٹورسو کی سمت')

        plt.tight_layout()
        plt.show()

    def print_performance_metrics(self):
        """کارکردگی کے معیار کو پرنٹ کریں"""
        stable_percentage = sum(self.history['stable']) / len(self.history['stable']) * 100
        final_com_x = self.history['com_x'][-1] if self.history['com_x'] else 0
        max_lean = np.max(np.abs(np.degrees(self.history['theta']))) if self.history['theta'] else 0

        print("\n=== کارکردگی کے معیار ===")
        print(f"توازن برقرار رکھنا: {stable_percentage:.1f}%")
        print(f"آخری CoM آفسیٹ: {final_com_x:.3f}m")
        print(f"زیادہ سے زیادہ جھکاؤ کا زاویہ: {max_lean:.2f}°")
        print(f"آخر میں مستحکم: {self.history['stable'][-1] if self.history['stable'] else False}")
```

### 4. مرکزی شبیہ سازی لوپ

```python
def main():
    print("جسمانی مصنوعی ذہانت توازن کنٹرولر شبیہ سازی شروع ہو رہی ہے")
    print("=" * 50)

    # شبیہ سازی کا ماحول بنائیں
    sim = SimulationEnvironment()

    # شبیہ سازی چلائیں
    print("5 سیکنڈ کے لیے شبیہ سازی چل رہی ہے...")
    sim.run_simulation(5.0)

    # نتائج کا تجزیہ کریں
    print("\nشبیہ سازی مکمل ہو گئی!")
    sim.print_performance_metrics()

    # نتائج کو پلاٹ کریں
    sim.plot_results()

    # مختلف رکاوٹوں کے ساتھ جانچ کریں
    print("\n" + "="*50)
    print(" مضبوط رکاوٹ کے ساتھ جانچ جاری ہے...")

    sim2 = SimulationEnvironment()
    sim2.run_simulation(3.0)  # مضبوط رکاوٹ کے لیے چھوٹی مدت

    print("\n مضبوط رکاوٹ کی جانچ مکمل ہو گئی!")
    sim2.print_performance_metrics()
    sim2.plot_results()

if __name__ == "__main__":
    main()
```

## کنٹرولر کا تجزیہ

### استحکام کا تجزیہ

ہمارے توازن کنٹرولر کا استحکام کئی عوامل پر منحصر ہے:

1. **کنٹرول گینز**: مناسب ٹیون کیے گئے PD گینز انتہائی اہم ہیں
2. **نمونہ لینے کی شرح**: زیادہ کنٹرول شرح عام طور پر استحکام میں بہتری لاتی ہے
3. **ماڈل کی درستی**: ہمارا سادہ کردہ ماڈل حقیقت کی کتنی اچھی نمائندگی کرتا ہے
4. **رکاوٹ کی مقدار**: بڑی رکاوٹوں کو مسترد کرنا مشکل ہوتا ہے

### کارکردگی کے معیار

ہم اپنے کنٹرولر کو مندرجہ ذیل کے استعمال سے جانچتے ہیں:

- **توازن برقرار رکھنا**: روبوٹ کے مستحکم رہنے کا فیصد
- **CoM کا انحراف**: CoM کتنا مطلوبہ مقام سے دور چلا جاتا ہے
- **سمت کی غلطی**: عمودی مقام سے انحراف
- ** settling وقت**: رکاوٹوں سے بازیافت کرنے کا وقت

## پروجیکٹ کو بڑھانا

### 1. چلنے کی صلاحیت شامل کریں

```python
class WalkingController(BalanceController):
    def __init__(self):
        super().__init__()
        self.step_phase = 0.0
        self.step_frequency = 1.0  # Hz
        self.step_length = 0.3     # m

    def compute_walking_control(self, robot_state, dt):
        """چلنے کا نمونہ توازن کنٹرول میں شامل کریں"""
        # چلنے کا نمونہ پیدا کریں
        self.step_phase += 2 * np.pi * self.step_frequency * dt
        foot_position = self.step_length * np.sin(self.step_phase)

        # چلنے کے نمونے کے پیچھے CoM کو تبدیل کرنے کے لیے مطلوبہ CoM کو تبدیل کریں
        self.set_desired_state(foot_position, 0.8, 0.0)

        return self.compute_control(robot_state, dt)
```

### 2. اصلاحی کنٹرول

```python
class AdaptiveController(BalanceController):
    def __init__(self):
        super().__init__()
        self.adaptation_rate = 0.01
        self.model_error = 0.0

    def adapt_gains(self, state_error, control_effort):
        """کارکردگی کے مطابق کنٹرول گینز کو ایڈجسٹ کریں"""
        # سادہ گین ایڈاپٹیشن الگورتھم
        self.kp_com += self.adaptation_rate * state_error * control_effort
        self.kp_com = np.clip(self.kp_com, 10, 200)  # گینز کو محدود کریں
```

### 3. مشین لرننگ کا انضمام

```python
class RLBalanceController:
    def __init__(self):
        self.weights = np.random.randn(6)  # سادہ لکیری کنٹرولر
        self.learning_rate = 0.001

    def compute_action(self, state):
        """سیکھے گئے پالیسی کا استعمال کرتے ہوئے کنٹرول ایکشن کا حساب کریں"""
        # سادہ لکیری پالیسی: ایکشن = ویژنز * حالت
        return np.dot(self.weights, state)

    def update_weights(self, state, reward, next_state):
        """پالیسی گریڈیئنٹ کا استعمال کرتے ہوئے ویژنز کو اپ ڈیٹ کریں"""
        # سادہ کردہ پالیسی گریڈیئنٹ اپ ڈیٹ
        grad = state * reward
        self.weights += self.learning_rate * grad
```

## حقیقی دنیا کے خیالات

### سینسر کی آواز اور تاخیر

حقیقی روبوٹس کے پاس ناقص سینسر ہوتے ہیں:

```python
def add_sensor_noise(measurement, noise_std):
    """حقیقی سینسر کی آواز شامل کریں"""
    return measurement + np.random.normal(0, noise_std)

def simulate_sensor_delay(measurement, delay_steps, buffer):
    """سینسر کی تاخیر کی شبیہ سازی کریں"""
    buffer.append(measurement)
    if len(buffer) > delay_steps:
        return buffer.pop(0)
    return measurement
```

### ایکچویٹر کی حدود

حقیقی ایکچویٹر کے قیود ہوتے ہیں:

```python
def apply_actuator_limits(torques, max_torque):
    """ایکچویٹر کی حدود لگائیں"""
    return np.clip(torques, -max_torque, max_torque)

def simulate_motor_dynamics(desired_torque, current_torque, time_constant):
    """پہلی درجہ کے موٹر ڈائنامکس کی شبیہ سازی کریں"""
    tau = time_constant
    dt = 0.01
    return current_torque + (desired_torque - current_torque) * (dt / tau)
```

## عام مسائل کا حل

### 1. عدم استحکام

**علامات**: روبوٹ جھولتا ہے اور گر جاتا ہے
**حل**:
- کنٹرول گینز کو کم کریں
- کنٹرول کی فریکوئنسی بڑھائیں
- ڈیمپنگ ٹرمس شامل کریں
- ماڈل پیرامیٹر چیک کریں

### 2. سست ردعمل

**علامات**: روبوٹ کو رکاوٹوں سے بازیافت میں بہت وقت لگتا ہے
**حل**:
- تناسب کے گینز بڑھائیں
- فیڈ فارورڈ ٹرمس شامل کریں
- سینسر فیوژن کو بہتر کریں
- کمپیوٹیشنل تاخیر چیک کریں

### 3. اوور شوٹنگ

**علامات**: روبوٹ زیادہ درست کرتا ہے اور جھولتا ہے
**حل**:
- ماخوذ گینز بڑھائیں
- اینٹی ونڈ اپ میکنزم شامل کریں
- گین شیڈولنگ نافذ کریں
- زیادہ ترقی یافتہ کنٹرول کے طریقے استعمال کریں

## اعلیٰ موضوعات

### ماڈل پریڈکٹو کنٹرول (MPC)

```python
def mpc_balance_controller(robot_state, prediction_horizon=10):
    """توازن کے لیے ماڈل پریڈکٹو کنٹرول"""
    # مستقبل کی حالت کی پیش گوئی
    future_states = predict_trajectory(robot_state, prediction_horizon)

    # کنٹرول ترتیب کو بہتر بنائیں
    optimal_control = optimize_control_sequence(future_states)

    # پہلا کنٹرول ایکشن واپس کریں
    return optimal_control[0]
```

### مضبوط کنٹرول

```python
def robust_balance_controller(robot_state, uncertainty_bounds):
    """ماڈل کی عدم یقینی کے خلاف مستحکم توازن کنٹرولر"""
    # بدترین منظر کے لیے کنٹرولر ڈیزائن کریں
    robust_gains = design_robust_gains(uncertainty_bounds)

    # مضبوط کنٹرول لا کو لاگو کریں
    return apply_robust_control(robot_state, robust_gains)
```

## خاتمہ

یہ پروجیکٹ سادہ ہیومنوڈ روبوٹ کے لیے ایک توازن کنٹرولر نافذ کر کے جسمانی مصنوعی ذہانت کے بنیادی اصولوں کو ظاہر کرتا ہے۔ ہم نے دیکھا کہ ڈائنامکس، کنٹرول تھیوری، اور شبیہ سازی ایک ذہی جسمانی نظام بنانے کے لیے ایک ساتھ کام کرتے ہیں۔

اہم نکات:
- جسمانی مصنوعی ذہانت کو متعدد مضامین کے انضمام کی ضرورت ہوتی ہے
- محفوظ ترقی کے لیے شبیہ سازی ضروری ہے
- کنٹرول ڈیزائن جسمانی قیود کو مدنظر رکھنا چاہیے
- حقیقی دنیا کے نفاذ میں اضافی چیلنج پیش آتے ہیں

اگلے ماڈیولز یہ بنیادیں استعمال کرتے ہوئے ہیومنوڈ روبوٹکس کے زیادہ پیچیدہ پہلوؤں کو تلاش کریں گے بشمول لوکوموشن کنٹرول، ادراک کے نظام، اور انسان-روبوٹ تعامل۔

[اگلا: ماڈیول 2: ہیومنوڈز کے لیے کنٹرول سسٹم →](../module-2/balance-locomotion)