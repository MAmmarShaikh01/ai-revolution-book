---
sidebar_position: 4
---

# شبیہ سازی کے ماحول

## جسمانی مصنوعی ذہانت میں شبیہ سازی کا کردار

شبیہ سازی کے ماحول جسمانی مصنوعی ذہانت اور ہیومنوڈ روبوٹکس کے نظام کی ترقی میں اہم کردار ادا کرتے ہیں۔ وہ الگورتھم کی جانچ، کنٹرولرز کی تربیت، اور حقیقی ہارڈ ویئر پر عملدرآمد سے پہلے ڈیزائن کی توثیق کے لیے محفوظ، قیمت مؤثر، اور قابل کنٹرول پلیٹ فارم فراہم کرتے ہیں۔ ہیومنوڈ روبوٹس کے لیے، شبیہ سازی خاص طور پر اہم ہے کیونکہ جسمانی پلیٹ فارم کی پیچیدگی اور قیمت کی وجہ سے۔

### شبیہ سازی کے فوائد

- **حفاظت**: خطرناک منظار کی جانچ کریں بغیر ہارڈ ویئر یا انسانوں کے لیے خطرہ کے
- **قیمت مؤثر**: ہارڈ ویئر کی قیمت اور ترقی کے وقت کو کم کریں
- **دوبارہ تیار کرنا**: ماحولیاتی حالات کو بالکل کنٹرول کریں
- **رفتار**: متوازی شبیہ سازی کے ذریعے تربیت کو تیز کریں
- **ڈیبگنگ**: ہارڈ ویئر پر قابل پیمائش نہ ہونے والی اندرونی حالت اور قوتوں تک رسائی حاصل کریں
- **.scalability**: ایک وقت میں متعدد ماحول میں تربیت کریں

### شبیہ سازی کے چیلنج

- **حقیقت کا فرق**: شبیہ سازی شدہ اور حقیقی ماحول کے درمیان فرق
- **معلوماتی لاگت**: زیادہ معیار کی شبیہ سازی کو نمایاں وسائل کی ضرورت ہوتی ہے
- **ماڈل کی درستی**: حقیقی دنیا کی فزکس کی نامکمل ماڈلنگ
- **سینسر شبیہ سازی**: حقیقی سینسر کے رویے کی قریب ترین نقل

## اہم شبیہ سازی پلیٹ فارم

### NVIDIA اسیک سِم

اسیک سِم NVIDIA کا روبوٹکس کے لیے جامع شبیہ سازی کا ماحول ہے، جو اومنی ورس پلیٹ فارم پر تعمیر کیا گیا ہے۔

#### خصوصیات

- **فوٹو ریلائزٹک رینڈرنگ**: زیادہ معیار کی بصری شبیہ سازی
- **فِزکس فِزکس انجن**: درست فزکس شبیہ سازی
- **ROS/ROS 2 انضمام**: روبوٹکس فریم ورکس کے ساتھ بے داغ انضمام
- **AI تربیت کی حمایت**: مضبوط لرننگ کے لیے ان بورڈ ٹولز
- **قابلِ توسیع کمپیوٹنگ**: کلاؤڈ ڈیپلائمنٹ کی صلاحیتیں

#### استعمال کے معاملات

- ادراک کے نظام کی تربیت
- نیویگیشن اور راستہ منصوبہ بندی
- مینیولیشن مہارت کی تعلیم
- انسان-روبوٹ تعامل کے منظار

#### مثال سیٹ اپ

```python
import omni
from omni.isaac.kit import SimulationApp

# اسیک سِم لانچ کریں
config = {"headless": False}
simulation_app = SimulationApp(config)

from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage

# دنیا بنائیں
world = World(stage_units_in_meters=1.0)

# ہیومنوڈ روبوٹ شامل کریں
asset_path = get_assets_root_path() + "/NVIDIA/Assets/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Robot")

# ری سیٹ اور اسٹیپ
world.reset()
for i in range(1000):
    world.step(render=True)

simulation_app.close()
```

### گیزبو اور اگنیشن

گیزبو روبوٹکس شبیہ سازی میں کئی سالوں سے ایک اہم جزو رہا ہے، اگنیشن گیزبو اس کی اگلی نسل کی نمائندگی کرتا ہے۔

#### خصوصیات

- **فزکس انجن**: ODE، بُلیٹ، سِم بอดی، اور DART کی حمایت
- **سینسر شبیہ سازی**: کیمرے، لیڈار، IMU، اور مزید
- **پلگ ان آرکیٹیکچر**: قابلِ توسیع فعلیت
- **ROS انضمام**: نیٹیو ROS/ROS 2 حمایت
- **متعدد روبوٹ شبیہ سازی**: ایک وقت میں متعدد روبوٹس کی شبیہ سازی

#### مثال دنیا کی فائل

```xml
<sdf version="1.6">
  <world name="humanoid_world">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <!-- ہیومنوڈ روبوٹ شامل کریں -->
    <include>
      <uri>model://humanoid_robot</uri>
    </include>
  </world>
</sdf>
```

### MuJoCo (مُلٹی جوئنٹ ڈائنامکس وِد کنٹیکٹ)

MuJoCo اپنی زبردست فزکس شبیہ سازی کے لیے جانا جاتا ہے اور روبوٹکس اور بائیو مکینکس کی تحقیق میں خاص طور پر مقبول ہے۔

#### خصوصیات

- **تیز فزکس**: زبردست کارکردگی کا شبیہ سازی انجن
- **درست کنٹیکٹ**: ترقی یافتہ کنٹیکٹ ماڈلنگ
- **بہترین کنٹرول**: ٹریجکٹری اصلاح کے لیے ان بورڈ ٹولز
- **بائیو مکینکس**: انسان اور جانوروں کی شبیہ سازی کے لیے مخصوص

#### مثال

```python
import mujoco
import mujoco.viewer

# ماڈل لوڈ کریں
model = mujoco.MjModel.from_xml_path('humanoid_model.xml')
data = mujoco.MjData(model)

# شبیہ سازی لوپ
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # یہاں کنٹرول لاگک
        # کنٹرول پالیسی کے مطابق ٹورک لگائیں

        # شبیہ سازی اسٹیپ
        mujoco.mj_step(model, data)

        # ویویر کو اپ ڈیٹ کریں
        viewer.sync()

        # حقیقی وقت کی شرح برقرار رکھیں
        time_until_next_step = step_start + model.opt.timestep - time.time()
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
```

### PyBullet

PyBullet فزکس شبیہ سازی فراہم کرتا ہے جس کا مرکز قابل رسائی اور مضبوط لرننگ پر ہے۔

#### خصوصیات

- **پائی تھون انٹرفیس**: براہ راست پائی تھون API
- **مضبوط لرننگ**: RL فریم ورکس کے ساتھ انضمام
- **کولیژن ڈیٹیکشن**: تیز کولیژن ڈیٹیکشن
- **نرم بڑی شبیہ سازی**: قابلِ تشکیل اشیاء کی حمایت

#### مثال

```python
import pybullet as p
import pybullet_data

# فزکس سرور سے رابطہ قائم کریں
physicsClient = p.connect(p.GUI)  # یا p.DIRECT غیر گرافیکل ورژن کے لیے

# گریویٹی سیٹ کریں
p.setGravity(0, 0, -9.81)

# پلین اور روبوٹ لوڈ کریں
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeId = p.loadURDF("plane.urdf")
robotStartPos = [0, 0, 1]
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("humanoid.urdf", robotStartPos, robotStartOrientation)

# شبیہ سازی لوپ
for i in range(10000):
    p.stepSimulation()

    # روبوٹ کی حالت حاصل کریں
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotId)

    # کنٹرول لگائیں
    # joint_commands = compute_control(robotPos, robotOrn)
    # p.setJointMotorControlArray(robotId, joint_indices, p.TORQUE_CONTROL, forces=joint_commands)

p.disconnect()
```

## فزکس شبیہ سازی کی بنیادیں

### کولیژن ڈیٹیکشن

شبیہ سازی کے ماحول کو مؤثر طریقے سے یہ پتہ لگانا چاہیے کہ اشیاء رابطے میں کب آتی ہیں:

- **براؤڈ فیز**: غیر-ٹکراؤ والے جوڑوں کو جلدی ختم کرنا
- **نیرو فیز**: بالکل رابطہ پوائنٹ کا حساب
- **مسلسل کولیژن ڈیٹیکشن**: زیادہ رفتار پر ٹنلنگ کو روکنا

### کنٹیکٹ ماڈلنگ

ہیومنوڈ شبیہ سازی کے لیے درست کنٹیکٹ ماڈلز انتہائی اہم ہیں:

#### پینلٹی میتھڈس

- پینیٹریشن گہرائی کے متناسب قوتیں لگائیں
- کمپیوٹیشنل طور پر کارآمد لیکن غیر مستحکم ہو سکتی ہے

#### قیود پر مبنی میتھڈس

- رابطوں کو قیود کے طور پر بیان کریں
- زیادہ مستحکم لیکن کمپیوٹیشنل طور پر مہنگا

### انٹیگریشن اسکیم

مختلف عددی انٹیگریشن کے طریقے شبیہ سازی کی درستی کو متاثر کرتے ہیں:

#### اولر انٹیگریشن

- سادہ لیکن عددی طور پر غیر مستحکم
- بنیادی شبیہ سازی کے لیے مناسب

#### رنگ کوٹا میتھڈس

- زیادہ درست لیکن کمپیوٹیشنل طور پر مہنگا
- زیادہ معیار کی شبیہ سازی کے لیے بہتر

#### سِمپلکٹک انٹیگریٹرز

- محفوظ نظاموں میں توانائی کو محفوظ رکھیں
- طویل مدتی استحکام کے لیے اہم

## سینسر شبیہ سازی

### بصری سینسر

#### کیمرہ شبیہ سازی

```python
import numpy as np
import omni
from omni.isaac.sensor import Camera

# کیمرہ بنائیں
camera = Camera(
    prim_path="/World/Camera",
    position=np.array([1.0, 1.0, 1.0]),
    orientation=np.array([0.0, 0.0, 0.0, 1.0])
)

# RGB ڈیٹا حاصل کریں
rgb_data = camera.get_rgb()
```

#### لیڈار شبیہ سازی

```python
from omni.isaac.range_sensor import _range_sensor

lidar_interface = _range_sensor.acquire_lidar_sensor_interface()
lidar_data = lidar_interface.get_linear_depth_data("/World/Lidar")
```

### اندرونی سینسر

#### IMU شبیہ سازی

```python
# شبیہ سازی شدہ IMU پڑھائیاں
def simulate_imu(robot_state, noise_params):
    # لکیری تیزی
    linear_acc = robot_state.linear_acceleration + \
                 np.random.normal(0, noise_params['acc_std'])

    # زاویہ وار رفتار
    angular_vel = robot_state.angular_velocity + \
                  np.random.normal(0, noise_params['gyro_std'])

    # سمت (زاویہ وار رفتار سے انٹیگریٹ کی گئی)
    orientation = integrate_gyro(angular_vel)

    return {
        'linear_acceleration': linear_acc,
        'angular_velocity': angular_vel,
        'orientation': orientation
    }
```

## ڈومین رینڈمائزیشن

ڈومین رینڈمائزیشن حقیقت کا فرق کو کم کرنے کی ایک تکنیک ہے جو متنوع شبیہ سازی شدہ ماحول میں تربیت کر کے حاصل کی جاتی ہے۔

### رینڈمائزیشن پیرامیٹر

```python
# ڈومین رینڈمائزیشن کی مثال
domain_params = {
    'mass': uniform(0.8, 1.2),  # ماس کو رینڈمائز کریں
    'friction': uniform(0.5, 1.5),  # اصطکال کو رینڈمائز کریں
    'restitution': uniform(0.0, 0.5),  # باؤنسی نیس کو رینڈمائز کریں
    'gravity': normal(9.81, 0.1),  # گریویٹی کو رینڈمائز کریں
    'visual': {
        'lighting': random_lighting_conditions(),
        'textures': random_textures(),
        'camera_noise': random_camera_noise()
    }
}
```

### نفاذ کی حکمت عملی

1. ** critical پیرامیٹر کی شناخت**: تعین کریں کہ کون سے پیرامیٹر کارکردگی کو سب سے زیادہ متاثر کرتے ہیں
2. **رینڈمائزیشن کی حدود کی تعریف**: رینڈمائزیشن کے لیے حقیقت پسندانہ حدود مقرر کریں
3. **درجہ وار تربیت**: تنگ حدود کے ساتھ شروع کریں، وقت کے ساتھ وسعت کریں
4. **توثیق**: ہارڈ ویئر پر منتقلی کو یقینی بنانے کے لیے جانچ کریں

## شبیہ سازی میں مضبوط لرننگ

### تربیت کے ماحول

```python
import gym
from gym import spaces
import numpy as np

class HumanoidEnv(gym.Env):
    def __init__(self):
        # ایکشن اور مشاہدہ کی جگہوں کی تعریف کریں
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(30,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(60,), dtype=np.float32
        )

    def reset(self):
        # شبیہ سازی کو ابتدائی حالت میں ری سیٹ کریں
        self.sim.reset()
        return self._get_observation()

    def step(self, action):
        # شبیہ سازی پر ایکشن لگائیں
        self.sim.apply_action(action)
        self.sim.step()

        # مشاہدہ، انعام، ختم، معلومات حاصل کریں
        obs = self._get_observation()
        reward = self._compute_reward()
        done = self._check_termination()
        info = {}

        return obs, reward, done, info

    def _get_observation(self):
        # حالت کا مشاہدہ واپس کریں
        pass

    def _compute_reward(self):
        # کام کی بنیاد پر انعام کا حساب کریں
        pass

    def _check_termination(self):
        # چیک کریں کہ ایپی سود ختم ہونا چاہیے
        pass
```

### متوازی شبیہ سازی

متعدد متوازی ماحول کے ساتھ تربیت:

```python
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3 import PPO

# متعدد ماحول بنائیں
def make_env():
    def _init():
        return HumanoidEnv()
    return _init

env = SubprocVecEnv([make_env() for _ in range(16)])  # 16 متوازی ماحول

# ایجنٹ کو تربیت دیں
model = PPO('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=1000000)
```

## شبیہ سازی سے حقیقت کی منتقلی

### حقیقت کا فرق کو کم کرنے کی تکنیکیں

#### سسٹم کی شناخت

- حقیقی روبوٹ کے مطابق شبیہ سازی کے پیرامیٹر کیلیبریٹ کریں
- حقیقی پیرامیٹر کا اندازہ لگانے کے لیے سسٹم کی شناخت کے طریقے استعمال کریں

#### اصلاحی کنٹرول

- ماڈلنگ کی غلطیوں کے مطابق ایڈجسٹ ہونے والے کنٹرولرز نافذ کریں
- کنٹرول پیرامیٹر کو ایڈجسٹ کرنے کے لیے آن لائن لرننگ استعمال کریں

#### مضبوط کنٹرول

- ان پیرامیٹر کی تبدیلیوں کے مطابق کام کرنے والے کنٹرولرز ڈیزائن کریں
- H-infinity یا دیگر مضبوط کنٹرول کے طریقے استعمال کریں

### توثیق کی حکمت عملی

1. **ہارڈ ویئر ان دی لوپ**: حقیقی سینسر/ایکچویٹر کو شبیہ سازی سے منسلک کریں
2. **پیش رفت کی منتقلی**: شبیہ سازی کی پیچیدگی کو تدریج بڑھائیں
3. **کراس توثیق**: متعدد حقیقی دنیا کے منظار میں جانچ کریں
4. **حفاظت کی نگرانی**: منتقلی کے دوران حفاظتی چیکس نافذ کریں

## بہترین طریقے

### ماڈل کی درستی

- حقیقی ہارڈ ویئر کے مقابل شبیہ سازی کے ماڈل کی توثیق کریں
- اہم اجزاء کے لیے زیادہ معیار کے ماڈل استعمال کریں
- کارکردگی کو بہتر بنانے کے لیے غیر اہم پہلوؤں کو سادہ کریں

### کمپیوٹیشنل کارآمدی

- مناسب شبیہ سازی ٹائم اسٹیپس استعمال کریں
- منظر کی پیچیدگی کو بہتر بنائیں
- جہاں ممکن ہو GPU تیزی کا فائدہ اٹھائیں

### دوبارہ تیار کرنا

- مسلسل نتائج کے لیے بے ترتیب سیڈ فکس کریں
- شبیہ سازی کے پیرامیٹر دستاویز کریں
- شبیہ سازی کے اثاثوں کے لیے ورژن کنٹرول

### جانچ کی حکمت عملی

- ہارڈ ویئر ڈیپلائمنٹ سے پہلے شبیہ سازی میں جانچ کریں
- متعدد شبیہ سازی کے ماحول استعمال کریں
- باقاعدگی سے ہارڈ ویئر پر توثیق کریں

## مستقبل کی سمتیں

### ڈیجیٹل ٹوئنز

- شبیہ سازی اور حقیقت کے درمیان حقیقی وقت کا ہم وقت
- جاری ماڈل کی ترقی
- پیش گوئی کی مرمت اور بہتری

### کلاؤڈ بیسڈ شبیہ سازی

- قابلِ توسیع کمپیوٹنگ وسائل
- مشترکہ ترقی
- جاری تربیت کی بنیاد

### متعدد فزکس شبیہ سازی

- متعدد جسمانی ڈومین کا انضمام
- الیکٹرومیگنیٹک، حرارتی، اور سیال کے تعاملات
- زیادہ جامع سسٹم ماڈلنگ

## خاتمہ

شبیہ سازی کے ماحول جسمانی مصنوعی ذہانت اور ہیومنوڈ روبوٹکس کے نظام کی ترقی کے لیے ناگزیر ٹولز ہیں۔ وہ پیچیدہ الگورتھم کی محفوظ، قیمت مؤثر ترقی اور جانچ کو فعال کرتے ہیں۔ جیسے جیسے شبیہ سازی کی تکنالوجی ترقی کرتی رہتی ہے، شبیہ سازی شدہ اور حقیقی ماحول کے درمیان فرق کم ہوتا جا رہا ہے، جو روبوٹکس کی ترقی کے لیے شبیہ سازی کو ایک بڑھتی ہوئی قیمتی ٹول بناتا ہے۔

اگلے حصے میں ہم ان بنیادی تصورات کو عملی پروجیکٹس میں کیسے لاگو کریں گے اس کا جائزہ لیں گے۔

[اگلا: جسمانی مصنوعی ذہانت پروجیکٹ →](./physical-ai-project)