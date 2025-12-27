---
id: module-2-chapter-6
title: "Sim-to-Real Transfer Techniques"
sidebar_label: "2.6 Sim-to-Real Transfer"
sidebar_position: 6
description: "Bridge the reality gap: domain randomization, system identification, progressive fidelity, and validation strategies for deploying simulated algorithms on physical robots"
keywords: [sim-to-real, domain-randomization, system-identification, reality-gap, transfer-learning]
estimated_time: 75
prerequisites:
  - module-2-chapter-1
  - module-2-chapter-3
learning_outcomes:
  - Understand the causes of the reality gap
  - Apply domain randomization techniques
  - Perform system identification for accurate models
  - Design progressive fidelity training curricula
  - Validate sim-to-real transfer success metrics
hardware_tier: premium
---

# باب 2.6: سِم-ٹو-ریئل ٹرانسفر ٹیکنیکس

حقیقت کا فرق—سِمولیشن اور فیزیکل حقیقت کے درمیان فرق—روبوٹکس کی مرکزی چیلنجز میں سے ایک ہے۔ یہ باب اس فرق کو پورا کرنے اور سِمولیٹڈ الگورتھم کو حقیقی روبوٹس پر کامیابی سے تعینات کرنے کے طریقے پیش کرتا ہے۔

## حقیقت کے فرق کو سمجھنا

حقیقت کا فرق سِمولیشن اور حقیقی دنیا کے برتاؤ کے درمیان اختلافات کو ظاہر کرتا ہے جو الگورتھم کو فیزیکل روبوٹس پر تعینات کرنے پر ناکام بناتے ہیں۔

### حقیقت کے فرق کے ذرائع

**فیزکس کا عدم مطابقت**:
- آسان رابطہ ماڈلز
- غیر درست رگڑ کے ضریب
- غیر کامل ایکچویٹر ڈائنامکس
- غیر ماڈل شدہ لچک اور مطابقت

**سینسر کے فرق**:
- شور کی خصوصیات
- کیلیبریشن کی غلطیاں
- ماحولیاتی اثرات (روشنی، عکاسی)
- ٹائمنگ اور سنکرونائزیشن کے مسائل

**ماحولیاتی عوامل**:
- غیر متوقع اشیاء کی خصوصیات
- ہوا کا مزاحمت اور ہوا
- درجہ حرارت کا مواد پر اثر
- سطح کی مختلف حالتیں

### حقیقت کے فرق کی پیمائش

سِم-ٹو-ریئل ٹرانسفر کی کامیابی کی پیمائش کریں:

```python
import numpy as np

def reality_gap_metric(sim_performance, real_performance):
    """
    Calculate percentage performance drop from sim to real
    """
    gap = (sim_performance - real_performance) / sim_performance * 100
    return gap

# Example: Object grasping task
sim_success_rate = 0.95  # 95% success in simulation
real_success_rate = 0.72  # 72% success on real robot

gap = reality_gap_metric(sim_success_rate, real_success_rate)
print(f"Reality gap: {gap:.1f}% performance drop")
# Output: Reality gap: 24.2% performance drop
```

## ڈومین رینڈمائزیشن

ڈومین رینڈمائزیشن مختلف سِمولیشن پیرامیٹرز پر تربیت دے کر پالیسی کی مضبوطی کو بڑھاتا ہے۔

### ویژول ڈومین رینڈمائزیشن

ظاہری شکل کو رینڈمائز کریں تا کہ ویژن بیسڈ پالیسیز کو بہتر بنایا جا سکے:

```python
import random

class VisualRandomizer:
    def __init__(self):
        self.light_range = (100, 1500)  # lux
        self.color_temp_range = (2500, 7000)  # Kelvin
        self.texture_library = load_texture_library()

    def randomize_lighting(self):
        intensity = random.uniform(*self.light_range)
        color_temp = random.uniform(*self.color_temp_range)
        direction = random_unit_vector()
        return {
            'intensity': intensity,
            'color_temp': color_temp,
            'direction': direction
        }

    def randomize_textures(self, objects):
        for obj in objects:
            obj.set_texture(random.choice(self.texture_library))

    def randomize_camera(self):
        return {
            'exposure': random.uniform(0.5, 2.0),
            'gain': random.uniform(0.8, 1.5),
            'blur': random.uniform(0, 0.02)
        }
```

### فیزکس ڈومین رینڈمائزیشن

تربیت کے دوران فیزیکل پیرامیٹرز کو مختلف کریں:

```python
class PhysicsRandomizer:
    def randomize_object_properties(self, obj):
        """Randomize mass, friction, restitution"""
        mass_scale = random.uniform(0.8, 1.2)
        obj.set_mass(obj.base_mass * mass_scale)

        friction = random.uniform(0.3, 0.9)
        obj.set_friction(friction)

        restitution = random.uniform(0.0, 0.3)  # bounciness
        obj.set_restitution(restitution)

    def randomize_robot_dynamics(self, robot):
        """Add actuator noise and delays"""
        for joint in robot.joints:
            # Torque noise
            torque_noise = random.uniform(0.95, 1.05)
            joint.torque_scale = torque_noise

            # Control delay (1-10ms)
            delay = random.uniform(0.001, 0.010)
            joint.control_delay = delay

            # Backlash
            backlash = random.uniform(0, 0.02)  # radians
            joint.backlash = backlash

    def randomize_ground(self):
        """Vary ground properties"""
        return {
            'friction': random.uniform(0.5, 1.0),
            'compliance': random.uniform(0, 0.01),
            'slope': random.uniform(-2, 2)  # degrees
        }
```

### کریکولم ڈومین رینڈمائزیشن

رینڈمائزیشن کی مشکل کو تدریجاً بڑھائیں:

```python
class CurriculumRandomizer:
    def __init__(self):
        self.training_phase = 0
        self.phases = [
            {'name': 'easy', 'randomization': 0.1},
            {'name': 'medium', 'randomization': 0.5},
            {'name': 'hard', 'randomization': 1.0}
        ]

    def get_randomization_bounds(self, base_value, param_name):
        """Scale randomization range by curriculum phase"""
        phase = self.phases[self.training_phase]
        scale = phase['randomization']

        variation = base_value * 0.2 * scale  # up to 20% variation
        return (base_value - variation, base_value + variation)

    def advance_phase(self, success_rate):
        """Move to harder randomization if doing well"""
        if success_rate > 0.85 and self.training_phase < len(self.phases) - 1:
            self.training_phase += 1
            print(f"Advanced to phase: {self.phases[self.training_phase]['name']}")
```

## سسٹم شناخت

حقیقی روبوٹ کے پیرامیٹرز کی پیمائش کریں تا کہ سِمولیشن کی درستی کو بہتر بنایا جا سکے۔

### ماس اور انرشیا شناخت

```python
import numpy as np
from scipy.optimize import least_squares

class InertiaIdentifier:
    def __init__(self, robot):
        self.robot = robot

    def collect_motion_data(self, trajectory):
        """Record joint positions, velocities, torques"""
        data = {
            'time': [],
            'positions': [],
            'velocities': [],
            'torques': []
        }

        self.robot.execute_trajectory(trajectory)
        # Record data during execution
        return data

    def estimate_inertia(self, data):
        """Fit inertia parameters to motion data"""
        def residual(params):
            # params: [mass, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
            predicted_torques = self.inverse_dynamics(
                data['positions'],
                data['velocities'],
                params
            )
            return data['torques'] - predicted_torques

        # Initial guess from CAD model
        initial_params = self.get_cad_parameters()

        # Optimize to fit measured data
        result = least_squares(residual, initial_params)
        return result.x

    def inverse_dynamics(self, q, qd, params):
        """Compute expected torques given inertia parameters"""
        # Implementation depends on robot kinematics
        pass
```

### رگڑ کی شناخت

```python
class FrictionIdentifier:
    def identify_joint_friction(self, joint):
        """Measure Coulomb and viscous friction"""
        velocities = []
        torques = []

        # Slow velocity ramp
        for vel_cmd in np.linspace(-2, 2, 100):  # rad/s
            joint.set_velocity(vel_cmd)
            time.sleep(0.1)

            actual_vel = joint.get_velocity()
            torque = joint.get_torque()

            velocities.append(actual_vel)
            torques.append(torque)

        # Fit friction model: tau = tau_c*sign(v) + b*v
        velocities = np.array(velocities)
        torques = np.array(torques)

        # Separate positive and negative velocities
        pos_mask = velocities > 0.1
        neg_mask = velocities < -0.1

        coulomb_pos = np.mean(torques[pos_mask])
        coulomb_neg = abs(np.mean(torques[neg_mask]))
        coulomb = (coulomb_pos + coulomb_neg) / 2

        # Viscous friction (slope)
        viscous = np.polyfit(velocities, torques, 1)[0]

        return {'coulomb': coulomb, 'viscous': viscous}
```

## سینسر کیلیبریشن اور ماڈلنگ

### کیمرہ کیلیبریشن

```python
import cv2

class CameraCalibrator:
    def calibrate_intrinsics(self, checkerboard_images):
        """Calibrate camera intrinsic parameters"""
        pattern_size = (9, 6)  # checkerboard corners
        square_size = 0.025  # 25mm squares

        obj_points = []  # 3D points
        img_points = []  # 2D image points

        # Prepare object points
        objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2)
        objp *= square_size

        for image in checkerboard_images:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern_size)

            if ret:
                obj_points.append(objp)
                img_points.append(corners)

        # Calibrate
        ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, gray.shape[::-1], None, None
        )

        return {
            'camera_matrix': K,
            'distortion': dist
        }

    def update_simulation(self, sim_camera, calibration):
        """Apply calibration to simulated camera"""
        K = calibration['camera_matrix']

        sim_camera.set_focal_length(K[0,0], K[1,1])
        sim_camera.set_principal_point(K[0,2], K[1,2])
        sim_camera.set_distortion(calibration['distortion'])
```

### IMU شور کی خصوصیات

```python
class IMUCalibrator:
    def characterize_noise(self, static_data, duration=600):
        """Measure IMU noise while stationary"""
        accel_data = static_data['accelerometer']  # [N, 3]
        gyro_data = static_data['gyroscope']  # [N, 3]

        # Bias: mean while stationary
        accel_bias = np.mean(accel_data, axis=0)
        gyro_bias = np.mean(gyro_data, axis=0)

        # Noise: standard deviation
        accel_noise = np.std(accel_data, axis=0)
        gyro_noise = np.std(gyro_data, axis=0)

        # Allan variance for random walk
        allan_var_accel = self.compute_allan_variance(accel_data)
        allan_var_gyro = self.compute_allan_variance(gyro_data)

        return {
            'accel_bias': accel_bias,
            'gyro_bias': gyro_bias,
            'accel_noise_density': accel_noise,
            'gyro_noise_density': gyro_noise,
            'accel_random_walk': allan_var_accel,
            'gyro_random_walk': allan_var_gyro
        }

    def apply_to_simulation(self, sim_imu, calibration):
        """Configure simulated IMU with measured noise"""
        sim_imu.set_accelerometer_bias(calibration['accel_bias'])
        sim_imu.set_gyroscope_bias(calibration['gyro_bias'])
        sim_imu.set_accelerometer_noise(calibration['accel_noise_density'])
        sim_imu.set_gyroscope_noise(calibration['gyro_noise_density'])
```

## تدریجی فیڈیلٹی تربیت

تدریجاً بڑھتی ہوئی حقیقت پسندی کے ساتھ پالیسیز کو تربیت دیں۔

### فیڈیلٹی لیولز

```python
class ProgressiveFidelityTrainer:
    def __init__(self):
        self.fidelity_stages = [
            {
                'name': 'kinematic',
                'physics': False,
                'sensor_noise': False,
                'randomization': 0.0
            },
            {
                'name': 'basic_physics',
                'physics': True,
                'sensor_noise': False,
                'randomization': 0.2
            },
            {
                'name': 'realistic',
                'physics': True,
                'sensor_noise': True,
                'randomization': 0.8
            }
        ]

    def train_stage(self, stage_config, policy, episodes):
        """Train policy at specific fidelity level"""
        self.configure_simulation(stage_config)

        for episode in range(episodes):
            # Train with current fidelity
            observation = self.env.reset()
            done = False

            while not done:
                action = policy.get_action(observation)
                observation, reward, done, info = self.env.step(action)
                policy.update(observation, reward)

    def should_advance(self, success_rate, threshold=0.85):
        """Check if ready for higher fidelity"""
        return success_rate > threshold
```

## تصدیقی حکمت عملیاں

### سِم-ٹو-ریئل پرفارمنس میٹرکس

```python
class TransferValidator:
    def __init__(self):
        self.metrics = []

    def evaluate_transfer(self, policy, sim_env, real_env, trials=100):
        """Compare policy performance in sim vs real"""
        sim_results = self.evaluate_policy(policy, sim_env, trials)
        real_results = self.evaluate_policy(policy, real_env, trials)

        metrics = {
            'sim_success_rate': sim_results['success_rate'],
            'real_success_rate': real_results['success_rate'],
            'transfer_gap': sim_results['success_rate'] - real_results['success_rate'],
            'sim_avg_reward': sim_results['avg_reward'],
            'real_avg_reward': real_results['avg_reward']
        }

        return metrics

    def evaluate_policy(self, policy, env, trials):
        """Run policy for multiple trials"""
        successes = 0
        total_reward = 0

        for trial in range(trials):
            obs = env.reset()
            done = False
            episode_reward = 0

            while not done:
                action = policy.get_action(obs)
                obs, reward, done, info = env.step(action)
                episode_reward += reward

            successes += info['success']
            total_reward += episode_reward

        return {
            'success_rate': successes / trials,
            'avg_reward': total_reward / trials
        }
```

## عملی مشق: سِم-ٹو-ریئل ٹرانسفر

گریسپنگ ٹاسک کے لیے ڈومین رینڈمائزیشن کو نافذ کریں:

1. **بنیادی**: فکسڈ سِمولیشن میں گریسپنگ پالیسی کو تربیت دیں
2. **ویژول رینڈمائزیشن شامل کریں**: روشنی اور ٹیکسچرز کو رینڈمائز کریں
3. **فیزکس رینڈمائزیشن شامل کریں**: آبجیکٹ ماس اور رگڑ کو مختلف کریں
4. **سسٹم شناخت**: حقیقی روبوٹ کی رگڑ کی پیمائش کریں
5. **تصدیق**: حقیقی روبوٹ پر پالیسی کا ٹیسٹ کریں، کامیابی کی شرح کی پیمائش کریں

**تصدیق**:
- [ ] بنیادی پالیسی سِمولیشن میں >80% کامیابی حاصل کرتی ہے
- [ ] رینڈمائزڈ پالیسی سِمولیشن میں >70% کامیابی برقرار رکھتی ہے
- [ ] حقیقی روبوٹ کی کامیابی کی شرح >60% (بمقابلہ <40% غیر رینڈمائزڈ کے لیے)
- [ ] حقیقت کا فرق کم از کم 30% کم ہو گیا ہے

## اعلیٰ ٹیکنیکس

### ایڈورسیریل ڈومین رینڈمائزیشن

چیلنجنگ سینیریوز پیدا کرنے کے لیے ایک ڈسکرمنیٹر کو تربیت دیں:

```python
import torch
import torch.nn as nn

class AdversarialRandomizer:
    def __init__(self):
        self.discriminator = nn.Sequential(
            nn.Linear(state_dim, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid()
        )

    def generate_parameters(self):
        """Discriminator suggests hard randomization"""
        # Sample parameters that make task harder
        pass

    def update_discriminator(self, success):
        """Learn from policy performance"""
        # Reward discriminator for creating challenging scenarios
        pass
```

### ریزیڈوئل پالیسی لرننگ

حقیقی روبوٹ پر تصحیح پالیسی سیکھیں:

```python
class ResidualPolicy:
    def __init__(self, sim_policy):
        self.sim_policy = sim_policy  # pre-trained in simulation
        self.residual_policy = SmallNetwork()  # learns corrections

    def get_action(self, observation):
        # Base action from simulation policy
        base_action = self.sim_policy(observation)

        # Correction from residual policy
        correction = self.residual_policy(observation)

        # Combined action
        return base_action + correction
```

## کیس اسٹڈی: مینیپولیشن ٹرانسفر

اشیاء کو پکڑنے کے لیے سِم-ٹو-ریئل ٹرانسفر کا حقیقی دنیا کا مثال:

**سِمولیشن تربیت**:
- گیزیبو میں ڈومین رینڈمائزیشن کے ساتھ 1M ایپی سوڈز
- 200+ آبجیکٹ ماڈلز مختلف خصوصیات کے ساتھ
- کامیابی کی شرح: 94%

**حقیقت کے فرق کو کم کرنا**:
- کیمرہ انٹرنسک کیلیبریشن
- گریپر رگڑ کی پیمائش
- ٹیکٹائل سینسر شور کی خصوصیات

**حقیقی روبوٹ کے نتائج**:
- کامیابی کی شرح: 78% (50 آبجیکٹس، ہر ایک کے لیے 20 ٹرائلز)
- حقیقت کا فرق: 16% (رینڈمائزیشن کے بغیر 45% سے کم)
- ٹرانسفر کا وقت: 2 دن بمقابلہ 6 ماہ صرف حقیقی دنیا کی تربیت کے لیے

## ٹرانسفر فیلیئرز کی خرابی دور کرنا

| علامت | ممکنہ وجہ | حل |
|---------|--------------|----------|
| سِم میں کام کرتا ہے، حقیقی میں فوری طور پر ناکام ہو جاتا ہے | غیر ماڈل شدہ ڈائنامکس | فیزکس رینڈمائزیشن، سسٹم شناخت شامل کریں |
| ویژن بیسڈ پالیسی ناکام ہو جاتی ہے | روشنی/ظاہری شکل کا عدم مطابقت | ویژول ڈومین رینڈمائزیشن |
| کبھی کبھار حقیقی دنیا میں ناکامی | ناکافی ایکسپلوریشن | رینڈمائزیشن کی تنوع کو بڑھائیں |
| سِم پرفارمنس خراب ہو جاتی ہے | زیادہ رینڈمائزیشن | کریکولم لرننگ استعمال کریں |

## اگلا باب

[باب 2.7: ملٹی سینسر فیوژن (اگر موجود ہو)، یا ماڈیول 3](../module-3-isaac/chapter-3-1.md) میں، آپ NVIDIA Isaac Sim کے بارے میں سیکھیں گے جو فوٹوریلسٹک سِمولیشن اور تیز رفتار پرسپشن کے لیے ہے۔