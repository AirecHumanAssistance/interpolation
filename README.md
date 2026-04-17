# Interpolator Node

低周波で受け取る関節目標値を高周波制御周期へ補間し、上腕の Position 制御と下腕の Impedance 制御へ分割して送信する ROS ノードです。

必要に応じて使用するコントローラや使用する関節名は変えてほしいです。

---

## 概要

このノードは、`/joint_predictions` から受け取る `sensor_msgs/JointState` の関節目標値をもとに、制御周期ごとに線形補間を行い、ロボットへ滑らかな関節コマンドを送信します。

入力される 14 次元の関節値は、以下の 2 系統に分割されます。

- **Position制御**
  - 左腕 `joint_1 ~ joint_6`
  - 右腕 `joint_1 ~ joint_6`
- **Impedance制御**
  - 左腕 `joint_7`
  - 右腕 `joint_7`

深層学習モデルなどが低周波で出力する関節予測値を、そのままロボットへ送ると動きが不連続になることがあります。このノードを間に入れることで、低周波の目標値を高周波制御向けに補間し、より滑らかな動作を実現できます。

---
## 起動方法

補間ノードは以下のコマンドで起動します。

```bash
roslaunch interpolation bringup_interpolation.launch
```

## 主な機能

- `/joint_predictions` から関節目標値を受信
- 前回コマンドと新しい目標値の間を線形補間
- 14 次元の関節値を Position 用と Impedance 用に分割
- 2 つの `trajectory_msgs/JointTrajectory` トピックへ publish
- 急激な関節目標値の変化を検出し、安全のためコマンドを無効化

---

## ノード構成

### Subscribe

#### `/joint_predictions`
- 型: `sensor_msgs/JointState`

深層学習モデルなどから予測された関節目標値を受け取ります。  
このノードでは `msg.position` を使用し、14 要素の配列であることを前提としています。

---

### Publish

#### `/torobo/online_joint_upperarm_trajectory_controller/command`
- 型: `trajectory_msgs/JointTrajectory`
- 用途: upper arm の Position 制御

#### `/torobo/online_joint_lowerarm_impedance_controller/command`
- 型: `trajectory_msgs/JointTrajectory`
- 用途: lower arm の Impedance 制御

---

## 補間処理

このノードでは、前回コマンド `q0` と新しい目標値 `q1` の間を線形補間します。

```python

def linear_interpolation(q0, q1, T, t):
    s = min(1.0, max(0.0, t / T))
    return (q1 - q0)*s + q0

```

## 推論側コード最小構成例
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

rospy.init_node("joint_prediction_sender")

joint_pub = rospy.Publisher("/joint_predictions", JointState, queue_size=1)

joint_names = [
    'left_arm/joint_1',
    'left_arm/joint_2',
    'left_arm/joint_3',
    'left_arm/joint_4',
    'left_arm/joint_5',
    'left_arm/joint_6',
    'left_arm/joint_7',
    'right_arm/joint_1',
    'right_arm/joint_2',
    'right_arm/joint_3',
    'right_arm/joint_4',
    'right_arm/joint_5',
    'right_arm/joint_6',
    'right_arm/joint_7'
]

rate = rospy.Rate(3)  # 例: 推論結果を 3Hz で送信

while not rospy.is_shutdown():
    # pred_joint は推論モデルの出力を想定
    # 14要素以上ある場合は先頭14要素を使用
    pred_joint = [0.0] * 14

    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = joint_names
    joint_msg.position = pred_joint[:14]

    joint_pub.publish(joint_msg)
    rate.sleep()
```
