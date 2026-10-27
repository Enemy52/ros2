# 任务④：端到端视觉导航（PPO + CNN）

## 架构
```
/camera/image_raw (640x480 RGB)
        ↓
   CNN (64x48 灰度)
        ↓
   PPO Policy
        ↓
   /uav/cmd_vel (linear_x, angular_z)
```

## 运行
```bash
# 1. 启动 bridge
roslaunch carlair_ros_bridge main.launch

# 2. 训练（2万步，约 5-10 分钟）
rosrun uav_e2e_rl train.py

# 3. 部署
roslaunch uav_e2e_rl e2e_deploy.launch
```

## 关键参数
- 观测：48x64 灰度图，归一化 [0,1]
- 动作：[linear_x ∈ [0,2] m/s, angular_z ∈ [-1,1] rad/s]
- 奖励：前进 * 0.5 - 碰撞 * 5 - 转向 * 0.1
- 碰撞阈值：scan min < 0.6m
- 算法：PPO（CnnPolicy, n_steps=64, batch=32, lr=3e-4）

## 文件
- `scripts/train.py` — PPO 训练脚本
- `scripts/deploy_node.py` — 部署节点
- `launch/e2e_deploy.launch` — 启动部署
- `models/ppo_e2e.zip` — 训练好的权重
