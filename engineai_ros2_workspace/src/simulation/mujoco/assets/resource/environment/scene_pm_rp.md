# PM_V2 与 Renderpeople 人体模型场景配置文档

## 1. MuJoCo 场景坐标系

```
        Z (上)
        |
        |
        +-------→ Y (前方)
       /
      /
     X (右)
```

- **X轴**: 左右方向（从机器人视角看，+X为右侧）
- **Y轴**: 前后方向（+Y为前方）
- **Z轴**: 上下方向（+Z为上方，地面Z=0）

---

## 2. PM_V2 机器人配置

### 2.1 位置
| 参数 | 值 | 说明 |
|------|------|------|
| X | 0 | 原点 |
| Y | 0 | 原点 |
| Z | 0 | 脚底在地面 |
| LINK_BASE高度 | 0.82m | 躯干基座高度 |

### 2.2 朝向
- **正面朝向**: +Y方向
- **四元数**: (1, 0, 0, 0) - 无旋转
- **验证依据**: 头部RGBD相机配置 `xyaxes="1 0 0 0 0 1"` 表示相机朝+Y方向

### 2.3 尺寸
| 参数 | 值 |
|------|------|
| 总高度 | ~1.2-1.3m |
| 头部高度 | ~1.2m (LINK_HEAD_YAW) |
| 相机高度 | ~1.4m (头顶+0.2m) |

### 2.4 配置文件
- 主文件: `robot/pm_v2/xml/serial_pm_v2.xml`
- 关节定义: `robot/pm_v2/xml/serial_links.xml`
- 模型资源: `robot/pm_v2/meshes/`

---

## 3. Renderpeople 人体模型配置

### 3.1 原始模型信息
| 参数 | 值 |
|------|------|
| 模型文件 | `rp_posed_00178_29_OBJ/rp_posed_00178_29.obj` |
| 原始单位 | 厘米 (cm) |
| 原始高度 | 172.5 cm |
| 原始坐标系 | Y轴向上，人脸朝-Z |

### 3.2 坐标系转换
需要将模型从其原始坐标系转换到MuJoCo场景坐标系：

| 转换步骤 | 操作 | 效果 |
|----------|------|------|
| 1. 缩放 | scale="0.01 0.01 0.01" | 厘米→米 (172.5cm → 1.725m) |
| 2. 站立 | 绕X轴+90° | 原Y轴→Z轴，人物站立 |
| 3. 转身 | 绕Z轴旋转 | 调整人脸朝向 |

### 3.3 朝向计算

**目标**: 人体模型面朝-Y方向（面向机器人）

原始模型人脸朝-Z，经过绕X轴+90°后：
- 原-Z方向 → +Y方向
- 人脸朝+Y（背对机器人）

要让人脸朝-Y（面向机器人），需要再绕Z轴+180°

**重要**: PM_V2使用 `eulerseq="zyx"`，euler格式为 `"Z轴 Y轴 X轴"`

**最终euler角**: `euler="180 0 90"` (Z=180°转身, Y=0°, X=90°站立)

---

## 4. 场景布局

### 4.1 俯视图
```
                    Y轴 (+Y 前方)
                      ↑
                      |
        人物3         |         人物2
    pos=(-1.5, 2.5)   |     pos=(1.5, 2.5)
        ◎             |             ◎
         \            |            /
          \     人物1 |           /
           \  pos=(0, 2.0)      /
            \    ◎             /
             \   |            /
              \  |           /
               \ |  2m距离  /
                \|         /
    ─────────────●─────────────→ X轴 (+X 右侧)
              PM_V2
            pos=(0, 0)
```

### 4.2 模型位置与朝向

| 模型 | 位置 (X, Y, Z) | euler角 (Z,Y,X) | 描述 |
|------|----------------|-----------------|------|
| PM_V2 | (0, 0, 0) | 无旋转 | 机器人，正面朝+Y |
| 人物1 | (0, 2.0, 0) | (180, 0, 90) | 正前方2m，正对机器人 |
| 人物2 | (1.5, 2.5, 0) | (150, 0, 90) | 右前方，偏右30° |
| 人物3 | (-1.5, 2.5, 0) | (210, 0, 90) | 左前方，偏左30° |

### 4.3 距离与角度说明
- **人物1**: 距离机器人2.0m，正面面对面
- **人物2**: 距离机器人约2.9m，位于右前方，身体朝向机器人偏右30°
- **人物3**: 距离机器人约2.9m，位于左前方，身体朝向机器人偏左30°

---

## 5. 尺寸比例对比

| 对象 | 高度 | 相对机器人 |
|------|------|------------|
| PM_V2机器人 | ~1.25m | 1.0x |
| Renderpeople人体 | 1.725m | ~1.38x |
| 真实成人 | 1.70m | 参考值 |

**比例合理性**: 人体模型高度1.725m符合成人平均身高，与1.25m的机器人形成合理的视觉对比。

---

## 6. 配置文件

### 6.1 human_models.xml

```xml
<mujoco model="human_models">
    <asset>
        <texture name="rp_human_tex" type="2d" 
                 file="rp_posed_00178_29_OBJ/tex/rp_posed_00178_29_basecolor.png"/>
        <material name="rp_human_mat" texture="rp_human_tex" 
                  specular="0.1" shininess="0.1"/>
        <mesh name="rp_human" 
              file="rp_posed_00178_29_OBJ/rp_posed_00178_29.obj"
              scale="0.01 0.01 0.01"/>
    </asset>

    <worldbody>
        <!-- 人物1: 正前方，面对机器人 (euler格式: Z Y X) -->
        <body name="rp_human_1" pos="0 2.0 0" euler="180 0 90">
            <geom name="rp_human_1_geom" type="mesh" mesh="rp_human"
                  material="rp_human_mat" contype="0" conaffinity="0"/>
        </body>

        <!-- 人物2: 右前方 -->
        <body name="rp_human_2" pos="1.5 2.5 0" euler="150 0 90">
            <geom name="rp_human_2_geom" type="mesh" mesh="rp_human"
                  material="rp_human_mat" contype="0" conaffinity="0"/>
        </body>

        <!-- 人物3: 左前方 -->
        <body name="rp_human_3" pos="-1.5 2.5 0" euler="210 0 90">
            <geom name="rp_human_3_geom" type="mesh" mesh="rp_human"
                  material="rp_human_mat" contype="0" conaffinity="0"/>
        </body>
    </worldbody>
</mujoco>
```

### 6.2 引用方式 (pm_v2.xml)

```xml
<mujoco model="global_model">
    <include file="robot/pm_v2/xml/serial_pm_v2.xml"/>
    <include file="environment/ground.xml"/>
    <include file="environment/human_models.xml"/>
</mujoco>
```

---

## 7. 调整指南

### 7.1 调整人物距离
修改body的`pos`属性中的Y值：
- 增大Y值 = 人物离机器人更远
- 减小Y值 = 人物离机器人更近

### 7.2 调整人物朝向
修改body的`euler`属性中的第一个值（Z轴旋转，eulerseq="zyx"）：
- `euler="180 0 90"` = 正对机器人
- `euler="150 0 90"` = 朝向机器人偏右30°
- `euler="210 0 90"` = 朝向机器人偏左30°

### 7.3 添加更多人物
复制body块，修改name、pos和euler：
```xml
<body name="rp_human_4" pos="2.0 3.0 0" euler="150 0 90">
    <geom name="rp_human_4_geom" type="mesh" mesh="rp_human"
          material="rp_human_mat" contype="0" conaffinity="0"/>
</body>
```

---

## 8. 注意事项

1. **纹理格式**: MuJoCo只支持PNG格式纹理，需将JPG转换为PNG
2. **碰撞检测**: `contype="0" conaffinity="0"` 禁用碰撞，避免与机器人干涉
3. **坐标轴**: MuJoCo使用Z-up坐标系，许多3D模型使用Y-up需要转换
4. **单位转换**: 确保模型单位与场景一致（米制）

---

## 9. 文件结构

```
environment/
├── human_models.xml          # 人体模型配置
├── scene_pm_rp.md           # 本文档
├── ground.xml               # 地面配置
└── rp_posed_00178_29_OBJ/   # Renderpeople模型目录
    ├── rp_posed_00178_29.obj
    └── tex/
        └── rp_posed_00178_29_basecolor.png
```

