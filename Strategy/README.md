# Strategy

赛道地图和策略决策目录。

- `track_map.*`：维护学习到的路段事件表，并提供 Flash 保存/加载。
- `track_strategy.*`：根据当前循迹状态、IMU、编码器和地图事件输出车辆模式、目标速度和转向限幅。

第一圈学习逻辑是启发式实现，后续可根据实际赛道标志和传感器稳定性继续细化阈值。

丢线搜索会使用最后一次有效循迹误差决定搜索方向，避免丢线时无效大误差导致车辆固定向同一侧搜索。

地图学习、地图 Flash 和预测调速分别受 `CAR_ENABLE_TRACK_MAP_LEARNING`、`CAR_ENABLE_TRACK_MAP_FLASH`、`CAR_ENABLE_MAP_PREDICTION` 控制，便于先单独验证基础循迹和速度控制。
