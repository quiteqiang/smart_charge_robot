# 泊靠/离桩结果上报（latched，TRANSIENT_LOCAL）。
# sequence 由 dock_controller 单调递增赋值，使结果与请求"世代"绑定：
# mission 端只接受晚于本次请求发出时刻发布的结果，避免重订阅/重启后
# 消费到 latched 的旧结果造成假成功。
bool success
uint32 sequence
