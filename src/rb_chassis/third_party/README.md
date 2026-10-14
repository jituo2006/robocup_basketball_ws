# 为什么自带 bupt_can（而不是用 /usr/local 的）

`/usr/local/lib/libbupt_can.a`（2025-04 安装）是**旧版**，缺少一个关键补丁：

## 补丁内容（来自 26_R1 对 2026-07-08 事故的修复）

```cpp
void Can::send_can(const can_frame &frame) {
    std::lock_guard<std::mutex> lock(send_que_mutex);
    constexpr std::size_t kMaxQueuedFrames = 64;
    if (send_que.size() >= kMaxQueuedFrames) {
        std::cerr << "CAN send queue overflow, dropping stale frames" << std::endl;
        std::queue<can_frame> empty;
        send_que.swap(empty);          // ← 丢弃积压的旧速度帧
    }
    send_que.push(frame);
    ...
}
```

## 为什么这个补丁必须要有（对应"停下后电机突然转动"）

`Can::send_thread()` 从 `send_que` 取帧写 socket。CAN 总线忙/出错时 `write()` 会阻塞，
`send_que` 无限增长。等总线恢复，积压的**旧速度帧**一股脑发出去 —— 于是你明明已经
发了停车指令，电机却先执行完积压的旧速度，表现为"停下后突然转动"。

旧版没有上限，队列能积压成千上万帧；补丁版超过 64 帧就整队丢弃，只保留最新命令。

## 另外顺手修了一个 bug

原库 `Can::~Can()` 无条件 `send_thread_->join()`，但这两个线程只在 `can_start()`
里创建 —— 构造了但没 `can_start()` 就会空指针段错误。这里已加 `joinable()` 判空。

## 头部一致性

`bupt_can.h` 与 /usr/local 版**逐字节一致**，所以 `motor_lib`（系统安装）里的
`DJIBoard` 拿到的 `Can` 类型 ABI 完全兼容，直接链接本目录编译出的 `rb_can` 即可。
