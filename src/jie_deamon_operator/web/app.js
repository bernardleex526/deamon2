/**
 * 机器人控制的Web可视化前端
 * 使用WebSocket接收实时数据，Canvas绘制可视化
 */

// 配置常量
const CONFIG = {
    WS_PORT: 8890,
    DISPLAY_RANGE_FRONT: 3.0,  // 前方显示范围 (米)
    DISPLAY_RANGE_BACK: 2.0,   // 后方显示范围 (米)
    TARGET_RADIUS: 0.3,        // 目标半径 (米)
    MAX_LINEAR_SPEED: 1.0,
    MAX_ANGULAR_SPEED: 2.0,
    RECONNECT_DELAY: 2000
};

// 全局状态
let ws = null;
let canvas, ctx;
let canvasWidth = 800;   // canvas宽度
let canvasHeight = 800;  // canvas高度（会动态调整）
let metersToPixels = 100; // 每米对应像素数
let robotCenter = { x: canvasWidth / 2, y: canvasHeight - CONFIG.DISPLAY_RANGE_BACK * metersToPixels };

// 当前数据
let lidarData = {
    points: [],
    target: { x: 0.4, y: 0 },
    isMovingEnabled: false,
    isActive: false,
    velocity: { vx: 0, vy: 0, wz: 0 },
    rectangleWidth: 0.35  // 默认值
};

// 初始化
document.addEventListener('DOMContentLoaded', () => {
    initCanvas();
    initWebSocket();
    initControls();
    requestAnimationFrame(render);
});

// 初始化Canvas
function initCanvas() {
    canvas = document.getElementById('lidarCanvas');
    ctx = canvas.getContext('2d');

    resizeCanvas();
    window.addEventListener('resize', resizeCanvas);

    // 双击/双触摸设置目标
    canvas.addEventListener('dblclick', handleCanvasClick);

    // 双触摸实现
    let lastTouchTime = 0;
    canvas.addEventListener('touchend', (e) => {
        const now = Date.now();
        if (now - lastTouchTime < 300) {  // 300ms内双触摸
            handleCanvasTouch(e);
        }
        lastTouchTime = now;
    });

    initTabSwitching();
    initDirectControl();
}

// 调整Canvas尺寸（高DPI支持，矩形布局）
function resizeCanvas() {
    if (!canvas || !ctx) return;

    const rect = canvas.parentElement.getBoundingClientRect();
    const containerWidth = rect.width;
    const containerHeight = rect.height;

    // 如果尺寸为0（面板隐藏中），不进行调整
    if (containerWidth <= 0 || containerHeight <= 0) return;

    // 计算canvas尺寸：宽度填满容器，高度增加300像素
    const width = containerWidth;
    const height = Math.min(containerHeight, width + 300);  // 增加300像素高度

    const dpr = window.devicePixelRatio || 1;

    canvas.width = width * dpr;
    canvas.height = height * dpr;
    canvas.style.width = width + 'px';
    canvas.style.height = height + 'px';

    ctx.scale(dpr, dpr);
    canvasWidth = width;
    canvasHeight = height;

    // 计算每米对应的像素数（以宽度为基准，假设左右各显示2米范围）
    const horizontalRange = 4.0;  // 左右共4米显示范围
    metersToPixels = canvasWidth / horizontalRange;

    // 机器人位置：距离底部1.5米（后方显示范围）
    robotCenter = {
        x: canvasWidth / 2,
        y: canvasHeight - CONFIG.DISPLAY_RANGE_BACK * metersToPixels
    };
}

// 标签页切换
function initTabSwitching() {
    const tabs = document.querySelectorAll('.nav-item');
    const panels = document.querySelectorAll('.tab-content');

    tabs.forEach(tab => {
        tab.addEventListener('click', () => {
            // 移除所有激活状态
            tabs.forEach(t => t.classList.remove('active'));
            panels.forEach(p => {
                p.classList.remove('active');
                p.style.display = 'none';
            });

            // 激活当前标签
            tab.classList.add('active');
            const targetId = `panel-${tab.dataset.tab}`;
            const targetPanel = document.getElementById(targetId);
            if (targetPanel) {
                targetPanel.style.display = 'flex';
                // 强制重绘以触发动画
                void targetPanel.offsetWidth;
                targetPanel.classList.add('active');

                // 如果切换到雷达跟随面板，刷新canvas尺寸
                if (tab.dataset.tab === 'follow') {
                    setTimeout(resizeCanvas, 50);
                }
            }

            // 发送模式切换指令
            let mode = 1; // 默认跟随
            if (tab.dataset.tab === 'direct') mode = 0;
            else if (tab.dataset.tab === 'nav') mode = 2;

            sendCommand({
                type: 'switch_mode',
                mode: mode
            });

            // 如果切换到直接控制，自动开启运动使能（可选，根据需求）
            /*
            if (mode === 0) {
                sendCommand({ type: 'set_moving', enabled: true });
            }
            */
        });
    });
}

// 直接控制初始化
function initDirectControl() {
    initJoystick();
    initSlider();
    initSpeedModeSwitch();
}

// 速度模式切换初始化
function initSpeedModeSwitch() {
    const toggle = document.getElementById('speedModeToggle');
    const container = document.querySelector('.speed-mode-switch');

    if (!toggle || !container) return;

    toggle.addEventListener('change', () => {
        isHighSpeedMode = toggle.checked;
        container.classList.toggle('high-speed', isHighSpeedMode);
        console.log('速度模式:', isHighSpeedMode ? '高速' : '低速');
    });
}

// 虚拟摇杆逻辑
function initJoystick() {
    const base = document.getElementById('joystickBase');
    const stick = document.getElementById('joystickStick');
    const area = document.querySelector('.joystick-area');

    let activeTouchId = null; // 追踪触控点ID
    const maxRadius = 140; // (360-80)/2, 适配360px底盘

    function handleStart(e) {
        if (e.type === 'mousedown') {
            activeTouchId = 'mouse';
        } else {
            // 记录这个控件对应的touch ID
            activeTouchId = e.changedTouches[0].identifier;
        }
        const touch = e.type === 'mousedown' ? e : e.changedTouches[0];
        const rect = base.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        updateJoystick(touch.clientX, touch.clientY, centerX, centerY);
    }

    function handleMove(e) {
        if (activeTouchId === null) return;
        e.preventDefault();

        let touch;
        if (e.type === 'mousemove') {
            if (activeTouchId !== 'mouse') return;
            touch = e;
        } else {
            // 查找对应ID的触控点
            touch = Array.from(e.touches).find(t => t.identifier === activeTouchId);
            if (!touch) return;
        }

        const rect = base.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        updateJoystick(touch.clientX, touch.clientY, centerX, centerY);
    }

    function handleEnd(e) {
        if (activeTouchId === null) return;

        if (e.type === 'mouseup') {
            if (activeTouchId !== 'mouse') return;
        } else {
            // 检查是否是我们追踪的触控点结束
            const ended = Array.from(e.changedTouches).find(t => t.identifier === activeTouchId);
            if (!ended) return;
        }

        activeTouchId = null;
        stick.style.transform = `translate(-50%, -50%)`;
        // 只清零平移速度，保持当前角速度
        sendVelocity(0, 0, currentWz);
    }

    function updateJoystick(clientX, clientY, centerX, centerY) {
        let dx = clientX - centerX;
        let dy = clientY - centerY;
        const distance = Math.sqrt(dx * dx + dy * dy);

        if (distance > maxRadius) {
            const angle = Math.atan2(dy, dx);
            dx = Math.cos(angle) * maxRadius;
            dy = Math.sin(angle) * maxRadius;
        }

        stick.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;

        // 映射到速度 (前后x, 左右y)
        const vx = -(dy / maxRadius) * CONFIG.MAX_LINEAR_SPEED;
        const vy = -(dx / maxRadius) * CONFIG.MAX_LINEAR_SPEED;

        sendVelocity(vx, vy, currentWz);
    }

    base.addEventListener('mousedown', handleStart);
    document.addEventListener('mousemove', handleMove);
    document.addEventListener('mouseup', handleEnd);

    base.addEventListener('touchstart', handleStart, { passive: false });
    document.addEventListener('touchmove', handleMove, { passive: false });
    document.addEventListener('touchend', handleEnd);
    document.addEventListener('touchcancel', handleEnd);
}

// 旋转滑块逻辑
let currentWz = 0;
function initSlider() {
    const knob = document.getElementById('rotationKnob');
    const container = document.querySelector('.slider-container');

    let activeTouchId = null; // 追踪触控点ID

    function handleStart(e) {
        if (e.type === 'mousedown') {
            activeTouchId = 'mouse';
        } else {
            activeTouchId = e.changedTouches[0].identifier;
        }
    }

    function handleMove(e) {
        if (activeTouchId === null) return;
        e.preventDefault();

        let touch;
        if (e.type === 'mousemove') {
            if (activeTouchId !== 'mouse') return;
            touch = e;
        } else {
            touch = Array.from(e.touches).find(t => t.identifier === activeTouchId);
            if (!touch) return;
        }

        const rect = container.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;

        let offset = touch.clientX - centerX;
        offset = Math.max(-rect.width / 2 + 20, Math.min(rect.width / 2 - 20, offset));

        knob.style.transform = `translateX(calc(-50% + ${offset}px))`;

        // 映射角速度: 向左(offset<0) -> z>0, 向右(offset>0) -> z<0
        currentWz = -(offset / (rect.width / 2)) * CONFIG.MAX_ANGULAR_SPEED;
        sendVelocity(lastVx, lastVy, currentWz);
    }

    function handleEnd(e) {
        if (activeTouchId === null) return;

        if (e.type === 'mouseup') {
            if (activeTouchId !== 'mouse') return;
        } else {
            const ended = Array.from(e.changedTouches).find(t => t.identifier === activeTouchId);
            if (!ended) return;
        }

        activeTouchId = null;
        knob.style.transform = `translateX(-50%)`;
        currentWz = 0;
        sendVelocity(lastVx, lastVy, 0);
    }

    knob.addEventListener('mousedown', handleStart);
    document.addEventListener('mousemove', handleMove);
    document.addEventListener('mouseup', handleEnd);

    knob.addEventListener('touchstart', handleStart, { passive: false });
    document.addEventListener('touchmove', handleMove, { passive: false });
    document.addEventListener('touchend', handleEnd);
    document.addEventListener('touchcancel', handleEnd);
}

// 缓存最后的速度指令
let lastVx = 0;
let lastVy = 0;

// 速度模式：false=低速(0.5倍), true=高速(1.0倍)
let isHighSpeedMode = false;
const LOW_SPEED_RATIO = 0.5;
const HIGH_SPEED_RATIO = 1.0;

// 发送限频：10Hz (100ms 间隔)
let lastSendTime = 0;
const MIN_SEND_INTERVAL = 100;

// 缓存待发送的速度
let pendingVx = 0;
let pendingVy = 0;
let pendingWz = 0;
let velocitySendTimer = null;

function sendVelocity(vx, vy, wz) {
    lastVx = vx;
    lastVy = vy;

    // 缓存最新速度
    pendingVx = vx;
    pendingVy = vy;
    pendingWz = wz;

    // 检查是否需要立即发送（速度归零时强制发送）
    const isZeroVelocity = (vx === 0 && vy === 0 && wz === 0);
    const now = Date.now();

    if (isZeroVelocity || now - lastSendTime >= MIN_SEND_INTERVAL) {
        doSendVelocity();
    }
}

function doSendVelocity() {
    lastSendTime = Date.now();

    // 根据速度模式应用倍率
    const ratio = isHighSpeedMode ? HIGH_SPEED_RATIO : LOW_SPEED_RATIO;
    let scaledVx = pendingVx * ratio;
    let scaledVy = pendingVy * ratio;
    let scaledWz = pendingWz * ratio;

    // 添加死区过滤：确保纯粹的移动
    // 有些底层控制器对同时有vx和vy的复合指令响应不佳
    if (Math.abs(scaledVx) < 0.05) scaledVx = 0;
    if (Math.abs(scaledVy) < 0.05) scaledVy = 0;
    if (Math.abs(scaledWz) < 0.05) scaledWz = 0;

    console.log(`[${Date.now()}] 发送速度: vx=${scaledVx.toFixed(2)}, vy=${scaledVy.toFixed(2)}, wz=${scaledWz.toFixed(2)}`);

    sendCommand({
        type: 'direct_cmd',
        x: scaledVx,
        y: scaledVy,
        z: scaledWz
    });
}

// 坐标转换：机器人坐标 -> 像素坐标
function toPixel(robotX, robotY) {
    return {
        x: robotCenter.x - robotY * metersToPixels,
        y: robotCenter.y - robotX * metersToPixels
    };
}

// 坐标转换：像素坐标 -> 机器人坐标
function toRobot(pixelX, pixelY) {
    return {
        x: (robotCenter.y - pixelY) / metersToPixels,
        y: (robotCenter.x - pixelX) / metersToPixels
    };
}

// 处理画布双击
function handleCanvasClick(e) {
    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;
    setTarget(x, y);
}

// 处理双触摸
function handleCanvasTouch(e) {
    e.preventDefault();
    if (e.changedTouches.length > 0) {
        const touch = e.changedTouches[0];
        const rect = canvas.getBoundingClientRect();
        const x = touch.clientX - rect.left;
        const y = touch.clientY - rect.top;
        setTarget(x, y);
    }
}

// 设置目标位置（整个canvas区域都可以设置）
function setTarget(pixelX, pixelY) {
    const robot = toRobot(pixelX, pixelY);
    sendCommand({
        type: 'set_target',
        x: robot.x,
        y: robot.y
    });
}

// 初始化WebSocket
function initWebSocket() {
    const host = window.location.hostname || 'localhost';
    const wsUrl = `ws://${host}:${CONFIG.WS_PORT}`;

    updateConnectionStatus('connecting');

    try {
        ws = new WebSocket(wsUrl);

        ws.onopen = () => {
            console.log('WebSocket已连接');
            updateConnectionStatus('connected');

            // 发送初始模式：根据当前激活的标签页设置模式
            const activeTab = document.querySelector('.nav-item.active');
            if (activeTab) {
                let mode = 1; // 默认跟随
                if (activeTab.dataset.tab === 'direct') mode = 0;
                else if (activeTab.dataset.tab === 'nav') mode = 2;

                sendCommand({
                    type: 'switch_mode',
                    mode: mode
                });
                console.log('发送初始控制模式:', mode);
            }
        };

        ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                handleData(data);
            } catch (e) {
                console.error('解析数据失败:', e);
            }
        };

        ws.onclose = () => {
            console.log('WebSocket已断开');
            updateConnectionStatus('disconnected');
            setTimeout(initWebSocket, CONFIG.RECONNECT_DELAY);
        };

        ws.onerror = (error) => {
            console.error('WebSocket错误:', error);
            updateConnectionStatus('disconnected');
        };
    } catch (e) {
        console.error('创建WebSocket失败:', e);
        updateConnectionStatus('disconnected');
        setTimeout(initWebSocket, CONFIG.RECONNECT_DELAY);
    }
}

// 更新连接状态
function updateConnectionStatus(status) {
    const el = document.getElementById('connectionStatus');
    const textEl = el.querySelector('.status-text');

    el.className = 'connection-status ' + status;

    switch (status) {
        case 'connected':
            textEl.textContent = '已连接';
            break;
        case 'disconnected':
            textEl.textContent = '已断开';
            break;
        default:
            textEl.textContent = '连接中...';
    }
}

// 处理接收到的数据
function handleData(data) {
    if (data.type === 'scan_data') {
        lidarData.points = data.points || [];
        lidarData.target = data.target || lidarData.target;
        lidarData.isMovingEnabled = data.is_moving_enabled || false;
        lidarData.isActive = data.is_active || false;
        lidarData.velocity = {
            vx: data.velocity?.vx || 0,
            vy: data.velocity?.vy || 0,
            wz: data.velocity?.wz || 0
        };
        lidarData.rectangleWidth = data.rectangle_width || 0.35;

        updateUI();
    }
}

// 更新UI
function updateUI() {
    // 目标位置
    document.getElementById('targetX').textContent = lidarData.target.x.toFixed(2) + ' m';
    document.getElementById('targetY').textContent = lidarData.target.y.toFixed(2) + ' m';

    // 绘制速度显示
    drawSpeedDisplay();

    // 按钮显示点击后的动作：运动中显示"停止"，停止时显示"开始"
    updateButtonState('movingToggle', lidarData.isMovingEnabled, '停止运动', '开始运动', '⏸️', '▶️');
}

// 绘制速度显示（十字坐标+圆弧）
function drawSpeedDisplay() {
    const canvas = document.getElementById('speedCanvas');
    if (!canvas) return;

    // 自适应容器宽度
    const container = canvas.parentElement;
    if (container) {
        canvas.width = container.clientWidth || 180;
    }

    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;

    // 清除画布
    ctx.fillStyle = '#1a2332';
    ctx.fillRect(0, 0, width, height);

    // 根据宽度计算布局
    const barLength = Math.min(30, height / 2 - 10);
    const arcRadius = barLength + 6;
    const centerX = arcRadius + 5;  // 圆弧左边距
    const centerY = height / 2;
    const labelX = centerX + arcRadius + 8;  // 标签位置

    const vx = lidarData.velocity.vx;
    const vy = lidarData.velocity.vy;
    const wz = lidarData.velocity.wz;

    // 绘制角速度圆弧背景（先绘制在底层）
    ctx.strokeStyle = 'rgba(100, 100, 100, 0.5)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(centerX, centerY, arcRadius, 0, Math.PI * 2);
    ctx.stroke();

    // 零点标记（顶部）
    ctx.strokeStyle = 'rgba(150, 150, 150, 0.8)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - arcRadius - 4);
    ctx.lineTo(centerX, centerY - arcRadius + 4);
    ctx.stroke();

    // 绘制角速度弧线
    if (Math.abs(wz) > 0.01) {
        const startAngle = -Math.PI / 2;
        const arcAngle = -(wz / CONFIG.MAX_ANGULAR_SPEED) * Math.PI;
        const endAngle = startAngle + arcAngle;

        ctx.strokeStyle = '#3b82f6';
        ctx.lineWidth = 3;
        ctx.lineCap = 'round';
        ctx.beginPath();
        if (arcAngle > 0) {
            ctx.arc(centerX, centerY, arcRadius, startAngle, endAngle);
        } else {
            ctx.arc(centerX, centerY, arcRadius, endAngle, startAngle);
        }
        ctx.stroke();

        // 末端圆点
        const arrowAngle = endAngle;
        const arrowX = centerX + arcRadius * Math.cos(arrowAngle);
        const arrowY = centerY + arcRadius * Math.sin(arrowAngle);
        ctx.beginPath();
        ctx.arc(arrowX, arrowY, 3, 0, Math.PI * 2);
        ctx.fillStyle = '#3b82f6';
        ctx.fill();
    }

    // 绘制十字参考线
    ctx.strokeStyle = 'rgba(100, 100, 100, 0.5)';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(centerX - barLength - 3, centerY);
    ctx.lineTo(centerX + barLength + 3, centerY);
    ctx.moveTo(centerX, centerY - barLength - 3);
    ctx.lineTo(centerX, centerY + barLength + 3);
    ctx.stroke();

    // 计算速度条长度
    const barX = (vx / CONFIG.MAX_LINEAR_SPEED) * barLength;
    const barY = (vy / CONFIG.MAX_LINEAR_SPEED) * barLength;

    // 绘制Vx（红色垂直线，向上为正）
    if (Math.abs(barX) > 1) {
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 3;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(centerX, centerY - barX);
        ctx.stroke();

        // 箭头
        const arrowY = centerY - barX;
        const arrowDir = barX > 0 ? 1 : -1;
        ctx.beginPath();
        ctx.moveTo(centerX, arrowY - arrowDir * 5);
        ctx.lineTo(centerX - 3, arrowY);
        ctx.lineTo(centerX + 3, arrowY);
        ctx.closePath();
        ctx.fillStyle = '#ef4444';
        ctx.fill();
    }

    // 绘制Vy（绿色水平线，向左为正）
    if (Math.abs(barY) > 1) {
        ctx.strokeStyle = '#10b981';
        ctx.lineWidth = 3;
        ctx.lineCap = 'round';
        ctx.beginPath();
        ctx.moveTo(centerX, centerY);
        ctx.lineTo(centerX - barY, centerY);
        ctx.stroke();

        // 箭头
        const arrowX = centerX - barY;
        const arrowDir = barY > 0 ? 1 : -1;
        ctx.beginPath();
        ctx.moveTo(arrowX + arrowDir * 5, centerY);
        ctx.lineTo(arrowX, centerY - 3);
        ctx.lineTo(arrowX, centerY + 3);
        ctx.closePath();
        ctx.fillStyle = '#10b981';
        ctx.fill();
    }

    // 中心点
    ctx.beginPath();
    ctx.arc(centerX, centerY, 3, 0, Math.PI * 2);
    ctx.fillStyle = '#ffffff';
    ctx.fill();

    // 速度数值标签（右侧竖直排列，数值右对齐）
    ctx.font = '16px sans-serif';
    const valueX = width - 5; // 右边距5px

    // Vx
    ctx.textAlign = 'left';
    ctx.fillStyle = '#ef4444';
    ctx.fillText('Vx:', labelX, centerY - 18);
    ctx.textAlign = 'right';
    ctx.fillText(vx.toFixed(2), valueX, centerY - 18);

    // Vy
    ctx.textAlign = 'left';
    ctx.fillStyle = '#10b981';
    ctx.fillText('Vy:', labelX, centerY + 2);
    ctx.textAlign = 'right';
    ctx.fillText(vy.toFixed(2), valueX, centerY + 2);

    // Wz
    ctx.textAlign = 'left';
    ctx.fillStyle = '#3b82f6';
    ctx.fillText('Wz:', labelX, centerY + 22);
    ctx.textAlign = 'right';
    ctx.fillText(wz.toFixed(2), valueX, centerY + 22);
}

// 更新按钮状态
function updateButtonState(id, isActive, activeText, inactiveText, activeIcon, inactiveIcon) {
    const btn = document.getElementById(id);
    btn.classList.toggle('active', isActive);
    btn.querySelector('.btn-text').textContent = isActive ? activeText : inactiveText;
    btn.querySelector('.btn-icon').textContent = isActive ? activeIcon : inactiveIcon;
}

// 初始化控制按钮
function initControls() {
    document.getElementById('movingToggle').addEventListener('click', () => {
        sendCommand({
            type: 'set_moving',
            enabled: !lidarData.isMovingEnabled
        });
    });

    const btnLieDown = document.getElementById('btnLieDown');
    if (btnLieDown) {
        btnLieDown.addEventListener('click', () => {
            sendCommand({ type: 'action_cmd', action: 'liedown' });
            // 添加点击反馈动画
            btnLieDown.style.transform = 'scale(0.95)';
            setTimeout(() => btnLieDown.style.transform = '', 100);
        });
    }

    const btnStandUp = document.getElementById('btnStandUp');
    if (btnStandUp) {
        btnStandUp.addEventListener('click', () => {
            sendCommand({ type: 'action_cmd', action: 'standup' });
            btnStandUp.style.transform = 'scale(0.95)';
            setTimeout(() => btnStandUp.style.transform = '', 100);
        });
    }
}

// 发送命令
function sendCommand(cmd) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(cmd));
    }
}

// 渲染循环
function render() {
    drawLidar();
    requestAnimationFrame(render);
}

// 绘制雷达可视化
function drawLidar() {
    const dpr = window.devicePixelRatio || 1;
    ctx.resetTransform();
    ctx.scale(dpr, dpr);

    // 清除画布
    ctx.fillStyle = '#0a0e17';
    ctx.fillRect(0, 0, canvasWidth, canvasHeight);

    // 绘制网格
    drawGrid();

    // 绘制目标连接线和矩形区域
    drawTargetZone();

    // 绘制雷达点
    drawPoints();

    // 绘制目标圆
    drawTarget();

    // 绘制机器人
    drawRobot();

    // 绘制状态
    drawStatus();
}

// 绘制网格
function drawGrid() {
    ctx.strokeStyle = 'rgba(34, 211, 238, 0.1)';
    ctx.lineWidth = 1;

    // 同心圆
    for (let r = 1; r <= 4; r++) {
        ctx.beginPath();
        ctx.arc(robotCenter.x, robotCenter.y, r * metersToPixels, 0, Math.PI * 2);
        ctx.stroke();

        // 距离标签
        ctx.fillStyle = 'rgba(148, 163, 184, 0.5)';
        ctx.font = '12px sans-serif';
        ctx.fillText(r + 'm', robotCenter.x + 5, robotCenter.y - r * metersToPixels + 15);
    }

    // 十字线
    ctx.strokeStyle = 'rgba(34, 211, 238, 0.15)';
    ctx.beginPath();
    ctx.moveTo(0, robotCenter.y);
    ctx.lineTo(canvasWidth, robotCenter.y);
    ctx.moveTo(robotCenter.x, 0);
    ctx.lineTo(robotCenter.x, canvasHeight);
    ctx.stroke();
}

// 绘制目标区域和矩形
function drawTargetZone() {
    const target = lidarData.target;
    const dist = Math.sqrt(target.x * target.x + target.y * target.y);
    const rectWidth = lidarData.rectangleWidth;

    if (dist > 0.01) {
        const targetPixel = toPixel(target.x, target.y);
        const angle = Math.atan2(target.y, target.x);
        const halfWidth = rectWidth / 2.0;

        // 绘制矩形区域（与OpenCV一致）
        const corners = [
            { x: 0 - halfWidth * Math.sin(angle), y: 0 + halfWidth * Math.cos(angle) },
            { x: 0 + halfWidth * Math.sin(angle), y: 0 - halfWidth * Math.cos(angle) },
            { x: target.x + halfWidth * Math.sin(angle), y: target.y - halfWidth * Math.cos(angle) },
            { x: target.x - halfWidth * Math.sin(angle), y: target.y + halfWidth * Math.cos(angle) }
        ];

        const pixelCorners = corners.map(c => toPixel(c.x, c.y));

        // 填充矩形
        ctx.fillStyle = 'rgba(80, 80, 80, 0.5)';
        ctx.beginPath();
        ctx.moveTo(pixelCorners[0].x, pixelCorners[0].y);
        ctx.lineTo(pixelCorners[1].x, pixelCorners[1].y);
        ctx.lineTo(pixelCorners[2].x, pixelCorners[2].y);
        ctx.lineTo(pixelCorners[3].x, pixelCorners[3].y);
        ctx.closePath();
        ctx.fill();

        // 矩形边框
        ctx.strokeStyle = 'rgba(100, 100, 100, 0.8)';
        ctx.lineWidth = 1;
        ctx.stroke();

        // 连接线
        ctx.strokeStyle = 'rgba(16, 185, 129, 0.6)';
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(robotCenter.x, robotCenter.y);
        ctx.lineTo(targetPixel.x, targetPixel.y);
        ctx.stroke();
        ctx.setLineDash([]);
    }
}

// 绘制雷达点
function drawPoints() {
    const target = lidarData.target;
    // 避免除以零
    const targetSq = target.x * target.x + target.y * target.y;
    const targetLen = Math.sqrt(targetSq);
    const rectHalfWidth = lidarData.rectangleWidth / 2.0;

    const normalPoints = [];
    const specialPoints = [];

    for (const point of lidarData.points) {
        let isSpecial = false;

        if (targetLen > 0.01) {
            // 计算投影长度 proj_x = (P . T) / |T|
            const proj_x = (point.x * target.x + point.y * target.y) / targetLen;

            // 计算垂直距离 proj_y = (P x T) / |T|  (这里用叉积模长，或者旋转坐标后的y分量)
            // y' = y*cos - x*sin = y*(tx/len) - x*(ty/len)
            const proj_y = (point.y * target.x - point.x * target.y) / targetLen;

            // 检查是否在矩形通道内
            if (proj_x >= 0 && proj_x <= targetLen && Math.abs(proj_y) <= rectHalfWidth) {
                // 检查是否在目标圆外
                const dx = point.x - target.x;
                const dy = point.y - target.y;
                if (dx * dx + dy * dy > CONFIG.TARGET_RADIUS * CONFIG.TARGET_RADIUS) {
                    isSpecial = true;
                }
            }
        }

        // 转换为像素坐标
        const p = toPixel(point.x, point.y);

        if (isSpecial) {
            specialPoints.push(p);
        } else {
            normalPoints.push(p);
        }
    }

    // 1. 绘制特殊点的连线 (半透明黄色)
    if (specialPoints.length > 0) {
        ctx.strokeStyle = 'rgba(255, 255, 0, 0.3)';
        ctx.lineWidth = 3;
        ctx.beginPath();
        for (const p of specialPoints) {
            ctx.moveTo(robotCenter.x, robotCenter.y);
            ctx.lineTo(p.x, p.y);
        }
        ctx.stroke();
    }

    // 2. 绘制普通点 (红色)
    if (normalPoints.length > 0) {
        ctx.fillStyle = '#ef4444';
        ctx.shadowColor = '#ef4444';
        ctx.shadowBlur = 3;
        ctx.beginPath();
        for (const p of normalPoints) {
            ctx.moveTo(p.x + 2, p.y);
            ctx.arc(p.x, p.y, 2, 0, Math.PI * 2);
        }
        ctx.fill();
    }

    // 3. 绘制特殊点 (黄色)
    if (specialPoints.length > 0) {
        ctx.fillStyle = '#ffff00';
        ctx.shadowColor = '#ffff00';
        ctx.shadowBlur = 5;
        ctx.beginPath();
        for (const p of specialPoints) {
            ctx.moveTo(p.x + 3, p.y);
            ctx.arc(p.x, p.y, 3, 0, Math.PI * 2);
        }
        ctx.fill();
    }

    ctx.shadowBlur = 0;
}

// 绘制目标
function drawTarget() {
    const target = lidarData.target;
    const p = toPixel(target.x, target.y);
    const radius = CONFIG.TARGET_RADIUS * metersToPixels;

    // 目标圆
    ctx.strokeStyle = '#a855f7';
    ctx.lineWidth = 2;
    ctx.shadowColor = '#a855f7';
    ctx.shadowBlur = 10;
    ctx.beginPath();
    ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
    ctx.stroke();

    // 十字
    ctx.beginPath();
    ctx.moveTo(p.x - 10, p.y);
    ctx.lineTo(p.x + 10, p.y);
    ctx.moveTo(p.x, p.y - 10);
    ctx.lineTo(p.x, p.y + 10);
    ctx.stroke();

    ctx.shadowBlur = 0;
}

// 绘制机器人
function drawRobot() {
    // 机器人主体
    ctx.fillStyle = '#22d3ee';
    ctx.shadowColor = '#22d3ee';
    ctx.shadowBlur = 15;
    ctx.beginPath();
    ctx.arc(robotCenter.x, robotCenter.y, 12, 0, Math.PI * 2);
    ctx.fill();

    // 方向指示
    ctx.strokeStyle = '#22d3ee';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(robotCenter.x, robotCenter.y);
    ctx.lineTo(robotCenter.x, robotCenter.y - 25);
    ctx.stroke();

    // 箭头
    ctx.beginPath();
    ctx.moveTo(robotCenter.x, robotCenter.y - 30);
    ctx.lineTo(robotCenter.x - 8, robotCenter.y - 20);
    ctx.lineTo(robotCenter.x + 8, robotCenter.y - 20);
    ctx.closePath();
    ctx.fill();

    ctx.shadowBlur = 0;
}

// 绘制状态
function drawStatus() {
    const status = lidarData.isMovingEnabled ? 'MOVING' : 'STOPPED';
    const color = lidarData.isMovingEnabled ? '#10b981' : '#ef4444';

    ctx.font = 'bold 16px sans-serif';
    ctx.fillStyle = color;
    ctx.shadowColor = color;
    ctx.shadowBlur = 5;
    ctx.fillText(status, 10, 22);

    ctx.shadowBlur = 0;
}
