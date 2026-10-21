(() => {
const clampToEllipseInput = OverlayCore.clampNormalizedOffsetToEllipse;
const {clamp01, clamp11} = OverlayCore;

// Every input that reports a 0..1 "pressed amount" (buttons, bumpers, stick
// clicks, triggers). The stick movement axes (LX/LY/RX/RY) and d-pad axes
// (DX/DY) are handled separately because they are signed.
const DIGITAL_STATE_KEYS = [
    "A", "B", "X", "Y",
    "SELECT", "START", "GUIDE",
    "LB", "RB", "LS", "RS",
    "LT", "RT",
];

function createEmptyGamepadState() {
    return {
        A: 0, B: 0, X: 0, Y: 0,
        SELECT: 0, START: 0, GUIDE: 0,
        LB: 0, RB: 0, LS: 0, RS: 0,
        LX: 0, LY: 0, RX: 0, RY: 0,
        LT: 0, RT: 0,
        DX: 0, DY: 0,
    };
}

function normalizeGamepadState(partialState, {deadzoneMode = "none", fixedDeadzone = 0.0} = {}) {
    const state = {...createEmptyGamepadState(), ...partialState};
    for (const key of DIGITAL_STATE_KEYS) {
        state[key] = clamp01(state[key]);
    }

    const applyDeadzone = (value) => {
        value = clamp11(value);
        if (deadzoneMode === "none") {
            return value;
        }
        return Math.abs(value) < fixedDeadzone ? 0 : value;
    };

    state.LX = applyDeadzone(state.LX);
    state.LY = applyDeadzone(state.LY);
    state.RX = applyDeadzone(state.RX);
    state.RY = applyDeadzone(state.RY);
    state.DX = Math.sign(clamp11(state.DX));
    state.DY = Math.sign(clamp11(state.DY));
    return state;
}

function mapBrowserGamepadToState(gamepad) {
    if (!gamepad) {
        return createEmptyGamepadState();
    }
    const b = gamepad.buttons;
    const a = gamepad.axes;
    const up = clamp01(b[12]?.value ?? 0);
    const down = clamp01(b[13]?.value ?? 0);
    const left = clamp01(b[14]?.value ?? 0);
    const right = clamp01(b[15]?.value ?? 0);

    return {
        A: clamp01(b[0]?.value ?? 0),
        B: clamp01(b[1]?.value ?? 0),
        X: clamp01(b[2]?.value ?? 0),
        Y: clamp01(b[3]?.value ?? 0),
        LB: clamp01(b[4]?.value ?? 0),
        RB: clamp01(b[5]?.value ?? 0),
        LT: clamp01(b[6]?.value ?? 0),
        RT: clamp01(b[7]?.value ?? 0),
        SELECT: clamp01(b[8]?.value ?? 0),
        START: clamp01(b[9]?.value ?? 0),
        LS: clamp01(b[10]?.value ?? 0),
        RS: clamp01(b[11]?.value ?? 0),
        GUIDE: clamp01((b[16]?.value ?? b[17]?.value) ?? 0),
        LX: clamp11(a[0] ?? 0),
        LY: clamp11(a[1] ?? 0),
        RX: clamp11(a[2] ?? 0),
        RY: clamp11(a[3] ?? 0),
        DX: right - left,
        DY: down - up,
    };
}

class BrowserGamepadSource {
    #config;
    #pollTimer;
    #lastTimestamp;
    #cleanup;
    #lastPadListSignature;
    #selectedPadId;
    #selectedPadIndex;

    constructor(config) {
        this.#config = config;
        this.#pollTimer = null;
        this.#lastTimestamp = null;
        this.#cleanup = null;
        this.#lastPadListSignature = null;
        this.#selectedPadId = null;
        this.#selectedPadIndex = null;
    }

    start() {
        const onConnect = () => {
            if (this.#pollTimer == null) {
                this.#schedule();
            }
        };
        const onDisconnect = () => {
            const gamepads = navigator.getGamepads?.() ?? [];
            if (!gamepads[0]) {
                this.#config.onState(createEmptyGamepadState());
            }
        };
        window.addEventListener("gamepadconnected", onConnect);
        window.addEventListener("gamepaddisconnected", onDisconnect);
        this.#cleanup = () => {
            window.removeEventListener("gamepadconnected", onConnect);
            window.removeEventListener("gamepaddisconnected", onDisconnect);
        };
        this.#schedule();
    }

    stop() {
        if (this.#pollTimer != null) {
            clearTimeout(this.#pollTimer);
            this.#pollTimer = null;
        }
        this.#cleanup?.();
    }

    #schedule() {
        this.#pollTimer = setTimeout(() => this.#tick(), this.#config.pollMs);
    }

    #tick() {
        const gamepads = Array.from(navigator.getGamepads?.() ?? []);
        const selected = this.#selectGamepad(gamepads);
        this.#logPadListIfChanged(gamepads);

        if (!selected) {
            this.#config.onState(createEmptyGamepadState());
            this.#schedule();
            return;
        }

        if (typeof selected.timestamp === "number" && Number.isFinite(selected.timestamp)) {
            if (this.#lastTimestamp !== null && selected.timestamp === this.#lastTimestamp) {
                this.#schedule();
                return;
            }
            this.#lastTimestamp = selected.timestamp;
        }

        this.#config.onState(mapBrowserGamepadToState(selected));
        this.#schedule();
    }

    #selectGamepad(gamepads) {
        const connected = gamepads.filter((pad) => pad != null && pad.connected);
        const eligible = this.#config.padRequireStandard
            ? connected.filter((pad) => pad.mapping === "standard")
            : connected;
        eligible.sort((a, b) => a.index - b.index);

        if (eligible.length === 0) {
            this.#selectedPadId = null;
            this.#selectedPadIndex = null;
            return null;
        }

        const needle = (this.#config.padIdContains || "").trim().toLowerCase();
        if (needle) {
            const preferred = eligible.filter((pad) => pad.id.toLowerCase().includes(needle));
            if (preferred.length > 0) {
                return this.#rememberSelectedPad(preferred[0]);
            }
        }

        if (this.#selectedPadId != null || this.#selectedPadIndex != null) {
            const lockedExact = eligible.find((pad) => pad.id === this.#selectedPadId && pad.index === this.#selectedPadIndex);
            if (lockedExact) {
                return this.#rememberSelectedPad(lockedExact);
            }
            const lockedById = this.#selectedPadId == null
                ? null
                : eligible.find((pad) => pad.id === this.#selectedPadId);
            if (lockedById) {
                return this.#rememberSelectedPad(lockedById);
            }
            const lockedByIndex = this.#selectedPadIndex == null
                ? null
                : eligible.find((pad) => pad.index === this.#selectedPadIndex);
            if (lockedByIndex) {
                return this.#rememberSelectedPad(lockedByIndex);
            }
        }

        if (this.#config.padIndex != null) {
            const byIndexHint = eligible.find((pad) => pad.index === this.#config.padIndex);
            if (byIndexHint) {
                return this.#rememberSelectedPad(byIndexHint);
            }
        }

        return this.#rememberSelectedPad(eligible[0]);
    }

    #rememberSelectedPad(pad) {
        this.#selectedPadId = pad.id;
        this.#selectedPadIndex = pad.index;
        return pad;
    }

    #logPadListIfChanged(gamepads) {
        const connected = gamepads.filter((pad) => pad != null && pad.connected);
        const signature = connected
            .map((pad) => `${pad.index}|${pad.id}|${pad.mapping}|${pad.connected ? 1 : 0}`)
            .join("||");
        if (this.#lastPadListSignature === signature) {
            return;
        }
        this.#lastPadListSignature = signature;
        if (connected.length === 0) {
            console.info("[pads] none connected");
            return;
        }
        console.info("[pads] connected gamepads:");
        for (const pad of connected) {
            const line = [
                `id=\"${pad.id}\"`,
                `mapping=${pad.mapping || ""}`,
                `index=${pad.index}`,
                `connected=${pad.connected ? "yes" : "no"}`,
            ].join(" | ");
            console.info(`  - ${line}`);
        }
        console.info("[pads] Tip: use the exact id string (or a unique contiguous substring) with padIdContains=<value> to prefer a gamepad.");
    }
}

class WebSocketGamepadSource {
    #config;
    #retryDelay;
    #stopped;
    #ws;

    constructor(config) {
        this.#config = config;
        this.#retryDelay = 1000;
        this.#stopped = false;
        this.#ws = null;
    }

    start() {
        this.#connect();
    }

    stop() {
        this.#stopped = true;
        this.#ws?.close();
    }

    #connect() {
        if (this.#stopped) {
            return;
        }
        const ws = new WebSocket(this.#config.wsUrl);
        this.#ws = ws;
        let firstMessage = true;
        ws.onopen = () => {
            this.#retryDelay = 1000;
        };
        ws.onmessage = (event) => {
            if (firstMessage) {
                firstMessage = false;
                return;
            }
            try {
                this.#config.onState(JSON.parse(event.data));
            } catch {
                this.#config.onState(createEmptyGamepadState());
            }
        };
        ws.onclose = () => {
            this.#config.onState(createEmptyGamepadState());
            if (!this.#stopped) {
                setTimeout(() => this.#connect(), this.#retryDelay);
                this.#retryDelay = Math.min(this.#retryDelay * 2, 10000);
            }
        };
        ws.onerror = () => ws.close();
    }
}

class DemoGamepadSource {
    #config;
    #raf;
    constructor(config) {
        this.#config = config;
        this.#raf = null;
    }
    start() {
        const DEMO_DIGITAL_PULSE_CHANCE = 0.992;
        const leftButtonHoldFrames = {up: 0, left: 0, down: 0, right: 0};
        const rightButtonHoldFrames = {up: 0, left: 0, down: 0, right: 0};
        const cornerButtonHoldFrames = {LB: 0, RB: 0, SELECT: 0, START: 0};
        const stickClickHoldFrames = {LS: 0, RS: 0};
        const leftStickState = {x: 0, y: 0, targetX: 0, targetY: 0};
        const rightStickState = {x: 0, y: 0, targetX: 0, targetY: 0};
        const triggerState = {left: {value: 0, target: 0}, right: {value: 0, target: 0}};

        const holdPulse = (holdMap, key, chance = 0.985) => {
            if (holdMap[key] > 0) {
                holdMap[key] -= 1;
                return 1;
            }
            if (Math.random() > chance) {
                holdMap[key] = 12 + Math.floor(Math.random() * 16);
                return 1;
            }
            return 0;
        };

        const tick = () => {
            if (Math.random() > 0.96) {
                leftStickState.targetX = Math.random() * 2 - 1;
                leftStickState.targetY = Math.random() * 2 - 1;
                rightStickState.targetX = Math.random() * 2 - 1;
                rightStickState.targetY = Math.random() * 2 - 1;
            }
            leftStickState.x += (leftStickState.targetX - leftStickState.x) * 0.09;
            leftStickState.y += (leftStickState.targetY - leftStickState.y) * 0.09;
            rightStickState.x += (rightStickState.targetX - rightStickState.x) * 0.09;
            rightStickState.y += (rightStickState.targetY - rightStickState.y) * 0.09;

            if (Math.random() > 0.965) {
                triggerState.left.target = Math.random();
                triggerState.right.target = Math.random();
            }
            triggerState.left.value += (triggerState.left.target - triggerState.left.value) * 0.14;
            triggerState.right.value += (triggerState.right.target - triggerState.right.value) * 0.14;

            this.#config.onState({
                LX: leftStickState.x,
                LY: leftStickState.y,
                RX: rightStickState.x,
                RY: rightStickState.y,
                LT: triggerState.left.value,
                RT: triggerState.right.value,
                LS: holdPulse(stickClickHoldFrames, "LS", DEMO_DIGITAL_PULSE_CHANCE),
                RS: holdPulse(stickClickHoldFrames, "RS", DEMO_DIGITAL_PULSE_CHANCE),
                A: holdPulse(rightButtonHoldFrames, "down"),
                B: holdPulse(rightButtonHoldFrames, "right"),
                X: holdPulse(rightButtonHoldFrames, "left"),
                Y: holdPulse(rightButtonHoldFrames, "up"),
                SELECT: holdPulse(cornerButtonHoldFrames, "SELECT", DEMO_DIGITAL_PULSE_CHANCE),
                START: holdPulse(cornerButtonHoldFrames, "START", DEMO_DIGITAL_PULSE_CHANCE),
                LB: holdPulse(cornerButtonHoldFrames, "LB", DEMO_DIGITAL_PULSE_CHANCE),
                RB: holdPulse(cornerButtonHoldFrames, "RB", DEMO_DIGITAL_PULSE_CHANCE),
                DX: holdPulse(leftButtonHoldFrames, "right") - holdPulse(leftButtonHoldFrames, "left"),
                DY: holdPulse(leftButtonHoldFrames, "down") - holdPulse(leftButtonHoldFrames, "up"),
            });
            this.#raf = requestAnimationFrame(tick);
        };
        this.#raf = requestAnimationFrame(tick);
    }
    stop() {
        if (this.#raf != null) {
            cancelAnimationFrame(this.#raf);
        }
    }
}

class GamepadOverlayRenderer {
    #overlay;
    #lastAppliedState;
    #queuedState;
    #renderScheduled;
    #deadzone;
    #digitalBindings;

    constructor({overlay, deadzoneMode = "none", fixedDeadzone = 0.0}) {
        this.#overlay = overlay;
        this.#deadzone = {deadzoneMode, fixedDeadzone};
        this.#lastAppliedState = null;
        this.#queuedState = null;
        this.#renderScheduled = false;

        this.leftStick = overlay.entities.left.entities.analogStick;
        this.rightStick = overlay.entities.right.entities.analogStick;
        this.leftStickControl = overlay.entities.left.entities.analogStickControl ?? this.leftStick;
        this.rightStickControl = overlay.entities.right.entities.analogStickControl ?? this.rightStick;
        this.leftTrigger = overlay.entities.left.entities.leftTrigger;
        this.rightTrigger = overlay.entities.right.entities.rightTrigger;

        this.#digitalBindings = [
            [overlay.entities.right.entities.downButton, (state) => state.A],
            [overlay.entities.right.entities.rightButton, (state) => state.B],
            [overlay.entities.right.entities.leftButton, (state) => state.X],
            [overlay.entities.right.entities.upButton, (state) => state.Y],
            [overlay.entities.left.entities.select, (state) => state.SELECT],
            [overlay.entities.right.entities.start, (state) => state.START],
            [overlay.entities.guide, (state) => state.GUIDE],
            [overlay.entities.left.entities.leftBumper, (state) => state.LB],
            [overlay.entities.right.entities.rightBumper, (state) => state.RB],
            [this.leftStickControl, (state) => state.LS],
            [this.rightStickControl, (state) => state.RS],
            [overlay.entities.left.entities.leftButton, (state) => Math.max(0, -state.DX)],
            [overlay.entities.left.entities.rightButton, (state) => Math.max(0, state.DX)],
            [overlay.entities.left.entities.upButton, (state) => Math.max(0, -state.DY)],
            [overlay.entities.left.entities.downButton, (state) => Math.max(0, state.DY)],
        ];

        this.leftStickControl?.bringLayersToFront?.();
        this.rightStickControl?.bringLayersToFront?.();

        this.applyNormalizedState(normalizeGamepadState(createEmptyGamepadState(), this.#deadzone));
    }

    applyState(partialState) {
        const state = normalizeGamepadState(partialState, this.#deadzone);
        if (!this.#hasMeaningfulChange(state)) {
            return false;
        }
        this.#queuedState = state;
        if (!this.#renderScheduled) {
            this.#renderScheduled = true;
            requestAnimationFrame(() => {
                this.#renderScheduled = false;
                if (this.#queuedState) {
                    this.applyNormalizedState(this.#queuedState);
                    this.#queuedState = null;
                }
            });
        }
        return true;
    }

    applyNormalizedState(state) {
        this.#lastAppliedState = state;

        this.#applyStickPositions(state);
        this.#applyAnalogPressAmounts(state);
        this.#applyDigitalInputs(state);
    }

    #applyStickPositions(state) {
        this.leftStick?.setTranslation(clampToEllipseInput({
            offset: {x: state.LX, y: state.LY},
            halfSize: this.#overlay.leftAnalogClampHalfSize,
        }));
        this.rightStick?.setTranslation(clampToEllipseInput({
            offset: {x: state.RX, y: state.RY},
            halfSize: this.#overlay.rightAnalogClampHalfSize,
        }));
    }

    #applyAnalogPressAmounts(state) {
        this.leftTrigger?.setInputAmount(state.LT);
        this.rightTrigger?.setInputAmount(state.RT);
    }

    #applyDigitalInputs(state) {
        for (const [entity, getValue] of this.#digitalBindings) {
            entity?.setPressAmount(getValue(state));
        }
    }

    #hasMeaningfulChange(nextState) {
        if (this.#lastAppliedState == null) {
            return true;
        }
        for (const [key, value] of Object.entries(nextState)) {
            if (value !== this.#lastAppliedState[key]) {
                return true;
            }
        }
        return false;
    }
}

function createGamepadSource({
    mode,
    wsUrl,
    pollMs,
    padIndex = null,
    padIdContains = "",
    padRequireStandard = true,
    onState,
}) {
    const config = {wsUrl, pollMs, padIndex, padIdContains, padRequireStandard, onState};
    if (mode === "websocket") {
        return new WebSocketGamepadSource(config);
    }
    if (mode === "demo") {
        return new DemoGamepadSource(config);
    }
    return new BrowserGamepadSource(config);
}

window.OverlayInput = Object.freeze({
    createGamepadSource,
    GamepadOverlayRenderer,
});
})();
