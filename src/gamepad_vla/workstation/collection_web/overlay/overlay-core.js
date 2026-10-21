(() => {
const OVERLAY_ASSERTION_ENABLE = true;

function requireFiniteNumber(value, name) {
    if (Number.isFinite(value)) {
        return value;
    }
    const type = typeof value;
    const got = value?.constructor?.name ?? type;
    if (type !== "number") {
        throw new TypeError(`${name} must be a number; got ${got}`);
    }
    throw new RangeError(`${name} must be a finite number; got ${value}`);
}

function assertFiniteNumber(value, name) {
    if (!OVERLAY_ASSERTION_ENABLE) {
        return value;
    }
    return requireFiniteNumber(value, name);
}

function requireInstanceOf(value, ctor, name) {
    if (value instanceof ctor) {
        return value;
    }
    const got = value?.constructor?.name ?? typeof value;
    throw new TypeError(`${name} must be an instance of ${ctor.name}; got ${got}`);
}

function assertInstanceOf(value, ctor, name) {
    if (!OVERLAY_ASSERTION_ENABLE) {
        return value;
    }
    return requireInstanceOf(value, ctor, name);
}

function unreachable(message = "Unreachable") {
    const error = new Error(message);
    error.name = "AssertionError";
    throw error;
}

class Vector2 {
    static ZERO = Object.freeze(this.splat(0));
    static TWO = Object.freeze(this.splat(2));

    static #ADD = 0;
    static #SUBTRACT = 1;
    static #MULTIPLY = 2;
    static #DIVIDE = 3;

    static require(value, name) {
        return requireInstanceOf(value, this, name);
    }
    static assert(value, name) {
        return assertInstanceOf(value, this, name);
    }
    static splat(value) {
        return new this({x: value, y: value});
    }

    #x;
    #y;
    constructor({x, y} = {}) {
        this.set({x, y});
    }
    set x(value) { this.#x = assertFiniteNumber(value, "x"); }
    set y(value) { this.#y = assertFiniteNumber(value, "y"); }
    get x() { return this.#x; }
    get y() { return this.#y; }
    set({x, y} = {}) { this.x = x; this.y = y; return this; }
    update({x, y} = {}) { if (x != null) { this.x = x; } if (y != null) { this.y = y; } return this; }
    add({x, y} = {}) { this.#applyComponent(Vector2.#ADD, true, x, 0); this.#applyComponent(Vector2.#ADD, false, y, 0); return this; }
    subtract({x, y} = {}) { this.#applyComponent(Vector2.#SUBTRACT, true, x, 0); this.#applyComponent(Vector2.#SUBTRACT, false, y, 0); return this; }
    multiply({x, y} = {}) { this.#applyComponent(Vector2.#MULTIPLY, true, x, 1); this.#applyComponent(Vector2.#MULTIPLY, false, y, 1); return this; }
    divide({x, y} = {}) { this.#applyComponent(Vector2.#DIVIDE, true, x, 1); this.#applyComponent(Vector2.#DIVIDE, false, y, 1); return this; }
    clone() { return new this.constructor(this); }
    toString() { return `${this.x},${this.y}`; }
    equals(other) { return other instanceof Vector2 && this.x === other.x && this.y === other.y; }

    #applyComponent(operation, isX, operand, skipValue) {
        if (operand == null || operand === skipValue) {
            return;
        }
        const name = isX ? "x" : "y";
        assertFiniteNumber(operand, name);
        let result = isX ? this.#x : this.#y;
        switch (operation) {
            case Vector2.#ADD: result += operand; break;
            case Vector2.#SUBTRACT: result -= operand; break;
            case Vector2.#MULTIPLY: result *= operand; break;
            case Vector2.#DIVIDE:
                if (operand === 0) { throw new RangeError(`${name} divisor must not be 0`); }
                result /= operand;
                break;
            default:
                unreachable("Invalid operation");
        }
        assertFiniteNumber(result, name);
        if (isX) { this.#x = result; } else { this.#y = result; }
    }
}

class Region {
    static fromCenter({center, size}) {
        const topLeft = center.clone().subtract(size.clone().divide(Vector2.TWO));
        return new this({topLeft, size});
    }

    #topLeft;
    #size;
    #cache = Object.create(null);
    constructor({topLeft, size}) {
        this.#setTopLeft(topLeft);
        this.#setSize(size);
    }
    set topLeft(value) { this.#setTopLeft(value); this.#clearCache(); }
    set size(value) { this.#setSize(value); this.#clearCache(); }
    get topLeft() { return this.#topLeft; }
    get size() { return this.#size; }
    set({topLeft, size}) { this.#setTopLeft(topLeft); this.#setSize(size); this.#clearCache(); return this; }
    update({topLeft, size} = {}) { let changed = false; if (topLeft != null) { this.#setTopLeft(topLeft); changed = true; } if (size != null) { this.#setSize(size); changed = true; } if (changed) { this.#clearCache(); } return this; }
    scale(factor) { return this.update(Region.fromCenter({center: this.center, size: new Vector2({x: this.size.x * factor.x, y: this.size.y * factor.y})})); }
    get halfSize() { return this.#cache.halfSize ??= Object.freeze(this.#size.clone().divide(Vector2.TWO)); }
    get topCenter() { return this.#cache.topCenter ??= Object.freeze(this.#topLeft.clone().add({x: this.halfSize.x})); }
    get topRight() { return this.#cache.topRight ??= Object.freeze(this.#topLeft.clone().add({x: this.#size.x})); }
    get centerLeft() { return this.#cache.centerLeft ??= Object.freeze(this.#topLeft.clone().add({y: this.halfSize.y})); }
    get center() { return this.#cache.center ??= Object.freeze(this.#topLeft.clone().add(this.halfSize)); }
    get centerRight() { return this.#cache.centerRight ??= Object.freeze(this.#topLeft.clone().add({x: this.#size.x, y: this.halfSize.y})); }
    get bottomLeft() { return this.#cache.bottomLeft ??= Object.freeze(this.#topLeft.clone().add({y: this.#size.y})); }
    get bottomCenter() { return this.#cache.bottomCenter ??= Object.freeze(this.#topLeft.clone().add({x: this.halfSize.x, y: this.#size.y})); }
    get bottomRight() { return this.#cache.bottomRight ??= Object.freeze(this.#topLeft.clone().add(this.#size)); }
    clone() { return new this.constructor(this); }
    toString() { return `${this.topLeft} ${this.size.x}x${this.size.y}`; }
    #clearCache() { this.#cache = Object.create(null); }
    #setTopLeft(topLeft) { this.#topLeft = Object.freeze(Vector2.assert(topLeft, "topLeft").clone()); }
    #setSize(size) { this.#size = Object.freeze(Vector2.assert(size, "size").clone()); }
}

class DpadLayout {
    static fromCenter({buttonLength, buttonWidth, center}) {
        return new this({
            buttonLength,
            buttonWidth,
            topLeft: center.clone().subtract(Vector2.splat(buttonLength + buttonWidth / 2)),
        });
    }

    #size;
    #buttonCentersRegion;
    #horizontalButtonSize;
    #verticalButtonSize;
    #cornerButtonSize;
    #originSize;
    #cache = Object.create(null);

    constructor({buttonLength, buttonWidth, topLeft}) {
        this.#size = Object.freeze(Vector2.splat(buttonLength * 2 + buttonWidth));
        this.#buttonCentersRegion = new Region({
            topLeft: topLeft.clone().add(Vector2.splat(buttonLength / 2)),
            size: Vector2.splat(buttonWidth + buttonLength),
        });
        this.#horizontalButtonSize = new Vector2({x: buttonLength, y: buttonWidth});
        this.#verticalButtonSize = new Vector2({x: buttonWidth, y: buttonLength});
        this.#cornerButtonSize = Vector2.splat(buttonLength);
        this.#originSize = Vector2.splat(buttonWidth);
    }
    get size() { return this.#size; }
    get origin() { return this.#cache.origin ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.center, size: this.#originSize})); }
    get left() { return this.#cache.left ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.centerLeft, size: this.#horizontalButtonSize})); }
    get right() { return this.#cache.right ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.centerRight, size: this.#horizontalButtonSize})); }
    get up() { return this.#cache.up ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.topCenter, size: this.#verticalButtonSize})); }
    get down() { return this.#cache.down ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.bottomCenter, size: this.#verticalButtonSize})); }
    get topLeft() { return this.#cache.topLeft ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.topLeft, size: this.#cornerButtonSize})); }
    get bottomLeft() { return this.#cache.bottomLeft ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.bottomLeft, size: this.#cornerButtonSize})); }
    get topRight() { return this.#cache.topRight ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.topRight, size: this.#cornerButtonSize})); }
    get bottomRight() { return this.#cache.bottomRight ??= Object.freeze(Region.fromCenter({center: this.#buttonCentersRegion.bottomRight, size: this.#cornerButtonSize})); }
    get crossPoints() {
        return this.#cache.crossPoints ??= Object.freeze([
            this.left.bottomRight, this.left.bottomLeft, this.left.topLeft,
            this.up.bottomLeft, this.up.topLeft, this.up.topRight,
            this.right.topLeft, this.right.topRight, this.right.bottomRight,
            this.down.topRight, this.down.bottomRight, this.down.bottomLeft,
        ]);
    }
    get analogRegion() {
        return this.#cache.analogRegion ??= Object.freeze(Region.fromCenter({
            center: this.origin.center,
            size: Vector2.splat(Math.hypot(this.origin.size.x, this.origin.size.y)),
        }));
    }
}

function clamp01(value) {
    return Math.max(0, Math.min(1, Number(value) || 0));
}

function clamp11(value) {
    return Math.max(-1, Math.min(1, Number(value) || 0));
}

function clamp255(value) {
    return Math.max(0, Math.min(255, Number(value) || 0));
}

// Resolve a CSS color string (rgb(), rgba(), or "r,g,b[,a]") to an rgba()
// string, applying fallbackAlpha when the source has no explicit alpha.
function parseColorToRgba(value, fallbackAlpha) {
    const raw = String(value || "").trim();
    if (!raw) {
        return null;
    }
    const rgbMatch = raw.match(/^rgb\(\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\)$/i);
    if (rgbMatch) {
        return `rgba(${clamp255(rgbMatch[1])}, ${clamp255(rgbMatch[2])}, ${clamp255(rgbMatch[3])}, ${clamp01(fallbackAlpha)})`;
    }
    const rgbaMatch = raw.match(/^rgba\(\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*,\s*([\d.]+)\s*\)$/i);
    if (rgbaMatch) {
        return `rgba(${clamp255(rgbaMatch[1])}, ${clamp255(rgbaMatch[2])}, ${clamp255(rgbaMatch[3])}, ${clamp01(rgbaMatch[4])})`;
    }
    const commaSeparatedParts = raw.split(",").map((part) => part.trim());
    if (commaSeparatedParts.length === 3) {
        return `rgba(${clamp255(commaSeparatedParts[0])}, ${clamp255(commaSeparatedParts[1])}, ${clamp255(commaSeparatedParts[2])}, ${clamp01(fallbackAlpha)})`;
    }
    if (commaSeparatedParts.length === 4) {
        return `rgba(${clamp255(commaSeparatedParts[0])}, ${clamp255(commaSeparatedParts[1])}, ${clamp255(commaSeparatedParts[2])}, ${clamp01(commaSeparatedParts[3])})`;
    }
    return raw;
}

// Rewrite an element's --btn-released/--btn-pressed custom properties into
// fully-resolved rgba() values, folding in the configured default alphas.
function normalizeButtonColorVars(element) {
    const computedStyle = getComputedStyle(element);
    const releasedDefaultAlpha = Number.parseFloat(computedStyle.getPropertyValue("--btn-released-default-alpha")) || 0.7;
    const pressedDefaultAlpha = Number.parseFloat(computedStyle.getPropertyValue("--btn-pressed-default-alpha")) || 1;
    const releasedResolved = parseColorToRgba(computedStyle.getPropertyValue("--btn-released"), releasedDefaultAlpha);
    const pressedResolved = parseColorToRgba(computedStyle.getPropertyValue("--btn-pressed"), pressedDefaultAlpha);
    if (releasedResolved != null) {
        element.style.setProperty("--btn-released", releasedResolved);
    }
    if (pressedResolved != null) {
        element.style.setProperty("--btn-pressed", pressedResolved);
    }
}

function clampNormalizedOffsetToEllipse({offset, halfSize}) {
    const x = assertFiniteNumber(offset.x, "offset.x");
    const y = assertFiniteNumber(offset.y, "offset.y");
    const halfWidth = Math.max(0, assertFiniteNumber(halfSize.x, "halfSize.x"));
    const halfHeight = Math.max(0, assertFiniteNumber(halfSize.y, "halfSize.y"));
    if (halfWidth === 0 || halfHeight === 0) {
        return Vector2.ZERO;
    }
    const unitDistance = (x * x) + (y * y);
    if (unitDistance <= 1) {
        return new Vector2({x: x * halfWidth, y: y * halfHeight});
    }
    const scale = 1 / Math.sqrt(unitDistance);
    return new Vector2({x: x * scale * halfWidth, y: y * scale * halfHeight});
}

window.OverlayCore = Object.freeze({
    OVERLAY_ASSERTION_ENABLE,
    requireFiniteNumber,
    assertFiniteNumber,
    requireInstanceOf,
    assertInstanceOf,
    unreachable,
    Vector2,
    Region,
    DpadLayout,
    clampNormalizedOffsetToEllipse,
    clamp01,
    clamp11,
    clamp255,
    parseColorToRgba,
    normalizeButtonColorVars,
});
})();
