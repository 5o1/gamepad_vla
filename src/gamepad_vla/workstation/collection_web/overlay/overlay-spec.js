(() => {
const OverlayModelCore = OverlayCore;

const Vector2 = OverlayCore.Vector2;
const Region = OverlayCore.Region;

class OverlayShapeModel {
    #region;
    #shapeType;
    #cornerRadiusPercent;

    constructor({region, shapeType, cornerRadiusPercent = 0}) {
        this.#region = region;
        this.#shapeType = shapeType;
        this.#cornerRadiusPercent = this.#normalizeCornerRadiusPercent(cornerRadiusPercent);
    }

    get region() { return this.#region; }
    get shapeType() { return this.#shapeType; }
    get cornerRadiusPercent() { return this.#cornerRadiusPercent; }

    #normalizeCornerRadiusPercent(value) {
        if (value == null) {
            return Vector2.ZERO;
        }
        if (value instanceof Vector2) {
            return value;
        }
        if (typeof value === "number") {
            return Vector2.splat(value);
        }
        const x = Number(value.x);
        const y = Number(value.y);
        if (Number.isFinite(x) && Number.isFinite(y)) {
            return new Vector2({x, y});
        }
        return Vector2.ZERO;
    }

    expanded(amount) {
        if (!(amount > 0)) {
            return this;
        }
        const expandedRegion = Region.fromCenter({
            center: this.#region.center,
            size: this.#region.size.clone().add(Vector2.splat(amount * 2)),
        });
        let cornerRadiusPercent = this.#cornerRadiusPercent;
        if (this.#shapeType === "rect") {
            const originalRadiusX = this.#region.halfSize.x * this.#cornerRadiusPercent.x;
            const originalRadiusY = this.#region.halfSize.y * this.#cornerRadiusPercent.y;
            const expandedRadiusX = Math.min(expandedRegion.halfSize.x, Math.max(0, originalRadiusX + amount));
            const expandedRadiusY = Math.min(expandedRegion.halfSize.y, Math.max(0, originalRadiusY + amount));
            cornerRadiusPercent = new Vector2({
                x: expandedRegion.halfSize.x === 0 ? 0 : expandedRadiusX / expandedRegion.halfSize.x,
                y: expandedRegion.halfSize.y === 0 ? 0 : expandedRadiusY / expandedRegion.halfSize.y,
            });
        }
        return new OverlayShapeModel({
            region: expandedRegion,
            shapeType: this.#shapeType,
            cornerRadiusPercent,
        });
    }

    trianglePoints() {
        const shapeType = this.#shapeType;
        const region = this.#region;
        if (shapeType === "triUp") {
            return [region.topCenter, region.bottomLeft, region.bottomRight];
        }
        if (shapeType === "triDown") {
            return [region.bottomCenter, region.topLeft, region.topRight];
        }
        if (shapeType === "triLeft") {
            return [region.centerLeft, region.topRight, region.bottomRight];
        }
        if (shapeType === "triRight") {
            return [region.centerRight, region.topLeft, region.bottomLeft];
        }
        return null;
    }
}

class OverlayBorderModel {
    #innerSize;

    constructor({innerSize = 0} = {}) {
        this.#innerSize = Math.max(0, Number(innerSize) || 0);
    }

    get innerSize() { return this.#innerSize; }
    get width() { return this.#innerSize; }
    get halfWidth() { return this.#innerSize / 2; }

    appliesToBordered(includeBorder) {
        return Boolean(includeBorder) && this.#innerSize > 0;
    }

    expandAmount() {
        return this.#innerSize / 2;
    }

    expandedShape(shapeModel, includeBorder) {
        if (!this.appliesToBordered(includeBorder)) {
            return shapeModel;
        }
        return shapeModel.expanded(this.expandAmount());
    }
}

const MODEL_DEFAULTS = Object.freeze({
    buttonLength: 132,
    buttonWidth: 132,
    borderWidth: 8,
    innerBorderSize: null,
    gap: 1,
    betweenHalvesGap: 0,
    leftTriggerMode: "analog",
    rightTriggerMode: "analog",
    analogStickRingPercent: 70,
    leftDpadOriginRingPercent: 70,
});

function createOverlayModel(config = {}) {
    const {
        buttonLength,
        buttonWidth,
        borderWidth,
        innerBorderSize,
        gap,
        betweenHalvesGap,
        leftTriggerMode,
        rightTriggerMode,
        analogStickRingPercent,
        leftDpadOriginRingPercent,
    } = {...MODEL_DEFAULTS, ...config};
    const resolvedInnerBorderSize = Math.max(0, Number(innerBorderSize) || (Number(borderWidth) || 0) / 2);
    const topLeft = new OverlayModelCore.Vector2({x: 0, y: 0});
    const gapPixels = gap * borderWidth;
    const betweenHalvesGapPixels = betweenHalvesGap * borderWidth;
    const leftTopLeft = topLeft.clone().add(OverlayModelCore.Vector2.splat(borderWidth));
    const leftLayout = new OverlayModelCore.DpadLayout({buttonLength, buttonWidth, topLeft: leftTopLeft});
    const rightLayout = new OverlayModelCore.DpadLayout({
        buttonLength,
        buttonWidth,
        topLeft: leftTopLeft.clone().add(new OverlayModelCore.Vector2({
            x: leftLayout.size.x + borderWidth * 2 + gapPixels + betweenHalvesGapPixels,
            y: 0,
        })),
    });

    const width = rightLayout.topRight.bottomRight.x - topLeft.x + borderWidth;
    const height = leftLayout.bottomLeft.bottomRight.y - topLeft.y + borderWidth;

    const scaledRegion = (region, scaleX, scaleY) => OverlayModelCore.Region.fromCenter({center: region.center, size: new OverlayModelCore.Vector2({x: region.size.x * scaleX, y: region.size.y * scaleY})});
    const cornerCompensation = gapPixels + borderWidth * 2;
    const applyCornerCompensation = (region, regionName, compensation, compensateOuterEdges = true) => {
        const x = Math.max(0, compensation.x);
        const y = Math.max(0, compensation.y);
        const insets = {top: 0, right: 0, bottom: 0, left: 0};
        switch (regionName) {
            case "topLeft":
                insets.right += x;
                insets.bottom += y;
                if (compensateOuterEdges) { insets.left += x; insets.top += y; }
                break;
            case "topRight":
                insets.left += x;
                insets.bottom += y;
                if (compensateOuterEdges) { insets.right += x; insets.top += y; }
                break;
            case "bottomLeft":
                insets.right += x;
                insets.top += y;
                if (compensateOuterEdges) { insets.left += x; insets.bottom += y; }
                break;
            case "bottomRight":
                insets.left += x;
                insets.top += y;
                if (compensateOuterEdges) { insets.right += x; insets.bottom += y; }
                break;
            default:
                break;
        }
        return new OverlayModelCore.Region({
            topLeft: region.topLeft.clone().add({x: insets.left, y: insets.top}),
            size: new OverlayModelCore.Vector2({
                x: Math.max(0, region.size.x - insets.left - insets.right),
                y: Math.max(0, region.size.y - insets.top - insets.bottom),
            }),
        });
    };
    const cornerButtonRegion = (layout, regionName, scaleX, scaleY) => {
        const compensated = applyCornerCompensation(
            layout[regionName],
            regionName,
            OverlayModelCore.Vector2.splat(cornerCompensation),
            true
        );
        return scaledRegion(compensated, scaleX, scaleY);
    };

    const analogStickRingScale = Math.max(0, Number(analogStickRingPercent) || 70) / 100;
    const leftOriginRingScale = Math.max(0, Number(leftDpadOriginRingPercent) || 70) / 100;

    const buttons = {
        left: {
            up: {region: leftLayout.up, shape: "rect", pressMode: "digital", cornerRadiusPercent: 0},
            right: {region: leftLayout.right, shape: "rect", pressMode: "digital", cornerRadiusPercent: 0},
            down: {region: leftLayout.down, shape: "rect", pressMode: "digital", cornerRadiusPercent: 0},
            left: {region: leftLayout.left, shape: "rect", pressMode: "digital", cornerRadiusPercent: 0},
            origin: {region: leftLayout.origin, shape: "rect", pressMode: "digital", cornerRadiusPercent: 0},
            leftBumper: {region: cornerButtonRegion(leftLayout, "topLeft", 1.0, 0.7), shape: "rect", pressMode: "digital", cornerRadiusPercent: 0.25},
            select: {region: cornerButtonRegion(leftLayout, "topRight", 0.7, 0.7), shape: "ellipse", pressMode: "digital"},
            leftTrigger: leftTriggerMode === "none"
                ? null
                : {region: cornerButtonRegion(leftLayout, "bottomLeft", 1.0, 1.0), shape: "triDown", pressMode: leftTriggerMode, pressFillDirection: "down"},
            analogArea: {region: OverlayModelCore.Region.fromCenter({center: leftLayout.analogRegion.center, size: leftLayout.analogRegion.size.clone()}), shape: "ellipse", pressMode: "none"},
            analogStick: {
                region: OverlayModelCore.Region.fromCenter({
                    center: leftLayout.analogRegion.center,
                    size: leftLayout.analogRegion.size.clone().multiply(OverlayModelCore.Vector2.splat(0.65)),
                }),
                shape: "ellipse",
                pressMode: "digital",
                includeOuterBorder: true,
            },
            analogStickRing: {
                region: OverlayModelCore.Region.fromCenter({
                    center: leftLayout.analogRegion.center,
                    size: leftLayout.analogRegion.size.clone().multiply(OverlayModelCore.Vector2.splat(0.65 * analogStickRingScale)),
                }),
                shape: "ellipse",
                pressMode: "none",
                includeOuterBorder: true,
            },
            originRingPercent: leftOriginRingScale * 100,
        },
        right: {
            up: {region: rightLayout.up, shape: "ellipse", pressMode: "digital"},
            right: {region: rightLayout.right, shape: "ellipse", pressMode: "digital"},
            down: {region: rightLayout.down, shape: "ellipse", pressMode: "digital"},
            left: {region: rightLayout.left, shape: "ellipse", pressMode: "digital"},
            origin: null,
            start: {region: cornerButtonRegion(rightLayout, "topLeft", 0.7, 0.7), shape: "ellipse", pressMode: "digital"},
            rightBumper: {region: cornerButtonRegion(rightLayout, "topRight", 1.0, 0.7), shape: "rect", pressMode: "digital", cornerRadiusPercent: 0.25},
            rightTrigger: rightTriggerMode === "none"
                ? null
                : {region: cornerButtonRegion(rightLayout, "bottomRight", 1.0, 1.0), shape: "triDown", pressMode: rightTriggerMode, pressFillDirection: "down"},
            analogArea: {region: OverlayModelCore.Region.fromCenter({center: rightLayout.analogRegion.center, size: rightLayout.analogRegion.size.clone()}), shape: "ellipse", pressMode: "none"},
            analogStick: {
                region: OverlayModelCore.Region.fromCenter({
                    center: rightLayout.analogRegion.center,
                    size: rightLayout.analogRegion.size.clone().multiply(OverlayModelCore.Vector2.splat(0.65)),
                }),
                shape: "ellipse",
                pressMode: "digital",
                includeOuterBorder: true,
            },
            analogStickRing: {
                region: OverlayModelCore.Region.fromCenter({
                    center: rightLayout.analogRegion.center,
                    size: rightLayout.analogRegion.size.clone().multiply(OverlayModelCore.Vector2.splat(0.65 * analogStickRingScale)),
                }),
                shape: "ellipse",
                pressMode: "none",
                includeOuterBorder: true,
            },
        },
    };

    const state = {
        A: 0, B: 0, X: 0, Y: 0, SELECT: 0, START: 0, LB: 0, RB: 0, LS: 0, RS: 0,
        LX: 0, LY: 0, RX: 0, RY: 0, LT: 0, RT: 0, DX: 0, DY: 0,
    };

    return {
        borderWidth,
        innerBorderSize: resolvedInnerBorderSize,
        leftLayout,
        rightLayout,
        width,
        height,
        buttons,
        state,
    };
}

window.OverlaySpec = Object.freeze({
    MODEL_DEFAULTS,
    ShapeModel: OverlayShapeModel,
    BorderModel: OverlayBorderModel,
    createOverlayModel,
});
})();
