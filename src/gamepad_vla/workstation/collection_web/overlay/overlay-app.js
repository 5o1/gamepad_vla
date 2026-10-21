(() => {
    const CSS_DEFAULTS = Object.freeze({
        "--btn-released": "rgb(44, 47, 51)",
        "--btn-pressed": "rgb(63, 140, 255)",
        "--btn-analog-area": "rgb(0, 0, 0)",
        "--btn-released-default-alpha": "0.8",
        "--btn-pressed-default-alpha": "1",
        "--btn-analog-area-default-alpha": "0.925",
        "--overlay-border-inner-size": "4",
        "--overlay-border-outer-size": "4",
    });

    function applyCssDefaults(element = document.documentElement) {
        const styles = getComputedStyle(element);
        for (const [name, value] of Object.entries(CSS_DEFAULTS)) {
            const existing = styles.getPropertyValue(name).trim();
            if (!existing) {
                element.style.setProperty(name, value);
            }
        }
    }

    window.OverlayTheme = Object.freeze({
        CSS_DEFAULTS,
        applyCssDefaults,
        normalizeButtonColorVars: OverlayCore.normalizeButtonColorVars,
    });
})();

(() => {
    const DEFAULTS = Object.freeze({
        digitalThreshold: 20,
        topLeftX: 0,
        topLeftY: 0,
        wsPort: 8765,
        pollHz: 240,
        blur: 0,
        blurReferenceScale: 2,
    });

    function parseInputSource(query) {
        const source = (query.get("source") || "websocket").toLowerCase();
        return ["browser", "websocket", "demo"].includes(source)
            ? source
            : "browser";
    }

    function queryNumber(query, key, fallback) {
        const value = Number.parseFloat(query.get(key) || "");
        return Number.isFinite(value) ? value : fallback;
    }

    function queryInt(query, key, fallback) {
        const value = Number.parseInt(query.get(key) || "", 10);
        return Number.isFinite(value) ? value : fallback;
    }

    function cssNumber(element, propertyName) {
        const value = Number.parseFloat(
            getComputedStyle(element).getPropertyValue(propertyName)
        );
        return Number.isFinite(value) ? value : null;
    }

    function resolveWebSocketUrl(query) {
        const explicitHost = (query.get("wsHost") || "").trim();
        if (explicitHost) {
            return `ws://${explicitHost}:${DEFAULTS.wsPort}/gamepad-overlay`;
        }

        if (
            window.location.protocol === "http:"
            || window.location.protocol === "https:"
        ) {
            const wsProtocol = (
                window.location.protocol === "https:" ? "wss:" : "ws:"
            );
            return `${wsProtocol}//${window.location.host}/gamepad-overlay`;
        }

        return `ws://localhost:${DEFAULTS.wsPort}/gamepad-overlay`;
    }

    function createSource({query, mode, renderer}) {
        const pollHz = Math.max(1, queryInt(query, "pollHz", DEFAULTS.pollHz));
        return OverlayInput.createGamepadSource({
            mode,
            wsUrl: resolveWebSocketUrl(query),
            pollMs: Math.max(1, Math.round(1000 / pollHz)),
            padIndex:
                query.get("padIndex") == null
                    ? null
                    : Math.max(0, Number.parseInt(query.get("padIndex"), 10) || 0),
            padIdContains: (query.get("padIdContains") || "").trim(),
            padRequireStandard: query.get("padAllowAll") !== "1",
            onState: (state) => renderer.applyState(state),
        });
    }

    function sanitizeProfileName(value) {
        const normalized = String(value || "").trim().toLowerCase();
        if (!normalized) {
            return null;
        }
        return /^[a-z0-9_-]+$/.test(normalized) ? normalized : null;
    }

    function attachThemeStylesheet(themeName) {
        const existing = document.getElementById("overlay-theme-css");
        if (existing) {
            existing.remove();
        }
        const link = document.createElement("link");
        link.id = "overlay-theme-css";
        link.rel = "stylesheet";
        link.href = `overlay-theme-${themeName}.css`;
        document.head.appendChild(link);
        return new Promise((resolve) => {
            link.addEventListener("load", () => resolve(true), {once: true});
            link.addEventListener("error", () => resolve(false), {once: true});
        });
    }

    async function loadThemeCss(themeName) {
        const requestedTheme = sanitizeProfileName(themeName);
        if (requestedTheme == null) {
            console.error(`[theme] Invalid theme '${String(themeName)}'; allowed chars: a-z, 0-9, _, -. Falling back to 'xbox'.`);
            await attachThemeStylesheet("xbox");
            return;
        }

        const loadedRequestedTheme = await attachThemeStylesheet(requestedTheme);
        if (loadedRequestedTheme) {
            return;
        }

        console.error(`[theme] Could not load overlay-theme-${requestedTheme}.css; falling back to 'xbox'.`);
        const loadedFallbackTheme = await attachThemeStylesheet("xbox");
        if (!loadedFallbackTheme) {
            console.error("[theme] Could not load fallback overlay-theme-xbox.css.");
        }
    }

    function attachLayoutScript(layoutName) {
        const existing = document.getElementById("overlay-layout-script");
        if (existing) {
            existing.remove();
        }
        delete window.OverlayLayoutProfile;
        const script = document.createElement("script");
        script.id = "overlay-layout-script";
        script.src = `overlay-layout-${layoutName}.js`;
        document.head.appendChild(script);
        return new Promise((resolve) => {
            script.addEventListener("load", () => resolve(true), {once: true});
            script.addEventListener("error", () => resolve(false), {once: true});
        });
    }

    function normalizeTriggerMode(value, fallback = "analog") {
        return ["analog", "digital", "none"].includes(value) ? value : fallback;
    }

    function resolveLayoutProfile(profile) {
        const model = profile?.model ?? {};
        const controls = profile?.controls ?? {};
        const defaultTheme = sanitizeProfileName(profile?.defaultTheme) ?? "xbox";
        const defaultBlur = Number.isFinite(Number(profile?.defaultBlur))
            ? Number(profile.defaultBlur)
            : null;
        const modelDefaults = OverlaySpec.MODEL_DEFAULTS;
        const buttonLength = Math.max(1, Number(model.buttonLength) || modelDefaults.buttonLength);
        const buttonWidth = Math.max(1, Number(model.buttonWidth) || modelDefaults.buttonWidth);
        const gap = Number.isFinite(Number(model.gap)) ? Number(model.gap) : modelDefaults.gap;
        const betweenHalvesGap = Number.isFinite(Number(model.betweenHalvesGap))
            ? Number(model.betweenHalvesGap)
            : modelDefaults.betweenHalvesGap;
        const defaultInnerBorderSize = (Number(modelDefaults.innerBorderSize) || Number(modelDefaults.borderWidth) / 2);
        const borderInnerSize = Math.max(0, Number(model.borderInnerSize) || defaultInnerBorderSize);
        const borderOuterSize = Math.max(0, Number(model.borderOuterSize) || defaultInnerBorderSize);
        const analogStickRingPercent = Math.max(0, Number(model.analogStickRingPercent) || modelDefaults.analogStickRingPercent);
        const leftDpadOriginRingPercent = Math.max(0, Number(model.leftDpadOriginRingPercent) || modelDefaults.leftDpadOriginRingPercent);
        return {
            defaultTheme,
            defaultBlur,
            model: {
                buttonLength,
                buttonWidth,
                gap,
                betweenHalvesGap,
                borderInnerSize,
                borderOuterSize,
                analogStickRingPercent,
                leftDpadOriginRingPercent,
            },
            controls: {
                hasAnalogStick: controls.hasAnalogStick !== false,
                leftTriggerMode: normalizeTriggerMode(controls.leftTriggerMode, "analog"),
                rightTriggerMode: normalizeTriggerMode(controls.rightTriggerMode, "analog"),
                drawLeftOriginRingWithoutStick: controls.drawLeftOriginRingWithoutStick !== false,
                digitalThreshold: Math.max(0, Math.min(100, Number(controls.digitalThreshold) || DEFAULTS.digitalThreshold)),
            },
        };
    }

    async function loadLayoutProfile(layoutName) {
        const requestedLayout = sanitizeProfileName(layoutName);
        if (requestedLayout == null) {
            console.error(`[layout] Invalid layout '${String(layoutName)}'; allowed chars: a-z, 0-9, _, -. Falling back to 'xbox'.`);
            await attachLayoutScript("xbox");
            return resolveLayoutProfile(window.OverlayLayoutProfile);
        }

        const loadedRequestedLayout = await attachLayoutScript(requestedLayout);
        if (loadedRequestedLayout) {
            return resolveLayoutProfile(window.OverlayLayoutProfile);
        }

        console.error(`[layout] Could not load overlay-layout-${requestedLayout}.js; falling back to 'xbox'.`);
        const loadedFallbackLayout = await attachLayoutScript("xbox");
        if (!loadedFallbackLayout) {
            console.error("[layout] Could not load fallback overlay-layout-xbox.js.");
        }
        return resolveLayoutProfile(window.OverlayLayoutProfile);
    }

    function createSharedModel(layoutProfile) {
        const borderWidth = layoutProfile.model.borderInnerSize + layoutProfile.model.borderOuterSize;
        return OverlaySpec.createOverlayModel({
            buttonLength: layoutProfile.model.buttonLength,
            buttonWidth: layoutProfile.model.buttonWidth,
            borderWidth,
            innerBorderSize: layoutProfile.model.borderInnerSize,
            gap: layoutProfile.model.gap,
            betweenHalvesGap: layoutProfile.model.betweenHalvesGap,
            leftTriggerMode: layoutProfile.controls.leftTriggerMode,
            rightTriggerMode: layoutProfile.controls.rightTriggerMode,
            analogStickRingPercent: layoutProfile.model.analogStickRingPercent,
            leftDpadOriginRingPercent: layoutProfile.model.leftDpadOriginRingPercent,
        });
    }

    function applyLayoutCssOverrides(layoutProfile, element = document.documentElement) {
        element.style.setProperty("--overlay-border-inner-size", String(layoutProfile.model.borderInnerSize));
        element.style.setProperty("--overlay-border-outer-size", String(layoutProfile.model.borderOuterSize));
    }

    function resolveDefaultBlur(layoutProfile, element = document.documentElement) {
        const themeBlur = cssNumber(element, "--overlay-default-blur");
        if (themeBlur != null) {
            return Math.max(0, themeBlur);
        }
        if (layoutProfile.defaultBlur != null) {
            return Math.max(0, layoutProfile.defaultBlur);
        }
        return DEFAULTS.blur;
    }

    async function init({svgId}) {
        const query = new URLSearchParams(window.location.search);
        const layout = (query.get("layout") || "xbox").toLowerCase();
        const background = query.get("background");
        if (background != null && background.trim() !== "") {
            document.body.style.background = background;
        }
        const layoutProfile = await loadLayoutProfile(layout);
        const hasExplicitTheme = query.has("theme");
        const theme = hasExplicitTheme
            ? (query.get("theme") || "")
            : layoutProfile.defaultTheme;
        const digitalThresholdPct = query.has("digitalThreshold")
            ? queryNumber(query, "digitalThreshold", layoutProfile.controls.digitalThreshold)
            : layoutProfile.controls.digitalThreshold;
        const digitalThreshold = Math.max(0, Math.min(1, digitalThresholdPct / 100));
        await loadThemeCss(theme);
        OverlayTheme.applyCssDefaults(document.documentElement);
        OverlayTheme.normalizeButtonColorVars(document.documentElement);
        applyLayoutCssOverrides(layoutProfile, document.documentElement);
        const defaultBlur = resolveDefaultBlur(
            layoutProfile,
            document.documentElement,
        );
        const model = createSharedModel(layoutProfile);
        const context = new OverlayRenderer.SvgContext(
            document.getElementById(svgId)
        );
        const overlay = new OverlayRenderer.GamepadOverlay({
            context,
            model,
            topLeft: new OverlayCore.Vector2({
                x: DEFAULTS.topLeftX,
                y: DEFAULTS.topLeftY,
            }),
            digitalThreshold,
            prewarmPressFillVisuals: true,
            themeVariables: {},
            hasAnalogStick: layoutProfile.controls.hasAnalogStick,
            drawLeftOriginRingWithoutStick: layoutProfile.controls.drawLeftOriginRingWithoutStick,
        });

        const overlayRegion = overlay.region;
        context.svg.setAttribute(
            "viewBox",
            `${overlayRegion.topLeft.x} ${overlayRegion.topLeft.y} ${overlay.width} ${overlay.height}`
        );
        context.svg.setAttribute("preserveAspectRatio", "xMidYMid meet");
        context.svg.style.setProperty("shape-rendering", "geometricPrecision");

        const blur = queryNumber(query, "blur", defaultBlur);
        if (blur > 0) {
            const filter = context.addFilter("overlayBlur", blur);
            const blurNode = filter.querySelector("feGaussianBlur");
            const rootGroup = document.getElementById("gamepadOverlayGroup");
            if (rootGroup && blurNode) {
                rootGroup.setAttribute("filter", `url(#${filter.id})`);

                function overlayScale() {
                    const svgRect = context.svg.getBoundingClientRect();
                    const scaleX = svgRect.width / overlay.width;
                    const scaleY = svgRect.height / overlay.height;
                    return Math.min(scaleX, scaleY) || 1;
                }

                function updateBlur() {
                    const scale = overlayScale();
                    blurNode.setAttribute(
                        "stdDeviation",
                        String((blur * DEFAULTS.blurReferenceScale) / scale)
                    );
                }

                updateBlur();
                window.requestAnimationFrame(updateBlur);
                if (typeof ResizeObserver === "function") {
                    new ResizeObserver(updateBlur).observe(context.svg);
                } else {
                    window.addEventListener("resize", updateBlur);
                }
            }
        }

        const renderer = new OverlayInput.GamepadOverlayRenderer({
            overlay,
            deadzoneMode: "none",
            fixedDeadzone: 0,
        });
        const source = createSource({
            query,
            mode: parseInputSource(query),
            renderer,
        });
        source.start();
    }

    window.OverlayApp = Object.freeze({
        DEFAULTS,
        init,
    });
})();
