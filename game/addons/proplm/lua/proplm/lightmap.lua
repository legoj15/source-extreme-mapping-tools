-- proplm: turns the baked lightmap VTF into the form the $detail multiply needs.
--
-- Why: VRAD stores the stock .ppl byte halved, s = p / 2, and p = 255 * 0.5 * L^(1/2.2) (L = light in
-- linear units, see LinearToVertexLight). The stock shaders therefore show albedo * 2p/255 in gamma
-- space, i.e. a linear multiplier X = (4s/255)^2.2. A $detail texture is sampled as linear data and
-- multiplied as 2*D, so feeding the VTF straight in gives X = 2s/255: the wrong curve (measured against
-- real stock renders: 61% of stock brightness where the light is strong, 138% where it is weak).
-- A $basetexture is sampled through the sRGB decode, which supplies the missing power. So each lightmap
-- is drawn once, through a base-texture material with raw (linear) output and a gain, into a 16-bit
-- float render target, and that target is what the instance materials use as $detail.
--
-- 16-bit float so that dark values do not band (the converted values are tiny near black).

PROPLM = PROPLM or {}
local LM = {}
PROPLM.lightmap = LM

-- The material multiplies albedo by X = 1 - f + 2 f GAIN decode(s), f = BLEND ($detailblendfactor). A BLEND
-- above 1 extrapolates the detail multiply, which subtracts a small offset: it cancels the lift the
-- hardware sRGB decode gives to dark values. GAIN and BLEND were fitted to the stock .ppl rendering
-- (pooled stone luminance 1.03 x stock, see docs/proplm.md "Calibration"); SetGain() below is for refitting.
LM.GAIN = 9.8
LM.BLEND = 1.052

local TEXTUREFLAGS_CLAMP = 4 + 8 -- CLAMPS | CLAMPT: charts must not wrap to the opposite edge
local FLOAT_FORMATS = { IMAGE_FORMAT_RGBA16161616F, IMAGE_FORMAT_RGBA8888 }

-- A render target drawn while the map is still loading comes up empty (measured). Each converted
-- lightmap is therefore redrawn on every frame for REFRESH_SECONDS, counted from the first frame
-- rendered after it was created or re-armed (PreRender hook at the bottom; a long load can put seconds
-- between the creation and that frame); after that its contents are stable. The window belongs to the
-- target: creating or looking up one lightmap never redraws the others. A video mode change can empty
-- every target at once; the OnScreenSizeChanged hook and LM.Refresh (called by a reload) cover that.
local REFRESH_SECONDS = 3

local cache = {}  -- lightmap path -> {path, rt, src, refreshUntil, armed} or {path, err, kind}; survives proplm_reload (RTs cannot be freed)
local active = {} -- the cache entries still inside their refresh window (compacted by the PreRender hook)
local serial = 0

local function arm(c)
    c.refreshUntil = nil -- the PreRender hook starts the window at the next frame it sees
    if not c.armed then
        c.armed = true
        active[#active + 1] = c
    end
end

local function drawQuad(c)
    render.Clear(0, 0, 0, 255, false, false)
    render.SetMaterial(c.src)
    render.DrawScreenQuad()
end

-- Draws one converted lightmap into its render target. The target is always popped again, even if the
-- draw raises, so a failure here cannot leave the render target stack unbalanced.
function LM.Draw(c)
    render.PushRenderTarget(c.rt)
    local ok, err = pcall(drawQuad, c)
    render.PopRenderTarget()
    if not ok then
        c.refreshUntil = 0
        if not c.drawFailed then
            c.drawFailed = true
            PROPLM.Msg("lightmap '%s': drawing its render target failed: %s", c.path, tostring(err))
        end
    end
    return ok
end

-- Builds the converted target for the (already cached) entry c; see LM.Get for the return values.
local function create(c)
    local lmPath = c.path
    serial = serial + 1
    local src = CreateMaterial("proplm_cv_" .. serial, "UnlitGeneric", {
        ["$basetexture"] = lmPath,
        ["$linearwrite"] = 1,
        ["$color"] = string.format("[%f %f %f]", LM.GAIN, LM.GAIN, LM.GAIN),
        ["$ignorez"] = 1,
    })
    local tex = src and src:GetTexture("$basetexture")
    if not tex or tex:IsError() then
        c.err, c.kind = "lightmap texture '" .. lmPath .. "' is missing (VRAD ran without the bake, or with -fast)", "lightmap"
        return nil, c.err, c.kind
    end
    local w, h = tex:Width(), tex:Height()
    if w < 1 or h < 1 or w > 4096 or h > 4096 then
        c.err = "lightmap texture '" .. lmPath .. "' has odd size " .. w .. "x" .. h .. "; the props that use it stay plain"
        PROPLM.Msg(c.err)
        return nil, c.err
    end

    local rt
    for i, fmt in ipairs(FLOAT_FORMATS) do
        rt = GetRenderTargetEx("proplm_rt_" .. serial .. "_" .. fmt, w, h, RT_SIZE_NO_CHANGE, MATERIAL_RT_DEPTH_NONE,
            TEXTUREFLAGS_CLAMP, 0, fmt)
        if rt and not rt:IsError() then
            if i > 1 then
                PROPLM.Msg("lightmap '%s': no 16-bit float render target available, using RGBA8888 (dark lighting may show banding)", lmPath)
            end
            break
        end
        rt = nil
    end
    if not rt then
        c.err = "cannot create a render target for lightmap '" .. lmPath .. "' (" .. w .. "x" .. h .. "); the props that use it stay plain"
        PROPLM.Msg(c.err)
        return nil, c.err
    end
    c.rt, c.src = rt, src
    LM.Draw(c) -- only the new target; the others have their own windows
    arm(c)
    return rt
end

-- Returns the converted ITexture for the VTF at `lmPath` (no materials/, no .vtf), or nil + reason
-- (+ "lightmap" as a third value when the cause is the missing lightmap texture; the caller prints that
-- one once per map). Every other failure prints its own line here, once, when the entry is created;
-- the verdict is cached, so asking again costs a table lookup and draws nothing. A creation step that
-- raises is a failure like any other: the entry gets its reason, so later asks do not answer nil with none.
function LM.Get(lmPath)
    local c = cache[lmPath]
    if c then return c.rt, c.err, c.kind end
    c = { path = lmPath }
    cache[lmPath] = c

    local ok, rt, err, kind = pcall(create, c)
    if not ok then
        c.rt, c.src = nil, nil
        c.err = "cannot create the converted lightmap '" .. lmPath .. "': " .. tostring(rt) .. "; the props that use it stay plain"
        PROPLM.Msg(c.err)
        return nil, c.err
    end
    return rt, err, kind
end

-- Redraws every converted lightmap now and keeps redrawing each for REFRESH_SECONDS. For events that can
-- empty all targets at once (video mode change, proplm_reload).
function LM.Refresh()
    for _, c in pairs(cache) do
        if c.rt then
            LM.Draw(c)
            arm(c)
        end
    end
end

-- Changes the gain and redraws every converted lightmap (a tuning aid; the shipped value is GAIN above).
function LM.SetGain(g)
    LM.GAIN = g
    for _, c in pairs(cache) do
        if c.rt then
            c.src:SetVector("$color", Vector(g, g, g))
            LM.Draw(c)
        end
    end
end

hook.Add("PreRender", "proplm_lightmap", function()
    local n = #active
    if n == 0 then return end
    local now, keep = RealTime(), 0
    for i = 1, n do
        local c = active[i]
        if not c.refreshUntil then c.refreshUntil = now + REFRESH_SECONDS end -- first frame since arm()
        if now < c.refreshUntil then
            LM.Draw(c)
        end
        if now < c.refreshUntil then -- a failed draw ends the window
            keep = keep + 1
            active[keep] = c
        else
            c.armed = false
        end
    end
    for i = keep + 1, n do active[i] = nil end
end)

hook.Add("OnScreenSizeChanged", "proplm_lightmap", function() LM.Refresh() end)
