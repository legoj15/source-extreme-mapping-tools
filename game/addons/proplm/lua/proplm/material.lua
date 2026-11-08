-- proplm: builds the per-instance materials.
--
-- Each material group of a model, on each instance, gets one VertexLitGeneric material:
--   $basetexture  the albedo of the model's own material (UV0)
--   $detail       the baked lightmap (UV2, via $separatedetailuvs), converted by lightmap.lua
--   $detailblendmode 0  multiply by two
-- The draw suppresses engine lighting (render.SuppressEngineLighting) so that nothing is lit twice.
-- UnlitGeneric is NOT used: measured on GMod x64, an UnlitGeneric mesh ignores texcoord 1 for
-- $detail (it samples at texcoord 0) while VertexLitGeneric honours $separatedetailuvs.
-- Lighting-dependent features of the source material (normal map, phong, envmap, selfillum) are not
-- carried over; see docs/proplm.md, "Lua addon".

PROPLM = PROPLM or {}
local M = {}
PROPLM.material = M

-- MaterialVarFlags_t bits that matter here
local FLAG_ADDITIVE = bit.lshift(1, 7)
local FLAG_ALPHATEST = bit.lshift(1, 8)
local FLAG_NOCULL = bit.lshift(1, 13)
local FLAG_TRANSLUCENT = bit.lshift(1, 21)

local SHADER = "VertexLitGeneric"

local sourceCache = {} -- source material name -> params table, or false + reason in sourceWhy
local sourceWhy = {}
local serial = 0

function M.Reset()
    sourceCache, sourceWhy = {}, {}
end

local function nonDefaultColor(src)
    local c1 = src:GetVector("$color")
    local c2 = src:GetVector("$color2")
    -- an undefined colour variable reads back as (0,0,0); a black model is not a real case
    if not c1 or c1:LengthSqr() == 0 then c1 = Vector(1, 1, 1) end
    if not c2 or c2:LengthSqr() == 0 then c2 = Vector(1, 1, 1) end
    local r, g, b = c1.x * c2.x, c1.y * c2.y, c1.z * c2.z
    if math.abs(r - 1) < 0.004 and math.abs(g - 1) < 0.004 and math.abs(b - 1) < 0.004 then return nil end
    return string.format("[%.4f %.4f %.4f]", r, g, b)
end

-- What of the model's own material we can reproduce. Cached per material name.
-- Returns {params = <CreateMaterial keyvalues>, xform = <VMatrix or nil>}, or nil + reason.
local function sourceParams(name)
    local cached = sourceCache[name]
    if cached then return cached end
    if cached == false then return nil, sourceWhy[name] end

    local function fail(why)
        sourceCache[name], sourceWhy[name] = false, why
        return nil, why
    end

    local src = Material(name)
    if not src or src:IsError() then return fail("material '" .. name .. "' not found") end
    local bt = src:GetTexture("$basetexture")
    if not bt or bt:IsError() then return fail("material '" .. name .. "' has no usable $basetexture") end

    local p = {
        -- proplm_debug 2 (then proplm_reload): white albedo, so only the baked lightmap shows
        ["$basetexture"] = PROPLM.cvDebug:GetInt() >= 2 and "vgui/white" or bt:GetName(),
        ["$detailscale"] = 1,
        ["$detailblendfactor"] = PROPLM.lightmap.BLEND,
        ["$detailblendmode"] = 0,
        -- both spellings: Valve's shader parameter is the misspelled one
        ["$separatedetailuvs"] = 1,
        ["$seperatedetailuvs"] = 1,
    }
    local flags = src:GetInt("$flags") or 0
    if bit.band(flags, FLAG_ALPHATEST) ~= 0 then
        p["$alphatest"] = 1
        local ref = src:GetFloat("$alphatestreference")
        if ref and ref > 0 then p["$alphatestreference"] = ref end
    end
    if bit.band(flags, FLAG_TRANSLUCENT) ~= 0 then p["$translucent"] = 1 end
    if bit.band(flags, FLAG_ADDITIVE) ~= 0 then p["$additive"] = 1 end
    if bit.band(flags, FLAG_NOCULL) ~= 0 then p["$nocull"] = 1 end
    local alpha = src:GetFloat("$alpha")
    if alpha and alpha > 0 and alpha < 1 then p["$alpha"] = alpha end
    local color = nonDefaultColor(src)
    if color then p["$color"] = color end

    local xform = src:GetMatrix("$basetexturetransform")
    local entry = { params = p }
    if xform and not xform:IsIdentity() then entry.xform = xform end
    sourceCache[name] = entry
    return entry
end

-- Builds the materials for one instance: one per group of the model state.
-- groups[i].material is the source material name. Returns an array of IMaterial, or nil + reason
-- (+ "lightmap" as a third value when the cause is the missing lightmap texture).
function M.ForInstance(rec, groups)
    -- The baked VTF, converted once into the form the $detail multiply needs (see lightmap.lua).
    local lmTex, lmWhy, lmKind = PROPLM.lightmap.Get(rec.lm)
    if not lmTex then return nil, lmWhy, lmKind end

    local mats = {}
    for i, g in ipairs(groups) do
        local entry, why = sourceParams(g.material)
        if not entry then return nil, why end
        local params = {}
        for k, v in pairs(entry.params) do params[k] = v end
        params["$detail"] = lmTex:GetName()

        serial = serial + 1
        local mat = CreateMaterial(string.format("proplm_%d_%d_%d", serial, rec.id, i), SHADER, params)
        if not mat or mat:IsError() then return nil, "CreateMaterial failed for '" .. g.material .. "'" end
        if entry.xform then mat:SetMatrix("$basetexturetransform", entry.xform) end
        mats[i] = mat
    end
    return mats
end
