-- proplm: the renderer. Loads the map's proplm data, matches it to the client's prop_dynamic entities,
-- and replaces each matched entity's drawing with the lightmapped meshes.
--
-- Data flow:  maps/<map>.bsp --(bsp.lua, lump.lua, mapents.lua)--> records (one per proplm entity)
--             + lump index (one entry per model)
--             record + client entity --(findRecord)--> attach --(apply)--> ent.RenderOverride
-- apply() sets RenderOverride last, after everything for that instance succeeded, so a failure at any
-- earlier step leaves the engine's own drawing of the prop untouched.

PROPLM = PROPLM or {}
local P = PROPLM
P.S = P.S or {}
local S = P.S

local PENDING_SECONDS = 6 -- how long a new entity may take before its model/origin are networked
local SUMMARY_DELAY = 3

-- Engine depth-only passes (flashlight/sun shadow maps, SSAO) must keep drawing the plain model.
local SKIP_PASSES = bit.bor(STUDIO_SHADOWDEPTHTEXTURE or 0x40000000, STUDIO_SSAODEPTHTEXTURE or 0x08000000)

local debugDraw = false -- mirrors proplm_debug without a ConVar lookup per frame

local function resetState()
    S.map, S.loaded, S.entry, S.index, S.mcidBase = nil, false, nil, nil, nil
    S.recs, S.byModel, S.models = {}, {}, {}
    S.pending, S.warned = {}, {}
    S.drawN, S.drawT, S.depthN = 0, 0, 0
    S.loadMs = 0
    S.generation = (S.generation or 0) + 1
end
if not S.recs then resetState() end

-- ---------------------------------------------------------------------------------------------
-- unload

local function removeOverride(rec)
    local ent = rec.ent
    if IsValid(ent) and rec.draw and ent.RenderOverride == rec.draw then ent.RenderOverride = nil end
end

function P.RemoveOverrides()
    for _, rec in ipairs(S.recs) do
        removeOverride(rec)
        if rec.status == "overridden" then rec.status, rec.reason = "disabled", "proplm_enable is 0" end
    end
end

function P.Unload()
    for _, rec in ipairs(S.recs) do removeOverride(rec) end
    for _, ms in pairs(S.models) do P.mesh.Destroy(ms) end
    P.material.Reset()
    resetState()
end

-- ---------------------------------------------------------------------------------------------
-- matching

-- The bake is only valid at the pose it was made at, so a record matches an entity of its model
-- sitting at its origin (within half a unit) and angles (within 1.5 degrees).
--
-- Measured on GMod x64: Entity:MapCreationID() is NOT the hammerid. It is a running number over the
-- entity lump: worldspawn (lump entry 0) has some base value B and entry k has B + k. So an entity
-- whose MapCreationID equals B + record.index is the record's entity. B is read from the world entity
-- at load time. When B and the entity's MapCreationID are both known the number decides: a record at
-- the same pose whose number differs belongs to another entity (several records of one model at one
-- origin, or a plain prop_dynamic standing where a converted one was baked) and is not a candidate.
-- Only when either number is unknown does the pose alone decide (first candidate wins).
local POS_TOL2, ANG_TOL = 0.25, 1.5

local function sameAngles(a, b)
    return math.abs(math.AngleDifference(a.p, b.p)) < ANG_TOL
        and math.abs(math.AngleDifference(a.y, b.y)) < ANG_TOL
        and math.abs(math.AngleDifference(a.r, b.r)) < ANG_TOL
end

-- The entity's MapCreationID, or nil when it has none (-1 for entities the map did not create).
local function mapCreationId(ent)
    local ok, id = pcall(ent.MapCreationID, ent)
    if ok and type(id) == "number" and id >= 0 then return id end
end

local function findRecord(ent, list)
    local pos, ang = ent:GetPos(), ent:GetAngles()
    local base = S.mcidBase
    local mcid = base and mapCreationId(ent)
    local first
    for _, rec in ipairs(list) do
        if (rec.ent == ent or not IsValid(rec.ent)) and rec.origin:DistToSqr(pos) < POS_TOL2 and sameAngles(ang, rec.angles) then
            if mcid then
                if mcid == base + rec.index then return rec, "origin and MapCreationID" end
            else
                first = first or rec
            end
        end
    end
    if first then return first, "origin" end
end

-- ---------------------------------------------------------------------------------------------
-- model state

local function modelState(name)
    local ms = S.models[name]
    if ms then return ms end
    local info = S.index.models[name]
    if not info then
        ms = { name = name, ok = false, groups = {}, reason = "the plm2 lump has no entry for this model" }
    else
        ms = P.mesh.Prepare(S.map, info)
    end
    S.models[name] = ms
    if ms.ok then
        P.Dbg("model %s: matched %d/%d triangles (%.1f%%), %d degenerate skipped, %d lump triangles unused, %d group(s), %.1f ms (read %.1f, lookup %.1f, GetModelMeshes %.1f, match %.1f)",
            name, ms.matched, ms.total, ms.rate * 100, ms.degenerate, ms.lumpUnused, #ms.groups, ms.ms,
            ms.t.read, ms.t.lookup, ms.t.engine, ms.t.match)
    else
        P.Msg("model %s stays a plain prop: %s", name, ms.reason)
    end
    return ms
end

-- ---------------------------------------------------------------------------------------------
-- drawing

local function drawGroups(mats, groups)
    for gi = 1, #groups do
        render.SetMaterial(mats[gi])
        local meshes = groups[gi].meshes
        for mi = 1, #meshes do meshes[mi]:Draw() end
    end
end

local function makeDraw(rec, ms)
    local matrix = Matrix()
    local groups, mats = ms.groups, rec.mats
    return function(self, flags)
        if flags and bit.band(flags, SKIP_PASSES) ~= 0 then
            S.depthN = (S.depthN or 0) + 1
            self:DrawModel(flags)
            return
        end
        local t0 = debugDraw and SysTime()
        matrix:SetTranslation(self:GetPos())
        matrix:SetAngles(self:GetAngles())
        cam.PushModelMatrix(matrix)
        render.SuppressEngineLighting(true) -- the lightmap is the lighting; VertexLitGeneric must not add its own
        local ok, err = pcall(drawGroups, mats, groups)
        render.SuppressEngineLighting(false)
        cam.PopModelMatrix()
        if not ok then
            -- never leave the prop broken: back to the engine's drawing, once, loudly. The failure is latched
            -- on the record (apply checks it), so a rescan or the entity re-entering the PVS does not install
            -- the override again; proplm_reload builds new records and clears it.
            self.RenderOverride = nil
            rec.drawFailed = "drawing failed: " .. tostring(err)
            rec.status, rec.reason = "skipped", rec.drawFailed
            P.Msg("prop #%d (%s): drawing failed, back to the plain prop: %s", rec.id, rec.model, tostring(err))
            self:DrawModel(flags)
            return
        end
        if t0 then S.drawN, S.drawT = S.drawN + 1, S.drawT + (SysTime() - t0) end
    end
end

-- ---------------------------------------------------------------------------------------------
-- applying

-- Returns true, or false + reason (+ a warning category printed once per map).
local function apply(rec, ent)
    -- (order matters: nothing up to the final RenderOverride assignment may change the engine's drawing)
    if rec.drawFailed then return false, rec.drawFailed end
    local skin = ent:GetSkin()
    if skin ~= 0 then return false, "skin " .. skin .. " (the lump holds the materials of skin 0 only)" end
    local scale = ent:GetModelScale()
    if math.abs(scale - 1) > 0.001 then return false, "modelscale " .. scale .. " (the bake ignores it)" end
    local c = ent:GetColor()
    if c.r ~= 255 or c.g ~= 255 or c.b ~= 255 or c.a ~= 255 then return false, "render colour or alpha is set" end
    if ent.RenderOverride and ent.RenderOverride ~= rec.draw then return false, "another addon already overrides its rendering" end

    local ms = modelState(rec.model)
    if not ms.ok then return false, "model: " .. ms.reason end

    if not rec.mats then
        local mats, why, kind = P.material.ForInstance(rec, ms.groups)
        if not mats then return false, why, kind end
        rec.mats = mats
    end
    if not ms.meshesBuilt then
        -- once per model: a failure marks the model unusable (ms.ok false), so its other instances stop at
        -- the check above without repeating the work or the message
        local built, why = P.mesh.EnsureMeshes(ms, rec.mats)
        if not built then
            P.Msg("model %s stays a plain prop: %s", rec.model, why)
            return false, "model: " .. why
        end
    end

    rec.draw = rec.draw or makeDraw(rec, ms)
    ent.RenderOverride = rec.draw
    return true
end

local function attach(rec, ent, how)
    rec.ent, rec.how = ent, how
    ent.proplmRec = rec
    if not P.Enabled() then
        rec.status, rec.reason = "disabled", "proplm_enable is 0"
        return
    end
    local ok, res, why, kind = pcall(apply, rec, ent)
    if not ok then res, why, kind = false, "error: " .. tostring(res), "error" end
    if res then
        rec.status, rec.reason = "overridden", nil
        P.Dbg("prop #%d (%s) at %d %d %d: drawn with its lightmap (matched by %s)", rec.id, rec.model,
            rec.origin.x, rec.origin.y, rec.origin.z, how)
    else
        rec.status, rec.reason = "skipped", why
        if kind and not S.warned[kind] then
            S.warned[kind] = true
            P.Msg("prop #%d (%s) stays a plain prop: %s", rec.id, rec.model, why)
        else
            P.Dbg("prop #%d (%s) stays a plain prop: %s", rec.id, rec.model, why)
        end
    end
end

-- true: settled (attached, or not one of ours); false: try again shortly
local function consider(ent)
    if not IsValid(ent) or ent:GetClass() ~= "prop_dynamic" then return true end
    local done = ent.proplmRec
    if done and done.ent == ent and done.status == "overridden" and ent.RenderOverride == done.draw then return true end
    local model = ent:GetModel()
    if not model or model == "" then return false end
    local list = S.byModel[P.Norm(model)]
    if not list then return true end
    local rec, how = findRecord(ent, list)
    if not rec then return false end
    attach(rec, ent, how)
    return true
end

function P.Scan()
    if not S.loaded then return end
    local deadline = RealTime() + PENDING_SECONDS
    for _, ent in ipairs(ents.FindByClass("prop_dynamic")) do
        if not consider(ent) then S.pending[ent] = deadline end
    end
end

-- Re-applies every record (after proplm_enable goes back to 1). A scan finds the already matched entities
-- again, because findRecord accepts a record whose entity is the one being looked at.
function P.Refresh()
    P.Scan()
end

-- ---------------------------------------------------------------------------------------------
-- load

local function readMap(map, res)
    local f = P.bsp.Open(map)
    if not f then error("cannot open maps/" .. map .. ".bsp", 0) end
    local ok, err = pcall(function()
        local bsp = P.bsp.Header(f) -- not a readable BSP: stays silent, it is not a proplm problem
        res.dirRead = true -- from here on a corrupt game lump directory is worth a message
        local entry = P.bsp.FindGameLump(bsp, P.lump.ID_BYTES)
        if not entry then return end
        res.found = true
        res.entry = entry
        res.index = P.lump.Index(f, entry)
        res.records, res.bad, res.badWhy = P.mapents.Parse(P.bsp.EntityText(bsp))
    end)
    f:Close()
    if not ok then error(err, 0) end
end

function P.Load()
    P.Unload()
    local map = game.GetMap()
    S.map = map
    local t0 = SysTime()

    local res = {}
    local ok, err = pcall(readMap, map, res)
    if not ok then
        if res.dirRead then
            P.Msg("map %s: proplm data unusable, props stay plain: %s", map, tostring(err))
        else
            P.Dbg("map %s: no proplm data (%s)", map, tostring(err))
        end
        return
    end
    if not res.found then return P.Dbg("map %s has no plm2 lump", map) end
    if #res.records == 0 and (res.bad or 0) == 0 then
        return P.Dbg("map %s has a plm2 lump but no proplm entities", map)
    end
    if (res.bad or 0) > 0 then
        P.Msg("map %s: %d proplm entit%s unusable (%s)", map, res.bad, res.bad == 1 and "y" or "ies", res.badWhy)
    end

    S.entry, S.index, S.recs = res.entry, res.index, res.records
    -- no IsValid here: the client's world entity reports invalid although MapCreationID works on it.
    -- If the call fails or answers nothing usable, matching goes by pose alone.
    local okBase, base = pcall(function()
        local world = game.GetWorld()
        return world and world:MapCreationID()
    end)
    S.mcidBase = okBase and type(base) == "number" and base > 0 and base or nil
    for _, rec in ipairs(S.recs) do
        rec.status = "waiting"
        local list = S.byModel[rec.model]
        if not list then list = {}; S.byModel[rec.model] = list end
        list[#list + 1] = rec
    end
    S.loaded = true
    S.loadMs = (SysTime() - t0) * 1000
    P.Dbg("map %s: plm2 lump %d bytes, %d model(s), %d triangles; %d proplm entit%s; read in %.1f ms; MapCreationID base %s",
        map, S.index.lumpBytes, S.index.modelCount, S.index.triTotal, #S.recs, #S.recs == 1 and "y" or "ies", S.loadMs, tostring(S.mcidBase))

    -- a reload also redraws the converted lightmaps that survive it (render targets cannot be freed). Only
    -- here, once the map is known to have proplm props: a map without them has no use for the old targets
    P.lightmap.Refresh()
    P.Scan()

    local gen = S.generation
    timer.Simple(SUMMARY_DELAY, function()
        if S.generation == gen then P.Summary() end
    end)
end

function P.Counts()
    local c = { total = #S.recs, overridden = 0, skipped = 0, disabled = 0, waiting = 0 }
    for _, rec in ipairs(S.recs) do c[rec.status] = (c[rec.status] or 0) + 1 end
    return c
end

function P.Summary()
    if not S.loaded then return end
    local c = P.Counts()
    local models = 0
    for _ in pairs(S.byModel) do models = models + 1 end
    local line = string.format("map %s: %d proplm prop(s), %d model(s): %d drawn with baked lightmaps", S.map, c.total, models, c.overridden)
    if c.skipped > 0 then line = line .. string.format(", %d left as plain props", c.skipped) end
    if c.disabled > 0 then line = line .. string.format(", %d disabled", c.disabled) end
    if c.waiting > 0 then line = line .. string.format(", %d not on this client yet", c.waiting) end
    if c.skipped > 0 or c.waiting > 0 then line = line .. " (proplm_status for details)" end
    P.Msg(line)
end

-- ---------------------------------------------------------------------------------------------
-- hooks

hook.Add("InitPostEntity", "proplm", function() P.Load() end)

hook.Add("OnEntityCreated", "proplm", function(ent)
    if S.loaded and IsValid(ent) and ent:GetClass() == "prop_dynamic" then
        S.pending[ent] = RealTime() + PENDING_SECONDS
    end
end)

hook.Add("EntityRemoved", "proplm", function(ent)
    if not S.loaded then return end
    S.pending[ent] = nil
    local rec = ent.proplmRec
    if rec and rec.ent == ent then
        rec.ent = nil
        if rec.status == "overridden" then rec.status = "waiting" end
    end
end)

hook.Add("Think", "proplm", function()
    if not S.loaded or next(S.pending) == nil then return end
    local now = RealTime()
    for ent, deadline in pairs(S.pending) do
        if not IsValid(ent) or consider(ent) or now > deadline then S.pending[ent] = nil end
    end
end)

hook.Add("PostCleanupMap", "proplm", function()
    -- game.CleanUpMap recreates the map's entities; the old records point at dead ones
    for _, rec in ipairs(S.recs or {}) do
        rec.ent = nil
        if rec.status == "overridden" then rec.status = "waiting" end
    end
    timer.Simple(0.5, P.Scan)
end)

hook.Add("ShutDown", "proplm", function()
    for _, ms in pairs(S.models) do P.mesh.Destroy(ms) end
end)

cvars.AddChangeCallback("proplm_enable", function(_, _, new)
    if tobool(new) then P.Refresh() else P.RemoveOverrides() end
end, "proplm")

cvars.AddChangeCallback("proplm_debug", function(_, _, new) debugDraw = tobool(new) end, "proplm")
debugDraw = P.cvDebug:GetBool()
