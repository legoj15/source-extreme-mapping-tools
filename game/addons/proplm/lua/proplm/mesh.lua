-- proplm: per-model geometry. Joins the engine's mesh (util.GetModelMeshes) with the lump's UV2.
--
-- The two sources list the same triangles in different order and corner order, so a triangle is
-- identified by the sorted triple of its corner keys. A corner key is the position quantised to
-- 1/16 unit (the quantisation VBSP's checksum uses) plus UV0 quantised to 1/4096. The lump gives the
-- lightmap UV for every corner key of the triangle.
--
-- Model state (one per model, shared by all instances):
--   {name, ok, reason, total, matched, missed, degenerate, lumpUnused, rate, ms, meshMs, meshFailed,
--    groups = { {material = <source material name>, verts = {...} | nil, meshes = {IMesh...} | nil} }}
-- A model is usable only when every non-degenerate engine triangle found its lump triangle and its
-- IMeshes could be built. The verdict, good or bad, is final for the load: a failure sets ok = false and
-- reason (meshFailed too, when it was the IMesh build), and every later instance reads it from here.

PROPLM = PROPLM or {}
local M = {}
PROPLM.mesh = M

local floor = math.floor
local CHUNK_TRIS = 8000 -- stay well under the 32768 vertices an IMesh can hold

local function vkey(x, y, z, u, v)
    return floor(x * 16 + 0.5) .. "," .. floor(y * 16 + 0.5) .. "," .. floor(z * 16 + 0.5) .. ","
        .. floor(u * 4096 + 0.5) .. "," .. floor(v * 4096 + 0.5)
end

-- Sorted triple of three strings joined into one key.
local function tkey(a, b, c)
    if a > b then a, b = b, a end
    if b > c then b, c = c, b end
    if a > b then a, b = b, a end
    return a .. "|" .. b .. "|" .. c
end

-- lookup[tri key] = entry {k1,k2,k3, u1,v1,u2,v2,u3,v3, dup = next entry with the same key}
local function buildLookup(T)
    local lookup = {}
    local pos, uv0, uv2 = T.pos, T.uv0, T.uv2
    for t = 1, T.n do
        local c = (t - 1) * 3
        local keys, e = {}, {}
        for k = 1, 3 do
            local pi, ui = (c + k - 1) * 3, (c + k - 1) * 2
            keys[k] = vkey(pos[pi + 1], pos[pi + 2], pos[pi + 3], uv0[ui + 1], uv0[ui + 2])
            e[k], e[3 + k * 2 - 1], e[3 + k * 2] = keys[k], uv2[ui + 1], uv2[ui + 2]
        end
        local tk = tkey(keys[1], keys[2], keys[3])
        e.dup = lookup[tk]
        lookup[tk] = e
    end
    return lookup
end

-- One engine vertex plus the lightmap UV the lump holds for its key.
local function corner(v, key, e)
    local u2, v2
    if key == e[1] then u2, v2 = e[4], e[5]
    elseif key == e[2] then u2, v2 = e[6], e[7]
    else u2, v2 = e[8], e[9] end
    return { pos = v.pos, normal = v.normal, u = v.u, v = v.v, u2 = u2, v2 = v2 }
end

-- Fills `ms` for one model. `info` is the lump index entry. A failure sets ms.reason and returns early;
-- anything unexpected is an error that M.Prepare catches.
local function build(ms, mapName, info)
    local t0 = SysTime()

    local tR = SysTime()
    local okRead, T = pcall(PROPLM.bsp.With, mapName, function(f) return PROPLM.lump.ReadTriangles(f, info) end)
    if not okRead then
        ms.reason = "cannot read its triangles from the lump: " .. tostring(T)
        return
    end
    local tL = SysTime()
    local lookup = buildLookup(T)

    local tE = SysTime()
    local okGet, groups = pcall(util.GetModelMeshes, info.name, 0, 0)
    local tM = SysTime()
    if not okGet or not groups or #groups == 0 then
        ms.reason = "util.GetModelMeshes returned no geometry for it"
        return
    end

    local total, matched, degenerate = 0, 0, 0
    for _, g in ipairs(groups) do
        local tv = g.triangles
        if not tv or #tv % 3 ~= 0 or type(g.material) ~= "string" then
            ms.reason = "util.GetModelMeshes returned a malformed group"
            return
        end
        local verts = {}
        for i = 1, #tv, 3 do
            local a, b, c = tv[i], tv[i + 1], tv[i + 2]
            local pa, pb, pc = a.pos, b.pos, c.pos
            local ka = vkey(pa.x, pa.y, pa.z, a.u, a.v)
            local kb = vkey(pb.x, pb.y, pb.z, b.u, b.v)
            local kc = vkey(pc.x, pc.y, pc.z, c.u, c.v)
            local tk = tkey(ka, kb, kc)
            local e = lookup[tk]
            if e then
                lookup[tk] = e.dup -- each lump triangle serves one engine triangle
                total, matched = total + 1, matched + 1
                local n = #verts
                verts[n + 1] = corner(a, ka, e)
                verts[n + 2] = corner(b, kb, e)
                verts[n + 3] = corner(c, kc, e)
            elseif (pb - pa):Cross(pc - pa):LengthSqr() < 1e-6 then
                -- collapsed triangle: VBSP leaves it out of the lump and it covers no pixel
                degenerate = degenerate + 1
            else
                total = total + 1
            end
        end
        ms.groups[#ms.groups + 1] = { material = g.material, verts = verts }
    end

    local unused = 0
    for _, e in pairs(lookup) do
        while e do unused, e = unused + 1, e.dup end
    end

    ms.total, ms.matched, ms.degenerate = total, matched, degenerate
    ms.missed = total - matched
    ms.lumpUnused = unused
    ms.rate = total > 0 and matched / total or 0
    ms.ok = ms.missed == 0 and matched > 0
    if not ms.ok then
        ms.reason = string.format("only %d of %d triangles matched the lump (%.1f%%)", matched, total, ms.rate * 100)
        for _, g in ipairs(ms.groups) do g.verts = nil end
    end
    ms.ms = (SysTime() - t0) * 1000
    ms.t = { read = (tL - tR) * 1000, lookup = (tE - tL) * 1000, engine = (tM - tE) * 1000, match = (SysTime() - tM) * 1000 }
end

-- Builds the state for one model. Never raises: failures, including a surprise in the data the engine
-- returned (a missing field, a wrong type), become ms.ok = false with ms.reason.
function M.Prepare(mapName, info)
    local ms = { name = info.name, ok = false, groups = {} }
    local ok, err = pcall(build, ms, mapName, info)
    if not ok then
        ms.ok, ms.groups = false, {}
        ms.reason = "unexpected error while joining its geometry: " .. tostring(err)
    end
    return ms
end

local function destroyGroup(g)
    if g.meshes then
        for _, im in ipairs(g.meshes) do
            if IsValid(im) then im:Destroy() end
        end
        g.meshes = nil
    end
end

function M.Destroy(ms)
    for _, g in ipairs(ms.groups or {}) do destroyGroup(g) end
end

-- Builds the shared IMeshes of a model the first time an instance needs them. `mats` are that
-- instance's materials (one per group); they only supply the vertex format. Returns true, or false +
-- reason. A failure is recorded on the model (ok = false, reason, meshFailed) and is final: the
-- geometry is released and later calls answer from the record without redoing the work.
function M.EnsureMeshes(ms, mats)
    if ms.meshFailed then return false, ms.meshFailed end
    if ms.meshesBuilt then return true end
    local t0 = SysTime()
    local okAll, err = pcall(function()
        for gi, g in ipairs(ms.groups) do
            g.meshes = {}
            local verts = g.verts
            local n = #verts / 3
            for first = 1, n, CHUNK_TRIS do
                local last = math.min(first + CHUNK_TRIS - 1, n)
                local im = Mesh(mats[gi])
                g.meshes[#g.meshes + 1] = im
                mesh.Begin(im, MATERIAL_TRIANGLES, last - first + 1)
                for i = (first - 1) * 3 + 1, last * 3 do
                    local v = verts[i]
                    mesh.Position(v.pos)
                    mesh.Normal(v.normal)
                    mesh.TexCoord(0, v.u, v.v)
                    mesh.TexCoord(1, v.u2, v.v2)
                    mesh.AdvanceVertex()
                end
                mesh.End()
            end
        end
    end)
    if not okAll then
        pcall(mesh.End) -- close a half-built mesh so the builder is usable again
        M.Destroy(ms)
        for _, g in ipairs(ms.groups) do g.verts = nil end
        ms.meshFailed = "building its meshes failed: " .. tostring(err)
        ms.ok, ms.reason = false, ms.meshFailed
        return false, ms.meshFailed
    end
    for _, g in ipairs(ms.groups) do g.verts = nil end
    ms.meshesBuilt = true
    ms.meshMs = (SysTime() - t0) * 1000
    return true
end
