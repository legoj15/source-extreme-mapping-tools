-- proplm: parser for the 'plm2' game lump (format: src/public/proplm.h, docs/proplm.md).
--
--   int32 version, int32 modelCount
--   per model: char name[128], int32 materialCount, int32 triCount, uint32 posChecksum (skipped: the
--              triangle join in mesh.lua is the integrity check), char material[materialCount][96],
--              triCount x { int32 materialIndex, 3 x { float pos[3], normal[3], uv0[2], uv2[2] } }
--
-- Index() walks only the model headers (seeks over the triangles). ReadTriangles() reads one model's
-- triangles on demand. Every size is checked against the lump length; corruption raises error(msg, 0).

PROPLM = PROPLM or {}
local L = {}
PROPLM.lump = L

L.VERSION = 1
L.ID_BYTES = "2mlp" -- the int 'plm2' as it appears on disk

local NAME_BYTES, MATERIAL_BYTES = 128, 96
local MODEL_HEAD = NAME_BYTES + 12
local CORNER_BYTES = 40
local TRI_BYTES = 4 + 3 * CORNER_BYTES
local MAX_MATERIALS = 65536

local function cstring(s)
    local z = s:find("\0", 1, true)
    if z then s = s:sub(1, z - 1) end
    return s
end

-- Returns {models = {[name] = info}, order = {name...}, modelCount, triTotal, lumpBytes}.
-- info = {name, matCount, triCount, triOfs}
function L.Index(f, entry)
    if entry.flags ~= 0 then
        error(string.format("the lump has flags %d (compressed game lumps are not supported)", entry.flags), 0)
    end
    if entry.version ~= L.VERSION then error("unsupported game lump version " .. entry.version, 0) end
    if entry.len < 8 then error("the lump is shorter than its header", 0) end

    local lumpEnd = entry.ofs + entry.len
    f:Seek(entry.ofs)
    local version, count = f:ReadLong(), f:ReadLong()
    if version ~= L.VERSION then error("unsupported lump header version " .. version, 0) end
    if count < 0 or count * MODEL_HEAD > entry.len then error("model count " .. count .. " does not fit the lump", 0) end

    local idx = { models = {}, order = {}, modelCount = count, triTotal = 0, lumpBytes = entry.len }
    local pos = entry.ofs + 8
    for i = 1, count do
        if pos + MODEL_HEAD > lumpEnd then error("the lump ends inside the header of model " .. i, 0) end
        f:Seek(pos)
        local name = PROPLM.Norm(cstring(f:Read(NAME_BYTES)))
        local matCount, triCount = f:ReadLong(), f:ReadLong() -- the uint32 checksum after them is not read
        if matCount < 0 or matCount > MAX_MATERIALS or triCount < 0 then
            error(string.format("model %d has %d materials and %d triangles", i, matCount, triCount), 0)
        end
        local triOfs = pos + MODEL_HEAD + matCount * MATERIAL_BYTES
        local nextPos = triOfs + triCount * TRI_BYTES
        if nextPos > lumpEnd then error("the lump ends inside model " .. i .. " (" .. name .. ")", 0) end
        if name == "" then error("model " .. i .. " has no name", 0) end
        if idx.models[name] then error("model " .. name .. " appears twice", 0) end
        idx.models[name] = { name = name, matCount = matCount, triCount = triCount, triOfs = triOfs }
        idx.order[i] = name
        idx.triTotal = idx.triTotal + triCount
        pos = nextPos
    end
    return idx
end

-- IEEE-754 single (little endian bytes b1..b4) to a Lua number. Exact: a float32 always fits a double.
local POW2 = {}
for e = 0, 255 do POW2[e] = 2 ^ (e - 127) end
local function f32(b1, b2, b3, b4)
    local exp = (b4 % 128) * 2 + (b3 >= 128 and 1 or 0)
    local mant = (b3 % 128) * 65536 + b2 * 256 + b1
    local v
    if exp == 0 then v = mant * 2 ^ -149
    elseif exp == 255 then v = mant == 0 and math.huge or 0 / 0
    else v = (1 + mant / 8388608) * POW2[exp] end
    return b4 >= 128 and -v or v
end

local function i32(b1, b2, b3, b4)
    local v = b1 + b2 * 256 + b3 * 65536 + b4 * 16777216
    return v >= 2147483648 and v - 4294967296 or v
end

-- Reads one model's triangles into flat arrays (3 corners per triangle, 1-based):
--   T.n                      triangle count
--   T.pos[(c-1)*3 + 1..3]    corner c position       (c = (t-1)*3 + 1 .. 3)
--   T.uv0[(c-1)*2 + 1..2]    corner c original UV
--   T.uv2[(c-1)*2 + 1..2]    corner c lightmap UV
-- Normals are not needed by the addon (the engine mesh supplies them): they are neither decoded nor checked.
-- The whole block is fetched with one File:Read and decoded here: File:ReadFloat costs ~3 us a call
-- (measured), which made a 1000-triangle model take 80 ms; this takes a few.
function L.ReadTriangles(f, info)
    local n = info.triCount
    local want = n * TRI_BYTES
    f:Seek(info.triOfs)
    local data = f:Read(want)
    if not data or #data ~= want then error("the lump ends inside the triangles of " .. info.name, 0) end

    local byte = string.byte
    local pos, uv0, uv2 = {}, {}, {}
    local pi, ui = 0, 0
    for t = 1, n do
        local base = (t - 1) * TRI_BYTES
        local mi = i32(byte(data, base + 1, base + 4))
        if mi < 0 or mi >= info.matCount then
            error(string.format("triangle %d of %s has material index %d (model has %d)", t, info.name, mi, info.matCount), 0)
        end
        for c = 0, 2 do
            local o = base + 5 + c * CORNER_BYTES -- first byte of this corner
            local x, y, z = f32(byte(data, o, o + 3)), f32(byte(data, o + 4, o + 7)), f32(byte(data, o + 8, o + 11))
            -- bytes 12..23 are the normal
            local u, v = f32(byte(data, o + 24, o + 27)), f32(byte(data, o + 28, o + 31))
            local u2, v2 = f32(byte(data, o + 32, o + 35)), f32(byte(data, o + 36, o + 39))
            -- n - n is 0 for every finite n and NaN for NaN and for both infinities (NaN ~= 0 is true), so one
            -- test per value covers all three. Any of them would poison the keys or the lightmap UVs.
            if x - x ~= 0 or y - y ~= 0 or z - z ~= 0 or u - u ~= 0 or v - v ~= 0 or u2 - u2 ~= 0 or v2 - v2 ~= 0 then
                error(string.format("triangle %d of %s holds a non-finite value", t, info.name), 0)
            end
            pos[pi + 1], pos[pi + 2], pos[pi + 3] = x, y, z
            uv0[ui + 1], uv0[ui + 2] = u, v
            uv2[ui + 1], uv2[ui + 2] = u2, v2
            pi, ui = pi + 3, ui + 2
        end
    end
    return { n = n, pos = pos, uv0 = uv0, uv2 = uv2 }
end
