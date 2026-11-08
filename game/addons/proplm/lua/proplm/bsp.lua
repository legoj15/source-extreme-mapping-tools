-- proplm: minimal BSP reader. Header, lump directory, game lump directory, entity lump text.
-- Reads with seeks only; the 50 MB file is never loaded whole. Corrupt input raises error(msg, 0),
-- callers wrap in pcall and print one message.

PROPLM = PROPLM or {}
local B = {}
PROPLM.bsp = B

local IDENT = 0x50534256 -- "VBSP" read as a little-endian int32
local LUMP_ENTITIES, LUMP_GAMELUMP = 0, 35
local HEADER_BYTES = 8 + 64 * 16 + 4
local MAX_GAME_LUMPS = 1024

-- Opens maps/<map>.bsp from the game search path (mounted addons and downloads included).
function B.Open(mapName)
    return file.Open("maps/" .. mapName .. ".bsp", "rb", "GAME")
end

-- Reads and checks the header; returns a handle {f, size, version}.
function B.Header(f)
    local size = f:Size()
    if not size or size < HEADER_BYTES then error("file is too small to be a BSP", 0) end
    f:Seek(0)
    if f:ReadLong() ~= IDENT then error("not a VBSP file", 0) end
    return { f = f, size = size, version = f:ReadLong() }
end

-- Returns fileofs, filelen, version, fourCC of lump i (0-based), bounds-checked against the file.
function B.Lump(bsp, i)
    local f = bsp.f
    f:Seek(8 + i * 16)
    local ofs, len, ver, four = f:ReadLong(), f:ReadLong(), f:ReadLong(), f:ReadLong()
    if ofs < 0 or len < 0 or ofs + len > bsp.size then
        error(string.format("lump %d (offset %d, length %d) lies outside the file (%d bytes)", i, ofs, len, bsp.size), 0)
    end
    return ofs, len, ver, four
end

-- The entity lump as text. LZMA-compressed lumps (fourCC holds the uncompressed size) are not supported.
function B.EntityText(bsp)
    local ofs, len, _, four = B.Lump(bsp, LUMP_ENTITIES)
    if four ~= 0 then error("the entity lump is compressed", 0) end
    bsp.f:Seek(ofs)
    return bsp.f:Read(len) or ""
end

-- Finds a game lump by the four id bytes as they appear on disk (e.g. "2mlp" for 'plm2').
-- Returns {ofs, len, flags, version} or nil when the map has no such lump.
function B.FindGameLump(bsp, idBytes)
    local f = bsp.f
    local ofs, len = B.Lump(bsp, LUMP_GAMELUMP)
    if len < 4 then return nil end
    f:Seek(ofs)
    local count = f:ReadLong()
    if count < 0 or count > MAX_GAME_LUMPS or 4 + count * 16 > len then
        error("the game lump directory is corrupt", 0)
    end
    for _ = 1, count do
        local id = f:Read(4)
        local flags, version = f:ReadUShort(), f:ReadUShort()
        local fo, fl = f:ReadLong(), f:ReadLong()
        if id == idBytes then
            if fo < 0 or fl < 0 or fo + fl > bsp.size then
                error(string.format("the lump (offset %d, length %d) lies outside the file (%d bytes)", fo, fl, bsp.size), 0)
            end
            return { ofs = fo, len = fl, flags = flags, version = version }
        end
    end
    return nil
end

-- Runs fn(f) with the map open, always closes it. Errors are re-raised as plain messages.
function B.With(mapName, fn)
    local f = B.Open(mapName)
    if not f then error("cannot open maps/" .. mapName .. ".bsp", 0) end
    local ok, a, b, c = pcall(fn, f)
    f:Close()
    if not ok then error(a, 0) end
    return a, b, c
end
