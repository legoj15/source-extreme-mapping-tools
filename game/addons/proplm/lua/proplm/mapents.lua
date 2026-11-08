-- proplm: reads the proplm props out of the map's entity lump text.
--
-- The text is a list of  { "key" "value" ... }  blocks. It is tokenised: a quoted string is consumed whole,
-- from its opening quote to the next quote (Source has no escapes), and only braces outside quotes open or
-- close a block, so a brace inside a key or value can neither end a block nor shift the entity index.
-- Inside a block the strings alternate key, value. Only prop_dynamic entities with proplm "1" are kept,
-- and only the keys the addon needs.

PROPLM = PROPLM or {}
local E = {}
PROPLM.mapents = E

local WANTED = {
    classname = true, proplm = true, model = true, origin = true, angles = true,
    proplm_id = true, proplm_lm = true, hammerid = true,
}

-- Returns an array of records:
--   {id, index, model, origin (Vector), angles (Angle), lm, hammerid (or nil)}
-- plus a count of proplm entities that were unusable (missing/invalid keys), and the first reason.
function E.Parse(text)
    local P = PROPLM
    local find, sub, byte = string.find, string.sub, string.byte
    local records, bad, badWhy = {}, 0, nil

    local function finish(kv, index)
        if kv.classname ~= "prop_dynamic" or kv.proplm ~= "1" then return end
        local ox, oy, oz = P.ParseTriple(kv.origin)
        local ap, ay, ar = P.ParseTriple(kv.angles or "0 0 0")
        local lm = kv.proplm_lm
        local model = P.Norm(kv.model)
        if not ox or not ap or model == "" or not lm or lm == "" then
            bad = bad + 1
            badWhy = badWhy or "an entity lacks a valid model, origin, angles or proplm_lm"
            return
        end
        records[#records + 1] = {
            id = tonumber(kv.proplm_id) or (#records),
            index = index, -- position of the entity in the lump, 0-based (worldspawn is 0)
            model = model,
            origin = Vector(ox, oy, oz),
            angles = Angle(ap, ay, ar),
            lm = lm,
            hammerid = tonumber(kv.hammerid),
        }
    end

    local kv, key -- the open block's keys so far, and the key still waiting for its value
    local index = -1
    local pos = 1
    while true do
        local s0 = find(text, '[{}"]', pos)
        if not s0 then break end
        local c = byte(text, s0)
        if c == 34 then -- a quoted string: take it whole
            local e = find(text, '"', s0 + 1, true)
            if not e then break end -- unterminated string: the text ends here
            if kv then
                if key == nil then
                    key = sub(text, s0 + 1, e - 1)
                else
                    if WANTED[key] then kv[key] = sub(text, s0 + 1, e - 1) end
                    key = nil
                end
            end
            pos = e + 1
        elseif c == 123 then -- {
            kv, key, pos, index = {}, nil, s0 + 1, index + 1
        else -- }
            if kv then finish(kv, index) end
            kv, key, pos = nil, nil, s0 + 1
        end
    end
    return records, bad, badWhy
end
