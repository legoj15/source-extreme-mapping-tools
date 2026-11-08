-- proplm: shared helpers (messages, console variables, name normalisation).
-- Every module hangs its API off the global PROPLM table; modules look each other up at call time,
-- so the load order only matters for this file (it creates the table).

PROPLM = PROPLM or {}
local P = PROPLM

P.cvEnable = CreateClientConVar("proplm_enable", "1", true, false,
    "1: draw props converted by the proplm compiler profile with their baked lightmap; 0: leave them as plain props.", 0, 1)
P.cvDebug = CreateClientConVar("proplm_debug", "0", false, false,
    "1: print what proplm is doing (matching, per-model match rates, timings). 2: also show the lightmap alone on white props (after proplm_reload).", 0, 2)

function P.Msg(fmt, ...)
    if select("#", ...) > 0 then fmt = string.format(fmt, ...) end
    print("[proplm] " .. fmt)
end

function P.Dbg(fmt, ...)
    if P.cvDebug:GetBool() then P.Msg(fmt, ...) end
end

function P.Enabled()
    return P.cvEnable:GetBool()
end

-- Lowercase, forward slashes, no leading slash: the spelling used by the lump and by Entity:GetModel().
function P.Norm(path)
    return (string.lower(path or ""):gsub("\\", "/"):gsub("^/+", ""))
end

-- Source entity vectors are written "x y z"; returns nil when the text is not three numbers.
function P.ParseTriple(text)
    local a, b, c = string.match(text or "", "^%s*(%S+)%s+(%S+)%s+(%S+)")
    a, b, c = tonumber(a), tonumber(b), tonumber(c)
    if a and b and c then return a, b, c end
end
