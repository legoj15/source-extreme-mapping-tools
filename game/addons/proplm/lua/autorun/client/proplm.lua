-- proplm: draws props compiled with the proplm option using a second UV set for their lightmap.
-- Entry point. Client only; everything lives in lua/proplm/. See README.md and docs/proplm.md.

if not CLIENT then return end

include("proplm/util.lua")
include("proplm/bsp.lua")
include("proplm/lump.lua")
include("proplm/mapents.lua")
include("proplm/lightmap.lua")
include("proplm/material.lua")
include("proplm/mesh.lua")
include("proplm/core.lua")
include("proplm/console.lua")

-- Installed or reloaded while already in a map (InitPostEntity has come and gone).
if IsValid(LocalPlayer()) then
    timer.Simple(0, function() PROPLM.Load() end)
end
