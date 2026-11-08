-- proplm, server side: marks the client modules for download (AddCSLuaFile), so that a server with this
-- addon installed can hand them to joining players. It draws nothing and has no effect on a client by
-- itself, and a player who installed the addon does not need it. Only a listen server was ever tried;
-- whether a remote server delivers the addon to clients that lack it is untested.

if SERVER then
    for _, name in ipairs({ "util", "bsp", "lump", "mapents", "lightmap", "material", "mesh", "core", "console" }) do
        AddCSLuaFile("proplm/" .. name .. ".lua")
    end
end
