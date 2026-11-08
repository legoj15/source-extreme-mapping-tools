-- proplm: console commands. proplm_status describes what happened to every proplm prop,
-- proplm_reload rereads the map data and rebuilds everything.

PROPLM = PROPLM or {}
local P = PROPLM

function P.Status()
    local S = P.S
    P.Msg("enabled=%s debug=%s map=%s", P.cvEnable:GetString(), P.cvDebug:GetString(), tostring(S.map or game.GetMap()))
    if not S.loaded then
        return P.Msg("no proplm data loaded for this map (no plm2 lump, or it was unusable; see proplm_debug 1 and proplm_reload)")
    end
    P.Msg("plm2 lump: %d bytes, %d model(s), %d triangles, flags %d, version %d; read in %.1f ms; MapCreationID base %s",
        S.index.lumpBytes, S.index.modelCount, S.index.triTotal, S.entry.flags, S.entry.version, S.loadMs, tostring(S.mcidBase))

    local names = {}
    for name in pairs(S.byModel) do names[#names + 1] = name end
    table.sort(names)
    for _, name in ipairs(names) do
        local ms = S.models[name]
        if not ms then
            P.Msg("model %s: %d prop(s), not built yet (no instance has been on this client)", name, #S.byModel[name])
        elseif ms.ok then
            P.Msg("model %s: %d prop(s), matched %d/%d triangles (%.1f%%), %d degenerate skipped, %d lump triangles unused, %d group(s), match %.1f ms, mesh %s ms",
                name, #S.byModel[name], ms.matched, ms.total, ms.rate * 100, ms.degenerate, ms.lumpUnused, #ms.groups, ms.ms,
                ms.meshMs and string.format("%.1f", ms.meshMs) or "-")
        else
            P.Msg("model %s: %d prop(s), NOT USABLE: %s", name, #S.byModel[name], ms.reason)
        end
    end

    for _, rec in ipairs(S.recs) do
        local o = rec.origin
        P.Msg("prop #%d hammerid=%s %s at %g %g %g: %s%s%s", rec.id, tostring(rec.hammerid), rec.model, o.x, o.y, o.z,
            rec.status, rec.how and (" (matched by " .. rec.how .. ")") or "", rec.reason and (": " .. rec.reason) or "")
    end

    local c = P.Counts()
    P.Msg("props: %d total, %d overridden, %d skipped, %d disabled, %d waiting (not on this client)", c.total, c.overridden, c.skipped, c.disabled, c.waiting)
    if S.drawN > 0 then
        P.Msg("draw cost (proplm_debug 1): %d calls, %.1f us per call (CPU submit time); %d depth/shadow passes left to the engine",
            S.drawN, S.drawT / S.drawN * 1e6, S.depthN or 0)
    end
end

concommand.Add("proplm_status", function() P.Status() end, nil, "Show what proplm did with each converted prop on this map.")

concommand.Add("proplm_reload", function()
    P.Load()
    P.Msg("reloaded")
end, nil, "Reread the proplm data of the current map and rebuild every converted prop.")
