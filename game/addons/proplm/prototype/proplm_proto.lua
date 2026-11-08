local function P(...) print("[PROTO] ", ...) end
local MODEL = "models/props_canal/boat001a.mdl"
local ent, meshes = nil, {}

-- procedural 'lightmap': 16x16 RT, left half warm, right half cool, bright band in the middle
local rt = GetRenderTargetEx("proplm_proto_lm", 16, 16, RT_SIZE_NO_CHANGE, MATERIAL_RT_DEPTH_NONE, 1 + 256, 0, IMAGE_FORMAT_RGBA8888)
local function fill_rt()
    render.PushRenderTarget(rt); render.Clear(0, 0, 0, 255); cam.Start2D()
    for x = 0, 15 do
        local t = x / 15
        surface.SetDrawColor(math.floor(255 * (1 - t)), math.floor(160 * (1 - math.abs(t - .5) * 2) + 40), math.floor(255 * t), 255)
        surface.DrawRect(x, 0, 1, 16)
    end
    cam.End2D(); render.PopRenderTarget()
end

local function build(model)
    local t0 = SysTime()
    local groups = util.GetModelMeshes(model, 0, 0)
    if not groups then P("GetModelMeshes returned nil"); return end
    local mins, maxs = Vector(1e9, 1e9, 1e9), Vector(-1e9, -1e9, -1e9)
    local ntri = 0
    for gi, g in ipairs(groups) do
        P("group", gi, "material", g.material, "tris(verts)", #g.triangles, "keys", (function() local k = {} for kk in pairs(g.triangles[1]) do k[#k + 1] = kk end table.sort(k) return table.concat(k, ",") end)())
        for _, v in ipairs(g.triangles) do
            local p = v.pos
            mins.x, mins.y, mins.z = math.min(mins.x, p.x), math.min(mins.y, p.y), math.min(mins.z, p.z)
            maxs.x, maxs.y, maxs.z = math.max(maxs.x, p.x), math.max(maxs.y, p.y), math.max(maxs.z, p.z)
        end
        ntri = ntri + #g.triangles / 3
    end
    local size = maxs - mins
    P("total tris", ntri, "extent", tostring(size))
    -- planar fake UV2 along model X (front-to-back)
    for gi, g in ipairs(groups) do
        local base = Material(g.material)
        local bt = base:GetTexture("$basetexture")
        local m = CreateMaterial("proplm_proto_" .. gi .. "_" .. os.time(), "UnlitGeneric", {
            ["$basetexture"] = bt and bt:GetName() or "dev/dev_measuregeneric01",
            ["$detail"] = rt:GetName(), ["$detailscale"] = 1, ["$detailblendmode"] = 0, ["$detailblendfactor"] = 1,
            ["$separatedetailuvs"] = 1, ["$nocull"] = 1,
        })
        local im = Mesh(m)
        mesh.Begin(im, MATERIAL_TRIANGLES, #g.triangles / 3)
        for _, v in ipairs(g.triangles) do
            mesh.Position(v.pos); mesh.Normal(v.normal)
            mesh.TexCoord(0, v.u, v.v)
            mesh.TexCoord(1, (v.pos.x - mins.x) / size.x, (v.pos.y - mins.y) / size.y)
            mesh.AdvanceVertex()
        end
        mesh.End()
        meshes[#meshes + 1] = { mesh = im, mat = m }
    end
    P("build ms", (SysTime() - t0) * 1000)
end

local frames, started = 0, false
hook.Add("InitPostEntity", "proplm_proto", function()
    timer.Simple(6, function()
        fill_rt()
        local ply = LocalPlayer()
        local ang = ply:EyeAngles(); ang.p = 0; ply:SetEyeAngles(ang)
        ent = ents.CreateClientProp(MODEL)
        ent:SetPos(ply:EyePos() + ang:Forward() * 260 - Vector(0, 0, 40)); ent:SetAngles(Angle(0, ang.y + 90, 0)); ent:Spawn()
        build(MODEL)
        local calls = 0
        ent.RenderOverride = function(self)
            calls = calls + 1
            local m = Matrix(); m:SetTranslation(self:GetPos()); m:SetAngles(self:GetAngles())
            cam.PushModelMatrix(m)
            for _, e in ipairs(meshes) do render.SetMaterial(e.mat); e.mesh:Draw() end
            cam.PopModelMatrix()
        end
        timer.Simple(3, function() P("RenderOverride calls so far", calls) end)
        started = true
    end)
end)
hook.Add("PostRender", "proplm_proto_cap", function()
    if not started then return end
    frames = frames + 1
    if frames == 40 then
        file.Write("proplm_proto.png", render.Capture({ format = "png", quality = 100, x = 0, y = 0, w = ScrW(), h = ScrH() }) or "")
        P("captured"); P("DONE")
    end
end)
