# proplm: second-UV prop lightmaps

Agent-facing design and format notes. Tracking issues: #54 (umbrella), #59 (Lua addon), #60 (compiler half).

## Why

The stock per-texel prop lighting (`texelslighting_N.ppl`) is sampled at the model's only UV set (UV0, after
`$basetexturetransform`), so models with overlapping UVs get corrupted lighting. The proplm profile gives opted-in props
a second UV set (UV2) computed at compile time. A universal GMod Lua addon (`game/addons/proplm`) draws those props with
albedo on UV0 and the baked lightmap on UV2 using stock shaders. Probe results are in issue #55.

## Opt-in

A `prop_static` / `static_prop` with keyvalue `proplm "1"` is converted by VBSP. Other props are untouched.
`lightmapresolutionx/y` are read as usual; the larger of the two (default 256 when both are missing or 0, clamped to
16..2048, rounded up to a power of two because VRAD writes a VTF) is the square resolution `proplm_res`. All instances of
one model share one UV2 layout, packed for the first instance's resolution; no instance gets a smaller `proplm_res` than
that, so chart padding stays at least two texels. A layout whose charts do not fit that size (models with very many charts
at a low resolution) is packed at the next power of two that fits, and every instance of the model then uses at least that.

## What VBSP does (src/utils/vbsp/proplm_uv2.cpp)

1. For each distinct opted-in model (keyed by lowercase name, so `Props_Canal/Boat001A.mdl` and the lowercase spelling are
   one model): read MDL, VVD (with fixups), `.dx90.vtx` (fall back to `.dx80.vtx`), walk LOD0 trilist strips of model 0
   of each bodypart (the default bodygroup; matches `util.GetModelMeshes(model, 0, 0)`). The model need not be
   `$staticprop`: it ends up on a `prop_dynamic`. Strips that are not trilists are skipped.
2. Run xatlas on positions and normals (geometry charts, no input UVs; `maxIterations` 4, padding 2, bilinear) to get
   UV2, packed into exactly `res` x `res` texels at one uniform texel density. xatlas returns triangles in input order,
   so lump triangle t is the t-th trilist triangle in the walk above. A triangle xatlas cannot chart (zero area or
   collapsed vertices) is dropped from the lump, not kept with zero UV2; the lump then holds fewer triangles than the
   LOD0 walk, and VBSP prints how many it dropped. Nothing downstream depends on VTX order, only on the lump's own
   triangles.
3. Write the `plm2` game lump (format in `src/public/proplm.h`). One entry per model, shared by all instances. Model
   and material names are lowercase. The material of a triangle is the mesh's material for skin family 0 (a skin that
   swaps materials does not change the triangle set). The mesh's `material` field is an index into the skinref table,
   and the name is the texture that table's first row (skin family 0) points at. This differs from the older SMD writer
   in `staticprop.cpp`, which indexes the texture list with `mesh->material` directly; the two agree only when skin
   family 0 of the skinref table is the identity. A name of 96 characters or more does not fit the lump: it is truncated
   to 95 (two long names with the same first 95 characters merge into one entry) and VBSP prints a Warning once per such
   name. `m_PosChecksum` is the standard CRC-32 (zlib `crc32`) of the little
   endian int32 array x0 y0 z0 x1 y1 z1 ... over the corners of all kept triangles in lump order, each coordinate
   `floor(coordinate * 16 + 0.5)` (computed in double).
4. Rewrite the prop's entity into a `prop_dynamic` in place (it stays in the entity lump and is not added to the
   static-prop lump): same model, origin, angles, skin; `solid` is read the way the static prop path reads it
   (`IntForKey`, so a missing key is 0): a missing key or `solid "0"` gives `solid "0"` (Not solid), any other value gives
   `solid "6"` (VPhysics); plus `proplm "1"`, `proplm_id` (0, 1, 2, ... in entity order across the map), `proplm_lm`
   (`maps/<mapname>/proplm/<id>`, mapname lowercase) and `proplm_res`. Other keys (`lightmapresolutionx/y`, fade keys)
   are left as they were. Without the addon the prop still renders as an ordinary dynamic prop. Any failure (model,
   VVD or VTX unreadable or mismatched, no triangles, xatlas cannot fit) prints one Warning per prop, naming the model,
   the prop's origin and the reason, and leaves the entity an ordinary static prop; note that stock VBSP then refuses
   models that are not valid static props.
   Works in a full compile and in `-onlyents` (which regenerates the entity lump and the `plm2` lump together; a `plm2`
   lump from an earlier compile is replaced, or removed when no prop asks for proplm any more). It does not work in
   `-onlyprops`: that mode writes the static prop lump but never the entity lump, so a newly marked prop would be skipped
   from the static props and never appear as an entity, and a prop whose proplm was removed would come back as a static
   prop beside its old prop_dynamic. VBSP therefore stops with an Error under `-onlyprops` when a static prop in the VMF
   has `proplm "1"` or the loaded BSP still holds a `plm2` lump; run a full compile or `-onlyents` instead.
5. The `plm2` lump must stay uncompressed: `CompressGameLump` (src/utils/common/bsplib.cpp, the code behind a compress
   callback / repack) skips it and clears a compressed flag if an earlier pass set one. That only protects tools built
   from this repo's bsplib; a stock `bspzip -repack` has its own copy and can compress the lump (game lump flag 1).

## Lump facts for consumers (VRAD, Lua addon)

* Lump id is the int `'plm2'`; on disk the four bytes read `2mlp`, as `sprp` reads `prps`. Flags 0, game lump version 1,
  and `ProplmHeader_t.m_Version` is 1.
* Corners are in VTX index order, which is clockwise seen from outside (Source's front face): `(v1-v0) x (v2-v0)` points
  into the model. UV2 winding is not consistent: xatlas rotates charts by transposing them, which mirrors them. A
  rasterizer must not depend on the sign of the triangle's UV2 area.
* UV2 is in [0,1]; the lightmap row is `v * res`, v not flipped, texel centers at `(i + 0.5) / res` (this is xatlas's own
  convention). Normals are unit length from the VVD, `m_UV0` is the VVD texcoord untouched.
* Charts are separated by at least two texels at the model's atlas resolution plus xatlas's bilinear border, so a
  lightmap sampled at that resolution or higher does not blend neighbouring charts. Texel density is roughly uniform
  across charts (small charts are stretched a little to whole texels), so the texel footprint of a triangle scales with
  its 3D area.

## What VRAD does (src/utils/vrad/vradproplm.cpp)

`ComputePropLmLighting()` runs from `VRAD_ComputeOtherLighting`, right after the static prop lighting but on its own
condition: it bakes whenever at least one `prop_dynamic` has `proplm "1"`, with or without `-StaticPropLighting`, because
VBSP has already turned those props into entities and the static prop lighting never sees them. It does nothing, and does
not read the lump, when no entity asks for it. It does not bake under `-fast`; in that case, and when the `plm2` lump is
missing or unreadable, VRAD prints one Warning naming the number of proplm props and saying their lightmap VTFs were not
written (the addon must cope with a map that has no VTFs).

For each `prop_dynamic` with `proplm "1"`:

1. Look the entity's `model` up in the `plm2` lump (lowercase, forward slashes). A missing model, an empty `proplm_lm`, a
   `proplm_res` outside 1..2048 (the cap VBSP applies) or a duplicate `proplm_lm` is a Warning and that entity is skipped.
   A short or corrupt lump is a Warning and nothing is baked.
2. Instance transform from `origin` and `angles` (`pitch yaw roll`), same `AngleMatrix` the static props use: positions
   rotate then translate, normals rotate only. `modelscale` is not honored (see Known limits).
3. Rasterize every lump triangle over UV2 at `proplm_res` x `proplm_res`. Texel (x, y) has its center at
   ((x+0.5)/res, (y+0.5)/res); row = v * res, no v flip (row 0 is the top row of the VTF, as for the `.ppl` over UV0).
   A texel is covered when its center lies inside a triangle (tiny edge slack); the first triangle to claim a texel keeps it.
   World position and normal come from the corner data by barycentric interpolation, then the instance transform.
4. Light every covered texel with `ComputeTexelLightingAtPoint` (vradstaticprops.cpp): direct light, plus indirect when
   `-bounce` is at least 1. This is the one sampler behind both the `.ppl` and proplm texels. World geometry shadows it;
   these props are not in the tracer, so they neither shadow anything nor themselves (see limits). Rows are spread over the
   worker threads.
5. Encode with the stock `.ppl` path (`EncodeTexelsRGB888`: 3x3 box filter over covered texels, then
   `ConvertLinearToRGBA8888`), dilate: uncovered texels within 4 texels of a covered one take the average of their filled
   neighbors, so bilinear filtering never blends in black. Then halve every byte, rounded to nearest (`(v + 1) >> 1`).
   Stored values never exceed 128; consumers multiply by two.
6. Write `materials/<proplm_lm>.vtf` to the pak: RGB888 (Source enum value 2, bytes in memory order R, G, B; BGR888 is 3),
   `res` x `res`, one frame, exactly one mip (the mip count is forced; VTF Init would build a full chain even with NOMIP),
   flags `NOMIP | NOLOD | CLAMPS | CLAMPT`, not sRGB. NOMIP: one image, no mip chain. NOLOD: the engine's texture-quality
   setting never drops it to a lower resolution. CLAMPS/CLAMPT: charts never wrap to the opposite edge. The VTF has no
   low-res thumbnail. A covered-texel count of zero writes nothing.

No `.ppl` is written for these props; they are not in the `sprp` lump, so the static prop code never sees them.

## Known limits

* LOD0 and default bodygroup only. Skins change only the material, not geometry, so they work.
* Converted props no longer occlude light as static props do: VRAD does not add `proplm` props to the tracer, so they cast
  no shadows on the world or on each other, and a concave prop does not shadow itself. (Adding them as occluders would
  mean feeding their triangles through the static prop ray-trace path and skipping the prop's own id when lighting it.)
* LDR: lightmap values above 2.0 clip.
* Static prop keyvalues that are not honored on a converted prop: `ignorenormals`, `disableselfshadowing`,
  `generatelightmaps` and `modelscale` (VBSP and VRAD read none of them; the stock static prop path turns the first
  three into flags in `EmitStaticProps`). They are tracked in a follow-up issue; see issues.

## Delegation log

* scout (Haiku): code facts for VBSP/VRAD prop paths, build state; wrote findings inline, no escalation.
* impl (Sonnet) VBSP half: built and tested, 6 mutants caught; no escalation.
* impl (Sonnet) VRAD half: built and tested, 6 mutants caught; no escalation.
* reviewer (Haiku): found the `-onlyprops` entity loss (major) and the silent VRAD gate (medium), both real.
* review_fast round 1 and 2 (shop26, glm-flash): round 1 low only; round 2 shop26 approve (glm-flash still running at commit).
* impl (Sonnet) review fixes: all seven applied and verified; one doc key name corrected (`ignorenormals`).
