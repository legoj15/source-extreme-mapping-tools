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
   from this repo's bsplib; a stock `bspzip -repack -compress` has its own copy and compresses the lump (game lump flag 1),
   which the Lua addon cannot read. That command also compresses every BSP lump, and the compile tools refuse such a map
   with an Error naming the cause. A map whose game sub-lumps alone are LZMA-compressed loads: `ParseGameLump`
   (src/utils/common/bsplib.cpp) inflates them, so VRAD bakes from it and writes them back uncompressed (#63).

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
   are scaled by `modelscale` (missing, zero, negative or non-finite: 1, with a Warning for a bad value), then rotate and
   translate; normals rotate only.
3. Rasterize every lump triangle over UV2 at `proplm_res` x `proplm_res`. Texel (x, y) has its center at
   ((x+0.5)/res, (y+0.5)/res); row = v * res, no v flip (row 0 is the top row of the VTF, as for the `.ppl` over UV0).
   A texel is covered when its center lies inside a triangle (tiny edge slack); the first triangle to claim a texel keeps it.
   World position and normal come from the corner data by barycentric interpolation, then the instance transform.
4. Light every covered texel with `ComputeTexelLightingAtPoint` (vradstaticprops.cpp): direct light, plus indirect when
   `-bounce` is at least 1. This is the one sampler behind both the `.ppl` and proplm texels, called with the instance's
   skip id and flags the way the static props are: `ignorenormals 1` gives `GATHERLFLAGS_IGNORE_NORMALS`, and the prop
   skips its own triangles only with `disableselfshadowing 1` or `-nossprops`. Rows are spread over the worker threads.

Shadows (#61): during scene setup, right after the static props, `PropLm_AddPolysForRayTrace` adds every instance's lump
triangles (same transform) to the ray tracer as `TRACE_ID_STATICPROP | (0x800000 + job index)`, an id range above any
static prop index, unless the entity has `disableshadows 1`. So converted props shadow the world, static props, each
other and themselves as the static props they replaced did. They are added as triangles whether or not `-StaticPropPolys`
is given (the lump holds no collision hull), opaque (no `$alphatest` texture shadows), and also under `-fast` (only the
bake is skipped). The job list (lump parse and per-entity checks, with their Warnings) is built once for both uses.
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
* Converted props occlude light as opaque triangles: texture shadows (`-textureshadows` on alpha-tested materials) do
  not apply to them.
* LDR: lightmap values above 2.0 clip.
* Static prop keyvalues honored on a converted prop (VRAD reads them from the entity, #61): `ignorenormals`,
  `disableselfshadowing`, `disableshadows` and `modelscale`. Not honored: `generatelightmaps` (a converted prop always
  gets its lightmap; not tracked). The addon leaves a prop with `GetModelScale` other than 1 stock, so a scaled bake is
  only used for its shadows.
* `lightingorigin` is ignored on a converted prop: VRAD lights each texel at its own position. VBSP prints one Warning per
  prop that sets it.

## Rebake route (issue #57; src/utils/vbsp/proplm_rebake.cpp)

A second way to fix overlapping UVs that needs no addon: VBSP ships a copy of the model whose one UV set no longer overlaps,
with its textures re-laid to match, and the prop stays a static prop. Stock `vrad -StaticPropLighting` then writes a correct
`texelslighting_N.ppl` and a stock engine draws it. VRAD finds the copy because it adds the BSP itself to the GAME and MOD
search paths (vrad.cpp), so the pak's files load like loose ones.

### Opt-in

* `proplm "rebake"` on a `prop_static`. With `generatelightmaps 0`, or no `generatelightmaps` key (read as 0), VBSP warns
  and skips it (there is no lightmap to fix).
* `-proplmrebake` on the VBSP command line: every static prop with `generatelightmaps 1`, except ones marked `proplm "1"`
  (those take the UV2 route).
* A model whose UVs do not overlap when folded into one tile is left alone ("needed no rebake"). A model that cannot be
  rebaked keeps its original model; VBSP prints one Warning per prop with the model, origin and reason. The compile
  never stops.
* With `-forcedynamicpropsasstatic`, VBSP patches a model that lacks the `$staticprop` flag before the rebake reads it,
  in one of two modes. Fallback mode (flag patching, when the model has no `.dx90.vtx` or decompile mode fails; a model
  without a usable VVD is dropped as a static prop before either mode runs) puts the flag-patched MDL, a rotated VVD, the `.dx90.vtx` and the `.phy` (those it has) into the pak under the original name;
  the rebake reads the pak first, so it rebakes that patched model when the files it needs exist. Decompile
  mode recompiles the model as a `<name>_static.mdl` under `maps/<map>/`, and the dictionary entry names that; the rebake
  still reads the original MDL, refuses it (no `$staticprop` flag), and the prop keeps the patched `_static` model; the
  Warning then says "keeps the patched model from -forcedynamicpropsasstatic".
* The work is done once per model, the first time a prop VBSP keeps (it passed the `allowstatic` check) asks for it.

### What VBSP writes (all into the pak, uncompressed)

* `models/maps/<map>/rebake/<rel>.mdl`, `.vvd`, `.dx90.vtx`, `.dx80.vtx`, and `.phy` if the source has one; `<rel>` is
  the model path under `models/` without `.mdl`. The static prop lump's dictionary names this copy.
* `materials/maps/<map>/rebake/<rel>/<texture name>.vmt` and the rebaked `.vtf`s it points at. Every model gets its own
  copies, even of a texture another rebaked model also uses.
* Nothing goes into the pak until every step that can fail has passed (the lower-LOD vertex copies, VTX LOD switching,
  the MDL rewrite, the read-back check of the rewritten files and every VTF encode run into memory first), so a model
  that fails leaves no bytes behind.
* One log line per model (islands, overlap before and after, layout, LOD handling with the triangle count of the least
  detailed LOD still drawn by distance, e.g. "LOD 4: 106 triangles", textures, bytes) and a map total.
  The overlap figure is "UV overlap folded into one tile": the share of covered cells (folded mod 1) that the shared
  detector counts as overlapped, in any of its categories (see "UV overlap detector" below).

### How

1. Islands: LOD0 triangles of every model of every bodypart (every strip must be a triangle list, or the model fails),
   grouped by the shared detector's rule (below; the vertex identity is the raw VVD slot).
2. Overlap is measured by the shared detector at 512 cells per UV unit. An island whose fold claims (a second, third ...
   of its own triangles on one unfolded cell; k triangles count k-1) exceed 1% of its unfolded cells, and number more
   than two, fails: moving islands cannot fix it. This is the rule the rebake used before the shared detector, kept so the
   decisions did not change with it. A model with no repeat or other-island overlap needs no rebake (a fold under that
   limit stays as it is); that too is the earlier rule.
3. Layout: islands are only translated. They are shelf-packed (deterministically) into N x M texture repeats, N and M
   powers of two at most 2:1 apart, and square when any material has `$detail`. Offsets are whole multiples of 4 texels of
   every texture, so texels copy exactly and DXT blocks line up. Padding on each side is the larger of 4 texels of the
   coarsest texture and 2 lightmap texels at the smallest lightmap size of the model's rebake props. A prop without
   `lightmapresolutionx/y` is padded as if it were 32: that is the value Hammer's FGD fills in, a guess, since VRAD itself
   has no default for a missing key.
   New UV = (old UV + offset) / layout size, inside 0..1. The lightmap covers the whole layout, so an N x M layout gives
   the model N x M times fewer lightmap texels than its `lightmapresolutionx/y` implies: the fridge's 2x2 is lit at a
   quarter of them. Mappers should raise `lightmapresolutionx/y` by the layout size per axis (the log names it) to keep
   the same density. A layout of 2x1 repeats squeezes UVs twice as much along one axis, so on a square lightmap the
   texels become up to 2:1 non-square in world space.
4. Textures: each UV0 texture is rebuilt for the layout. That is `$basetexture`, `$envmapmask`, `$phongexponenttexture`,
   `$selfillummask` and `$ambientoccltexture` through `$basetexturetransform`, plus `$bumpmap` and `$normalmap` through
   `$bumptransform`. The textures of an island are those of its LOD0 triangles plus those of the lower-LOD triangles
   that land in it and stay drawn (a LOD before the first switched-off one, or a shadow LOD, which stays on). A
   texture-table entry that no LOD0 or drawn lower-LOD triangle uses, in any skin family, gets no rebuilt VTF and no
   checks; its source VMT is copied unchanged into the rebake folder, so every entry still resolves under the copy's one
   cdtexture (VRAD reads texture 0's reflectivity). No copy is written when a used entry has the same name (its rebuilt
   VMT owns that path) or when the source VMT cannot be read. Each island's padded box is copied from the source with the
   source's own addressing per axis: wrapped, or clamped to the edge texel where the VTF has `CLAMPS` / `CLAMPT` (an island
   whose source UVs run past 0..1 on a clamped axis then repeats the edge texels, as the engine drew it, and so does its
   padding). The copy is texel-exact (box filtered when scaled down), or bilinear when the material has a transform, which
   is then baked in and dropped from the VMT. Empty texels get 8 rings of
   dilation, then the mean colour. The output keeps the source's texel density. It is scaled down by powers of two (at
   most 8x) when it would exceed 8192 per side, and the log says so. It is re-encoded to the source format (DXT1 and DXT5
   go through the S3TC encoder; a format it cannot write falls back to BGRA8888/BGR888 with one warning), with the full
   mip chain unless NOMIP. An encoded VTF that is byte-identical to one an earlier rebake in the same compile wrote is
   not added again: the material names the earlier file, and the log line says how many were shared (#68). Matches are
   picked by CRC and length and confirmed by a full compare. Measured: on stock content no two distinct models share
   (gear01 and gear02 use the same source texture but lay out different islands, so their VTFs differ); a renamed copy of
   gear01 shares its 349,632-byte VTF.
5. VMT: a patch-expanded copy with its fallback blocks (one level: a fallback block that holds a further block, other
   than `Proxies`, fails the material). The proxy and transform checks run on the root (the
   material's own keys) and on every fallback block. Every texture key a block sets is rebuilt through that block's
   effective transform (its own `$basetexturetransform` or `$bumptransform`, else the root's, else identity), repointed,
   and every transform key is then dropped from every block. Because a dropped transform must already be baked into the
   texture it applies to, these fail the material with a per-prop Warning:
   * A block's own `$envmapmask` whose effective `$envmapmasktransform` (its own, else the root's, else identity) differs
     from that block's effective `$basetexturetransform`; the mask is rebuilt through the base transform.
   * A block that sets its own `$basetexturetransform` or `$bumptransform`, differing from the root's, but does not set a
     texture of that group the root sets (`$basetexture`, `$phongexponenttexture`, `$selfillummask`,
     `$ambientoccltexture`; `$bumpmap`, `$normalmap`): that texture is rebuilt once, through the root's transform.
   * A block that sets its own `$envmapmasktransform`, differing from the root's `$basetexturetransform`, but inherits the
     root's `$envmapmask`, which is rebuilt through the root's base transform.
   A block whose own transform equals the root's, while inheriting the texture, is fine: the texture was rebuilt through
   that same transform. `$detailscale` is multiplied by the layout size in every block that has it. The root gets
   4 x layout size whenever it has no `$detailscale` of its own and any block has `$detail`, so a block without its own
   inherits a scaled value too. The `$detail` texture is not rebuilt (it tiles at
   its own scale), so its phase can shift across an island seam where the base texture is continuous. Keys known not to
   read UV0 (`$detail`, `$envmap`, `$lightwarptexture`, `$phongwarptexture`, `$flashlighttexture`, the editor-only `$tooltexture`) are left as they are.
   Any other `$` key in any block whose value names an existing VTF under `materials/` (`$basetexture2`,
   `$blendmodulatetexture`, `$selfillumtexture` and the like) fails the material: the rebuilt UVs would read that
   texture at the wrong place, and the rebake does not rebuild it (#69). Refusing is the default for keys it does not know.
6. Model: when no lower-LOD triangle spans two islands (step 7), the VVD is copied with new UVs only: positions,
   normals, tangents, fixups and vertex count are unchanged. Otherwise the VVD, the MDL's vertex counts and offsets and
   both VTX are rewritten for the added vertices (step 7). Positions, normals, bone weights and tangents of every source
   vertex are copied unchanged either way (a translation does not change the UV derivatives), so MDL, VVD and VTX keep
   the source's checksum. The MDL copy gets the new name (in `name` and in studiohdr2) and one cdtexture, the rebake
   directory. The PHY is copied unchanged.
7. Lower LODs: their vertices follow LOD0's islands. Matching tries, in order: position, UV and handedness; position and
   UV; the island of a LOD triangle the vertex shares; then UV alone. A lower-LOD triangle whose corners end up in two
   islands would stretch across the layout, so it is drawn in one island instead (#67): of its corners' islands, the
   one whose source UV box grows least by taking the whole triangle, then the one most corners move with, then the
   lowest index. Each corner that moves with another island gets a copy of its VVD vertex (UV = source UV plus the
   chosen island's offset, like that island's own vertices), and the island's box, which the layout packs and the
   textures fill, grows to hold it. VVD vertices are shared by all LODs, which is why copies are needed:
   * VVD: a copy joins its mesh's vertex list among the vertices of its level, the highest LOD that draws it. Each mesh
     list stays sorted by level, highest first, because that is where `numLODVertexes` cuts when `r_rootlod` drops the
     detailed LODs (`Studio_SetRootLOD`). The vertex block is written in the new order, so no fixup table is needed
     unless the source had one or the meshes' levels do not fall across the whole list; then one is written with one
     entry per run of equal level. `numLODVertexes` (header and every mesh) count the copies of level >= that LOD.
   * MDL: per mesh `numvertices`, `vertexoffset` and `vertexdata.numLODVertexes`; per model `numvertices`, `vertexindex`
     and `tangentsindex`. Nothing moves in the file.
   * VTX (both copies): every vertex of every LOD gets its renumbered `origMeshVertID`. A strip group whose drawn
     triangles need copies gets new vertices (one per VTX vertex and island, the source entry with the copy's ID),
     inserted right after its vertex table; the file's relative offsets are recomputed for the bytes that moved, and the
     strip's vertex range is widened to cover them. So a VTX grows by 9 bytes per added vertex.
   * Before anything goes into the pak, the rewritten files are read back through the same checks as the source
     (`Proplm_ValidateMdl`, `Proplm_LoadVvdVertices`, every LOD of both VTX), LOD0 must have the source's triangles,
     positions and the new UVs, and every drawn lower-LOD triangle must lie in one island; a failure fails the model.
   Shadow LODs (switch point < 0) are split the same way. A model this cannot rewrite keeps the old behaviour, with the
   reason in the log line ("not split: ..."): the first LOD with a spanning triangle, and every LOD after it, gets switch
   point 1e7 in both VTX copies (shadow LODs stay on and stretched). That happens when the VVD and VTX LOD counts
   differ, a mesh has flexes, models or meshes are not packed in order in the vertex list, a mesh's per-LOD vertex
   counts are not sorted, source fixups do not follow the meshes' counts, a lower-LOD strip is not a triangle list, two
   strip groups share a table, a VTX table is out of range, or a mesh or strip group would pass 65536 vertices. The
   switch point is what keeps those LODs from being picked by distance; LOD0 is then drawn at every distance.
   `numAllowedRootLODs` is also capped at that first switched-off LOD, so `r_rootlod` cannot make a switched-off LOD
   the root (with LOD 1 off, the root is pinned to LOD 0). That cap does not disable any LOD by itself. A lower LOD that
   has a VTX material replacement entry is switched off the same way (its replacement materials are not rebuilt); the
   LODs before it are still split. A replacement entry on LOD0, or on a shadow LOD (which cannot be switched off),
   fails the model.
   Added copies grow island boxes, so the layout can re-pack: LOD0 texcoords of a model with copies can differ from a
   build without them by one translation per island. A lower-LOD triangle reaching into a part of the box that no LOD0
   triangle covers samples the texture correctly there (the whole box is filled from the source), but VRAD only lights
   lightmap texels that LOD0 triangles cover plus one ring around them (vradstaticprops.cpp), so its lightmap there is
   unlit. The same holds for any lower-LOD triangle whose UVs leave LOD0's coverage, copies or not.
8. Bounds, in order (`Proplm_ConvertMdl`, then `Proplm_ValidateMdl`, both in proplm_uv2.cpp): the header size and the
   studio version (44..48) are checked first. For versions 44..47 the local animation descriptor table
   (`numlocalanim` at `localanimindex`), the only table `Studio_ConvertStudioHdrToNewVersion` walks and writes, is checked
   against the file before that conversion runs; for 48 the conversion does nothing. Then every offset and count the
   routes follow in the MDL (textures and their names, cdtextures, skin families, bodyparts, models, meshes, studiohdr2),
   the VTX (every table down to strip indices, the LOD count against the header's) and the VVD is checked against the file
   size with 64-bit arithmetic before it is dereferenced. A VVD vertex whose position, normal or texcoord is not finite,
   or (rebake only) whose texcoord magnitude exceeds 1024, fails the model. The UV2 route runs the same code, without the
   texcoord limit. Stock VBSP code reads the MDL before the rebake runs: `LoadStudioModel` (staticprop.cpp) now calls
   `Proplm_ConvertMdl` too, so a bad version or animation table there is a Warning and an unloadable model, not a crash.
   Stock code still crashes on other corrupt input before these checks, such as a NaN VVD position in the collision
   hull built by vphysics (#70).

### Fails (the prop keeps its model and VBSP warns)

An island that overlaps itself. Studio version outside 44..48, `$includemodel`, or external animation blocks. An MDL not
compiled as a `$staticprop` (with `-forcedynamicpropsasstatic` in decompile mode the prop keeps the patched `_static`
model; see Opt-in). A strip that is not a triangle list. A VTX material replacement entry on LOD0 or on a shadow LOD.
A table or offset out of range, a non-finite VVD vertex, or a texcoord beyond +-1024. A material with a texture-animating proxy (`TextureScroll`, `TextureTransform`,
`AnimatedTexture`, `AnimatedOffsetTexture`, `TextureToggle`) in any material whose VMT is rebuilt (unused entries copied
unchanged are not checked), a fallback block nested inside another, one of the fallback-block transform cases in
step 5, or a texture key the rebake does not rebuild (step 5). A source VTF that is animated, a cubemap, a volume texture, or in
an unhandled format. An island spanning more than 16 repeats. A layout that does not fit 32 x 32 repeats within 8192
texels. A copy name of 128 characters or more (the static prop lump's limit), or a rebaked texture or material path that would not
fit 260 characters (texture names keep a 32-character reserve for the uniquing suffix). A lower-LOD vertex that cannot
be placed. Rewritten files (lower-LOD copies, step 7) that fail their read-back check, which would be a rebake bug.

Not a failure: a map name longer than 63 characters. The copies go under the first 63 (VBSP's `mapbase`, which its
other per-map pak paths use too), with one Warning.

### Measured (HL2DM content, staticpropbounce fixture, 2026-11-12)

All four in one compile (`.agents/scripts/proplm_rebake_ab.py`). "Lower LODs" is what the copy draws; "before #67" is the
build before lower-LOD splitting, which switched every lower LOD off on all four. The last LOD's triangle count is over
every model of every bodypart.

| model | islands | UV overlap folded into one tile, before -> after | layout | lower LODs | texture | pak bytes (before #67) |
|---|---|---|---|---|---|---|
| props_wasteland/kitchen_fridge001a | 89 | 36.9% -> 0.0% | 2x2 | all 1 kept; 52 spanning tris, 40 vertices added; LOD 1: 245 tris | prisonkitchen01 1024x1024 DXT5 | 1,482,065 (1,478,785) |
| props_wasteland/gear01 | 48 | 65.5% -> 0.0% | 2x1 | all 4 kept; 14 spanning, 7 added; LOD 4: 106 tris | quarryobjects01 1024x512 DXT1 | 453,443 (452,707) |
| props_wasteland/gear02 | 50 | 31.6% -> 0.0% | 2x1 | all 4 kept; 7 spanning, 5 added; LOD 4: 126 tris | quarryobjects01 1024x512 DXT1 | 430,562 (430,062) |
| props_wasteland/prison_wallpile002a | 89 | 93.8% -> 0.0% | 1x1 | all 4 kept; 84 spanning, 49 added; LOD 4: 102 tris | prison_objects002 512x512 DXT5 | 457,888 (453,294) |

The extra bytes are the added vertices (48 + 16 bytes each in the VVD, 9 per strip group vertex in each VTX) and VTFs
whose islands moved; layouts did not change size. A scratch check of the output (every drawn index in range and inside
the mesh's `numLODVertexes` for its LOD, every root LOD 0..4 simulated through `Studio_LoadVertexes` /
`Studio_SetRootLOD` reading the same vertex bytes as root 0, no drawn lower-LOD triangle across two islands, LOD0
positions, normals and tangents byte-identical to the build before, PHY identical) passed on all four. Lower-LOD texel
samples landing on lightmap texels VRAD does not light (more than one texel from any LOD0 triangle) at 512x512: 0.1% to
4.9% per LOD, the most on the least detailed LODs. No in-game check of the kept LODs has been run.

The overlap column is the shared detector's share of distinct covered cells (#56). Before #56 the rebake counted extra
claims over all claims and logged 28.5% / 54.1% / 31.6%; the pak output is byte-identical under both. VRAD, which now
runs the same detector at the prop's lightmap size, reports no overlap for the rebaked copies (its old detector still
flagged one fridge texel lying exactly on a creased shared edge). The `boat001a` fixture prop cannot be used here,
because its prop_data lacks `allowstatic` and VBSP refuses it as a static prop.

VRAD change that affects every static prop, not only rebaked ones: `Rasterizer::Build` (vradstaticprops.cpp) now skips a
triangle with zero UV area. Before, such a triangle's barycentrics came out inf/NaN and the texels along its line tested as
inside it, overwriting their owners' samples and counting as overlap. Any prop that has such triangles lights differently
from stock VRAD.

The island rule was measured on HL2 props: the whole-edge rule splits islands that the older any-matching-corner rule
merged, for example `props_wasteland/prison_wallpile002a` 83 -> 89 islands (rebaked: 78.5% -> 0.0%, 1x1 layout, 453,279
bytes) and `props_wasteland/lighthouse_stairs` 372 -> 432. The three models above have the same island counts under both
rules.

In GMod x64 (dxlevel 95, 1280x720), with `mat_fullbright 1` on maps without `-StaticPropLighting`, the rebaked props match
the originals pixel for pixel (mean difference 0.19 / 0.00 / 0.01 per channel). Lit with `-StaticPropLighting`, the
original fridge renders black (its lightmap is corrupted by the overlap), while the rebaked one shows the expected light
falloff. Note that `mat_fullbright` does not remove a static prop's per-texel lightmap; build a map without
`-StaticPropLighting` to compare textures alone.

Guard: `game/bin/x64/unit_test/proplm/test_proplm_rebake.py` checks checksums, that LOD0 has the source's triangles
in VTX order with untouched positions, zero overlap between islands, texel match after DXT decode, and that VMT/VTF are
in the pak.

## UV overlap detector (issue #56; src/utils/common/uvoverlap.cpp)

One detector for VBSP (the rebake's need / can / did checks) and VRAD (the per-model warning and report of the static
prop lighting, `AnalyzeUVOverlap` in vradstaticprops.cpp). It lives in `utils/common`, the only tree both tools compile,
and reads no files: VBSP feeds it from the bounds-checked `proplm_shared.h` readers, VRAD from the SDK studio accessors
(LOD0 trilists of every bodypart and model, `.dx80.vtx`). Rules, also in `uvoverlap.h`:

* Islands: triangles sharing a vertex are one island; across a hard edge two triangles join only when they share a whole
  edge whose endpoints match bit for bit in position, UV and tangent handedness (so mirrored halves stay apart, and two
  islands touching at one UV point stay apart).
* Raster: a cell (resX x resY per UV unit) belongs to a triangle when its center is inside. A center exactly on an edge
  goes to one side only: edge functions are evaluated with the endpoints in a fixed order, so both triangles of a shared
  edge get exactly opposite values, and a tie goes to the triangle whose counter-clockwise edge points up (or exactly
  towards -x). Neighbours that meet along an edge or at a corner never share a cell, so no neighbour exemption is needed;
  a triangle folded back over its neighbour does share cells and counts. Zero-UV-area triangles cover nothing.
* Cells are folded into one tile (the lightmap wraps). Categories per cell: fold (two triangles of one island on the same
  unfolded cell), repeat (one island, or one triangle wider or taller than a tile, at two unfolded places), other (two
  islands). The report: covered cells, overlapped cells and their fraction, per-category counts, spanning and zero-area
  triangles, and per island its triangles, cells, fold / repeat / other cells, spanning triangles and UV bounds.
* VRAD measures at the prop's lightmap size, capped at 2048 per axis; a triangle spanning more than 64 tiles, or a layout
  whose raster would visit more than 2^28 cells, is not measured (one Warning). The rebake measures at 512 and fails an
  island spanning more than 16 repeats.

`vrad_rtx -game <dir> -uvoverlap <model.mdl | mesh.uvmesh> [-uvoverlapres N]` prints the report for one model and exits
(no map). A `.uvmesh` is plain text: `v x y z u v [sign]` and `f a b c` lines. Guard:
`game/bin/x64/unit_test/proplm/test_proplm_uvoverlap.py` compares the report on five hand-made meshes (`uvoverlap/`) with
an exact rational-arithmetic reference of the same rules, and checks the fridge gives 89 islands as the rebake does.

## Delegation log

* scout (Haiku): code facts for VBSP/VRAD prop paths, build state; wrote findings inline, no escalation.
* impl (Sonnet) VBSP half: built and tested, 6 mutants caught; no escalation.
* impl (Sonnet) VRAD half: built and tested, 6 mutants caught; no escalation.
* reviewer (Haiku): found the `-onlyprops` entity loss (major) and the silent VRAD gate (medium), both real.
* review_fast round 1 and 2 (shop26, glm-flash): round 1 low only; round 2 shop26 approve (glm-flash still running at commit).
* impl (Sonnet) review fixes: all seven applied and verified; one doc key name corrected (`ignorenormals`).
* impl (Sonnet) Lua addon: built, boat 1044/1044, brightness calibrated in-game; two brief assumptions (UnlitGeneric, MapCreationID) were wrong and the agent corrected them.
* reviewer (Haiku) + review_fast (shop26, glm-flash) on the addon: one major (render-target redraw cost) and eight lows.
* impl (Sonnet) addon review fixes: nine items applied, 68 mock checks and the in-game rerun pass; no escalation.
* review_final (glm-5.3, dgx, kimi hedge) on the merge to master: two lows agreed by two lanes (VTX offsets unchecked, plm2 floats unchecked) and lightingorigin; all fixed in the rebake rounds.
* scout (Haiku) rebake: old branch 16bbe554, loaders, libraries, formats; no escalation.
* impl-opus (Opus) rebake (#57): built, fixture + GMod pixel match; started at Opus as a hard class (binary model rewrite). Four fix rounds on the same agent, no escalation needed.
* reviewer (Haiku) rebake: round 1 major (corner-only island union, the brief's own rule), round 2 major (conversion before bounds checks), round 3 the stock-path gap; all real. Final sha PASS.
* impl-opus (Opus) rebake post-merge follow-up (review_final glm-5.3, dgx, kimi): shadow-LOD replacement refusal, clamp addressing, nested-block refusal, name-length bound, map-name warning, pak writes after the last failure point; fixture pak byte-identical; no escalation.
* review_fast rebake (glm-flash, shop26, deepseek-flash stand-in): round 1 two highs (unchecked MDL/VTX writes) and the -forcedynamicpropsasstatic medium, real; round 3 glm caught the shadow-LOD texture regression, real. Last three lows (orphan VMT name clash, spanning shadow LOD, proxy doc) fixed by the orchestrator after round three.

## Lua addon

`game/addons/proplm` (issue #59). Client realm only. Draws every `prop_dynamic` with `proplm "1"` from cached IMeshes:
albedo on UV0, the VRAD lightmap on UV2. Everything comes from the map file and the models; nothing is keyed to a map,
model or entity name. `game/addons/proplm/prototype/proplm_proto.lua` is the old throwaway probe and stays as is.

### Module map (lua/proplm/, all hang off the global `PROPLM`; entry is lua/autorun/client/proplm.lua)

| file | job |
|---|---|
| `util.lua` | cvars, `P.Msg` / `P.Dbg` (prefix `[proplm] `), name normalisation, "x y z" parser |
| `bsp.lua` | seek-only BSP reader: header, lump entry, game lump directory, entity text. Corruption raises `error(msg, 0)` |
| `lump.lua` | `plm2` parser. `Index` walks only the model headers; `ReadTriangles` reads one model on demand with a single `File:Read` and decodes floats in Lua |
| `mapents.lua` | entity-lump tokenizer (a quoted string is taken whole, only braces outside quotes open or close a block, strings alternate key and value), keeps `prop_dynamic` + `proplm "1"` and the keys the addon uses; records the entity's lump index |
| `lightmap.lua` | converts each baked VTF into a 16-bit float render target (see Calibration); each target has its own refresh window |
| `material.lua` | per instance and material group: one `VertexLitGeneric` with `$basetexture` (the model's albedo), `$detail` (the converted lightmap), `$separatedetailuvs` |
| `mesh.lua` | joins `util.GetModelMeshes(model, 0, 0)` with the lump's UV2 and builds the shared IMeshes |
| `core.lua` | load, entity matching, `apply`, the `RenderOverride`, hooks |
| `console.lua` | `proplm_status`, `proplm_reload` |
| `lua/autorun/proplm_sv.lua` | `AddCSLuaFile` of the modules, so a server install can ship them. It has no effect on a client by itself, and a player who installed the addon does not need it (only a listen server was tried; a remote server was not) |

### Data flow

`InitPostEntity` -> `P.Load`: open `maps/<map>.bsp` (`file.Open(..., "rb", "GAME")`, seeks only), find the `2mlp` game lump (flags
must be 0, version 1), index it, scan the entity lump -> records. `P.Scan` walks `ents.FindByClass("prop_dynamic")`; entities
that are not ready (no model yet, position not networked) wait in `S.pending` and are retried every frame for 6 s;
`OnEntityCreated` queues new ones (entities stream in and out with the PVS), `PostCleanupMap` rescans.
`apply(rec, ent)` order: skin / scale / colour checks -> model state (built once per model) -> lightmap + materials (once per
record) -> IMeshes (once per model) -> `ent.RenderOverride = rec.draw`. Nothing before the last step changes the engine's
drawing, so any failure leaves the stock prop. A model's verdict, good or bad (no lump entry, unmatched triangles, an
unexpected error while joining, a failed IMesh build), is stored on its model state for the whole load: the first instance
prints one line, every other instance of that model reads the verdict without redoing the work or printing again, and
`proplm_reload` clears it. A lightmap verdict (missing VTF, odd size, no render target, a creation step that raised) is cached
per lightmap path for the Lua session, because render targets cannot be freed. `P.mesh.Prepare` never raises; a surprise in
the engine's geometry becomes an unusable model.

### Matching an entity to a record

A record matches a client entity of the same model whose `GetPos` is within 0.5 units of the record origin and whose
`GetAngles` is within 1.5 degrees (the bake is only valid at the baked pose). **Measured (GMod x64, 2026-11-08):**

* `hammerid` survives VBSP's rewrite: the converted `prop_dynamic` keeps it in the entity lump (`"hammerid" "1643"`).
* `Entity:MapCreationID()` works on the client for `prop_dynamic`, but it is **not** the hammerid. It is a running number over the
  entity lump: `worldspawn` (lump entry 0) reads 1235 on every map tried (e2e2 and four test maps), entry k reads 1235 + k.
  Seen: boat 1237 (entry 2); seven props 1237..1243. `Entity(0)` / `game.GetWorld()` report `IsValid() == false` on the
  client but `MapCreationID()` still returns the base, so the addon reads the base from the world entity without `IsValid`.
* The addon therefore uses origin + angles as the key. When the world's base and the entity's `MapCreationID` are both known
  (the entity's is >= 0; it is -1 for an entity the map did not create) the number decides: the entity takes the record with
  `MapCreationID == base + record.index` and no other, so a plain `prop_dynamic` standing on a converted prop's pose is left
  alone, and several records of one model at one origin are told apart. When either number is unknown (the call raises, the base
  is unreadable, the entity is not map-created) the first free record at the pose is taken. The world's call is wrapped in
  `pcall`. The status line says which rule matched ("origin" or "origin and MapCreationID").

### Mesh join

`util.GetModelMeshes` returns groups `{material, triangles}`, three vertices per triangle, `pos` in model space, `u`/`v` equal to
the lump's UV0 (**no v flip, no transform**), unit normals. A corner key is `floor(x*16+0.5), y, z` plus
`floor(u*4096+0.5), floor(v*4096+0.5)`; a triangle key is the sorted triple of corner keys, so corner order and triangle order are
free. Duplicate keys are chained and consumed one per engine triangle. An engine triangle with no lump match that has no area
(`|cross|^2 < 1e-6`) is "degenerate" and not drawn (VBSP drops those from the lump too); any other unmatched triangle
fails the whole model: the rate (matched / non-degenerate) is printed once per model per load and no instance of that model is
overridden. A failed IMesh build gets the same treatment (one line, the model marked unusable, its geometry released).
Measured: boat001a 1044/1044, trashdumpster02 828/828 (+8 degenerate), furniturestove001a 694/694, gravestone001a 182/182,
002a 262/262, 003a 188/188; 0 lump triangles unused in every case. IMeshes are chunked at 8000 triangles.

### Rendering

`RenderOverride(self, flags)`: depth passes (`STUDIO_SHADOWDEPTHTEXTURE`, `STUDIO_SSAODEPTHTEXTURE`) are handed back to the engine with
`self:DrawModel(flags)` (measured: hundreds per second with the flashlight on, no errors); otherwise push a model matrix from
`GetPos`/`GetAngles`, `render.SuppressEngineLighting(true)`, draw the cached meshes inside a `pcall`, restore. A draw error drops the
override of that prop, prints one line for it, and draws the stock model that frame. The failure is latched on the record
(`rec.drawFailed`, checked first in `apply`), so a rescan, `proplm_enable` going back to 1, or the entity re-entering the PVS
leaves the prop stock and silent; `proplm_reload` builds new records and clears it.

**Shader choice (measured): `UnlitGeneric` is wrong for this.** An `UnlitGeneric` mesh ignores texcoord 1 for `$detail`
(it samples at texcoord 0; the prototype used it and showed a flat white boat). `VertexLitGeneric` with
`$separatedetailuvs 1` (both spellings set) honours it, and `SuppressEngineLighting(true)` makes it unlit. Source-material flags
`$alphatest` (+reference), `$translucent`, `$additive`, `$nocull`, `$color`/`$color2` (multiplied), `$alpha` and a non-identity
`$basetexturetransform` are copied; normal map, phong, envmap, selfillum and flashlight response are lost. Materials get unique
names (`proplm_<serial>_<id>_<group>`) and cannot be freed (a few KB each).

### Calibration: the stored value is not "multiply by two"

The stock path (`ConvertLinearToRGBA8888` -> `LinearToVertexLight`, overbright 2): `.ppl` byte `p = 255 * 0.5 * L^(1/2.2)`, shown as
albedo x `2p/255` in gamma space. The VTF holds `s = p/2`, so stock shows albedo x `4s/255`; as a linear multiplier
`X = (4s/255)^2.2`.

Measured pipeline facts (GMod x64, `mat_hdr_level 0`; a `$detail` ramp VTF whose texel value is its column):

* `$basetexture` is sRGB-decoded on read; `$detail` is read **linear**; output is sRGB-encoded. A white base with
  `$detailblendmode 0` therefore shows `(2s/255)^(1/2.2)`: 0.61 of stock where the light is strong, 1.38 where weak, 0.795 pooled.
* `$linearread_basetexture` is not honoured by `VertexLitGeneric`; `$linearwrite 1` is. So no gamma-space material exists.

So `lightmap.lua` draws each VTF once into an RGBA16161616F render target through a `UnlitGeneric` base-texture material with
`$linearwrite 1` and `$color [9.8 9.8 9.8]` (the sRGB decode supplies the power), and the instance materials use that target
as `$detail` with `$detailblendfactor 1.052`. A blend factor above 1 extrapolates the multiply (`X = 1 - f + 2 f D`), a small
negative offset that cancels the lift the hardware decode gives dark values. Constants: `GAIN = 9.8`, `BLEND = 1.052`
(`lightmap.lua`); `LM.SetGain` redraws for refitting.

Validation: four `gravestone` props (0% UV overlap, so the stock `.ppl` is clean) placed radially around the light so none
shadows another, same camera, stock `prop_static` build against the proplm build, `mat_fullbright 1` on the proplm build as the
albedo reference, compared per pixel in raw RGB of `render.Capture`. Mean luminance over the stone bodies, proplm / stock:
1.029 pooled (per stone 0.990, 1.031, 1.089, 1.165 from brightest to dimmest), mean absolute error 2.8 display units. The raw VTF
as `$detail` gives 0.795 pooled (0.61, 0.83, 1.09, 1.38) and 18.3. By stock display value: 16-32 -> 23.4 vs 23.7, 32-64 -> 44.6 vs 42.4,
64-128 -> 99.6 vs 99.0, 128-250 -> 143 vs 145; below 16 the addon is darker (0.2 vs 1.7, 3.6 vs 11.7), invisible in practice.
HDR levels 1 and 2 were not measured. The lightmap multiply happens in linear light in both, so the same X should hold, but
that is unverified.

**A render target drawn while the map is loading comes up empty** (measured), so `lightmap.lua` redraws each target on every
`PreRender` for 3 s, counted from the first frame rendered after that target's creation (not from the creation itself, so a long
load between the two cannot use the window up). The window belongs to the target: creating a lightmap draws only the new
one, looking one up again draws nothing, so the cost over a map is linear in the number of lightmaps. `OnScreenSizeChanged`
redraws and re-arms all targets, since a video mode change can empty all of them at once; `proplm_reload` (`LM.Refresh`) does the
same, but only once the reread map is known to have proplm props.
Without this the props rendered black. A draw that raises still pops the render target and is reported once per target.

**`render.Capture` PNGs carry a varying alpha channel.** Viewers that composite it show brighter pictures than the real RGB;
measure with the alpha dropped (PIL `.convert("RGB")`).

A VRAD-side alternative that removes the render target: write the VTF as RGBA16161616F holding `0.5 * (4s/255)^2.2` and use it as
`$detail` with blend factor 1. Not done: VRAD writes RGB888 today (the contract in `proplm.h`).

### Console

| | |
|---|---|
| `proplm_enable` (1, archived) | 0 removes every override (props draw as stock) and keeps the data; 1 reapplies |
| `proplm_debug` (0) | 1: per-model match rate and timing split, per-prop result, draw cost. 2: also white albedo (lightmap alone); needs `proplm_reload` |
| `proplm_status` | lump size, every model (props, match rate, ms), every prop (hammerid, origin, status, matched-by, reason), totals, draw cost |
| `proplm_reload` | unload (overrides removed, IMeshes destroyed), reread the BSP, rematch |

Quiet by default: maps without a `plm2` lump print nothing (a file that is not a readable BSP prints nothing either); maps with
proplm entities print one summary line three seconds after load, plus one line for each problem (a model that cannot be used,
a corrupt lump or game lump directory, an entity lump that ends inside a quoted string, the first missing lightmap, a lightmap
of odd size, a render target that cannot be created or whose creation raised, a prop whose draw failed, use of the RGBA8888
fallback). Corrupt lump (truncated, wrong version, flags set, absurd model count) or game lump
directory (absurd entry count, lump outside the file): one line, no draw changes. Bad triangle data (material index out of
range, NaN or infinity in a position or UV): one line per model, that model stays stock. Missing lightmap VTFs (VRAD `-fast` or
no bake): one line, props stay stock. The lump cases and the missing VTF were tested in GMod; the lightmap size, render target
and fallback lines, and the other cases here, were exercised offline against a mocked API (see below).

### Limits

* Skin ~= 0, `GetModelScale` ~= 1, a render colour or alpha on the entity, or another addon's `RenderOverride`: left as stock
  (status says why). The bake is at the compile-time pose and the match is made once, when the addon first sees the entity: a
  prop that has already left that pose by then stays stock, but a matched prop that map logic moves later keeps drawing with the
  lightmap baked for its old position (the lighting does not follow it).
* Shadows: converted props cast shadows in VRAD like the static props they replaced (#61); the addon only draws the result.
* Shader features of the source material beyond the list above are lost; translucent materials draw in the opaque pass.
* Lightmap resolution is whatever VRAD wrote (up to 2048 squared, RGB888); a 2048 map is 12 MB of VTF plus a 32 MB float target.
* Only LOD0 and the default bodygroup, as in the lump.
* Render targets cannot be freed in GMod; they are created once per lightmap path per Lua session.

### Numbers (GMod x64, 1280x720 windowed, this dev box)

* Model build, first instance of a model: 6 to 9 ms for 180 to 1044 triangles (was 22 to 124 ms before the bulk-read decode:
  `File:ReadFloat` costs about 3 us per call). Split for the boat: read 2.8, key lookup 1.2, `GetModelMeshes` 2.5, join 2.1, IMesh
  build 0.5 ms. Roughly 8 us per triangle, once per model.
* Per-instance draw: 3.5 to 5.5 us CPU submit (`proplm_debug 1`). Frame rate pinned at the 300 fps cap with and without the addon
  (3.336 vs 3.339 ms), 7 props; no frame cost visible here.
* Lump header read plus entity scan: 0.8 to 1.5 ms.

### How to test headlessly

Without the game: the python `lupa` package ships LuaJIT 2.1, the runtime GMod x64 uses. `loadstring` on every file is a real syntax
check, and a short mock of the GMod API (`hook`, `render`, `Vector`, `file.Open` over a BSP held in a string) is enough to drive
`P.Load`, matching, the lightmap scheduler and the loud/silent matrix; nothing in the tree depends on it.

Copy the addon to `GarrysMod/garrysmod/addons/<name>`, drop the BSP in `garrysmod/maps`, launch `bin/win64/gmod.exe` from the GarrysMod
folder with `-console -condebug -novid -windowed -w 1280 -h 720 -noworkshop +gamemode sandbox +map <name>`, read
`garrysmod/console.log`. A throwaway client script in the same addon folder can wait, set a `CalcView` camera, `render.Capture` in
`PostRender`, `file.Write` the PNG (lands in `garrysmod/data`), and print a marker; kill the process from outside (Lua cannot quit).
Calibration pair builds: `vbsp_lmo.exe` and `vrad_rtx.exe` with `-game <hl2mp>` and `-StaticPropLighting -bounce 1`. Models that
stock VBSP keeps as `prop_static` and VRAD reports 0% UV overlap for: `props_c17/gravestone001a`, `002a`, `003a`.
