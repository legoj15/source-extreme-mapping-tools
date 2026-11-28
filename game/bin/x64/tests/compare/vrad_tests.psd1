# VRAD-RTX Test Manifest
# Each entry defines a test case for the three-way validation harness.
#
# Keys:
#   MapName             - BSP filename (without extension) in tests\maps\ (or tests\compare\maps\)
#   ExtraArgs           - Additional vrad arguments (array of strings). They go to all three legs, the SDK's stock vrad.exe included,
#                         which stops on an option it does not know: only stock options here.
#   TimeoutMultiplier   - GPU timeout = ControlTime * Multiplier (+ extension)
#   RefCpuTolerance    - Max visual diff % for ref-cpu vs cpu
#   CpuGpuTolerance   - Max visual diff % for cpu vs gpu (GPU)
#   RefCpuLightmapThreshold - image_compare.py lightmaps --threshold for the SDK reference against our CPU build
#   CpuGpuLightmapThreshold - the same for our CPU build against -cuda
#                         Both are the percent of decoded light that may differ, set 2026-11-27 from
#                         vrad_noise_floor.ps1. Since #174 our CPU and GPU builds give the same lightmap
#                         on every run (their noise is 0.0000 on every map), so CPU vs GPU is one
#                         number per map on a given machine: the limit is 0.005 to 0.01 where the two
#                         agree, and where a known difference stands, its figure plus a margin and the
#                         issue that tracks it, so the test still fails when it gets worse. The stock
#                         reference still differs from run to run (it has the uninitialised reads #174
#                         removed here), so the reference limits are about three times the largest figure
#                         seen. A change to the CPU ray tracer can move its result by 0.05 to 0.13 with no
#                         change to the lighting (#173): if a limit trips after one, measure with
#                         vrad_noise_floor.ps1 before touching a number.
#   ArchiveSuffix       - Subfolder suffix in tests\out\logs\
#   Groups              - Array of group tags for -Group selection (optional)

@{
    full                   = @{
        MapName           = "validation"
        ExtraArgs         = @()
        TimeoutMultiplier = 1.5
        RefCpuTolerance  = 1.0
        CpuGpuTolerance = 15.0
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "full"
        Groups            = @("core")
    }
    quick                  = @{
        MapName           = "validation_vrad_quick"
        ExtraArgs         = @()
        TimeoutMultiplier = 2.5
        RefCpuTolerance  = 1.0
        CpuGpuTolerance = 8.0
        RefCpuLightmapThreshold = 0.05
        CpuGpuLightmapThreshold = 0.01
        ArchiveSuffix     = "quick"
        Groups            = @("core")
    }
    props                  = @{
        MapName           = "validation_props"
        ExtraArgs         = @("-largeDispSampleRadius", "-textureshadows", "-StaticPropPolys", "-StaticPropLighting", "-final", "-lights", "E:\lights_custom.rad")
        TimeoutMultiplier = 2.0
        RefCpuTolerance  = 0.5
        CpuGpuTolerance = 0.5
        # CPU vs GPU stands at 0.074, all of it the CPU's: under this map's 45-degree sun its tracer lets
        # sun rays that land on a seam between two sky triangles through, and darkens the luxels at
        # face edges (#173; the stock compiler does the same). With the sun at pitch -44 the two
        # paths agree to 0.0000.
        RefCpuLightmapThreshold = 0.1
        CpuGpuLightmapThreshold = 0.1
        ArchiveSuffix     = "props"
        Groups            = @("props")
    }
    radiosity              = @{
        MapName           = "validation_radiosity"
        ExtraArgs         = @()
        TimeoutMultiplier = 1.5
        RefCpuTolerance  = 0.5
        CpuGpuTolerance = 0.5
        # CPU vs GPU stands at 0.040: the same CPU leak as on props (#173), 0.0000 at pitch -44
        RefCpuLightmapThreshold = 0.05
        CpuGpuLightmapThreshold = 0.06
        ArchiveSuffix     = "radiosity"
        Groups            = @("core")
    }
    supersampling_indoors  = @{
        MapName           = "validation_vrad_supersampling_indoors"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 1.0
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.02
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "supersampling_indoors"
        Groups            = @("supersampling")
    }
    supersampling_outdoors = @{
        MapName           = "validation_vrad_supersampling_outdoors"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 5
        RefCpuLightmapThreshold = 0.01
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "supersampling_outdoors"
        Groups            = @("supersampling")
    }
    props_textureshadows = @{
        MapName           = "validation_vrad_proptextureshadows"
        ExtraArgs         = @("-textureshadows", "-StaticPropPolys", "-StaticPropLighting", "-lights", "E:\lights_custom.rad")
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 5
        # CPU vs GPU stands at 0.100: the CPU counts a transparent triangle's alpha more than once (#172)
        RefCpuLightmapThreshold = 0.03
        CpuGpuLightmapThreshold = 0.12
        ArchiveSuffix     = "props_textureshadows"
        Groups            = @("props")
    }
    named_lights = @{
        MapName           = "validation_vrad_namedlights"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 5
        RefCpuLightmapThreshold = 0.02
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "named_lights"
        Groups            = @("props")
    }
    gridlines        = @{
        MapName           = "validation_vrad_gridlines"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "gridlines"
        Groups            = @("core")
    }
    many_styles      = @{
        # Six narrow spotlights of six light styles on two faces of a pillar, three on each, and no
        # bounce: every cluster's light list holds six styles while no face is lit by more than
        # three. A sample has four style slots, so the GPU must give them to the lights that reach
        # the sample (#175: handing them to the list's first four lost two whole lights on one
        # face, which the comparison reports as a structural difference).
        MapName           = "validation_vrad_manystyles"
        ExtraArgs         = @("-bounce", "0")
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "many_styles"
        Groups            = @("core")
    }
    bounce_shadow    = @{
        # A room built on the grid (768 x 512 x 256, every face bump-lit: 18 of 18) with a
        # floor-to-ceiling pillar and one point light, bounced. Many of its patch edges lie exactly
        # on luxel edges, and there which patch a touching sample lights is a tie that falls on the
        # last float bit (#178): with polylib.cpp compiled /fp:fast with fused multiply-adds, our
        # build left dark a column of patches at the edge of the pillar's shadow that the stock
        # compiler lights, 1 % of the first bounce's light and 0.134 % of the lightmaps. With strict
        # arithmetic there every patch has the stock compiler's light (0.0041 %), and with the
        # form factor no longer dropped at a right angle (#181) the lightmaps are at 0.0015 %.
        # #175's two bump-face items were measured here as well: our CPU and -cuda agree exactly.
        MapName           = "validation_vrad_bounceshadow"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "bounce_shadow"
        Groups            = @("core")
    }
    bump_sky         = @{
        # The same kind of room under a sky: no brush ceiling, a light_environment with a 5-degree
        # sun (30 sun rays a sample, 29 of them jittered) and a sky ambient, a pillar and a low slab, every face
        # bump-lit. The GPU kernel counted a sky direction for a normal when its dot was over 1e-6,
        # the CPU when it is over 0.001: a direction between the two entered one bump normal's
        # count on the GPU only, and that bump lightmap came out 1.8 % darker on every wall of one
        # orientation (#175; 0.0167 % of the lightmaps, 0.159 % with the sky ambient alone and no
        # bounce). With the CPU's threshold in the kernel the two are at 0.0002 %. The stock
        # compiler against our CPU was 0.0261 % until #181 (form factors dropped at right angles)
        # and is 0.0002 % as well since (vrad_noise_floor.ps1, two runs of each).
        MapName           = "validation_vrad_bumpsky"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "bump_sky"
        Groups            = @("core")
    }
    skybox3d         = @{
        # A room open to the sky with a 3D skybox (a sky_camera at scale 16) that holds one block:
        # in world scale it stands south-west of the room, 1,200 units tall, in the path of the
        # sun's rays to about half of the floor. The CPU follows a sun or sky ray that reaches the
        # sky into the 3D skybox (TestLine_DoesHitSky), so the block shadows the floor; the GPU
        # kernel did not, and lit all of it: 79 % of the decoded light between the two (#175 item
        # 5). With the same second trace in the kernel they are identical. -noskyboxrecurse
        # switches it off on both. The stock compiler against our CPU is 0.0012 % here (one
        # run of each); its limit is the manifest's smallest, 0.005.
        MapName           = "validation_vrad_skybox3d"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "skybox3d"
        Groups            = @("core")
    }
    disp_bump        = @{
        # A bump-mapped displacement hill (power 4, 96 units high) under a sun and a sky ambient,
        # with a pillar's shadow across it. On a displacement or a smoothed face the CPU builds the
        # three bump directions per sample, from that sample's own normal; the GPU used one set per
        # face: the hill's bump lightmaps were 42, 5 and 4 % off, 5.3 % of the decoded light
        # (#175). The kernel now builds them per sample, and applies the CPU's rule that a sky
        # direction lights the bump directions of every sample of a group of four when one of the
        # four faces it (0.043 % was left without that rule). The two are at 0.0005 %; the stock
        # compiler against our CPU is 0.0002 % (one run of each).
        MapName           = "validation_vrad_dispbump"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "disp_bump"
        Groups            = @("core")
    }
    barrel_bump      = @{
        # A bump-mapped barrel vault under a sun and a sky ambient: half a cylinder built from eight
        # wedge brushes, its facets 22.5 degrees apart. VRAD smooths two faces that share an edge and
        # are under 45 degrees apart, so each facet is a "not flat" bump face like a displacement,
        # and unlike a displacement it is supersampled. The CPU gathers the four sky-ambient
        # sub-positions of a supersampled luxel as one group of four (a sky direction one of them
        # faces lights the bump directions of the others, a sub-position outside the face included);
        # the GPU lit each alone and the facets' bump lightmaps came out 0.03 to 0.47 % darker,
        # 0.0169 % of the decoded light here and 0.0319 % under the sky ambient alone (#175). With
        # the groups uploaded the two are at 0.0001 %. Any curved wall with a bump-mapped
        # material is this case.
        MapName           = "validation_vrad_barrel"
        ExtraArgs         = @()
        TimeoutMultiplier = 3.0
        RefCpuTolerance  = 3
        CpuGpuTolerance = 0.2
        RefCpuLightmapThreshold = 0.005
        CpuGpuLightmapThreshold = 0.005
        ArchiveSuffix     = "barrel_bump"
        Groups            = @("core")
    }
}
