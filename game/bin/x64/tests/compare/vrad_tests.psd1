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
#                         on every run (their noise is 0.0000 on all nine maps), so CPU vs GPU is one
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
        # CPU vs GPU stands at 0.074: how the GPU supersamples luxels a face covers in part (#175)
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
        # CPU vs GPU stands at 0.040: the same border luxels, carried on by the bounce (#175)
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
}
