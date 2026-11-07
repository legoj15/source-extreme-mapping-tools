# VVIS-OPTIX Test Manifest
# Each entry defines a test case for the three-way validation harness.
#
# Three-Way Test Strategy:
#   ref-cpu  = vvis.exe        (SDK reference binary) — ground truth
#   cpu      = vvis_optix.exe   -threads ControlThreads — compared to ref-cpu
#   gpu      = vvis_optix.exe   (all threads) — must match cpu bit for bit
#
# Keys:
#   MapName             - BSP filename (without extension) in bsp_unit_tests\
#   ModDir              - Relative path to the game mod directory
#   VbspSource          - "local" (.\vbsp.exe) or "sdk" (SDK Base 2013 vbsp.exe)
#   ExtraVbspArgs       - Additional vbsp arguments (array of strings)
#   TimeoutMultiplier   - GPU timeout = max(MinTimeoutSeconds, CpuTime * Multiplier)
#   MinTimeoutSeconds   - Floor for the timeout calculation (seconds)
#   RefCpuTolerance     - Max visual diff % for ref-cpu vs cpu
#   CpuGpuTolerance     - Max diff % for cpu vs gpu (0: the PVS does not depend on the thread count)
#   ControlThreads      - -threads for the cpu leg (1 = the one-thread stock order; more on maps
#                         too slow for one thread)
#   ArchiveSuffix       - Subfolder suffix in bsp_unit_test_logs\
#   Groups              - Array of group tags for -Group selection

@{
    basic             = @{
        MapName           = "validation"
        ModDir            = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
        VbspSource        = "sdk"
        ExtraVbspArgs     = @()
        TimeoutMultiplier = 1.5
        MinTimeoutSeconds = 30
        RefCpuTolerance   = 0.5
        CpuGpuTolerance   = 0.0
        ControlThreads    = 1
        ArchiveSuffix     = "basic"
        Groups            = @("core")
    }
    harder            = @{
        MapName           = "validation_harder"
        ModDir            = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
        VbspSource        = "sdk"
        ExtraVbspArgs     = @()
        TimeoutMultiplier = 1.5
        MinTimeoutSeconds = 30
        RefCpuTolerance   = 0.5
        CpuGpuTolerance   = 0.0
        ControlThreads    = 1
        ArchiveSuffix     = "harder"
        Groups            = @("core")
    }
    visibility        = @{
        MapName           = "validation_vvis_visibility"
        ModDir            = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
        VbspSource        = "sdk"
        ExtraVbspArgs     = @()
        TimeoutMultiplier = 5.0
        MinTimeoutSeconds = 60
        RefCpuTolerance   = 0.5
        CpuGpuTolerance   = 0.0
        ControlThreads    = 1
        ArchiveSuffix     = "visibility"
        Groups            = @("core")
    }
    production_harder = @{
        MapName           = "fightspace3_unittest"
        ModDir            = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
        VbspSource        = "sdk"
        ExtraVbspArgs     = @("-notjunc")
        TimeoutMultiplier = 5.0
        MinTimeoutSeconds = 60
        RefCpuTolerance   = 0.5
        CpuGpuTolerance   = 0.0
        ControlThreads    = 4
        ArchiveSuffix     = "production_harder"
        Groups            = @("production")
    }
    production        = @{
        MapName           = "d1_trainstation_02_hrcs"
        ModDir            = "E:\Steam\steamapps\common\Source SDK Base 2013 Multiplayer\sourcetest"
        VbspSource        = "sdk"
        ExtraVbspArgs     = @("-notjunc")
        TimeoutMultiplier = 5.0
        MinTimeoutSeconds = 60
        RefCpuTolerance   = 0.5
        CpuGpuTolerance   = 0.0
        ControlThreads    = 4
        ArchiveSuffix     = "production"
        Groups            = @("production")
    }
}
