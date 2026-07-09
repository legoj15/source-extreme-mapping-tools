# VRAD-Nextgen Test Manifest
# Each entry defines a test case for the Two-Way validation harness
# comparing vrad_rtx.exe (CPU, -bounce 0) against vrad_nextgen.exe.
#
# Keys:
#   MapName             - BSP filename (without extension) in unit_test_maps\
#   CpuExtraArgs        - Additional vrad_rtx.exe arguments (array of strings)
#   NextgenExtraArgs    - Additional vrad_nextgen.exe arguments (array of strings)
#   TimeoutMultiplier   - Nextgen timeout = CpuTime * Multiplier (+ extension)
#   CpuNextgenTolerance - Max visual diff % for cpu vs nextgen
#   LightmapThreshold   - bsp_diff_lightmaps.py --threshold value
#   ArchiveSuffix       - Subfolder suffix in unit_test_logs\
#   Groups              - Array of group tags for -Group selection (optional)

@{
    radiosity_simple = @{
        MapName             = "validation_vrad_radiosity_simple"
        CpuExtraArgs        = @("-bounce", "0")
        NextgenExtraArgs    = @()
        TimeoutMultiplier   = 3.0
        CpuNextgenTolerance = 15.0
        LightmapThreshold   = 0.5
        ArchiveSuffix       = "nextgen_radiosity_simple"
        Groups              = @("nextgen")
    }
    nextgen_supersampling_indoors = @{
        MapName             = "validation_vrad_supersampling_indoors"
        CpuExtraArgs        = @("-bounce", "0")
        NextgenExtraArgs    = @()
        TimeoutMultiplier   = 3.0
        CpuNextgenTolerance = 15.0
        LightmapThreshold   = 0.5
        ArchiveSuffix       = "nextgen_supersampling_indoors"
        Groups              = @("nextgen")
    }

}
