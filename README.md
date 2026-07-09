# Source Extreme Mapping Tools

A suite of modernization efforts, Python build pipelines, and hardware-accelerated drop-in replacements for the Source Engine's mapping tools (VBSP, VVIS, and VRAD). Designed to bring modern compilation speeds, a local PowerShell/Python validation suite, and new micro-optimization workflows to Source Engine level design.

## The Toolchain

### 1. Lightmap Optimizer (`lmoptimizer_gui.py` & `lmoptimizer.py`)
A Python-based toolkit and graphical interface that automates the micro-optimization of map lighting and performance.
- **Auto-Compile Mode:** An iterative build loop that automatically integrates VBSP, VVIS (`-fast` execution), and VRAD directly into the optimizer to rapidly test and shrink global lightmap scales.
- **Vis-Debug Painter:** Automatically identifies geometry flags and masks never-visible and emissive (texlight) faces from optimization, highlighting them in-engine for easy iteration.
- **Python Automation API:** The backbone used for all automated map compilation, unit tests, and validation test harnesses in this repository.

### 2. VBSP LMO (`vbsp_lmo.exe`)
A patched version of the base VBSP engine geometry compiler.
- **Brush Carving Enhancements:** Correctly manages and propagates Map Overlays during brush carving operations.
- **Detail Sprite Support:** Retains detail sprites and correct alpha-blending logic on WorldVertexTransition (WVT) faces after patches.
- **Integration:** Hooks smoothly into the Lightmap Optimizer pipeline.

### 3. VVIS OptiX (`vvis_optix.exe`)
A hardware-accelerated Visibility Processor.
- **GPU PortalFlow Raytracing:** Shifts the heavy inner-loop raycasting workload to RT Cores on modern NVIDIA GPUs utilizing CUDA and OptiX SDK 9.1.
- **Hybrid Architecture:** Combines fast CPU Setup and bounds checking with GPU massive concurrency trace calls.
- **Modernized CPU Path:** For non-CUDA users, the fallback CPU path features SSE2 SIMD inner-loop optimizations, critical-section work dispatch, and support for up to 64 threads.
- **Parity:** The rewritten CPU path is tested to produce bit-for-bit identical visibility to the original Source SDK reference algorithm—only significantly faster. The GPU path is stochastic (4096 rays per portal pair, targeting under 1% leakage) and is validated against the CPU result within a 1–15% tolerance rather than bit-for-bit.

### 4. VRAD RTX (`vrad_rtx.exe`)
Hardware-accelerated global illumination and radiosity bounce lighting.
- **OptiX Raytracing:** Hardware-accelerated VisMatrix generation, radiosity bouncing, and shadow ray batching.
- **Meta-Batching Architecture:** A highly parallelized data strategy that significantly outperforms legacy Source CPU lighting algorithms.
- **Visually Indistinguishable:** Produces lighting within ~0.03/255 average RGB deviation of the CPU reference (validated at 0.2–15% test tolerances), while providing much faster build times.

### 5. VRAD Nextgen (`vrad_nextgen.exe`) — Experimental
An OptiX-based lighting research prototype. **Not suitable for production maps.** It is LDR-only: it does not compute HDR lighting and purges any existing HDR lightmap lumps from the BSP, and it emits placeholder (dummy) worldlight and leaf-ambient lumps so the engine will load its LDR lightmaps at all. Treat it as an experiment, not a drop-in `vrad.exe` replacement.

## Key Improvements Across All Tools

- **Portable Filesystem Handling:** Legacy SDK tools often demanded execution from within a specific `bin` folder. All tools here utilize modernized `|all_source_engine_paths|` resolution logic, allowing you to run them from anywhere.
- **Advanced Game Resolution:** Can cleanly run against remote content through the new `-binroot` flag (ideal for Garry's Mod compatibility, taking advantage of the SDK 2013 Multiplayer base).
- **Modernized Limits & Cleanup:** Raised thread limits from the ancient 16 up to 64 maximum threads, compiled out the obsolete VMPI (Valve Message Passing Interface) code paths behind an `#ifdef MPI` guard to prevent crashes, and fixed various threading pool deadlocks (such as pipe hangs in PowerShell environments).
- **Validation Suite:** A set of PowerShell and Python test harnesses (`run_vrad_tests.ps1`, `run_vvis_tests.ps1`, and `run_vrad_nextgen_tests.ps1` under `game/bin/x64/unit_test/`) that validate each modification against the SDK reference within defined tolerances.

## Build Requirements

If compiling the tools from source:
- Visual Studio 2026 (untested on older versions)
- NVIDIA CUDA SDK 13.2
- NVIDIA OptiX SDK 9.1.0 
- Python 3.10+ (for using `lmoptimizer`, tools, and validation scripts)

## Usage

Tested primarily heavily on an RTX 3090, but should work on comparable hardware.

1. Replace your existing `vvis.exe`, `vrad.exe`, and `vbsp.exe` with the compiled binaries and extension `.dll` / `.ptx` files inside your workflow.
2. The core compilers run as drop-in replacements for standard builds.
3. For Python tools, you can execute `lmoptimizer_gui.py` to start the GUI.

To explicitly invoke GPU hardware paths inside compile scripts:
```bat
vvis_optix.exe -cuda -game <path_to_mod_folder> <mapname>
vrad_rtx.exe -rtx -game <path_to_mod_folder> <mapname>
```

If the `-rtx` flag is omitted for VRAD, it defaults to the aggressively optimized parallel CPU path. 

**Garry's Mod Note:** Because GMod binaries are unique, please specify `-binroot <Path_To_Source_SDK_2013_Multiplayer>` when utilizing these tools against GMod maps so that it successfully locates and mounts the correct required core HL2 library resources.

## License

Derived from the Source SDK 2013 Base. See [LICENSE](LICENSE) for details.
