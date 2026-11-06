# Source Extreme Mapping Tools

A suite of modernization efforts, Python build pipelines, and hardware-accelerated drop-in replacements for the Source Engine's mapping tools (VBSP, VVIS, and VRAD). Designed to bring modern compilation speeds, robust CI/CD automated validation, and new micro-optimization workflows to Source Engine level design.

## The Toolchain

### 1. Lightmap Optimizer (`lmoptimizer_gui.py` & `lmoptimizer.py`)
A Python-based toolkit and graphical interface that automates the micro-optimization of map lighting and performance.
- **Auto-Compile Mode:** An iterative build loop that automatically integrates VBSP, VVIS (`-fast` execution), and VRAD directly into the optimizer to rapidly test and shrink global lightmap scales.
- **Never-Visible Faces:** Walks every place a player can reach and finds the faces no player can ever see, then gives them the coarsest lightmap so they cost nothing. Emissive (texlight) faces are left alone.
- **Vis-Debug Painter (`vis_simulator_gui.py`):** Paints a copy of the map with the visible and never-visible faces in two colours, so you can check the result in Hammer.
- **Python Automation API:** The backbone used for all automated map compilation, unit tests, and validation test harnesses in this repository.

### 2. VBSP LMO (`vbsp_lmo.exe`)
A patched version of the base VBSP engine geometry compiler.
- **Brush Carving Enhancements:** Correctly manages and propagates Map Overlays during brush carving operations.
- **Detail Sprite Support:** Retains detail sprites and correct alpha-blending logic on WorldVertexTransition (WVT) faces after patches.
- **Integration:** Hooks smoothly into the Lightmap Optimizer pipeline.

### 3. VVIS (`vvis_optix.exe`)
A faster visibility compiler. It runs on the CPU; no graphics card is needed.
- **Fast mode (`-cuda`):** Spreads the work evenly over every CPU core, so one huge area no longer holds up the whole compile. A large map that ran for hours without finishing now takes minutes. The flag keeps its old name so existing scripts still work.
- **Same result:** The visibility it writes matches the original tool exactly.

### 4. VRAD RTX (`vrad_rtx.exe`)
Hardware-accelerated global illumination and radiosity bounce lighting.
- **OptiX Raytracing:** Hardware-accelerated VisMatrix generation, radiosity bouncing, and shadow ray batching.
- **Meta-Batching Architecture:** A highly parallelized data strategy that significantly outperforms legacy Source CPU lighting algorithms.
- **Bit-Accurate:** Ensures SDK parity while providing blistering-fast build times.

## Key Improvements Across All Tools

- **Portable Filesystem Handling:** Legacy SDK tools often demanded execution from within a specific `bin` folder. All tools here utilize modernized `|all_source_engine_paths|` resolution logic, allowing you to run them from anywhere.
- **Advanced Game Resolution:** Can cleanly run against remote content through the new `-binroot` flag (ideal for Garry's Mod compatibility, taking advantage of the SDK 2013 Multiplayer base).
- **Modernized Limits & Cleanup:** Raised thread limits from the ancient 16 up to 64 maximum threads, stripped out obsolete VMPI (Valve Message Passing Interface) dependencies to prevent crashes, and fixed various threading pool deadlocks (such as pipe hangs in PowerShell environments).
- **Validation Suite:** An ironclad set of PowerShell and Python scripts (`run_tests.ps1`, `vrad_test_harness.ps1`, `vvis_test_harness.ps1`) to ensure pixel and bit-perfect fidelity against all modifications.

## Build Requirements

If compiling the tools from source:
- Visual Studio 2026 (untested on older versions)
- NVIDIA CUDA Toolkit 13.4 (13.2 or newer works; 13.1 does not)
- NVIDIA OptiX SDK 9.1.0 
- Python 3.10+ (for using `lmoptimizer`, tools, and validation scripts)

To build: run `src\createallprojects.bat`, open `src\mapping_tools.sln` in Visual Studio, and build the Release configuration. The tools land in `game\bin\x64`.

## Usage

Tested primarily heavily on an RTX 3090, but should work on comparable hardware.

1. Replace your existing `vvis.exe`, `vrad.exe`, and `vbsp.exe` with the compiled binaries, the `.dll` files VVIS and VRAD load, and the `.ptx` files VRAD and bspwalk use inside your workflow.
2. The core compilers run as drop-in replacements for standard builds.
3. For Python tools, you can execute `lmoptimizer_gui.py` to start the GUI.

For the fast paths in compile scripts:
```bat
vvis_optix.exe -cuda -game <path_to_mod_folder> <mapname>
vrad_rtx.exe -rtx -game <path_to_mod_folder> <mapname>
```

`-cuda` makes VVIS use every CPU core well; `-rtx` runs VRAD on the graphics card. Without them both tools work the original way.

**Garry's Mod Note:** Because GMod binaries are unique, please specify `-binroot <Path_To_Source_SDK_2013_Multiplayer>` when utilizing these tools against GMod maps so that it successfully locates and mounts the correct required core HL2 library resources.

## License

Derived from the Source SDK 2013 Base. See [LICENSE](LICENSE) for details.
