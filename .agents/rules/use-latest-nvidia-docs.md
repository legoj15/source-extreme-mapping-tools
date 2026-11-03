---
trigger: always_on
---

The `nvidia-cuda-docs` MCP server (project-scoped, `.mcp.json`) serves current first-party NVIDIA CUDA docs and code examples to agents. Prefer it for CUDA API questions over training-data knowledge. It does NOT cover OptiX — use the ray-tracing links below for that.

CUDA 13.2 has implimented important changes, such as the removal of libnvvp. Please refer to the online documentation for CUDA and Nsight online: https://docs.nvidia.com/cuda/

Documentation for profiling is separate, and can be found in two different spots, depending on whether or not Python is being used to initiate CUDA:
https://docs.nvidia.com/cupti/
https://docs.nvidia.com/cupti-python/

GameWorks documentation is old, but goes over topics such as DirectX and OptiX:
https://docs.nvidia.com/gameworks-products/index.html
https://archive.docs.nvidia.com/gameworks/index.html
https://archive.docs.nvidia.com/gameworks/index.html#gameworkslibrary/gameworks_library.htm


GPUDirect Storage could be useful, its selling point is as follows:
*GPUDirect Storage is designed for workloads that need to move large amounts of data efficiently between storage and GPUs. By avoiding unnecessary CPU copies, it helps improve throughput, reduce latency, and free CPU resources for other work.*
It's documentation can be found here: https://docs.nvidia.com/gpudirect-storage/index.html

Importantly, there are more dedicated ray-tracing pages here:
https://docs.nvidia.com/ray-tracing/index.html
https://raytracing-docs.nvidia.com/
https://raytracing-docs.nvidia.com/optix9/index.html

The information at "https://developer.nvidia.com/rtx/ray-tracing/optix" appears to be out of date, only referencing OptiX 7 and 8, while the latest release is 9.1.0.