# Cloud fusion experiment

This benchmark compares the actual frozen 3bf8744 Cartesian + direct PointCloud2
writer with this checkout's scalar fusion, 32/64/128-point scalar tiles and
32/64/128-point explicit AArch64 NEON tiles. All variants run in one process with
identical compiler options. Frozen headers compile under a separate namespace to
avoid linking alternate definitions of the same template symbol. The candidate
ROS factory currently selects scalar fusion; tile/NEON variants are experiments.

Use the wrapper's retained `ouster_driver:cpu-crop-organized-v1-tests` ARM64 image.
Source `/opt/ros/lyrical/setup.bash` and `/workspace/install/setup.bash`, then:

```sh
cmake -S /candidate/ouster-ros/test/cloud_fusion_benchmark -B /tmp/cloud-bench \
  -DCMAKE_BUILD_TYPE=Release \
  -DBASELINE_ROS_SOURCE=/baseline/ouster-ros \
  -DCANDIDATE_ROS_SOURCE=/candidate/ouster-ros \
  -DFIXTURE_HELPERS=/wrapper/tools/gpu_cloud_test \
  -DSDK_SOURCE=/workspace/src/ouster-ros/ouster-ros/ouster-sdk \
  -DSDK_BUILD=/workspace/build/ouster_ros/ouster-sdk
cmake --build /tmp/cloud-bench --parallel 3
/tmp/cloud-bench/cloud_fusion_benchmark /fixtures 64 9 0 2048
/tmp/cloud-bench/cloud_fusion_benchmark /fixtures 64 9 962 1149
/tmp/cloud-bench/cloud_fusion_benchmark /fixtures 64 9 1474 637
```

Mount the unchanged crop checkout at `/baseline`, this checkout at `/candidate`,
the wrapper at `/wrapper` and its captured scan fixtures at `/fixtures`, read-only.
Captured fixtures are private local artifacts, not source-controlled. Correctness
runs before timing: all eight variants must match every captured reference's
selected 48-byte points exactly and match baseline message metadata/density. The
unit suite separately covers profiles, returns, masks, row reduction, missing
columns, timestamps, retained buffers and arithmetic edge cases.

The baseline uses optimized cropped Cartesian computation when a narrow ROI is
requested, not full-frame computation followed by copying. Timings include
Cartesian computation, field conversion and final buffer writes; reuse existing
output capacity, and exclude fixture/scan loading, LUT creation, packet decoding,
ROS serialization, transport and driver scheduling. The ROI fixtures come from a
full-azimuth capture, not live narrow-azimuth sensor input. Benchmark order rotates
and reverses across rounds. Timings on Mac-hosted ARM64 Linux cannot establish
Orin CPU savings; a coordinated Orin measurement is required. Explicit NEON FMA
is tested against the current ARM64 production compiler's FMA behavior, not a
promise of identical results with arbitrary floating-point compiler flags.

## Recorded experiment, 2026-11-13

GCC 15.2.0, `-O3`, exact 8-frame validation passed for all eight variants. The
Orin runs used the same binary as the Mac runs, MAXN/schedutil, CPU affinity 2,
with the sensor in standby and the existing idle driver/router unchanged. All
transfers finished before measurement. Six invocations used full/1149/637/637/
1149/full output widths, each with nine interleaved rounds of 128 frames. The
table pools the 18 round means for each ROI and reports median process CPU
milliseconds per frame, not whole-driver CPU usage or individual-frame latency.

| Variant | Full 2048 | ROI 1149 | ROI 637 |
|---|---:|---:|---:|
| Actual production 3bf8744 | 3.1196 | 1.7746 | 0.8587 |
| Scalar fusion | 2.4619 | 1.3007 | 0.5893 |
| Scalar tile 32 | 2.8109 | 1.6433 | 0.8163 |
| Scalar tile 64 | 2.9233 | 1.7459 | 0.8468 |
| Scalar tile 128 | 3.0694 | 1.8073 | 0.8886 |
| NEON tile 32 | 2.7442 | 1.5236 | 0.7404 |
| NEON tile 64 | 2.8557 | 1.6097 | 0.7787 |
| NEON tile 128 | 3.0895 | 1.7114 | 0.8618 |

Scalar fusion reduced this measured stage by 21.1%, 26.7% and 31.4%, respectively.
Wall time closely tracked process CPU time. Larger tiles introduced enough cache
management and intermediate writes to lose the benefits of fusion; some narrow
ROI cases regressed against production. NEON tiling improved on production but
never beat scalar fusion. These observations support the scalar candidate rather
than a SIMD guarantee. Assembly confirms scalar FMADD remains in both production
and fused paths and explicit vector FMLA/LD3/ST3 appears in the NEON experiment.
Mac-hosted results independently favored scalar fusion by about 25–26%; absolute
timings differ and should not be compared across hosts.

Raw logs, CSV, host state, compiler, binary SHA and disassembly are retained in
the wrapper's ignored `var/cpu-next-cloud-evidence-20261113` directory. The first
sanitizer run exposed a test-oracle issue: `-O1` instrumentation inhibited scalar
FMA contraction, while explicit NEON still performed one rounding. NEON edge
checks therefore use `std::fma` as their declared arithmetic oracle; production
scalar fusion continues to compare directly with `cartesianT` in both Release
and sanitizer builds. The SIMD scalar tail also uses explicit FMA so its arithmetic policy matches
its vector lanes under instrumentation. The default production scalar fusion
continues to use the original expressions; no accuracy tolerance was added.
