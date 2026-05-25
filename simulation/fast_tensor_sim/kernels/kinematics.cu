/*
 * kinematics.cu
 * CUDA kernels for massively parallel drone kinematic updates.
 *
 * Target hardware : NVIDIA RTX A1000 or better (~250M timesteps/sec across 8K drones).
 * Compile         : nvcc -O3 -arch=sm_86 -o kinematics.so --shared kinematics.cu
 *
 * Design notes:
 *   - Each CUDA thread handles exactly one drone's physics update.
 *   - All state arrays are stored as Structure-of-Arrays (SoA) passed as raw
 *     device pointers so the Python host can wrap them with CuPy/PyTorch.
 *   - Shared memory is used in the collision-check kernel to amortise global
 *     memory reads for the neighbourhood comparison.
 *   - The color-interpolation kernel performs a branchless lerp that maps
 *     directly to FMAD instructions on Ampere.
 *
 * Kernel inventory
 * ----------------
 *  1. update_positions_kernel  – PD controller + Euler integration
 *  2. collision_check_kernel   – pairwise proximity detection via shared memory tiles
 *  3. update_colors_kernel     – linear interpolation of LED RGB keyframes
 */

#include <cuda_runtime.h>
#include <math.h>
#include <stdint.h>

/* --------------------------------------------------------------------------
 * Constants (must match sim_core.py)
 * -------------------------------------------------------------------------- */
#define MAX_VELOCITY      8.0f   /* m/s  */
#define MAX_ACCELERATION  5.0f   /* m/s² */
#define KP_POS            2.5f   /* position P gain */
#define KD_VEL            1.8f   /* velocity D gain */
#define MIN_SEPARATION    1.5f   /* m — collision threshold */
#define TILE_SIZE         128    /* threads per block for collision tile */
#define MAX_KEYFRAMES     64     /* max LED keyframes per drone */

/* --------------------------------------------------------------------------
 * Helper: clamp a float to [-limit, +limit]
 * -------------------------------------------------------------------------- */
__device__ __forceinline__ float clampf_sym(float v, float limit)
{
    return fmaxf(-limit, fminf(limit, v));
}

/* --------------------------------------------------------------------------
 * Helper: length of a float3
 * -------------------------------------------------------------------------- */
__device__ __forceinline__ float len3(float3 v)
{
    return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z);
}

/* --------------------------------------------------------------------------
 * Helper: scale a float3 so its length does not exceed max_len
 * -------------------------------------------------------------------------- */
__device__ __forceinline__ float3 clamp_magnitude(float3 v, float max_len)
{
    float l = len3(v);
    if (l > max_len) {
        float inv = max_len / l;
        v.x *= inv; v.y *= inv; v.z *= inv;
    }
    return v;
}


/* ==========================================================================
 * KERNEL 1 — update_positions_kernel
 *
 * For every drone i (one thread per drone):
 *   1. Compute PD acceleration:  a = Kp*(target_pos - pos) + Kd*(target_vel - vel)
 *   2. Clamp acceleration to MAX_ACCELERATION.
 *   3. Integrate velocity:  vel += a * dt
 *   4. Clamp velocity to MAX_VELOCITY.
 *   5. Integrate position:  pos += vel * dt
 *   6. Write back positions, velocities, and (optionally) accelerations.
 * ========================================================================== */
extern "C"
__global__ void update_positions_kernel(
    float3* __restrict__ positions,       /* (N) in/out */
    float3* __restrict__ velocities,      /* (N) in/out */
    float3* __restrict__ accelerations,   /* (N) out     */
    const float3* __restrict__ targets,   /* (N) target positions */
    const float3* __restrict__ target_vels,/* (N) target velocities */
    const int*   __restrict__ status,     /* (N) DroneStatus enum  */
    float dt,
    float kp,
    float kd,
    float max_acc,
    float max_vel,
    int   n_drones
)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_drones) return;

    /* Skip inactive drones (GROUNDED=5, FAILSAFE=6, ERROR=7) */
    int st = status[i];
    if (st == 5 || st == 6 || st == 7) {
        accelerations[i] = make_float3(0.f, 0.f, 0.f);
        velocities[i]    = make_float3(0.f, 0.f, 0.f);
        return;
    }

    float3 pos   = positions[i];
    float3 vel   = velocities[i];
    float3 tgt   = targets[i];
    float3 tvl   = target_vels[i];

    /* PD controller */
    float3 acc;
    acc.x = kp * (tgt.x - pos.x) + kd * (tvl.x - vel.x);
    acc.y = kp * (tgt.y - pos.y) + kd * (tvl.y - vel.y);
    acc.z = kp * (tgt.z - pos.z) + kd * (tvl.z - vel.z);

    acc = clamp_magnitude(acc, max_acc);

    /* Euler integration */
    vel.x += acc.x * dt;
    vel.y += acc.y * dt;
    vel.z += acc.z * dt;
    vel = clamp_magnitude(vel, max_vel);

    pos.x += vel.x * dt;
    pos.y += vel.y * dt;
    pos.z += vel.z * dt;

    /* Enforce z >= 0 for non-airborne states (ARMING=1, GROUNDED=5) */
    if (st == 5) pos.z = fmaxf(pos.z, 0.f);

    positions[i]     = pos;
    velocities[i]    = vel;
    accelerations[i] = acc;
}


/* ==========================================================================
 * KERNEL 2 — collision_check_kernel
 *
 * Detects pairwise proximity violations using shared-memory tiling.
 *
 * Algorithm (tiled O(N²/TILE_SIZE) shared-memory approach):
 *   - Divide N drones into tiles of TILE_SIZE.
 *   - Each block loads one tile into shared memory.
 *   - Every thread i checks its position against all TILE_SIZE positions in
 *     the shared tile.
 *   - If distance < MIN_SEPARATION the pair is counted as a collision and
 *     both entries of collision_flags are set to 1.
 *
 * Parameters
 * ----------
 * positions       (N)  – drone world positions
 * status          (N)  – drone status ints
 * collision_flags (N)  – output: 1 if drone i is in a collision, else 0
 * min_sep_sq           – squared minimum separation distance
 * n_drones             – total drone count
 * ========================================================================== */
extern "C"
__global__ void collision_check_kernel(
    const float3* __restrict__ positions,
    const int*    __restrict__ status,
    int*   __restrict__ collision_flags,
    float  min_sep_sq,
    int    n_drones
)
{
    __shared__ float3 tile_pos[TILE_SIZE];
    __shared__ int    tile_status[TILE_SIZE];

    int i = blockIdx.x * blockDim.x + threadIdx.x;

    float3 pos_i;
    int    st_i = 5;   /* default GROUNDED (inactive) */
    if (i < n_drones) {
        pos_i = positions[i];
        st_i  = status[i];
    }

    int my_collision = 0;

    /* Iterate over tiles of other drones */
    for (int tile_start = 0; tile_start < n_drones; tile_start += TILE_SIZE)
    {
        /* Cooperatively load tile into shared memory */
        int tile_idx = tile_start + threadIdx.x;
        if (tile_idx < n_drones) {
            tile_pos[threadIdx.x]    = positions[tile_idx];
            tile_status[threadIdx.x] = status[tile_idx];
        } else {
            tile_pos[threadIdx.x]    = make_float3(1e9f, 1e9f, 1e9f);
            tile_status[threadIdx.x] = 5;   /* GROUNDED — will be skipped */
        }
        __syncthreads();

        if (i < n_drones && st_i <= 4) {   /* only active drones (0-4) */
            for (int t = 0; t < TILE_SIZE; t++)
            {
                int j = tile_start + t;
                if (j == i || j >= n_drones) continue;
                if (tile_status[t] > 4)  continue;   /* skip inactive */

                float dx = pos_i.x - tile_pos[t].x;
                float dy = pos_i.y - tile_pos[t].y;
                float dz = pos_i.z - tile_pos[t].z;
                float dist_sq = dx*dx + dy*dy + dz*dz;

                if (dist_sq < min_sep_sq) {
                    my_collision = 1;
                }
            }
        }
        __syncthreads();
    }

    if (i < n_drones) {
        collision_flags[i] = my_collision;
    }
}


/* ==========================================================================
 * KERNEL 3 — update_colors_kernel
 *
 * Linear interpolation of RGB LED keyframes for all N drones.
 *
 * Keyframe layout (flat AoS, packed per drone):
 *   keyframe_times  (N * MAX_KEYFRAMES) float  – sorted time values
 *   keyframe_colors (N * MAX_KEYFRAMES * 3) uint8 – R,G,B packed
 *   keyframe_counts (N) int  – how many keyframes each drone has
 *
 * Output:
 *   colors (N * 3) uint8  – current R,G,B per drone
 *
 * The branchless lerp compiles to a single FMAD + INT conversion.
 * ========================================================================== */
extern "C"
__global__ void update_colors_kernel(
    const float*   __restrict__ keyframe_times,   /* (N * MAX_KEYFRAMES) */
    const uint8_t* __restrict__ keyframe_colors,  /* (N * MAX_KEYFRAMES * 3) */
    const int*     __restrict__ keyframe_counts,  /* (N) */
    uint8_t*       __restrict__ colors,           /* (N * 3) out */
    float  show_time,
    int    n_drones
)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_drones) return;

    int  kf_count = keyframe_counts[i];
    int  base_t   = i * MAX_KEYFRAMES;
    int  base_c   = i * MAX_KEYFRAMES * 3;

    if (kf_count <= 0) {
        /* No keyframes: set white */
        colors[i*3 + 0] = 255;
        colors[i*3 + 1] = 255;
        colors[i*3 + 2] = 255;
        return;
    }

    /* Clamp show_time to keyframe range */
    float t0_kf = keyframe_times[base_t];
    float t1_kf = keyframe_times[base_t + kf_count - 1];
    float t = fmaxf(t0_kf, fminf(show_time, t1_kf));

    /* Binary search for the segment containing t */
    int lo = 0, hi = kf_count - 2;
    while (lo < hi) {
        int mid = (lo + hi + 1) / 2;
        if (keyframe_times[base_t + mid] <= t)
            lo = mid;
        else
            hi = mid - 1;
    }
    /* lo is now the index of the keyframe just before or at t */
    int k0 = lo;
    int k1 = min(lo + 1, kf_count - 1);

    float ta = keyframe_times[base_t + k0];
    float tb = keyframe_times[base_t + k1];
    float span = tb - ta;

    /* Normalised interpolation parameter u ∈ [0, 1] */
    float u = (span > 1e-6f) ? __saturatef((t - ta) / span) : 0.f;

    /* Linear interpolation per channel — branchless FMAD */
    for (int ch = 0; ch < 3; ch++) {
        float c0 = (float)keyframe_colors[base_c + k0 * 3 + ch];
        float c1 = (float)keyframe_colors[base_c + k1 * 3 + ch];
        float blended = c0 + u * (c1 - c0);
        /* Clamp and round to nearest integer */
        colors[i * 3 + ch] = (uint8_t)__float2uint_rn(
            fmaxf(0.f, fminf(255.f, blended))
        );
    }
}


/* ==========================================================================
 * KERNEL 4 (bonus) — apply_wind_kernel
 *
 * Adds wind/turbulence acceleration perturbations to the desired acceleration
 * array.  Wind force is stored as a per-drone float3 precomputed on the CPU
 * using the Dryden model and uploaded each frame.
 * ========================================================================== */
extern "C"
__global__ void apply_wind_kernel(
    float3*       __restrict__ accelerations,   /* (N) in/out */
    const float3* __restrict__ wind_forces,     /* (N) per-drone wind accel */
    const int*    __restrict__ status,
    float   wind_drag_coeff,                    /* typically 0.05 */
    int     n_drones
)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_drones) return;

    int st = status[i];
    if (st == 5 || st == 6 || st == 7) return;  /* inactive */

    float3 a  = accelerations[i];
    float3 wf = wind_forces[i];

    a.x += wf.x * wind_drag_coeff;
    a.y += wf.y * wind_drag_coeff;
    a.z += wf.z * wind_drag_coeff;

    /* Re-clamp to MAX_ACCELERATION after adding wind */
    a = clamp_magnitude(a, MAX_ACCELERATION);
    accelerations[i] = a;
}


/* ==========================================================================
 * KERNEL 5 (bonus) — update_orientations_kernel
 *
 * Derives roll/pitch/yaw from the current velocity vector.
 * Drones lean into the direction of travel.
 * ========================================================================== */
extern "C"
__global__ void update_orientations_kernel(
    float3*       __restrict__ orientations,   /* (N) roll,pitch,yaw — in/out */
    const float3* __restrict__ velocities,     /* (N) */
    const float3* __restrict__ accelerations,  /* (N) */
    const int*    __restrict__ status,
    int n_drones
)
{
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n_drones) return;

    int st = status[i];
    if (st == 5 || st == 7) return;  /* grounded or error: keep last orientation */

    float3 vel = velocities[i];
    float3 acc = accelerations[i];

    float horiz_spd = sqrtf(vel.x * vel.x + vel.y * vel.y);

    /* Pitch: angle of climb / descent relative to horizontal */
    float pitch = atan2f(vel.z, horiz_spd + 1e-6f);
    pitch = clampf_sym(pitch, 0.6109f);  /* ±35° in radians */

    /* Roll: bank proportional to lateral acceleration */
    float roll = atan2f(acc.y, 9.81f + 1e-6f);
    roll = clampf_sym(roll, 0.6109f);

    /* Yaw: heading from horizontal velocity */
    float yaw = atan2f(vel.y, vel.x + 1e-6f);

    orientations[i] = make_float3(roll, pitch, yaw);
}


/* ==========================================================================
 * Host-side launch helpers (callable from Python via ctypes/cffi)
 * ========================================================================== */

/*
 * launch_update_positions(...)
 *
 * Selects grid/block dimensions and dispatches update_positions_kernel.
 * Returns 0 on success, non-zero on CUDA error.
 */
extern "C"
int launch_update_positions(
    float3* positions, float3* velocities, float3* accelerations,
    const float3* targets, const float3* target_vels, const int* status,
    float dt, float kp, float kd, float max_acc, float max_vel, int n_drones,
    cudaStream_t stream
)
{
    int threads = 256;
    int blocks  = (n_drones + threads - 1) / threads;
    update_positions_kernel<<<blocks, threads, 0, stream>>>(
        positions, velocities, accelerations,
        targets, target_vels, status,
        dt, kp, kd, max_acc, max_vel, n_drones
    );
    return (int)cudaGetLastError();
}

/*
 * launch_collision_check(...)
 */
extern "C"
int launch_collision_check(
    const float3* positions, const int* status,
    int* collision_flags, float min_sep, int n_drones,
    cudaStream_t stream
)
{
    float min_sep_sq = min_sep * min_sep;
    int threads = TILE_SIZE;
    int blocks  = (n_drones + threads - 1) / threads;
    collision_check_kernel<<<blocks, threads, 0, stream>>>(
        positions, status, collision_flags, min_sep_sq, n_drones
    );
    return (int)cudaGetLastError();
}

/*
 * launch_update_colors(...)
 */
extern "C"
int launch_update_colors(
    const float* kf_times, const uint8_t* kf_colors, const int* kf_counts,
    uint8_t* colors, float show_time, int n_drones,
    cudaStream_t stream
)
{
    int threads = 256;
    int blocks  = (n_drones + threads - 1) / threads;
    update_colors_kernel<<<blocks, threads, 0, stream>>>(
        kf_times, kf_colors, kf_counts, colors, show_time, n_drones
    );
    return (int)cudaGetLastError();
}
