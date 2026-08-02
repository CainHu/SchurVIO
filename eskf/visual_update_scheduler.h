#pragma once

#include <cstddef>
#include <cstdint>

// Compile-time visual measurement scheduling policies.
//
// Override from CMake with, for example:
//   cmake -S . -B build -DSCHUR_VIO_VISUAL_SCHEDULER=LEGACY
// or directly define SCHUR_VIO_VISUAL_SCHEDULER to one of the numeric macros
// below. MSCKF is the default because it consumes every accepted feature
// observation at most once and therefore has the clearest Bayesian lifecycle.
#define SCHUR_VIO_SCHEDULER_LEGACY 0
#define SCHUR_VIO_SCHEDULER_SCHURVINS 1
#define SCHUR_VIO_SCHEDULER_MSCKF 2
#define SCHUR_VIO_SCHEDULER_VINS_MONO 3
#define SCHUR_VIO_SCHEDULER_RDVIO 4

#ifndef SCHUR_VIO_VISUAL_SCHEDULER
#define SCHUR_VIO_VISUAL_SCHEDULER SCHUR_VIO_SCHEDULER_MSCKF
#endif

static_assert(SCHUR_VIO_VISUAL_SCHEDULER >= SCHUR_VIO_SCHEDULER_LEGACY &&
              SCHUR_VIO_VISUAL_SCHEDULER <= SCHUR_VIO_SCHEDULER_RDVIO,
              "Unknown SCHUR_VIO_VISUAL_SCHEDULER value");

namespace slam {
    enum class VisualUpdateScheduler : uint8_t {
        // Historical implementation: keep only keyframes, but execute the same
        // persistent full-window visual batch at every camera timestamp.
        Legacy = SCHUR_VIO_SCHEDULER_LEGACY,
        // SchurVINS paper/code style: augment every image, retain a compact
        // 2-keyframe + recent-frame window, and repeatedly solve active tracks.
        SchurVINS = SCHUR_VIO_SCHEDULER_SCHURVINS,
        // Recommended structureless filter lifecycle: update a track once when
        // it is lost, reaches the clone boundary, or reaches maximum length.
        MSCKF = SCHUR_VIO_SCHEDULER_MSCKF,
        // VINS-Mono frame-retention policy: augment every image and remove the
        // oldest frame or second-newest non-keyframe when the window is full.
        // This project remains an ESKF, so this is a scheduling A/B mode rather
        // than a replacement for VINS-Mono's nonlinear marginalization prior.
        VINSMono = SCHUR_VIO_SCHEDULER_VINS_MONO,
        // RD-VIO-inspired R/N hierarchical scheduling adapted to this ESKF:
        // delayed triangulation and depth-free constraints on rotation frames,
        // plus RR/NN/RN/NR keyframe/subframe transitions.
        RDVIO = SCHUR_VIO_SCHEDULER_RDVIO
    };

    constexpr VisualUpdateScheduler VISUAL_UPDATE_SCHEDULER =
        static_cast<VisualUpdateScheduler>(SCHUR_VIO_VISUAL_SCHEDULER);

    [[nodiscard]] constexpr const char *visualUpdateSchedulerName(
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        switch (scheduler) {
            case VisualUpdateScheduler::Legacy: return "legacy";
            case VisualUpdateScheduler::SchurVINS: return "schurvins";
            case VisualUpdateScheduler::MSCKF: return "msckf";
            case VisualUpdateScheduler::VINSMono: return "vins_mono";
            case VisualUpdateScheduler::RDVIO: return "rdvio";
        }
        return "unknown";
    }

    [[nodiscard]] constexpr bool schedulerAugmentsEveryImage(
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        return scheduler != VisualUpdateScheduler::Legacy;
    }

    [[nodiscard]] constexpr bool schedulerConsumesTracksOnce(
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        return scheduler == VisualUpdateScheduler::MSCKF ||
               scheduler == VisualUpdateScheduler::RDVIO;
    }

    // Number of clones retained after the current image has been processed.
    // One additional current clone can temporarily participate in an update.
    [[nodiscard]] constexpr size_t schedulerRetainedCloneCount(
        const VisualUpdateScheduler scheduler = VISUAL_UPDATE_SCHEDULER) {
        switch (scheduler) {
            case VisualUpdateScheduler::Legacy: return 29;
            case VisualUpdateScheduler::SchurVINS: return 3;
            case VisualUpdateScheduler::MSCKF: return 10;
            case VisualUpdateScheduler::VINSMono: return 10;
            case VisualUpdateScheduler::RDVIO: return 18;
        }
        return 10;
    }
}
