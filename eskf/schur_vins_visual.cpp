/**
 * @file schur_vins_visual.cpp
 * @brief 相机 clone 管理、特征轨迹调度与 Schur 视觉后验。
 *
 * 本文件只负责视觉侧算法。IMU 传播、三角化和外层数据流分别位于
 * schur_vins_imu.cpp、schur_vins_triangulation.cpp 和 schur_vins.cpp。
 * 这样拆分不会改变矩阵装配或浮点运算顺序，只缩小每个编译单元的职责范围。
 */

#include "schur_vins.h"
#include "rdvio_constraints.h"
#include "rdvio_scheduler.h"
#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
#include <Eigen/SparseQR>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

using namespace slam;

void SchurVINS::pushFrame(const CameraData &cam_data, bool is_keyframe) {
    using A = AugState;

    if (!is_keyframe && !framePolicyAugmentsEveryImage()) {
        // KEYFRAME_ONLY 策略不为普通图像分配 clone，也不把该帧像素写入持久轨迹。
        // 轨迹是否在当前图像仍可见，后续直接查询 cam_data.measurements 判断。
        return;
    }

    // 创建关键帧或时间帧 clone。每个 clone 仅保存 6 自由度相机载体位姿误差
    // delta x_c=[delta theta_c, delta p_c]，其物理协方差块由 ordering 标识。
    auto frm = map_.pushFrame(cam_data.timestamp, is_keyframe);
    frm->timestamp = state_.timestamp;
    frm->q() = state_.orientation;
    frm->p() = state_.position;
    // 保存 FEJ（First-Estimate Jacobian）参考位姿。名义位姿会继续被更新，
    // 但 FEJ 副本保持冻结，用来构造全局平移与全局偏航四维不可观子空间。
    frm->record_to_state_fej();

    // 状态克隆相当于线性映射 delta x_c = J_clone delta x_INS，其中当前实现的
    // J_clone 直接从 INS 顶部姿态/位置误差块复制。因此协方差增广满足：
    //   P_xc = P_xx J_clone^T，P_cc = J_clone P_xx J_clone^T。
    // 由于 clone 与当前 INS 使用相同的 6 维姿态/位置误差排列，下面可用块复制
    // 实现同一公式，并通过 ordering 把新 clone 放到固定物理槽位。
    auto idx = map_.getWinLatestIndex();
    const auto i = INSState::SIZE + idx * A::SIZE;
    const auto j = (WIN_SIZE - (idx + 1)) * A::SIZE;
    if constexpr (CONFIG_DEBUG) {
        cov_.middleRows<A::SIZE>(i).noalias() = cov_.topRows<A::SIZE>();
        cov_.middleCols<A::SIZE>(i).noalias() = cov_.leftCols<A::SIZE>();
        cov_.block<A::SIZE, A::SIZE>(i, i).noalias() = cov_.topLeftCorner<A::SIZE, A::SIZE>();
    } else {
        cov_.block(i, 0, A::SIZE, i).noalias() =
            cov_.block(0, 0, A::SIZE, i);
        cov_.block(i, i + A::SIZE, A::SIZE, j).noalias() =
            cov_.block(0, i + A::SIZE, A::SIZE, j);

        cov_.block(0, i, i, A::SIZE).noalias() =
            cov_.block(0, 0, i, A::SIZE);
        cov_.block(i + A::SIZE, i, j, A::SIZE).noalias() =
            cov_.block(i + A::SIZE, 0, j, A::SIZE);

        cov_.block<A::SIZE, A::SIZE>(i, i).noalias() = cov_.topLeftCorner<A::SIZE, A::SIZE>();
    }
    if (cov_.cols() > static_cast<Eigen::Index>(COV_SIZE)) {
        const Eigen::Index persistent_size = cov_.cols() - COV_SIZE;
        cov_.block(i, COV_SIZE, A::SIZE, persistent_size) =
            cov_.block(0, COV_SIZE, A::SIZE, persistent_size);
        cov_.block(COV_SIZE, i, persistent_size, A::SIZE) =
            cov_.block(i, COV_SIZE, A::SIZE, persistent_size).transpose();
    }

//    std::cout << "Output" << std::endl;
}
void SchurVINS::popFrame(const size_t chronological_index) {
    map_.popFrame(chronological_index);
}
void SchurVINS::updateVisual(const CameraData &cam_data, const std::unordered_map<size_t, Vec3> &lmk_map, const double dt) {
    /*
     * 视觉后验总流程（默认 Hybrid MSCKF + Schur）：
     *
     * 1. 帧分类：判断关键帧以及 R（旋转主导）/N（平移主导）运动类型。
     * 2. clone 调度：按帧策略增广当前图像时刻的 IMU 位姿，并在真正删帧前生成待删除集合。
     * 3. 轨迹扫描与分流：已有持久 ID 不进入普通集合；普通轨迹满足 lost、
     *    touchBoundary 或达到长度上限时请求一次性消费；
     *    低视差且仍可见的轨迹可以延迟删除，等待后续平移基线。
     * 4. 普通轨迹准备：由多帧观测三角化世界点并记录几何质量；失败轨迹可准备
     *    无深度旋转残差、延迟消费或归档。
     * 5. 持久点更新：已有持久点用当前关键帧像素直接更新完整联合状态 [x_M, p_L]。
     * 6. 普通轨迹与旋转约束在更新后的状态处线性化：
     *        r_i = z_i - pi(T_ci,w p_f)
     *            ≈ H_x,i delta x + H_f,i delta p_f + n_i。
     * 7. 累加联合正规方程并逐点 Schur 消元：
     *        [Hpp Hpl; Hlp Hll] [delta x; delta p_f] = [gp; gl]，
     *        Hs = Hpp - Hpl Hll^dagger Hlp，
     *        gs = gp  - Hpl Hll^dagger gl。
     * 8. FEJ 可观性约束：保护全局平移 3 维与重力方向偏航 1 维 gauge。
     * 9. 联合 EKF：把 Hs 对角化为互不相关的一维伪量测，用 Joseph 形式更新完整协方差；
     *     虽然普通 MSCKF 的量测雅可比只直接作用于 x_M，P_LM 会把修正传播到已有持久点。
     * 10. 条件初始化：少量成熟普通点复用本次 Hll/Hpl/gl 追加为新持久点；未晋升点只更新
     *     影子候选统计，不再产生第二次导航量测更新，也不让历史 pose 快照进入 H/g。
     *
     * 调度 finalizer 在所有提前返回路径执行，保证轨迹消费和 clone 删除不会遗漏。
     */
    bool is_keyframe = map_.isKeyFrame(cam_data);
    bool is_rotation_frame = false;
    TYPE rdvio_misalignment_deg = TYPE(180);
    uint8_t rdvio_case = 0;

    if constexpr (frame_selection_policy == FrameSelectionPolicy::VINSMono) {
        const VINSMonoKeyframeParameters parameters{
            10,
            vins_mono_min_common_tracks,
            vins_mono_keyframe_parallax_deg,
            vins_mono_max_keyframe_interval_us};
        const VINSMonoFrameDecision decision = decideVINSMonoFrame(
            map_, cam_data, state_.orientation, ext_.q_ic, parameters);
        is_keyframe = decision.is_keyframe;
    }

    // R/N 运动类型判定与 RD-VIO 的关键帧/压窗策略解耦。默认 MSCKF 只读取
    // is_rotation_frame 来启用无深度旋转约束，仍保持自己的 FIFO clone 窗口；
    // 只有显式选择 RDVIO frame policy 时才应用 RR/NN/RN/NR 调度决策。
    const RDVIOParameters rdvio_parameters{
        rdvio_rotation_threshold_deg,
        rdvio_min_common_tracks,
        rdvio_subframe_size,
        rdvio_rotation_compression_trigger,
        framePolicyRetainedCloneCount()};
    const RDVIOFrameDecision motion_decision = decideRDVIOFrame(
        map_, cam_data, state_.orientation, ext_.q_ic,
        is_keyframe, rdvio_parameters);
    is_rotation_frame = motion_decision.is_rotation_frame;
    rdvio_misalignment_deg = motion_decision.misalignment_deg;
    if (rdvio_misalignment_deg < TYPE(179)) {
        if (is_rotation_frame) {
            ++n_rotation_dominant_frames_;
        } else {
            ++n_translation_dominant_frames_;
        }
    }

    // motion_decision 对所有模式都会计算，因为默认 MSCKF 也需要 R/N 标签来决定是否
    // 构造无深度旋转残差；但 RR/NN/RN/NR 的关键帧切换和压窗动作只属于显式 RDVIO
    // 帧策略。promote_previous_to_keyframe 用于在运动段切换时回溯确认上一帧为关键帧。
    if constexpr (frame_selection_policy == FrameSelectionPolicy::RDVIO) {
        const RDVIOFrameDecision &decision = motion_decision;
        is_keyframe = decision.is_keyframe;
        rdvio_case = static_cast<uint8_t>(decision.transition);
        if (decision.promote_previous_to_keyframe && !map_.sfw.empty()) {
            map_.sfw[map_.sfw.size() - 1]->is_key_frame = true;
        }
        if (decision.transition != RDVIOCase::None) {
            ++n_rdvio_cases_[rdvio_case];
        }
        if (is_rotation_frame) {
            ++n_rdvio_rotation_frames_;
        } else {
            ++n_rdvio_normal_frames_;
        }
    }
    // 关键帧统计描述“策略判定”，store_current_frame 描述“是否创建 clone”，二者不能
    // 混为一谈：除 KeyframeOnly 外，其余策略即使判为非关键帧，也会为该图像增广 clone。
    if (is_keyframe) {
        ++n_keyframes_selected_;
    } else {
        ++n_nonkeyframes_selected_;
    }
    const bool store_current_frame = is_keyframe || framePolicyAugmentsEveryImage();
    Frame *current_frame = nullptr;
    if (store_current_frame) {
        // addObservations() 会把当前图像的全部 ID 临时写入 Map，包括已存在于联合状态中的
        // 持久 ID。后面的 isPersistentLandmark() 分支会立即把这些 ID 与普通 MSCKF 集合
        // 分开，并由 finalizer 清理临时 Map 轨迹，因此持久像素不会再次进入 ids/Hpl/Hll。
        pushFrame(cam_data, is_keyframe);
        ++n_frames_stored_;
        current_frame = map_.getWinLatestFrame();
        current_frame->is_rotation_frame = is_rotation_frame;
        current_frame->rdvio_case = rdvio_case;
        current_frame->rdvio_misalignment_deg = rdvio_misalignment_deg;
        map_.addObservations(current_frame, cam_data);
    }

    // 低视差轨迹被最终丢弃时，只归档首尾 bearing、相机位姿快照和几何质量摘要。
    // 这里用新观测更新影子候选证据；归档 pose 不属于当前滤波状态，绝不能直接重建
    // H/g，否则会忽略该 pose 被边缘化时丢失的相关性并造成虚假信息增益。
    pruneDeferredTrackArchives(cam_data.timestamp);
    if (current_frame && enable_hybrid_persistent_landmarks_ &&
        schedulerConsumesTracksOnce(visual_update_scheduler)) {
        for (const auto &[id, measurement] : cam_data.measurements) {
            updateShadowCandidateFromArchive(
                id, *current_frame, measurement, cam_data.timestamp);
        }
    }

    // 必须先规划本帧结束时要删除哪些 clone，再判断哪些轨迹触及窗口边界。若先删 clone，
    // 对应 Observation/Feature 引用会消失，普通 MSCKF 将失去在边缘化前消费整条轨迹的机会。
    const size_t current_win_size = map_.sfw.size();
    std::vector<size_t> frames_to_remove;
    size_t rdvio_compressed_frames = 0;
    if constexpr (frame_selection_policy == FrameSelectionPolicy::KeyframeOnly) {
        // 只保存关键帧；超预算时删除时间最早的关键帧 clone。
        if (current_win_size > framePolicyRetainedCloneCount()) {
            frames_to_remove.push_back(0);
        }
    } else if constexpr (
        frame_selection_policy == FrameSelectionPolicy::KeyframeRedundancy) {
        if (current_win_size > framePolicyRetainedCloneCount()) {
            KeyframeRedundancyParameters parameters;
            parameters.minimum_parallax_deg = triangulation_min_parallax_deg;
            const KeyframeRemovalDecision decision =
                planKeyframeRedundancyRemoval(
                    map_, ext_.q_ic, framePolicyRetainedCloneCount(), parameters);

            // 规划器异常或候选集合为空时仍使用最老帧，保证窗口上界和默认 MSCKF
            // 的确定性行为不被破坏。正常路径会同时记录被选帧的几何诊断量，供
            // frame_policy_summary.csv 判断“非最老删除”是否真的减少了低视差损失。
            const size_t selected_index = decision.valid
                ? decision.selected.chronological_index
                : size_t(0);
            frames_to_remove.push_back(selected_index);
            ++n_keyframe_redundancy_removals_;
            if (selected_index != 0) {
                ++n_keyframe_redundancy_nonoldest_removals_;
            }
            if (!decision.valid || decision.used_oldest_fallback) {
                ++n_keyframe_redundancy_oldest_fallbacks_;
            }
            if (decision.valid) {
                n_keyframe_redundancy_selected_low_parallax_tracks_ +=
                    decision.selected.low_parallax_tracks;
                n_keyframe_redundancy_selected_unique_tracks_ +=
                    decision.selected.unique_geometry_tracks;
                keyframe_redundancy_score_sum_ += decision.selected.score;
                keyframe_redundancy_ratio_sum_ +=
                    decision.selected.redundancy_ratio;
                keyframe_redundancy_parallax_loss_sum_ +=
                    decision.selected.mean_parallax_loss_ratio;
            }
        }
    } else if constexpr (frame_selection_policy == FrameSelectionPolicy::KeyframePriority) {
        if (current_win_size > framePolicyRetainedCloneCount()) {
            // 始终保留当前图像，然后优先保留较新的关键帧；剩余预算再由较新的
            // 时间帧补足。预算为 3 时退化为“两个关键帧 + 一个最近帧”，统一
            // 预算消融时则保持同样的关键帧优先原则。
            std::vector<bool> keep(current_win_size, false);
            keep.back() = true;
            size_t kept = 1;
            for (size_t i = current_win_size; i > 0 && kept <
                 framePolicyRetainedCloneCount(); --i) {
                const size_t index = i - 1;
                if (!keep[index] && map_.sfw[index]->is_key_frame) {
                    keep[index] = true;
                    ++kept;
                }
            }
            for (size_t i = current_win_size; i > 0 && kept <
                 framePolicyRetainedCloneCount(); --i) {
                const size_t index = i - 1;
                if (!keep[index]) {
                    keep[index] = true;
                    ++kept;
                }
            }
            for (size_t i = 0; i < current_win_size; ++i) {
                if (!keep[i]) {
                    frames_to_remove.push_back(i);
                    break;
                }
            }
        }
    } else if constexpr (frame_selection_policy == FrameSelectionPolicy::FIFO) {
        // 标准 MSCKF 固定长度滑窗：每张图像都增广，超预算后删除最老 clone。
        if (current_win_size > framePolicyRetainedCloneCount()) {
            frames_to_remove.push_back(0);
        }
    } else if constexpr (frame_selection_policy == FrameSelectionPolicy::VINSMono) {
        // 对齐 VINS-Mono 的滑窗语义：次新帧不是关键帧时优先删除次新帧；若次新帧已被
        // 确认为关键帧，则保留关键帧并删除最老帧。这里只比较帧选择/压窗，不引入 BA。
        if (current_win_size > framePolicyRetainedCloneCount()) {
            const size_t second_newest = current_win_size - 2;
            frames_to_remove.push_back(map_.sfw[second_newest]->is_key_frame
                ? size_t(0) : second_newest);
        }
    } else if constexpr (frame_selection_policy == FrameSelectionPolicy::RDVIO) {
        // 当前 ESKF 没有可在删中间帧时拼接的 clone 间预积分因子，因此这里只
        // 复用 RD-VIO 的 R/N 分类与窗口压缩；触及被删 clone 的轨迹必须先消费。
        const RDVIOParameters parameters{
            rdvio_rotation_threshold_deg,
            rdvio_min_common_tracks,
            rdvio_subframe_size,
            rdvio_rotation_compression_trigger,
            framePolicyRetainedCloneCount()};
        RDVIOFrameRemovalPlan removal_plan =
            planRDVIOFrameRemovals(map_, parameters);
        frames_to_remove = std::move(removal_plan.chronological_indices);
        rdvio_compressed_frames = removal_plan.compressed_frame_count;
    }

    // 删除计划可能来自多段 RD-VIO 压缩，先排序去重；同时在删除 clone 之前保存 FrameID，
    // 因为轨迹调度按稳定的 FrameID 查询是否触及边界，而不是按会移动的窗口下标查询。
    std::sort(frames_to_remove.begin(), frames_to_remove.end());
    frames_to_remove.erase(
        std::unique(frames_to_remove.begin(), frames_to_remove.end()),
        frames_to_remove.end());
    std::vector<FrameID> frame_ids_to_remove;
    frame_ids_to_remove.reserve(frames_to_remove.size());
    for (const size_t index : frames_to_remove) {
        frame_ids_to_remove.push_back(map_.sfw[index]->id);
    }

    std::vector<LandmarkID> tracks_to_consume;
    std::vector<LandmarkID> persistent_tracks_to_clear;
    size_t one_shot_tracks_used = 0;
    // RAII finalizer 覆盖函数中所有提前 return：
    // 1) 先删除已经消费的普通轨迹以及持久 ID 在 Map 中的临时轨迹，解除它们对帧的引用；
    // 2) 再按下标逆序删除 clone，避免先删低下标导致后续下标整体左移；
    // 3) persistent_tracks_to_clear 只清理 Map::lmk_map 中的临时对象，不会删除联合状态里的
    //    PersistentLandmark，也不会缩减 P_LL/P_LM。
    ExitHandler schedule_finalizer([&] {
        if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
            n_tracks_consumed_ += tracks_to_consume.size();
            n_tracks_dropped_ += tracks_to_consume.size() - one_shot_tracks_used;
            for (const LandmarkID id : tracks_to_consume) {
                map_.removeLandmark(id);
            }
            for (const LandmarkID id : persistent_tracks_to_clear) {
                map_.removeLandmark(id);
            }
        }
        if constexpr (frame_selection_policy == FrameSelectionPolicy::RDVIO) {
            n_rdvio_compressed_frames_ += rdvio_compressed_frames;
        }
        for (auto index = frames_to_remove.rbegin();
             index != frames_to_remove.rend(); ++index) {
            popFrame(*index);
        }
    });

    // 少于两个 clone 时既无法三角化，也无法构造相邻 bearing 的旋转约束；仍让 finalizer
    // 执行，以便完成本帧可能产生的轨迹/窗口清理。
    if (current_win_size < 2) {
        ++n_visual_updates_skipped_;
        return;
    }

    auto observedInFrame = [](const Landmark &landmark, const Frame *frame) {
        return frame && landmark.frm2fet.find(frame->id) != landmark.frm2fet.end();
    };
    auto isSchurVinsActiveTrack = [&](const Landmark &landmark) {
        const size_t recent_count = std::min<size_t>(2, map_.sfw.size());
        for (size_t offset = 0; offset < recent_count; ++offset) {
            if (observedInFrame(landmark, map_.sfw[map_.sfw.size() - 1 - offset])) {
                return true;
            }
        }
        return false;
    };

    // ids、track_qualities、track_continues_after_update 三个数组按同一下标严格对齐：
    // ids[i] 给出参与普通 Schur 的轨迹，另外两个数组分别服务于质量排序和“当前仍可见”判断。
    // static ids 先 resize 再 clear 只是复用上次分配的容量，不代表预先填入有效元素。
    // rotation_only_tracks 会在本帧后删除；retained_rotation_only_tracks 只消费最新旋转像素，
    // 轨迹本体继续留在 Map 中等待平移基线。
    size_t num_obs = 0;
    static std::vector<std::pair<LandmarkID, Landmark*>> ids;
    ids.resize(map_.lmk_map.size());
    ids.clear();
    std::vector<TrackGeometryQuality> track_qualities;
    track_qualities.reserve(map_.lmk_map.size());
    std::vector<bool> track_continues_after_update;
    track_continues_after_update.reserve(map_.lmk_map.size());
    std::vector<Landmark *> rotation_only_tracks;
    std::vector<Landmark *> retained_rotation_only_tracks;

    for (const auto &[id, lmk] : map_.lmk_map) {
        // 已有持久 ID 的当前像素由 updatePersistentLandmarks() 直接从 cam_data 读取。
        // 这里必须 continue，保证 Z^P（持久点直接观测）与 Z^M（普通 MSCKF 观测）互斥；
        // persistent_tracks_to_clear 仅记录 addObservations() 临时创建、帧末需清理的 Map 轨迹。
        if (isPersistentLandmark(id)) {
            if (cam_data.measurements.find(id) != cam_data.measurements.end()) {
                persistent_tracks_to_clear.push_back(id);
            }
            continue;
        }
        const size_t observation_count = lmk->frm2fet.size();
        // 三个布尔量分别表达不同阶段，不能合并：
        // requested_consumption：原始一次性调度器是否因 lost/触边/长度上限要求结算轨迹；
        // consume_track：经过低视差延迟判定后，本帧是否真正删除该轨迹；
        // schedule_track：本帧是否进入三角化和普通重投影线性化。
        bool schedule_track = true;
        bool consume_track = false;
        bool requested_consumption = false;
        bool lost = false;

        if constexpr (visual_update_scheduler == VisualUpdateScheduler::SchurVINS) {
            schedule_track = isSchurVinsActiveTrack(*lmk);
        } else if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
            const bool observed_current =
                cam_data.measurements.find(id) != cam_data.measurements.end();
            lost = !observed_current;
            const bool touches_marginalized_clone = std::any_of(
                frame_ids_to_remove.begin(), frame_ids_to_remove.end(),
                [&](const FrameID frame_id) {
                    return lmk->frm2fet.find(frame_id) != lmk->frm2fet.end();
                });
            const bool reached_track_limit =
                observation_count >= framePolicyRetainedCloneCount();
            consume_track = lost || touches_marginalized_clone || reached_track_limit;
            requested_consumption = consume_track;
            schedule_track = consume_track;
        }

        // 单观测点的 H_f 为 2x3，消元后不能独立约束位姿。若它已经触发一次性消费，
        // 仍加入 tracks_to_consume，最终计入 dropped 并清理，避免无效单点长期滞留。
        if (!schedule_track || observation_count <= 1) {
            if (consume_track) {
                tracks_to_consume.push_back(id);
            }
            continue;
        }

        // last_triangulation_frame_id 防止在完全相同的观测集合上重复做昂贵三角化；只要
        // 最新观测帧变化，就允许重新尝试。RD-VIO 的 R 帧显式跳过深度初始化，因为纯旋转
        // 不提供三角化基线。失败时仍保存视差、条件数、重投影 RMSE，供延迟和影子筛选使用。
        // GroundTruth/OraclePosition 是实验诊断模式，不属于默认实际估计路径。
        const FrameID latest_observation_frame_id = lmk->frm2fet.empty()
            ? FrameID(0)
            : lmk->frm2fet.rbegin()->first;
        TriangulationStatus latest_triangulation_status =
            lmk->is_triangulated
                ? TriangulationStatus::Success
                : TriangulationStatus::InsufficientViews;
        if (!lmk->is_triangulated &&
            !(visual_update_scheduler == VisualUpdateScheduler::RDVIO &&
              current_frame && current_frame->is_rotation_frame) &&
            lmk->last_triangulation_frame_id != latest_observation_frame_id) {
            lmk->last_triangulation_frame_id = latest_observation_frame_id;
            if (landmark_initialization_mode_ == LandmarkInitializationMode::GroundTruth) {
                const auto truth = lmk_map.find(id);
                if (truth != lmk_map.end()) {
                    lmk->position = truth->second;
                    lmk->cov_position = Mat3_3::Identity() * TYPE(1e-4);
                    lmk->shadow_position = lmk->position;
                    lmk->shadow_cov_position = lmk->cov_position;
                    lmk->shadow_initialized = true;
                    lmk->is_triangulated = true;
                }
            } else {
                const auto triangulation = triangulateLandmark(*lmk);
                latest_triangulation_status = triangulation.status;
                lmk->geometry_max_parallax_deg = triangulation.max_parallax_deg;
                lmk->geometry_condition_number = triangulation.condition_number;
                lmk->geometry_reprojection_rmse = triangulation.reprojection_rmse;
                lmk->geometry_observation_count = triangulation.observation_count;
                logTriangulationAttempt(*lmk, triangulation, cam_data.timestamp, lmk_map);
                if (triangulation.status == TriangulationStatus::Success) {
                    lmk->position = triangulation.position;
                    lmk->cov_position = triangulation.covariance;
                    if (landmark_initialization_mode_ ==
                        LandmarkInitializationMode::TriangulationWithOraclePosition) {
                        const auto truth = lmk_map.find(id);
                        if (truth != lmk_map.end()) {
                            lmk->position = truth->second;
                        }
                    }
                    lmk->shadow_position = lmk->position;
                    lmk->shadow_cov_position = lmk->cov_position;
                    lmk->shadow_initialized = true;
                    lmk->is_triangulated = true;
                    lmk->geometry_position_std = std::sqrt(std::max(
                        TYPE(0), triangulation.covariance.trace() / TYPE(3)));
                    lmk->geometry_score = evaluateTrackGeometry(*lmk).score;
                }
            }
        }

        // 低视差且当前仍可见的轨迹可以撤销“本帧删除”，继续等待未来平移基线。注意此时
        // requested_consumption 保持为 true：这样仍可先提取最新一对无深度旋转信息，而
        // consume_track=false 则保证轨迹本体和未消费像素继续保留。
        if (consume_track && !lmk->is_triangulated &&
            shouldDeferTrackConsumption(
                *lmk, latest_triangulation_status, lost)) {
            consume_track = false;
            schedule_track = false;
            ++lmk->deferred_consumption_count;
            ++n_tracks_deferred_;
        }
        if (requested_consumption && !lmk->is_triangulated &&
            observation_count > 1 &&
            enable_depth_free_rotation_constraints_) {
            // 即使轨迹因仍在视野中而被延迟，也先消费其中最新的一对纯旋转观测。
            // 这些像素的 visual_update_count 会被置位；后续若获得平移基线，深度
            // Schur 更新只使用尚未消费的观测，保持“一条像素量测只用一次”。
            if (consume_track) {
                rotation_only_tracks.push_back(lmk);
            } else {
                retained_rotation_only_tracks.push_back(lmk);
            }
        }
        // 只有最终确定删除且仍无法三角化的轨迹才写入历史摘要；被延迟的轨迹仍拥有完整
        // Map 观测，不需要归档。tracks_to_consume 的实际删除统一交给 finalizer。
        if (consume_track && !lmk->is_triangulated) {
            archiveDeferredTrack(
                *lmk, latest_triangulation_status, cam_data.timestamp);
        }
        if (consume_track) {
            tracks_to_consume.push_back(id);
        }

        // 只有成功获得深度的普通轨迹才进入 ids，并同步追加质量、生命周期和观测数。
        // one_shot_tracks_used 表示“消费轨迹中确实贡献了普通视觉因子”的数量，最终用于
        // 区分 consumed 与 dropped；它不等于像素数，也不包含仍保留的旋转轨迹。
        if (lmk->is_triangulated) {
            ids.emplace_back(id, lmk);
            track_qualities.push_back(evaluateTrackGeometry(*lmk));
            track_continues_after_update.push_back(!lost);
            num_obs += observation_count;
            if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
                ++one_shot_tracks_used;
            }
        }
    }

    // 更新顺序是“已有持久点直接联合 EKF”在前，“普通 MSCKF Schur 更新”在后。
    // 两者不重复使用像素：已有持久 ID 已在上面的循环中 continue，不会进入 ids；普通
    // MSCKF 随后虽能通过 P_LM 间接修正持久点，但使用的是另一组普通轨迹观测。
    const size_t persistent_updates = updatePersistentLandmarks(
        cam_data, current_frame, is_keyframe);

    // 晋升分两层：本段只根据历史稳定证据、当前轨迹质量、总预算和图像网格设置“允许尝试”；
    // 真正追加联合状态还要等普通轨迹完成线性化，并通过有效观测数、Hll 可逆性和回代检查。
    std::vector<bool> promotion_candidates(ids.size(), false);
    if (enable_hybrid_persistent_landmarks_ &&
        schedulerConsumesTracksOnce(visual_update_scheduler) &&
        persistent_landmarks_.size() < persistent_landmark_budget_) {
        std::vector<size_t> ranked_candidates;
        for (size_t index = 0; index < ids.size(); ++index) {
            // 只晋升本次更新后仍可见的点。已经 lost 的轨迹即使几何很好，也无法在下一帧
            // 作为持久点继续提供直接重投影观测，因而不占用固定持久点预算。
            if (track_continues_after_update[index] &&
                shadowCandidateReady(ids[index].first, track_qualities[index])) {
                ranked_candidates.push_back(index);
            }
        }
        std::sort(
            ranked_candidates.begin(), ranked_candidates.end(),
            [&](const size_t lhs, const size_t rhs) {
                return track_qualities[lhs].score > track_qualities[rhs].score;
            });
        const size_t remaining_budget =
            persistent_landmark_budget_ - persistent_landmarks_.size();

        // 持久点的价值来自长期、方向互补的几何，而不是单一区域的重复纹理。
        // 将归一化像平面 [-1,1]x[-0.75,0.75] 划成网格，每格限制晋升数；
        // 边界外的观测夹到最近网格，避免异常大坐标产生越界。
        const size_t grid_columns = std::max<size_t>(1, persistent_grid_columns_);
        const size_t grid_rows = std::max<size_t>(1, persistent_grid_rows_);
        std::vector<size_t> grid_occupancy(grid_columns * grid_rows, 0);
        const auto gridCell = [&](const Vec2 &measurement) {
            const TYPE normalized_x = std::clamp(
                (measurement.x() + TYPE(1)) / TYPE(2), TYPE(0), TYPE(1));
            const TYPE normalized_y = std::clamp(
                (measurement.y() + TYPE(0.75)) / TYPE(1.5), TYPE(0), TYPE(1));
            const size_t column = std::min(
                grid_columns - 1,
                static_cast<size_t>(normalized_x * TYPE(grid_columns)));
            const size_t row = std::min(
                grid_rows - 1,
                static_cast<size_t>(normalized_y * TYPE(grid_rows)));
            return row * grid_columns + column;
        };
        // 只有当前图像实际可见的已有持久点占用当前网格；不可见点不应永久封锁该区域。
        for (const auto &persistent : persistent_landmarks_) {
            const auto measurement = cam_data.measurements.find(persistent.id);
            if (measurement != cam_data.measurements.end()) {
                ++grid_occupancy[gridCell(measurement->second)];
            }
        }

        size_t selected = 0;
        for (const size_t candidate_index : ranked_candidates) {
            if (selected >= remaining_budget) {
                break;
            }
            const auto measurement =
                cam_data.measurements.find(ids[candidate_index].first);
            if (measurement == cam_data.measurements.end()) {
                continue;
            }
            const size_t cell = gridCell(measurement->second);
            if (grid_occupancy[cell] >= persistent_grid_cell_quota_) {
                continue;
            }
            // 此处只是候选标记，不做 EKF 更新，也不修改协方差；条件初始化在普通 MSCKF
            // 联合状态修正完成后，复用该轨迹已经构造的 Hll/Hpl/gl 执行。
            promotion_candidates[candidate_index] = true;
            ++grid_occupancy[cell];
            ++selected;
        }
    }

    const bool has_rdvio_zero_translation =
        visual_update_scheduler == VisualUpdateScheduler::RDVIO &&
        current_frame && current_frame->is_rotation_frame &&
        map_.sfw.size() >= 2;
    // 第一种空更新：持久点、普通点、旋转约束和 RD-VIO 零平移先验均不存在，计为 skipped。
    if (ids.empty() && rotation_only_tracks.empty() &&
        retained_rotation_only_tracks.empty() &&
        !has_rdvio_zero_translation && persistent_updates == 0) {
        ++n_visual_updates_skipped_;
        return;
    }
    // 第二种空更新：已有持久点刚刚已经完成直接 EKF，但本帧没有后续普通/旋转因子。
    // 直接返回即可，不能再记 skipped，否则统计会把有效的持久点更新误报为空更新。
    if (ids.empty() && rotation_only_tracks.empty() &&
        retained_rotation_only_tracks.empty() &&
        !has_rdvio_zero_translation) {
        return;
    }

    // 三类调度器沿用各自历史噪声语义：
    // 1) MSCKF/RD-VIO 一次性轨迹：R_uv = sigma_uv^2 * msckf_visual_noise_scale；
    //    每个像素批次只使用一次，不能再除以 dt，否则相机频率越高会凭空增加信息量。
    // 2) SchurVINS 历史活跃轨迹路径：保持 R_uv = sigma_uv^2。
    // 3) Legacy/VINS-Mono 等重复窗口对照：把 uv_var 视为连续时间信息密度，离散成 uv_var/dt。
    const TYPE image_variance = std::max(
        triangulation_uv_std * triangulation_uv_std, TYPE(1e-12));
    const TYPE visual_batch_variance = [&] {
        if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
            return image_variance * std::max(msckf_visual_noise_scale, TYPE(1));
        } else if constexpr (visual_update_scheduler == VisualUpdateScheduler::SchurVINS) {
            return image_variance;
        } else {
            return uv_var / std::max(TYPE(dt), TYPE(1e-6));
        }
    }();


// 当前项目的统一验证/报告路径。QR 实现保留用于历史 A/B 对照，但默认不编译执行。
// 后续视觉算法修改应先保证 USE_SCHUR 路径正确，再按需单独回归 QR。
//#define USE_QR
#define USE_SCHUR
#if defined(USE_QR) && defined(USE_SCHUR)
#error "USE_QR and USE_SCHUR are mutually exclusive"
#endif
#if defined(USE_QR)
    auto t1 = clock();

    constexpr static size_t UV_SIZE = 2;
    MatXX J_POSE = MatXX::Zero(UV_SIZE * WIN_SIZE, AugState::SIZE * WIN_SIZE);
    // 外参雅可比: 仅在估计外参时才分配(见 ExtState::ESTIMATE_EXTRINSIC)
    MatXX J_EXT = MatXX::Zero(ExtState::ESTIMATE_EXTRINSIC ? UV_SIZE * WIN_SIZE : 0,
                              ExtState::ESTIMATE_EXTRINSIC ? ExtState::SIZE : 0);
    MatXX J_LMK = MatXX::Zero(UV_SIZE * WIN_SIZE, LMK_SIZE);
    VecX ERR = VecX::Zero(UV_SIZE * WIN_SIZE);
    std::vector<FrameOrder> pose_order;
    pose_order.reserve(WIN_SIZE);

    MatXX J_STATE = MatXX::Zero(UV_SIZE * num_obs, AugState::SIZE * WIN_SIZE);
    VecX E_STATE = VecX::Zero(UV_SIZE * num_obs);

    MatXX Q1Jp_s = MatXX::Zero(LMK_SIZE * ids.size(), AugState::SIZE * WIN_SIZE);
    VecX Q1e_s = VecX::Zero(LMK_SIZE * ids.size());
    MatXX RP_s = MatXX::Zero(LMK_SIZE * ids.size(), LMK_SIZE);

    auto t_stage1 = clock();

    // 遍历 landmark
    size_t row_idx = 0;
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto id = ids[i].first;
        auto lmk = ids[i].second;
        pose_order.clear();

        // 遍历关键帧的观测（只处理滑窗内的关键帧）
        for (auto &it : lmk->frm2fet) {
            const auto fet = it.second;
            const auto obs = fet->obs[0];
            const auto frm = fet->frame;

            const auto Rwi = frm->q().toRotationMatrix();
            const auto Ric = ext_.q_ic.toRotationMatrix();
            const auto d_ij_w = lmk->position - frm->p();
            const auto d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
            const auto d_cj_c = Ric.transpose() * d_cj_i;
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const auto inv_d2 = inv_d * inv_d;
            const auto est = d_cj_c.head<2>() * inv_d;
            const auto err = obs->un_pt.head<2>() - est;

            Mat2_3 J;
            J << inv_d, TYPE(0), -d_cj_c.x() * inv_d2,
                    TYPE(0), inv_d, -d_cj_c.y() * inv_d2;

            Mat2_3 J_lmk = J * (frm->q() * ext_.q_ic).inverse().toRotationMatrix();

            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_lmk * hat(d_ij_w);;
            J_pose.rightCols<3>().noalias() = -J_lmk;

            const size_t row_start = UV_SIZE * pose_order.size();
            const size_t col_start = AugState::SIZE * pose_order.size();
            ERR.segment<2>(row_start) = err;
            J_LMK.middleRows<2>(row_start) = J_lmk;
            J_POSE.block<2, AugState::SIZE>(row_start, col_start) = J_pose;

            // 外参雅可比: 保留代码但默认不运行(外参目前不在状态里，J_EXT 无人读取)。
            // 用 if constexpr 而非 #ifdef，这样它始终参与语法/类型检查，不会腐烂。
            if constexpr (ExtState::ESTIMATE_EXTRINSIC) {
                Mat2_6 J_ext;
                J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
                J_ext.leftCols<3>().noalias() = -J_ext.rightCols<3>() * hat(d_cj_i);
                J_EXT.middleRows<2>(row_start) = J_ext;
            }

            // 记录 J_POSE 中的 J_pose 在 state 中对应的 ordering
            pose_order.emplace_back(frm->ordering);
        }

        // 注意：非关键帧的观测暂不在这里处理，稍后单独更新 landmark
        // 联合量测方程：[J_POSE, J_LMK] [dxp; dxl] = e。
        // 对点雅可比做列选主元 QR：
        //   J_LMK = Q [R; 0] P^-1 = [Q1,Q2] [R;0] P^-1。
        // 左乘 Q^T 后得到：
        //  Q1^T * [J_POSE, J_LMK] * dx = [Q1^T * J_POSE, R * P^-1] * [dxp; dxl]
        //                              = Q1^T * J_POSE * dxp + R * P^-1 * dxl
        //                              = Q1^T * e
        // 以及：
        //  Q2^T * [J_POSE, J_LMK] * dx = [Q2^T * J_POSE, 0] * [dxp; dxl]
        //                              = Q2^T * J_POSE * dxp
        //                              = Q2^T * e
        // 1）先用 Q2^T J_POSE dxp = Q2^T e 消去点后更新状态；
        // 2）再用 R P^-1 dxl = Q1^T e - Q1^T J_POSE dxp 回代点增量。
        const size_t row_end = UV_SIZE * pose_order.size();
        const size_t col_end = AugState::SIZE * pose_order.size();
        auto &&J_lmk = J_LMK.topRows(row_end);
        auto &&J_pose = J_POSE.topLeftCorner(row_end, col_end);
        auto &&qr_lmk = J_lmk.colPivHouseholderQr();
        auto &&Q = qr_lmk.householderQ();
        const MatXX R = qr_lmk.matrixR().topLeftCorner(LMK_SIZE, LMK_SIZE).template triangularView<Eigen::Upper>();
        auto &&P = qr_lmk.colsPermutation();

        // [Q1^T * e; Q2^T * e]
        auto &&QTe = Q.transpose() * ERR.head(row_end);
        auto &&Q1e = QTe.head(LMK_SIZE);
        auto &&Q2e = QTe.tail(Q.cols() - LMK_SIZE);

        // [Q1^T * J_POSE; Q2^T * J_POSE]
        auto &&QTJp = Q.transpose() * J_pose;
        auto &&Q1Jp = QTJp.topRows(LMK_SIZE);
        auto &&Q2Jp = QTJp.bottomRows(Q.cols() - LMK_SIZE);

        // 存储 Augment State 对应的 Jacobian
        for (size_t j = 0; j < pose_order.size(); ++j) {
            J_STATE.block(row_idx, AugState::SIZE * pose_order[j], Q2Jp.rows(), AugState::SIZE) = Q2Jp.middleCols(AugState::SIZE * j, AugState::SIZE);
        }
        E_STATE.segment(row_idx, Q2e.rows()) = Q2e;

        // 存储 Landmark 相关的信息
        for (size_t j = 0; j < pose_order.size(); ++j) {
            Q1Jp_s.block(LMK_SIZE * i, AugState::SIZE * pose_order[j], Q1Jp.rows(), AugState::SIZE) = Q1Jp.middleCols(AugState::SIZE * j, AugState::SIZE);
        }
        Q1e_s.segment(LMK_SIZE * i, LMK_SIZE) = Q1e;
        RP_s.middleRows(LMK_SIZE * i, LMK_SIZE) = R * P.transpose();

        row_idx += Q2Jp.rows();
    }
//    std::cout << "Update Finished" << std::endl;

    auto t_stage2 = clock();
    t_perlmk_qr_ += t_stage2 - t_stage1;

    // 对 J_STATE 进行 QR 分解
    //
    // 优化点1: 用 householderQr 取代 colPivHouseholderQr。
    //   列选主元对 (2*num_obs) x 180 的矩阵开销很大，而这里不需要 rank-revealing:
    //   即使 J_STATE 降秩，R 的对应行会趋于 0，序贯更新时 hT ~ 0 => K ~ 0，
    //   该行自然不贡献修正量，退化是平滑的。同时省掉了 R * P^T 这次乘法。
    //
    // 优化点2: 把 E_STATE 拼成增广矩阵 [J_STATE, E_STATE] 一起分解，
    //   R 的最后一列前 n 行即 (Q^T * E_STATE).head(n)，省掉单独应用一次
    //   Householder 序列。同时只对前 row_idx 行分解（后面是未填充的零行）。
    //   注: 实测这两点对总耗时无可测量的影响(提速几乎全部来自优化点1)，
    //   保留是因为省了一趟 O(m*n) 运算且代码更紧凑，不是性能考虑。
    const size_t n_cols = AugState::SIZE * WIN_SIZE;
    const size_t m_eff = row_idx;
    const size_t n_eff = std::min(m_eff, n_cols);

    MatXX JE = MatXX::Zero(m_eff, n_cols + 1);
    JE.leftCols(n_cols) = J_STATE.topRows(m_eff);
    JE.col(n_cols) = E_STATE.head(m_eff);

    auto qr = JE.householderQr();
    auto &&QR = qr.matrixQR();

    // R_red: n_eff x n_cols 的上三角部分；H_red 不再需要乘 permutation
    const MatXX H_red = QR.topLeftCorner(n_eff, n_cols).template triangularView<Eigen::Upper>();

    // e_red: 增广列的前 n_eff 行，即 (Q^T * E_STATE).head(n_eff)
    const VecX e_red = QR.col(n_cols).head(n_eff);

    auto t_stage3 = clock();
    t_bigqr_ += t_stage3 - t_stage2;

    // 序贯更新 State
    // Q2^T * J_POSE * dxp = Q2^T * e
    auto &&cov_p = cov_;
    VecX dx_p = VecX::Zero(COV_SIZE);
    VecX hT = VecX::Zero(INSState::SIZE + AugState::SIZE * WIN_SIZE);
    for (size_t j = 0; j < n_eff; ++j) {
        // 重构出量测矩阵 H（INS 部分恒为 0，只需重写 tail）
        hT.tail(AugState::SIZE * WIN_SIZE) = H_red.row(j).transpose();

        TYPE r = visual_batch_variance;
        VecX PhT = cov_p * hT;
        TYPE var = hT.dot(PhT) + r;
        VecX K = PhT / var;
        cov_p -= K * PhT.transpose();

        PhT = cov_p * hT;
        cov_p.triangularView<Eigen::Upper>() += (K * r - PhT) * K.transpose();
        cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();

        // 修正 e
        auto e = e_red(j) - hT.dot( dx_p);
        dx_p += K * e;
    }
    updateState(dx_p);

    auto t_stage4 = clock();
    t_seq_state_ += t_stage4 - t_stage3;
    n_seq_rows_ += n_eff;

    // 更新 Landmarks
    // R * P^-1 * dxl = Q1^T * e - Q1^T * J_POSE * dxp
    // 计算 (Q1^T * e) - (Q1^T * J_POSE) * dxp -> (Q1^T * e)
    Q1e_s -= Q1Jp_s * dx_p.tail(AugState::SIZE * WIN_SIZE);
    VecX dx_l = VecX::Zero(LMK_SIZE);
    for (size_t i = 0; i < ids.size(); ++i) {
        const auto id = ids[i].first;
        auto lmk = ids[i].second;

        auto &&cov_l = lmk->cov_position;
        dx_l.setZero();

        // 计算 R * P^-1 -> RP
        auto &&RP = RP_s.middleRows(i * LMK_SIZE, LMK_SIZE);
        for (size_t j = 0; j < LMK_SIZE; ++j) {
            auto &&hT = RP.row(j).transpose();

            const auto r = visual_batch_variance;
            VecX PhT = cov_l * hT;
            TYPE var = hT.dot(PhT) + r;
            VecX K = PhT / var;
            cov_l -= K * PhT.transpose();

            PhT = cov_l * hT;
            cov_l.triangularView<Eigen::Upper>() += (K * r - PhT) * K.transpose();
            cov_l.triangularView<Eigen::StrictlyLower>() = cov_l.triangularView<Eigen::StrictlyUpper>().transpose();

            // 修正 e
            auto e = Q1e_s(i * LMK_SIZE + j) - hT.dot(dx_l);
            dx_l += K * e;
        }
        lmk->position += dx_l;
        recordLandmarkRefinement(*lmk);
    }

    // 方案4（Zero-copy）：非关键帧的观测直接从 cam_data 读取来 refine landmark，
    // 不创建 Frame / Feature / Observation，也不写入 lmk_map 的持久关联。
    // 位姿直接用当前状态 state_，即"临时帧"的位姿。
    auto t_refine_1 = clock();
    t_lmk_update_ += t_refine_1 - t_stage4;
    if (!is_keyframe) {
        // 这些量对整帧都是常量，提到循环外
        const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();
        const Mat3_3 Rwi_T = state_.orientation.toRotationMatrix().transpose();
        const Mat3_3 Rwc_T = (state_.orientation * ext_.q_ic).inverse().toRotationMatrix();
        const auto &p_wi = state_.position;
        const auto r = visual_batch_variance;

        for (const auto &meas : cam_data.measurements) {
            const auto lmk_id = meas.first;

            // 只处理已被关键帧观测过的 landmark
            auto lmk_it = map_.lmk_map.find(lmk_id);
            if (lmk_it == map_.lmk_map.end()) {
                continue;
            }
            auto lmk = lmk_it->second;

            // 至少有 1 个关键帧观测，加上本帧观测才能约束
            if (lmk->frm2fet.empty()) {
                continue;
            }

            if (!lmk->is_triangulated) {
                continue;
            }

            // 计算残差和雅可比（观测 meas.second 直接取用，无中间结构）
            const Vec3 d_ij_w = lmk->position - p_wi;
            const Vec3 d_cj_i = Rwi_T * d_ij_w - ext_.t_ic;
            const Vec3 d_cj_c = Ric.transpose() * d_cj_i;
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const auto inv_d2 = inv_d * inv_d;
            const Vec2 est = d_cj_c.head<2>() * inv_d;
            const Vec2 err = meas.second - est;

            Mat2_3 J;
            J << inv_d, TYPE(0), -d_cj_c.x() * inv_d2,
                    TYPE(0), inv_d, -d_cj_c.y() * inv_d2;

            const Mat2_3 J_lmk = J * Rwc_T;

            // 序贯 EKF 更新 landmark（只更新 landmark，不约束 pose）
            auto &&cov_l = lmk->cov_position;
            Vec3 dx_l = Vec3::Zero();

            for (size_t j = 0; j < 2; ++j) {  // 2个残差分量（u, v）
                const Vec3 hT = J_lmk.row(j).transpose();

                Vec3 PhT = cov_l * hT;
                const TYPE var = hT.dot(PhT) + r;
                const Vec3 K = PhT / var;
                cov_l -= K * PhT.transpose();

                PhT = cov_l * hT;
                cov_l.triangularView<Eigen::Upper>() += (K * r - PhT) * K.transpose();
                cov_l.triangularView<Eigen::StrictlyLower>() = cov_l.triangularView<Eigen::StrictlyUpper>().transpose();

                const auto e = err(j) - hT.dot(dx_l);
                dx_l += K * e;
            }
            lmk->position += dx_l;
            recordLandmarkRefinement(*lmk);
        }
    }
    auto t_refine_2 = clock();
    t_refine_cost_ += t_refine_2 - t_refine_1;
    n_lmk_total_ += ids.size();


    auto t2 = clock();
    t_cost_ += t2 - t1;
    ++posterior_times_;

#elif defined(USE_SCHUR)
    auto t1 = clock();

    /*
     * 对每个 Landmark 独立保存 3x3 的 Hll 块。Landmark 之间不存在直接残差，
     * 因而 Hll=diag(Hll_1,...,Hll_M)。对第 j 个点：
     *
     *   Hs <- Hs - Hpl_j Hll_j^dagger Hpl_j^T,
     *   gs <- gs - Hpl_j Hll_j^dagger gl_j。
     *
     * 伪逆必须只保留 Hll_j 的有效特征子空间；同一保留基随后也用于 Landmark
     * 回代和协方差变换，避免 H、g 与 P 使用不同秩判据。
     */

    // [数据采集] 记录视觉更新【前】的先验状态
    UpdateLog log{};
    if (enable_logging_) {
        log.timestamp = cam_data.timestamp;
        log.p_prior = state_.position;
        log.v_prior = state_.velocity;
        log.q_prior = state_.orientation;
        log.n_lmk = ids.size();
        log.win_size = map_.sfw.size();
        log.is_keyframe = is_keyframe;
        log.is_rotation_frame = is_rotation_frame;
        log.rdvio_case = rdvio_case;
        log.rdvio_misalignment_deg = rdvio_misalignment_deg;
    }

    // Hessian 矩阵
    //
    // 优化: Hll 是块对角矩阵(landmark 之间没有直接耦合，只通过 pose 间接耦合)，
    //   原本按 lmk_size x lmk_size 稠密分配 = (3*326)^2 ~ 96万个 double,
    //   而实际只用到对角线上 326 个 3x3 块 = 2934 个。
    //   改成只存对角块: lmk_size x 3，省掉 99.7% 的分配和清零。
    const auto lmk_size = LMK_SIZE * ids.size();
    MatXX Hpp(COV_SIZE, COV_SIZE);
    MatXX Hpl(COV_SIZE, lmk_size);
    Eigen::Matrix<TYPE, Eigen::Dynamic, LMK_SIZE> Hll_diag(lmk_size, LMK_SIZE);
    Hpp.setZero();
    Hpl.setZero();
    Hll_diag.setZero();

    // Gradient
    VecX gp(COV_SIZE);
    VecX gl(lmk_size);
    gp.setZero();
    gl.setZero();
    std::vector<size_t> valid_observations_per_landmark(ids.size(), 0);
    // 局部 Landmark 参数增量到世界 XYZ 增量的雅可比。锚定参数化只改变本次
    // 线性化坐标；持久 Landmark::position 始终保存世界 XYZ，因此回代增量和
    // 协方差都必须通过该雅可比变换回世界坐标。
    std::vector<Mat3_3> landmark_parameter_to_world(
        ids.size(), Mat3_3::Identity());
    std::vector<Frame *> landmark_anchor_frames(ids.size(), nullptr);

    // 优化: 外参相关量对整帧是常量，提到所有循环外(原本每个观测都重算一次)
    const Mat3_3 Ric = ext_.q_ic.toRotationMatrix();
    size_t observations_used = 0;
    size_t observations_downweighted = 0;
    size_t observations_rejected = 0;
    size_t observations_new = 0;
    size_t observations_reused = 0;
    size_t rotation_only_constraints = 0;
    size_t zero_translation_constraints = 0;

    // 两类轨迹最终都把无深度旋转信息累加到同一个 Hpp/gp，区别只在生命周期：
    // - rotation_only_tracks：轨迹本帧结束后删除，若残差有效则算作一次成功消费；
    // - retained_rotation_only_tracks：只标记并消费最新旋转像素，轨迹继续等待平移基线。
    // RD-VIO 原生调度使用完整信息倍率 1；默认 Hybrid MSCKF 使用较保守的
    // depth_free_rotation_information_scale_，避免弱纯旋转模型压过可观的平移/深度信息。
    const auto accumulateDepthFreeRotation = [&]
        (const std::vector<Landmark *> &tracks,
         const bool add_zero_translation,
         const bool removed_after_update) {
        if (tracks.empty() && !add_zero_translation) {
            return;
        }
        const RDVIOConstraintStatistics statistics =
            accumulateRDVIOConstraints(
                map_, tracks, Ric, add_zero_translation,
                enforce_observability_constraint_, visual_batch_variance,
                visual_update_scheduler == VisualUpdateScheduler::RDVIO
                    ? TYPE(1)
                    : depth_free_rotation_information_scale_,
                rdvio_zero_translation_std, visual_hard_reprojection_limit,
                Hpp, gp);
        observations_used += statistics.observations_used;
        observations_new += statistics.new_observations;
        n_new_observations_ += statistics.new_observations;
        if (removed_after_update) {
            // 只有本帧结束后真正删除的轨迹才参与 consumed/dropped 统计。
            // 被延迟的轨迹虽然已有一对旋转观测被消费，但仍留在 Map 中等待基线。
            one_shot_tracks_used += statistics.tracks_used;
        }
        rotation_only_constraints += statistics.rotation_constraints;
        zero_translation_constraints += statistics.zero_translation_constraints;
        n_depth_free_rotation_constraints_ += statistics.rotation_constraints;
        if constexpr (visual_update_scheduler == VisualUpdateScheduler::RDVIO) {
            n_rdvio_rotation_constraints_ += statistics.rotation_constraints;
            n_rdvio_zero_translation_constraints_ +=
                statistics.zero_translation_constraints;
        }
    };
    // 先累加即将删除的轨迹，并可按 RD-VIO 策略附加相邻 R clone 的零平移约束；
    // 再累加保留轨迹。像素级 visual_update_count 由约束构造器维护，后续普通深度
    // Schur 只会读取尚未消费的 Observation，从而保持单个像素量测最多使用一次。
    accumulateDepthFreeRotation(
        rotation_only_tracks, has_rdvio_zero_translation, true);
    accumulateDepthFreeRotation(
        retained_rotation_only_tracks, false, false);
    if (ids.empty() && rotation_only_constraints == 0 &&
        zero_translation_constraints == 0) {
        // 调度阶段可能找到低视差轨迹，但残差门控会拒绝全部候选。此时 Hpp/gp
        // 仍为零，继续做 LDLT 只会产生“max diag=0”的无效更新与日志噪声。
        return;
    }

    // 单个观测无法在消元 landmark 后约束位姿。先暂存每个点的第一条有效观测，
    // 只有第二条到来后才一起写入全局 Hessian；否则 Hpp 会错误地把该点当成固定地图点。
    struct LinearizedObservation {
        Mat2_6 J_pose;
        Mat2_6 J_anchor_pose{Mat2_6::Zero()};
        Mat2_3 J_landmark;
        Vec2 residual;
        TYPE weight{TYPE(1)};
        size_t frame_index{0};
        size_t anchor_frame_index{0};
        bool has_anchor_pose{false};
        Observation *observation{};
    };
    std::vector<LinearizedObservation> first_observation(ids.size());
    auto accumulate_observation = [&](const size_t landmark_index,
                                      const LinearizedObservation &linearized) {
        Hpp.block<6, 6>(linearized.frame_index, linearized.frame_index)
            .triangularView<Eigen::Upper>() +=
                linearized.weight * linearized.J_pose.transpose() * linearized.J_pose;
        if (linearized.has_anchor_pose) {
            Hpp.block<6, 6>(linearized.anchor_frame_index,
                            linearized.anchor_frame_index)
                .triangularView<Eigen::Upper>() +=
                    linearized.weight * linearized.J_anchor_pose.transpose() *
                    linearized.J_anchor_pose;
            if (linearized.frame_index < linearized.anchor_frame_index) {
                Hpp.block<6, 6>(linearized.frame_index,
                                linearized.anchor_frame_index).noalias() +=
                    linearized.weight * linearized.J_pose.transpose() *
                    linearized.J_anchor_pose;
            } else {
                Hpp.block<6, 6>(linearized.anchor_frame_index,
                                linearized.frame_index).noalias() +=
                    linearized.weight * linearized.J_anchor_pose.transpose() *
                    linearized.J_pose;
            }
        }
        Hll_diag.middleRows<3>(landmark_index).triangularView<Eigen::Upper>() +=
            linearized.weight * linearized.J_landmark.transpose() * linearized.J_landmark;
        Hpl.block<6, 3>(linearized.frame_index, landmark_index).noalias() +=
            linearized.weight * linearized.J_pose.transpose() * linearized.J_landmark;
        if (linearized.has_anchor_pose) {
            Hpl.block<6, 3>(linearized.anchor_frame_index, landmark_index).noalias() +=
                linearized.weight * linearized.J_anchor_pose.transpose() *
                linearized.J_landmark;
        }
        gp.segment<6>(linearized.frame_index).noalias() +=
            linearized.weight * linearized.J_pose.transpose() * linearized.residual;
        if (linearized.has_anchor_pose) {
            gp.segment<6>(linearized.anchor_frame_index).noalias() +=
                linearized.weight * linearized.J_anchor_pose.transpose() *
                linearized.residual;
        }
        gl.segment<3>(landmark_index).noalias() +=
            linearized.weight * linearized.J_landmark.transpose() * linearized.residual;
        ++observations_used;
        observations_downweighted += linearized.weight < TYPE(1) ? 1 : 0;
        if (linearized.observation) {
            if (linearized.observation->visual_update_count == 0) {
                ++observations_new;
                ++n_new_observations_;
            } else {
                ++observations_reused;
                ++n_reused_observations_;
            }
            ++linearized.observation->visual_update_count;
        }
    };

    // 遍历 landmarks
    for (size_t i = 0; i < ids.size(); ++i) {
        auto lmk = ids[i].second;
        const size_t lmk_index = LMK_SIZE * i;

        const LandmarkParameterizationLinearization parameterization =
            linearizeLandmarkParameterization(
                *lmk, Ric, ext_.t_ic, enforce_observability_constraint_,
                landmark_parameterization);
        if (!parameterization.valid) {
            continue;
        }
        landmark_parameter_to_world[i] = parameterization.parameter_to_world;
        landmark_anchor_frames[i] = parameterization.anchor;

        // 遍历 landmark 的 所有 observations
        for (auto &it : lmk->frm2fet) {
#ifdef ONE_SHOT
            if (it.first != curr_frame_id) {
                continue;
            }
#endif

            const auto fet = it.second;
            const auto obs = fet->obs[0];
            const auto frm = fet->frame;

            // 无结构 MSCKF 轨迹是一批只能消费一次的量测。若同一像素样本第二次
            // 进入线性化器，必须在累加 H/g 前拒绝，避免生命周期错误悄悄造成
            // 重复计数与协方差过度收缩。
            if constexpr (schedulerConsumesTracksOnce(visual_update_scheduler)) {
                if (obs && obs->visual_update_count != 0) {
                    if (obs->used_by_depth_free_rotation) {
                        // 该像素已按设计进入球面旋转残差。深度后来变得可观时，只能
                        // 使用同轨迹中剩余的新像素，不能把它再次写入重投影 Schur 系统。
                        continue;
                    }
                    ++n_duplicate_observations_blocked_;
                    ++observations_rejected;
                    continue;
                }
            }

            const Mat3_3 Rwi = frm->q().toRotationMatrix();
            const Vec3 d_ij_w = lmk->position - frm->p();
            const Vec3 d_cj_i = Rwi.transpose() * d_ij_w - ext_.t_ic;
            const Vec3 d_cj_c = Ric.transpose() * d_cj_i;
            if (!d_cj_c.allFinite() || d_cj_c.z() <= TYPE(0.05)) {
                ++observations_rejected;
                continue;
            }
            const auto inv_d = TYPE(1) / d_cj_c.z();
            const Vec2 est = d_cj_c.head<2>() * inv_d;
            const Vec2 err = obs->un_pt.head<2>() - est;
            const TYPE residual_norm = err.norm();
            if (!std::isfinite(residual_norm) ||
                residual_norm > visual_hard_reprojection_limit) {
                ++observations_rejected;
                continue;
            }
            const TYPE huber_delta = std::max(
                visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
            const TYPE robust_weight = residual_norm > huber_delta
                ? huber_delta / residual_norm : TYPE(1);

            // 残差始终在当前名义状态计算；启用可观性约束时，位姿雅可比改在每个
            // clone 的首次估计（FEJ）处计算。Landmark 仍在当前点位置重线性化，
            // 避免长轨迹长期使用过时深度；同一轨迹的所有观测共享同一个点位置，
            // 因此联合系统仍保留相同的 gauge 零空间。
            const Mat3_3 Rwi_jac = enforce_observability_constraint_
                ? frm->q_fej().toRotationMatrix()
                : Rwi;
            const Vec3 d_ij_w_jac = enforce_observability_constraint_
                ? lmk->position - frm->p_fej()
                : d_ij_w;
            const Vec3 d_cj_i_jac = Rwi_jac.transpose() * d_ij_w_jac - ext_.t_ic;
            const Vec3 d_cj_c_jac = Ric.transpose() * d_cj_i_jac;
            if (!d_cj_c_jac.allFinite() || d_cj_c_jac.z() <= TYPE(0.05)) {
                ++observations_rejected;
                continue;
            }
            const TYPE inv_d_jac = TYPE(1) / d_cj_c_jac.z();
            const TYPE inv_d2_jac = inv_d_jac * inv_d_jac;

            Mat2_3 J;
            J << inv_d_jac, TYPE(0), -d_cj_c_jac.x() * inv_d2_jac,
                    TYPE(0), inv_d_jac, -d_cj_c_jac.y() * inv_d2_jac;

            // Rwc^T = (Rwi * Ric)^T = Ric^T * Rwi^T，复用已算好的 Rwi/Ric,
            // 避免再做一次四元数乘法 + 求逆 + toRotationMatrix
            Mat2_3 J_lmk_world;
            J_lmk_world.noalias() = J * (Ric.transpose() * Rwi_jac.transpose());
            Mat2_3 J_lmk;
            J_lmk.noalias() = J_lmk_world * landmark_parameter_to_world[i];

            Mat2_6 J_pose;
            J_pose.leftCols<3>().noalias() = J_lmk_world * hat(d_ij_w_jac);
            J_pose.rightCols<3>().noalias() = -J_lmk_world;

            Mat2_6 J_anchor_pose = Mat2_6::Zero();
            size_t anchor_frame_index = 0;
            bool has_anchor_pose = false;
            if constexpr (isAnchoredLandmarkParameterization(
                              landmark_parameterization)) {
                const Frame *anchor = landmark_anchor_frames[i];
                J_anchor_pose = landmarkAnchorPoseJacobian(
                    J_lmk_world, *lmk, *anchor,
                    enforce_observability_constraint_);
                anchor_frame_index = INSState::SIZE +
                    AugState::SIZE * anchor->ordering;
                const size_t observing_frame_index = INSState::SIZE +
                    AugState::SIZE * frm->ordering;
                if (anchor_frame_index == observing_frame_index) {
                    // 当点参数锚定在当前观测相机时，相机和点一起做相同刚体运动
                    // 不改变投影；因此锚点位姿雅可比需并入当前观测位姿雅可比。
                    J_pose += J_anchor_pose;
                } else {
                    has_anchor_pose = true;
                }
            }

            // 外参雅可比: 保留代码但默认不运行(外参目前不在状态里，算了也没人读)。
            // 用 if constexpr 而非 #ifdef，这样它始终参与语法/类型检查，不会腐烂。
            // 开启 ESTIMATE_EXTRINSIC 时还需把 J_ext 累加进 Hpp/Hpl/gp 的外参块。
            if constexpr (ExtState::ESTIMATE_EXTRINSIC) {
                Mat2_6 J_ext;
                J_ext.rightCols<3>().noalias() = -J * Ric.transpose();
                J_ext.leftCols<3>().noalias() =
                    -J_ext.rightCols<3>() * hat(d_cj_i_jac);
            }

            const size_t frm_index = INSState::SIZE + AugState::SIZE * frm->ordering;
            LinearizedObservation current{
                J_pose, J_anchor_pose, J_lmk, err, robust_weight, frm_index,
                anchor_frame_index, has_anchor_pose, obs};
            auto &valid_count = valid_observations_per_landmark[i];
            if (valid_count == 0) {
                first_observation[i] = current;
                valid_count = 1;
                continue;
            }
            if (valid_count == 1) {
                accumulate_observation(lmk_index, first_observation[i]);
            }
            accumulate_observation(lmk_index, current);
            ++valid_count;
        }
    }
    Hpp.triangularView<Eigen::StrictlyLower>() = Hpp.triangularView<Eigen::StrictlyUpper>().transpose();
    // Hll_diag 的每个 3x3 块单独对称化
    for (size_t i = 0; i < ids.size(); ++i) {
        auto &&h = Hll_diag.middleRows<LMK_SIZE>(i * LMK_SIZE);
        h.triangularView<Eigen::StrictlyLower>() = h.triangularView<Eigen::StrictlyUpper>().transpose();
    }

    // 构造 FEJ 零空间基，供泄漏诊断和可选硬投影使用。Schur 消元前，一个 gauge
    // 向量同时含位姿与 Landmark 分量；只投影位姿块会破坏联合正规方程。因此，
    // 任何仅作用于位姿系统的投影都必须等到 Landmark 分量完全消去后再执行。
    constexpr int OC_DIM = 4;
    Eigen::Matrix<TYPE, Eigen::Dynamic, OC_DIM> oc_basis(COV_SIZE, OC_DIM);
    oc_basis.setZero();
    TYPE oc_leak_before = TYPE(0);
    TYPE oc_leak_after = TYPE(0);
    if (map_.sfw.size() > 0) {
        Vec3 gravity_axis = state_.gravity;
        if (!gravity_axis.allFinite() || gravity_axis.norm() < TYPE(1e-8)) {
            gravity_axis = Vec3::UnitZ();
        } else {
            gravity_axis.normalize();
        }
        const Vec3 anchor_position = map_.sfw[0]->p_fej();
        for (size_t frame_number = 0; frame_number < map_.sfw.size(); ++frame_number) {
            const auto frame = map_.sfw[frame_number];
            const size_t offset = INSState::SIZE + AugState::SIZE * frame->ordering;
            oc_basis.block<3, 3>(offset + AugState::P, 0).setIdentity();
            oc_basis.block<3, 1>(offset + AugState::Q, 3) = gravity_axis;
            oc_basis.block<3, 1>(offset + AugState::P, 3) =
                -hat(frame->p_fej() - anchor_position) * gravity_axis;
        }
    }

    auto t_sc1 = clock();
    t_build_H_ += t_sc1 - t1;

    // 计算 schur 补
    //
    // 注: 试过利用 Hpl 的块稀疏性(只在被观测帧的块上运算)，实测反而变慢
    //   (2.60s -> 3.49s)。原因是每个 landmark 平均被 14.8 个关键帧观测到
    //   (WIN_SIZE = 30)，稀疏度只有 2 倍，而 K*K ~ 219 次 6x6 小块乘法的
    //   标量索引开销超过了省下的乘零。稠密 GEMM 的向量化更划算，保持原样。
    MatXX tmp(COV_SIZE, LMK_SIZE);
    for (size_t i = 0; i < ids.size(); ++i) {
        if (valid_observations_per_landmark[i] < 2) {
            continue;
        }
        auto index = i * LMK_SIZE;

        // STEP1：在 Hll 的有效子空间内构造 Moore-Penrose 伪逆。
        // 对称特征分解为 Hll = V diag(lambda) V^T；仅保留
        // lambda_i > tau * lambda_max 的方向，其余方向通常对应弱视差下不可观的
        // 深度分量。于是：
        //   Hll^dagger = V diag(lambda_i^{-1} 或 0) V^T。
        // Hll、gl 必须使用同一组保留方向，否则会出现“信息矩阵已丢弃某方向，
        // 梯度却仍含该方向能量”的不一致 Schur 系统。
        const Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);
        Eigen::SelfAdjointEigenSolver<Mat3_3> hll_es(hll);
        if (hll_es.info() != Eigen::Success ||
            !hll_es.eigenvalues().allFinite()) {
            continue;
        }
        const TYPE hll_max = hll_es.eigenvalues().maxCoeff();
        if (!(hll_max > TYPE(0))) {
            continue;
        }
        const TYPE hll_threshold = hll_rank_relative_threshold_ * hll_max;
        Vec3 hll_inverse = Vec3::Zero();
        const Vec3 hll_gradient_coeff =
            hll_es.eigenvectors().transpose() * gl.segment<LMK_SIZE>(index);
        TYPE discarded_gradient_sq = TYPE(0);
        size_t discarded_directions = 0;
        TYPE hll_min_retained = std::numeric_limits<TYPE>::infinity();
        for (size_t direction = 0; direction < LMK_SIZE; ++direction) {
            if (hll_es.eigenvalues()(direction) > hll_threshold) {
                hll_inverse(direction) = TYPE(1) / hll_es.eigenvalues()(direction);
                hll_min_retained = std::min(
                    hll_min_retained, hll_es.eigenvalues()(direction));
            } else {
                discarded_gradient_sq += hll_gradient_coeff(direction) *
                                         hll_gradient_coeff(direction);
                ++discarded_directions;
            }
        }
        const TYPE discarded_gradient_ratio = std::sqrt(discarded_gradient_sq) /
            std::max(hll_gradient_coeff.norm(), TYPE(1e-15));
        ++n_hll_rank_tests_;
        n_hll_discarded_directions_ += discarded_directions;
        hll_discarded_gradient_ratio_sum_ += discarded_gradient_ratio;
        hll_discarded_gradient_ratio_max_ =
            std::max(hll_discarded_gradient_ratio_max_, discarded_gradient_ratio);
        if (std::isfinite(hll_min_retained) && hll_min_retained > TYPE(0)) {
            const TYPE effective_condition = hll_max / hll_min_retained;
            ++n_hll_condition_tests_;
            hll_effective_condition_sum_ += effective_condition;
            hll_effective_condition_max_ =
                std::max(hll_effective_condition_max_, effective_condition);
        }
        const Mat3_3 hll_inv = hll_es.eigenvectors() * hll_inverse.asDiagonal()
                             * hll_es.eigenvectors().transpose();

        // STEP2：计算 Hpl Hll^dagger。
        tmp.noalias() = Hpl.middleCols<LMK_SIZE>(index) * hll_inv;

        // STEP3：累加 Schur 信息矩阵 Hs = Hpp - Hpl Hll^dagger Hlp。
        Hpp.triangularView<Eigen::Upper>() -= tmp * Hpl.middleCols<LMK_SIZE>(index).transpose();

        // STEP4：同步消去梯度 gs = gp - Hpl Hll^dagger gl。
        gp.noalias() -= tmp * gl.segment<LMK_SIZE>(index);
    }
    Hpp.triangularView<Eigen::StrictlyLower>() = Hpp.triangularView<Eigen::StrictlyUpper>().transpose();

    // 若后面的硬投影成功，这三项就是状态更新唯一允许使用的信息子空间；
    // 后续不得再用另一套阈值重新判秩，否则 H 与 g 又会落到不同子空间。
    MatXX projected_state_basis;
    VecX projected_state_diagonal;
    VecX projected_state_rhs;
    bool has_consistent_projection = false;

    // 在 Schur 消元后的位姿系统上诊断 FEJ 零空间泄漏。令 N 为全局平移与
    // 偏航的 FEJ 基，最小投影
    //   Pi = I - N(N^T N)^{-1}N^T
    // 可强制 Hpp N = 0。该硬投影默认关闭：实验表明，当 Schur 数值零空间已被
    // 截断时继续投影 gp 会损伤精度；具体反例见可观性专题文档。
    if (map_.sfw.size() > 0) {
        const TYPE hpp_norm = std::max(Hpp.norm(), TYPE(1e-15));
        const TYPE basis_norm = std::max(oc_basis.norm(), TYPE(1e-15));
        oc_leak_before = (Hpp * oc_basis).norm() / (hpp_norm * basis_norm);

        if (enforce_observability_constraint_ && project_observability_constraint_) {
            // 先用先验协方差平方根白化，使姿态（rad）和位置（m）进入同一无量纲
            // 度量，再构造零空间正交基，避免单位尺度直接影响投影结果。
            const MatXX navigation_covariance =
                cov_.topLeftCorner(COV_SIZE, COV_SIZE);
            MatXX prior_covariance = TYPE(0.5) *
                (navigation_covariance + navigation_covariance.transpose());
            Eigen::LLT<MatXX> prior_llt(prior_covariance);
            if (prior_llt.info() != Eigen::Success) {
                const TYPE jitter = TYPE(1e-12) * std::max(
                    prior_covariance.diagonal().cwiseAbs().maxCoeff(), TYPE(1));
                prior_covariance.diagonal().array() += jitter;
                prior_llt.compute(prior_covariance);
            }
            if (prior_llt.info() == Eigen::Success) {
                const MatXX prior_sqrt = prior_llt.matrixL();
                const MatXX nullspace_whitened =
                    prior_sqrt.triangularView<Eigen::Lower>().solve(oc_basis);
                Eigen::ColPivHouseholderQR<MatXX> nullspace_qr(nullspace_whitened);
                nullspace_qr.setThreshold(TYPE(1e-10));
                if (nullspace_qr.rank() == OC_DIM) {
                    const MatXX q_null = nullspace_qr.householderQ() *
                        MatXX::Identity(COV_SIZE, OC_DIM);
                    MatXX hpp_whitened =
                        prior_sqrt.transpose() * Hpp * prior_sqrt;
                    VecX gp_whitened = prior_sqrt.transpose() * gp;

                    // 计算 Pi H Pi 与 Pi g，其中 Pi=I-Qn Qn^T；通过低秩乘法完成，
                    // 不显式构造 COV_SIZE x COV_SIZE 的稠密投影矩阵。
                    hpp_whitened.noalias() -=
                        q_null * (q_null.transpose() * hpp_whitened);
                    hpp_whitened.noalias() -=
                        (hpp_whitened * q_null) * q_null.transpose();
                    hpp_whitened = TYPE(0.5) *
                        (hpp_whitened + hpp_whitened.transpose());
                    gp_whitened.noalias() -=
                        q_null * (q_null.transpose() * gp_whitened);

                    Eigen::SelfAdjointEigenSolver<MatXX> hpp_es(hpp_whitened);
                    if (hpp_es.info() == Eigen::Success &&
                        hpp_es.eigenvalues().allFinite()) {
                        const TYPE hpp_max = hpp_es.eigenvalues().maxCoeff();
                        const TYPE hpp_threshold =
                            hpp_rank_relative_threshold_ * hpp_max;
                        const VecX gradient_coeff =
                            hpp_es.eigenvectors().transpose() * gp_whitened;
                        std::vector<Eigen::Index> retained;
                        retained.reserve(COV_SIZE);
                        TYPE discarded_gradient_sq = TYPE(0);
                        for (Eigen::Index direction = 0;
                             direction < static_cast<Eigen::Index>(COV_SIZE);
                             ++direction) {
                            if (hpp_es.eigenvalues()(direction) > hpp_threshold) {
                                retained.push_back(direction);
                            } else {
                                discarded_gradient_sq += gradient_coeff(direction) *
                                                         gradient_coeff(direction);
                            }
                        }

                        const TYPE discarded_gradient_ratio =
                            std::sqrt(discarded_gradient_sq) /
                            std::max(gradient_coeff.norm(), TYPE(1e-15));
                        ++n_hpp_rank_tests_;
                        n_hpp_discarded_directions_ += COV_SIZE - retained.size();
                        hpp_discarded_gradient_ratio_sum_ += discarded_gradient_ratio;
                        hpp_discarded_gradient_ratio_max_ = std::max(
                            hpp_discarded_gradient_ratio_max_, discarded_gradient_ratio);

                        if (hpp_max > TYPE(0) && !retained.empty()) {
                            MatXX retained_vectors(COV_SIZE, retained.size());
                            projected_state_diagonal.resize(retained.size());
                            projected_state_rhs.resize(retained.size());
                            for (size_t column = 0; column < retained.size(); ++column) {
                                const Eigen::Index direction = retained[column];
                                retained_vectors.col(column) =
                                    hpp_es.eigenvectors().col(direction);
                                projected_state_diagonal(column) =
                                    hpp_es.eigenvalues()(direction);
                                projected_state_rhs(column) = gradient_coeff(direction);
                            }
                            // v_i^T*z = (L^-T*v_i)^T*dx.
                            projected_state_basis = prior_sqrt.transpose()
                                .triangularView<Eigen::Upper>()
                                .solve(retained_vectors);
                            has_consistent_projection = true;
                            ++n_oc_projections_;

                            const TYPE whitened_hpp_norm =
                                std::max(hpp_whitened.norm(), TYPE(1e-15));
                            const TYPE whitened_basis_norm =
                                std::max(nullspace_whitened.norm(), TYPE(1e-15));
                            oc_leak_after =
                                (hpp_whitened * nullspace_whitened).norm() /
                                (whitened_hpp_norm * whitened_basis_norm);
                        }
                    }
                }
            }
        }

        if (!has_consistent_projection) {
            const TYPE projected_hpp_norm = std::max(Hpp.norm(), TYPE(1e-15));
            oc_leak_after = (Hpp * oc_basis).norm() /
                            (projected_hpp_norm * basis_norm);
        }
        oc_max_leak_before_ = std::max(oc_max_leak_before_, oc_leak_before);
        oc_max_leak_after_ = std::max(oc_max_leak_after_, oc_leak_after);
    }

    auto t_sc2 = clock();
    t_schur_ += t_sc2 - t_sc1;

//    std::cout << "Update State" << std::endl;
    // [[ 更新 State ]]
    // 对 H 使用特征分解: H = V * λ * V^T
    // y = V * λ * V^T * x + V * sqrt(λ) * V^T * n
    // V^T * y = λ * V^T * x + sqrt(λ) * V^T * n
    // Cov[λ * V^T * n] = sqrt(λ) * V^T * Cov[n] * V * sqrt(λ)
    // 如果 Cov[n] = σ^2 * I,
    // 则 Cov[λ * V^T * n] = σ^2 * λ
    // 所以 V^T * y = λ * V^T * x + sqrt(λ) * n, v ~ N[0, σ]
    // 进一步有 λ^-1 * V^T * y = V^T * x + sqrt(λ)^-1 * n, n ~ N[0, σ]
    // 记 w = sqrt(λ)^-1 * n, n ~ N[0, σ]
    // 则有 Cov[w] = σ^2 * λ^-1
    // 序贯 V.col(i)^T * y / λ(i) = V.col(i)^T * x + w(i), var[w] = σ^2 / λ(i)
    // 联合误差状态排列为 delta x_joint=[delta x_M, delta p_L]，其中 x_M 包含 INS、外参
    // 和全部 clone，p_L 是已有持久点。普通 MSCKF 已消掉本次临时点，所以它的直接
    // 雅可比为 h_full=[h_M, 0]；但 Kalman 增益
    //   K = P h_full^T S^-1 = [P_MM h_M^T; P_LM h_M^T] S^-1
    // 的持久点行通常非零，因此普通轨迹仍会通过交叉协方差 P_LM 一致地修正已有持久点。
    // 这是相关性传播，不是再次使用 updatePersistentLandmarks() 的持久点像素。
    VecX dx_joint = VecX::Zero(cov_.rows());
    TYPE nis_sum = TYPE(0);
    size_t nis_count = 0;
    {
        auto &&cov_p = cov_;
        auto &&ep = gp;

        // 序贯更新需要各标量量测互不相关，即把 Cov[e] = σ²·H 对角化。
        // 特征分解和 LDLT 都能做到，区别只在用哪组基:
        //
        //   特征分解 H = V·λ·V^T:  Cov[V^T·e] = σ²·V^T·H·V = σ²·λ      (对角)
        //   LDLT     H = L·D·L^T:  Cov[L^-1·e] = σ²·L^-1·H·L^-T = σ²·D  (对角)
        //
        // 两者都给出 COV_SIZE 个独立标量量测:
        //   特征分解: h_i = V.col(i),  z_i = (V^T·gp)_i / λ_i,  R_i = σ²/λ_i
        //   LDLT:     h_i = M.col(i),  z_i = (M^-1·gp)_i / D_i, R_i = σ²/D_i
        //     其中 M = P^T·L (Eigen 的 LDLT 带主元置换: A = P^T·L·D·L^T·P)
        //
        // 注意两者并非逐位等价 —— 用的是不同的基，中间量不同，
        // 但都是同一个信息矩阵的合法分解，最终后验应当一致(数值误差内)。
        VecX H_BASIS_diag(COV_SIZE);   // λ 或 D
        MatXX H_BASIS(COV_SIZE, COV_SIZE);  // V 或 M
        VecX rhs(COV_SIZE);            // V^T·gp 或 M^-1·gp

        auto t_e0 = clock();
        if (has_consistent_projection) {
            H_BASIS.setZero();
            H_BASIS_diag.setZero();
            rhs.setZero();
            const Eigen::Index retained_count = projected_state_diagonal.size();
            H_BASIS.leftCols(retained_count) = projected_state_basis;
            H_BASIS_diag.head(retained_count) = projected_state_diagonal;
            rhs.head(retained_count) = projected_state_rhs;
        } else if constexpr (USE_LDLT_FOR_HPP) {
            Eigen::LDLT<MatXX> ldlt(Hpp);

            // M = P^T · L，使得 Hpp = M · D · M^T
            H_BASIS = ldlt.transpositionsP().transpose()
                    * MatXX(ldlt.matrixL());
            H_BASIS_diag = ldlt.vectorD();

            // rhs = M^-1 · gp，用三角回代而不是显式求逆:
            //   M·rhs = gp  =>  P^T·L·rhs = gp  =>  L·rhs = P·gp
            rhs = ldlt.transpositionsP() * ep;
            ldlt.matrixL().solveInPlace(rhs);
        } else {
            Eigen::SelfAdjointEigenSolver<MatXX> es(Hpp);
            H_BASIS = es.eigenvectors();
            H_BASIS_diag = es.eigenvalues();
            rhs.noalias() = H_BASIS.transpose() * ep;
        }
        t_eig_decomp_ += clock() - t_e0;

        // Step-0: 过滤掉(近似)为 0 的对角元。
        //   特征分解: eigenvalues 已升序排列，找到第一个足够大的即可
        //   LDLT:     D 无序，必须逐个判断，所以下面用 skip 而不是起始下标
        const TYPE d_max = H_BASIS_diag.maxCoeff();
        const TYPE d_thresh = has_consistent_projection
            ? TYPE(0)
            : hpp_rank_relative_threshold_ * d_max;
        if (d_max <= TYPE(0)) {
            std::cerr << "Hpp is not positive: max diag = " << d_max << std::endl;
        }

        // Step-1：将每个有效信息方向解释成一个一维伪量测并序贯更新：
        //   z_i = rhs_i / d_i，h_i = H_BASIS.col(i)，R_i = sigma_uv^2 / d_i，
        //   e_i = z_i - h_i^T delta x，S_i = h_i^T P h_i + R_i，
        //   K_i = P h_i / S_i。
        // 协方差采用 Joseph 形式：
        //   P+ = (I-K_i h_i^T)P(I-K_i h_i^T)^T + K_i R_i K_i^T。
        // 代码把它展开成两个上三角低秩更新，数值上比直接 P-KSK^T 更稳。
        //
        // 优化: cov_p 全程保持对称，所以
        //   1) cov_p * hT 用 selfadjointView 做对称矩阵-向量乘 (只读一半)
        //   2) 两次 rank-1/rank-2 更新只写上三角，循环内不再重建下三角
        //      (原本每次迭代都做一次 198x198 的 StrictlyLower = StrictlyUpper^T)
        //   3) 下三角在循环结束后统一恢复一次
        // 数学上完全等价: 中间过程只有 selfadjointView 在读 cov_p，它只看上三角。
        VecX h_full = VecX::Zero(cov_.rows());
        VecX PhT(cov_.rows());
        VecX K(cov_.rows());
        const Eigen::Index basis_direction_count = has_consistent_projection
            ? projected_state_diagonal.size()
            : static_cast<Eigen::Index>(COV_SIZE);
        for (Eigen::Index i = 0; i < basis_direction_count; ++i) {
            const auto d = H_BASIS_diag(i);
            if (d <= d_thresh) {
                ++n_skipped_;           // 诊断: 被判定为零空间的方向数
                if (d < TYPE(0)) {
                    ++n_negative_;      // 诊断: 严格为负(Hpp 不定)的方向数
                }
                continue;   // 零空间方向，不提供信息
            }

            const auto R = visual_batch_variance / d;
            const auto hT = H_BASIS.col(i);
            // Hpp/H_BASIS 只定义在固定维度的主状态 x_M 上；扩展到完整联合状态时，
            // 新增持久点列显式补零。不能截断 cov_，否则会丢失 P_LM 带来的间接修正。
            h_full.setZero();
            h_full.head(COV_SIZE) = hT;

            PhT.noalias() =
                cov_p.selfadjointView<Eigen::Upper>() * h_full;
            const TYPE var = h_full.dot(PhT) + R;
            // 量测 z_i = rhs(i)/d，残差 = z_i - h_i^T·dx。
            // 序贯更新已经把伪量测噪声对角化，因此 e^2/var 可以直接累加为 NIS。
            const TYPE e = rhs(i) / d - h_full.dot(dx_joint);
            if (enable_logging_ && var > TYPE(0) && std::isfinite(var) && std::isfinite(e)) {
                nis_sum += e * e / var;
                ++nis_count;
            }
            K.noalias() = PhT / var;
            cov_p.triangularView<Eigen::Upper>() -= K * PhT.transpose();

            PhT.noalias() =
                cov_p.selfadjointView<Eigen::Upper>() * h_full;
            cov_p.triangularView<Eigen::Upper>() += (K * R - PhT) * K.transpose();

            dx_joint.noalias() += K * e;
        }
        cov_p.triangularView<Eigen::StrictlyLower>() = cov_p.triangularView<Eigen::StrictlyUpper>().transpose();
    }
    const VecX dx_p = dx_joint.head(COV_SIZE);
    // 一次性注入完整联合修正：前 COV_SIZE 维更新 INS/外参/clone，尾部每 3 维更新一个
    // 已有持久点。随后 applyJointStateCorrection() 负责名义状态注入，cov_ 已在上面同步更新。
    applyJointStateCorrection(dx_joint);

    if (enable_hybrid_persistent_landmarks_ &&
        schedulerConsumesTracksOnce(visual_update_scheduler)) {
        // 晋升不是第二次 K=P H^T S^-1 量测更新。普通轨迹的像素已经在上面的 Schur/Joseph
        // 更新中使用一次；这里复用同一线性化得到的 Hll/Hpl/gl，计算
        //   delta p_f = Hll^-1(gl - Hlp delta x_M)
        // 及其与旧状态的条件交叉协方差，然后把新点追加到联合状态。旧 cov_ 左上块保持
        // 不变，只扩展 P_xf/P_ff，因此不会再次收紧导航状态。未晋升轨迹只更新影子统计。
        for (size_t index = 0; index < ids.size(); ++index) {
            Landmark &landmark = *ids[index].second;
            bool promoted = false;
            if (promotion_candidates[index] &&
                valid_observations_per_landmark[index] >= 2) {
                const size_t landmark_offset = index * LMK_SIZE;
                const Mat3_3 hll =
                    Hll_diag.middleRows<LMK_SIZE>(landmark_offset);
                const Eigen::Matrix<TYPE, Eigen::Dynamic, 3> hpl =
                    Hpl.middleCols<LMK_SIZE>(landmark_offset);
                promoted = promotePersistentLandmark(
                    landmark, track_qualities[index], hll, hpl,
                    gl.segment<LMK_SIZE>(landmark_offset),
                    landmark_parameter_to_world[index], dx_p,
                    visual_batch_variance, cam_data.timestamp);
            }
            if (!promoted) {
                updateShadowCandidate(
                    landmark, track_qualities[index], cam_data.timestamp);
            }
        }
    }
//    std::cout << "Update State Finished" << std::endl;

    // [数据采集] 记录视觉更新【后】的后验状态与修正量
    if (enable_logging_) {
        using I = INSState;
        log.p_post = state_.position;
        log.v_post = state_.velocity;
        log.q_post = state_.orientation;
        log.bg_post = state_.gyro_bias;
        log.ba_post = state_.accel_bias;
        log.g_post = state_.gravity;
        log.dx_q_norm = dx_p.segment<3>(I::Q).norm();
        log.dx_p_norm = dx_p.segment<3>(I::P).norm();
        log.dx_v_norm = dx_p.segment<3>(I::V).norm();
        log.cov_q_trace = cov_.diagonal().segment<3>(I::Q).sum();
        log.cov_p_trace = cov_.diagonal().segment<3>(I::P).sum();
        log.cov_v_trace = cov_.diagonal().segment<3>(I::V).sum();
        log.nis_mean = nis_count ? nis_sum / static_cast<TYPE>(nis_count) : TYPE(0);
        log.nis_dof = nis_count;
        log.n_obs_used = observations_used;
        log.n_obs_downweighted = observations_downweighted;
        log.n_obs_rejected = observations_rejected;
        log.n_obs_new = observations_new;
        log.n_obs_reused = observations_reused;
        log.n_tracks_consumed = schedulerConsumesTracksOnce()
            ? tracks_to_consume.size() : 0;
        log.oc_leak_before = oc_leak_before;
        log.oc_leak_after = oc_leak_after;
        log.rotation_only_constraints = rotation_only_constraints;
        logs_.emplace_back(log);
    }

    auto t_sc3 = clock();
    t_eig_state_ += t_sc3 - t_sc2;

//    std::cout << "Update Landmark" << std::endl;
    // [[ 更新 Landmark ]]
    //
    // 旧 IndependentEkf 路径只保留为消融实验：它不保存导航状态与点之间的
    // 交叉协方差 P_xl，却会把同一滑窗观测反复当成新的独立量测，因而容易过度
    // 自信。Fixed、Retriangulate 与 SchurBackSubstitution 不会制造这种假独立性。
    const LandmarkUpdateMode landmark_mode = schedulerConsumesTracksOnce()
        ? LandmarkUpdateMode::Fixed
        : (refine_landmarks_ ? landmark_update_mode_ : LandmarkUpdateMode::Fixed);
    const bool independent_landmark_mode =
        landmark_mode == LandmarkUpdateMode::IndependentEkf ||
        landmark_mode == LandmarkUpdateMode::IndependentEkfInflated ||
        landmark_mode == LandmarkUpdateMode::IndependentEkfAdaptive;
    if (independent_landmark_mode ||
        (landmark_mode == LandmarkUpdateMode::SchurBackSubstitution && is_keyframe)) {
        gl.noalias() -= Hpl.transpose() * dx_p;
    }

    const Mat3_3 Ric_landmark = ext_.q_ic.toRotationMatrix();
    auto landmarkReprojectionCost = [&](const Landmark &landmark,
                                        const Vec3 &position) -> TYPE {
        const TYPE huber_delta = std::max(
            visual_huber_delta_sigma * triangulation_uv_std, TYPE(1e-8));
        TYPE cost = TYPE(0);
        size_t count = 0;
        for (const auto &[frame_id, feature] : landmark.frm2fet) {
            (void)frame_id;
            if (!feature || !feature->frame || !feature->obs[0]) {
                continue;
            }
            const auto *frame = feature->frame;
            const Mat3_3 Rwi = frame->q().toRotationMatrix();
            const Vec3 d_camera = Ric_landmark.transpose() *
                (Rwi.transpose() * (position - frame->p()) - ext_.t_ic);
            if (!d_camera.allFinite() || d_camera.z() <= TYPE(0.05)) {
                return std::numeric_limits<TYPE>::infinity();
            }
            const Vec2 estimate = d_camera.head<2>() / d_camera.z();
            const TYPE residual_norm =
                (feature->obs[0]->un_pt.head<2>() - estimate).norm();
            if (!std::isfinite(residual_norm)) {
                return std::numeric_limits<TYPE>::infinity();
            }
            cost += residual_norm <= huber_delta
                ? TYPE(0.5) * residual_norm * residual_norm
                : huber_delta * (residual_norm - TYPE(0.5) * huber_delta);
            ++count;
        }
        return count >= 2 ? cost / static_cast<TYPE>(count)
                          : std::numeric_limits<TYPE>::infinity();
    };

    // Schur 路径的量测只来自持久关键帧。若本次没有为该点加入新的关键帧观测，
    // 再次修正只是在重复求解几乎相同的批次，会放大相关量测的重复使用并浪费计算。
    auto observedInCurrentKeyframe = [&](const Landmark &landmark) {
        if (!is_keyframe) {
            return false;
        }
        for (const auto &[frame_id, feature] : landmark.frm2fet) {
            (void)frame_id;
            if (feature && feature->frame &&
                feature->frame->timestamp == cam_data.timestamp) {
                return true;
            }
        }
        return false;
    };

    if (landmark_mode == LandmarkUpdateMode::Retriangulate && is_keyframe) {
        for (const auto &[id, lmk] : ids) {
            (void)id;
            if (!observedInCurrentKeyframe(*lmk)) {
                continue;
            }
            ++n_lmk_update_attempts_;
            const TYPE cost_before = landmarkReprojectionCost(*lmk, lmk->position);
            const TriangulationResult triangulation = triangulateLandmark(*lmk);
            if (triangulation.status != TriangulationStatus::Success) {
                continue;
            }
            const TYPE cost_after =
                landmarkReprojectionCost(*lmk, triangulation.position);
            if (!std::isfinite(cost_after) || cost_after > cost_before + TYPE(1e-15)) {
                continue;
            }
            lmk->position = triangulation.position;
            lmk->cov_position = triangulation.covariance;
            ++n_lmk_update_accepted_;
            ++n_lmk_retriangulation_success_;
            lmk_reprojection_cost_reduction_ += cost_before - cost_after;
            recordLandmarkRefinement(*lmk);
        }
    } else if (landmark_mode == LandmarkUpdateMode::SchurBackSubstitution && is_keyframe) {
        for (size_t i = 0; i < ids.size(); ++i) {
            if (valid_observations_per_landmark[i] < 2) {
                continue;
            }
            auto lmk = ids[i].second;
            if (!observedInCurrentKeyframe(*lmk)) {
                continue;
            }
            const size_t index = i * LMK_SIZE;
            const Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);
            const Vec3 el = gl.segment<LMK_SIZE>(index);
            ++n_lmk_update_attempts_;

            Eigen::SelfAdjointEigenSolver<Mat3_3> es(hll);
            if (es.info() != Eigen::Success || !es.eigenvalues().allFinite()) {
                continue;
            }
            const TYPE hll_max = es.eigenvalues().maxCoeff();
            if (!(hll_max > TYPE(0))) {
                continue;
            }
            const TYPE threshold = TYPE(1e-6) * hll_max;
            const Vec3 inverse = (es.eigenvalues().array() > threshold)
                .select(es.eigenvalues().array().inverse(), TYPE(0));
            const Vec3 parameter_increment = es.eigenvectors() * inverse.asDiagonal()
                                           * es.eigenvectors().transpose() * el;
            const Vec3 increment = landmark_parameter_to_world[i] * parameter_increment;
            if (!increment.allFinite() || increment.norm() > TYPE(100)) {
                continue;
            }

            const TYPE cost_before = landmarkReprojectionCost(*lmk, lmk->position);
            TYPE step = TYPE(1);
            bool accepted = false;
            for (size_t line_search = 0; line_search < 5; ++line_search) {
                const Vec3 candidate = lmk->position + step * increment;
                const TYPE cost_after = landmarkReprojectionCost(*lmk, candidate);
                if (std::isfinite(cost_after) && cost_after <= cost_before + TYPE(1e-15)) {
                    lmk->position = candidate;
                    ++n_lmk_update_accepted_;
                    lmk_reprojection_cost_reduction_ += cost_before - cost_after;
                    recordLandmarkRefinement(*lmk);
                    accepted = true;
                    break;
                }
                step *= TYPE(0.5);
            }
            (void)accepted;
        }
    } else if (independent_landmark_mode) {
        for (size_t i = 0; i < ids.size(); ++i) {
            if (valid_observations_per_landmark[i] < 2) {
                continue;
            }
            auto id = ids[i].first;
            auto lmk = ids[i].second;
            const size_t index = i * LMK_SIZE;
            Vec3 dx_l = Vec3::Zero();
            auto &&cov_p = lmk->cov_position;
            Mat3_3 hll = Hll_diag.middleRows<LMK_SIZE>(index);
            Vec3 el = gl.segment<LMK_SIZE>(index);
            ++n_lmk_update_attempts_;

            // 持久点协方差以世界系 XYZ 保存；若本次线性化使用锚定参数，必须先把
            // 局部参数的正规方程变换回世界坐标基，再供独立地图实验使用。
            if constexpr (isAnchoredLandmarkParameterization(
                              landmark_parameterization)) {
                if (!transformLandmarkNormalToWorld(
                        landmark_parameter_to_world[i], hll, el)) {
                    continue;
                }
            }

            if (landmark_mode != LandmarkUpdateMode::IndependentEkf) {
                TYPE inflation_scale = TYPE(1);
                if (landmark_mode == LandmarkUpdateMode::IndependentEkfAdaptive) {
                    const TYPE excess_nis = std::max(
                        TYPE(0), lmk->independent_nis_ema - TYPE(1));
                    inflation_scale = std::clamp(
                        landmark_adaptive_inflation_gain_ * excess_nis,
                        TYPE(0), landmark_adaptive_inflation_max_scale_);
                }
                cov_p.diagonal().array() +=
                    landmark_process_noise_density_ * std::max(TYPE(dt), TYPE(0)) *
                    inflation_scale;
            }

            Vec3 hll_diag;
            Mat3_3 hll_basis;
            Vec3 hll_rhs;
            if constexpr (USE_LDLT_FOR_HLL) {
                Eigen::LDLT<Mat3_3> ldlt(hll);
                hll_basis = ldlt.transpositionsP().transpose() * Mat3_3(ldlt.matrixL());
                hll_diag = ldlt.vectorD();
                hll_rhs = ldlt.transpositionsP() * el;
                ldlt.matrixL().solveInPlace(hll_rhs);
            } else {
                Eigen::SelfAdjointEigenSolver<Mat3_3> es(hll);
                hll_basis = es.eigenvectors();
                hll_diag = es.eigenvalues();
                hll_rhs.noalias() = hll_basis.transpose() * el;
            }

            const TYPE hll_max = hll_diag.maxCoeff();
            const TYPE hll_thresh = hll_rank_relative_threshold_ * hll_max;
            if (hll_max <= TYPE(0)) {
                std::cerr << "Hll not positive: id = " << id
                          << ", diag = " << hll_diag.transpose() << std::endl;
                continue;
            }
            TYPE landmark_nis_sum = TYPE(0);
            size_t landmark_nis_count = 0;
            for (size_t j = 0; j < LMK_SIZE; ++j) {
                const TYPE d = hll_diag(j);
                if (d <= hll_thresh) {
                    continue;
                }
                const TYPE R = visual_batch_variance / d;
                const Vec3 hT = hll_basis.col(j);
                Vec3 PhT = cov_p * hT;
                const TYPE var = hT.dot(PhT) + R;
                const TYPE e = hll_rhs(j) / d - hT.dot(dx_l);
                if (var > TYPE(0) && std::isfinite(var) && std::isfinite(e)) {
                    landmark_nis_sum += e * e / var;
                    ++landmark_nis_count;
                }
                const Vec3 K = PhT / var;
                cov_p -= K * PhT.transpose();
                PhT = cov_p * hT;
                cov_p.triangularView<Eigen::Upper>() +=
                    (K * R - PhT) * K.transpose();
                cov_p.triangularView<Eigen::StrictlyLower>() =
                    cov_p.triangularView<Eigen::StrictlyUpper>().transpose();
                dx_l += K * e;
            }
            if (landmark_nis_count > 0) {
                const TYPE batch_nis = landmark_nis_sum /
                    static_cast<TYPE>(landmark_nis_count);
                const TYPE alpha = std::clamp(
                    landmark_nis_ema_alpha_, TYPE(0), TYPE(1));
                lmk->independent_nis_ema =
                    (TYPE(1) - alpha) * lmk->independent_nis_ema +
                    alpha * batch_nis;
            }
            lmk->position += dx_l;
            ++n_lmk_update_accepted_;
            recordLandmarkRefinement(*lmk);
        }
    }

    updateShadowLandmarks(cam_data, ids, is_keyframe, dt);

//    std::cout << "Update Landmark Finished" << std::endl;

    auto t_sc4 = clock();
    t_eig_lmk_ += t_sc4 - t_sc3;
    n_lmk_total_ += ids.size();

    auto t2 = clock();
    t_cost_ += t2 - t1;
    ++posterior_times_;

#endif

    // 轨迹消费与调度器特有的 clone 删除统一由 schedule_finalizer 处理，
    // 因而正常结束和所有提前返回路径都具有完全相同的生命周期语义。
}
// 视觉后验实现到此结束；文件职责与调用关系见 docs/SOURCE_LAYOUT.md。
