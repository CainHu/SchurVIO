//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_MAP_H
#define VINSEKF_MAP_H

#include "../type.h"
#include "observation.h"
#include "feature.h"
#include "frame.h"
#include "landmark.h"
#include "pool.h"
#include "sliding_window.h"

namespace slam {
    using FrameDeque = std::deque<Frame *>;
    using FrameList = std::list<Frame *>;
    using FrameVector = std::vector<Frame *>;
    using LandmarkMap = std::unordered_map<LandmarkID, Landmark *>;

    struct Map {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Map() = default;
        ~Map() = default;

        Landmark *addLandmark(LandmarkID id);

        // 判断是否为关键帧
        template<typename IMG_INFO>
        bool isKeyFrame(const IMG_INFO &image_info) {
            // 特征点过少, 直接丢弃
            if (image_info.measurements.size() < 10) {
                return false;
            }

            // Every-image schedulers also keep temporal frames in sfw. Compare
            // against the latest actual keyframe rather than simply the latest
            // clone, otherwise no later image could ever pass the time gate.
            const Frame *latest_keyframe = nullptr;
            for (size_t i = sfw.size(); i > 0; --i) {
                const Frame *candidate = sfw[i - 1];
                if (candidate && candidate->is_key_frame) {
                    latest_keyframe = candidate;
                    break;
                }
            }
            if (latest_keyframe &&
                latest_keyframe->timestamp + 200000 > image_info.timestamp) {
                return false;
            }

            return true;
        }

        // Add either a keyframe or temporal frame to the clone window. The
        // physical slot index is also the covariance-block ordering.
        Frame* pushFrame(Tus timestamp, bool is_keyframe) {
            auto frame = pool_frm.allocate();
            sfw.pushFrame(frame);
            frame->timestamp = timestamp;
            frame->id = timestamp;
            frame->ordering = sfw.getLatestIndex();
            frame->is_key_frame = is_keyframe;

//            std::cout << "frame->ordering = " << frame->ordering << std::endl;

            return frame;
        }

        // Historical compatibility wrapper.
        Frame* pushKeyFrame(Tus timestamp) { return pushFrame(timestamp, true); }

        // 创建临时帧（非关键帧），不加入滑窗，仅用于观测关联
        Frame* createTempFrame(Tus timestamp) {
            auto frame = pool_frm.allocate();
            frame->timestamp = timestamp;
            frame->id = timestamp;
            frame->ordering = 0; // 非关键帧没有ordering
            frame->is_key_frame = false;

            return frame;
        }

        // 添加观测到map（关键帧和非关键帧都调用）
        template<typename IMG_INFO>
        void addObservations(Frame* frame, const IMG_INFO &image_info) {
            for (const auto &meas : image_info.measurements) {
                // Create Feature
                auto fet = pool_fet.allocate();

                // Add Frame to Feature
                fet->frame = frame;

                // Create Observation and Add Feature to Observation
                auto obs = pool_obs.allocate();
                obs->un_pt = Vec3(meas.second.x(), meas.second.y(), 1);
                obs->camera_id = 0;
                obs->fet = fet;

                // Add Observation to Feature
                fet->obs[0] = obs;

                // Add { Landmark ID, Feature } to Frame
                const auto lmk_id = meas.first;
                frame->lmk2fet.emplace(lmk_id, fet);

                // Add New Landmark to Map
                Landmark *lmk;
                if (auto it = lmk_map.find(lmk_id); it == lmk_map.end()) {
                    lmk = pool_lmk.allocate();
                    lmk->id = lmk_id;
                    lmk->map = this;
                    lmk_map.emplace(lmk_id, lmk);

                    // Select Anchor Frame
                    lmk->anchor_obs = obs;
                } else {
                    lmk = it->second;
                }

                // Add { Frame ID, Feature } to Landmark
                lmk->frm2fet.emplace(frame->id, fet);

                // Add Landmark to Feature
                fet->landmark = lmk;
            }
        }

        void popFrame(size_t chronological_index = 0) {
            static size_t count = 0;
            ++count;

            Frame *frm = sfw.popFrame(chronological_index);
            if (!frm) {
                throw std::runtime_error("null frame removed from sliding window");
            }
            auto frm_id = frm->id;
            for (auto &it : frm->lmk2fet) {
                LandmarkID lmk_id = it.first;
                const auto landmark_it = lmk_map.find(lmk_id);
                if (landmark_it == lmk_map.end()) {
                    continue;
                }
                Landmark *lmk = landmark_it->second;

                // 把 Frame 从 Landmark 中删去
                if (!lmk->delete_frame(frm_id)) {
                    std::cerr << "Frame " << frm_id << " is not in Landmark " << lmk_id << std::endl;
                }

                // 如果 Landmark 不再与任何 Key Frame 关联，则删除 Landmark
                if (lmk->frm2fet.empty()) {
//                    std::cout << "Deleted Landmark ID = " << lmk_id << std::endl;
                    lmk_map.erase(lmk_id);
                    pool_lmk.deallocate(lmk, [](Landmark &landmark) {
                        landmark.reset();
                    });
                }

                // 删除 Feature
                Feature *fet = it.second;
                for (auto &obs : fet->obs) {
                    if (!obs) {
                        continue;
                    }
                    // 删除 Observation
                    pool_obs.deallocate(obs, [](Observation &observation) {
                        observation.reset();
                    });

                }
                pool_fet.deallocate(fet);
            }

            // 删除 Frame
            pool_frm.deallocate(frm, [](Frame &frame) {
                frame.reset();
            });
        }

        // Consume a complete structureless track. All frame-side references
        // are erased before pooled Feature/Observation objects are released so
        // a later measurement with the same external id starts a fresh track.
        bool removeLandmark(LandmarkID landmark_id) {
            const auto landmark_it = lmk_map.find(landmark_id);
            if (landmark_it == lmk_map.end()) {
                return false;
            }
            Landmark *landmark = landmark_it->second;
            for (const auto &[frame_id, feature] : landmark->frm2fet) {
                (void)frame_id;
                if (!feature) {
                    continue;
                }
                if (feature->frame) {
                    feature->frame->lmk2fet.erase(landmark_id);
                }
                for (auto *observation : feature->obs) {
                    if (observation) {
                        pool_obs.deallocate(observation, [](Observation &value) {
                            value.reset();
                        });
                    }
                }
                pool_fet.deallocate(feature, [](Feature &value) {
                    value.reset();
                });
            }
            landmark->frm2fet.clear();
            lmk_map.erase(landmark_it);
            pool_lmk.deallocate(landmark, [](Landmark &value) {
                value.reset();
            });
            return true;
        }

        [[nodiscard]] bool isWinFull() const { return sfw.isFull(); }
        [[nodiscard]] size_t getWinSize() const { return sfw.getWindowSize(); }
        [[nodiscard]] size_t getWinLatestIndex() const { return sfw.getLatestIndex(); }
        [[nodiscard]] Frame *getWinLatestFrame() const { return sfw.getLatestFrame(); }

        constexpr static size_t N_WIN = 30;
        constexpr static size_t N_LMK = 100000;
        constexpr static size_t N_FRM = 10000;
        constexpr static size_t N_FET = 1000 * N_FRM;
        constexpr static size_t N_OBS = N_FET * N_CAMERA;

        Pool<Observation> pool_obs{N_OBS};
        Pool<Feature> pool_fet{N_FET};
        Pool<Frame> pool_frm{N_FRM};
        Pool<Landmark> pool_lmk{N_LMK};

        std::array<Quat, N_CAMERA> q_ic;
        std::array<Vec3, N_CAMERA> t_ic;

        FrameDeque frm_deq;
        FrameList frm_lst;
        FrameVector frm_vec;

        SlidingWindow sfw {N_WIN};
        LandmarkMap lmk_map;
    };
}

#endif //VINSEKF_MAP_H
