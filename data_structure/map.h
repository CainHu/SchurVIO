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

            // 判断是否为 Key Frame
            if (!sfw.empty() && sfw.getLatestFrame()->timestamp + 200000 > image_info.timestamp) {
                return false;
            }

            return true;
        }

        // 将帧加入滑窗（仅关键帧）
        Frame* pushKeyFrame(Tus timestamp) {
            auto frame = pool_frm.allocate();
            sfw.pushFrame(frame);
            frame->timestamp = timestamp;
            frame->id = timestamp;
            frame->ordering = sfw.getLatestIndex();
            frame->is_key_frame = true;

            std::cout << "frame->ordering = " << frame->ordering << std::endl;

            return frame;
        }

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

        // 删除临时帧（非关键帧用完后清理）
        void removeTempFrame(Frame* frame) {
            if (frame->is_key_frame) {
                std::cerr << "Error: trying to remove a key frame as temp frame!" << std::endl;
                return;
            }

            auto frm_id = frame->id;
            for (auto &it : frame->lmk2fet) {
                LandmarkID lmk_id = it.first;

                if (auto lmk_it = lmk_map.find(lmk_id); lmk_it != lmk_map.end()) {
                    Landmark *lmk = lmk_it->second;

                    // 把 Frame 从 Landmark 中删去
                    lmk->delete_frame(frm_id);

                    // 如果 Landmark 不再与任何 Frame 关联，则删除 Landmark
                    if (lmk->frm2fet.empty()) {
                        lmk_map.erase(lmk_id);
                        pool_lmk.deallocate(lmk, [](Landmark &landmark) {
                            landmark.reset();
                        });
                    }
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
            pool_frm.deallocate(frame, [](Frame &frame) {
                frame.reset();
            });
        }

        void popFrame() {
            static size_t count = 0;
            ++count;

            Frame *frm = sfw.popFrame();
            auto frm_id = frm->id;
            for (auto &it : frm->lmk2fet) {
                LandmarkID lmk_id = it.first;
                Landmark *lmk = lmk_map.at(lmk_id);

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
