//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_MAP_H
#define VINSEKF_MAP_H

#include "../type.h"
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

        template<typename IMG_INFO>
        bool pushImageInfo(const IMG_INFO &image_info) {
            // 特征点过少, 直接丢弃
            if (image_info.measurements.size() < 10) {
                std::cout << "Features are not enough" << std::endl;
                return false;
            }

            // TODO: 判断是否为 Key Frame
            if (!sfw.empty() && sfw.getLatestFrame()->timestamp + 500000 > image_info.timestamp) {
                std::cout << "Not Key Frame" << std::endl;
                return false;
            }
            std::cout << "Find Key Frame" << std::endl;

            // Create Frame
            auto frame = pool_frm.allocate();
            sfw.pushFrame(frame);
            frame->timestamp = image_info.timestamp;
            frame->id = image_info.timestamp;
            frame->ordering = sfw.getLatestIndex();

            std::cout << "frame->ordering = " << frame->ordering << std::endl;

            for (const auto &meas : image_info.measurements) {
                // Create Feature
                auto fet = pool_fet.allocate();
                fet->un_pt = Vec3(meas.second.x(), meas.second.y(), 1);

                // Add { Camera ID, Frame } to Feature
                fet->camera_id = 0;
                fet->frame = frame;

                // Create Feature Message
                const auto lmk_id = meas.first;
                auto msg = pool_msg.allocate();
                (*msg)[0] = fet;

                // Add { Landmark ID, Feature Message } to Frame
                frame->lmk2msg.emplace(lmk_id, msg);

                // Add New Landmark to Map
                Landmark *lmk;
                if (auto it = lmk_map.find(lmk_id); it == lmk_map.end()) {
                    lmk = pool_lmk.allocate();
                    lmk_map.emplace(lmk_id, lmk);

                    // Select Anchor Frame
                    lmk->anchor_fet = fet;
                } else {
                    lmk = it->second;
                }

                // Add { Frame ID, Feature Message } to Landmark
                lmk->frm2msg.emplace(frame->id, msg);

                // Add { Landmark ID, Landmark } to Feature
                fet->landmark_id = lmk_id;
                fet->landmark = lmk;
            }

            return true;
        }

        void popFrame() {
            static size_t count = 0;
            ++count;

            Frame *frm = sfw.popFrame();
            auto frm_id = frm->id;
            for (auto &it : frm->lmk2msg) {
                LandmarkID lmk_id = it.first;
                Landmark *lmk = lmk_map.at(lmk_id);

                // 把 Frame 从 Landmark 中删去
                if (!lmk->delete_frame(frm_id)) {
                    std::cerr << "Frame " << frm_id << " is not in Landmark " << lmk_id << std::endl;
                }

                // 如果 Landmark 不再与任何 Key Frame 关联，则删除 Landmark
                if (lmk->frm2msg.empty()) {
                    std::cout << "Deleted Landmark ID = " << lmk_id << std::endl;
                    lmk_map.erase(lmk_id);
                    pool_lmk.deallocate(lmk, [](Landmark &landmark) {
                        landmark.reset();
                    });
                }

                // 删除 Message
                FeatureMsg *msg = it.second;
                for (auto &fet : *msg) {
                    // 删除 Feature
                    pool_fet.deallocate(fet, [](Feature &feature) {
                        feature.reset();
                    });

                }
                pool_msg.deallocate(msg);
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

        constexpr static size_t N_WIN = 7;
        constexpr static size_t N_MSG = 100000;
        constexpr static size_t N_FET = 1000000;
        constexpr static size_t N_FRM = 1000;
        constexpr static size_t N_LMK = 100000;

        Pool<FeatureMsg> pool_msg{N_MSG};
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
