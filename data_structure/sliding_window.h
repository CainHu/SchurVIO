//
// Created by Cain on 2025/9/10.
//

#pragma once

#include "../type.h"
#include "feature.h"
#include "frame.h"
#include "landmark.h"

namespace slam {
    struct SlidingWindow {
        explicit SlidingWindow(size_t window_size) {
            win_size = window_size;
            frm_win.resize(win_size);
            free_idx.resize(win_size);
            for (size_t i = 0; i < win_size; ++i) {
                free_idx.emplace_back(i);
            }
        }

        bool pushFrame(Frame *frame) {
            if (free_idx.empty()) {
                throw std::invalid_argument("no free space in sfw");
            }

            // TODO: 完善选择 KeyFrame 的策略
            if (free_idx.size() < win_size) {
                if (frm_win[latest_idx]->timestamp + 500000 > frame->timestamp) {
                    std::cout << "Not Key Frame" << std::endl;
                    return false;
                }
            }
            std::cout << "Key Frame" << std::endl;

            const auto idx = free_idx.back();
            latest_idx = idx;
            frm_win[idx] = frame;
            free_idx.pop_back();

            return true;
        }

        KeyFrame *popFrame() {
            // TODO: 加入选择策略
            const auto idx = (latest_idx + 1) % win_size;
            free_idx.emplace_back(idx);
            return frm_win[idx];
        }

        [[nodiscard]] bool isFull() const { return free_idx.empty(); }
        [[nodiscard]] Frame *getLatestFrame() const { return frm_win[latest_idx]; }

        size_t latest_idx{};
        size_t win_size;
        std::vector<KeyFrame *> frm_win;
        std::vector<size_t> free_idx;
    };
}
