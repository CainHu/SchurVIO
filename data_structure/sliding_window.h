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
                free_idx[i] = win_size - i - 1;
            }
        }

        bool pushFrame(Frame *frame) {
            if (free_idx.empty()) {
                throw std::invalid_argument("no free space in sfw");
            }

            const auto idx = free_idx.back();
            latest_idx = idx;
            frm_win[idx] = frame;
            free_idx.pop_back();
            active_idx.push_back(idx);

            return true;
        }

        // Remove a frame by chronological index (0 = oldest).  The previous
        // ring-only implementation could remove correctly only when all slots
        // were occupied; scheduling policies with compact windows need removal
        // to remain correct while most physical covariance slots are unused.
        KeyFrame *popFrame(size_t chronological_index = 0) {
            if (chronological_index >= active_idx.size()) {
                throw std::out_of_range("sliding-window frame index out of range");
            }
            const auto active_it = active_idx.begin() +
                                   static_cast<std::ptrdiff_t>(chronological_index);
            const auto idx = *active_it;
            active_idx.erase(active_it);
            KeyFrame *frame = frm_win[idx];
            frm_win[idx] = nullptr;
            free_idx.emplace_back(idx);
            latest_idx = active_idx.empty() ? 0 : active_idx.back();
            return frame;
        }

        [[nodiscard]] bool isFull() const { return free_idx.empty(); }
        [[nodiscard]] size_t getWindowSize() const { return win_size; }
        [[nodiscard]] size_t getLatestIndex() const { return latest_idx; }
        [[nodiscard]] Frame *getLatestFrame() const { return frm_win[latest_idx]; }
        [[nodiscard]] size_t size() const { return frm_win.size() - free_idx.size(); }
        [[nodiscard]] bool empty() const { return free_idx.size() == frm_win.size(); }

        [[nodiscard]] size_t physicalIndex(size_t chronological_index) const {
            return active_idx.at(chronological_index);
        }

        KeyFrame * operator[](size_t i) { return frm_win.at(active_idx.at(i)); }
        const KeyFrame * operator[](size_t i) const { return frm_win.at(active_idx.at(i)); }

        size_t latest_idx{};
        size_t win_size;
        std::vector<KeyFrame *> frm_win;
        std::vector<size_t> free_idx;
        std::deque<size_t> active_idx;
    };
}
