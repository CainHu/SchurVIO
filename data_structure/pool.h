//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_POOL_H
#define VINSEKF_POOL_H

#include "../type.h"
#include "feature.h"
#include "frame.h"
#include "landmark.h"

namespace slam {
    template<typename T>
    class Pool {
    public:
        Pool(const Pool&) = delete;
        Pool& operator=(const Pool&) = delete;
        Pool(Pool&&) = delete;
        Pool& operator=(Pool&&) = delete;

        explicit Pool(size_t initial_size=0) {
            if (initial_size) {
                data_.resize(initial_size);
                std::iota (free_idx_.begin (), free_idx_.end (), 0);
            }
        }

        T* allocate() {
            if (free_idx_.empty()) {
                std::cout << "Memory over." << std::endl;
                data_.emplace_back(T());
                return &data_.back();
            } else {
                const size_t idx = free_idx_.back();
                free_idx_.pop_back();
                return &data_[idx];
            }
        }

        void deallocate(T* ptr) {
            if (ptr == nullptr) {
                return;
            }

            size_t idx = ptr - data_.data();

            if (idx >= data_.size()) {
                throw std::invalid_argument("释放的指针不属于当前内存池");
            }

            if (std::find(free_idx_.begin(), free_idx_.end(), idx) != free_idx_.end()) {
                throw std::invalid_argument("指针已被释放");
            }

            free_idx_.emplace_back(idx);
        }

        void reset() {
            free_idx_.clear();
            std::iota (free_idx_.begin (), free_idx_.end (), 0);
        }

        void reserve(size_t n) {
            if (n <= data_.size()) {
                return;
            }

            size_t old_size = data_.size();
            data_.resize(n);
            for (size_t i = old_size; i < n; ++i) {
                free_idx_.emplace_back(i);
            }
        }

        [[nodiscard]] size_t size() const { return data_.size(); }
        [[nodiscard]] size_t capacity() const { return data_.capacity(); }

    private:
        std::vector<T> data_;
        std::vector<size_t> free_idx_;
    };
}

#endif //VINSEKF_POOL_H
