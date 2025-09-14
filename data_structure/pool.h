//
// Created by 许家仁 on 2025/8/23.
//

#ifndef VINSEKF_POOL_H
#define VINSEKF_POOL_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <functional>
#include <iostream>  // 用于测试输出
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
                reserve(initial_size);
            }
        }

        T* allocate(std::function<void(T&)> init_func = [](T&) {}) {
            if (free_idx_.empty()) {
                // 无空闲内存，新增元素并初始化
                std::cerr << "Memory over." << std::endl;
                data_.emplace_back(T());
                init_func(data_.back());
                return &data_.back();
            } else {
                // 复用空闲内存并初始化
                size_t idx = *free_idx_.begin();  // 取任意空闲索引
                free_idx_.erase(idx);             // 从空闲集合移除
                // 应用初始化
                init_func(data_[idx]);
                return &data_[idx];
            }
        }

        void deallocate(T* ptr, std::function<void(T&)> reset_func = [](T&) {}) {
            if (ptr == nullptr) {
                return;
            }

            // 计算指针在vector中的索引
            size_t idx = ptr - data_.data();

            // 检查指针合法性
            if (idx >= data_.size()) {
                throw std::invalid_argument("释放的指针不属于当前内存池");
            }
            // 检查是否已释放（O(1)查找）
            if (free_idx_.count(idx)) {
                throw std::invalid_argument("指针已被释放");
            }

            // 调用重置函数清理对象
            reset_func(data_[idx]);

            // 将索引放回空闲列表
            free_idx_.emplace(idx);
        }

        void reset(std::function<void(T&)> reset_func = [](T&) {}) {
            // 对所有已分配的元素执行重置
            for (size_t i = 0; i < data_.size(); ++i) {
                // 只重置正在使用的元素（不在free_idx_中）
                if (!free_idx_.count(i)) {
                    reset_func(data_[i]);
                }
            }

            // 重新初始化空闲列表
            free_idx_.clear();
            for (size_t i = 0; i < data_.size(); ++i) {
                free_idx_.emplace(i);
            }
        }

        void reserve(size_t n) {
            if (n <= data_.size()) {
                return;
            }

            size_t old_size = data_.size();
            data_.resize(n);    // 扩容时默认构造新元素
            // 新元素的索引加入空闲列表
            for (size_t i = old_size; i < n; ++i) {
                free_idx_.emplace(i);
            }
        }

        [[nodiscard]] size_t size() const { return data_.size(); }
        [[nodiscard]] size_t capacity() const { return data_.capacity(); }
        [[nodiscard]] size_t free_count() const { return free_idx_.size(); }

    private:
        std::vector<T> data_;
        std::unordered_set<size_t> free_idx_;     // 空闲索引集合（O(1)查找）
    };
}

#endif //VINSEKF_POOL_H
