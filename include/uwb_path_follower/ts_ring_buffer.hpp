#pragma once
#include <deque>
#include <utility>
#include <optional>
#include <mutex>
#include <chrono>
#include <algorithm>

namespace uwb_path {

template <class T>
class TsRingBuffer {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit TsRingBuffer(std::chrono::milliseconds horizon = std::chrono::milliseconds(2000))
    : horizon_(horizon) {}

    void push(TimePoint t, const T& v) {
        std::lock_guard<std::mutex> lk(mu_);
        buf_.emplace_back(t, v);
        purge_();
    }

    // Get the sample at time t (linear interpolation if T supports + * scalar)
    std::optional<T> get(TimePoint t) const {
        std::lock_guard<std::mutex> lk(mu_);
        if (buf_.empty()) return std::nullopt;
        
        // ensure chronological
        auto it = std::lower_bound(buf_.begin(), buf_.end(), t,
            [](auto const& p, TimePoint tt){ return p.first < tt; });
        
        if (it == buf_.begin()) return it->second;
        if (it == buf_.end())   return buf_.back().second;
        
        auto const& [t1, v1] = *(it-1);
        auto const& [t2, v2] = *it;
        if (t2 == t1) return v2;
        
        double a = double((t - t1).count()) / double((t2 - t1).count());
        return lerp_(v1, v2, a);
    }

    // Get raw newest/oldest time for diagnostics
    std::optional<TimePoint> newest_time() const {
        std::lock_guard<std::mutex> lk(mu_);
        if (buf_.empty()) return std::nullopt;
        return buf_.back().first;
    }

    std::optional<TimePoint> oldest_time() const {
        std::lock_guard<std::mutex> lk(mu_);
        if (buf_.empty()) return std::nullopt;
        return buf_.front().first;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mu_);
        return buf_.size();
    }

    void clear() {
        std::lock_guard<std::mutex> lk(mu_);
        buf_.clear();
    }

private:
    void purge_() {
        if (buf_.empty()) return;
        auto cutoff = buf_.back().first - horizon_;
        while (!buf_.empty() && buf_.front().first < cutoff) {
            buf_.pop_front();
        }
    }

    static T lerp_(T const& a, T const& b, double u) {
        // requires T to support: a + (b-a)*u
        return a + (b - a) * u;
    }

    mutable std::mutex mu_;
    std::deque<std::pair<TimePoint, T>> buf_;
    std::chrono::milliseconds horizon_;
};

} // namespace uwb_path