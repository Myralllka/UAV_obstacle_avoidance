//
// Created by myralllka on 17/07/2022.
//

#ifndef UAV_CAMERA_LOCALIZATION_BARRIER_HPP
#define UAV_CAMERA_LOCALIZATION_BARRIER_HPP
namespace camera_localization {
    class barrier {
    private:
        std::atomic<size_t> thread_count;
        std::atomic<size_t> counter;
        std::atomic<size_t> waiting;

        std::mutex m_m{};
        std::condition_variable m_cv{};

    public:
        explicit barrier(size_t count) : thread_count(count), counter(0), waiting(0) {}

        void wait() {
            //fence mechanism
            ++counter;
            ++waiting;
            std::unique_lock<std::mutex> lk(m_m);
            m_cv.wait(lk, [&] { return counter >= thread_count; });
            m_cv.notify_all();
            --waiting;
            if (waiting == 0) {
                counter = 0;
            }
            lk.unlock();
        }
    };
}
#endif //UAV_CAMERA_LOCALIZATION_BARRIER_HPP
