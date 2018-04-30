//
// Based on from https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads
//

#include <mutex>
#include <condition_variable>

class Countdown {
public:
    Countdown (int initialCount)
    {
        count = initialCount;
    }

    void signal()
    {
        //std::printf("Decrementing countdown...\n");
        std::unique_lock<std::mutex> lock(mtx);
        --count;
        cv.notify_one();
        std::printf("Decremented countdown...\n");
    }

    void wait()
    {
        //std::printf("Beginning wait for countdown...\n");
        std::unique_lock<std::mutex> lock(mtx);

        while(count > 0){
            cv.wait(lock);
        }
        std::printf("Done waiting for countdown...\n");
    }

private:
    std::mutex mtx;
    std::condition_variable cv;
    int count;
};