/* Copyright 2024 The The Agilex Robotics Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#pragma once
#ifndef _BLOCKING_DEQUE_H_
#define _BLOCKING_DEQUE_H_

#include <mutex>
#include <math.h>
#include <condition_variable>

template<typename T>
class BlockingDeque{
private:
    std::deque<T> deque;
    std::mutex mutex;
    std::condition_variable condition;

public:
    void push_back(const T& item){
        std::lock_guard<std::mutex> lock(mutex);
        while(deque.size() >= 2000)
            deque.pop_front();
        deque.push_back(item);
        condition.notify_one();
    }

    T getRecentItem(double time){
        T item;
        std::lock_guard<std::mutex> lock(mutex);
        double minTimeDiff = INFINITY;
        for(int i=0; i < deque.size(); i++){
            double timeDiff = fabs(deque.at(i).header.stamp.toSec() - time);
            if(timeDiff < minTimeDiff){
                item = deque.front();
                deque.pop_front();
            }
            else
                break;
        }
        return item;
    }

    T pop_front(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.front();
        deque.pop_front();
        return item;
    }

    T front(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.front();
        return item;
    }

    T pop_back(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.back();
        deque.pop_back();
        return item;
    }

    T back(){
        std::unique_lock<std::mutex> lock(mutex);
        while(deque.empty())
            condition.wait(lock);
        T item = deque.back();
        return item;
    }

    int size(){
        std::lock_guard<std::mutex> lock(mutex);
        return deque.size();
    }
};

#endif
