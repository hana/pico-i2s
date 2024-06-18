#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <utility>

namespace   {
    namespace LED{
        constexpr auto Pin = 14;
        bool state = false;
    }
}

template<typename T, std::size_t N>
class static_fifo {
private:
    // std::array<T, N> buffer;
    std::vector<T> buffer;
    std::size_t read_index = N / 2;
    std::size_t write_index = 0;
    bool writing = false;
    bool reading = false;
    int proceed_count = 1;
public:

    static_fifo() {
        buffer = std::vector<T>(N);
        clear();
        gpio_init(LED::Pin);
        gpio_set_dir(LED::Pin, GPIO_OUT);    
    }

    T& get_read_head() {
        reading = true;        
        return buffer[read_index];
    }

    void put(const T& data) {        
        buffer[write_index++] = data;
        
        if (N <= write_index) {
            write_index = 0;
        }
    }

    void pop() {        
        read_index += proceed_count;
        if(N <= read_index) {
            read_index -= N;            
        }
        reading = false;
    }

    void push() {         
        write_index++;
        if(N <= write_index) {
            write_index = 0;
        }                    
        writing = false;        
    }

    auto size() {            
        return read_index < write_index ? write_index - read_index : N - read_index + write_index;         
    }

    bool available()  {
        return size() < N;
    }

    void clear() {
        read_index = N / 2;
        write_index = 0;
        writing = false;        
        for(auto& b : buffer) {
            b = T();
        }        
    }

    T* get_next_buffer() {
        auto ptr = &buffer[read_index];

        if(N <= read_index) {
            read_index = 0;
        }

        return ptr;
    }

    std::pair<T*, int> get_read_buffer(const int max_count) {
        gpio_put(LED::Pin, LED::state = !LED::state);
             
        if(N <= (read_index + max_count)) {
            auto head =  &buffer[read_index];
            auto count = N - read_index;            
            proceed_count = count;
            return {head, count};    
        } else {
            auto head = &buffer[read_index];
            auto count = max_count;
            proceed_count = count;
            return {head, count};
        }
    }
};