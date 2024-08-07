#pragma once

#include <array>
#include <vector>

#include "pico/critical_section.h"
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"

/** \file i2s.hpp
 *  \defgroup pico_i2s_pio pico_i2s_pio
 *  I2S audio output using the PIO
 *
 *  This library uses the \ref hardware_pio system to implement a I2S audio interface
 *  Uses 32bit sample as stereo-16bit samples.
 *
 * \todo Must be more we need to say here.
 * \todo certainly need an example
 * 
 */

#include "static_fifo.hpp"

namespace pico {
    class I2S {
        public:
            struct config {                                
                int pin_bclk, pin_din, pin_dout;
            };

            static constexpr auto Audio_Vector_Count = 4; // stereo
            static constexpr auto Audio_Vector_Length = 64;
            static constexpr auto sample_rate = 44100;

            using audio_sample_t = int32_t;
            using audio_vector_t = std::array<audio_sample_t, Audio_Vector_Count>;

            /** \brief \todo
             * \ingroup pico_i2s_pio
             *             
             * \param pin_bclk
             * \param pin_din
             * \param pin_dout             
             * \return
             */
            
            I2S(const int pin_bclk, const int pin_din, const int pin_dout);            

            void setup();
            void begin();            
            void update();            

            static constexpr auto DMA_Buffer_Size = Audio_Vector_Count;            
            static std::vector<std::reference_wrapper<I2S>> instances;
                        
        private:
            enum Stream_Mode {
                In,
                Out,
                InOut
            };
            
            Stream_Mode stream_mode;
            critical_section cs;
            mutex_t mutex;

            PIO pio = pio0;
            uint irq_index = DMA_IRQ_0;
            const int pin_bclk, pin_din, pin_dout, pin_lrclk;
            // const config c; 
                
            int sm = -1;
            void pio_init();        

            int dma_channel = -1;
            void dma_init(const int _dma_channel = -1);

            // audio_vector_t read_buffer, write_buffer, swap_buffer;
            static_fifo<audio_sample_t, Audio_Vector_Length> fifo;
            bool need_next_frame = true;
            bool ready_to_transfer = true;

            void transfer_next_buffer();

            void lock() {
                // critical_section_enter_blocking(&cs);        
                mutex_enter_blocking(&mutex); 
            }

            void unlock() {
                // critical_section_exit(&cs);
                mutex_exit(&mutex);
            }

        public:
            inline int get_dma_channel() const {
                return dma_channel;
            };

            inline void push() {
                fifo.push();
            }

            inline void pop() {
                fifo.pop();
            }

            inline auto& get_read_buffer() {
                return fifo.get_read_head();
            }

            inline auto& get_next_read_buffer() { 
                pop();                               
                return fifo.get_read_head();
            }

            inline auto available()  {
                lock();
                const auto result = fifo.available();
                unlock();
                return result;
            }

            inline void put(const int16_t L, const int16_t R) {
                // critical_section_enter_blocking(&cs);    
                lock();
                fifo.put(uint32_t(L) << 16 | uint32_t(R));        
                unlock();
            }

            void irq_callback();

            static void dma_irq0_handler();
            static void dma_irq1_handler();
    };
}

