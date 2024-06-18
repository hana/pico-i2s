#include <functional>
#include <stdio.h>
#include <limits>

#include "hardware/dma.h"
#include "hardware/irq.h"

#include "i2s.hpp"
#include "i2s.pio.h"

using namespace pico;

std::vector<std::reference_wrapper<I2S>> I2S::instances;

void I2S::irq_callback() {
    ready_to_transfer = true;
    dma_irqn_acknowledge_channel(irq_index - DMA_IRQ_0, dma_channel);   
    transfer_next_buffer();
}

void I2S::dma_irq0_handler() {    
    for(auto instance : I2S::instances) {        
        auto& i2s = instance.get();
        if(i2s.irq_index == DMA_IRQ_0) {
            i2s.irq_callback();
            return;
        }
    }
}

void I2S::dma_irq1_handler() {
    for(auto instance : I2S::instances) {
        auto& i2s = instance.get();
        if(i2s.irq_index == DMA_IRQ_1) {
            i2s.irq_callback();
            return;
        } 
    }   
}

I2S::I2S(const int _pin_bclk, const int _pin_din, const int _pin_dout) : pin_bclk(_pin_bclk), pin_din(_pin_din), pin_dout(_pin_dout), pin_lrclk(_pin_bclk+1) {
    stream_mode = Stream_Mode::Out;

    if(pin_din < 0 && pin_dout < 0) {    
        assert(false);
    } else if(pin_din < 0 && 0 <= pin_dout) {
        stream_mode = Stream_Mode::Out;        
    } else if(pin_dout < 0 && 0 <= pin_din) {
        stream_mode = Stream_Mode::In;
    } else {
        stream_mode = Stream_Mode::InOut;  
    }

    instances.emplace_back(*this);    
};

void I2S::setup() {
    fifo.clear();
    critical_section_init(&cs);
    pio_init();
    dma_init();
}

void I2S::begin() {
    irq_callback();
};

void I2S::update() {
    if(ready_to_transfer) {
        transfer_next_buffer();
    }
}

void I2S::pio_init() {         
    const auto offset = pio_add_program(pio, &i2s_16s_program);
    auto sm_config = i2s_16s_program_get_default_config(offset);
    sm = pio_claim_unused_sm(pio, true);

    if (sm < 0) {
        pio = pio1;
        sm = pio_claim_unused_sm(pio, true);

        if(sm < 0) assert(false);
    }

    const auto func = pio == pio0 ? GPIO_FUNC_PIO0 : GPIO_FUNC_PIO1;
    gpio_set_function(pin_bclk, func);
    gpio_set_function(pin_lrclk, func);

    if(stream_mode == Stream_Mode::In || stream_mode == Stream_Mode::InOut) {
        gpio_set_function(pin_din, func);
    }  
    
    if (stream_mode == Stream_Mode::Out || stream_mode == Stream_Mode::InOut) {
        gpio_set_function(pin_dout, func);
    }

    if(stream_mode == Stream_Mode::In || stream_mode == Stream_Mode::InOut) {
        sm_config_set_out_pins(&sm_config, pin_din, 1);
    }  
    
    if (stream_mode == Stream_Mode::Out || stream_mode == Stream_Mode::InOut) {
        sm_config_set_out_pins(&sm_config, pin_dout, 1);
    }

    sm_config_set_sideset_pins(&sm_config, pin_bclk);

    constexpr auto autopull = true;
    constexpr auto shift_right = false;
    sm_config_set_out_shift(&sm_config, shift_right, autopull, 32);

    if(stream_mode == Stream_Mode::In) {
        sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
    } else if (stream_mode == Stream_Mode::Out) {
        sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
    }

    const float div = (float)clock_get_hz(clk_sys) / (sample_rate * 2 * 32);    
    sm_config_set_clkdiv(&sm_config, div);

    pio_sm_init(pio, sm, offset, &sm_config);
    uint32_t pin_dirs, pin_mask;

    switch (stream_mode)    {
    case Stream_Mode::In:
        pin_dirs = (3u << pin_bclk);
        pin_mask = (1u << pin_din) | (3u << pin_bclk);
        break;
    case Stream_Mode::Out:
        pin_dirs = (1u << pin_dout) | (3u << pin_bclk);
        pin_mask = (1u << pin_dout) |(3u << pin_bclk);
        break;
    default:
        pin_dirs = (1u << pin_dout) | (3u << pin_bclk);
        pin_mask = (1u << pin_dout) | (1u << pin_din) | (3u << pin_bclk);
        break;
    }

    pio_sm_set_pindirs_with_mask(pio, sm, pin_dirs, pin_mask);
    pio_sm_set_pins(pio, sm, 1); // clear pins

    pio_sm_exec(pio, sm, pio_encode_jmp(offset + i2s_16s_offset_entry_point));    
    pio_sm_set_enabled(pio, sm, true);
}

void I2S::dma_init(const int _dma_channel) {
    if(_dma_channel < 0) {
        dma_channel = dma_claim_unused_channel(true);
    } else {
        dma_channel = _dma_channel;
    }

    auto c = dma_channel_get_default_config(dma_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);

    const auto dreq = pio_get_dreq(pio, sm, true);    
    channel_config_set_dreq(&c, dreq);
    
    dma_channel_configure(
        dma_channel, 
        &c,
        &pio->txf[sm], 
        nullptr,    //read_buffer.data(), 
        0,
        false
    );

    constexpr auto exclusive_handler_required = true;
    if (irq_get_exclusive_handler(DMA_IRQ_0) == nullptr) {
        irq_index = DMA_IRQ_0;
        irq_set_exclusive_handler(irq_index, I2S::dma_irq0_handler);              
    } else if (irq_get_exclusive_handler(DMA_IRQ_1) == nullptr) {
        irq_index = DMA_IRQ_1;
        irq_set_exclusive_handler(irq_index, I2S::dma_irq1_handler);                
    } else {
        if(exclusive_handler_required) {
            assert(false); 
        } else {
            if (!irq_has_shared_handler(DMA_IRQ_0)) {
                irq_index = DMA_IRQ_0;
                irq_add_shared_handler(irq_index, I2S::dma_irq0_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
            } else if (!irq_has_shared_handler(DMA_IRQ_1)) {
                irq_index = DMA_IRQ_1;
                irq_add_shared_handler(irq_index, I2S::dma_irq1_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
            }  else {
                irq_index = DMA_IRQ_0;  
                irq_add_shared_handler(irq_index, I2S::dma_irq0_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
            }            
        }
    }

    const auto dma_irqn = irq_index - DMA_IRQ_0;    
    dma_irqn_set_channel_enabled(dma_irqn, dma_channel, true);
    irq_set_enabled(irq_index, true);        
}

void I2S::transfer_next_buffer() {    
    fifo.pop(); 
    ready_to_transfer = false;
    auto [next_buffer, buffer_size] = fifo.get_read_buffer(Audio_Vector_Count);
    dma_channel_transfer_from_buffer_now(dma_channel, next_buffer, buffer_size);
}
