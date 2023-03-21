/** @file encoder.h
 *  @brief Headder file for encoder
    @author Chow 
 *  @date 2022-11-01 Pulled code into .cpp and .h for easier use 
 */
#include <Arduino.h>
class encoder
{
    protected:
        int8_t pin1;
        int8_t pin2;
        int32_t counter;
        int8_t aLastState;
        int8_t aState;
        uint8_t pin_ch2;
        int8_t duty;

    public:
        encoder(int8_t out_a, int8_t out_b);
        int32_t measure(void);
        void reset(void);

};