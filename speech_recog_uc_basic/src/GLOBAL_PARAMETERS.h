/* MIT License

Copyright (c) 2018 Universidade de Coimbra

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

// EDITABLE VALUES
//################################################
//
// Vad Finite State Machine VADFSMCLASS.CPP
#define START_OF_SPEECH_BACKWARD_OFFSET 3

// Vad CircularBuffer CIRCULARBUFFERCLASS.CPP
#define A_WEIGHTING_FILTER_CONSTANT_A 0.1202
#define A_WEIGHTING_FILTER_CONSTANT_G 2.2814
#define A_WEIGHTING_FILTER_CONSTANT_H 2.0072
#define VAD_WEIGHTING_UPDATE_CONST 0.8

// Vad CLASS VADCLASS.CPP
#define SAMPLE_FORMAT_BIT_MAX_SCALE 32767 //2^(15)-1
#define SAMPLE_FORMAT_BIT_MIN_SCALE -32768 //-2^15
#define USE_DEFAULT_DEVICE_FLAG true
#define DEVICE_NAME "ASTRA"
#define ANTI_ALIASING_FILTERING_AND_DECIMATION_FLAG true

// Main ROS NODE
#define GLOBAL_OUTPUT_MODE true
#define MAIN_FILE_DIRECTORY_PATH "/home/speech"

#define DEFAULT_VAD_ENERGY_THRESHOLD_OFFSET_FOR_LISTENING 8.0
#define DEFAULT_VAD_INTERNAL_COUNTER_FOR_LISTENING 3
#define DEFAULT_VAD_INTERNAL_COUNTER_SIL 12
#define DEFAULT_VAD_INTERNAL_TIMEOUT_COUNTER 140

// KEEP IN MIND THE ACQUISITION SAMPLE RATE WILL BE DECIMATION_FACTOR_SAMPLES*GLOBAL_SAMPLE_RATE
#define GLOBAL_SAMPLE_RATE 16000
#define GLOBLAL_NUMBER_OF_CHANNELS 1
#define GLOBLAL_CHUNK_DURATION_IN_SECONDS 0.1
#define GLOBAL_CIRCULAR_BUFFER_DURATION 15
