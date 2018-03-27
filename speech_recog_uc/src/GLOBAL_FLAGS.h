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

Developed by: 
Jose Pedro Oliveira, joliveira@deec.uc.pt
Fernando Perdigao, fp@deec.uc.pt

Copyright (c) 2018 Universidade de Coimbra
*/

////////////////////////////////////////////////////////////////////////////////

// FLAG VALUES - DO NOT CHANGE UNLESS YOU KNOW WHAT YOU ARE DOING

// Vad Finite State Machine VADFSMCLASS.CPP
#define END_OF_SSPEECH_BACKWARD_FLAG -1
#define NORMAL_SSPEECH_BACKWARD_FLAG 0
#define NON_SPEECH_FLAG 0

// Main ROS NODE
#define GOOGLE_STREAMING_CONFIG_SAMPLE_RATE 16000
#define GOOGLE_STREAMING_CONFIG_GET_INTERIM_RESULTS true

//includes
#include <chrono>
#include <string>
#include <fstream>
#include <cstdlib>
#include <thread>
#include <math.h> 
#include <cstring>
#include <stdlib.h>
#include <stdio.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <boost/bind.hpp>

// Google stuff
#include <grpc++/grpc++.h>
#include "google/cloud/speech/v1/cloud_speech.grpc.pb.h"
#include <speech_recog_uc/SpeechResult.h>
#include <speech_recog_uc/DOAResult.h>
