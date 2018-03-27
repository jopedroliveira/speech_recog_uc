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

// Container for all classes headers used in the speech recognition module
//Global parameters
#include "GLOBAL_FLAGS.h"
#include "GLOBAL_PARAMETERS.h"



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  ####  #####  #    # ###### ######     ####  #        ##    ####   ####  
// #    # #    # #    # #      #         #    # #       #  #  #      #      
// #      #####  #    # #####  #####     #      #      #    #  ####   ####  
// #      #    # #    # #      #         #      #      ######      #      # 
// #    # #    # #    # #      #         #    # #      #    # #    # #    # 
//  ####  #####   ####  #      #          ####  ###### #    #  ####   #### 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifndef CBUFFCLASS_H
#define CBUFFCLASS_H

//// Custom Types
const float dec_h[33] = {-0.00137865,0.00000000,0.00226523,0.00334109,
	-0.00000000,-0.00712983,-0.01003564,0.00000000,0.01861181,0.02480557,
	-0.00000000,-0.04387216,-0.05964528,0.00000000,0.13303785,0.27329340,
	0.33341323,0.27329340,0.13303785,0.00000000,-0.05964528,-0.04387216,
	-0.00000000,0.02480557,0.01861181,0.00000000,-0.01003564,-0.00712983,
	-0.00000000,0.00334109,0.00226523,0.00000000,-0.00137865};
short * dec_delay_malloc;
short * dec_delay_ptr;
short * dec_downsampling_input_buffer;

//==============================================================================
class CircularBuffer {
	// CircularBuffer attributes
	short * data_buffer;		// data 
	int read_index; 			// reading index
	int write_index;			// writting index
	int chunk_size_bytes; 		// size of a chunk in bytes
	int chunk_size_samples;		// size of a chunk in shorts (eqv sample)
	int chunk_size_frames;		// number of frames in a chunk (frame: 1 
														//mono sample, two streo samples, or nchan samples)
	int num_chunks_buff;		// number of chunks in data_buffer
	int nchan;					// number of channels 0 1 2 3 ...
	bool is_full;				// is the data_buffer full?
	bool is_empty;				// is the data_buffer empty?
	int fs;						// sample rate in Hertz

	float computeRMS(int index);
	void computeRMSmean(float * means);
	void init(int chan, int fsamp, int buffer_duration, float chunk_duration);

public:
	//Constructor 
	CircularBuffer(int chan = 1, int fsamp = 16000, int buffer_duration = 15,
		float chunk_duration = 0.1);


	// Methods 
	// copy a givven chunk of data into the buffer and returns its RMS value
	// Also is responsible to increment the circular buffer indexes
	float put(short * chunk);
	// changes the argument pointer to points to the current cirbuff read index.
	// returns true/false if the buffer is/is not empty
	// Also is responsible to increment the circular buffer indexes
	bool get(void * dest);
	// Returns the pointer to the current circulsarbuffer read index, if not empy.
	// Also is responsible to increment the circular buffer indexes
	short * getPointer(void);
	// Backward the circularbuffer read pointer n indexes
	void backward(int n);
	// Increment the read indexe 1 step, ignoring the current index
	void pass(void);
	bool isEmpty(void);
	bool isFull(void);
	int getReadIndex(void);
	int getBufferSizeBytes(void);
	int getChunkSizeBytes(void);
	int getNumOfChunks(void);
	int getFrameSizeBytes(void);
	int getChunkSizeSamples(void);
	int getChunkSizeFrames(void);
	int getNumOfFilledChunks(void);
	short * getPointerNoIncrement(void);
};


#endif



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// #     # #######  #####  #     #                                       
// #     # #       #     # ##   ##     ####  #        ##    ####   ####  
// #     # #       #       # # # #    #    # #       #  #  #      #      
// #     # #####    #####  #  #  #    #      #      #    #  ####   ####  
//  #   #  #             # #     #    #      #      ######      #      # 
//   # #   #       #     # #     #    #    # #      #    # #    # #    # 
//    #    #        #####  #     #     ####  ###### #    #  ####   #### 
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


#ifndef VADMACHINECLASS_H
#define VADMACHINECLASS_H


// Globals
typedef void vadcb (int N, bool isSpeech, void* vaddata);

class VADFSMachine{
	//MACHINE STATES
	enum STATES {STsilence, STpossible_speech, STspeech, STpossible_silence	};

	// VARS
	int rstTimeOutInternalCounter;
	int timeOutInternalCounter;
	bool firstTime = true;

	float thOffset; //decision threshold
	int counterSpeech; //internal speech decision counter
	int rstCounterSpeech;  //number of chunks/frames for decide as speech
	int counterSilence; //internal silence decision counter
	int rstCounterSilence; //number of chunks/frames for decide as silence
	int status; //current state of the FSM
	bool inSpeech; //if status is speech
	float a,a1; //a and (1-a)
	float threshold;
	float ymin_prev;
	float ymax_prev;
	float ymed_prev;
	float ener_prev;
	int nBackward;


	void init(float setthOffset, int setcounterSpeech, int setcounterSil,
		float setaval);

	public:
		// Constructors
		VADFSMachine(float thOffset = 15.0F, int counterSp = 5, int counterSil = 5,
			float aConst = 0.8F);


		// Public Methods
		/* update the current machine state according the new rms value, 
		the previous rms value, max rms value, min rms value and mean. 
		It takes into account the internal counters and the current state to 
		deremine the next state. 
		Depending on the next state (the current chunk state), it evoques 
		the VADClass callback that manage the circular buffer and the audio
		 content according the state */
		void updateStatus(float rmsValue);
		// A method that indicates the current status of the machine
		bool isSpeech(void);
		// Indicates the number of audio chunks already in the buffer, 
		// when voice activity was detected (related to the internal counteres)
		//returns the the the number of chunks of speech already in the buffer.
		int getStartOfSpeechOffset(void); 
		void setThreshold(float thOffset_new);
		void setSpeechCounter(int cntSpeek_new);
};

#endif



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// #     #    #    ######      #####  #          #     #####   #####  
// #     #   # #   #     #    #     # #         # #   #     # #     # 
// #     #  #   #  #     #    #       #        #   #  #       #       
// #     # #     # #     #    #       #       #     #  #####   #####  
//  #   #  ####### #     #    #       #       #######       #       # 
//   # #   #     # #     #    #     # #       #     # #     # #     # 
//    #    #     # ######      #####  ####### #     #  #####   #####  
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

#ifndef VADCLASS_H
#define VADCLASS_H
// Includes
#include "portaudio.h"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime> // Needed for the true randomization
#include <cstdlib> 

// External user callback function, used to evoque the callback function on
// the user side to handle the audio that contains speech
typedef void usercb(short * data, int N, bool isSOS, bool isEOS, void * func);

// Structure that contains pointers to the objects that are needed on the 
//PortAudio callback (to put the audio in the circular buffer and to evaluate 
// the new chunk audio content on the finite state machine). 
struct PaCallbackPointers {
	CircularBuffer * cbuff;
	VADFSMachine * vadmachine;
	int nchan;
};

// Strcuture that contains pointers to the objects that are needed on the 
// vadmachine callback, used to evoque the external callback 
// when voice activity is detected.
struct VADMachineCallbackPointers {
	CircularBuffer * cbuff;
	void * externalusercallback;
	void * externaluserdata;
};




struct wavHeader { //Para HEADER DE 44 bytes,
	char	RIFF[4];	//4: 'RIFF', "low endian"
	int	RIFFsize;	//4: size of "RIFF chunk" (do not use)
	char	fmt[8];	//8: string with 'WAVEfmt '
	int	fmtSize;	//4: size of "format chunk": usualy 14, 16, 18 or 40 bytes
	short	fmtTag;	//2: "format tag". Only consider 1=PCM (no encoding)
	short	nchan;	//2: no. of channel (only mono or stereo; 
								//otherwhise it is not WAVEFORMATEXTENSIBLE)
	int	fs;		//4: sample rate
	int	avgBps;	//4: AvgBytesPerSec
	short	nBlockAlign;//2: nchan*bytes_per_value = bytes per sample;	
	short	 bps;		//2: Number of bits per sample of mono data 
							//(only if it is WAVEFORMATEX: fmtSize>=16)
	char	data[4];	//4: 'data' chunk
	int	datasize;	//4: no. Data Bytes (size of "data chunk")
};



class VADClass {
	// Main components: a circular buffer, a vad finite state
	// machine and a portaudio instance
	CircularBuffer * cbuff = NULL;
	VADFSMachine * vadmachine = NULL;
	PaStream * stream = NULL;

	int sample_rate;								// audio sample rate in Hertz
	int num_channels;								// number of channels
	float chunk_duration;						// chunk duration in seconds 
	int cbuffer_duration;						// circular buffer duration in seconds 
	float vmachine_thresholdoffset;	// vad machine threshold offset in dB
	int cntSpeech;
	int cntSil;
	float aTemporalConst;
	bool isFromFile;
	char * filepath;
	std::thread lookupthread;

	PaError err;										// portaudio error controller
	usercb * externalfunction;			// routine to consume the speech data
	void * userdata;
	
	
	//private methods
	void init(usercb * usercallbackfunction, void * userdata, int fs, int nchan,
		float chunkdur, int bufdur, float vadthoffset, int cntSpeek, int cntSilence,
		float aTimeConst,bool isFile, char * file_path);
	void vadinit(void);
	void readWavHeader(wavHeader *wavhdr, FILE *fi);
	void vadmachine_lookup_thread(void);


public:
	//Auxiliar compontents for audio filtering and decimation
	// Only constructors are public
	VADClass(usercb * usercallbackfunction, void * userdata, int fs = 16000,
		int nchan = 1, float chunkdur = 0.1, int bufdur = 15, float vadthoffset = 15,
		int cntSpeek = 5, int cntSilence = 8, float aTimeConst = 0.8F, 
		bool isFile = false, char* file_path = NULL);
	void vadterminate(void);
	void vadMachineAdjustParameters(float thOffset_new, int cntSpeek_new);
	static int portAudioCallback(const void *inputBuffer, void *outputBuffer,
		unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo* timeInfo,
		PaStreamCallbackFlags statusFlags, void * datastructs);
	void setusercallback(usercb * usercallback_new);
	void stopStream(void);
	void resumeStream(void);
};

#endif

