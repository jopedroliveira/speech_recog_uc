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

// Container for all classes code used in the speech recognition module
// higher -> lower level
#include "speech_node_classes.h"



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
//  ####   #         ####  #####    ##   #       ####  
// #    #  #        #    # #    #  #  #  #      #
// #       #        #    # #####  #    # #       ####
// #  ###  #        #    # #    # ###### #           #
// #    #  #        #    # #    # #    # #      #    #
//  ####   ######    ####  #####  #    # ######  ####  
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////



// CHANNEL SELECTION AND AUDIO DECIMATION ======================================
void channel_selection_function(short * dest, short* orig, int framesPerBuffer){
	// Function that performs channel selection
	for (int n = 0; n < framesPerBuffer; n++) {
		dest[n] = orig[n*GLOBAL_NUMBER_OF_CHANNELS];
	}
}


void decimation_function(short * dest, short * orig, 
	int framesPerBuffer){
	// Receives a stereo 48kHz chunk. 
	// Yields a 16kHz mono sig with no quality loss
	// Uses only the left channel

	short * leftChannelOnly = (short *) malloc(framesPerBuffer*sizeof(short));
	channel_selection_function(leftChannelOnly, orig, framesPerBuffer);

	int Cbuff_frameLen = framesPerBuffer/dec_M;
	int n,m;
	if(ANTI_ALIASING_FILTERING_AND_DECIMATION_FLAG){
		for(n = 0 ; n < dec_NN; n++){
			m = n*dec_M;
			float s = 0.0F;
			for(int k = 0; k < dec_N; k++){
				if(m<0)
					s+=dec_h[k]*dec_delay_ptr[m];
				else
					s+= dec_h[k] * leftChannelOnly[m];
				m--;
			}
			if (int(s)> 32767) 
				s = 32767; 
			else if (int(s)<-32768) 
				s = -32768;
			dest[n] = s;
		}

		for( ; n<Cbuff_frameLen; n++){
			m = n*dec_M; 
			float s = 0.0F;
			for(int k = 0; k < dec_N; k++){
				s+= dec_h[k]*leftChannelOnly[m];
				m--;
			}
			
			if (int(s)> 32767) 
				s = 32767; 
			else if (int(s)<-32768) 
				s = -32768;
			dest[n] = s;
		}
	} else {
		for(int i = 0; i< Cbuff_frameLen; i++){
			dest[i] = leftChannelOnly[dec_M*i];
		}
	}
	memcpy(dec_delay_malloc,&leftChannelOnly[framesPerBuffer-dec_N],dec_N
		* sizeof(short));
}

// DIRECTION OF ARRIVAL ESTIMATION ==============================================

void reset_doa_vars(){
// Restarts doa estimation variables
	for(int i = -DOA_HISTOGRAM_MAX_LAG; i <= DOA_HISTOGRAM_MAX_LAG; i++){		
		lags[i+DOA_HISTOGRAM_MAX_LAG] = i;
	}
	for (int clc = 0; clc  < DOA_HISTOGRAM_TOT_NUM; clc++) {
		direction_of_arrival_histogram[clc] = 0;
	}
	for (int k = 0; k < GLOBAL_NFFT; k++) {
			x[k] = 0; y[k] = 0;
			x0[k] = 0; x1[k] = 0;
	}
	// Compute the hamming window of chunkSizeSamples_N samples
	for(int wn = 0; wn < chunkSizeSamples_N; wn++ ){
		hamwin[wn] = (Real) (0.54 - 0.46*cos(wn*t));
	}
};










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

//=============================================================================
// Constructors
//=============================================================================
VADClass::VADClass(usercb * usercallbackfunction, void * userdata, int fs, 
	int nchan, float chunkdur, int bufdur, float vadthoffset, int cntSpeek, 
	int cntSilence, float aTimeConst, bool isFile, char * file_path) {
	init(usercallbackfunction, userdata, fs, nchan, chunkdur, bufdur, 
	vadthoffset, cntSpeek, cntSilence, aTimeConst,isFile,file_path);
}

//== init  ========================================================================
void VADClass::init(usercb * usercallbackfunction, void * usrdta, int fs, 
	int nchan, float chunkdur, int bufdur, float vadthoffset, int cntSpeek, 
	int cntSilence, float aTimeConst, bool isFile, char * file_path) {
	externalfunction = usercallbackfunction;
	userdata = usrdta;
	sample_rate = fs;
	num_channels = nchan;
	chunk_duration = chunkdur;
	cbuffer_duration = bufdur;
	vmachine_thresholdoffset = vadthoffset;
	cntSpeech = cntSpeek;
	cntSil = cntSilence;
	aTemporalConst = aTimeConst;
	isFromFile = isFile;
	filepath = file_path;
	dec_delay_malloc = (short *)malloc(dec_N*sizeof(short)*1); 
	dec_delay_ptr = &dec_delay_malloc[dec_N*1];
	vadinit();
}



//==============================================================================
void VADClass::vadmachine_lookup_thread(void){
	// Thread that looks up the vad finite state machine and acts according its status
	int silence = 0;
	int speech = 1;
	int previous_state = silence; // silence
	Pa_Sleep(100); //security
	while(true){
		if(!cbuff->isEmpty()){
			if(!vadmachine->isSpeech()){
				if(previous_state == speech){
					while(!cbuff->isEmpty()){
						externalfunction(cbuff->getPointer(), cbuff->getChunkSizeBytes(), 
							false, false, userdata);
					} 
					externalfunction(NULL,0,false,true,userdata);
					previous_state = silence;
				} else {
					cbuff->pass();
				}
			} else {
				if(previous_state == silence){
					cbuff->backward(START_OF_SPEECH_BACKWARD_OFFSET);
					externalfunction(cbuff->getPointer(), cbuff->getChunkSizeBytes(), 
						true, false, userdata);
				}
				
				while(!cbuff->isEmpty()){
					externalfunction(cbuff->getPointer(), cbuff->getChunkSizeBytes(), 
						false, false, userdata);
				}
				previous_state = speech;
			}
		}
	Pa_Sleep(90);
	}
}

//=== INITIALIZATION FUNCTION ==================================================
void VADClass::vadinit(void) {

		
	vadterminate();
	vadmachine = new VADFSMachine(vmachine_thresholdoffset, cntSpeech, cntSil, 
		aTemporalConst);

	if(!isFromFile){
		// If the audio comes from microphone, then doa_histogram limits
		// must be computed with the global parameters
		DOA_HISTOGRAM_MAX_LAG = (int) round(GLOBAL_MIC_DISTANCE/GLOBAL_SPEED_OF_SOUND*
		GLOBAL_SAMPLE_RATE)-1;
		DOA_HISTOGRAM_TOT_NUM = 2*DOA_HISTOGRAM_MAX_LAG+1;
		ROS_WARN("Re-defined global macro DOA_HISTOGRAM_MAX_LAG as %d and \
DOA_HISTOGRAM_TOT_NUM as %d",DOA_HISTOGRAM_MAX_LAG,DOA_HISTOGRAM_TOT_NUM);
		direction_of_arrival_histogram = new int[DOA_HISTOGRAM_TOT_NUM];
		lags = new int[DOA_HISTOGRAM_TOT_NUM]; 
		reset_doa_vars();

		// If the audio comes from microphone, then the buffer must have the given
		// parameters
		cbuff = new CircularBuffer(num_channels, sample_rate, cbuffer_duration, 
			chunk_duration); 
		ROS_DEBUG("STARTUP THE VADMACHINE LOOKUP THREAD");
		std::thread t(&VADClass::vadmachine_lookup_thread,this);
		t.detach();

		// SETUP PORTAUDIO
		PaStreamParameters inputParameters;
		err = paNoError;
		// STARTUP PORTAUDIO
		err = Pa_Initialize(); // Initializing PortAudio API
		if (err != paNoError) 
			ROS_ERROR("Error initializing portaudio");

		// Find desired device
		int deviceIndex = -1;
		int deviceCount = Pa_GetDeviceCount();

		if(!USE_DEFAULT_DEVICE_FLAG){
			for (int i = 0; i < deviceCount; i++) {
			const PaDeviceInfo * deviceInfo = Pa_GetDeviceInfo(i);
				std::string deviceName = (std::string) deviceInfo->name;
				if(deviceName.find(DEVICE_NAME) != std::string::npos){
					// Shows the selected device
					deviceIndex = i;
					ROS_DEBUG("index: %d\n", 
						deviceIndex);
					ROS_DEBUG("name: %s\n", 
						deviceInfo->name);
					ROS_DEBUG("Host API: %s\n", 
						Pa_GetHostApiInfo(deviceInfo->hostApi)->name);
					ROS_DEBUG("Max inputs = %d", 
						deviceInfo->maxInputChannels);
					ROS_DEBUG("Max outputs = %d\n", 
						deviceInfo->maxOutputChannels);
					ROS_DEBUG("Default low input latency: %8.4f\n", 
						deviceInfo->defaultLowInputLatency);
					ROS_DEBUG("Default low output latency: %8.4f\n", 
						deviceInfo->defaultLowOutputLatency);
					ROS_DEBUG("Default high input latency: %8.4f\n", 
						deviceInfo->defaultHighInputLatency);
					ROS_DEBUG("Default high output latency: %8.4f\n", 
						deviceInfo->defaultHighOutputLatency);
					ROS_DEBUG("Selected device: %s", deviceInfo->name);
				}
			}
			inputParameters.device = deviceIndex;
		} else {
			inputParameters.device = Pa_GetDefaultInputDevice();
		}

		if (inputParameters.device == paNoDevice) {
			inputParameters.device = Pa_GetDefaultInputDevice();
			ROS_ERROR("Audio input device not found. Default device Selected");
		}

		inputParameters.channelCount = num_channels;
		inputParameters.sampleFormat = paInt16;
		inputParameters.suggestedLatency = 
			Pa_GetDeviceInfo(inputParameters.device)->defaultLowInputLatency;
		inputParameters.hostApiSpecificStreamInfo = NULL;

		PaCallbackPointers * structures = new PaCallbackPointers();
		structures->cbuff = cbuff;
		structures->vadmachine = vadmachine;

		// Open PortAudio Stream
		ROS_DEBUG("Portaudio sample_rate and framesPerBuffer: %d , %d",
			sample_rate, cbuff->getChunkSizeFrames());
	
		// setup stream
		err = Pa_OpenStream(&stream, &inputParameters, NULL, sample_rate, 
		(unsigned long) cbuff->getChunkSizeFrames(), paClipOff, 
		portAudioCallback , structures); 
		
		if (err != paNoError){
			ROS_ERROR("Error opening the PortAudio Stream. Please retry.");
			exit(0);
		}
		// Finally initialize portaudio stream
		err = Pa_StartStream(stream);
		if (err != paNoError){ 
			ROS_ERROR("Unable to start the stream.");
			exit(0);
		}
		ROS_INFO("Portaudio Initialized");
	
	} else {
		// READ FROM FILE
		// This routine will read a file chunk by chunk and publish it into the 
		// circularbuffer without pre-processing the audio.
		// Please use files with sample rate 16kHz and mono. 
		// The software will pre-process before sending to google
		// The audio must contain silence in the begining 

		// If the audio comes from a file, then the buffer and remaining
		// functions must use the same parameters:
		// GLOBAL_SAMPLE_RATE must be re-written
		// GLOLAL_NUMBER_OF_CHANNELS must be re-written
		
  		FILE * wavfileptr = fopen(filepath, "rb");
  		if (wavfileptr == NULL)
			ROS_ERROR("Error loading the file. Path: %s" ,filepath);
		wavHeader *wavhdr = new wavHeader();
		
		VADClass::readWavHeader(wavhdr, wavfileptr);
		short *audiodata = (short *)malloc(wavhdr->datasize);
		fread(audiodata, wavhdr->datasize, 1, wavfileptr);
		fclose(wavfileptr);

		if(wavhdr->fs != 16000){
			if(wavhdr->fs != 48000)
				ROS_ERROR("The given file IS NOT at 16kHz nor 48kHz SAMPLE RATE");
		}

		// Update macros values
		GLOBAL_NUMBER_OF_CHANNELS = wavhdr->nchan;
		GLOBAL_SAMPLE_RATE = wavhdr->fs;

		ROS_WARN("Re-defined global macro GLOBAL_NUMBER_OF_CHANNELS as %d and \
GLOBAL_SAMPLE_RATE as %d", (int)GLOBAL_NUMBER_OF_CHANNELS, 
			(int)GLOBAL_SAMPLE_RATE);

		// If the audio comes from a file, then doa_histogram limits
		// must be computed with the new global parameters
		DOA_HISTOGRAM_MAX_LAG = (int) round(GLOBAL_MIC_DISTANCE/GLOBAL_SPEED_OF_SOUND*
		GLOBAL_SAMPLE_RATE)-1;
		DOA_HISTOGRAM_TOT_NUM = 2*DOA_HISTOGRAM_MAX_LAG+1;
		ROS_WARN("Re-defined global macro DOA_HISTOGRAM_MAX_LAG as %d and \
DOA_HISTOGRAM_TOT_NUM as %d",DOA_HISTOGRAM_MAX_LAG,DOA_HISTOGRAM_TOT_NUM);
		direction_of_arrival_histogram = new int[DOA_HISTOGRAM_TOT_NUM];
		lags = new int[DOA_HISTOGRAM_TOT_NUM]; 
		reset_doa_vars();


		cbuff = new CircularBuffer(GLOBAL_NUMBER_OF_CHANNELS, GLOBAL_SAMPLE_RATE,
			cbuffer_duration, chunk_duration); 

		ROS_DEBUG("STARTUP THE VADMACHINE LOOKUP THREAD");
		std::thread t(&VADClass::vadmachine_lookup_thread,this);
		t.detach();

		int chunk_size_bytes = cbuff->getChunkSizeBytes();
		int num_chunks = (wavhdr->datasize) / chunk_size_bytes;
		// int a = cbuff->getChunkSizeFrames();

		float result;	int itreadchunk = 0;
		int Cbuff_frameLenSamp = cbuff->getChunkSizeSamples();
		while (itreadchunk < num_chunks) {
			short * inbuff = &audiodata[itreadchunk*Cbuff_frameLenSamp];
			result = cbuff->put(inbuff);
			vadmachine->updateStatus(result);	
			itreadchunk++;

			Pa_Sleep(100);
		}
	}
}

//==============================================================================
void VADClass::stopStream(void){
	if(Pa_IsStreamActive(stream) == 1){
		err = Pa_StopStream(stream); //Close stream
		if (err != paNoError){
			ROS_ERROR("Unable to stop the stream.");
			exit(0);
		} else {
			ROS_INFO("Stream stopped");
		}
	}
}

//==============================================================================		
void VADClass::resumeStream(void){
	if(Pa_IsStreamStopped(stream) == 1){
		err = Pa_StartStream(stream);
		if (err != paNoError){
		 	ROS_ERROR("Unable to start the stream.");
			exit(0);
		} else {
			ROS_INFO("Stream restarted");
		}
	}
}

//==============================================================================
void VADClass::vadterminate(void) {
	if(stream != NULL){
		err = Pa_CloseStream(stream); //Close stream
		if (err != paNoError) {
			ROS_ERROR("Error closing the stream.");
			exit(0);
		} else {
			ROS_INFO("Stream was successfully closed");
		}
		Pa_Terminate(); //Terminate Portaudio
	}
}

//==============================================================================
void VADClass::vadMachineAdjustParameters(float thOffset_new, int cntSpeek_new){
	vadmachine->setThreshold(thOffset_new);
	vadmachine->setSpeechCounter(cntSpeek_new);
}

//==============================================================================
// Replace the high level callback function
void VADClass::setusercallback(usercb * usercallback_new){
	externalfunction = usercallback_new;
}

//=== PORTAUDIO CALLBACK FUNCTION ==============================================
int VADClass::portAudioCallback(const void *inputBuffer, void *outputBuffer, 
	unsigned long framesPerBuffer, 	const PaStreamCallbackTimeInfo* timeInfo,	
	PaStreamCallbackFlags statusFlags, void * datastructs){ 
	// Receives the audio from the microphones (in chunks), puts each chunk into 
	//the circular buffer and updates the finite state machine status
	PaCallbackPointers * structures = (PaCallbackPointers *)datastructs;
	CircularBuffer * cbuff = structures->cbuff;
	VADFSMachine * vadmachine = structures->vadmachine;
	
	float proceed = cbuff->put((short *)inputBuffer);
	if (proceed != 0) {
		vadmachine->updateStatus(proceed);
		return paContinue;
	}
	ROS_WARN("The internal circular buffer is full!");
	ROS_WARN("This may be caused by the non-consummation of the audio in buffer");
	return paContinue;
}

//==============================================================================
void VADClass::readWavHeader(wavHeader *wavhdr, FILE *fi){
// Reads the WAV header considering the following restrictions
// -format tag must be 1=PCM (no encoding)
// -<data chunk> must come before data (data chunks aren't expected after'data')
// Yields a pointer to the beginning of the data
	char *tag = (char *)wavhdr;
	fread(wavhdr, 34, 1, fi); //tag inicial tem de ser "RIFF"
	if (tag[0] != 'R' || tag[1] != 'I' || tag[2] != 'F' || tag[3] != 'F')
	{
		fclose(fi);
		ROS_ERROR("NO 'RIFF'.");
	}
	if (wavhdr->fmtTag != 1)
	{
		fclose(fi);
		ROS_ERROR("WAV file data are encoded or WAVEFORMATEXTENSIBLE.");
	}
	if (wavhdr->fmtSize == 14)
		wavhdr->bps = 16;
	if (wavhdr->fmtSize >= 16)
		fread(&wavhdr->bps, 2, 1, fi);
	if (wavhdr->fmtSize == 18) {
		short lixo;
		fread(&lixo, 2, 1, fi);
	}

	tag += 36; //aponta para wavhdr->data
	fread(tag, 4, 1, fi); //data chunk deve estar aqui.
	while (tag[0] != 'd' || tag[1] != 'a' || tag[2] != 't' || tag[3] != 'a')
	{	//tenta encontrar o data chunk mais à frente (sempre em múltilos de 4)
		fread(tag, 4, 1, fi);
		if (ftell(fi) >= long(wavhdr->RIFFsize)) {
			fclose(fi);
			ROS_ERROR("Bad WAV header!");
		}
	}
	fread(&wavhdr->datasize, 4, 1, fi); //data size
	//Assumes that the header ends here and data starts here until the EOF
}




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

//==============================================================================
// Constructor
//==============================================================================

//== Default  ==================================================================
CircularBuffer::CircularBuffer(int chan, int fs, int buff_duration,
	float chunk_duration) {
	init(chan, fs, buff_duration, chunk_duration);
}

//== init  ========================================================================
void CircularBuffer::init(int chan, int fsamp, int buffer_duration,
	float chunk_duration) {
	nchan = chan;
	fs = fsamp;
	read_index = write_index = 0;
	is_full = false;
	is_empty = true;
	num_chunks_buff = int(buffer_duration / chunk_duration + 0.5);
	chunk_size_frames = int(chunk_duration*fs);
	chunk_size_samples = chunk_size_frames * nchan;
	chunk_size_bytes = chunk_size_samples * sizeof(short);
	data_buffer = (short *)malloc(num_chunks_buff*chunk_size_bytes);
	if (data_buffer == NULL){
		ROS_ERROR("Error allocating the memeory. No memory.");
		exit(0);
	}
}

//==============================================================================
float CircularBuffer::put(short *chunk) {
	//Put a chunk of audio data in the circular buffer. Update the write pointer
	float result = 0;
	if (!is_full) {
		//buffer is of samples
		memcpy(&data_buffer[write_index*chunk_size_samples], chunk, chunk_size_bytes);
		result = computeRMS(write_index);
		//printf("rms: %f\n",result);
		write_index++;
		is_empty = false;
		if (write_index == num_chunks_buff)
			write_index = 0;
		if (write_index == read_index) {
			is_full = true;
		}
		return result;
		//return true;
	}
	
	return result;
}

//==============================================================================
bool CircularBuffer::get(void * dest) {
	// Get a chunk of audio data from the buffer and copy it to dest.
	// User must allocate chunk_size_bytes bytes for the chunk.
	if (!is_empty) {
		memcpy(dest, &data_buffer[read_index*chunk_size_samples], chunk_size_bytes);
		read_index++;
		if (read_index == num_chunks_buff)
			read_index = 0;
		if (read_index == write_index) {
			is_empty = true;
		}
		is_full = false;
		return true;
	}
	else {
		// the buffer is empty
		return false; 
	}
}
//==============================================================================
short * CircularBuffer::getPointer(void) {
	// Get a chunk of audio data from the buffer and return it.
	if (!is_empty) {
		// samples because it's a short *
		short * result = &(data_buffer[read_index*chunk_size_samples]); 
		read_index++;
		if (read_index == num_chunks_buff)
			read_index = 0;
		if (read_index == write_index) {
			is_empty = true;
		}
		is_full = false;
		return result;
	}
	else {
		//the buffer is empty
		return getPointer(); 
	}
}

//==============================================================================
short * CircularBuffer::getPointerNoIncrement(void) {
	//samples because it's a short *		
	return &(data_buffer[read_index*chunk_size_samples]); 
}


//==============================================================================
void CircularBuffer::backward(int n) {
//Moves the read index n positions back.
	if (read_index < n) {
		read_index = num_chunks_buff - (n - read_index);
	} else {
		read_index = read_index - n;
	}
}

//==============================================================================
void CircularBuffer::pass(void) {
	// Ignores the current chunk advancing the read index one position
	// (noise or sil, not speech) 
	read_index++;
	if (read_index == num_chunks_buff)
		read_index = 0;
	if (read_index == write_index) {
		is_empty = true;
	}
}

//==============================================================================
bool CircularBuffer::isEmpty(void) {
	return is_empty;
}
//==============================================================================
bool CircularBuffer::isFull(void) {
	return is_full;
}

//==============================================================================
int CircularBuffer::getBufferSizeBytes(void) {
	return num_chunks_buff*chunk_size_bytes; //sizeof(data_buffer);
}

//==============================================================================
int CircularBuffer::getChunkSizeBytes(void) {
	// return num of bytes per chunk
	return chunk_size_bytes;
}

//==============================================================================
int CircularBuffer::getChunkSizeSamples(void) {
	// return num of samples per chunk
	return chunk_size_samples;
}

//==============================================================================
int CircularBuffer::getChunkSizeFrames(void) {
	// return num of frames per chunk
	return chunk_size_frames;
}

//==============================================================================
int CircularBuffer::getFrameSizeBytes(void) {
	return nchan * sizeof(short);
}
//==============================================================================
int CircularBuffer::getNumOfChunks(void) {
	return num_chunks_buff;
}
//==============================================================================
int CircularBuffer::getReadIndex(void){
	return read_index;
}
//==============================================================================
int CircularBuffer::getNumOfFilledChunks(void)
{
	if (is_empty)
		return 0;
	if (is_full) 
		return num_chunks_buff;
	if (read_index > write_index) 
		return (num_chunks_buff - read_index + write_index);
	return (write_index - read_index);
}

//==============================================================================
float CircularBuffer::computeRMS(int index) {
	//in direct form II filter.
	static float w = 0; 
	float xn, yn;
	float E = 0;
	short * data = &data_buffer[index*chunk_size_samples];
	if (nchan == 1) {
		for (int n = 0; n < chunk_size_frames; n++) {
			//convert short to float << chunk
			xn = data[n]; 
			//weighting filter output
			yn = A_WEIGHTING_FILTER_CONSTANT_G*xn - A_WEIGHTING_FILTER_CONSTANT_H*w; 
			//sum of squares.
			E += yn*yn; 
			//unit delay update
			w = xn + A_WEIGHTING_FILTER_CONSTANT_A*w; 
		}
	}
	else {
		for (int n = 0; n < chunk_size_frames; n++) {
			//convert short to float
			xn = data[n*nchan]; 
			//weighting filter output
			yn = A_WEIGHTING_FILTER_CONSTANT_G*xn - A_WEIGHTING_FILTER_CONSTANT_H*w; 
			//sum of squares.
			E += yn*yn; 
			//unit delay update
			w = xn + A_WEIGHTING_FILTER_CONSTANT_A*w; 
		}
	}
	//only 1 channel; weighted RMS in dB.
	return 10.0F*log10(E / chunk_size_frames); 
}

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

//==============================================================================
// Constructor
//==============================================================================
//==============================================================================
//== Length  & thOffset & counterSpeech & counterSil & a factor ================
VADFSMachine::VADFSMachine(float setthOffset, int setcounterSpeech, 
	int setcounterSil, float setaval){
	init(setthOffset, setcounterSpeech, setcounterSil, setaval);
}

//== init ====================
void VADFSMachine::init(float setthOffset, int setcounterSpeech, 
	int setcounterSil, float setaval){
	timeOutInternalCounter =  DEFAULT_VAD_INTERNAL_TIMEOUT_COUNTER;
	rstTimeOutInternalCounter = DEFAULT_VAD_INTERNAL_TIMEOUT_COUNTER;
	thOffset = setthOffset;
	counterSpeech = rstCounterSpeech = setcounterSpeech;
	counterSilence = rstCounterSilence = setcounterSil;
	status = STsilence;
	inSpeech = false;
	threshold = 0;
	a = setaval; a1 = 1 - a;
	ymin_prev = 0;
	ymax_prev = 0;
	ymed_prev = 0;
	ener_prev = 0;
	nBackward = rstCounterSpeech+3;
}

//==============================================================================
void VADFSMachine::updateStatus(float energy) {
	// compute max
	float ymax;
	float ymin;
	float ymed;
	if (firstTime) {
		ymin_prev = energy;
		ymax_prev = energy;
		ymed_prev = energy;
		ener_prev = energy;
		firstTime = false;
	}
	

	if (energy > ymax_prev){
		ymax = energy;
	} else {
		ymax = a*ymax_prev + a1*ymed_prev;
	}

	if (energy < ymin_prev){
		ymin = energy;
	} else {
		ymin = a*ymin_prev + a1*ymed_prev;
	}
	
	ymed = (ymin+ymax)/2.0F;

	// Startup the machine
	switch (status) {
	case STsilence:
		if (energy > ymed_prev + thOffset) {
			status = STpossible_speech; //------------------------------------ NEXT STATE
			threshold = ymed_prev + thOffset;
			counterSpeech = rstCounterSpeech - 1;
		}
		break;

	case STpossible_speech:
		counterSpeech--;
		if (energy > threshold && energy > ymed) {
			if (counterSpeech  <= 0) {
				counterSpeech = rstCounterSpeech;
				status = STspeech; //------------------------------------ NEXT STATE
				timeOutInternalCounter = rstTimeOutInternalCounter-rstCounterSpeech;
			} else {
				status = STpossible_speech;
			}
		} else {
			status = STsilence; //next state
		}
		break;

	case STspeech:
		if (energy < ymed) {
			status = STpossible_silence;  //---------------------------------- NEXT STATE
			threshold = ymed; //new threshold to decide silence
			counterSilence = rstCounterSilence-1;
		} else {
			status = STspeech;
		}
		break;

	case STpossible_silence:
		counterSilence--;
		if (energy > ymed) {
			status = STspeech;
			counterSilence = rstCounterSilence - 1;
			break;
		}
			
		if (counterSilence == 0) {
			status = STsilence;
			break;
		}

		status = STpossible_silence;
		break;
	default:
		status = STsilence;
	}

	if(status != STsilence){
		timeOutInternalCounter = timeOutInternalCounter-1;
	}

	if(timeOutInternalCounter == 0){
		status = STsilence;
		timeOutInternalCounter = rstTimeOutInternalCounter;
		ROS_WARN("SPEECH IS TAKING MORE TIME THAN EXPECTED.");
		ROS_WARN("END OF SPEECH FLAG TRIGGERED");
	}

	ymin_prev = ymin;
	ymax_prev = ymax;
	ymed_prev = ymed;
	ener_prev = energy;
}


//==============================================================================
bool VADFSMachine::isSpeech() {
	if (status == STspeech || status == STpossible_silence) return true;
	return false;
}

//==============================================================================
int VADFSMachine::getStartOfSpeechOffset(void) {
	return rstCounterSpeech + rstCounterSpeech+START_OF_SPEECH_BACKWARD_OFFSET;
}

//==============================================================================
void VADFSMachine::setThreshold(float thOffset_new){
	thOffset = thOffset_new;
}

//==============================================================================
void VADFSMachine::setSpeechCounter(int cntSpeek_new){
	counterSpeech = rstCounterSpeech = cntSpeek_new;
}
