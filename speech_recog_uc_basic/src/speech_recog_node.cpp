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


/*
Main program to Speech Recognition.
This software contains:
* VADClass: a class that manages the audio acquisition, storage and processes it
 in order to determine 
if the audio contains speech and when does it start and ends. 
This class implements:
		- PortAudio or ReadFromFile: a function that reads audio from a file or 
		from a microphone
		- VADFSMachine: finite state machine responsible to determine if the audio 
		is speech or not speech
		- CircularBuffer: a cicrcular buffer that stores the audio 
* speech_callback function: a callback function that handles the
 audio that contains speech, sending it to the recognizer,
though google cloud speech api
*/

// Includes 
#include "speech_node_classes.h"



// Namespaces
using google::cloud::speech::v1::RecognitionConfig;
using google::cloud::speech::v1::Speech;
using google::cloud::speech::v1::StreamingRecognizeRequest;
using google::cloud::speech::v1::StreamingRecognizeResponse;

// New types
typedef google::cloud::speech::v1::StreamingRecognizeRequest writer;
typedef google::cloud::speech::v1::StreamingRecognizeResponse reader;


/*
Structure to contain the pointers needed during the recognition
This pointers are needed to send data to google and retrieve the response
while the VADClass returns speech audio.

In other words, once the callback dies, its vars also dies, so, it's 
necessary to store the information about the streamer and the context of a given
audio-speech (from the start of speech untid the end of speech). 
Those pointers are initialized on Start Of Speech, creating a new comu. channel.
Then, they are stored in recog_pointers_data_structures until the end of speech.
At the end of speech, the objects are destoyed.

std::unique_ptr deletion is implicit when a new value is assigned to the var
*/

struct recog_pointers_data_structures {
	std::unique_ptr<Speech::Stub> speech_p;
	std::unique_ptr<grpc::ClientReaderWriter<writer,reader>> streamer;
	grpc::ClientContext *context_ptr;
	char *lang;
	std::thread m_thread;
	FILE * file;
	wavHeader * header_for_test;

};




// Globals
VADClass * vc;
std::string home(getenv("HOME"));
std::string maindir = home + "/speech";

// Global ROS Stuff
ros::Publisher * words_pub;
ros::Publisher * sentences_pub;

/*
The worker of the response handler thread
Handles the google's response and publishes it in the correspondent ros topic
*/
static void gResponseHandlerThread(
	std::unique_ptr<grpc::ClientReaderWriter<writer,reader>>* streamer_ref ){
	grpc::ClientReaderWriter<writer,reader> * streamer = streamer_ref->get();
	StreamingRecognizeResponse response;
	speech_recog_uc_basic::SpeechResult interim_sentence;
	try{
		while(streamer->Read(&response)) {	//blocking function
			auto result = response.results(0);
			auto alternative = result.alternatives(0);
			if(result.stability() > 0){ 
				std::cout << "interim results: "<< alternative.confidence() 
				<< " " << alternative.transcript() << std::endl;
				interim_sentence.result = alternative.transcript();
				interim_sentence.confidence = alternative.confidence();
				words_pub->publish(interim_sentence);
			}
		}
		auto result = response.results(0);
		auto alternative = result.alternatives(0);


		speech_recog_uc_basic::SpeechResult sentence;

		sentence.result = alternative.transcript();
		sentence.confidence = alternative.confidence();
		sentences_pub->publish(sentence);
		std::cout << "interim results: "<< alternative.confidence() 
		<< " " << alternative.transcript() << std::endl;
		streamer_ref->reset();
			
	} catch (...){
		ROS_WARN("Some error occured with the transcription, please retry");
		// Returning an empty message to the DM
		speech_recog_uc_basic::SpeechResult bad_transcription;
		bad_transcription.result = "";
		bad_transcription.confidence = 0;
		sentences_pub->publish(bad_transcription);
	 }
}


/*
Callback that handles the speech audio.
When a new chunk of speech audio arrives to the VAD it is detected as speech
and the speech_callback is evoked.
On the first speech-chunk, i.e., on start of speech, the callback generates
a channel to comunicates with google cloud speech api and stores the pointers
on a google_pointers_data_structure so they can be used after the callback dies
On the last speech-chunk, i.e., at the end of speech, the callback retreives 
the response and deletes all pointers, closing the communication channel
On the meantime, while speech chunks are arriving, they're only sended to google
*/
static void speech_callback(short * data, int readbyteSize, bool isSOS,
	bool isEOS, void * extradata) {
	// callback to put audio data in a raw file
	recog_pointers_data_structures * recog_pointers = 
		(recog_pointers_data_structures *)extradata;
	grpc::Status status;
	bool aa;
	const size_t bytes_read = readbyteSize;
	int array_length = readbyteSize/sizeof(short); //num of shorts in the buffer

	// handling the file writting 

	if (isSOS) {
		/* START OF SPEECH CHUNK */

		ROS_INFO("isSOS ------------------ New recognition started.");
		if(GLOBAL_OUTPUT_MODE){
			char name[100];
			std::chrono::milliseconds ms = 
			std::chrono::duration_cast<std::chrono::milliseconds>(
				std::chrono::system_clock::now().time_since_epoch());
			
			snprintf(name, sizeof(char) * 100, 
				"/recordings/recognition_audio_%ld.wav", ms.count());
			char pathname[200]; strcpy(pathname,maindir.c_str()); strcat(pathname,name);
			ROS_INFO("New file created: %s\n", pathname);
			recog_pointers->file = fopen(pathname, "wb");
			if ((recog_pointers->file) == NULL) perror("");

			recog_pointers->header_for_test = new wavHeader();
			strncpy(recog_pointers->header_for_test->RIFF, "RIFF",4);
			recog_pointers->header_for_test->RIFFsize = 0;
			strncpy(recog_pointers->header_for_test->fmt, "WAVEfmt ",8);
			recog_pointers->header_for_test->fmtSize = 16;
			recog_pointers->header_for_test->fmtTag = 1;
			recog_pointers->header_for_test->nchan = GLOBLAL_NUMBER_OF_CHANNELS; 
			recog_pointers->header_for_test->fs = GLOBAL_SAMPLE_RATE;       
			recog_pointers->header_for_test->avgBps = 
			GLOBAL_SAMPLE_RATE * GLOBLAL_NUMBER_OF_CHANNELS * sizeof(short); 
			recog_pointers->header_for_test->nBlockAlign = 
				GLOBLAL_NUMBER_OF_CHANNELS * sizeof(short);

			// Uncomment the follow lines and comment the 4 lines above to store in 
			// a file what is sent to google
			// recog_pointers->header_for_test->nchan = 1; 
			// recog_pointers->header_for_test->fs = 16000; 
			// recog_pointers->header_for_test->avgBps = 16000 * 1 * sizeof(short);
			// recog_pointers->header_for_test->nBlockAlign = 1 *sizeof(short);

			recog_pointers->header_for_test->bps = 16; 
			strncpy(recog_pointers->header_for_test->data, "data", 4);
			recog_pointers->header_for_test->datasize = 0;

			fwrite(recog_pointers->header_for_test, 
				sizeof(*(recog_pointers->header_for_test)), 1, recog_pointers->file);
		}

		auto creds = grpc::GoogleDefaultCredentials();
		auto channel = grpc::CreateChannel("speech.googleapis.com", creds);
		recog_pointers->speech_p = Speech::NewStub(channel);
								
		StreamingRecognizeRequest request;
		auto* streaming_config_ = request.mutable_streaming_config();
		RecognitionConfig * streaming_config = streaming_config_->mutable_config();
		
		// CONFIGURATIONS// <<<<<<<<<<<<<<<<<<<<<<<<<
		streaming_config->set_language_code(recog_pointers->lang);						 
		streaming_config->set_sample_rate_hertz(GOOGLE_STREAMING_CONFIG_SAMPLE_RATE);
		streaming_config->set_encoding(RecognitionConfig::LINEAR16);
		streaming_config_->set_interim_results(true);

		// Begin a stream.
		recog_pointers->context_ptr = new grpc::ClientContext();
		recog_pointers->streamer = recog_pointers->speech_p->StreamingRecognize(
			recog_pointers->context_ptr);
		recog_pointers->streamer->Write(request);
		recog_pointers->m_thread = std::thread(gResponseHandlerThread, 
			&recog_pointers->streamer);

		StreamingRecognizeRequest new_request;
		new_request.set_audio_content(data,bytes_read); 
		aa = (recog_pointers->streamer.get())->Write(new_request); 

	}

	if(!isEOS){
		if(GLOBAL_OUTPUT_MODE){	
			fwrite(data, readbyteSize, 1, recog_pointers->file);
		} 
		StreamingRecognizeRequest request;
		request.set_audio_content(data,bytes_read);
		aa = (recog_pointers->streamer.get())->Write(request);	
	}

	if(isEOS){
		/* END OF SPEECH CHUNK */		
		ROS_INFO("isEOS ------------------ Recognition stoped.");
		if(GLOBAL_OUTPUT_MODE){
			int fsize = ftell(recog_pointers->file);
			rewind(recog_pointers->file);
			recog_pointers->header_for_test->datasize = 
				fsize - sizeof(*(recog_pointers->header_for_test));
			recog_pointers->header_for_test->RIFFsize = 
				fsize - sizeof(*(recog_pointers->header_for_test)) + 36;
			fwrite(recog_pointers->header_for_test,
				sizeof(*(recog_pointers->header_for_test)), 1, recog_pointers->file);
			fclose(recog_pointers->file);
		}
		// finishes the publication in google's pipe
		aa = (recog_pointers->streamer.get())->WritesDone();
		//If the internet connection fails, the thread will close after a few seconds
		recog_pointers->m_thread.join(); //detach induces a segmentation fault 
		recog_pointers->streamer = NULL;
		delete recog_pointers->context_ptr;
	}
}







int main(int argc, char* argv[]) {
	char * usage = (char *) "\n\nROS Node for Speech Recognition.\nDescription:\n\
	This software contains:  \
	\n* VADClass: a class that manages the audio acquisition, storage and\
	processes it in order to determine if the audio contains speech and when does\
	it start and ends. This class implements:\
	\n\t-PortAudio or ReadFromFile: a \
	function that reads audio from a file or from a microphone\
	\n\t- VADFSMachine:\
	 finite state machine responsible to determine if the audio is speech or not\
	 speech\
	 \n\t- CircularBuffer: a cicrcular buffer that stores the audio  \
	\n* GoogleSpeechRecognizer: the integration of Google Cloud Speech API to\
	 perform speech recognition in the cloud  \
	\nSpecific parameters can be found in GLOBAL_PARAMETERS.h \
	\nTo output a RAW files containing the audio sent for recognition change \
	the flag GLOBAL_OUTPUT_MODE for TRUE ('false' by default)  \
	\n\nUsage:	\
	\n\n rosrun speech_recog_uc_basic speech_recog_uc_basic_node [language]  \
	\n\nExample:	\
	\n rosrun speech_recog_uc_basic speech_recog_uc_basic_node pt-PT";
	
	char * lang = (char *) "en-EN";
	char * read_file_pointer = NULL;
	bool readFromFile = false;
	

	
	// Process language
	if(argc > 1){
		if((std::string) argv[1] == "-h"){
			ROS_INFO("%s",usage);
			return 1;
		} else
			lang = argv[1];
	}

	//Parsing options
	for(int arg = 1; arg < argc; arg++){
		if (argv[arg][0] == '-'){
			char var = argv[arg][1];
			switch(var){
				case 'h':
					ROS_INFO("%s",usage);
					return 1;
					break;
				case 'f':
					readFromFile = true;
					read_file_pointer = argv[arg+1];
					ROS_INFO("Reading audio from file: %s",read_file_pointer);
					break;
			}
		}
	}


	ROS_INFO("Initializing Speech Recognition node");

	// Initialize ROS
	ros::init(argc, argv, "speech_recog_node");
	ros::NodeHandle n;

	words_pub = new ros::Publisher(n.advertise<speech_recog_uc_basic::SpeechResult>(
		"/speech_recog_uc/words", 10));
	sentences_pub = new ros::Publisher(n.advertise<speech_recog_uc_basic::SpeechResult>(
		"/speech_recog_uc/sentences", 10));
	

	ROS_INFO("---------------------------------");
	ROS_INFO("Defined language: %s", lang);		
	ROS_INFO(GLOBAL_OUTPUT_MODE ? "Recognition output mode ON":"Recognition\
		output mode OFF");		
	ROS_INFO("--------------------------------\n");


	recog_pointers_data_structures * user_structure_recog_pointers_container =
		new recog_pointers_data_structures();
	user_structure_recog_pointers_container->speech_p = NULL;
	user_structure_recog_pointers_container->streamer = NULL;
	user_structure_recog_pointers_container->lang = lang;
	user_structure_recog_pointers_container->file = new FILE();
	

	ROS_INFO("Initializing Voice Activity Detection Engine");
	vc = new VADClass(speech_callback, 
		user_structure_recog_pointers_container,
		GLOBAL_SAMPLE_RATE,
		GLOBLAL_NUMBER_OF_CHANNELS,
		GLOBLAL_CHUNK_DURATION_IN_SECONDS,
		GLOBAL_CIRCULAR_BUFFER_DURATION,
		DEFAULT_VAD_ENERGY_THRESHOLD_OFFSET_FOR_LISTENING,
		DEFAULT_VAD_INTERNAL_COUNTER_FOR_LISTENING,
		DEFAULT_VAD_INTERNAL_COUNTER_SIL,
		VAD_WEIGHTING_UPDATE_CONST,
		readFromFile,
		read_file_pointer);

	ROS_INFO("Voice Activity Detection Engine Initialized");

	
	ros::spin();

	vc->vadterminate();
	ROS_INFO("Engine terminated");
	return 0;
}
