BLUETEXT='\033[0;34m'
NC='\033[0m'

SOURCEPATH=$(pwd)
echo $SOURCEPATH

# Install portaudio 
printf "${BLUETEXT}Installing Portaudio and dependecies${NC}\n"
sudo apt-get install libasound-dev
wget -O pafile.tgz "http://portaudio.com/archives/pa_stable_v190600_20161030.tgz" 
tar -xvzf pafile.tgz
cd portaudio
./configure
sudo make install
rm pafile.tgz
rm -rf portaudio

# Move to correct folder
printf "${BLUETEXT}Creating speech folder${NC}\n"
cd ~
mkdir speech
cd speech
printf "${BLUETEXT}...done.${NC}\n"

# Create complementary folders and files
printf "${BLUETEXT}Creating complementary folders${NC}\n"
mkdir recordings
printf "${BLUETEXT}...done.${NC}\n"

#Install dependencies
printf "\n"
printf "${BLUETEXT}Installing dependencies${NC}\n"
sudo apt-get install build-essential autoconf libtool exo-utils libgflags-dev libgtest-dev clang libc++-dev libgsl0-dev

# Install Google Cloud
export CLOUD_SDK_REPO="cloud-sdk-$(lsb_release -c -s)"
echo "deb http://packages.cloud.google.com/apt $CLOUD_SDK_REPO main" | sudo tee -a /etc/apt/sources.list.d/google-cloud-sdk.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update && sudo apt-get install google-cloud-sdk
printf "${BLUETEXT}...done.${NC}\n"

# Clone stuff
printf "\n"
printf "${BLUETEXT}Getting code from repositories${NC}\n"
git clone https://github.com/grpc/grpc.git
git clone https://github.com/googleapis/googleapis.git
cd grpc
git submodule update --init
cd ..
printf "${BLUETEXT}...done.${NC}\n"

# Make grpc and protobuf
printf "\n"
printf "${BLUETEXT}Building and installing grpc and protobuf${NC}\n"
cd grpc
make
sudo make install
cd third_party/protobuf
./autogen.sh
./configure
make
sudo make install
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
cd ~/speech
printf "${BLUETEXT}...done.${NC}\n"

# Make googleapis
printf "\n"
printf "${BLUETEXT}Building googleapis${NC}\n"
export LANGUAGE=cpp
cd googleapis
make all
cd ..
printf "${BLUETEXT}...done.${NC}\n"

# Make googleapis ar file
export GOOGLEAPIS_GENS_PATH=$HOME/speech/googleapis/gens
mkdir ar_file
cd ar_file
cp $SOURCEPATH/make_googleapis_ar.make ./
make -f make_googleapis_ar.make
cd ..

# Add google credentials to bashrc
export GOOGLE_APPLICATION_CREDENTIALS=$HOME/speech/gcsapi_user_credentials.json
echo 'export GOOGLE_APPLICATION_CREDENTIALS=$HOME/speech/gcsapi_user_credentials.json' >> $HOME/.bashrc
