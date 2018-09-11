//
// Copyright 2011-2012 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <complex>
#include <csignal>

#include <stdexcept>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdlib.h>



#define CLOCK_SETUP_TIME 1


namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}


int create_sock(int sock_port){
    int sockfd;
    struct sockaddr_in dest;

    // create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == -1) {
        perror("socket");
    }
    // fcntl(sockfd, F_SETFL, O_NONBLOCK);

    // initialize structure dest
    bzero(&dest, sizeof(dest));
    dest.sin_family = AF_INET;

    // assgin a port number to socket
    dest.sin_port = htons(sock_port);
    dest.sin_addr.s_addr = htonl(INADDR_ANY);

    // kill "Address already in use" error message
    int tr = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &tr, sizeof(int)) == -1) {
        perror("setsockopt");
    }

    if (bind(sockfd, (struct sockaddr*)&dest, sizeof(dest)) == -1) {
        perror("bind");
    }

    /*
    // make it listen to socket with max 20 connections 
    if (listen(sockfd, 20) == -1) {
        perror("listen");
    }
    */
    return sockfd;

}



int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, file, type, ant, subdev, ref, sync, wirefmt;
    double rate, freq, lo_off, gain, amp, bw, delay, seconds_in_future;
    unsigned int sock_port;
    size_t spb;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("file", po::value<std::string>(&file)->default_value("usrp_tx.dat"), "name of the file to read binary samples from")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("rate", po::value<double>(&rate), "rate of outgoing samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("lo_off", po::value<double>(&lo_off)->default_value(0.0), "LO offset frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("amp", po::value<double>(&amp)->default_value(1.0), "set transmitter digital amplitude: 0 <= AMPL < 1.0")
        ("ant", po::value<std::string>(&ant)->default_value("TX/RX"), "daughterboard antenna selection")
        ("subdev", po::value<std::string>(&subdev), "daughterboard subdevice specification")
        ("bw", po::value<double>(&bw), "daughterboard IF filter bandwidth in Hz")
        ("sync", po::value<std::string>(&sync)->default_value("now"), "synchronization method: now, pps, mimo")
        ("delay", po::value<double>(&delay)->default_value(0.0), "specify a delay between repeated transmission of file (ms)")
        ("secs", po::value<double>(&seconds_in_future)->default_value(2.0), "number of seconds in the future to transmit")
        ("repeat", "repeatedly transmit file")
        ("dilv", "specify to disable inner-loop verbose")
        ("trigger", "specify to start transmit file by a UDP trigger")
        ("port",po::value<unsigned int>(&sock_port)->default_value(8881), "specify the socket port")

    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    size_t chan, chan_num, mboard, mboard_num;

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX samples from file %s") % desc << std::endl;
        return ~0;
    }

    bool repeat = vm.count("repeat");

    bool verbose = vm.count("dilv") == 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    chan_num = usrp->get_tx_num_channels();
    mboard_num = usrp->get_num_mboards();

    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_tx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    std::cout << boost::format("Setting TX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_tx_rate(rate);
    std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;

    //set the center frequency 
    if (not vm.count("freq")){
        std::cerr << "Please specify the center frequency with --freq" << std::endl;
        return ~0;
    } 
    uhd::tune_request_t tune_req(freq, lo_off);
    uhd::tune_result_t tune_ret;

    for (chan = 0; chan < chan_num; chan++){
        std::cout << boost::format("Setting Channel %d:") % chan << std::endl;
        std::cout << boost::format("    Trarget Freq: %f (MHz)") % (tune_req.target_freq /1e6) << std::endl;
        std::cout << boost::format("    RF Freq Policy: %c") % char(tune_req.rf_freq_policy) << std::endl;
        std::cout << boost::format("    RF Freq: %f (MHz)") % (tune_req.rf_freq / 1e6) << std::endl;
        std::cout << boost::format("    DSP Freq Policy: %c") % char(tune_req.dsp_freq_policy) << std::endl;
        std::cout << boost::format("    DSP Freq: %f (MHz)") % (tune_req.dsp_freq / 1e6) << std::endl;
        tune_ret = usrp->set_tx_freq(tune_req, chan);
        std::cout << boost::format("Channel %d %s") % chan % (tune_ret.to_pp_string()) << std::endl;
    }
    std::cout  << std::endl;
    //set the rf gain
    for (chan = 0; chan < chan_num; chan++){
        if (vm.count("gain")){
            std::cout << boost::format("Setting Channel %d TX Gain: %f dB...") % chan % gain << std::endl;
            usrp->set_tx_gain(gain, chan);
        }
        std::cout << boost::format("Actual Channel %d TX Gain: %f dB...") % chan % usrp->get_tx_gain() << std::endl;
    }
    std::cout  << std::endl;
    //set the IF filter bandwidth
    uhd::meta_range_t bw_range;
    for (chan = 0; chan < chan_num; chan++){
        bw_range = usrp->get_tx_bandwidth_range(chan);
        std::cout << boost::format("Tx Bandwidth Range: ");
        std::cout << bw_range.to_pp_string() << std::endl;
        if (vm.count("bw")){
            std::cout << boost::format("Setting Channel %d TX Bandwidth: %f MHz...") % chan % bw << std::endl;
            usrp->set_tx_bandwidth(bw, chan);
        }
        std::cout << boost::format("Actual Channel %d TX Bandwidth: %f MHz...") % chan % (usrp->get_tx_bandwidth()/1e6) << std::endl;
    }
    std::cout << std::endl;

    //set the antenna
    for (chan = 0; chan < chan_num; chan++){
        if (vm.count("ant")){
            usrp->set_tx_antenna(ant, chan );
        }
        std::cout << "Channel " << chan << " Antenna: " << usrp->get_tx_antenna(chan) << std::endl;
    }

 

    //Lock mboard clocks
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl << std::endl;
    if (sync == "now"){
        //This is not a true time lock, the devices will be off by a few RTT.
        //Rather, this is just to allow for demonstration of the code below.
        usrp->set_clock_source("internal");
        usrp->set_time_now(uhd::time_spec_t(0.0));
    }
    else if (sync == "pps"){
       usrp->set_clock_source("external");
       usrp->set_time_source("external");
       usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    }
    else if (sync == "mimo"){
       if (mboard_num == 1){
           //make mboard 0 a slave over the MIMO Cable
           usrp->set_clock_source("mimo", 0);
           usrp->set_time_source("mimo", 0);
       }
       else if (mboard_num == 2){
           //make mboard 0 a master over the MIMO Cable
           usrp->set_clock_source("internal",0);
           //make mboard 1 a slave over the MIMO Cable
           usrp->set_clock_source("mimo", 1);
           usrp->set_time_source("mimo", 1);
           //set time on the master (mboard 0)
           usrp->set_time_now(uhd::time_spec_t(0.0), 0);
       }else {
           std::cout << "Specified to sync with MIMO, but number of mboard is: " << mboard_num << std::endl;
       }
    }
    std::cout << "Wait for setup complete" << std::endl;
    boost::this_thread::sleep(boost::posix_time::seconds(CLOCK_SETUP_TIME)); //allow for some setup time

    //Check Ref and LO Lock detect
    std::vector<std::string> sensor_names;
    for (chan = 0; chan < chan_num; chan++){
        sensor_names = usrp->get_tx_sensor_names(chan);
        if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
            uhd::sensor_value_t lo_locked = usrp->get_tx_sensor("lo_locked",chan);
            std::cout << boost::format("Checking TX channel %d: %s ...") % chan % lo_locked.to_pp_string() << std::endl;
            UHD_ASSERT_THROW(lo_locked.to_bool());
        }
    }
                           
    for (mboard = 0; mboard < mboard_num; mboard++){
        sensor_names = usrp->get_mboard_sensor_names(mboard);
        if ((sync == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
            uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked",mboard);
            std::cout << boost::format("Checking TX mboard %d: %s ...") % mboard % mimo_locked.to_pp_string() << std::endl;
            UHD_ASSERT_THROW(mimo_locked.to_bool());
        }
        if ((sync == "pps") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
            uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked",mboard);
            std::cout << boost::format("Checking TX mboard %d: %s ...") % mboard % ref_locked.to_pp_string() << std::endl;
            UHD_ASSERT_THROW(ref_locked.to_bool());
        }
    }

    std::cout << std::endl;

    // set up handler
    std::signal(SIGINT, &sig_int_handler);

    //create a transmit streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32"); //complex floats
    for (chan = 0; chan < chan_num; chan++)
        stream_args.channels.push_back(chan); //linear mapping
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);


    // allocate buffers to receive with samples (one buffer per channel)
    std::vector< std::vector< std::complex<float> > > buffs(chan_num, std::vector< std::complex<float> > (spb));

    //create a vector of pointers to point to each of the channel buffers
    std::vector<std::complex<float> *> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++){
        buff_ptrs.push_back(&buffs[i].front());
    }

    // setup inputfiles    
    std::vector<std::ifstream *> infile(chan_num);
    if (chan_num > 1){
        std::string name, type, fname;
        char * cstr = new char[file.size()+1];
        strcpy(cstr, file.c_str());
        name = strtok(cstr, ".");
        type = strtok(NULL, ".");
        for (chan = 0; chan < chan_num; chan++){
            fname = name+"_"+boost::to_string(chan+1)+"."+type;
            std::cout << boost::format("Transmit from file: %s") % fname << std::endl << std::endl;
            infile[chan] = new std::ifstream(fname.c_str(), std::ifstream::binary);
        }
    }
    else {
        std::cout << boost::format("Transmit from file: %s") % file << std::endl << std::endl;
        infile[0] = new std::ifstream(file.c_str(), std::ifstream::binary);
    }


    // set up socket to wait for the trigger from the receiver
    int sockfd;
    struct sockaddr_in client_addr;
    unsigned int addrlen = sizeof(client_addr);
    char buf[255];
    int ret;

    if (vm.count("trigger")) {
        std::cout << "Wait for receiver..." << std::endl;
        sockfd = create_sock(sock_port);    
    }


    //set sigint if user wants to receive
    if(repeat){
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }

    //setup metadata for the first packet
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    md.has_time_spec = seconds_in_future > 0;
    md.time_spec = uhd::time_spec_t(seconds_in_future);




    //the first call to send() will block this many seconds before sending:
    const double timeout = seconds_in_future + 0.1; //timeout (delay before transmit + padding)

    //send from file
    do {
        // reset input file status
        for (chan = 0; chan < chan_num; chan++){
            infile[chan]->clear();
            // std::cout << "File " << chan << " at " << infile[chan]->tellg() <<  std::endl;
            infile[chan]->seekg(0, std::ifstream::beg);
            // std::cout << "Set file " << chan << " to " << infile[chan]->tellg() <<  std::endl;
        }

        // wait for trigger
        if (vm.count("trigger")) {
            std::cout << "Wait for trigger..." << std::endl;
            if ((ret = recvfrom(sockfd, buf, sizeof(buf), 0, (struct sockaddr *)&client_addr, &addrlen)) < 0) {
                perror("recvfrom");
            }
            // std::cout << "Received from socked, length: " << ret << std::endl;
            if (sync == "pps"){
                usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
            }
            else{
                usrp->set_time_now(uhd::time_spec_t(0.0), 0);
            }

            boost::this_thread::sleep(boost::posix_time::seconds(0.3)); //allow for some setup time
        }

        //loop until the entire file has been read
        size_t samps_to_send = 0;

        bool end_of_file = 0;

        while(not end_of_file and not stop_signal_called){
            samps_to_send = spb;
            for (chan = 0; chan < chan_num; chan++){
                infile[chan]->read((char*)buff_ptrs[chan], spb*sizeof(std::complex<float>));
                if (samps_to_send > infile[chan]->gcount()/sizeof(std::complex<float>)){
                    samps_to_send = infile[chan]->gcount()/sizeof(std::complex<float>);
                }
                if (infile[chan]->eof()){
                    end_of_file = true;
                }
                // scale the power of transmitted signal
                for (size_t i = 0; i<samps_to_send; i++){
                    buff_ptrs[chan][i] *= amp;
                }            
            }

            
            //send a single packet
            size_t num_tx_samps = tx_stream->send(
                buff_ptrs, samps_to_send, md, timeout
            );
            //do not use time spec for subsequent packets
            md.has_time_spec = false;

            if (num_tx_samps < samps_to_send) std::cerr << boost::format("Send timeout: %d/%d samples send") % num_tx_samps % samps_to_send << std::endl;
            if(verbose) std::cout << boost::format("Sent packet: %u samples") % num_tx_samps << std::endl;
        }


        if(repeat and delay != 0.0) boost::this_thread::sleep(boost::posix_time::milliseconds(delay));
        if (not vm.count("trigger")) {
            seconds_in_future = 0;
        }


        std::cout << '.';
        std::cout.flush();

    } while(repeat and not stop_signal_called);
    
    md.end_of_burst = true;
        tx_stream->send(buff_ptrs, 0, md, timeout);
        
    for (chan = 0; chan < chan_num; chan++){
        infile[chan]->close();
    }
    // wait to finish sending
    boost::this_thread::sleep(boost::posix_time::milliseconds(200)); //allow for some setup time
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
