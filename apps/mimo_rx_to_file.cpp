//
// Copyright 2010-2011 Ettus Research LLC
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
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>


#include <sys/types.h> 
#include <sys/socket.h>
#include <arpa/inet.h>

#define CLOCK_SETUP_TIME 1


namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}


int create_tx_sock()
{
    int sockfd;

    // create socket
    sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sockfd == -1) {
        perror("socket");
    }

    return sockfd;
}


int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args, file, type, ant, subdev, sync, wirefmt;
    size_t total_num_samps, spb;
    double rate, freq, lo_off, gain, bw;
    double seconds_in_future, interval;
    std::string tx_addrs;
    std::string tx_ports;
    std::vector<std::string> tx_addr;
    std::vector<std::string> tx_port;


    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
        ("file", po::value<std::string>(&file)->default_value("zimo_rx.dat"), "name of the file to write binary samples to")
        ("spb", po::value<size_t>(&spb)->default_value(10000), "samples per buffer")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive")
        ("interval", po::value<double>(&interval)->default_value(0.0), "the interval between receiving nsamps samples (ms)")
        ("secs", po::value<double>(&seconds_in_future)->default_value(2.0), "number of seconds in the future to receive")
        ("rate", po::value<double>(&rate), "rate of incoming samples")
        ("freq", po::value<double>(&freq), "RF center frequency in Hz")
        ("lo_off", po::value<double>(&lo_off)->default_value(0.0), "LO offset frequency in Hz")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
        ("ant", po::value<std::string>(&ant)->default_value("TX/RX"), "daughterboard antenna selection")
        ("subdev", po::value<std::string>(&subdev), "daughterboard subdevice specification")
        ("bw", po::value<double>(&bw), "daughterboard IF filter bandwidth in Hz")
        ("sync", po::value<std::string>(&sync)->default_value("now"), "synchronization method: now, pps, mimo")
        ("dilv", "specify to disable inner-loop verbose")
        ("trigger", "specify to start transmitting by a trigger")
        ("tx_addrs", po::value<std::string>(&tx_addrs), "IP address of tx (seperated by ',')")
        ("tx_ports", po::value<std::string>(&tx_ports), "Socket Port of tx (seperated by ',')")    
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    size_t chan, chan_num, mboard, mboard_num;
    

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    bool verbose = vm.count("dilv") == 0;
    
    chan_num = usrp->get_rx_num_channels();
    mboard_num = usrp->get_num_mboards();
    
    //always select the subdevice first, the channel mapping affects the other settings
    if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //set the sample rate
    if (not vm.count("rate")){
        std::cerr << "Please specify the sample rate with --rate" << std::endl;
        return ~0;
    }
    
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    
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
        tune_ret = usrp->set_rx_freq(tune_req, chan);
        std::cout << boost::format("Channel %d %s") % chan % (tune_ret.to_pp_string()) << std::endl;
    }
    std::cout  << std::endl;
    //set the rf gain
    for (chan = 0; chan < chan_num; chan++){
        if (vm.count("gain")){
            std::cout << boost::format("Setting Channel %d RX Gain: %f dB...") % chan % gain << std::endl;
            usrp->set_rx_gain(gain, chan);
        }
        std::cout << boost::format("Actual Channel %d RX Gain: %f dB...") % chan % usrp->get_rx_gain() << std::endl;
    }
    
    std::cout  << std::endl;
    //set the IF filter bandwidth
    uhd::meta_range_t bw_range;
    for (chan = 0; chan < chan_num; chan++){
        bw_range = usrp->get_rx_bandwidth_range(chan);
        std::cout << boost::format("Rx Bandwidth Range: ");
        std::cout << bw_range.to_pp_string() << std::endl;
        if (vm.count("bw")){
            std::cout << boost::format("Setting Channel %d RX Bandwidth: %f MHz...") % chan % bw << std::endl;
            usrp->set_rx_bandwidth(bw, chan);
        }
        std::cout << boost::format("Actual Channel %d RX Bandwidth: %f MHz...") % chan % (usrp->get_rx_bandwidth(chan)/1e6) << std::endl;
    }
    std::cout  << std::endl;
    //set the antenna
    
    for (chan = 0; chan < chan_num; chan++){
        if (vm.count("ant")){
            usrp->set_rx_antenna(ant, chan);
        }
        std::cout << "Channel " << chan << " Antenna: " << usrp->get_rx_antenna(chan) << std::endl;
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
        sensor_names = usrp->get_rx_sensor_names(chan);
        if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") != sensor_names.end()) {
            uhd::sensor_value_t lo_locked = usrp->get_rx_sensor("lo_locked", chan);
            std::cout << boost::format("Checking RX channel %d: %s ...") % chan % lo_locked.to_pp_string() << std::endl;
            UHD_ASSERT_THROW(lo_locked.to_bool());
        }
    }
    
    for (mboard = 0; mboard < mboard_num; mboard++){
        sensor_names = usrp->get_mboard_sensor_names(mboard);
        if ((sync == "mimo") and (std::find(sensor_names.begin(), sensor_names.end(), "mimo_locked") != sensor_names.end())) {
            uhd::sensor_value_t mimo_locked = usrp->get_mboard_sensor("mimo_locked", mboard);
            std::cout << boost::format("Checking RX mboard %d: %s ...") % mboard % mimo_locked.to_pp_string() << std::endl;
            UHD_ASSERT_THROW(mimo_locked.to_bool());
        }
        if ((sync == "pps") and (std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end())) {
            uhd::sensor_value_t ref_locked = usrp->get_mboard_sensor("ref_locked", mboard);
            std::cout << boost::format("Checking RX mboard %d: %s ...") % mboard % ref_locked.to_pp_string() << std::endl;
            UHD_ASSERT_THROW(ref_locked.to_bool());
        }
    }


    if (total_num_samps == 0){
        std::signal(SIGINT, &sig_int_handler);
        std::cout << "Press Ctrl + C to stop streaming..." << std::endl;
    }



    int num_total_samps = 0;
    int num_requested_samples = total_num_samps;

    //create a receive streamer
    //linearly map channels (index0 = channel0, index1 = channel1, ...)
    uhd::stream_args_t stream_args("fc32"); //complex floats
    for (chan = 0; chan < chan_num; chan++)
        stream_args.channels.push_back(chan); //linear mapping
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    uhd::rx_metadata_t md;


    // allocate buffers to receive with samples (one buffer per channel)
    std::vector<std::vector<std::complex<float> > > buffs(chan_num, std::vector<std::complex<float> > (spb));

    //create a vector of pointers to point to each of the channel buffers
    std::vector<std::complex<float> *> buff_ptrs;
    for (size_t i = 0; i < buffs.size(); i++){
        buff_ptrs.push_back(&buffs[i].front());
    }
    // setup outfiles    
    std::vector<std::ofstream *> outfile(chan_num);
    if (chan_num > 1) {
        std::string name, type, fname;
        char * cstr = new char[file.size()+1];
        strcpy(cstr, file.c_str());
        name = strtok(cstr, ".");
        type = strtok(NULL, ".");
        for (chan = 0; chan < chan_num; chan++){
            fname = name+"_"+boost::to_string(chan+1)+"."+type;
            std::cout << boost::format("Receive to file: %s") % fname << std::endl;
            outfile[chan] = new std::ofstream(fname.c_str(), std::ofstream::binary);
        }
    }
    else{
        std::cout << boost::format("Receive to file: %s") % file << std::endl;
        outfile[0] = new std::ofstream(file.c_str(), std::ofstream::binary);
    }
    
    


    if (vm.count("trigger")) {
        
        boost::split(tx_addr, tx_addrs, boost::is_any_of(","));
        boost::split(tx_port, tx_ports, boost::is_any_of(","));        
        
        if (tx_addr.size() < 1) {
            std::cout << "Error: Specified to use trigger, but the number of tx doesn't equal to the number of tx chan" << std::endl;
            exit(1);
        }
        
        if (tx_addr.size() != tx_port.size()) {
            std::cout << "Error: The number of addr and port should be equal" << std::endl;
            exit(1);
        }
        
        int sockfd;
        struct sockaddr_in tx_dest;
        int addrlen = sizeof(struct sockaddr_in);

        sockfd = create_tx_sock();

        char buf[2] = "y";
        
        
        // initialize value in dest
   
        std::vector<struct sockaddr_in> tx_dest_vec;
        std::vector<int> tx_ret_vec;
        unsigned int tx;

        for (tx=0; tx<tx_addr.size(); tx++) {
            std::cout << "Trigger Transmitter" << std::endl;
            bzero(&tx_dest, sizeof(struct sockaddr_in));
            tx_dest.sin_family = AF_INET;
            tx_dest.sin_port = htons(std::atoi(tx_port[tx].c_str()));
            boost::trim(tx_addr[tx]);
            tx_dest.sin_addr.s_addr = inet_addr(tx_addr[tx].c_str());
            std::cout << boost::format("Tx %d IP: %s Port: %d") % tx % tx_addr[tx] % tx_port[tx] << std::endl;
            
            tx_dest_vec.push_back(tx_dest);
            tx_ret_vec.push_back(0);
        }
        // send packet
        for (tx=0; tx<tx_addr.size(); tx++) {
            tx_ret_vec[tx] = sendto(sockfd, (char *)&buf[0], sizeof(buf), 0, (const struct sockaddr *)&tx_dest_vec[tx], addrlen);
        }
        // check status
        for (tx=0; tx<tx_addr.size(); tx++) {
            if (tx_ret_vec[tx] < 0) {
                std::cout << boost::format("Warning: Failed to notify transmitter: %d") % (tx+1) << std::endl;
            } else {
                std::cout << boost::format("Success to trigger transmitter: %d") % (tx+1) << std::endl;
            }
        }
    }

    if (sync == "pps"){
        usrp->set_time_unknown_pps(uhd::time_spec_t(0.0));
    }
    else{
        usrp->set_time_now(uhd::time_spec_t(0.0), 0);
    }
    
    boost::this_thread::sleep(boost::posix_time::seconds(0.3)); //allow for some setup time

    bool overflow_message = true;

    std::cout << std::endl;
    std::cout << boost::format(
        "Begin streaming %u samples, %f seconds in the future..."
    ) % num_requested_samples % seconds_in_future << std::endl;

    //setup streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE);
    if (interval == 0){
        stream_cmd.stream_mode = (num_requested_samples == 0)?
            uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
            uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    }
    else{
        stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
    }
    stream_cmd.num_samps = num_requested_samples;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
    usrp->issue_stream_cmd(stream_cmd);

    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout (delay before receive + padding)
    size_t num_rx_samps = 0;
    do{
        while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0)){
            num_rx_samps = rx_stream->recv(buff_ptrs, spb, md, timeout);

            //use a small timeout for subsequent packets
            timeout = 0.1;

            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
                std::cout << boost::format("Timeout while streaming") << std::endl;
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
            }
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
                if (overflow_message){
                    overflow_message = false;
                    std::cerr << boost::format(
                        "Got an overflow indication. Please consider the following:\n"
                        "  Your write medium must sustain a rate of %fMB/s.\n"
                        "  Dropped samples will not be written to the file.\n"
                        "  Please modify this example for your purposes.\n"
                        "  This message will not appear again.\n"
                    ) % (usrp->get_rx_rate()*sizeof(std::complex<float>)/1e6);
                }
                
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
                 
            }
            /*
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
            }
            */
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND){
                std::cout << boost::format("A stream command was issued in the past. %u full secs, %f frac secs") % 
                    md.time_spec.get_full_secs() % md.time_spec.get_frac_secs()  << std::endl;
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
            }
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN){
                std::cout << boost::format("Expected another stream command") << std::endl;
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
            }
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT){
                std::cout << boost::format("Multi-channel alignment failed.") << std::endl;
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
            }
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET){
                std::cout << boost::format("The packet could not be parsed.") << std::endl;
                throw std::runtime_error(str(boost::format(
                    "Unexpected error code 0x%x"
                ) % md.error_code));
            }

            if(verbose) std::cout << boost::format(
                "Received packet: %u samples, %u full secs, %f frac secs"
            ) % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;

            num_total_samps += num_rx_samps;
           
            for (chan = 0; chan < chan_num; chan++){
                outfile[chan]->write((const char*)buff_ptrs[chan], num_rx_samps*sizeof(std::complex<float>));
            }
        }

        std::cout << boost::format(
                "Total Received packet: %u samples, %u full secs, %f frac secs"
            ) % num_total_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;


        if (interval != 0.0){
            num_total_samps = 0;
            stream_cmd.stream_now = false;
            seconds_in_future = interval/1000 + num_rx_samps/usrp->get_rx_rate() + md.time_spec.get_full_secs() + md.time_spec.get_frac_secs();
            std::cout << std::endl;
            std::cout << boost::format(
                "Begin streaming %u samples, %f seconds in the future..."
                ) % num_requested_samples % seconds_in_future << std::endl;
            stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
            usrp->issue_stream_cmd(stream_cmd);
            timeout = seconds_in_future + 0.1;
        }

    }while(not stop_signal_called and (num_requested_samples != num_total_samps or num_requested_samples == 0));

    for (chan=0; chan < chan_num; chan++){
        outfile[chan]->close();
    }





    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
