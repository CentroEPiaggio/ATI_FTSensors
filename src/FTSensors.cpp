/*
 * Software License Agreement (BSD 3-Clause License)
 *
 *   FTSensors - Configure and read data from a force/torque sensor
 *   Copyright (c) 2016, Simone Ciotti (simone.ciotti@centropiaggio.unipi.it)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <FTSensors.h>

void FTSensors::ATI::NetFT::collectData(FTSensors::ATI::NetFT* sensor)
{
	boost::chrono::system_clock::time_point first_time;
	bool firstData;
	boost::chrono::nanoseconds data_time;
	//stores the number of byte received
	size_t result;
	//the endpoint of the UDP communication
	//this is the sensor from read the force/torque data
	boost::asio::ip::udp::endpoint sensorEndpoint;
	
	/*
		true for the first RDT packet
	*/
	firstData = true;
	
	// get and format data
	for(;;)
	{
		result = sensor->udpSock->receive_from(boost::asio::buffer((char*)&(sensor->record), sizeof(sensor->record)), sensorEndpoint);
		
		if(result < sizeof(sensor->record))
			break;
		
		boost::unique_lock<boost::mutex> lock(sensor->dataMutex);
		
		/*
			convert the data byte order from network to host
		*/
		sensor->record.rdt_sequence = ntohl(sensor->record.rdt_sequence);
		sensor->record.ft_sequence = ntohl(sensor->record.ft_sequence);
		sensor->record.status = ntohl(sensor->record.status);

		sensor->record.Fx = ntohl(sensor->record.Fx);
		sensor->record.Fy = ntohl(sensor->record.Fy);
		sensor->record.Fz = ntohl(sensor->record.Fz);
		sensor->record.Tx = ntohl(sensor->record.Tx);
		sensor->record.Ty = ntohl(sensor->record.Ty);
        sensor->record.Tz = ntohl(sensor->record.Tz);
		
		/*
			convert the force/torque data from sensor tick to the set force/torque measurement unit
		*/
		sensor->Fx = ((double)sensor->record.Fx)/sensor->countsPerForce;
		sensor->Fy = ((double)sensor->record.Fy)/sensor->countsPerForce;
		sensor->Fz = ((double)sensor->record.Fz)/sensor->countsPerForce;
		sensor->Tx = ((double)sensor->record.Tx)/sensor->countsPerTorque;
		sensor->Ty = ((double)sensor->record.Ty)/sensor->countsPerTorque;
		sensor->Tz = ((double)sensor->record.Tz)/sensor->countsPerTorque;

		/*
			save the RDT package sequence number
		*/
		sensor->packetNumber = sensor->record.rdt_sequence;

		/*
			if it is the first RDT packet set it sequence time to 0,
			and gets the reference time to compute the elapsed time between two packet
		*/
		if(firstData)
		{
			sensor->packetTime = 0;
			first_time = boost::chrono::system_clock::now();

			firstData = false;
		}
		else
		{
			/*
				compute the elapsed time between the previous packet and this packet
			*/
			data_time = boost::chrono::system_clock::now() - first_time;
			/*
				store the elapsed time in nanoseconds
			*/
			sensor->packetTime = data_time.count();
		}

		/*
			signals to function getData that new force/torque data is available
		*/
		sensor->dataAvailable.notify_one();
		/* create a cancellation point*/
		boost::this_thread::interruption_point();
	}
}
		
bool FTSensors::ATI::NetFT::setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter nbp, unsigned short int param_value)
{
	std::string command_str;
	double retrieve_param;
	
	boost::asio::ip::tcp::iostream tcp_socket;

	// The entire sequence of I/O operations must complete within 60 seconds.
    // If an expiry occurs, the socket is automatically closed and the stream
    // becomes bad.
    tcp_socket.expires_from_now(boost::posix_time::seconds(60));

     // Establish a connection to the server.
    tcp_socket.connect(this->IP, "http");
    if (!tcp_socket)
    	return false;
		
	//Retrieve NetBox active configuration
	if(!(this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::ACTIVE_CONFIGURATION, retrieve_param) ))
		return false;

	switch(nbp)
	{
		case FTSensors::ATI::NetFT::NetBoxParameter::FILTER_FREQUENCY:
																						command_str = "setting.cgi?setuserfilter=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::FORCE_UNIT:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgfu=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_UNIT:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtu=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::DATA_RATE:
																						command_str = "comm.cgi?comrdtrate=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::RDT_INTERFACE:
																						command_str = "comm.cgi?comrdte=" + boost::lexical_cast<std::string>(param_value);
																						break;
        case FTSensors::ATI::NetFT::NetBoxParameter::RDT_BUFFER:
																						command_str = "comm.cgi?comrdtbsiz=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::MULTI_UNIT_SYNC:
																						command_str = "comm.cgi?comrdtmsyn=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::THRESHOLD_MONITORING:
																						command_str = "moncon.cgi?setmce=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_DISTANCE_X:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtfx0=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_DISTANCE_Y:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtfx1=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_DISTANCE_Z:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtfx2=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_ROTATION_X:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtfx3=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_ROTATION_Y:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtfx4=" + boost::lexical_cast<std::string>(param_value);
																						break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_ROTATION_Z:
																						command_str = "config.cgi?cfgid=" + boost::lexical_cast<std::string>(retrieve_param) + "&cfgtfx5=" + boost::lexical_cast<std::string>(param_value);
																						break;
        default:
        		return false;
	}

    std::stringstream tcp_request;

    tcp_request << "GET /"<< command_str <<" HTTP/1.0\r\n";
    tcp_request << "Host: " << this->IP <<"\r\n";
	tcp_request << "Accept: */*\r\n";
    tcp_request << "Connection: close\r\n\r\n";

	tcp_socket << tcp_request.str();

	// By default, the stream is tied with itself. This means that the stream
    // automatically flush the buffered output before attempting a read. It is
    // not necessary not explicitly flush the stream at this point.

    // Check that response is OK.
    std::string http_version;
    unsigned int status_code;
    std::string status_message;

    tcp_socket >> http_version;
    tcp_socket >> status_code;
    
    std::getline(tcp_socket, status_message);
    if (!tcp_socket || http_version.substr(0, 5) != "HTTP/")
    	return false;

    if(status_code >= 400)
    	return false;

	return true;
}

bool FTSensors::ATI::NetFT::getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter nbp, double& param_value)
{
	boost::asio::ip::tcp::iostream tcp_socket;

	// The entire sequence of I/O operations must complete within 60 seconds.
    // If an expiry occurs, the socket is automatically closed and the stream
    // becomes bad.
    tcp_socket.expires_from_now(boost::posix_time::seconds(60));

     // Establish a connection to the server.
    tcp_socket.connect(this->IP, "http");
    if (!tcp_socket)
    	return false;

    std::stringstream tcp_request;

    tcp_request << "GET /netftapi2.xml HTTP/1.0\r\n";
    tcp_request << "Host: " << this->IP <<"\r\n";
	tcp_request << "Accept: */*\r\n";
    tcp_request << "Connection: close\r\n\r\n";

	tcp_socket << tcp_request.str();

    // By default, the stream is tied with itself. This means that the stream
    // automatically flush the buffered output before attempting a read. It is
    // not necessary not explicitly flush the stream at this point.

    // Check that response is OK.
    std::string http_version;
    unsigned int status_code;
    std::string status_message;

    tcp_socket >> http_version;
    tcp_socket >> status_code;
    
    std::getline(tcp_socket, status_message);
    if (!tcp_socket || http_version.substr(0, 5) != "HTTP/")
    	return false;

    if(status_code != 200)
    	return false;

    std::stringstream tcp_response;
    tcp_response << tcp_socket.rdbuf();
    std::string tcp_response_str(tcp_response.str());

    std::size_t idx_netft = tcp_response_str.find("<netft>");

    if(idx_netft == std::string::npos)
    	return false;
    else
    	boost::algorithm::erase_head(tcp_response_str, idx_netft);

    std::size_t pos_s;
    std::size_t pos_e;
    unsigned short int pos_disp;

	switch(nbp)
	{
		case FTSensors::ATI::NetFT::NetBoxParameter::ACTIVE_CONFIGURATION:
																				pos_s = tcp_response_str.find("<setcfgsel>");
																				pos_e = tcp_response_str.find("</setcfgsel>");
																				pos_disp = std::string("<setcfgsel>").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::COUNTS_PER_FORCE:
																				pos_s = tcp_response_str.find("<cfgcpf>");
																				pos_e = tcp_response_str.find("</cfgcpf>");
																				pos_disp = std::string("<cfgcpf>").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::COUNTS_PER_TORQUE:
																				pos_s = tcp_response_str.find("<cfgcpt>");
																				pos_e = tcp_response_str.find("</cfgcpt>");
																				pos_disp = std::string("<cfgcpt>").length();
																				break;
        case FTSensors::ATI::NetFT::NetBoxParameter::FORCE_SENSING_RANGE_X:
																				pos_s = tcp_response_str.find("<cfgmr>");
																				pos_e = tcp_response_str.find(";", pos_s);
																				pos_disp = std::string("<cfgmr>").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::FORCE_SENSING_RANGE_Y:	
																				pos_s = tcp_response_str.find(";", tcp_response_str.find("<cfgmr>"));
																				pos_e = tcp_response_str.find(";", pos_s + std::string(";").length());
																				pos_disp = std::string(";").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::FORCE_SENSING_RANGE_Z:
																				pos_s = tcp_response_str.find(";",
																					tcp_response_str.find(";",
																						tcp_response_str.find("<cfgmr>") )
																					+ std::string(";").length() );
																				pos_e = tcp_response_str.find(";", pos_s + std::string(";").length());
																				pos_disp = std::string(";").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_SENSING_RANGE_X:	
																				pos_s = tcp_response_str.find(";",
																					tcp_response_str.find(";",
																					tcp_response_str.find(";",
																						tcp_response_str.find("<cfgmr>") )
																					+ std::string(";").length() )
																					+ std::string(";").length() );
																				pos_e = tcp_response_str.find(";", pos_s + std::string(";").length());
																				pos_disp = std::string(";").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_SENSING_RANGE_Y:	
																				pos_s = tcp_response_str.find(";",
																					tcp_response_str.find(";",
																					tcp_response_str.find(";",
																					tcp_response_str.find(";",
																						tcp_response_str.find("<cfgmr>") )
																					+ std::string(";").length() )
																					+ std::string(";").length() )
																					+ std::string(";").length() );
																				pos_e = tcp_response_str.find(";", pos_s + std::string(";").length());
																				pos_disp = std::string(";").length();
																				break;
		case FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_SENSING_RANGE_Z:	
																				pos_s = tcp_response_str.find(";",
																					tcp_response_str.find(";",
																					tcp_response_str.find(";",
																					tcp_response_str.find(";",
																					tcp_response_str.find(";",
																						tcp_response_str.find("<cfgmr>") )
																					+ std::string(";").length() )
																					+ std::string(";").length() )
																					+ std::string(";").length() )
																					+ std::string(";").length() );
																				pos_e = tcp_response_str.find("</cfgmr>");
																				pos_disp = std::string(";").length();
																				break;
        default:
        		return false;
	}
	
	if((pos_s == std::string::npos) || (pos_e == std::string::npos))
		return false;

	param_value = boost::lexical_cast<double>(tcp_response_str.substr( pos_s + pos_disp, pos_e - (pos_s + pos_disp) ) );

	return true;
}

FTSensors::ATI::NetFT::NetFT()
{
	this->udpSock = new boost::asio::ip::udp::socket(this->udpIOService,boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 0));
	this->forceUnit = FTSensors::ATI::ForceUnit::NO_VALUE;
	this->torqueUnit = FTSensors::ATI::TorqueUnit::NO_VALUE;
	this->filterFrequency = FTSensors::ATI::FilterFrequency::NO_VALUE;
	this->dataRate = 0;
}

FTSensors::ATI::NetFT::~NetFT()
{
    /*
        stop RDT communication
    */
    this->stopDataStream();
}

bool FTSensors::ATI::NetFT::calibration(unsigned int samples_number)
{
	/*
		convert data byte order from host to network 
	*/
	this->request.command_header = htons(0x1234);//required
	this->request.command = htons(0x0042);//calibration command
	/*
		force/torque samples used to calibrate the sensor
		!!!SENSOR RUNS CALIBRATION INTERNALLY!!!
	*/
	if(samples_number == 0)
		this->request.sample_count = htonl(this->dataRate);
	else
		this->request.sample_count = htonl(samples_number);
	/*
		sends calibration command to the sensor
		if it fail return false otherwise return true
	*/
	std::size_t result = this->udpSock->send_to(boost::asio::buffer((char*)&(this->request),sizeof(this->request)),boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(this->IP), this->port));
	
	if(result < sizeof(this->request))
		return false;
	
	return true;
}

bool FTSensors::ATI::NetFT::setIP(std::string ip)
{ 
	/*
		Test if ip is a valid IP address in the form xxx.yyy.www.zzz
	*/
	if(std::count(ip.begin(),ip.end(),'.')<3)
		return false;
	
	boost::system::error_code ec;
	boost::asio::ip::address::from_string(ip, ec);
	if( ec )
		return false;

	this->IP = ip;
	
	//Host reachable?
	bool res_1, res_2, res_3, res_4, res_5, res_6, res_7, res_8, res_9, res_10;

	res_1 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::RDT_INTERFACE, 1);
	res_2 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::RDT_BUFFER, 1);
	res_3 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::MULTI_UNIT_SYNC, 0);
	res_4 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::THRESHOLD_MONITORING, 0);
	res_5 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_DISTANCE_X, 0);
	res_6 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_DISTANCE_Y, 0);
	res_7 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_DISTANCE_Z, 0);
	res_8 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_ROTATION_X, 0);
	res_9 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_ROTATION_Y, 0);
	res_10 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TOOL_TRANSFORMATION_ROTATION_Z, 0);

	if(!res_1 || !res_2 || !res_3 || !res_4 || !res_5 || !res_6 || !res_7 || !res_8 || !res_9 || !res_10 )
		return false;

	return true;
}

std::string FTSensors::ATI::NetFT::getIP()
{
    return this->IP;
}

unsigned short int FTSensors::ATI::NetFT::getPort()
{
    return this->port;
}

bool FTSensors::ATI::NetFT::setFilterFrequency(FTSensors::ATI::FilterFrequency ff)
{
	bool result;

	result = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::FILTER_FREQUENCY, static_cast<unsigned short int>(ff) );
	
	if(!result)
		return false;

	this->filterFrequency = ff;

	return true;
}

FTSensors::ATI::FilterFrequency FTSensors::ATI::NetFT::getFilterFrequency()
{
    return this->filterFrequency;
}

bool FTSensors::ATI::NetFT::setForceUnit(FTSensors::ATI::ForceUnit fu)
{
	bool res_1, res_2;
	double ret_param;

    res_1 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::FORCE_UNIT, static_cast<unsigned short int>(fu) );
	res_2 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::COUNTS_PER_FORCE, ret_param);

	if(!res_1 || !res_2)
		return false;

	this->forceUnit = fu;
	this->countsPerForce = ret_param;

	return true;
}

FTSensors::ATI::ForceUnit FTSensors::ATI::NetFT::getForceUnit()
{
    return this->forceUnit;
}

bool FTSensors::ATI::NetFT::getForceSensingRange(double& fx, double& fy, double& fz)
{
	bool res_1, res_2, res_3;
	double ret_param_1, ret_param_2, ret_param_3;

	res_1 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::FORCE_SENSING_RANGE_X, ret_param_1);
	res_2 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::FORCE_SENSING_RANGE_Y, ret_param_2);
	res_3 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::FORCE_SENSING_RANGE_Z, ret_param_3);

	if(!res_1 || !res_2 || !res_3)
		return false;

	fx = ret_param_1;
	fy = ret_param_2;
	fz = ret_param_3;

	return true;
}

bool FTSensors::ATI::NetFT::setTorqueUnit(FTSensors::ATI::TorqueUnit tu)
{   
	bool res_1, res_2;
	double ret_param;

   	res_1 = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_UNIT, static_cast<unsigned short int>(tu) );
	res_2 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::COUNTS_PER_TORQUE, ret_param);

	if(!res_1 || !res_2)
		return false;

	this->torqueUnit = tu;
	this->countsPerTorque = ret_param;

	return true;
}

FTSensors::ATI::TorqueUnit FTSensors::ATI::NetFT::getTorqueUnit()
{
    return this->torqueUnit;
}

bool FTSensors::ATI::NetFT::getTorqueSensingRange(double& tx, double& ty, double& tz)
{
	bool res_1, res_2, res_3;
	double ret_param_1, ret_param_2, ret_param_3;

	res_1 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_SENSING_RANGE_X, ret_param_1);
	res_2 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_SENSING_RANGE_Y, ret_param_2);
	res_3 = this->getNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::TORQUE_SENSING_RANGE_Z, ret_param_3);

	if(!res_1 || !res_2 || !res_3)
		return false;

	tx = ret_param_1;
	ty = ret_param_2;
	tz = ret_param_3;

	return true;
}

bool FTSensors::ATI::NetFT::setDataRate(unsigned short int dr)
{
	bool result;

    if((dr>0) && (dr<=7000))
    {
        this->dataRate = dr;

       	result = this->setNetBoxParameter(FTSensors::ATI::NetFT::NetBoxParameter::DATA_RATE, dr);

        return result;
    }

    return false;
}

unsigned short int FTSensors::ATI::NetFT::getDataRate()
{
    return this->dataRate;
}

bool FTSensors::ATI::NetFT::startDataStream(bool calibration)
{
	/*
		start RDT communication
	*/
	this->request.command_header = htons(0x1234);//required
	this->request.command = htons(0x0002);//start command
	this->request.sample_count = htonl(0);//required
		
	std::size_t udpSendResult = this->udpSock->send_to(boost::asio::buffer((char*)&(this->request),sizeof(this->request)),boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(this->IP), this->port));
	
	if(udpSendResult < sizeof(this->request))
		return false;
	
	this->thCollectData = boost::thread(&(FTSensors::ATI::NetFT::collectData),this);
	this->thCollectData.detach();
        
    //if requried run the calibration
    if(calibration)
		return this->calibration();

	return true;
}

bool FTSensors::ATI::NetFT::getData(unsigned int& pn,double& pt,double& fx,double& fy,double& fz,double& tx,double& ty,double& tz)
{
	boost::unique_lock<boost::mutex> lock(this->dataMutex);
	
	if(this->dataAvailable.wait_for(lock, boost::chrono::seconds(this->DATA_WAIT_TIME_SEC)) == boost::cv_status::no_timeout )
	{
		//get the new data
		pn = this->packetNumber;
		pt = this->packetTime;
		
		fx = this->Fx;
        fy = this->Fy;
        fz = this->Fz;
        tx = this->Tx;
        ty = this->Ty;
        tz = this->Tz;
		
		return true;
	}
	else
	{
		pn = 0;
		pt = 0;
		
		fx = 0.0;
        fy = 0.0;
        fz = 0.0;
        tx = 0.0;
        ty = 0.0;
        tz = 0.0;
	}
	
	return false;
}

bool FTSensors::ATI::NetFT::getData(double& pt,double& fx,double& fy,double& fz,double& tx,double& ty,double& tz)
{
    unsigned int pn;

    return (this->getData(pn,pt,fx,fy,fz,tx,ty,tz));
}

bool FTSensors::ATI::NetFT::getData(unsigned int& pn,double& fx,double& fy,double& fz,double& tx,double& ty,double& tz)
{
    double pt;

    return (this->getData(pn,pt,fx,fy,fz,tx,ty,tz));
}

bool FTSensors::ATI::NetFT::getData(double& fx,double& fy,double& fz,double& tx,double& ty,double& tz)
{
    unsigned int pn;
    double pt;

    return (this->getData(pn,pt,fx,fy,fz,tx,ty,tz));
}

bool FTSensors::ATI::NetFT::stopDataStream()
{
	/*
		stop RDT communication
	*/
	this->request.command_header = htons(0x1234);//required
	this->request.command = htons(0x0000);//stop command
	this->request.sample_count = htonl(0);//required

	//send command to sensor
	std::size_t result = this->udpSock->send_to(boost::asio::buffer((char*)&(this->request),sizeof(this->request)),boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(this->IP), this->port));
	
	if(result < sizeof(this->request))
		return false;
	
	//send a cancellation request to the specified thread
	this->thCollectData.interrupt();

	return true;
}

std::ostream& FTSensors::ATI::operator<<(std::ostream& os, FTSensors::ATI::FilterFrequency ff)
{
    switch(ff)
    {
        case FTSensors::ATI::FilterFrequency::NO_FILTER      : os << "NO_FILTER"; break;
        case FTSensors::ATI::FilterFrequency::FILTER_5_HZ    : os << "5 Hz";      break;
        case FTSensors::ATI::FilterFrequency::FILTER_8_HZ    : os << "8 Hz";      break;
        case FTSensors::ATI::FilterFrequency::FILTER_18_HZ   : os << "18 Hz";     break;
        case FTSensors::ATI::FilterFrequency::FILTER_35_HZ   : os << "35 Hz";     break;
        case FTSensors::ATI::FilterFrequency::FILTER_73_HZ   : os << "73 Hz";     break;
        case FTSensors::ATI::FilterFrequency::FILTER_152_HZ  : os << "152 Hz";    break;
        case FTSensors::ATI::FilterFrequency::FILTER_326_HZ  : os << "326 Hz";    break;
        case FTSensors::ATI::FilterFrequency::FILTER_838_HZ  : os << "838 Hz";    break;
        case FTSensors::ATI::FilterFrequency::FILTER_1500_HZ : os << "1500 Hz";   break;
        case FTSensors::ATI::FilterFrequency::FILTER_2000_HZ : os << "2000 Hz";   break;
        case FTSensors::ATI::FilterFrequency::FILTER_2500_HZ : os << "2500 Hz";   break;
        case FTSensors::ATI::FilterFrequency::FILTER_3000_HZ : os << "3000 Hz";   break;
        case FTSensors::ATI::FilterFrequency::NO_VALUE 		 : os << "No Value";   break;
        default             							     : os.setstate(std::ios_base::failbit);
    }

    return os;
}

std::ostream& FTSensors::ATI::operator<<(std::ostream& os, FTSensors::ATI::ForceUnit fu)
{
    switch(fu)
    {
        case FTSensors::ATI::ForceUnit::lbf  		: os << "lbf";  break;
        case FTSensors::ATI::ForceUnit::N    		: os << "N";    break;
        case FTSensors::ATI::ForceUnit::klbf 		: os << "klbf"; break;
        case FTSensors::ATI::ForceUnit::KN   		: os << "kN";   break;
        case FTSensors::ATI::ForceUnit::kgf  		: os << "kgf";  break;
        case FTSensors::ATI::ForceUnit::gf   		: os << "gf";   break;
        case FTSensors::ATI::ForceUnit::NO_VALUE	: os << "No Value";   break;
        default   						     : os.setstate(std::ios_base::failbit);
    }

    return os;
}

std::ostream& FTSensors::ATI::operator<<(std::ostream& os, FTSensors::ATI::TorqueUnit tu)
{
    switch(tu)
    {
        case FTSensors::ATI::TorqueUnit::lbf_in 	: os << "lbf-in"; break;
        case FTSensors::ATI::TorqueUnit::lbf_ft 	: os << "lbf-ft"; break;
        case FTSensors::ATI::TorqueUnit::Nm     	: os << "Nm";     break;
        case FTSensors::ATI::TorqueUnit::Nmm    	: os << "Nmm";    break;
        case FTSensors::ATI::TorqueUnit::kgf_cm 	: os << "kgf-cm"; break;
        case FTSensors::ATI::TorqueUnit::KNm    	: os << "kN-m";   break;
        case FTSensors::ATI::TorqueUnit::NO_VALUE	: os << "No Value";   break;
        default		                                : os.setstate(std::ios_base::failbit);
    }

    return os;
}