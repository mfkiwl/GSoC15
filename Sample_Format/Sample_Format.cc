/*!
* \file Number_of_Satellites.c
*
* It locates all the satellites that could be tuned in by the receiver.
* It sets up the logging system, creates a ControlThread object,
* makes it run, and releases memory back when the main thread has ended.
* The gathered information can be used for auto-configuration of receiver.
* This program also separates out different metadata formats into .xml files.
*
* -------------------------------------------------------------------------
*
*/

#ifndef GNSS_SDR_VERSION
#define GNSS_SDR_VERSION "0.0.5"
#endif

#include <ctime>
#include <memory>
#include <queue>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/exception_ptr.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gnuradio/msg_queue.h>
#include "control_thread.h"
#include "concurrent_queue.h"
#include "concurrent_map.h"
#include "gps_ephemeris.h"
#include "gps_almanac.h"
#include "gps_iono.h"
#include "gps_utc_model.h"
#include "galileo_ephemeris.h"
#include "galileo_almanac.h"
#include "galileo_iono.h"
#include "galileo_utc_model.h"
#include "sbas_telemetry_data.h"
#include "sbas_ionospheric_correction.h"
#include "sbas_satellite_correction.h"
#include "sbas_ephemeris.h"
#include "sbas_time.h"
#include "math.h"
#include "string.h"
#include <GnssMetadata/Metadata.h>
#include <GnssMetadata/Xml/XmlProcessor.h>

/* !
* Please add all the GnssMetadata/Metadata.h files from the following link:
* https://github.com/sumitgautam-1990/API/tree/master/include/GnssMetadata
*
*/

using namespace GnssMetadata;

using google::LogMessage;

void ReadXmlFile(const char* pszFilename);

void WriteXmlFile(const char* pszFilename, char* file_name, int count, long double Sat_latitude, long double Sat_longitude, long double Sat_height);

DECLARE_string(log_dir);

/*!
* \todo make this queue generic for all the GNSS systems (javi)
*/

/*
* Concurrent queues that communicates the Telemetry Decoder
* to the Observables modules
*/


int main(int argc, char** argv)
{
    const std::string intro_help(
            std::string("\nGNSS-SDR is an Open Source GNSS Software Defined Receiver\n")
    +
    "Copyright (C) 2010-2015 (see AUTHORS file for a list of contributors)\n"
    +
    "This program comes with ABSOLUTELY NO WARRANTY;\n"
    +
    "See COPYING file to see a copy of the General Public License\n \n");

    const std::string gnss_sdr_version(GNSS_SDR_VERSION);
    google::SetUsageMessage(intro_help);
    google::SetVersionString(gnss_sdr_version);
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "Initializing GNSS-SDR v" << gnss_sdr_version << " ... Please wait." << std::endl;

    google::InitGoogleLogging(argv[0]);
    if (FLAGS_log_dir.empty())
        {
            std::cout << "Logging will be done at "
                << boost::filesystem::temp_directory_path()
                << std::endl
                << "Use gnss-sdr --log_dir=/path/to/log to change that."
                << std::endl;
        }
    else
        {
            const boost::filesystem::path p (FLAGS_log_dir);
            if (!boost::filesystem::exists(p))
                {
                    std::cout << "The path "
                        << FLAGS_log_dir
                        << " does not exist, attempting to create it"
                        << std::endl;
                    boost::filesystem::create_directory(p);
                }
            std::cout << "Logging with be done at "
                      << FLAGS_log_dir << std::endl;
        }

    std::unique_ptr<ControlThread> control_thread(new ControlThread());
    
    long int i=1; 
    long double _latitude[10000], _longitude[10000], _height[10000];
    long long int begin[10000], end[10000], active_time[10000];

    while(i!=0)
    {
        for(long double lat = fabs(-90); lat <= fabs(90); lat = lat + power(10,-5))
        {
            for(long double longi = fabs(-180); longi <= fabs(180); longi = longi + power(10,-5))
	    {
	        for(long int height = 2000; height <= 20000; height = height + 100)
 		{
		    ////////////////////////////////
      	            //Define the Session.
		    Session sess("%ld",i);
		    sess.Scenario("Example %ld", i);
		    sess.Campaign("GNSS Metadata Satellite Positioning");
		    sess.Contact("CTTC");
		    sess.Position( Position(lat, longi, height));
		    sess.AddComment("This is an example of locating the satellites that can be tuned by the receiver.");
        	   	        
		    // record the position of located satellite.
                    if(sess.Position!=0)
		    {
		         _latitude(i) = lat; _longitude(i) = longi; _height(i) = height;
			 i++;
		    }
 
                    // record startup time
    	   	    struct timeval tv;
    	   	    gettimeofday(&tv, NULL);
    	   	    begin(i) = tv.tv_sec * 1000000 + tv.tv_usec;
           	    try
    	   	    {
            	        control_thread->run();
    	   	    }
    	   	    catch( boost::exception & e )
    	   	    {
            	        LOG(FATAL) << "Boost exception: " << boost::diagnostic_information(e);
    	   	    }
    	   	    catch(std::exception const&  ex)
    	   	    {
            	    	LOG(FATAL) << "STD exception: " << ex.what();
    	   	    }
    	   	    // report the elapsed time
    	   	    gettimeofday(&tv, NULL);
    	   	    end(i) = tv.tv_sec * 1000000 + tv.tv_usec;
    	   	    std::cout << "Total GNSS-SDR run time "
              	              << (static_cast<double>(end(i) - begin(i))) / 1000000.0
              	     	      << " [seconds]" << std::endl;
           	    active_time(i) = end(i) - begin(i);

	        }
	    }
        }
 	if(lat == fabs(90) && longi == fabs(180) && height == 20000)
	{
	    long int n = i; i = 0;
        }
    } 	  
    
    // Displaying the total number of satellites found
    std::cout << "Total number of Satellites Located : " << n;

    long double Sat_latitude, Sat_longitude, Sat_height;
    int count = 0, Number_of_Bands = 0;

    std::cin >> "Enter the latitude value of Satellite : " >> Sat_latitude >> std::endl;
    std::cin >> "Enter the longitude value of Satellite : " >> Sat_longitude >> std::endl;	
    std::cin >> "Enter the height value of Satellite : " >> Sat_height >> std::endl;

    for (long int j = 1; j <= n; j++)
    {    if (_lattitude(j) == Sat_latitude && _longitude(j) == Sat_longitude && _height(j) == Sat_height)
	     count = 1;
	 else 
	     count = 0;
    } 

    if( count == 1)
    {   //Define the Session.
        Session sess("%d", j);
        sess.Scenario("Example %d", j);
        sess.Campaign("GNSS Metadata API Testing");
        sess.Contact("CTTC");
        sess.Position( Position(Sat_latitude, Sat_longitude, Sat_height));
        sess.AddComment("This part tunes the receiver to the specified positions.");

        //Define the System, Sources, and cluster.
	System sys("A2300-1");
	sys.BaseFrequency( Frequency(4, Frequency::MHz));
	sys.Equipment("ASR-2300");
	sys.AddComment( "ASR-2300 configured with standard firmware and FPGA id=1, version=1.18.");

	Cluster clstr("Antenna");

	Source src(Source::Patch, Source::RHCP, "L1 C/A");
	src.IdCluster("Antenna");

	sys.AddSource(src);
	sys.AddCluster(clstr);

        // record startup time
        struct timeval tv;
        gettimeofday(&tv, NULL);
        long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

        try
        {
                control_thread->run();
        }
        catch( boost::exception & e )
        {
                LOG(FATAL) << "Boost exception: " << boost::diagnostic_information(e);
        }
        catch(std::exception const&  ex)
        {
                LOG(FATAL) << "STD exception: " << ex.what();
        }
        // report the elapsed time
        gettimeofday(&tv, NULL);
        long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
        long long int total_time = end - begin;
        std::cout << "Total GNSS-SDR run time "
                  << (static_cast<double>(total_time)) / 1000000.0
                  << " [seconds]" << std::endl;

        // retrieve the bandwidth of the signal
        long long int bandwidth = 1.0/total_time;
        std::cout << "Total Bandwidth "
                  << (static_cast<double>(bandwidth))  
                  << " [MHz]" << std::endl;

	// finding the center frequency
	long long int center_freq = mean(bandwidth);
        std::cout << "Center Frequency  "
                  << (static_cast<double>(center_freq))  
                  << " [MHz]" << std::endl;

	////////////////////////////////
	//Define Band 1 and L1 C/A Stream spanning whole frequency.
	Band ch("L1External");
	ch.CenterFrequency(Frequency( center_freq, Frequency::MHz));
	ch.TranslatedFrequency(Frequency( 38400, Frequency::Hz));

	Stream sm("L1ca");
	sm.RateFactor(1);
	sm.Quantization(8);
	sm.Packedbits(16);
	sm.Encoding("INT8");
	sm.Format(Stream::IQ);
	sm.Bands().push_back(ch);
        Number_of_Bands = sizeof(sm.Bands().push_back(ch)) / sizeof(int);
	
    }
    else
    {
        std::cout << "No satellite detected in specified positions.";
    }

    std::cout << "Number of RF Channels =  "
              << (static_cast<double>(Number_of_Bands))  
              << " [MHz]" << std::endl;
	         
    
    // Get the filename if specified.
    
    for(int iter = 1; iter <= Number_of_Bands; iter++)
    {
 	char* file_name_1 = "141230-gps-4msps_";
	char* char_add = itoa (iter);
        char* file_name;
        strcat(file_name_1, iter);
	strcpy(file_name, file_name_1);
	strcat(file_name, ".xml");
                
	const char* pszFilename = file_name;
    	if( argc > 1) pszFilename = argv[1];

	printf("GNSS Metadata XML file translation\n");
    	printf("\n");
    	printf("Application implements writing and reading an XML file\n");
    	printf("Program creates a metadata file using the API and then parses it.\n");
    	printf("\n");
    	printf("Command line\n");
    	printf("GnssMetadataTestApp [xmlfile (default: '%s')]\n", pszfilename);

    	WriteXmlFile(pszFilename, file_name_1, iter, Sat_latitude, Sat_longitude, Sat_height);
    	ReadXmlFile(pszFilename);
    }

    google::ShutDownCommandLineFlags();
    std::cout << "GNSS-SDR program ended." << std::endl;
    
    return 0;   
}


void ReadXmlFile(const char* pszFilename)
{
    printf("\nReading GNSS Metadata to xml file: %s\n", pszFilename);

    Metadata md;
    XmlProcessor xproc;

    try
    {
        if( xproc.Load( pszFilename, false, md) )
        {
            printf("Xml Processed successfully.\n");


        }
    }
    catch( ApiException& e)
    {
        printf("An error occurred while saving the xml file: %s\n", e.what() );
    }
    catch( std::exception& ge)
    {
        printf("Unknown Exception Occured: %s\n", ge.what() );
    }
}


void WriteXmlFile(const char* pszFilename, char* file_name, int count, long double Sat_latitude, long double Sat_longitude, long double Sat_height)
{
    printf("\nWriting GNSS Metadata to xml file: %s\n", pszFilename);

    ////////////////////////////////
    //Define information about the datafile.

    //UTC:    24-Aug-2015 21:05:05
    //GPS:    1825/254334.906
    Date dt0( 254334.906, 1825);
    size_t offset = 588;
    strcat(file_name, ".bds");
    String sfile = file_name;
    String sfilemd = pszFilename;

    ////////////////////////////////
    //Define the Session.
    Session sess("%d", count);
    sess.Scenario("Example %d", count);
    sess.Campaign("GNSS Metadata API");
    sess.Contact("CTTC");
    sess.Position( Position(Sat_latitude, Sat_longitude, Sat_height));
    sess.AddComment("This locates the satellite with metadata specification having interleaved streams.");

    ////////////////////////////////
    //Define the System, Sources, and cluster.
    System sys("A2300-1");
    sys.BaseFrequency( Frequency( 4, Frequency::MHz));
    sys.Equipment("ASR-2300");
    sys.AddComment( "ASR-2300 configured with standard firmware and FPGA id=1, version=1.18.");

    Cluster clstr("Antenna");

    Source src( Source::Patch, Source::RHCP, "L1 C/A");
    src.IdCluster("Antenna");

    sys.AddSource(src);
    sys.AddCluster(clstr);

    // record startup time
    struct timeval tv;
    gettimeofday(&tv, NULL);
    long long int begin = tv.tv_sec * 1000000 + tv.tv_usec;

    try
    {
            control_thread->run();
    }
    catch( boost::exception & e )
    {
            LOG(FATAL) << "Boost exception: " << boost::diagnostic_information(e);
    }
    catch(std::exception const&  ex)
    {
            LOG(FATAL) << "STD exception: " << ex.what();
    }
    // report the elapsed time
    gettimeofday(&tv, NULL);
    long long int end = tv.tv_sec * 1000000 + tv.tv_usec;
    long long int total_time = end - begin;
    long long int bandwidth = 1.0/total_time;
    long long int center_freq = mean(bandwidth);
    
    ////////////////////////////////
    //Define Band 1 and L1 C/A Stream.
    Band ch("L1External");
    ch.CenterFrequency(Frequency( center_freq, Frequency::MHz));
    ch.TranslatedFrequency(Frequency( 38400, Frequency::Hz));

    //Stream sm will be added to the global metadata and as a reference to the lump.

    stream sm("L1ca");
    sm.RateFactor(1);
    sm.Quantization(8);
    sm.Packedbits(16);
    sm.Encoding("INT8");
    sm.Format(Stream::IQ);
    sm.Bands().push_back(ch);

    ////////////////////////////////
    //Define the lane
    Lump lump;
    lump.Streams().push_back(sm);
    lump.Streams().push_back(sm.ToReference<Stream>());	

    Chunk chunk;
    chunk.SizeWord(4);
    chunk.CountWords(1);
    chunk.Lumps().push_back(lump);

    Block blk(256);
    blk.Chunks().push_back(chunk);
	
    Lane lane("GPS SPS Data");
    lane.Sessions().push_back(sess);
    lane.Blocks().push_back(blk);
    lane.AddBandSource(ch, src);
	
    lane.Systems().push_back( sys.ToReference<System>());

    ////////////////////////////////
    //Define the file
    File df;
    df.Url(sfile);
    df.TimeStamp(dt0);
    df.Lane( lane, true);

    ////////////////////////////////
    //Assemble the Metadata object and write XML
 
    Metadata md;
    XmlProcessor proc;
    md.Lanes().push_back(lane);
    md.Files().push_back(df);
    md.Systems().push_back(sys);
    md.Streams().push_back(sm); 

    try
    {
        proc.Save(sfilemd.c_str(), md);
    }
    catch( ApiException& e)
    {
	printf("An error occurred while saving the xml file: %s\n", e.what());
    }
}
