#ifndef G3T1_2_HPP
#define G3T1_2_HPP

#include <fstream>
#include <chrono>
#include <string>
#include <iostream>

#include "ros/ros.h"

#include "bsn/generator/DataGenerator.hpp"
#include "bsn/range/Range.hpp"
#include "bsn/resource/Battery.hpp"
#include "bsn/generator/Markov.hpp"
#include "bsn/filters/MovingAverage.hpp"
#include "bsn/operation/Operation.hpp"
#include "bsn/configuration/SensorConfiguration.hpp"

#include "component/SchedulableComponent.hpp"

#include "messages/SensorData.h"
#include "messages/Status.h"
#include "messages/Event.h"

class G3T1_2 : public SchedulableComponent {
    
	private:
      	G3T1_2(const G3T1_2 &);
    	G3T1_2 &operator=(const G3T1_2 &);

  	public:
	    G3T1_2(const int32_t &argc, char **argv);
    	~G3T1_2();

    	void setUp();
    	void tearDown();
		void body();

		void sendStatus(const std::string &/*id*/, const double &/*value*/);
		void sendEvent(const std::string &/*type*/, const std::string &/*description*/);

  	private:
		std::string type;
		bsn::resource::Battery battery;
		bool available;

		double data_accuracy;
		double comm_accuracy;

		bool active;
		std::map<std::string,double> params;

		bsn::generator::Markov markov;
		bsn::filters::MovingAverage filter;
		bsn::configuration::SensorConfiguration sensorConfig;

		int persist;
		std::string path;
		std::ofstream fp;

		ros::Publisher status_pub, event_pub, dataPub;
};

#endif 