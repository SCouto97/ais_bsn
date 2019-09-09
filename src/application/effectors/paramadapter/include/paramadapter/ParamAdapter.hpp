#ifndef PARAM_ADAPTER_HPP
#define PARAM_ADAPTER_HPP

#include <string>

#include "ros/ros.h"

#include "services/EffectorRegister.h"
#include "archlib/target_system/Effector.hpp"
#include "archlib/AdaptationCommand.h"

class ParamAdapter : public arch::target_system::Effector {

	public:
    	ParamAdapter(int &argc, char **argv, const std::string &name);
    	virtual ~ParamAdapter();

    private:
      	ParamAdapter(const ParamAdapter &);
    	ParamAdapter &operator=(const ParamAdapter &);

  	public:
		virtual void setUp();
		virtual void tearDown();

        virtual void receiveAdaptationCommand(const archlib::AdaptationCommand::ConstPtr& msg);
		bool moduleConnect(services::EffectorRegister::Request &req, services::EffectorRegister::Response &res);

  	private:
		ros::NodeHandle handle;
		ros::ServiceServer register_service;
	  	std::map<std::string,ros::Publisher> target_arr;

};

#endif 