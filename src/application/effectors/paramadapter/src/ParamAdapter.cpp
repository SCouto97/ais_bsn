#include "paramadapter/ParamAdapter.hpp"

ParamAdapter::ParamAdapter(int  &argc, char **argv, const std::string &name) : Effector(argc, argv, name) {}
ParamAdapter::~ParamAdapter() {}

void ParamAdapter::setUp() {
	register_service = handle.advertiseService("EffectorRegister", &ParamAdapter::moduleConnect, this);
	ros::Subscriber reconfigure_sub = handle.subscribe("reconfigure", 1000, &ParamAdapter::receiveAdaptationCommand, this);
}

void ParamAdapter::tearDown() {}

void ParamAdapter::receiveAdaptationCommand(const archlib::AdaptationCommand::ConstPtr& msg) {
	if (target_arr.find(msg->target) != target_arr.end()){
    	target_arr[msg->target].publish(msg);
	} else {
		ROS_INFO("ERROR, target not found! [%s]", msg->target.c_str());
	}
}

bool ParamAdapter::moduleConnect(services::EffectorRegister::Request &req, services::EffectorRegister::Response &res) {

	try {
		if(req.connection == true) {

            ros::Publisher pub = handle.advertise<archlib::AdaptationCommand>("effect_" + req.name, 1);
            target_arr[req.name] = pub;

			ROS_INFO("Module Connected. [%s]", req.name.c_str());

		} else {

			std::map<std::string,ros::Publisher>::iterator it;
			it = target_arr.find(req.name);
			target_arr.erase(it);

			ROS_INFO("Module Disconnected. [%s]", req.name.c_str());
		}

		res.ACK = true;

	} catch(...) {
		res.ACK = false;
	}

	return true;
}