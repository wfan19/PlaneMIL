#include <stdio.h>
#include <cstdlib>
#include <readline/readline.h>
#include <readline/history.h>

#include <vector>
#include <sstream>

#include <gazebo/gazebo_client.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

gazebo::transport::NodePtr gznode;
gazebo::transport::PublisherPtr ctrlPub;
gazebo::transport::SubscriberPtr stateSub;

gazebo::msgs::Cessna state;

void onState(ConstCessnaPtr &_msg)
{
    state = *_msg;
}


void initConnection()
{
    gznode = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gznode->Init();
    ctrlPub = gznode->Advertise<gazebo::msgs::Cessna>("~/cessna_c172/control");
    stateSub = gznode->Subscribe<gazebo::msgs::Cessna>("~/cessna_c172/state", &onState);
    ctrlPub->WaitForConnection();

    // Test publish message:
    gazebo::msgs::Cessna testMsg;
    testMsg.set_cmd_propeller_speed(0.5f);
    ctrlPub->Publish(testMsg);
}

int main(int argc, char ** argv)
{
    gazebo::client::setup(argc, argv);
    initConnection();

    while(1)
    {
        initConnection();

        char *line = readline("> ");
        if(!line) break;
        if(*line) add_history(line);
        // std::string line_str = str(line);
        // /* Do something with the line here */
        
        // std::string delimiter = " ";
        // size_t pos = 0;
        // vector<string> parsedCmd;
        // std::stringstream ss(&line);
        // std::string item;
        // while(std::getline(ss, item, delimiter)){
        //     result.push_back(parsedCmd);
        // }
        
        // parsedCmd
        free(line);
    }
}