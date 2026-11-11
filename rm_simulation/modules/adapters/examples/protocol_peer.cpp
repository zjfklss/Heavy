#include "rm_sim/adapters/server.hpp"
#include <iostream>

// Transport smoke-test peer. It acknowledges input and never runs an aiming algorithm.
int main(int argc,char** argv){
  try{
    rm_sim::adapters::ServerOptions options;
    if(argc>1)options.adapter=argv[1];
    if(argc>2)options.port=std::stoi(argv[2]);
    if(argc>3)throw std::invalid_argument("Usage: rm_sim_adapter_peer [generic|sp_vision] [port]");
    options.peer_version="protocol-only-peer/1";
    rm_sim::adapters::ProtocolServer server(options);
    std::cout<<"Listening for simulation: "<<options.host<<":"<<options.port<<" profile="<<options.adapter<<std::endl;
    server.accept();size_t frames=0,steps=0;
    while(auto step=server.next()){
      frames+=step->header.at("images").size();++steps;
      // Your separately built algorithm consumes pixels/feedback here and supplies commands.
      server.reply({});
    }
    std::cout<<"Protocol session finished: steps="<<steps<<" images="<<frames<<'\n';
  }catch(const std::exception& e){std::cerr<<"[adapter peer] "<<e.what()<<'\n';return 1;}
}
