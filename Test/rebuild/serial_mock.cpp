// 仅使用 openpty 创建的内核伪终端；不接受真实串口路径作为参数。
#include "SerialPort.h"
#include <pty.h>
#include <unistd.h>
#include <poll.h>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <stdexcept>

int main() {
  int master=-1,slave=-1;char name[128]{};
  try {
    if(openpty(&master,&slave,name,nullptr,nullptr)!=0 || std::strncmp(name,"/dev/pts/",9)!=0)
      throw std::runtime_error("Cannot create isolated PTY");
    std::array<unsigned char,21> rx{};rx[0]=0xAA;rx[1]=0xBB;
    auto pack=[&](int offset,uint32_t value){for(int i=0;i<4;i++)rx[offset+i]=(value>>(24-8*i))&255;};
    pack(3,10000);pack(7,20000);pack(11,1800000);rx[15]=1;rx[16]=1;rx[17]=0;
    {
      io::SerialPort port(name);
      std::thread source([&]{for(int i=0;i<15;i++){
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if(::write(master,rx.data(),rx.size())!=21)std::terminate();
      }});
      std::this_thread::sleep_for(std::chrono::milliseconds(85));
      Data_Save ds{};port.Query(ds,std::chrono::steady_clock::now());source.join();
      const double scale=PI/180.;
      if(std::abs(ds.camera_yaw_angle-10*scale)>1e-6 ||
         std::abs(ds.camera_pitch_angle-2*scale)>1e-6 || std::abs(ds.camera_roll_angle)>1e-6 ||
         ds.color!=1 || ds.go_back!=1 || ds.armor_or_buff!=0)
        throw std::runtime_error("Original receive units or fields changed");
      Data_Get command;command.Vision2Control(15000,11000,0,1,0);command.Nav2Control(-123,456);
      port.SendData(command);pollfd wait{master,POLLIN,0};
      if(poll(&wait,1,500)<=0)throw std::runtime_error("Mock transmit timeout");
      std::array<unsigned char,21> sent{};
      if(::read(master,sent.data(),sent.size())!=21 || std::memcmp(sent.data(),command.raw_data,21)!=0)
        throw std::runtime_error("Original SendData bytes changed");
    }
    close(master);close(slave);
    std::cout<<"PASS serial_mock: actual A SerialPort RX/TX on isolated PTY, fire=0"<<std::endl;
    return 0;
  }catch(const std::exception&e){if(master>=0)close(master);if(slave>=0)close(slave);std::cerr<<e.what()<<std::endl;return 1;}
}
