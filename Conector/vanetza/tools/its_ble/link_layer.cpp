#include "link_layer.hpp"
#include "raw_socket_link.hpp"
#include "uart_link_rx.hpp"
#include "uart_link_tx.hpp"
#include <vanetza/access/ethertype.hpp>
#include <boost/asio/generic/raw_protocol.hpp>
#include <iostream>

std::unique_ptr<LinkLayer>
create_link_layer_tx(boost::asio::io_service& io_service, const std::string& serial_port_tx)
{
    std::unique_ptr<LinkLayer> link_layer;
    link_layer.reset(new UartLink_tx {io_service, serial_port_tx});
   
    return link_layer;
}

std::unique_ptr<LinkLayer>
create_link_layer_rx(boost::asio::io_service& io_service, const std::string& serial_port_rx)
{
    std::unique_ptr<LinkLayer> link_layer;
    link_layer.reset(new UartLink_rx { io_service, serial_port_rx});
   
    return link_layer;
}

void LinkLayer::mostrar(ByteBuffer::iterator inicio, ByteBuffer::iterator final){
    ByteBuffer::iterator aux = inicio;
    while(aux != final){
        printf("%02x ", *aux);
        aux++;
    }
    std::cout<<std::endl;
}


void LinkLayer::setDTR(boost::asio::serial_port &serial){
    int data = TIOCM_DTR; 
    int fd = serial.native_handle();    
    int err = ioctl(fd, TIOCMBIS, &data);
    if(err == -1){
        std::cerr<<"Error setDTR"<<std::endl;
    }
}

void LinkLayer::set(boost::asio::serial_port &serial){
    int status;
    int fd = serial.native_handle();
    ioctl(fd, TIOCMGET, &status);
    status |= (TIOCM_DTR); 
    int err = ioctl(fd, TIOCMBIC, &status);
    if(err == -1){
        std::cerr<<"Error setDTR"<<std::endl;
    }
}


int LinkLayer::get(boost::asio::serial_port &serial){
    int status = 0;
    int fd = serial.native_handle();
    ioctl(fd, TIOCMGET, &status);
    return status & TIOCM_DTR;
}
