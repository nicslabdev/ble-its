#include <iostream>
#include "log.hpp"


namespace log_module{
    void print_log(std::string s){
#ifdef ITS_BLE_LOG
        std::cout << "<log info> " << s << std::endl; 
#endif
    }
}

