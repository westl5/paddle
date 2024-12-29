#pragma once
#ifndef INCLUDED_SERIAL_PORT_HPP
#define INCLUDED_SERIAL_PORT_HPP

#define ARDUINO_WAIT_TIME 2000
#define MAX_DATA_LENGTH 255

//handles mac and windows
#include "serial/i_serial_device.hpp"
#include <string>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <termios.h>
    #include <fcntl.h>
#endif

class serial_port: public i_serial_device
{
private:
  static const constexpr uint32_t ArduinoWaitTime = 2000;
  static const constexpr size_t MaxDataLength = 255;
  //handles mac and windows
  #ifdef _WIN32
        HANDLE m_handler;
        COMSTAT m_status;
        DWORD m_errors;
    #else
        int m_fd;
        struct termios m_tty;
    #endif

    bool m_connected;

public:
  serial_port(const std::string &port_name, unsigned baud_rate = 9600);
  ~serial_port();
  uint32_t read(uint8_t *buffer, uint32_t buf_size) override;
  bool write(const uint8_t *buffer, uint32_t buf_size) override;
  bool is_connected() const override;
};

#endif  // INCLUDED_SERIAL_PORT_HPP
