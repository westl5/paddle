#include "serial/serial_port.hpp"
#include "util/format.hpp"
#include <stdexcept>
#include <algorithm>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <fcntl.h>
    #include <unistd.h>
    #include <errno.h>
    #include <termios.h>
    #include <sys/ioctl.h>
#endif

serial_port::serial_port(const std::string &port_name, unsigned baud_rate)
    : m_connected(false)
{
#ifdef _WIN32
    m_handler = CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, 
                          OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (m_handler == INVALID_HANDLE_VALUE) {
        throw std::runtime_error(util::format() << "Unable to open '" << port_name << "'");
    }
    
    DCB dcb_serial_params = {0};
    if (!GetCommState(m_handler, &dcb_serial_params)) {
        throw std::runtime_error("Failed to get serial parameters");
    }
    
    dcb_serial_params.BaudRate = baud_rate;
    dcb_serial_params.ByteSize = 8;
    dcb_serial_params.StopBits = ONESTOPBIT;
    dcb_serial_params.Parity = NOPARITY;
    
    if (!SetCommState(m_handler, &dcb_serial_params)) {
        throw std::runtime_error("Failed to set serial parameters");
    }
#else
    m_fd = open(port_name.c_str(), O_RDWR);
    if (m_fd < 0) {
        throw std::runtime_error(util::format() << "Error opening port: " << strerror(errno));
    }

    if (tcgetattr(m_fd, &m_tty) != 0) {
        throw std::runtime_error("Error from tcgetattr");
    }

    cfsetospeed(&m_tty, baud_rate);
    cfsetispeed(&m_tty, baud_rate);

    m_tty.c_cflag &= ~PARENB;
    m_tty.c_cflag &= ~CSTOPB;
    m_tty.c_cflag &= ~CSIZE;
    m_tty.c_cflag |= CS8;
    m_tty.c_cflag &= ~CRTSCTS;
    m_tty.c_cflag |= CREAD | CLOCAL;

    if (tcsetattr(m_fd, TCSANOW, &m_tty) != 0) {
        throw std::runtime_error("Error from tcsetattr");
    }
#endif
    m_connected = true;
}

serial_port::~serial_port()
{
#ifdef _WIN32
    if (m_connected) {
        CloseHandle(m_handler);
    }
#else
    if (m_connected) {
        close(m_fd);
    }
#endif
    m_connected = false;
}

uint32_t serial_port::read(uint8_t *buffer, uint32_t buf_size)
{
#ifdef _WIN32
    DWORD bytes_read = 0;
    ClearCommError(m_handler, &m_errors, &m_status);
    if (m_status.cbInQue > 0) {
        if (ReadFile(m_handler, buffer, buf_size, &bytes_read, NULL)) {
            return bytes_read;
        }
    }
    return 0;
#else
    int bytes_available;
    if (ioctl(m_fd, FIONREAD, &bytes_available) < 0) {
        return 0;
    }

    if (bytes_available > 0) {
        return ::read(m_fd, buffer, std::min(buf_size, (uint32_t)bytes_available));
    }
    return 0;
#endif
}

bool serial_port::write(const uint8_t *buffer, uint32_t buf_size)
{
#ifdef _WIN32
    DWORD bytes_sent;
    if (!WriteFile(m_handler, buffer, buf_size, &bytes_sent, 0)) {
        ClearCommError(m_handler, &m_errors, &m_status);
        return false;
    }
    return true;
#else
    return ::write(m_fd, buffer, buf_size) == buf_size;
#endif
}

bool serial_port::is_connected() const
{
    return m_connected;
}