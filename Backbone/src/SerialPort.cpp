#include <unistd.h>  // UNIX Standard Definitions
#include <fcntl.h>   // File Control Definitions
#include <errno.h>   // ERROR Number Definitions
#include <termios.h> // POSIX Terminal Control Definitions
#include "SerialPort.h"
#include "Coordinate.h"
std::mutex ds_mutex;
Data_Save ds;
void test(double a, double b);
static double pit = 0.0, yaw = 0.0;
static std::chrono::steady_clock::time_point last_time;
bool DEBUG_SERIAL1 =true;
Data_Get dg; // Serial_port

SerialPort::SerialPort(const char *filename)
{
    file_name_ = filename;
    buadrate_ = B115200;
    success_ = false;
    //    serial_mode = NO_INIT;
    fd = open(file_name_, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port                                           // No terminal will control the process
    last_fd = fd;
    if (fd == -1)
    {
#ifdef DEBUG_SERIAL
        printf("open_port wait to open %s .\n", file_name_);
#endif
        //  NOTICE("wait serial " << file_name_,1);
        return;
    }
    else if (fd != -1)
    {
        fcntl(fd, F_SETFL, 0);
        // fcntl(fd, F_SETFL, FNDELAY); // no late

#ifdef DEBUG_SERIAL
        printf("port is open %s.\n", file_name_);
#endif
    }
    struct termios port_settings;         // structure to store the port settings in
    cfsetispeed(&port_settings, B115200); // set baud rates

    cfsetospeed(&port_settings, B115200);
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK; // disable break processing
    port_settings.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;

    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port

    //    char write_buffer[]="hello word";
    //    int bytes_written=0;
    //    bytes_written=write(fd,write_buffer,sizeof(write_buffer));
}

int open_name = 0;

void SerialPort::restart_serial(void)
{
    //    cout << "test restart !!" << fd << " " << last_fd << endl;
    close(fd);
    if (open_name > 10)
    {
        open_name = 0;
    }
    if (open_name % 2 == 1)
    {
        fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
    }
    else
    {
        fd = open(SERIAL_PATH, O_RDWR | O_NOCTTY | O_SYNC); // Read/Write access to serial port
    }
    //    cout << serial_mode << endl;
    if (fd == -1 && last_fd != -1)
    {
#ifdef DEBUG_SERIAL
        printf("open_port wait to open %s .\n", file_name_);
#endif
        //        NOTICE("wait serial",1);
        last_fd = fd;
        return;
    }
    else if (fd != -1 && last_fd == -1)
    {
        fcntl(fd, F_SETFL, 0);
        // fcntl(fd, F_SETFL, FNDELAY); // no late

        //        NOTICE("port is open",1);
#ifdef DEBUG_SERIAL
        printf("port is open %s.\n", file_name_);
#endif
        last_fd = fd;
    }
    else
    {
        last_fd = fd;
        return;
    }
    struct termios port_settings; // structure to store the port settings in
        cfsetispeed(&port_settings, B115200); // set baud rates
        cfsetospeed(&port_settings, B115200);
    port_settings.c_cflag = (port_settings.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    port_settings.c_iflag &= ~IGNBRK; // disable break processing
    port_settings.c_lflag = 0;        // no signaling chars, no echo,
    // no canonical processing
    port_settings.c_oflag = 0;                // no remapping, no delays
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    port_settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    port_settings.c_cflag |= (CLOCAL | CREAD); // ignore modem controls,
    // enable reading
    port_settings.c_cflag &= ~(PARENB | PARODD); // shut off parity
    port_settings.c_cflag |= 0;
    port_settings.c_cflag &= ~CSTOPB;
    port_settings.c_cflag &= ~CRTSCTS;
    port_settings.c_iflag = ICANON;
    port_settings.c_cc[VMIN] = SERIAL_VMIN;   // read doesn't block
    port_settings.c_cc[VTIME] = SERIAL_VTIME; // 0.5 seconds read timeout

    tcsetattr(fd, TCSANOW, &port_settings); // apply the settings to the port
}

// read the datas,which is enermy color, aim mode and camera vertical angle
int SerialPort::read_data()
{ 
    
    tcflush(fd, TCIFLUSH);                /* Discards old data in the rx buffer             */
    unsigned char read_buffer[READ_SIZE]; /* Buffer to store the data received              */
    long bytes_read = 0;                  /* Number of bytes read by the read() system call */

    bytes_read = read(fd, read_buffer, READ_SIZE); /* Read the data   */    

    if (bytes_read == -1 || bytes_read == 0)
    {
        restart_serial();
        // cout<<"        串口未连接          "<<endl; 
    }
    else
    {
        if (read_buffer[0] == 0xAA && read_buffer[1] == 0xBB)
        {
            std::lock_guard<std::mutex> lock(ds_mutex);
            int32_t yaw_ = read_buffer[3] << 24 | read_buffer[4] << 16 | read_buffer[5] << 8 | read_buffer[6]; // 云台yaw
            int32_t pit_ = read_buffer[7] << 24 | read_buffer[8] << 16 | read_buffer[9] << 8 | read_buffer[10]; // 云台pitch
            int32_t roll_ = read_buffer[11] << 24 | read_buffer[12] << 16 | read_buffer[13] << 8 | read_buffer[14];
            ds.camera_yaw_angle = _std_radian(yaw_ / 10000.0 * PI / 180);
            ds.camera_pit_angle = pit_ / 10000.0 * PI / 180;
            ds.camera_roll_angle = _std_radian(roll_ / 10000.0 * PI / 180 -PI);
            ds.color = read_buffer[15];
            ds.Buff_or_Armor = read_buffer[16];
            C_C.GetTFMat(ds.camera_roll_angle, -ds.camera_pit_angle, ds.camera_yaw_angle);
        }
        else
        {
            if (DEBUG_SERIAL1 == true)
            {
                cout << "this is not the data I want ! " << endl;
            }
        }

    }
    
}

void SerialPort::send_data(const struct Data_Get &data)
{
    if (data.size != write(fd, data.raw_data, data.size))
    {
        open_name++;

        if (DEBUG_SERIAL1 == true)
        {
            // cout << "!!! send data failure !!!" << fd << "restart fd" << fd << endl;
        }
        restart_serial();
    }
    else
    {
        if (DEBUG_SERIAL1 == true)
        {
            // cout << "!!! send data success !!!" << fd << endl;
        }
    }
}

extern int Turn_back;
int DEDEDE(0);
extern int TRACKING;
void Data_Get::Vision2Control(int32_t x, int32_t y,int32_t ui_x, int32_t ui_y, uint8_t found, uint8_t fire)
{
    size = 21;

    raw_data[0] = 0xAA;
    raw_data[1] = 0xAA;
    raw_data[2] = (x >> 24) & 0xff;
    raw_data[3] = (x >> 16) & 0xff;
    raw_data[4] = (x >> 8) & 0xff;
    raw_data[5] = (x >> 0) & 0xff;
    raw_data[6] = (y >> 24) & 0xff;
    raw_data[7] = (y >> 16) & 0xff;
    raw_data[8] = (y >> 8) & 0xff;
    raw_data[9] = (y >> 0) & 0xff;
    raw_data[10] = (ui_x >> 24) & 0xff;
    raw_data[11] = (ui_x >> 16) & 0xff;
    raw_data[12] = (ui_x >> 8) & 0xff;
    raw_data[13] = (ui_x >> 0) & 0xff;
    raw_data[14] = (ui_y >> 24) & 0xff;
    raw_data[15] = (ui_y >> 16) & 0xff;
    raw_data[16] = (ui_y >> 8) & 0xff;
    raw_data[17] = (ui_y >> 0) & 0xff;
    raw_data[18] = found;
    raw_data[19] = fire;
    raw_data[20] = 0xBB;
    // cout << " yaw : " << x*1e-4 << "    pit : " << y*1e-4 <<" ui_x : "<<ui_x<<"  ui_y : "<<ui_y<< " found : " << static_cast<int>(found) << "   fire: " << static_cast<int>(fire)<< endl;
    
}

// void Data_Get::Vision2Control()
// {
//     size = 21;
//     MAI_raw_data[0] = 0xAA;
//     MAI_raw_data[1] = 0xAA;
//     MAI_raw_data[2] = 0xff;
//     MAI_raw_data[3] = 0xff;
//     MAI_raw_data[4] = 0xff;
//     MAI_raw_data[5] = 0xff;
//     MAI_raw_data[6] = 0xff;
//     MAI_raw_data[7] = 0xff;
//     MAI_raw_data[8] = 0xff;
//     MAI_raw_data[9] = 0xff;
//     MAI_raw_data[10] = 0xff;
//     MAI_raw_data[11] = 0xff;
//     MAI_raw_data[12] = 0xff;
//     MAI_raw_data[13] = 0xff;
//     MAI_raw_data[14] = 0xff;
//     MAI_raw_data[15] = 0xff;
//     MAI_raw_data[16] = 0xff;
//     MAI_raw_data[17] = 0xff;
//     MAI_raw_data[18] = 2;
//     MAI_raw_data[19] = 0xff;
//     MAI_raw_data[20] = 0xBB;
// }

void test(double a, double b)
{
    static int16_t pitch_connt, yaw_connt = 0;
    static float pitch_angle, last_pitch_angle, yaw_angle, last_yaw_angle = 0;

    // 陀螺仪原始数据被乘了100倍
    pitch_angle = static_cast<float>(b) / 100;
    yaw_angle = static_cast<float>(a) / 100;

    // 将角度改成连续的，不是360°变回0°
    if ((pitch_angle - last_pitch_angle) > 330)
        pitch_connt--;
    else if ((pitch_angle - last_pitch_angle) < -330)
        pitch_connt++;

    if ((yaw_angle - last_yaw_angle) > 330)
        yaw_connt--;
    else if ((yaw_angle - last_yaw_angle) < -330)
        yaw_connt++;

    pit = static_cast<double>(pitch_angle);
    last_pitch_angle = pitch_angle;
    yaw = static_cast<double>(yaw_angle + yaw_connt * 360);
    last_yaw_angle = yaw_angle;

    //    cout << yaw << "    " << pit << endl;
}
