#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

// #include <cstring>
#include <sstream>
// #include <cstdlib>
#include <libserial/SerialPort.h>
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{


public:
  // Helper to parse two integers from a string like "e 602048     8717952"
  static bool parse_two_numbers(const std::string& s, double& out1, double& out2) {
    std::istringstream iss(s);
    char skip;
    if (!(iss >> skip >> out1 >> out2)) {
      std::cerr << "Failed to parse two integers from string: " << s << std::endl;
      return false;
    }
    
    return true;
  }

  ArduinoComms() = default;

  void start()
  {
    // Start the hardware and spawn the thread
    RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Starting thread...");
    this->is_running_ = true;

    this->worker_thread_ = std::thread(&ArduinoComms::backgroundTask, this);
  }

  void stop()
  {
    // Stop the hardware and clean up the thread
    RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Stopping thread...");
    this->is_running_ = false;

    if (this->worker_thread_.joinable())
    {
      this->worker_thread_.join();
    }
  }

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send, bool has_no_response = false, bool print_output = true)
  {
    // The block below is to test timing issues
    // static std::chrono::steady_clock::time_point last_call_time = std::chrono::steady_clock::now();
    // auto now = std::chrono::steady_clock::now();
    // auto duration_since_last_call = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_call_time).count();
    // if (duration_since_last_call <= 50) {
    //   return "";
    // }
    
    last_call_time = now;
    // std::cerr << "sending " << msg_to_send << std::endl ;
    if (this->sending == true) {
      // std::cerr << "won't send" << std::endl;
      // return "";
    }
    serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send + "\r");

    std::string response = "";
    if (!has_no_response) {
      try
      {
        // this->sending = true;
        // Responses end with \r\n so we will read up to (and including) the \n.
        //if (serial_conn_.IsDataAvailable()) {
        //std::cerr << "about to read" << std::endl;
        serial_conn_.ReadLine(response, '\n', timeout_ms_);
        // std::cerr << "response " << response << std::endl;
        //}
      }
      catch (const LibSerial::ReadTimeout&)
      {
        // this->sending = false;
        // std::cerr << "The ReadByte() call has timed out." << std::endl ;
      }
    }
    this->sending = false;

    if (print_output)
    {
      // std::cerr << "Sent: " << msg_to_send  << std::endl << " Recv: " << response << std::endl;
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  void microcontroller_read_encoder_values() {
    std::string response = send_msg("e\r");
    
    parse_two_numbers(response, this->encoder1, this->encoder2);

    //std::cerr << "parsed encoder values " << this->encoder1 << " " << this->encoder2 << std::endl;
  }

  void read_encoder_values(int &val_1, int &val_2)
  {
    val_1 = this->encoder1;
    val_2 = this->encoder2;

    // std::cerr << "read encoder values " << val_1 << " " << val_2 << std::endl;
  }

  void microcontroller_read_velocities() {
    std::string response = send_msg("v\r");
    
    parse_two_numbers(response, this->velocity1, this->velocity2);
    
    // std::cerr << "parsed velocities " << this->velocity1 << " " << this->velocity2 << std::endl;
  }

  void read_velocities(double &val_1, double &val_2)
  {
    val_1 = this->velocity1;
    val_2 = this->velocity2;
  }

  void set_motor_values(int val_1, int val_2) {
    // std::cerr << "setting motor values " << val_1 << " " << val_2 << std::endl;
    this->motor_target1 = val_1;
    this->motor_target2 = val_2;
  }

  void arduino_set_motor_values()
  {
    std::stringstream ss;
    ss << "m " << this->motor_target1 << " " << this->motor_target2 << "\r";
    send_msg(ss.str(), true);
  }

  void arduino_set_pid_values(){
    std::stringstream ss;
    ss << "u " << k_p << " " << k_i << " " << k_d << " " << "\r";
    send_msg(ss.str(), true);
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    this->k_p = k_p;
    this->k_d = k_d;
    this->k_i = k_i;
    this->k_o = k_o;

    std::stringstream ss;
    ss << "u " << k_p << " " << k_i << " " << k_d << " "  << "\r";
    send_msg(ss.str(), true);
  }

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    bool sending = false;
    double motor_target1 = 0;
    double motor_target2 = 0;
    double encoder1 = 0;
    double encoder2 = 0;
    double velocity1 = 0;
    double velocity2 = 0;
    int k_p;
    int k_d;
    int k_i;
    int k_o;
  std::thread worker_thread_;         // The worker thread
  std::atomic<bool> is_running_;      // Flag to control the thread

  void backgroundTask()
  {
    RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Worker thread started.");

    while (this->is_running_)
    {
      // Perform periodic tasks (e.g., polling sensors)
      // arduino_set_pid_values();
      // std::this_thread::sleep_for(std::chrono::milliseconds(600));

      microcontroller_read_encoder_values();

      microcontroller_read_velocities();

      // Keep this right above the sleep
      // The pico needs time to react to the motor commands
      arduino_set_motor_values();

      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    RCLCPP_INFO(rclcpp::get_logger("MyHardwareInterface"), "Worker thread stopped.");
  }

};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
