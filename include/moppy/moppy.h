#include <thread>
#include "boost/asio.hpp"
#include "hardware_abstraction/differential_drive.hpp"
#include "hardware_abstraction/odometry_accumulator.hpp"

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> iterator;

class Moppy : public DifferentialDrive
{
public:

    enum MOTOR
    {
        LIDAR = 0,
        RIGHT = 1,
        LEFT =  2
    };

    enum MOTOR_MODE
    {
        COAST = 1<<2, //4
        BREAK = 1<<3, //8
        IN_IN = 1<<4, //16
        PHASE_ENABLE = 1<<5, //32
        VALID_MASK = COAST | BREAK | IN_IN | PHASE_ENABLE
    };

    Moppy();
    ~Moppy();

    Moppy(const std::string& device, const int& baud);

    void setHardwareParams(const double& characteristic_radius, const double& ticks_per_rev, const double& wheel_diameter);

    //set the ticks per meter coefficient for scaling velocity to quadrature encoding rate
    void setTicksPerRev(const double& ticks_per_rev);

    void setWheelDiameter(const double& wheel_diameter);

    void setMotorMode(const MOTOR& motor, const uint8_t& mode_mask);

    void setBatteryPublisher(std::function<void(const float&)> bat_pub);

    void setBumperPublisher(std::function<void(const std::vector<uint8_t>&)> bumper_pub);

    void setEdgePublisher(std::function<void(const std::vector<uint8_t>&)> edge_pub);

    void setOdomHandler(std::function<void(const Pose&, const Twist&)> odom_handler);

    //command a fractional output (-1 to 1)
    void commandMotorOutputs(const std::pair<double,double>& rates) const;

    //command a particular velocity in m/s
    void commandSpeed(const std::pair<double,double>& rates) const;

    //command a fractional output from the motor controller (-1 to 1)
    void commandMotorOpenLoop(const MOTOR& motor, float rate) const;

    //command a particular wheel revolution rate in turns/s
    void commandMotorClosedLoop(const MOTOR& motor, const float& rate) const;

    void sendLidarFeedback(const float& speed) const;

    void setPIDCoefficients(const MOTOR& motor, const float& P, const float& I, const float& D) const;

    bool connect(const std::string& device, const size_t& baudrate, const double& timeout = 0.0);

    bool connectOnce(const std::string& device, const size_t& baudrate);

    bool disconnect();

    bool isConnected() const;

    bool startReadThread();

    bool stopReadThread();

    void queueAsyncRead();

    std::pair<iterator, bool> messageReady(iterator begin, iterator end) const;

    uint8_t extractChar();

    void processOneMessage(const boost::system::error_code& ec, const std::size_t& bytes_transferred);

private:
    uint8_t lidar_motor_mode_;
    uint8_t right_motor_mode_;
    uint8_t left_motor_mode_;
    double ticks_per_rev_;
    double wheel_diameter_;

    std::function<void(const float&)> batt_pub_;
    std::function<void(const std::vector<uint8_t>&)> bumper_pub_;
    std::function<void(const std::vector<uint8_t>&)> edge_pub_;
    std::function<void(const Pose&, const Twist&)> odom_handler_;

    boost::asio::io_service io_service_;     ///<! io service
    mutable boost::asio::serial_port serial_port_;  ///<! port for serial device
    boost::asio::streambuf buffer_;

    // read thread management
    std::thread io_thread_;
    std::atomic_flag read_thread_running_ = ATOMIC_FLAG_INIT;

    OdometryAccumulator accumulator_;
};

