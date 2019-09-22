#include "moppy/moppy.h"

using namespace boost;

Moppy::Moppy() : DifferentialDrive(
    [this](std::pair<double,double> rates){this->commandSpeed(rates);},
    [this](std::pair<double,double> rates){this->commandMotorOutputs(rates);}),
    lidar_motor_mode_(MOTOR_MODE::PHASE_ENABLE), 
    right_motor_mode_(MOTOR_MODE::BREAK | MOTOR_MODE::IN_IN),
    left_motor_mode_(MOTOR_MODE::BREAK | MOTOR_MODE::IN_IN),
    ticks_per_rev_(0), wheel_diameter_(1), serial_port_(io_service_)
{}

Moppy::Moppy(const std::string& device, const int& baud) :
    Moppy()
{
    connect(device,baud);
    startReadThread();
}

void Moppy::setHardwareParams(const double& radius, const double& ticks_per_rev, const double& wheel_diameter)
{
    setCharacteristicRadius(radius);
    setTicksPerRev(ticks_per_rev);
    setWheelDiameter(wheel_diameter);
}

inline void Moppy::setTicksPerRev(const double& ticks_per_rev)
{
    ticks_per_rev_ = ticks_per_rev;
}

inline void Moppy::setWheelDiameter(const double& wheel_diameter)
{
    wheel_diameter_ = wheel_diameter;
}

void Moppy::setBatteryPublisher(std::function<void(const float&)> batt_pub)
{
    batt_pub_ = batt_pub;
}

void Moppy::setBumperPublisher(std::function<void(const std::vector<uint8_t>&)> bumper_pub)
{
    bumper_pub_ = bumper_pub;
}

void Moppy::setEdgePublisher(std::function<void(const std::vector<uint8_t>&)> edge_pub)
{
    edge_pub_ = edge_pub;
}

void Moppy::setOdomHandler(std::function<void(const Pose&, const Twist&)> odom_handler)
{
    odom_handler_ = odom_handler;
}

void Moppy::setMotorMode(const MOTOR& motor, const uint8_t& mode_mask)
{
    switch (motor) {
    case MOTOR::LIDAR:
        lidar_motor_mode_ = mode_mask & MOTOR_MODE::VALID_MASK;
        break;
    case MOTOR::RIGHT:
        right_motor_mode_ = mode_mask & MOTOR_MODE::VALID_MASK;
        break;
    case MOTOR::LEFT:
        left_motor_mode_ = mode_mask & MOTOR_MODE::VALID_MASK;
        break;
    }
}

void Moppy::commandMotorOutputs(const std::pair<double,double>& rates) const
{
    std::cout << "open loop: " << rates.first << " " << rates.second << std::endl;
    commandMotorOpenLoop(MOTOR::RIGHT,rates.first);
    commandMotorOpenLoop(MOTOR::LEFT,rates.second);
}

void Moppy::commandSpeed(const std::pair<double,double>& rates) const
{
    std::cout << "closed loop speed: " << rates.first << " " << rates.second << std::endl;
    commandMotorClosedLoop(MOTOR::RIGHT,rates.first/(wheel_diameter_*M_PIl));
    commandMotorClosedLoop(MOTOR::LEFT,rates.second/(wheel_diameter_*M_PIl));
}

class MotorCmd
{
public:
    MotorCmd(const char& key)
    {
        data_[0] = 0xAA;
        setKey(key);
    }

    void setKey(const char& key)
    {
        data_[1] = key;
    }

    void setMotor(const uint8_t& motor)
    {
        data_[2] = motor;
    }

    void setMotor(const Moppy::MOTOR& motor)
    {
        data_[2] = uint8_t(motor);
    }

    void setMode(const uint8_t& mode)
    {
        data_[3] = mode;
    }

    void setRate(const float& rate)
    {
        reinterpret_cast<float&>(data_[4]) = rate;
    }

    std::array<uint8_t,8> serialize()
    {
        return data_;
    }

protected:
    std::array<uint8_t,8> data_;

};

void Moppy::commandMotorOpenLoop(const MOTOR& motor, float rate) const
{
    MotorCmd cmd('M');
    cmd.setMotor(motor);

    switch (motor) {
    case MOTOR::LIDAR:
        cmd.setMode(lidar_motor_mode_);
        break;
    case MOTOR::RIGHT:
        cmd.setMode(right_motor_mode_);
        break;
    case MOTOR::LEFT:
        cmd.setMode(left_motor_mode_);
        break;
    }
    if (rate > 1) rate = 1;
    if (rate < -1) rate = -1;
    cmd.setRate(rate);

    auto data = cmd.serialize();
    asio::write(serial_port_,asio::buffer(data.data(),data.size()));
}

void Moppy::commandMotorClosedLoop(const MOTOR& motor, const float& rate) const
{
    MotorCmd cmd('R');
    cmd.setMotor(motor);

    switch (motor) {
    case MOTOR::LIDAR:
        cmd.setMode(lidar_motor_mode_);
        break;
    case MOTOR::RIGHT:
        cmd.setMode(right_motor_mode_);
        break;
    case MOTOR::LEFT:
        cmd.setMode(left_motor_mode_);
        break;
    }
    cmd.setRate(rate);

    auto data = cmd.serialize();
    std::cout << std::hex << std::setw(2);
    for (const auto& c : data) {
        std::cout << uint32_t(c) << " ";
    }
    std::cout << std::dec << std::setw(0) << std::endl;
    asio::write(serial_port_,asio::buffer(data.data(),data.size()));
}


bool Moppy::connect(const std::string& device, const size_t& baudrate, const double& timeout)
{
    auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
    //spin forever or until deadline
    while (timeout == 0.0 || std::chrono::steady_clock::now() < deadline) {
        //if success return true
        if (connectOnce(device, baudrate)) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    //guess we failed with a timeout
    return false;
}

bool Moppy::connectOnce(const std::string& device, const size_t& baudrate) {
    try {
        serial_port_.open(device);
        serial_port_.set_option(asio::serial_port::baud_rate(uint(baudrate)));
        serial_port_.set_option(
            asio::serial_port::parity(asio::serial_port::parity::none));
        serial_port_.set_option(asio::serial_port::character_size(
            asio::serial_port::character_size(8)));
        serial_port_.set_option(
            asio::serial_port::stop_bits(asio::serial_port::stop_bits::one));
    }
    catch(const system::error_code& e) {
             const int ecode = e.value();
             std::cout << "Error when opening '" << device << "': ";
             std::cout << e.category().message(ecode) <<" (error code: ";
             std::cout << std::to_string(ecode) << ")" << std::endl;;
    }
    return isConnected();
}

bool Moppy::disconnect() {
    system::error_code ec;
    serial_port_.close(ec);
    if(ec) return false;
    return true;
}

bool Moppy::isConnected() const 
{
    return serial_port_.is_open(); 
}

bool Moppy::startReadThread() 
{
    // no point reading if we arent connected to anything
    if(!isConnected()) return false;
    // can't start if we are already running
    if(read_thread_running_.test_and_set()) return false;
    // hit it!
    io_thread_ = std::thread([this] {
        queueAsyncRead();
        io_service_.run();
    });
    return true;
}

bool Moppy::stopReadThread() 
{
    bool rc = false;
    if(read_thread_running_.test_and_set()) {
        io_service_.stop();
        io_thread_.join();
        io_service_.reset();
        rc = true;
    }
    read_thread_running_.clear();
    return rc;
}

inline void Moppy::queueAsyncRead()
{
    asio::async_read_until(
        serial_port_, buffer_,
        std::bind(&Moppy::messageReady,
            this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&Moppy::processOneMessage,
            this, std::placeholders::_1, std::placeholders::_2) );
}

std::pair<iterator, bool> Moppy::messageReady(iterator begin, iterator end) const 
{
    const size_t available = size_t(std::distance(begin, end));

    if (available == 0) return std::make_pair(begin,false);

    iterator i = begin;
    if ( *i == 0xAA ) {
        if (available >= 27 ) {
            std::advance(i, 27);
            return std::make_pair(i,true);
        } else {
            return std::make_pair(begin,false);
        }
    }
    while( *i != 0xAA && i != end) {
        ++i;
    }

    return std::make_pair(i, true);

}

uint8_t Moppy::extractChar() {
    if(buffer_.sgetc() == EOF) {
        asio::read(serial_port_, buffer_, asio::transfer_exactly(1));
    }
    return uint8_t(buffer_.sbumpc());
}


void Moppy::processOneMessage(const boost::system::error_code& ec, const std::size_t& bytes_transferred)
{
    if(ec == asio::error::operation_aborted) {
        // operation_aborted error probably means the client is being closed
        // notify waiting request methods
        return;
    }

    const uint8_t msg_marker = extractChar();
    if(msg_marker != 0xAA) {
        buffer_.consume(bytes_transferred-1);
        queueAsyncRead();
        return;
    }

    uint32_t elapsed_micros;
    int32_t right_ticks, left_ticks;
    float right_rate, left_rate, batt_voltage;
    buffer_.sgetn(reinterpret_cast<char*>(&elapsed_micros),sizeof(elapsed_micros));
    buffer_.sgetn(reinterpret_cast<char*>(&right_ticks),sizeof(right_ticks));
    buffer_.sgetn(reinterpret_cast<char*>(&left_ticks),sizeof(left_ticks));

    if (right_ticks || left_ticks) {
        auto step = trackDistToMovement(
            right_ticks/ticks_per_rev_*wheel_diameter_*M_PIl ,
            left_ticks/ticks_per_rev_*wheel_diameter_*M_PIl );
        accumulator_.increment(elapsed_micros/1e-9, step.first, 0, step.second);

        if (odom_handler_) odom_handler_(accumulator_.getPose(),accumulator_.getTwist());
        //buffer_.sgetn(reinterpret_cast<char*>(&right_rate),sizeof(right_rate));
        //buffer_.sgetn(reinterpret_cast<char*>(&left_rate),sizeof(left_rate));
    }

    buffer_.sgetn(reinterpret_cast<char*>(&batt_voltage),sizeof(batt_voltage));
    if (batt_pub_) batt_pub_(batt_voltage);

    std::vector<uint8_t> bump;
    bump.emplace_back(extractChar());
    bump.emplace_back(extractChar());
    bump.emplace_back(extractChar());
    bump.emplace_back(extractChar());
    if (bumper_pub_) bumper_pub_(bump);

    std::vector<uint8_t> edge;
    edge.emplace_back(extractChar());
    edge.emplace_back(extractChar());
    if (edge_pub_) edge_pub_(edge);

    queueAsyncRead();
}
