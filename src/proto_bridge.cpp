#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/msg/string.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <set>
#include <mutex>

using json = nlohmann::json;
using namespace std::chrono_literals;

typedef websocketpp::server<websocketpp::config::asio> server;
typedef server::message_ptr message_ptr;

class ProtoBridgeCpp : public rclcpp::Node
{
public:
    ProtoBridgeCpp() : Node("proto_bridge_cpp"), port_(9090)
    {
        // Define QoS settings
        rclcpp::QoS qos_settings(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
        qos_settings.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_settings.durability(rclcpp::DurabilityPolicy::Volatile);

        // ROS subscriptions with explicit QoS
        can_sub_ = create_subscription<can_msgs::msg::Frame>(
            "/can/raw", qos_settings,
            std::bind(&ProtoBridgeCpp::can_callback, this, std::placeholders::_1));
        
        robot_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot_state", qos_settings,
            std::bind(&ProtoBridgeCpp::robot_state_callback, this, std::placeholders::_1));
        
        diagnostics_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticStatus>(
            "/diagnostics", qos_settings,
            std::bind(&ProtoBridgeCpp::diagnostics_callback, this, std::placeholders::_1));

        // Start WebSocket server in separate thread
        websocket_thread_ = std::thread(&ProtoBridgeCpp::run_websocket_server, this);
        
        RCLCPP_INFO(get_logger(), "C++ Protocol Bridge initialized on port %d", port_);
    }

    ~ProtoBridgeCpp()
    {
        // Stop server
        server_.stop();
        if (websocket_thread_.joinable()) {
            websocket_thread_.join();
        }
    }

private:
    void run_websocket_server()
    {
        try {
            // Set logging settings
            server_.set_access_channels(websocketpp::log::alevel::all);
            server_.clear_access_channels(websocketpp::log::alevel::frame_payload);
            
            // Initialize Asio
            server_.init_asio();
            
            // Set message handlers
            server_.set_message_handler(
                std::bind(&ProtoBridgeCpp::on_message, this, 
                         std::placeholders::_1, std::placeholders::_2));
            
            server_.set_open_handler(
                std::bind(&ProtoBridgeCpp::on_open, this, std::placeholders::_1));
            
            server_.set_close_handler(
                std::bind(&ProtoBridgeCpp::on_close, this, std::placeholders::_1));

            // Listen on port
            server_.listen(port_);
            server_.start_accept();

            RCLCPP_INFO(get_logger(), "WebSocket server listening on port %d", port_);
            
            // Run the server
            server_.run();
        }
        catch (websocketpp::exception const & e) {
            RCLCPP_ERROR(get_logger(), "WebSocket error: %s", e.what());
        }
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        connections_.insert(hdl);
        
        // Send welcome message
        json welcome = {
            {"type", "welcome"},
            {"timestamp", get_timestamp_ns()},
            {"message", "Connected to Evabot C++ Protocol Bridge"},
            {"available_topics", {"/can/raw", "/robot_state", "/diagnostics"}}
        };
        
        send_to_client(hdl, welcome.dump());
        RCLCPP_INFO(get_logger(), "Client connected");
    }

    void on_close(websocketpp::connection_hdl hdl)
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        connections_.erase(hdl);
        RCLCPP_INFO(get_logger(), "Client disconnected");
    }

    void on_message(websocketpp::connection_hdl /*hdl*/, message_ptr msg)
    {
        try {
            auto data = json::parse(msg->get_payload());
            
            if (data["type"] == "subscribe") {
                std::string topic = data["topic"];
                RCLCPP_INFO(get_logger(), "Client subscribed to %s", topic.c_str());
            }
            else if (data["type"] == "publish") {
                std::string topic = data["topic"];
                RCLCPP_INFO(get_logger(), "Client wants to publish to %s", topic.c_str());
                // TODO: Implement ROS publishing from client
            }
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Error processing client message: %s", e.what());
        }
    }

    void can_callback(const can_msgs::msg::Frame::SharedPtr msg)
    {
        json can_data = {
            {"type", "can_frame"},
            {"topic", "/can/raw"},
            {"timestamp", get_timestamp_ns()},
            {"data", {
                {"id", msg->id},
                {"dlc", msg->dlc},
                {"data", std::vector<uint8_t>(msg->data.begin(), msg->data.end())},
                {"is_extended", msg->is_extended},
                {"is_error", msg->is_error},
                {"is_rtr", msg->is_rtr}
            }}
        };
        
        broadcast_message(can_data.dump());
    }

    void robot_state_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        json robot_data = {
            {"type", "robot_state"},
            {"topic", "/robot_state"},
            {"timestamp", get_timestamp_ns()},
            {"data", msg->data}
        };
        
        broadcast_message(robot_data.dump());
    }

    void diagnostics_callback(const diagnostic_msgs::msg::DiagnosticStatus::SharedPtr msg)
    {
        json diagnostic_data = {
            {"type", "diagnostic"},
            {"topic", "/diagnostics"},
            {"timestamp", get_timestamp_ns()},
            {"data", {
                {"level", msg->level},
                {"name", msg->name},
                {"message", msg->message},
                {"hardware_id", msg->hardware_id}
            }}
        };
        
        broadcast_message(diagnostic_data.dump());
    }

    void broadcast_message(const std::string& message)
    {
        std::lock_guard<std::mutex> lock(connections_mutex_);
        
        for (auto hdl : connections_) {
            send_to_client(hdl, message);
        }
    }

    void send_to_client(websocketpp::connection_hdl hdl, const std::string& message)
    {
        try {
            server_.get_alog().write(websocketpp::log::alevel::app, message);
            server_.send(hdl, message, websocketpp::frame::opcode::text);
        }
        catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "Error sending to client: %s", e.what());
        }
    }

    uint64_t get_timestamp_ns()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    // ROS subscriptions
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_sub_;

    // WebSocket server
    server server_;
    std::thread websocket_thread_;
    int port_;
    
    // Connection management
    std::set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections_;
    std::mutex connections_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ProtoBridgeCpp>();
    
    try {
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
