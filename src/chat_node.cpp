#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <iostream>
#include <string>

class ChatNode : public rclcpp::Node {
public:
  ChatNode() : Node("chat_node") {
    // "chat" トピックへのパブリッシャーを作成
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);
    // "chat" トピックからのメッセージを受信するサブスクリプションを作成
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chat", 10,
      std::bind(&ChatNode::topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "チャットノードが起動しました");
  }

  // 受信したメッセージのコールバック関数
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "受信: '%s'", msg->data.c_str());
  }

  // メッセージを送信する関数
  void send_message(const std::string & message) {
    auto msg = std_msgs::msg::String();
    msg.data = message;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "送信: '%s'", msg.data.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChatNode>();

  // 別スレッドでspinを実行して、コールバック処理を非同期で行う
  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  // 端末からの入力ループ
  std::string input;
  while (rclcpp::ok()) {
    std::cout << "メッセージを入力してください (終了するには 'exit' と入力): ";
    std::getline(std::cin, input);
    if (input == "exit") {
      break;
    }
    node->send_message(input);
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
