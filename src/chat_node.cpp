#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <iostream>
#include <string>

class ChatNode : public rclcpp::Node {
public:
  ChatNode(const std::string & node_id)
  : Node(node_id), node_id_(node_id) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chat", 10,
      std::bind(&ChatNode::topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "chat_app [%s] started", node_id_.c_str());
  }

  // 受信コールバック：自分の送信メッセージは無視し、受信時にプロンプトを再表示する
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string prefix = node_id_ + ": ";
    if (msg->data.rfind(prefix, 0) == 0) {
      // 自分の送信したメッセージの場合は無視
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Subscribe: %s", msg->data.c_str());
    // メッセージ受信後に入力プロンプトを再表示
    std::cout << "Enter your message (type 'exit' to exit): " << std::flush;
  }

  // メッセージ送信関数：送信時にノードIDを付与
  void send_message(const std::string & message) {
    auto msg = std_msgs::msg::String();
    msg.data = message + [node_id_];
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Publish: %s", msg.data.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string node_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // コマンドライン引数からノードIDを取得（指定がなければデフォルト名を使用）
  std::string node_id;
  if (argc > 1) {
    node_id = argv[1];
  } else {
    node_id = "chat_node_default";
  }

  auto node = std::make_shared<ChatNode>(node_id);

  // 別スレッドで spin を実行（受信コールバックの処理用）
  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  // メインスレッドでユーザー入力を受付
  std::string input;
  while (rclcpp::ok()) {
    std::cout << "Enter your message (type 'exit' to exit): " << std::flush;
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
