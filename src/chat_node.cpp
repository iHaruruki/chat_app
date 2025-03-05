#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <iostream>
#include <string>

class ChatNode : public rclcpp::Node {
public:
  // コンストラクタでノード固有のID（node_id）を受け取る
  ChatNode(const std::string & node_id)
  : Node(node_id), node_id_(node_id) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("chat", 10);
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chat", 10,
      std::bind(&ChatNode::topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "チャットノード [%s] が起動しました", node_id_.c_str());
  }

  // 受信コールバック：自分自身の送信メッセージは無視する
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    // 自分のIDがメッセージの先頭に含まれているかチェック
    std::string prefix = node_id_ + ": ";
    if (msg->data.rfind(prefix, 0) == 0) {
      // 自分自身のメッセージなら何もせず戻る
      return;
    }
    RCLCPP_INFO(this->get_logger(), "受信: '%s'", msg->data.c_str());
  }

  // メッセージ送信関数：送信メッセージの先頭に自分のIDを付与
  void send_message(const std::string & message) {
    auto msg = std_msgs::msg::String();
    msg.data = node_id_ + ": " + message;
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "送信: '%s'", msg.data.c_str());
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::string node_id_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ノードのIDはコマンドライン引数から指定できるようにする
  std::string node_id;
  if (argc > 1) {
    node_id = argv[1];
  } else {
    // 複数の端末で同じノード名だと区別がつかないため、必ず異なる名前を与える
    node_id = "chat_node_default";
  }

  auto node = std::make_shared<ChatNode>(node_id);

  // 別スレッドで spin を実行（非同期で受信コールバックを処理）
  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  // メインスレッドでユーザー入力を受け付け、送信処理を実施
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
