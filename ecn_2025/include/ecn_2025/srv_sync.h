#ifndef SRV_SYNC_H
#define SRV_SYNC_H

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

template <class ServiceT>
class ServiceNodeSync
{
  typedef typename ServiceT::Request RequestT;
  typedef typename ServiceT::Response ResponseT;
public:
  ServiceNodeSync() {}

  void init(std::string name, std::string service, std::chrono::milliseconds timeout = 100ms)
  {
    node = std::make_shared<rclcpp::Node>(name);
    client = node->create_client<ServiceT>(service);
    this->timeout = timeout;
  }

  std::optional<ResponseT> call(const RequestT &req)
  {
    return call(std::make_shared<RequestT>(req));
  }

  std::optional<ResponseT> call(const std::shared_ptr<RequestT> &req_ptr)
  {
    if(!node) return {};

    if(!client->wait_for_service(timeout))
    {
      const std::string service{client->get_service_name()};

      RCLCPP_WARN(node->get_logger(), "Service %s is not reachable", service.c_str());
      // try to reconnect
      client = node->create_client<ServiceT>(service);
      if(!client->wait_for_service(timeout))
        return {};
      RCLCPP_INFO(node->get_logger(), "Reconnected to %s", service.c_str());
    }

    auto result = client->async_send_request(req_ptr);
    const auto spin_result{rclcpp::spin_until_future_complete(node, result, timeout)};

    if(spin_result == rclcpp::FutureReturnCode::SUCCESS)
      return *result.get();
    if(spin_result == rclcpp::FutureReturnCode::TIMEOUT)
      RCLCPP_WARN(node->get_logger(), "Call to %s timed out", client->get_service_name());
    return {};
  }

protected:
  rclcpp::Node::SharedPtr node;
  typename rclcpp::Client<ServiceT>::SharedPtr client;
  std::chrono::milliseconds timeout;

};

#endif // SRV_SYNC_H
