#pragma once
#ifndef INS570_DECODE_HPP
#define INS570_DECODE_HPP

#include <rclcpp/rclcpp.hpp>

namespace ins570_decode {

class DecodeNode : public rclcpp::Node {
   public:
    DecodeNode(const rclcpp::NodeOptions &options);
    ~DecodeNode();

   private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

}  // namespace ins570_decode

#endif  // INS570_DECODE_HPP