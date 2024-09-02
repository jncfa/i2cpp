#include <cstdio>
#include <ros2_i2ccpp/transaction.hpp>
#include <cstring>
#include <fmt/format.h>
#include <iostream>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  ros2_i2ccpp::I2CTransactionBuilderPooled ss(0x1);
  int a = 0xABAB;
  std::array<uint8_t, 42> stds;

  ss.add_read(stds);
  ss.add_write(a);

  auto trans = ss.getTransaction();

  for (uint64_t i = 0; i < 1000000; i++) {
    ss.add_read(stds);
    ss.add_write(a);
    auto trans = ss.getTransaction();
    for(const auto & j: trans.getSegments()) {
      fmt::print("value {}: {} - size {:}\n", i, fmt::ptr(j->get_data()), j->get_data_size());
    }
    ss.add_read(stds);
    ss.add_write(a);
    trans = ss.getTransaction();
    for(const auto & j: trans.getSegments()) {
      fmt::print("value {}: {} - size {:}\n", i, fmt::ptr(j->get_data()), j->get_data_size());
    }

  }

  fmt::print("value final: 0x{0:x}\n", a);

  return 0;
}
