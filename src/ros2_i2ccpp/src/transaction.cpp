extern "C"
{
#include <linux/i2c.h>
}

#include "ros2_i2ccpp/transaction.hpp"
#include "ros2_i2ccpp/impl/i2c_handler_impl.hpp"
#include "ros2_i2ccpp/i2c_handler.hpp"
#include <execution>
#include <algorithm>


namespace ros2_i2ccpp
{

template<typename Mutex>
void I2CTransactionBuilderImpl::apply_transaction(const I2CHandler<Mutex> & handler)
{
  apply_transaction(handler.handler.get());
}

void I2CTransactionBuilderImpl::apply_transaction(I2CHandlerImpl & handler)
{
  std::pmr::vector<i2c_msg> inner_buf{&mem_resource};

// indicate that operation can be vectorized
  std::transform(std::execution::unseq, transaction_segments.begin(), transaction_segments.end(),
std::back_inserter(inner_buf), [](const unique_ptr_pmr<I2CTransactionSegment> & segment) {
      return i2c_msg{segment->get_address(), segment->get_message_flags(), segment->get_data_size(),
      segment->get_data()};
});

// call directly on the internal handler
  handler.process_i2c_transaction(inner_buf);
}

}
