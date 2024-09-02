#include "ros2_i2ccpp/i2c_handler.hpp"
#include "ros2_i2ccpp/impl/i2c_handler_impl.hpp"
#include <memory>
#include <mutex>
#include <algorithm>

namespace ros2_i2ccpp
{

template<typename Mutex>
I2CHandler<Mutex>::I2CHandler(uint16_t i2c_addr, std::string i2c_adapter_path)
: handler(std::make_unique<I2CHandlerImpl>(i2c_addr, i2c_adapter_path))
{

}

template<typename Mutex>
I2CHandler<Mutex>::I2CHandler(std::string i2c_adapter_path)
: handler(std::make_unique<I2CHandlerImpl>(i2c_adapter_path))
{

}


template<typename Mutex>
uint64_t I2CHandler<Mutex>::get_adapter_func() const
{
  std::scoped_lock{mut};
  return handler->get_adapter_func();
}

template<typename Mutex>
uint64_t I2CHandler<Mutex>::get_current_device_addr() const
{
  std::scoped_lock{mut};
  return handler->get_current_device_addr();
}

template<typename Mutex>
bool I2CHandler<Mutex>::has_functionality(uint64_t flag) const
{
  std::scoped_lock{mut};
  return handler->has_functionality(flag);
}

template<typename Mutex>
void I2CHandler<Mutex>::set_pec(bool enable)
{
  std::scoped_lock{mut};
  handler->set_pec(enable);
}

template<typename Mutex>
void I2CHandler<Mutex>::apply_transaction(I2CTransaction && transaction_) const
{
  std::scoped_lock{mut};

  auto transaction = std::move(transaction_);

  std::pmr::vector<i2c_msg> inner_buf{&transaction.getMemoryResource()};

  // build i2c message buffer from transaction segments
  std::transform(
  transaction.getSegments().begin(), transaction.getSegments().end(), std::back_inserter(inner_buf),
    [](const std::shared_ptr<I2CTransactionSegment> & segment) {
      return i2c_msg{segment->get_address(), segment->get_message_flags(), segment->get_data_size(),
        segment->get_data()};
  });

  // ship i2c transaction
  handler->process_i2c_transaction(inner_buf);

  // the transaction should now be destroyed
}

template class I2CHandler<std::mutex>;
template class I2CHandler<null_mutex>;

}
