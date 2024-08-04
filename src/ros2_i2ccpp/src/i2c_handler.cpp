#include "ros2_i2ccpp/i2c_handler.hpp"
#include "ros2_i2ccpp/impl/i2c_handler_impl.hpp"
#include <memory>
#include <mutex>

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
template<typename PODType>
PODType I2CHandler<Mutex>::read_data(uint16_t address, uint16_t offset) const
{
  std::scoped_lock{mut};
  PODType pod{};

  return pod;
}

template<typename Mutex>
template<typename PODType>
void I2CHandler<Mutex>::write_data(uint16_t address, uint16_t offset, const PODType & data) const
{
  std::scoped_lock{mut};

}

template<typename Mutex>
void I2CHandler<Mutex>::set_pec(bool enable)
{
  std::scoped_lock{mut};
  handler->set_pec(enable);
}


}
