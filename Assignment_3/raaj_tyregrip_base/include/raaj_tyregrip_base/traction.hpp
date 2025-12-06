#ifndef RAAJ_TYREGRIP_BASE_TRACTION_HPP
#define RAAJ_TYREGRIP_BASE_TRACTION_HPP

#include <string>

namespace raaj_tyregrip_base
{
  class Weather
  {
    public:
      virtual ~Weather() {}
      virtual double get_multiplier(double speed) const = 0;
      virtual std::string get_type() const = 0;
  };

  class Tyre
  {
    public:
      virtual ~Tyre() {}
      virtual double get_multiplier(double speed, const std::string &weather_type) const = 0;
      virtual std::string get_type() const = 0;
  };
}

#endif
