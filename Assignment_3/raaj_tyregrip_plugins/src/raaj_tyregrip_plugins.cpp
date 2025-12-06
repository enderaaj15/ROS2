#include <raaj_tyregrip_base/traction.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace raaj_tyregrip_plugins
{
// ---------- Weather Plugins (Set to constant 1.0 multiplier) ----------
  class DryWeather : public raaj_tyregrip_base::Weather
  {
  public:
    double get_multiplier(double speed) const override
    {
      (void) speed;
      return 1.0;
    }
    std::string get_type() const override { return "dry"; }
  };

  class WetWeather : public raaj_tyregrip_base::Weather
  {
  public:
    double get_multiplier(double speed) const override
    {
      (void) speed;
      return 1.0;
    }
    std::string get_type() const override { return "wet"; }
  };

  class SnowWeather : public raaj_tyregrip_base::Weather
  {
  public:
    double get_multiplier(double speed) const override
    {
      (void) speed;
      return 1.0;
    }
    std::string get_type() const override { return "snow"; }
  };

// ---------- Tyre Plugins (Multiplier set to target mu_total) ----------
  class UHPTyre : public raaj_tyregrip_base::Tyre
  {
  public:
    double get_multiplier(double speed, const std::string &weather_type) const override
    {
      // Target: 1.1 to 0.9 (reduces with speed)
      if (weather_type == "dry") {
        if (speed < 120.0) return 1.10;
        else if (speed < 200.0) return 1.00;
        else return 0.90;
      }
      // Target: 0.9 to 0.7 (reduces with speed)
      else if (weather_type == "wet") {
        if (speed < 120.0) return 0.90;
        else if (speed < 200.0) return 0.80;
        else return 0.70;
      }
      // (constant)
      else if (weather_type == "snow") {
        return 0.30;
      }
      return 0.5;
    }
    std::string get_type() const override { return "UHP"; }
  };

  class EcoTyre : public raaj_tyregrip_base::Tyre
  {
  public:
    double get_multiplier(double speed, const std::string &weather_type) const override
    {
      // Target: 1.0 to 0.8 (reduces with speed)
      if (weather_type == "dry") {
        if (speed < 120.0) return 1.00;
        else if (speed < 200.0) return 0.90;
        else return 0.80;
      }
      // Target: 0.8 to 0.6 (reduces with speed)
      else if (weather_type == "wet") {
        if (speed < 120.0) return 0.80;
        else if (speed < 200.0) return 0.70;
        else return 0.60;
      }
      // (constant)
      else if (weather_type == "snow") {
        return 0.20;
      }
      return 0.5;
    }
    std::string get_type() const override { return "eco"; }
  };

  class WinterTyre : public raaj_tyregrip_base::Tyre
  {
  public:
    double get_multiplier(double speed, const std::string &weather_type) const override
    {
      (void) speed;

      // Target: 0.8 to 0.6 (reduces with speed)
      if (weather_type == "dry") {
        if (speed < 120.0) return 0.80;
        else if (speed < 200.0) return 0.70;
        else return 0.60;
      }
      // Target: 0.6 to 0.45 (reduces with speed)
      else if (weather_type == "wet") {
        if (speed < 120.0) return 0.60;
        else if (speed < 200.0) return 0.50;
        else return 0.45;
      }
      // (constant)
      else if (weather_type == "snow") return 0.50;

      return 0.5;
    }
    std::string get_type() const override { return "winter"; }
  };
}


PLUGINLIB_EXPORT_CLASS(raaj_tyregrip_plugins::DryWeather, raaj_tyregrip_base::Weather)
PLUGINLIB_EXPORT_CLASS(raaj_tyregrip_plugins::WetWeather, raaj_tyregrip_base::Weather)
PLUGINLIB_EXPORT_CLASS(raaj_tyregrip_plugins::SnowWeather, raaj_tyregrip_base::Weather)

PLUGINLIB_EXPORT_CLASS(raaj_tyregrip_plugins::UHPTyre, raaj_tyregrip_base::Tyre)
PLUGINLIB_EXPORT_CLASS(raaj_tyregrip_plugins::EcoTyre, raaj_tyregrip_base::Tyre)
PLUGINLIB_EXPORT_CLASS(raaj_tyregrip_plugins::WinterTyre, raaj_tyregrip_base::Tyre)
