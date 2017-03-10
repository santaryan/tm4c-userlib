/*
 * sensor_contants.h
 *
 *  Created on: Mar 4, 2015
 *      Author: Ryan
 */

#ifndef SENSOR_CONTANTS_H_
#define SENSOR_CONTANTS_H_

#include <math.h>

/* Constants */
#ifdef M_PI
#undef M_PI
#endif
#ifdef M_PI_DIV_2
#undef M_PI_DIV_2
#endif

/* Conversion Macros */

#define M_PI       3.14159265358979323846
#define M_PI_DIV_2 1.57079632679489661923

#define SURFACE_GRAVITY_SUN                     274
#define SURFACE_GRAVITY_JUPITER                 24.92
#define SURFACE_GRAVITY_NEPTUNE                 11.15
#define SURFACE_GRAVITY_SATURN                  10.44
#define SURFACE_GRAVITY_EARTH                   9.798
#define SURFACE_GRAVITY_URANUS                  8.87
#define SURFACE_GRAVITY_VENUS                   8.87
#define SURFACE_GRAVITY_MARS                    3.71
#define SURFACE_GRAVITY_MERCURY                 3.7
#define SURFACE_GRAVITY_MOON                    1.62
#define SURFACE_GRAVITY_PLUTO                   0.58

#define SENSORS_NO_CHANGE(s)                    (s)                            /**< No modification to sensor output */

#define SENSORS_ADD_EXA(s)                      ((s) * 10e18)                  /**< Adds Exa- SI prefix */
#define SENSORS_ADD_PETA(s)                     ((s) * 10e15)                  /**< Adds Peta- SI prefix */
#define SENSORS_ADD_TERA(s)                     ((s) * 10e12)                  /**< Adds Tera- SI prefix */
#define SENSORS_ADD_GIGA(s)                     ((s) * 10e9)                   /**< Adds Giga- SI prefix */
#define SENSORS_ADD_MEGA(s)                     ((s) * 10e6)                   /**< Adds Mega- SI prefix */
#define SENSORS_ADD_KILO(s)                     ((s) * 10e3)                   /**< Adds Kilo- SI prefix */
#define SENSORS_ADD_HECTO(s)                    ((s) * 10e2)                   /**< Adds Hecto- SI prefix */
#define SENSORS_ADD_DEKA(s)                     ((s) * 10e1)                   /**< Adds Deka- SI prefix */
#define SENSORS_ADD_DECI(s)                     ((s) * 10e-1)                  /**< Adds Deci- SI prefix */
#define SENSORS_ADD_CENTI(s)                    ((s) * 10e-2)                  /**< Adds Centi- SI prefix */
#define SENSORS_ADD_MILLI(s)                    ((s) * 10e-3)                  /**< Adds Milli- SI prefix */
#define SENSORS_ADD_MICRO(s)                    ((s) * 10e-6)                  /**< Adds Micro- SI prefix */
#define SENSORS_ADD_NANO(s)                     ((s) * 10e-9)                  /**< Adds Nano- SI prefix */
#define SENSORS_ADD_PICO(s)                     ((s) * 10e-12)                 /**< Adds Pico- SI prefix */
#define SENSORS_ADD_FEMTO(s)                    ((s) * 10e-15)                 /**< Adds Femto- SI prefix */
#define SENSORS_ADD_ATTO(s)                     ((s) * 10e-18)                 /**< Adds Atto- SI prefix */

#define SENSORS_DEGREES_TO_RADIANS(d)           ((d) * M_PI / 180.0)           /**< Degrees to Radians */
#define SENSORS_RADIANS_TO_DEGREES(r)           ((r) * 180.0 / M_PI)           /**< Radians to Degrees */

#define SENSORS_SUN_G_TO_MS2(g)                 ((g) * SURFACE_GRAVITY_SUN)    /**< The sun's gravity in m/s^2 */
#define SENSORS_JUPITER_G_TO_MS2(g)             ((g) * SURFACE_GRAVITY_JUPITER)/**< Jupiters's gravity in m/s^2 */
#define SENSORS_NEPTUNE_G_TO_MS2(g)             ((g) * SURFACE_GRAVITY_NEPTUNE)/**< Neptunes's gravity in m/s^2 */
#define SENSORS_SATURN_G_TO_MS2(g)              ((g) * SURFACE_GRAVITY_SATURN) /**< Saturns's gravity in m/s^2 */
#define SENSORS_EARTH_G_TO_MS2(g)               ((g) * SURFACE_GRAVITY_EARTH)  /**< Earth's gravity in m/s^2 */
#define SENSORS_URANUS_G_TO_MS2(g)              ((g) * SURFACE_GRAVITY_URANUS) /**< Uranus' gravity in m/s^2 */
#define SENSORS_VENUS_G_TO_MS2(g)               ((g) * SURFACE_GRAVITY_VENUS)  /**< Venus' gravity in m/s^2 */
#define SENSORS_MARS_G_TO_MS2(g)                ((g) * SURFACE_GRAVITY_MARS)   /**< Mars' gravity in m/s^2 */
#define SENSORS_MERCURY_G_TO_MS2(g)             ((g) * SURFACE_GRAVITY_MERCURY)/**< Mercury's gravity in m/s^2 */
#define SENSORS_MOON_G_TO_MS2(g)                ((g) * SURFACE_GRAVITY_MOON)   /**< The moon's gravity in m/s^2 */
#define SENSORS_PLUTO_G_TO_MS2(g)               ((g) * SURFACE_GRAVITY_PLUTO)  /**< Pluto's gravity in m/s^2 */

#define SENSORS_SUN_MS2_TO_G(g)                 ((g) / SURFACE_GRAVITY_SUN)    /**< The sun's acceleration (m/s^2) to g's */
#define SENSORS_JUPITER_MS2_TO_G(g)             ((g) / SURFACE_GRAVITY_JUPITER)/**< Jupiters's acceleration (m/s^2) to g's */
#define SENSORS_NEPTUNE_MS2_TO_G(g)             ((g) / SURFACE_GRAVITY_NEPTUNE)/**< Neptunes's acceleration (m/s^2) to g's */
#define SENSORS_SATURN_MS2_TO_G(g)              ((g) / SURFACE_GRAVITY_SATURN) /**< Saturns's acceleration (m/s^2) to g's */
#define SENSORS_EARTH_MS2_TO_G(g)               ((g) / SURFACE_GRAVITY_EARTH)  /**< Earth's acceleration (m/s^2) to g's */
#define SENSORS_URANUS_MS2_TO_G(g)              ((g) / SURFACE_GRAVITY_URANUS) /**< Uranus' acceleration (m/s^2) to g's */
#define SENSORS_VENUS_MS2_TO_G(g)               ((g) / SURFACE_GRAVITY_VENUS)  /**< Venus' acceleration (m/s^2) to g's */
#define SENSORS_MARS_MS2_TO_G(g)                ((g) / SURFACE_GRAVITY_MARS)   /**< Mars' acceleration (m/s^2) to g's */
#define SENSORS_MERCURY_MS2_TO_G(g)             ((g) / SURFACE_GRAVITY_MERCURY)/**< Mercury's acceleration (m/s^2) to g's */
#define SENSORS_MOON_MS2_TO_G(g)                ((g) / SURFACE_GRAVITY_MOON)   /**< The moon's acceleration (m/s^2) to g's */
#define SENSORS_PLUTO_MS2_TO_G(g)               ((g) / SURFACE_GRAVITY_PLUTO)  /**< Pluto's acceleration (m/s^2) to g's */

#define SENSORS_GAUSS_TO_TESLA(m)               ((m) * 1e4)                    /**< Gauss to Tesla multiplier */
#define SENSORS_TESLA_TO_GAUSS(m)               ((m) * 1e-4)                   /**< Tesla to Gauss multiplier */

#define SENSORS_PA_TO_BAR(p)                    ((p) * 10e-5)                  /**< Pascal to Bar multiplier */
#define SENSORS_PA_TO_AT(p)                     ((p) * 1.0197 * 10e-5)         /**< Pascal to AT multiplier */
#define SENSORS_PA_TO_ATM(p)                    ((p) * 9.8692 * 1e-6)          /**< Pascal to ATM multiplier */
#define SENSORS_PA_TO_TORR(p)                   ((p) * 7.5006 * 10e-3)         /**< Pascal to Torr multiplier */
#define SENSORS_PA_TO_PSI(p)                    ((p) * 1.450377 * 10e-4)       /**< Pascal to Bar multiplier */

#define SENSORS_BAR_TO_PASCAL(p)                ((p) * 10e5)                   /**< Bar to Pascal multiplier */
#define SENSORS_BAR_TO_AT(p)                    ((p) * 1.1097)                 /**< Bar to AT multiplier */
#define SENSORS_BAR_TO_ATM(p)                   ((p) * 0.98692)                /**< Bar to ATM multiplier */
#define SENSORS_BAR_TO_TORR(p)                  ((p) * 750.06)                 /**< Bar to Torr multiplier */
#define SENSORS_BAR_TO_PSI(p)                   ((p) * 14.50377)               /**< Bar to PSI multiplier */

#define SENSORS_AT_TO_PASCAL(p)                 ((p) * 9.80665 * 10e4)         /**< AT to Pascal multiplier */
#define SENSORS_AT_TO_BAR(p)                    ((p) * 0.980665)               /**< AT to Bar multiplier */
#define SENSORS_AT_TO_ATM(p)                    ((p) * 0.9678411)              /**< AT to ATM multiplier */
#define SENSORS_AT_TO_TORR(p)                   ((p) * 735.5592)               /**< AT to Torr multiplier */
#define SENSORS_AT_TO_PSI(p)                    ((p) * 14.22334)               /**< AT to PSI multiplier */

#define SENSORS_ATM_TO_PASCAL(p)                ((p) * 1.01325 * 10e5)         /**< ATM to Pascal multiplier */
#define SENSORS_ATM_TO_BAR(p)                   ((p) * 1.01325)                /**< ATM to Bar multiplier */
#define SENSORS_ATM_TO_AT(p)                    ((p) * 1.0332)                 /**< ATM to AT multiplier */
#define SENSORS_ATM_TO_TORR(p)                  ((p) * 760)                    /**< ATM to Torr multiplier */
#define SENSORS_ATM_TO_PSI(p)                   ((p) * 14.69595)               /**< ATM to PSI multiplier */

#define SENSORS_TORR_TO_PASCAL(p)               ((p) * 133.3224)               /**< Torr to Pascal multiplier */
#define SENSORS_TORR_TO_BAR(p)                  ((p) * 1.333224*10e-3)         /**< Torr to Bar multiplier */
#define SENSORS_TORR_TO_AT(p)                   ((p) * 1.359551*10e-3)         /**< Torr to AT multiplier */
#define SENSORS_TORR_TO_ATM(p)                  ((p) / 760)                    /**< Torr to ATM multiplier */
#define SENSORS_TORR_TO_PSI(p)                  ((p) * 1.933678 * 10e-2)       /**< Torr to PSI multiplier */

#define SENSORS_PSI_TO_PA(p)                    ((p) * 6.8948 * 10e3)          /**< PSI to PSI multiplier */
#define SENSORS_PSI_TO_BAR(p)                   ((p) * 6.8948 * 10e-2)         /**< PSI to Bar multiplier */
#define SENSORS_PSI_TO_AT(p)                    ((p) * 7.03069 * 10e-2)        /**< PSI to AT multiplier */
#define SENSORS_PSI_TO_ATM(p)                   ((p) * 6.8046 * 10e-2)         /**< PSI to ATM multiplier */
#define SENSORS_PSI_TO_TORR(p)                  ((p) * 51.71493)               /**< PSI to Torr multiplier */

#define SENSORS_FARENHEIT_TO_CELSIUS(t)         (((t) - 32.0) / 1.8)           /**< Farenheit to Celsius */
#define SENSORS_FARENHEIT_TO_KELVIN(t)          (((t) + 459.67) * 5.0 / 9.0)   /**< Farenheit to Kelvin */
#define SENSORS_CELSIUS_TO_FARENHEIT(t)         (((t) * 1.8) + 32.0)           /**< Celsius to Farenheit */
#define SENSORS_CELSIUS_TO_KELVIN(t)            ((t) + 273.15)                 /**< Celsius to Kelvin */
#define SENSORS_KELVIN_TO_FARENHEIT(t)          ((t) * 1.8 - 459.67)           /**< Kelvin to Farenheit */
#define SENSORS_KELVIN_TO_CELSIUS(t)            ((t) - 273.15)                 /**< Kelvin to Celsius */
#define SENSORS_KELVIN_TO_RANKINE(t)            ((t) * 1.8)                    /**< Kelvin to Rankine */
#define SENSORS_RANKINE_TO_KELVIN(t)            ((t) * 5.0 / 9.0)              /**< Rankine to Kelvin */

#define SENSORS_FEET_TO_METERS(l)               ((l) * 0.3048)                 /**< Feet to Meters */
#define SENSORS_FEET_TO_LIGHTYEARS(l)           ((l) * 3.22174e-17)            /**< Feet to Lightyears  */
#define SENSORS_FEET_TO_PARSECS(l)              ((l) * 9.87790e-18)            /**< Feet to Parsecs */
#define SENSORS_YARDS_TO_METERS(l)              ((l) * 0.91439999999999999059)
#define SENSORS_METERS_TO_FEET(l)               ((l) * 3.28083989501312335958) /**< Meters to Feet */
#define SENSORS_METERS_TO_YARDS(l)              ((l) * 1.093613298)            /**< Meters to Yards */
#define SENSORS_METERS_TO_LIGHTYEARS(l)         ((l) * 1.05700e-16)            /**< Meters to Lightyears (To Infinity and beyond) */
#define SENSORS_METERS_TO_PARSECS(l)            ((l) * 3.24078e-17)            /**< Meters to Parsecs */
#define SENSORS_LIGHTYEARS_TO_FEET(l)           ((l) / 3.22174e-17)            /**< Lightyears to Feet */
#define SENSORS_LIGHTYEARS_TO_METERS(l)         ((l) / 1.05700e-16)            /**< Lightyears to Meters (or not...) */
#define SENSORS_PARSECS_TO_FEET(l)              ((l) / 9.87790e-18)            /**< Parsecs to Feet */
#define SENSORS_PARSECS_TO_METERS(l)            ((l) / 3.24078e-17)            /**< Parsecs to Meters */

#define SENSORS_PA_TO_PRESSURE_ALTITUDE(p)      (44331.5 - (4946.62*pow((p),0.190263))) /**< Alt in meters */

#endif /* SENSOR_CONTANTS_H_ */
