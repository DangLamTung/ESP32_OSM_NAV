/**
 * mercator.h — Web Mercator projection helpers (header-only).
 * Same projection as the OpenStreetMap-esp32 library (256px tiles).
 */
#ifndef MERCATOR_H_
#define MERCATOR_H_

#include <math.h>

static inline double lon2wx(double lon, int z) { return (lon + 180.0) / 360.0 * (256.0 * (1 << z)); }

static inline double lat2wy(double lat, int z) {
  double r = lat * M_PI / 180.0;
  double t = tan(M_PI / 4 + r / 2.0);
  return (1.0 - log(t) / M_PI) / 2.0 * (256.0 * (1 << z));
}

static inline double wx2lon(double wx, int z) { return wx / (256.0 * (1 << z)) * 360.0 - 180.0; }

static inline double wy2lat(double wy, int z) {
  double n = M_PI - 2.0 * M_PI * wy / (256.0 * (1 << z));
  return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

#endif // MERCATOR_H_
