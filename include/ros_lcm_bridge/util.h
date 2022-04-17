#ifndef UTIL_H
#define UTIL_H

#include <vector>
#include <string>

void split(const std::string& s, std::vector<std::string>& tokens, char delim);

template <typename T>
T toRad(T deg)
{
  return deg * M_PI / 180.0;
}

template <typename T>
T toDeg(T rad)
{
  return rad * 180.0 / M_PI;
}

std::string to_string_with_high_precision(double value, int precision = 20);

#endif