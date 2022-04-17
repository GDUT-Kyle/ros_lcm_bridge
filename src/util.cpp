#include <vector>
#include <string>
#include <math.h>
#include <sstream>
#include <iostream> 
#include <iomanip>

/**
 * @brief 字符分割
 * @param s: 要分割的字符串
 * @param msg: 结果
 * @param delim: 分隔符
 */
void split(const std::string& s, std::vector<std::string>& msg, char delim = ' ')
{
  msg.clear();
  auto string_find_first_not = [s, delim](size_t pos = 0) -> size_t {
    for (size_t i = pos; i < s.size(); i++)
    {
      if (s[i] != delim)
        return i;
    }
    return std::string::npos;
  };
  size_t lastPos = string_find_first_not(0);
  size_t pos = s.find(delim, lastPos);
  while (lastPos != std::string::npos)
  {
    msg.emplace_back(s.substr(lastPos, pos - lastPos));
    lastPos = string_find_first_not(pos);
    pos = s.find(delim, lastPos);
  }
}

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

std::string to_string_with_high_precision(double value, int precision = 20)
{
	std::stringstream ss;
	// ss.precision(precision);
	ss << std::fixed << std::setprecision(precision) << value;
	return ss.str();
}
