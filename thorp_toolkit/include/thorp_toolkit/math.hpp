/*
 * Author: Jorge Santos
 */

#pragma once

#include <memory>  // for std::allocator
#include <numeric> // for std::accumulate in 18.04

#include <geometry_msgs/Vector3.h>

namespace thorp::toolkit
{

/**
 * Return the minimum value from a geometry_msgs/Vector3
 * @param v vector
 * @return minimum value
 */
inline double minValue(const geometry_msgs::Vector3& v)
{
  return std::min(std::min(v.x, v.y), v.z);
}

/**
 * Return the maximum value from a geometry_msgs/Vector3
 * @param v vector
 * @return maximum value
 */
inline double maxValue(const geometry_msgs::Vector3& v)
{
  return std::max(std::max(v.x, v.y), v.z);
}

/**
 * Normalize an angle between -π and +π
 * @param a Unnormalized angle
 * @return Normalized angle
 */
inline double wrapAngle(double a)
{
  a = fmod(a + M_PI, 2*M_PI);
  return a < 0.0 ? a + M_PI : a - M_PI;
}


// Template functions

/**
 * Return the median element of an iterable and randomly-accessible STL container (vector and deque, afaik)
 * without modifying it.
 * @param c source container
 * @return mean value
 */
template <template <typename, typename> class C,
          typename T,
          typename Allocator=std::allocator<T>>
T median(const C<T, Allocator>& cc)
{
  C<T, Allocator> c = cc;  // clone, as nth_element will short half of the vector
  std::nth_element(c.begin(), c.begin() + c.size()/2, c.end());
  return c[c.size()/2];
};

/**
 * Return the median element of an iterable and randomly-accessible STL container (vector and deque, afaik).
 * Slightly faster because works on the source container, modifying it.
 * @param c source container
 * @return mean value
 */
template <template <typename, typename> class C,
          typename T,
          typename Allocator=std::allocator<T>>
T median(C<T, Allocator>& c)
{
  std::nth_element(c.begin(), c.begin() + c.size()/2, c.end()); // short values till middle one
  return c[c.size()/2];
};

/**
 * Return the mean value of an iterable STL container.
 * @param c source container
 * @return mean value
 */
template <template <typename, typename> class C,
          typename T,
          typename Allocator=std::allocator<T>>
T mean(const C<T, Allocator>& c)
{
  return std::accumulate(c.begin(), c.end(), 0.0) / c.size();
};


/**
 * Return the variance of an iterable STL container.
 * @param c source container
 * @return variance
 */
template <template <typename, typename> class C,
          typename T,
          typename Allocator=std::allocator<T>>
T variance(const C<T, Allocator>& c)
{
  T acc = 0;
  T avg = mean(c);
  for (auto it = c.begin(); it != c.end(); ++it)
    acc += std::pow(*it - avg, 2);
  return acc/(c.size() - 1);
};

/**
 * Return the standard deviation of an iterable STL container.
 * @param c source container
 * @return standard deviation
 */
template <template <typename, typename> class C,
          typename T,
          typename Allocator=std::allocator<T>>
T std_dev(const C<T, Allocator>& c)
{
  return std::sqrt(variance(c));
}


template <typename T> T sign(T x)
{
  return x > 0.0 ? +1.0 : x < 0.0 ? -1.0 : 0.0;
}

// Other functions


} /* namespace thorp::toolkit */
