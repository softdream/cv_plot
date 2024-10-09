#include <opencv2/opencv.hpp>
#include <list>
#include <type_traits>
#include <iostream>

// ------------------------------- 获取可变参数模板中指定位置上的类型 --------------------------------- //
template<int N, typename... Ts>
struct get_paras_type;

template<typename T, typename... Ts>
struct get_paras_type<0, T, Ts...>
{
  using type = T;
};

template<int N, typename T, typename... Ts>
struct get_paras_type<N, T, Ts...>
{
  static assert( ( N <= sizeof...( Ts ) ), "N is Larger than the number of the parameters ." );
  using type = typename get_paras_type<N - 1, Ts...>::type;
};

// ------------------------------- 获取可变参数模板中指定位置上的类型 --------------------------------- //
template<int StartIdx, int EndIdx, int Idx, typename RequiredType, typename T, typename... Ts>
static constexpr bool is_required_types()
{
  if constexpr ( Idx < StartIdx ) return is_required_types<StartIdx, EndIdx, Idx + 1, RequiredType, Ts...>();
  else if constexpr ( Idx >= StartIdx && Idx < EndIdx ) {
    if constexpr ( !std::is_same_v<RequiredType, T> ) return false;
    else return is_required_types<StartIdx, EndIdx, Idx + 1, RequiredType, Ts...>();
  }
  else if constexpr ( Idx == EndIdx ) {
    if constexpr ( !std::is_same_v<RequiredType, T> ) return false;
    else return true;
  }
}

// ------------------------------- 获取可变参数模板中指定位置上的类型 --------------------------------- //
template<int StartIdx, int EndIdx, int Idx, typename T, typename... Ts>
static constexpr bool is_arithmetic_types()
{
  if constexpr ( Idx < StartIdx ) return is_arithmetic_types<StartIdx, EndIdx, Idx + 1, Ts...>();
  else if constexpr ( Idx >= StartIdx && Idx < EndIdx ) {
    if constexpr ( !std::is_arithmetic_v<T> ) return false;
    else return is_arithmetic_types<StartIdx, EndIdx, Idx + 1, Ts...>();
  }
  else if constexpr ( Idx == EndIdx ) {
    if constexpr ( !std::is_arithmetic_v<T> ) return false;
    else return true;
  }
}

// ------------------------------- 获取可变参数模板中指定位置上的类型 --------------------------------- //
template<int N, int Idx, typename T, typename... Ts>
static constexpr T get_para_value( T val, Ts... vals )
{
  if constexpr ( Idx == N ) return val;
  else return get_para_value<N, Idx + 1, Ts...>( vals... );
}

// ------------------------------- 获取可变参数模板中指定位置上的类型 --------------------------------- //
template<typename T>
struct get_values
{
  template<int StartIdx, int EndIdx, int Idx, int Cnt, typename U, typename... Us>
  static constexpr void get( T* values, U val, Us... vals ) 
  {
    if constexpr ( Idx < StartIdx ) {
      return get<StartIdx, EndIdx, Idx + 1, Cnt, Us...>( values, vals... );
    }
    else if constexpr ( Idx >= StartIdx && Idx < EndIdx ) {
      values[Cnt] = val;
      return get<StartIdx, EndIdx, Idx + 1, Cnt + 1, Us...>( values, vals... );
    }
    else if constexpr ( Idx == EndIdx ) {
      values[Cnt] = val;
      return;
    }
  }
};

template<typename... Paras>
static constexpr int partial( const int seg )
{
  return ( sizeof...( Paras ) / 3 ) * seg;
}

template<typename... Paras>
static constexpr int interval()
{
  return ( sizeof...( Paras ) / 3 - 1 );
}

enum Colors
{
  YELLOW = 1,
  RED,
  GREEN,
  BLACK,
  WHITE,
  BLUE
};

template<int StartIdx, int EndIdx, int Idx, int Cnt, int N, typename T, typename... Ts>
static constexpr void drawDescription( cv::Mat& img, Colors (&colors)[N], T val, Ts... vals )
{
  cv::Scalar color;
  switch ( colors[Cnt] ) {
    case YELLOW : color = cv::Scalar( 0, 255, 255 ); break;
    case RED : color = cv::Scalar( 0, 0, 255 ); break;
    case GREEN : color = cv::Scalar( 0, 255, 0 ); break;
    case BLACK : color = cv::Scalar( 0, 0, 0 ); break;
    case WHITE : color = cv::Scalar( 255, 255, 255 ); break;
    case BLUE : color = cv::Scalar( 255, 0, 0 ); break;
    default : break;
  }

  if constexpr ( Idx < StartIdx ) return drawDescription<StartIdx, EndIdx, Idx + 1, Cnt, N, Ts...>( img, colors, vals... );
  else if constexpr ( Idx >= StartIdx && Idx < EndIdx ) {
    cv::line( img, cv::Point( img.cols - img.cols / 3, 20 * ( Cnt + 1 ) ), cv::Point( img.cols - img.cols / 3 + 20, 20 * ( Cnt + 1 ) ), color, 1, cv::LINE_AA );
    cv::putText( img, val, cv::Point( img.cols - img.cols / 3 + 30, 20 * ( Cnt + 1 ) ), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1 );  
    return drawDescription<StartIdx, EndIdx, Idx + 1, Cnt + 1, N, Ts...>( img, colors, vals... );
  }
  else if constexpr ( Idx == EndIdx ) {
    cv::line( img, cv::Point( img.cols - img.cols / 3, 20 * ( Cnt + 1 ) ), cv::Point( img.cols - img.cols / 3 + 20, 20 * ( Cnt + 1 ) ), color, 1, cv::LINE_AA );
    cv::putText( img, val, cv::Point( img.cols - img.cols / 3 + 30, 20 * ( Cnt + 1 ) ), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1 ); 
    return;
  }
}

template<int N, int Idx, typename value_type, typename T, typename... Ts>
static constexpr void plot( cv::Mat& img, std::list<value_type> ( &que )[N], Colors (&colors)[N], int u_interval, bool reset_flag, T val, Ts... vals )
{
  cv::Scalar color;
  switch ( colors[Cnt] ) {
    case YELLOW : color = cv::Scalar( 0, 255, 255 ); break;
    case RED : color = cv::Scalar( 0, 0, 255 ); break;
    case GREEN : color = cv::Scalar( 0, 255, 0 ); break;
    case BLACK : color = cv::Scalar( 0, 0, 0 ); break;
    case WHITE : color = cv::Scalar( 255, 255, 255 ); break;
    case BLUE : color = cv::Scalar( 255, 0, 0 ); break;
    default : break;
  }  
  if constexpr ( Idx == N ) return;
  else {
    if ( reset_flag ) que[Idx].clear();

    que[Idx].push_back( val );

    if ( que[Idx].size() < img.cols / u_interval ) {
      cv::Point pre_pt( 0, que[Idx].front() );
      int i = 0;
      for ( const auto& v : que[Idx] ) {
        int u = i * u_interval;
        cv::Point cur_pt( u, v );
        cv::line( img, pre_pt, cur_pt, color, 1, cv::LINE_AA );
        pre_pt = cur_pt;
        i ++;
      }
    }
    else {
      cv::Point pre_pt( 0, que[Idx].front() );
      int i = 0;
      for ( const auto& v : que[Idx] ) {
        int u = i * u_interval;
        cv::Point cur_pt( u, v );
        cv::line( img, pre_pt, cur_pt, color, 1, cv::LINE_AA );
        pre_pt = cur_pt;
        i ++;
      }
      que[Idx].pop_front();
    }
    return plot<N, Idx + 1, value_type, Ts...>( img, que, color, u_interval, reset_flag, vals... );
  }
}

template<typename... Paras>
static void drawChart( cv::Mat& img, const int v_lower_bound, const int v_upper_bound, const int v_interval, const int u_interval, const bool reset_flag,
                       Paras... paras )
{
  static_assert( sizeof...( Paras ) % 3 == 0, "The Number of Paras is not Right ." );
  static_assert( is_arithmetic_types<partial<Paras...>( 0 ), partial<Paras...>( 0 ) + interval<Paras...>(), 0, Paras...>(), "Parameters Must be arithmetic type ." );
  static_assert( is_required_types<partial<Paras...>( 1 ), partial<Paras...>( 1 ) + interval<Paras...>(), 0, Colors, Paras...>(), "Parameters Must be Color Type ." );
  static_assert( is_required_types<partial<Paras...>( 2 ), partial<Paras...>( 2 ) + interval<Paras...>(), 0, std::string, Paras...>()
              || is_required_types<partial<Paras...>( 2 ), partial<Paras...>( 2 ) + interval<Paras...>(), 0, const char*, Paras...>(), "Parameters Must be string Type ." );
  
  img = cv::Mat::zeors( img.rows, img.cols, CV_8UC3 );
  auto v_blocks = std::abs( v_upper_bound - v_lower_bound ) / v_interval + 1;
  auto v_blocks_interval = img.rows / v_blocks;

  for ( int i = 0; i < v_blocks; i ++ ) cv::line( img, cv::Point( 0, i * v_blocks_interval ), cv::Point( img.cols, i * v_blocks_interval ), cv::Scalar( 100, 100, 100 ), 1 );

  static std::list<typename get_paras_type<0, Paras...>::type> ques[sizeof...( Paras ) / 3];
  static Colors colors[sizeof...( Paras ) / 3] = {RED};
  get_values<Colors>::get<partial<Paras...>( 1 ), partial<Paras...>( 1 ) + interval<Paras...>(), 0, 0, Paras...>( colors, paras... );

  drawDescription<partial<Paras...>( 2 ), partial<Paras...>( 2 ) + interval<Paras...>(), 0, 0, sizeof...( Paras ) / 3, Paras...>( img, colors, paras... );
  plot<sizeof...( Paras ) / 3, 0, typename get_paras_type<0, Paras...>::type, Paras...>( img, ques, colors, u_interval, reset_flag, paras... );
}
