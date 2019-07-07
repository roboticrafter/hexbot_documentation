#include "PVector.h"


 /**
 * @brief   		                  构造函数：初始化机体坐标
 * @code                        示例
 *      				                      PVector  point1( );
 * @endcode
 */
PVector::PVector()
{
    x = y = z = 0.0;
}

 /**
 * @brief   		                  构造函数：初始化机体坐标到设定值
 * @code                        示例
 *      				                      // 初始化机体坐标为: x = 20: y = 20: z = 20:                                     单位mm
 *      				                      PVector  point1( 20, 20, 20);
 * @endcode
 * @param[in]                 _x           x方向坐标                                                                            哪个方向
 * @param[in]                 _y           y方向坐标                                                                            哪个方向
 * @param[in]                 _z           z方向坐标                                                                            哪个方向
 */
PVector::PVector(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

 
 /**
 * @brief   		                  机体坐标自加函数（表示给机体的一个坐标增量）
 * @code                        示例
 *      				                      // 机体坐标的增量为: x = 10: y = 10: z = 5:                                PVector p1(10, 10, 5);
 *      				                      point1.add( p1) ;
 * @endcode
 * @param[in]                 p           PVector的一个对象，具体作用是作为机体的增量值存在
 */
void PVector::add(PVector p)
{
    x += p.x;
    y += p.y;
    z += p.z;
}

 /**
 * @brief   		                  设置机体坐标
 * @code                        示例
 *      				                      // 初始化机体坐标为: x = 20: y = 20: z = 20:                                     单位mm
 *      				                      PVector  point1( 20, 20, 20);
 * @endcode
 * @param[in]                 _x           x方向坐标                                                                            哪个方向
 * @param[in]                 _y           y方向坐标                                                                            哪个方向
 * @param[in]                 _z           z方向坐标                                                                            哪个方向
 */
void PVector::set_xyz(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

 /**
 * @brief   		                  机体坐标相减函数（注意：不是自减）
 * @code                        示例
 *      				                      // 与机体坐标相减的量为: x = 10: y = 10: z = 5:                                PVector p1(10, 10, 5);
 *      				                      point1.sub( p1) ;
 * @endcode
 * @param[in]                 p              PVector的一个对象，具体作用是作为一个被减掉的坐标
  * @return                     tmp          返回值：PVector的一个对象，作为回传相减后的值
 */
PVector PVector::sub(PVector p)
{
    PVector tmp;
    tmp.x = x - p.x;
    tmp.y = y - p.y;
    tmp.z = z - p.z;
    return tmp;
}


 /**
 * @brief   		                  归一化函数
 * @code                        示例
 *      				                      point1.normalize() ;
 * @endcode
 */
void PVector::normalize()
{
    double l = sqrt(x * x + y * y + z * z);
    x /= l;
    y /= l;
    z /= l;
}

 /**
 * @brief   		                  点乘函数
 * @code                        示例
 *      				                      // 点乘点的坐标为: x = 10: y = 10: z = 5:                                PVector p1(10, 10, 5);
 *      				                      point1.dot( p1) ;
 * @endcode
 * @param[in]                 p                                              PVector的一个对象
  * @return                     x * p.x + y * p.y + z * p.z          返回值：返回点乘值
 */
double PVector::dot(PVector p)
{
    return x * p.x + y * p.y + z * p.z;
}

 /**
 * @brief   		                  点距函数
 * @code                        示例
 *      				                      // 需求原始坐标与目标坐标: x = 10: y = 10: z = 5 的距离:                                PVector p1(10, 10, 5);
 *      				                      point1.dist( p1) ;
 * @endcode
 * @param[in]                 p                                              PVector的一个对象
  * @return                     sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z)          返回值：返回点与点之间的距离
 */
double PVector::dist(PVector p)
{
    double dist_x = x - p.x;
    double dist_y = y - p.y;
    double dist_z = z - p.z;
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}
