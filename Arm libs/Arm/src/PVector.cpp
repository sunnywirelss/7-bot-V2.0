#include "PVector.h"


 /**
 * @brief   		                  ���캯������ʼ����������
 * @code                        ʾ��
 *      				                      PVector  point1( );
 * @endcode
 */
PVector::PVector()
{
    x = y = z = 0.0;
}

 /**
 * @brief   		                  ���캯������ʼ���������굽�趨ֵ
 * @code                        ʾ��
 *      				                      // ��ʼ����������Ϊ: x = 20: y = 20: z = 20:                                     ��λmm
 *      				                      PVector  point1( 20, 20, 20);
 * @endcode
 * @param[in]                 _x           x��������                                                                            �ĸ�����
 * @param[in]                 _y           y��������                                                                            �ĸ�����
 * @param[in]                 _z           z��������                                                                            �ĸ�����
 */
PVector::PVector(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

 
 /**
 * @brief   		                  ���������ԼӺ�������ʾ�������һ������������
 * @code                        ʾ��
 *      				                      // �������������Ϊ: x = 10: y = 10: z = 5:                                PVector p1(10, 10, 5);
 *      				                      point1.add( p1) ;
 * @endcode
 * @param[in]                 p           PVector��һ�����󣬾�����������Ϊ���������ֵ����
 */
void PVector::add(PVector p)
{
    x += p.x;
    y += p.y;
    z += p.z;
}

 /**
 * @brief   		                  ���û�������
 * @code                        ʾ��
 *      				                      // ��ʼ����������Ϊ: x = 20: y = 20: z = 20:                                     ��λmm
 *      				                      PVector  point1( 20, 20, 20);
 * @endcode
 * @param[in]                 _x           x��������                                                                            �ĸ�����
 * @param[in]                 _y           y��������                                                                            �ĸ�����
 * @param[in]                 _z           z��������                                                                            �ĸ�����
 */
void PVector::set_xyz(double _x, double _y, double _z)
{
    x = _x;
    y = _y;
    z = _z;
}

 /**
 * @brief   		                  �����������������ע�⣺�����Լ���
 * @code                        ʾ��
 *      				                      // ����������������Ϊ: x = 10: y = 10: z = 5:                                PVector p1(10, 10, 5);
 *      				                      point1.sub( p1) ;
 * @endcode
 * @param[in]                 p              PVector��һ�����󣬾�����������Ϊһ��������������
  * @return                     tmp          ����ֵ��PVector��һ��������Ϊ�ش�������ֵ
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
 * @brief   		                  ��һ������
 * @code                        ʾ��
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
 * @brief   		                  ��˺���
 * @code                        ʾ��
 *      				                      // ��˵������Ϊ: x = 10: y = 10: z = 5:                                PVector p1(10, 10, 5);
 *      				                      point1.dot( p1) ;
 * @endcode
 * @param[in]                 p                                              PVector��һ������
  * @return                     x * p.x + y * p.y + z * p.z          ����ֵ�����ص��ֵ
 */
double PVector::dot(PVector p)
{
    return x * p.x + y * p.y + z * p.z;
}

 /**
 * @brief   		                  ��ຯ��
 * @code                        ʾ��
 *      				                      // ����ԭʼ������Ŀ������: x = 10: y = 10: z = 5 �ľ���:                                PVector p1(10, 10, 5);
 *      				                      point1.dist( p1) ;
 * @endcode
 * @param[in]                 p                                              PVector��һ������
  * @return                     sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z)          ����ֵ�����ص����֮��ľ���
 */
double PVector::dist(PVector p)
{
    double dist_x = x - p.x;
    double dist_y = y - p.y;
    double dist_z = z - p.z;
    return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}
