#include "TSPathFinder.h"


TSPathFinder::TSPathFinder( const Polygon_2&         outer_poly, 
                            const vector<Polygon_2>& obstacles ):
_check(outer_poly, obstacles)
{}

Path TSPathFinder::findPath( const Point_2& s1, const Point_2& e1, 
                             const Point_2& s2, const Point_2& e2 )
{
  return Path();
}
