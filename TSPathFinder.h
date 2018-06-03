#include "CGAL_defines.h"
#include "ConfigCheck.h"

/*----------------------------------------------------------------------------
 * A robot wants to move from source point to destination,
 * while the second robot is at obstacle point.
 * */
class TSMove
{
public:
    Point_2 src_pt_;
    Point_2 dst_pt_;
    Point_2 obs_pt_;
};

/*----------------------------------------------------------------------------
 * Algorithms entry point
 * */
class TSPathFinder 
{
public:
    TSPathFinder( const Polygon_2& outer_poly, 
                  const vector<Polygon_2>& obstacles );
    Path findPath( const Point_2& s1, const Point_2& e1, 
                   const Point_2& s2, const Point_2& e2 );

protected:
    ConfigCheck _check;
};


