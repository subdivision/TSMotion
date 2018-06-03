#ifndef CONFIG_CHECK_H
#define CONFIG_CHECK_H

#include <vector>
#include <list>
#include <map>
#include <set>
#include <math.h>
#include <utility>
#include "CGAL_defines.h"

using namespace std;
typedef vector<pair<Point_2, Point_2>> Path;

class TSMove;

//=============================================================================
class polygon_split_observer : public CGAL::Arr_observer<Arrangement_2> {
    void after_split_face(Face_handle f1, Face_handle f2, bool) override;
};

//=============================================================================
class ConfigCheck 
{
public:
    ConfigCheck( const Polygon_2&         outer_bound, 
                 const vector<Polygon_2>& obstacles );

public:
    bool isLegalMove( const TSMove& mv );

private:
    Arrangement_2 _arr;
    Landmarks_pl  _pl;

    void verticalDecomposition(Kernel &ker);

    void addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker);

    void addFrame(const Polygon_2& outer_bound);
};

#endif 
