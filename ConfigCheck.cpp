#include "TSPathFinder.h"
#include "ConfigCheck.h"
#include <CGAL/minkowski_sum_2.h>
#include <math.h>

#define EPS 0.0001

bool eps_eq(const Point_2& p1, const Point_2& p2 )
{
    Number_type x1 = p1.x();
    Number_type y1 = p1.y();
    Number_type x2 = p2.x();
    Number_type y2 = p2.y();
    return (fabs(x1-x2) <= EPS) && (fabs(y1-y2) <= EPS);
}

//----------------------------------------------------------------------------
void 
polygon_split_observer::after_split_face( Face_handle f1, Face_handle f2, bool ) 
{
    f2->set_contained(f1->contained());
}

//----------------------------------------------------------------------------
Polygon_2 get_bounding_box_plus_1(Polygon_2 plg) {
	FT max_X = 0;
	FT min_X = 0;
	FT max_Y = 0;
	FT min_Y = 0;
	for (auto p = plg.vertices_begin(); p != plg.vertices_end(); ++p) {
		max_X = max(max_X, p->x());
		max_Y = max(max_Y, p->y());
		min_X = min(min_X, p->x());
		min_Y = min(min_Y, p->y());
	}
	min_X -= 1;
	min_Y -= 1;
	max_X += 1;
	max_Y += 1;
	Polygon_2 ret;
	ret.push_back(Point_2(min_X, min_Y));
	ret.push_back(Point_2(max_X, min_Y));
	ret.push_back(Point_2(max_X, max_Y));
	ret.push_back(Point_2(min_X, max_Y));
	return ret;
}

Polygon_2 get_robot() {
	Polygon_2 minusRobot;
	minusRobot.push_back(Point_2(0, 0));
	minusRobot.push_back(Point_2(-1, 0));
	minusRobot.push_back(Point_2(-1, -1));
	minusRobot.push_back(Point_2(0, -1));
	return minusRobot;
}

#define all(x) x.begin(), x.end()

//----------------------------------------------------------------------------
ConfigCheck::ConfigCheck( const Polygon_2&          outer_obstacle, 
                          const vector<Polygon_2>&  obstacles )
{
    Polygon_2 minusRobot = get_robot();
	vector<Polygon_with_holes_2> minkObs;
	for (auto obs : obstacles)
		minkObs.push_back(CGAL::minkowski_sum_2(obs, minusRobot));

	vector<Polygon_2> outer_as_vector;
	outer_as_vector.push_back(outer_obstacle);
	Polygon_with_holes_2 outer_with_bounding_box(get_bounding_box_plus_1(outer_obstacle), all(outer_as_vector));
	Polygon_with_holes_2 outer_with_bounding_box_after_mink = CGAL::minkowski_sum_2(outer_with_bounding_box, minusRobot);
	Polygon_with_holes_2 outer(Polygon_2(), outer_with_bounding_box_after_mink.holes_begin(), outer_with_bounding_box_after_mink.holes_end());
	minkObs.push_back(outer);
	Polygon_set_2 ps2;
	ps2.join(all(minkObs));
	ps2.complement();

    _arr = ps2.arrangement();

    //ensure that when face split two side safe their property (inside/outside)
    Polygon_set_2::Traits_2 traits;
    polygon_split_observer observer;
    observer.attach(_arr);
    _pKer = &traits;
    verticalDecomposition(*_pKer);
    observer.detach();
    _arr.unbounded_face()->set_contained(true);

    _pl.attach(_arr);
}

bool 
ConfigCheck::isLegalMove( const TSMove& mv ) 
{
    bool bLocalFree = squaresFree( mv );
    if( !bLocalFree )
        return false;
    
    Segment_2 querySegment( mv.src_pt_, mv.dst_pt_ );

    vector<CGAL::Object> vecZoneElems;
    Face_handle hFace;

    CGAL::zone(_arr, querySegment, std::back_inserter(vecZoneElems), _pl);
    for (int i = 0; i < vecZoneElems.size(); ++i) 
    {
      if (CGAL::assign(hFace, vecZoneElems[i]) && !hFace->contained())
        return false;
    }

    return true;
}

bool
ConfigCheck::squaresFree( const TSMove& mv )
{
    Segment_2 segs1[4] = { Segment_2(mv.src_pt_, mv.dst_pt_),
                           Segment_2(mv.src_pt_+ Vector_2(1,0), mv.dst_pt_+ Vector_2(1,0)),
                           Segment_2(mv.src_pt_+ Vector_2(1,1), mv.dst_pt_+ Vector_2(1,1)),
                           Segment_2(mv.src_pt_+ Vector_2(0,1), mv.dst_pt_+ Vector_2(0,1)) };
    Segment_2 segs2[4] = { Segment_2(mv.obs_pt_, mv.obs_pt_ + Vector_2(1,0)),
                           Segment_2(mv.obs_pt_ + Vector_2(1,0), mv.obs_pt_ + Vector_2(1,1)),
                           Segment_2(mv.obs_pt_ + Vector_2(1,1), mv.obs_pt_ + Vector_2(0,1)),
                           Segment_2(mv.obs_pt_ + Vector_2(0,1), mv.obs_pt_ ) };
    for( int i = 0; i < 4; ++i )
    {
        for( int j = 0; j < 4; ++j )
        {
            CGAL::cpp11::result_of<Intersect_2(Segment_2, Segment_2)>::type
                result = intersection(segs1[i], segs2[j]);
            if( result ) 
            {
                if (const Segment_2* s = boost::get<Segment_2>(&*result)) 
                {
                    continue; 
                } 
                else 
                {
                    const Point_2* p = boost::get<Point_2 >(&*result);
                    if( !eps_eq( *p, segs1[i].source() ) && 
                        !eps_eq( *p, segs2[j].source() ) && 
                        !eps_eq( *p, segs1[i].target() ) &&
                        !eps_eq( *p, segs2[j].target() ) )
                        return false;
                }
            }
        }
    }
    return true;
}


void 
ConfigCheck::addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker) 
{
    X_monotone_curve_2 seg;
    Vertex_const_handle vh;
    Halfedge_const_handle hh;
    Face_const_handle fh;
    Vertex_handle v2;

    if (CGAL::assign(vh, obj)) {
        // The given feature is a vertex.
        seg = X_monotone_curve_2(v->point(), vh->point());
        v2 = _arr.non_const_handle(vh);
    } else if (CGAL::assign(hh, obj)) {
        // The given feature is a halfedge.
        if (hh->is_fictitious()) //We ignore fictitious halfedges.
        {
            return;
        }

        // Check whether v lies in the interior of the x-range of the edge (in
        // which case this edge should be split).
        const typename Kernel::Compare_x_2 cmp_x = ker.compare_x_2_object();
        if (cmp_x(v->point(), hh->target()->point()) == CGAL::EQUAL) {
            // In case the target of the edge already has the same x-coordinate as
            // the vertex v, just connect these two vertices.
            seg = X_monotone_curve_2(v->point(), hh->target()->point());
            v2 = _arr.non_const_handle(hh->target());
        } else {
            // Compute the vertical projection of v onto the segment associated
            // with the halfedge. Split the edge and connect v with the split point.
            Line_2 Line;
            Line_2 supp_line(hh->source()->point(), hh->target()->point());
            Line_2 vert_line(v->point(), Point_2(v->point().x(), v->point().y() + 1));
            Point_2 point;
            CGAL::assign(point, ker.intersect_2_object()(supp_line, vert_line));
            seg = X_monotone_curve_2(v->point(), point);
            _arr.split_edge(_arr.non_const_handle(hh),
                            X_monotone_curve_2(hh->source()->point(), point),
                            X_monotone_curve_2(point, hh->target()->point()));
            v2 = _arr.non_const_handle(hh->target());
        }
    } else {
        // Ignore faces and empty objects.
        return;
    }

    // Add the vertical segment to the arrangement using its two end vertices.
    _arr.insert_at_vertices(seg, v, v2);
}

void 
ConfigCheck::verticalDecomposition(Kernel &ker) 
{
    typedef pair<Vertex_const_handle, pair<CGAL::Object, CGAL::Object>> Vd_entry;

    // For each vertex in the arrangment, locate the feature that lies
    // directly below it and the feature that lies directly above it.
    list<Vd_entry> vd_list;
    CGAL::decompose(_arr, back_inserter(vd_list));

    // Go over the vertices (given in ascending lexicographical xy-order),
    // and add segements to the feautres below and above it.
    const typename Kernel::Equal_2 equal = ker.equal_2_object();
    typename list<Vd_entry>::iterator it, prev = vd_list.end();
    for (it = vd_list.begin(); it != vd_list.end(); ++it) {
        // If the feature above the previous vertex is not the current vertex,
        // add a vertical segment to the feature below the vertex.
        Vertex_const_handle v;
        if ((prev == vd_list.end()) ||
            !CGAL::assign(v, prev->second.second) ||
            !equal(v->point(), it->first->point())) {
            addVerticalSegment(_arr.non_const_handle(it->first),
                               it->second.first, ker);
        }
        // Add a vertical segment to the feature above the vertex.
        addVerticalSegment(_arr.non_const_handle(it->first),
                           it->second.second, ker);
        prev = it;
    }
}

