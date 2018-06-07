#include "TSPathFinder.h"

cPoint::cPoint() {}

cPoint::cPoint(Point_2 p1, Point_2 p2) {
    this->robotA = p1;
    this->robotB = p2;
}

TSPathFinder::TSPathFinder( const Polygon_2&         outer_poly,
                            const vector<Polygon_2>& obstacles):
    outer_poly(outer_poly), obstacles(obstacles)
{
    this->createArrangment();
    this->setRandomPoints();
}

bool CmpEdges::operator()(const Edge lhs, const Edge rhs) const {
    if(lhs.distance != rhs.distance)
        return lhs.distance > rhs.distance;

    if(lhs.from->robotA != rhs.from->robotA)
        return lhs.from->robotA > rhs.from->robotA;

    if(lhs.from->robotB != rhs.from->robotB)
        return lhs.from->robotB > rhs.from->robotB;

    if(lhs.to->robotA != rhs.to->robotA)
        return lhs.to->robotA > rhs.to->robotA;

    return lhs.to->robotB > rhs.to->robotB;
}

void polygon_split_observer::after_split_face(Face_handle f1, Face_handle f2, bool)
{
    f2->set_contained(f1->contained());
}

void TSPathFinder::addEdge(cPoint *current, Point_2 robotA, Point_2 robotB, bool robotAmoved) {
    cPoint *temp;
    auto pp = pair<Point_2, Point_2>(robotA, robotB);
    auto it = cMap.find(pp);
    if(it != cMap.end())
    {
        temp = &it->second;
        if(temp->visited)
            return;
    } else {
        cPoint cp(robotA, robotB);
        if(!isConfigurationLegal(&cp))
            return;
        cMap.insert(pair<pair<Point_2, Point_2>, cPoint>(pp, cp));
        temp = &(cMap[pp]);
    }


    if(temp->heuristic < 0 )
        temp->heuristic = heuristic(temp);
    double newDistance = current->distance + cPointDistance(current, temp) + temp->heuristic;

    this->queue.push({current, temp, newDistance, robotAmoved});
}

void TSPathFinder::addNeighbors(cPoint *current) {
    list<Point_2> L1,L2;
    list<Point_2>::const_iterator it1,it2;
    Fuzzy_sphere rc1(current->robotA, RADIUS);
    Fuzzy_sphere rc2(current->robotB, RADIUS);

    tree.search(std::back_inserter(L1), rc1);
    int numOfEdgesA = 0;
    int numOfEdgesB = 0;
    for (it1 = L1.begin(); it1 != L1.end(); it1++) {
        if (*it1 != current->robotA)
        {
            addEdge(current, *it1, current->robotB, true);
            numOfEdgesA++;
        }
    }
    tree.search(std::back_inserter(L2), rc2);
    for (it2 = L2.begin(); it2 != L2.end(); it2++) {
        if (*it2 != current->robotB)
        {
            addEdge(current, current->robotA, *it2, false);
            numOfEdgesB++;
        }
    }
    //cout << "num added edges " << numOfEdgesA << " B " << numOfEdgesB << endl;

}

bool TSPathFinder::findPath( const Point_2& s1, const Point_2& e1, const Point_2& s2, const Point_2& e2 )
{
    tree.insert(s1);
    tree.insert(e1);
    tree.insert(s2);
    tree.insert(e2);

    cMap.insert(pair<pair<Point_2, Point_2>, cPoint>({s1, e1}, cPoint(s1, e1)));
    cMap.insert(pair<pair<Point_2, Point_2>, cPoint>({s2, e2}, cPoint(s2, e2)));

    startCPoint = &(cMap[pair<Point_2, Point_2>(s1,e1)]);
    startCPoint->visited = true;
    endCPoint = &(cMap[pair<Point_2, Point_2>(s2,e2)]);

    addNeighbors(&(cMap[pair<Point_2, Point_2>(s1,e1)]));

    while (!queue.empty()) {
        //cout << "queue size " << queue.size() << endl;
        Edge currentEdge = queue.top();
        queue.pop();
        if(!isEdgeLegal(currentEdge))
            continue;

        /*cout << "from A " <<    CGAL::to_double(currentEdge.from->robotA.x()) << " " <<
                                CGAL::to_double(currentEdge.from->robotA.y()) << " B " <<
                                CGAL::to_double(currentEdge.from->robotB.x()) << " " <<
                                CGAL::to_double(currentEdge.from->robotB.y()) << " to A " <<
                                CGAL::to_double(currentEdge.to->robotA.x()) << " " <<
                                CGAL::to_double(currentEdge.to->robotA.y()) << " B " <<
                                CGAL::to_double(currentEdge.to->robotB.x()) << " " <<
                                CGAL::to_double(currentEdge.to->robotB.y()) << " ";
        cout << (currentEdge.robotAMoved ? "A" : "B");
        cout << endl;*/

        cPoint* currentCpoint = currentEdge.to;

        currentCpoint->last = currentEdge.from;
        currentCpoint->distance = currentEdge.distance - currentEdge.to->heuristic;
        currentCpoint->visited = true;

        if(currentEdge.to == endCPoint)
            return true;

        addNeighbors(currentCpoint);

    }
    return false;
}



double TSPathFinder::cPointDistance(cPoint *a, cPoint *b) {
    double xAdiff = CGAL::to_double(a->robotA.x()) - CGAL::to_double(b->robotA.x());
    double yAdiff = CGAL::to_double(a->robotA.y()) - CGAL::to_double(b->robotA.y());
    double xBdiff = CGAL::to_double(a->robotB.x()) - CGAL::to_double(b->robotB.x());
    double yBdiff = CGAL::to_double(a->robotB.y()) - CGAL::to_double(b->robotB.y());

    return xAdiff*xAdiff + yAdiff*yAdiff + xBdiff*xBdiff + yBdiff*yBdiff;
}

double TSPathFinder::heuristic(cPoint *cp) {
    return cPointDistance(cp, this->endCPoint);
}

void TSPathFinder::createArrangment() {
    Polygon_set_2 obstacles_set;

    Polygon_with_holes_2 space(outer_poly, obstacles.begin(), obstacles.end());
    obstacles_set.insert(space);

    arr = obstacles_set.arrangement();

    //ensure that when face split two side safe their property (inside/outside)
    Polygon_set_2::Traits_2 traits;
    polygon_split_observer observer;
    observer.attach(arr);
    Kernel *ker = &traits;
    verticalDecomposition(*ker);
    observer.detach();

    pl.attach(arr);
}



bool TSPathFinder::isEdgeLegal(Edge edge) {
    if(edge.to->visited)
        return false;

    Point_2 stadyPoint = edge.robotAMoved ? edge.to->robotB : edge.to->robotA;
    Point_2 startPoint = edge.robotAMoved ? edge.from->robotA : edge.from->robotB;
    Point_2 endPoint = edge.robotAMoved ? edge.to->robotA : edge.to->robotB;

    Segment_2 querySegment(startPoint, endPoint);

    vector<CGAL::Object> vecZoneElems;
    Face_handle hFace;

    CGAL::zone(arr, querySegment, std::back_inserter(vecZoneElems), pl);
    for (int i = 0; i < vecZoneElems.size(); ++i) {
        if (CGAL::assign(hFace, vecZoneElems[i]) && !hFace->contained())
            return false;
    }
    Point_2 closestPoint = getClosestPoint(startPoint, endPoint, stadyPoint);
    return !(abs(closestPoint.x() - stadyPoint.x()) < 1 && abs(closestPoint.y()-stadyPoint.y())<1);


}

Point_2 TSPathFinder::getClosestPoint(Point_2 start, Point_2 end, Point_2 stady) {
    Vector_2 AB(start, end);
    Vector_2 AP(start, stady);
    double lengthSqrAB = CGAL::to_double(AB.x() * AB.x() + AB.y() * AB.y());
    double t = CGAL::to_double(AP.x() * AB.x() + AP.y() * AB.y()) / lengthSqrAB;
    if(t < 0)
        t = 0;
    if(t > 1)
        t = 1;

    return start + AB * t;
}


bool TSPathFinder::isConfigurationLegal(cPoint *current) {
    return !(abs(current->robotA.x() - current->robotB.x()) < 1 && abs(current->robotA.y()-current->robotB.y())<1);

}

bool TSPathFinder::inLegalFace(const Point_2 &p) {
    CGAL::Object obj = pl.locate(p); //find p in pl

    Vertex_const_handle vertex;
    if (CGAL::assign(vertex, obj)) {
        Face_iterator it = arr.faces_begin();
        for(;it!=arr.faces_end();it++)
        {
            if(it == arr.unbounded_face())
                continue;

            ccb_haledge_circulator first = it->outer_ccb();
            ccb_haledge_circulator circ = first;
            do {
                Halfedge_const_handle temp = circ;
                if(temp->source()->point() == vertex->point())
                {
                    if(it->contained())
                        return true;
                    else
                        break;
                }
            } while (++circ != first);
        }
        return false;
    }

    Halfedge_const_handle  helfEdge; //check it's a halfedge
    if (CGAL::assign(helfEdge, obj)) {
        if (helfEdge->face()->contained())
            return true;
        else if(helfEdge->twin()->face()->contained())
            return true;
        return false;
    }

    // Check whether the point is contained inside a free bounded face.
    Face_const_handle face;
    if (CGAL::assign(face, obj)) //if obj is face
    {
        if(face->contained())
            return true;
    }
    return false;
}

Path TSPathFinder::fetchPath() {
    Path tempVector;
    cPoint *temp = this->endCPoint;
    cPoint *start = this->startCPoint;
    while (temp != start) {
        tempVector.push_back(pair<Point_2, Point_2>(temp->robotA, temp->robotB));
        temp = temp->last;
    }
    Path path;
    path.push_back(pair<Point_2, Point_2>(this->startCPoint->robotA, this->startCPoint->robotB));
    for (int i = static_cast<int>(tempVector.size() - 1); i >= 0; i--)
        path.push_back(tempVector[i]);

    return path;

}

void TSPathFinder::setFaceRandomPoints(Face_handle face) {
    ccb_haledge_circulator first = face->outer_ccb();
    ccb_haledge_circulator circ = first;
    vector<Point_2> points;
    do {
        Halfedge_const_handle temp = circ;
        points.emplace_back(temp->source()->point());
    } while (++circ != first);
    FT minx = points[0].x();
    FT miny = points[0].y();
    FT maxx = minx;
    FT maxy = miny;

    for(Point_2 p:points)
    {
        if(p.x() < minx)
            minx = p.x();
        else if(p.x() > maxx)
            maxx = p.x();

        if(p.y() < miny)
            miny = p.y();
        else if(p.y() > maxy)
            maxy = p.y();
    }


    double faceSize = CGAL::to_double((maxx-minx) * (maxy - miny));

    uniform_real_distribution<double> xUnif = uniform_real_distribution<double>(CGAL::to_double(minx), CGAL::to_double(maxx));
    uniform_real_distribution<double> yUnif = uniform_real_distribution<double>(CGAL::to_double(miny), CGAL::to_double(maxy));
    std::default_random_engine re;

    int numberOfPoints = (int)(faceSize*NUM_OF_POINTS_PER_SQUARE);
    vector<Point_2> cpoints;
    int numOfPoints = 0;
    for(int i=0; i<numberOfPoints; i++) {
        Point_2 p = {xUnif(re), yUnif(re)};
        if (inLegalFace(p))
        {
            tree.insert(p);
            numOfPoints++;
        }
    }
    //cout << "num of points " << numOfPoints << endl;

}

void TSPathFinder::setRandomPoints() {
    Face_iterator it = arr.faces_begin();
    for(;it!=arr.faces_end();it++)
    {
        if(!it->contained())
            continue;
        setFaceRandomPoints(it);
    }
}

void TSPathFinder::addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker) {
    X_monotone_curve_2 seg;
    Vertex_const_handle vh;
    Halfedge_const_handle hh;
    Face_const_handle fh;
    Vertex_handle v2;

    if (CGAL::assign(vh, obj)) { // The given feature is a vertex.
        seg = X_monotone_curve_2(v->point(), vh->point());
        v2 = arr.non_const_handle(vh);
    } else if (CGAL::assign(hh, obj)) { // The given feature is a halfedge.
        if (hh->is_fictitious()) //We ignore fictitious halfedges.
            return;

        // Check whether v lies in the interior of the x-range of the edge (in
        // which case this edge should be split).
        const typename Kernel::Compare_x_2 cmp_x = ker.compare_x_2_object();
        if (cmp_x(v->point(), hh->target()->point()) == CGAL::EQUAL) {
            // In case the target of the edge already has the same x-coordinate as
            // the vertex v, just connect these two vertices.
            seg = X_monotone_curve_2(v->point(), hh->target()->point());
            v2 = arr.non_const_handle(hh->target());
        }
        else {
            // Compute the vertical projection of v onto the segment associated
            // with the halfedge. Split the edge and connect v with the split point.
            Line_2 Line;
            Line_2 supp_line(hh->source()->point(), hh->target()->point());
            Line_2 vert_line(v->point(), Point_2(v->point().x(), v->point().y() + 1));
            Point_2  point;
            CGAL::assign(point, ker.intersect_2_object()(supp_line, vert_line));
            seg = X_monotone_curve_2(v->point(), point);
            arr.split_edge(arr.non_const_handle(hh),
                           X_monotone_curve_2(hh->source()->point(), point),
                           X_monotone_curve_2(point, hh->target()->point()));
            v2 = arr.non_const_handle(hh->target());
        }
    } else // Ignore faces and empty objects.
        return;

    // Add the vertical segment to the arrangement using its two end vertices.
    arr.insert_at_vertices(seg, v, v2);
}

void TSPathFinder::verticalDecomposition(Kernel &ker) {
    typedef pair<Vertex_const_handle, pair<CGAL::Object, CGAL::Object> > Vd_entry;

    // For each vertex in the arrangment, locate the feature that lies
    // directly below it and the feature that lies directly above it.
    list<Vd_entry>   vd_list;
    CGAL::decompose(arr, back_inserter(vd_list));

    // Go over the vertices (given in ascending lexicographical xy-order),
    // and add segements to the feautres below and above it.
    const typename Kernel::Equal_2 equal = ker.equal_2_object();
    typename list<Vd_entry>::iterator  it, prev = vd_list.end();
    for (it = vd_list.begin(); it != vd_list.end(); ++it) {
        // If the feature above the previous vertex is not the current vertex,
        // add a vertical segment to the feature below the vertex.
        Vertex_const_handle v;
        if ((prev == vd_list.end()) ||
            !CGAL::assign(v, prev->second.second) ||
            !equal(v->point(), it->first->point()))
            addVerticalSegment(arr.non_const_handle(it->first), it->second.first, ker);
        // Add a vertical segment to the feature above the vertex.
        addVerticalSegment(arr.non_const_handle(it->first), it->second.second, ker);
        prev = it;
    }
}
