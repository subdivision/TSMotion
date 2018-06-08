
#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <vector>
#include <random>
#include <math.h>
#include <map>
#include <queue>

#include "CGAL_defines.h"

#define NUM_OF_POINTS_PER_SQUARE 10
#define RADIUS 1

using namespace std;

typedef vector<pair<Point_2, Point_2>> Path;

struct cPoint;
struct Edge;


struct CmpEdges
{
    bool operator()(const Edge lhs, const Edge rhs) const;
};

class polygon_split_observer : public CGAL::Arr_observer<Arrangement_2> {
    void after_split_face(Face_handle f1, Face_handle f2, bool) override;
};

struct cPoint
{
public:
    cPoint();
    cPoint(Point_2 p1, Point_2 p2);
    Point_2 robotA, robotB;
    cPoint* last = nullptr;
    bool visited = false;
    double heuristic=-1;
    double distance = 0;
};

struct Edge{
    cPoint *from, *to;
    double distance;
    bool robotAMoved;
};

class PathFinder
{
private:
    Polygon_2& outer_poly;
    vector<Polygon_2>& obstacles;
    Arrangement_2 arr;
    Landmarks_pl pl;
    cPoint *startCPoint, *endCPoint;
    map<pair<Point_2, Point_2>, cPoint> cMap;
    priority_queue<Edge, vector<Edge>, CmpEdges> queue;
    Tree tree;

    void verticalDecomposition(Kernel &ker);
    void addVerticalSegment(Vertex_handle v, CGAL::Object obj, Kernel &ker);
    bool inLegalFace(const Point_2 &p);
    void createArrangment();

    void setRandomPoints();
    void setFaceRandomPoints(Face_handle face);
    double heuristic(cPoint *cp);
    double cPointDistance(cPoint *a, cPoint *b);

    void addEdge(cPoint *current, Point_2 robotA, Point_2 robotB, bool robotAmoved);
    void addNeighbors(cPoint *current);

    bool isConfigurationLegal(cPoint *current);
    bool isEdgeLegal(Edge edge);


public:
    PathFinder( Polygon_2& outer_poly,
                  vector<Polygon_2>& obstacles );
    bool findPath( const Point_2& s1, const Point_2& e1,
                   const Point_2& s2, const Point_2& e2 );
    Path fetchPath();
};

#endif //PATHFINDER_H
