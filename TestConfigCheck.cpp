#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include "TSPathFinder.h"
using namespace std;


Point_2 loadPoint_2(std::ifstream &is) {
    Kernel::FT x, y;
    is >> x >> y;
    Point_2 point(x, y);
    return point;
}

Polygon_2 loadPolygon(ifstream &is) {
    size_t polygon_size = 0;
    is >> polygon_size;
    Polygon_2 ret;
    while (polygon_size--)
        ret.push_back(loadPoint_2(is));
    CGAL::Orientation orient = ret.orientation();
    if (CGAL::COUNTERCLOCKWISE == orient)
        ret.reverse_orientation();
    return ret;
}

vector<Polygon_2> loadPolygons(ifstream &is) {
    size_t number_of_polygons = 0;
    is >> number_of_polygons;
    vector<Polygon_2> ret;
    while (number_of_polygons--)
        ret.push_back(loadPolygon(is));
    return ret;
}

void
loadTest(ifstream& is, TSMove& mv, bool& answer)
{
    mv.src_pt_ = loadPoint_2(is);
    mv.dst_pt_ = loadPoint_2(is);
    mv.obs_pt_ = loadPoint_2(is);
    int a;
    is >> a;
    answer = a != 0;
}

void 
printPoint(const Point_2& p)
{
    cout << "(" << p.x() << "," << p.y() << ")";
}

void
reportTest(TSMove& mv, bool b_expected, bool b_actual)
{
    printPoint( mv.src_pt_);
    cout << " => ";
    printPoint( mv.dst_pt_ );
    cout << " # ";
    printPoint( mv.obs_pt_ );
    cout << " Expected: " << (b_expected? "True" : "False");
    cout << " Obtained: " << (b_actual? "True" : "False");
    if( b_expected == b_actual)
        cout << "\tSUCCESS";
    else
        cout << "\tFAILED";
    cout << endl;
}

void
testMoves(ifstream& is,
         const Polygon_2 &outer_obstacle, 
         vector<Polygon_2> &obstacles) 
{
    ConfigCheck scene(outer_obstacle, obstacles);
    size_t n_tests = 0;
    is >> n_tests;
    while( n_tests-- )
    {
        TSMove mv;
        bool b_expected = true;
        loadTest(is, mv, b_expected);
        bool b_actual = scene.isLegalMove(mv);
        reportTest(mv, b_expected, b_actual); 
    }
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        cerr << "[USAGE]: inputObstacles movesTests" << endl;
        return 1;
    }

    ifstream movesTestsFile(argv[2]), inputObstaclesFile(argv[1]);
    if (!movesTestsFile.is_open() || !inputObstaclesFile.is_open()) {
        if (!movesTestsFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
        if (!inputObstaclesFile.is_open()) cerr << "ERROR: Couldn't open file: " << argv[1] << endl;
        return -1;
    }
    auto outer_obstacle = loadPolygon(inputObstaclesFile);
    auto obstacles = loadPolygons(inputObstaclesFile);
    inputObstaclesFile.close();

    boost::timer timer;
    testMoves(movesTestsFile, outer_obstacle, obstacles);
    auto secs = timer.elapsed();
    cout << "Tests done in:      " << secs << " secs" << endl;
    movesTestsFile.close();
    return 0;
}
