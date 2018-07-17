#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <boost/timer.hpp>
#include "CGAL_defines.h"
#include <time.h>
#include <stdlib.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/enum.h>
#include <CGAL/minkowski_sum_2.h>

#define all(x) x.begin(), x.end()
#define all_vertices(x) x.vertices_begin(), x.vertices_end()

using namespace std;


Point_2 load_point_2(std::ifstream &is) {
	Kernel::FT x, y;
	is >> x >> y;
	Point_2 point(x, y);
	return point;
}

Polygon_2 load_polygon(ifstream &is) {
	size_t polygon_size = 0;
	is >> polygon_size;
	Polygon_2 ret;
	while (polygon_size--)
		ret.push_back(load_point_2(is));
	CGAL::Orientation orient = ret.orientation();
	if (CGAL::COUNTERCLOCKWISE == orient)
		ret.reverse_orientation();
	return ret;
}

vector<Polygon_2> load_polygons(ifstream &is) {
	size_t number_of_polygons = 0;
	is >> number_of_polygons;
	vector<Polygon_2> ret;
	while (number_of_polygons--)
		ret.push_back(load_polygon(is));
	return ret;
}

template <class InputIterator, class outputStream>
void print_polygon_in_line(InputIterator begin, InputIterator end, int n, outputStream &fp) {
	fp << n;
	while (begin != end) {
		fp << " " << begin->x().to_double() << " " << begin->y().to_double();
		++begin;
	}
	fp << endl;
}

template <class outputStream>
void print_polygon_with_holes(Polygon_with_holes_2 &pwh, outputStream &fp) {
	print_polygon_in_line(all_vertices(pwh.outer_boundary()), (int)pwh.outer_boundary().size(), fp);
	fp << pwh.number_of_holes() << endl;
	for (auto hole = pwh.holes_begin(); hole != pwh.holes_end(); ++hole) {
		print_polygon_in_line(all_vertices((*hole)), (int)hole->size(), fp);
	}
}

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

int main(int argc, char *argv[]) {
	if (argc != 3) {
		cerr << "[USAGE]: inputObstacles outputObstacles" << endl;
		return 1;
	}

	Polygon_2 minusRobot = get_robot();

	ifstream inputObstaclesFile(argv[1]);
	if (!inputObstaclesFile.is_open()) {
		cerr << "ERROR: Couldn't open file: " << argv[2] << endl;
		return -1;
	}
	auto outer_obstacle = load_polygon(inputObstaclesFile);
	auto obstacles = load_polygons(inputObstaclesFile);
	inputObstaclesFile.close();

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
	
	//assuming there is still one polygon with holes now!
	if (ps2.number_of_polygons_with_holes() != 1) {
		cerr << "The obstacles file outer boundary got SMASHED! and splitted into multiple parts. This is no longer a legal format and as such it was not accepted.";
		return -1;
	}
	ofstream outputFile;
	outputFile.open(argv[2]);
	if (!outputFile.is_open()) {
		cerr << "ERROR: Couldn't open file: " << argv[3] << endl;
		return -1;
	}
	vector<Polygon_with_holes_2> pwh;
	ps2.polygons_with_holes(back_inserter(pwh));
	for (auto pwh1 : pwh) {
		print_polygon_with_holes(pwh1, outputFile);
	}
	outputFile.close();
	return 0;
}