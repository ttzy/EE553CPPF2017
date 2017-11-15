#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
using namespace std;

const double pi = 3.1415926535897;
class Shape {
private:
  double x,y,z;
public:
  Shape(double x, double y, double z) : x(x), y(y), z(z) {}
  double getX() {return x;}
  double getY() {return y;}
  double getZ() {return z;}
  virtual void write(const char filename[]) = 0;
  virtual void vertices() = 0;
  virtual ~Shape() {}
};

class Cylinder : public Shape {
private:
  double r, h;
  double facets;
  vector < vector<double> > pointstop;
  vector < vector<double> > pointsbot;
public:
  Cylinder(double x, double y, double z, double r, double h, double facets) : Shape(x, y, z), r(r), h(h), facets(facets) {}
  void vertices() {
    for (int i = 0; i < facets; i++) {
      vector<double> temp;
      temp.push_back(getX() + r * cos(2*pi/facets*i));
      temp.push_back(getY() + r * sin(2*pi/facets*i));
      temp.push_back(getZ() + h/2);
      pointstop.push_back(temp);
    }
    for (int i = 0; i < facets; i++) {
      vector<double> temp;
      temp.push_back(getX() + r * cos(2*pi/facets*i));
      temp.push_back(getY() + r * sin(2*pi/facets*i));
      temp.push_back(getZ() - h/2);
      pointsbot.push_back(temp);
    }
  }
  void normal(double a, double b, double& cx, double& cy, double& cz, vector < vector<double> > points ) {
    double dx = points[a][0] - points[b][0];
    double dy = points[a][1] - points[b][1];
    double dz = points[a][2] - points[b][2];
    double dx1 = points[a][0] - getX();
    double dy1 = points[a][1] - getY();
    double dz1 = points[a][2] - getZ();
    cx = dy * dz1 - dz * dy1;
    cy = dz * dx1 - dx * dz1;
    cz = dx * dy1 - dy * dx1;
    double magnitude = sqrt(cx * cx + cy * cy + cz * cz);
    cx = cx/magnitude;
    cy = cy/magnitude;
    cz = cz/magnitude;
  }
  void normalside(double a, double b, double& cx, double& cy, double& cz) {
    double dx = pointstop[a][0] - pointsbot[a][0];
    double dy = pointstop[a][1] - pointsbot[a][1];
    double dz = pointstop[a][2] - pointsbot[a][2];
    double dx1 = pointstop[a][0] - pointstop[b][0];
    double dy1 = pointstop[a][1] - pointstop[b][1];
    double dz1 = pointstop[a][2] - pointstop[b][2];
    cx = dy * dz1 - dz * dy1;
    cy = dz * dx1 - dx * dz1;
    cz = dx * dy1 - dy * dx1;
    double magnitude = sqrt(cx * cx + cy * cy + cz * cz);
    cx = cx/magnitude;
    cy = cy/magnitude;
    cz = cz/magnitude;
  }
  void triangle(double a, double b, ofstream& of) {
    double cx, cy, cz;
    normal(a, b, cx, cy, cz, pointstop);
    of << "facet normal " << cx << ' ' << cy  << ' ' << cz << '\n';
    of << "  outer loop" << '\n';
    of << "    vertex " << getX() <<' ' <<  getY() << ' ' << getZ() + h/2 << '\n';
    of << "    vertex " << pointstop[a][0] <<' ' <<  pointstop[a][1] << ' ' << pointstop[a][2] << '\n';
    of << "    vertex " << pointstop[b][0] << ' ' << pointstop[b][1] << ' ' <<pointstop[b][2]  << '\n';
    of << "  endloop" << '\n';
    of << "endfacet" << '\n';
    double ex, ey, ez;
    normal(a, b, ex, ey, ez, pointsbot);
    of << "facet normal " << ex << ' ' << ey << ' ' << ez << '\n';
    of << "  outer loop" << '\n';
    of << "    vertex " << getX() <<' ' <<  getY() << ' ' << getZ() - h/2 << '\n';
    of << "    vertex " << pointsbot[a][0] <<' ' <<  pointsbot[a][1] << ' ' << pointsbot[a][2] << '\n';
    of << "    vertex " << pointsbot[b][0] << ' ' << pointsbot[b][1] << ' ' <<pointsbot[b][2]  << '\n';
    of << "  endloop" << '\n';
    of << "endfacet" << '\n';
  }
  void triangleside(double a, double b, ofstream& of) {
    double cx, cy, cz;
    normalside(a, b, cx, cy, cz);
    of << "facet normal " << cx << ' ' << cy << ' ' << cz << '\n';
    of << "  outer loop" << '\n';
    of << "    vertex " << pointstop[a][0] <<' ' << pointstop[a][1] << ' ' << pointstop[a][2] << '\n';
    of << "    vertex " << pointsbot[a][0] <<' ' <<  pointsbot[a][1] << ' ' << pointsbot[a][2] << '\n';
    of << "    vertex " << pointstop[b][0] << ' ' << pointstop[b][1] << ' ' <<pointstop[b][2]  << '\n';
    of << "  endloop" << '\n';
    of << "endfacet" << '\n';
    of << "facet normal " << cx << ' ' << cy << ' ' << cz << '\n';
    of << "  outer loop" << '\n';
    of << "    vertex " << pointsbot[a][0] <<' ' << pointsbot[a][1] << ' ' << pointsbot[a][2] << '\n';
    of << "    vertex " << pointsbot[b][0] <<' ' <<  pointsbot[b][1] << ' ' << pointsbot[b][2] << '\n';
    of << "    vertex " << pointstop[b][0] << ' ' << pointstop[b][1] << ' ' <<pointstop[b][2]  << '\n';
    of << "  endloop" << '\n';
    of << "endfacet" << '\n';
  }
  void write(const char filename[]) {
    ofstream of;
    of.open(filename,fstream::app);
    for (int i = 0; i < facets; i++) {
      if (i == facets-1) {
        triangle(facets-1,0,of);
      }
      else {
        triangle(i,i+1,of);
      }
    }
    for (int i = 0; i < facets; i++) {
      if (i == facets-1) {
        triangleside(facets-1,0,of);
      }
      else {
        triangleside(i,i+1,of);
      }
    }
    of.close();
  }
};

class Cube: public Shape {
private:
	double size;
  vector< vector<double> > points;
  vector< vector<double> > point;

public:
	Cube(double x, double y, double z, double size) : Shape(x, y, z), size(size) {}
  void vertices() {
    points.push_back({getX(), getY(), getZ()});
    for (int i = 0; i < 3; i++) {
      point = points;
      for (int j = 0; j < points.size(); j++) {
        vector<double> temp;
        temp = points[j];
        temp[i] = size;
        point.push_back(temp);
      }
      points = point;
    }
  }
  void normal(double a, double b, double c, double& cx, double& cy, double& cz) {
    double dx = points[a][0] - points[b][0];
    double dy = points[a][1] - points[b][1];
    double dz = points[a][2] - points[b][2];
    double dx1 = points[a][0] - points[c][0];
    double dy1 = points[a][1] - points[c][1];
    double dz1 = points[a][2] - points[c][2];
    cx = dy * dz1 - dz * dy1;
    cy = dz * dx1 - dx * dz1;
    cz = dx * dy1 - dy * dx1;
    double magnitude = sqrt(cx * cx + cy * cy + cz * cz);
    cx = cx/magnitude;
    cy = cy/magnitude;
    cz = cz/magnitude;
  }
  void triangle(double a, double b, double c, ofstream& of) {
    double cx, cy, cz;
    normal(a, b, c, cx, cy, cz);
    of << "facet normal " << cx << ' ' << cy << ' ' << cz << ' ' << '\n';
    of << "  outer loop" << '\n';
    of << "    vertex " << points[a][0] <<' ' <<  points[a][1] << ' ' << points[a][2] << '\n';
    of << "    vertex " << points[b][0] <<' ' <<  points[b][1] << ' ' << points[b][2] << '\n';
    of << "    vertex " << points[c][0] << ' ' << points[c][1] << ' ' <<points[c][2]  << '\n';
    of << "  endloop" << '\n';
    of << "endfacet" << '\n';
  }
  void write(const char filename[]) {
    ofstream of;
    of.open(filename, fstream::app);
    triangle(0,1,2,of);
    triangle(1,2,3,of);
    triangle(4,5,6,of);
    triangle(5,6,7,of);
    triangle(0,2,4,of);
    triangle(2,4,6,of);
    triangle(2,3,6,of);
    triangle(3,6,7,of);
    triangle(1,3,5,of);
    triangle(3,5,7,of);
    triangle(0,1,4,of);
    triangle(1,4,5,of);
    of.close();
  }
};

class CAD {
private:
	vector<Shape*> shapes;
public:
  CAD() : shapes() {}
  ~CAD() {
    for (auto s : shapes) {
      delete s;
    }
  }
  // add functions
  void add(Shape* ss) {
    shapes.push_back(ss);
  }
  void write(const char filename[]) {
    for (auto s : shapes) {
      s -> vertices();
      s -> write(filename);
    }
  }
};

int main() {
	CAD c;
	c.add(new Cube(0,0,0,5));
	c.add(new Cylinder(100,0,0,3,10,10));
  c.write("test.stl");
}
