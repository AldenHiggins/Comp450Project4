#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#include <vector>

typedef struct Point{
	double x;
	double y;
}Point;

typedef struct Rectangle{
	Point bottomLeftCorner;
	Point bottomRightCorner;
	Point topLeftCorner;
	Point topRightCorner;
}Rectangle;

class Environment{
public:
	Environment();
	~Environment();
	std::vector<Rectangle> getObstacles();
	double getStartAxis();
	double getEndAxis();
private:
	std::vector<Rectangle> obstacles;
	double startAxisValue;
	double endAxisValue;
};

#endif // ENVIRONMENT_H