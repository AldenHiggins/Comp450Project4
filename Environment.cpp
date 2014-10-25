#include "Environment.h"

void addPoint(std::vector<Point> *addToThis, double x, double y)
{
	Point newPoint = {x,y};
	addToThis->push_back(newPoint);
}

Environment::Environment()
{
	std::vector<Point> botLeft;
	std::vector<Point> botRight;
	std::vector<Point> topLeft;
	std::vector<Point> topRight;

	addPoint(&botLeft,-10,6);
	addPoint(&botRight,10,6);
	addPoint(&topLeft,-10,8);
	addPoint(&topRight,10,8);

	addPoint(&botLeft,-10,-4);
	addPoint(&botRight,0,-4);
	addPoint(&topLeft,-10,4);
	addPoint(&topRight,0,4);

	addPoint(&botLeft,3,-4);
	addPoint(&botRight,10,-4);
	addPoint(&topLeft,3,4);
	addPoint(&topRight,10,4);

	addPoint(&botLeft,-10,-10);
	addPoint(&botRight,10,-10);
	addPoint(&topLeft,-10,-6);
	addPoint(&topRight,10,-6);

	for (int rectangleIndex = 0; rectangleIndex < botLeft.size(); rectangleIndex++)
	{
		Rectangle newRec;
		newRec.bottomLeftCorner = botLeft[rectangleIndex];
		newRec.bottomRightCorner = botRight[rectangleIndex];
		newRec.topLeftCorner = topLeft[rectangleIndex];
		newRec.topRightCorner = topRight[rectangleIndex];
		obstacles.push_back(newRec);
	}

	startAxisValue = -10;
	endAxisValue = 10;
}

Environment::~Environment()
{

}

std::vector<Rectangle> Environment::getObstacles()
{
	return obstacles;
}

double Environment::getStartAxis()
{
	return startAxisValue;
}

double Environment::getEndAxis()
{
	return endAxisValue;
}