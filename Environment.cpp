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


	addPoint(&botLeft,2.5,1);
	addPoint(&botRight,3,1);
	addPoint(&topLeft,2.5,2);
	addPoint(&topRight,3,2);

	addPoint(&botLeft,2,6);
	addPoint(&botRight,7,6);
	addPoint(&topLeft,2,7);
	addPoint(&topRight,7,7);

	addPoint(&botLeft,4,2);
	addPoint(&botRight,5,2);
	addPoint(&topLeft,4,4);
	addPoint(&topRight,5,4);

	addPoint(&botLeft,6,2);
	addPoint(&botRight,7,2);
	addPoint(&topLeft,6,5.8);
	addPoint(&topRight,7,5.8);

	addPoint(&botLeft,2,9);
	addPoint(&botRight,3,9);
	addPoint(&topLeft,2,10);
	addPoint(&topRight,3,10);

	addPoint(&botLeft,4,7);
	addPoint(&botRight,5,7);
	addPoint(&topLeft,4,9);
	addPoint(&topRight,5,9);

	addPoint(&botLeft,7,9);
	addPoint(&botRight,8,9);
	addPoint(&topLeft,7,10);
	addPoint(&topRight,8,10);

	for (int rectangleIndex = 0; rectangleIndex < botLeft.size(); rectangleIndex++)
	{
		Rectangle newRec;
		newRec.bottomLeftCorner = botLeft[rectangleIndex];
		newRec.bottomRightCorner = botRight[rectangleIndex];
		newRec.topLeftCorner = topLeft[rectangleIndex];
		newRec.topRightCorner = topRight[rectangleIndex];
		obstacles.push_back(newRec);
	}

	startAxisValue = 0;
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