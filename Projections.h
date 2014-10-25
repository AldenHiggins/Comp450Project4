#ifndef PROJECTIONS_H__
#define PROJECTIONS_H__

using namespace ompl;

class PendulumProjection : public base::ProjectionEvaluator
{
public:
	PendulumProjection(const base::StateSpacePtr &space) : base::ProjectionEvaluator(space)
	{
	}
	virtual unsigned int getDimension(void) const
	{
	    return 1;
	}
	virtual void defaultCellSizes(void)
	{
	    cellSizes_.resize(1);
	    cellSizes_[0] = 0.1;
	    // cellSizes_[1] = 0.25;
	}
	virtual void project(const base::State *state, base::EuclideanProjection &projection) const
	{
	    const double value = state->as<base::CompoundStateSpace::StateType>()->as<base::SO2StateSpace::StateType>(0)->value;
	    projection[0] = value;
	}
};

class CarProjection : public base::ProjectionEvaluator
{
public:
	CarProjection(const base::StateSpacePtr &space) : base::ProjectionEvaluator(space)
	{
	}
	virtual unsigned int getDimension(void) const
	{
	    return 2;
	}
	virtual void defaultCellSizes(void)
	{
	    cellSizes_.resize(2);
	    cellSizes_[0] = 0.25;
	    cellSizes_[1] = 0.25;
	}
	virtual void project(const base::State *state, base::EuclideanProjection &projection) const
	{
	    const double xValue = state->as<base::CompoundStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(0)->getX();
	    const double yValue = state->as<base::CompoundStateSpace::StateType>()->as<base::SE2StateSpace::StateType>(0)->getY();
	    projection[0] = xValue;
	    projection[1] = yValue;
	}
};

#endif // PROJECTIONS_H__