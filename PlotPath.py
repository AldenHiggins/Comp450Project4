import matplotlib.pyplot as plt

def main():
	# Read in file, if not exit!
	try:
		results = open('pathResults','r')
	except IOError:
		print "Path results not found!"
		return
	# Set axis of the plot
	axisLine = results.readline()
	axisNums = [float(x) for x in axisLine.split(":")]
	plt.axis([axisNums[0],axisNums[1],axisNums[0],axisNums[1]])
	# Read in the obstacles
	numberObstacles = results.readline()
	for i in range(0, int(numberObstacles)):
		newObstacle = results.readline()
		obs = [float(x) for x in newObstacle.split(":")]
		# Bottom Edge
		plt.plot([obs[0],obs[2]],[obs[1],obs[3]], color = 'black')
		# Left Edge
		plt.plot([obs[0],obs[4]],[obs[1],obs[5]], color = 'black')
		# Right Edge
		plt.plot([obs[2],obs[6]],[obs[3],obs[7]], color = 'black')
		# Top Edge
		plt.plot([obs[4],obs[6]],[obs[5],obs[7]], color = 'black')
	# Read in the path
	numberPathPointsLine = results.readline()
	numberPathPoints = numberPathPointsLine.split(" ")
	# Set the initial path point
	results.readline()
	pathPointLine = results.readline()
	# Handle the SE2 case
	if (pathPointLine == "Compound state [\n"):
		# results.readline()
		pathPoint = results.readline().split(" ")
		# Read in additional lines for so2state, ], and compound state ]
		results.readline()
		results.readline()
		results.readline()
		results.readline()
		results.readline()
		results.readline()
		previousX = int(pathPoint[1].replace("[", ""))
		previousY = int(pathPoint[2].replace("]\n", ""))
		# Plot the steps of the path
		for i in range(0, int(numberPathPoints[3]) - 1):
			pathPointLine = results.readline()
			pathPoint = pathPointLine.split(" ")
			xValue = pathPoint[1].replace("[", "")
			yValue = pathPoint[2].replace("]\n", "")
			plt.plot([previousX, xValue],[previousY, yValue], color = 'red')
			previousX = xValue
			previousY = yValue
			# Read in additional lines for so2state, ], and compound state ]
			results.readline()
			results.readline()
			results.readline()
			results.readline()
			results.readline()
			results.readline()
	# Handle the R2 case
	else:
		pathPoint = pathPointLine.split(" ")
		previousX = float(pathPoint[1].replace("[", "").replace("]\n", ""))
		pathPoint = results.readline().split(" ")
		previousY = float(pathPoint[1].replace("[", "").replace("]\n", ""))
		# Plot the steps of the path
		for i in range(0, int(numberPathPoints[3]) - 1):
			# Skip two lines to get to the next point
			results.readline()
			results.readline()
			pathPointLine = results.readline()
			pathPoint = pathPointLine.split(" ")
			xValue = float(pathPoint[1].replace("[", "").replace("]\n", ""))
			pathPoint = results.readline().split(" ")
			yValue = float(pathPoint[1].replace("[", "").replace("]\n", ""))
			plt.plot([previousX, xValue],[previousY, yValue], color = 'red')
			previousX = xValue
			previousY = yValue
	plt.show()

if __name__ == "__main__":
    main()
