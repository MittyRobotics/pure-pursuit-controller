package pure_pursuit;

import pure_pursuit.enums.PathType;
import pure_pursuit.paths.BezierCurvePath;

public class PathGenerator {
	PathType type;
	VelocityConstraints velocityConstraints;
	public PathGenerator(PathType type, VelocityConstraints velocityConstraints){
		this.type = type;
		this.velocityConstraints = velocityConstraints;
	}
	public Path generate(Waypoint[] waypoints, int steps){
		if(type == PathType.BEZIER_CURVE_PATH){
			Path path = new Path(velocityConstraints.getMaxAcceleration(), velocityConstraints.getMaxVelocity(), new BezierCurvePath(waypoints, steps));
			path.generatePath();
			path.calculateDistances();
			path.calculateCurvature();
			path.calculateVelocities();
			return path;
		}
		else{
			return null;
		}
	}

}
