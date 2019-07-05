package team1351.purepursuit;

import team1351.purepursuit.enums.PathType;
import team1351.purepursuit.paths.BezierCurvePath;
import team1351.purepursuit.paths.LinearPath;

/**
 * The master path generator object.
 *
 * @author Owen Leather
 * @version 1.0
 */
public class PathGenerator {
	private static PathGenerator ourInstance = new PathGenerator();

	/** kCurvature value to be passed into the generated path */
	private double kCurvature = 0.8;

	/**
	 * Returns the singleton instance.
	 *
	 * @return the instance of the {@link PathGenerator}.
	 */
	public static PathGenerator getInstance() {
		return ourInstance;
	}

	/**
	 * Constructor
	 */
	private PathGenerator() {
	}

	/**
	 * Generates a path based on the parameters.
	 *
	 * @param waypoints       the set of {@link Waypoint}s that define the path.
	 * @param type            the type of path that is generated.
	 * @param maxAcceleration the maximum acceleration value of the robot.
	 * @param maxVelocity     the maximum velocity value of the robot.
	 * @param steps           Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.
	 * @return a {@link Path} object generated based on the parameters.
	 */
	public Path generate(Waypoint[] waypoints, PathType type, double maxAcceleration, double maxVelocity, int steps) {
		return generate(waypoints,type,maxAcceleration,maxVelocity,0,0,steps);
	}

	/**
	 * Generates a path based on the parameters.
	 *
	 * @param waypoints       the set of {@link Waypoint}s that define the path.
	 * @param type            the type of path that is generated.
	 * @param maxAcceleration the maximum acceleration value of the robot.
	 * @param maxVelocity     the maximum velocity value of the robot.
	 * @param startVelocity   the robot starting forward velocity.
	 * @param endVelocity     the robot ending forward velocity.
	 * @param steps           Number of points generated. The more steps, the longer it takes to generate but the more accurate the path generation will be.
	 * @return a {@link Path} object generated based on the parameters.
	 */
	public Path generate(Waypoint[] waypoints, PathType type, double maxAcceleration, double maxVelocity, double startVelocity, double endVelocity, int steps) {
		if (type == PathType.BEZIER_CURVE_PATH) {
			Path path = new Path(maxAcceleration, maxVelocity, startVelocity, endVelocity, new BezierCurvePath(waypoints, steps));
			path.setKCurvature(kCurvature);
			path.generatePath();
			path.calculateDistances();
			path.calculateCurvature();
			path.calculateVelocities();
			return path;
		} else if (type == PathType.LINEAR_PATH) {
			Path path = new Path(maxAcceleration, maxVelocity, startVelocity, endVelocity, new LinearPath(waypoints, steps));
			path.setKCurvature(kCurvature);
			path.generatePath();
			path.calculateDistances();
			//path.calculateCurvature();
			path.calculateVelocities();
			return path;
		} else {
			return null;
		}
	}

	/**
	 * Sets the kCurvature value of the path generator.
	 * <p>
	 * This value is then passed into the generated path.
	 *
	 * @param kCurvature the kCurvature value to be passed into the generated path.
	 */
	public void setPathKCurvature(double kCurvature) {
		this.kCurvature = kCurvature;
	}

}
