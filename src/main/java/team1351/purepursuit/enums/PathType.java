package team1351.purepursuit.enums;

import team1351.purepursuit.Waypoint;

/**
 * Path type enum containing all path types.
 *
 * @author Owen Leather
 * @version 1.0
 */
public enum PathType {
	/** Bezier curve path type. This path type generates a bezier curve to follow. */
	BEZIER_CURVE_PATH,
	/** Linear path type. This path type generates lines between {@link Waypoint}s making a set of linear segments to follow. */
	LINEAR_PATH
}
