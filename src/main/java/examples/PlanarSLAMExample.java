package examples;

import gtsam.BearingRangeFactorPose2Point2;
import gtsam.BetweenFactorPose2;
import gtsam.Key;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.Marginals;
import gtsam.NonlinearFactorGraph;
import gtsam.Point2;
import gtsam.Pose2;
import gtsam.Rot2;
import gtsam.Values;
import gtsam.Vector2;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * A simple 2D planar slam example with landmarks
 * - The robot and landmarks are on a 2 meter grid
 * - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 * - The robot moves 2 meters each step
 * - We have full odometry between poses
 * - We have bearing and range information for measurements
 * - Landmarks are 2 meters away from the robot trajectory
 */
public class PlanarSLAMExample {

    public static void main(String args[]) throws Throwable {

        // Create a factor graph
        NonlinearFactorGraph graph = new NonlinearFactorGraph();

        // Create the keys we need for this simple example
        Key x1 = Key.X(1);
        Key x2 = Key.X(2);
        Key x3 = Key.X(3);
        Key l1 = Key.L(1);
        Key l2 = Key.L(2);

        // Add a prior on pose x1 at the origin. A prior factor consists of a mean and
        // a noise model (covariance matrix)
        Pose2 prior = new Pose2(0.0, 0.0, 0.0); // prior mean is at origin
        shared_ptr<Diagonal> priorNoise = Diagonal.Sigmas(new Vector3(0.3, 0.3, 0.1)); // 30cm std on x,y, 0.1 rad on theta
        graph.addPrior(x1, prior, priorNoise); // add directly to graph

        // Add two odometry factors
        Pose2 odometry = new Pose2(2.0, 0.0, 0.0);
        // create a measurement for both factors (the same in this case)
        shared_ptr<Diagonal> odometryNoise = Diagonal.Sigmas(new Vector3(0.2, 0.2, 0.1)); // 20cm std on x,y, 0.1 rad on theta
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(x1, x2, odometry, odometryNoise));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(x2, x3, odometry, odometryNoise));

        // Add Range-Bearing measurements to two different landmarks
        // create a noise model for the landmark measurements
        shared_ptr<Diagonal> measurementNoise = Diagonal.Sigmas(new Vector2(0.1, 0.2)); // 0.1 rad std on bearing, 20cm on range
        // create the measurement values - indices are (pose id, landmark id)
        Rot2 bearing11 = Rot2.fromDegrees(45);
        Rot2 bearing21 = Rot2.fromDegrees(90);
        Rot2 bearing32 = Rot2.fromDegrees(90);
        double range11 = Math.sqrt(4.0 + 4.0), range21 = 2.0, range32 = 2.0;

        // Add Bearing-Range factors
        graph.add(BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                x1, l1, bearing11, range11, measurementNoise));
        graph.add(BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                x2, l1, bearing21, range21, measurementNoise));
        graph.add(BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                x3, l2, bearing32, range32, measurementNoise));

        // Print
        graph.print("Factor Graph:\n");

        // Create (deliberately inaccurate) initial estimate
        Values initialEstimate = new Values();
        initialEstimate.insert(x1, new Pose2(0.5, 0.0, 0.2));
        initialEstimate.insert(x2, new Pose2(2.3, 0.1, -0.2));
        initialEstimate.insert(x3, new Pose2(4.1, 0.1, 0.1));
        initialEstimate.insert(l1, new Point2(1.8, 2.1));
        initialEstimate.insert(l2, new Point2(4.1, 1.8));

        // Print
        initialEstimate.print("Initial Estimate:\n");

        // Optimize using Levenberg-Marquardt optimization. The optimizer
        // accepts an optional set of configuration parameters, controlling
        // things like convergence criteria, the type of linear system solver
        // to use, and the amount of information displayed during optimization.
        // Here we will use the default set of parameters. See the
        // documentation for the full set of parameters.
        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(graph, initialEstimate);
        Values result = optimizer.optimize();
        result.print("Final Result:\n");

        // Calculate and print marginal covariances for all variables
        Marginals marginals = new Marginals(graph, result);
        marginals.marginalCovariance(x1).print("x1 covariance");
        marginals.marginalCovariance(x2).print("x2 covariance");
        marginals.marginalCovariance(x3).print("x3 covariance");
        marginals.marginalCovariance(l1).print("l1 covariance");
        marginals.marginalCovariance(l2).print("l2 covariance");

    }
}