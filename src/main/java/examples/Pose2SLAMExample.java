package examples;

import gtsam.BetweenFactorPose2;
import gtsam.GaussNewtonOptimizer;
import gtsam.GaussNewtonParams;
import gtsam.Key;
import gtsam.Marginals;
import gtsam.NonlinearFactorGraph;
import gtsam.Pose2;
import gtsam.Values;
import gtsam.Vector3;
import gtsam.shared_ptr;
import gtsam.noiseModel.Diagonal;

/**
 * A simple 2D pose slam example
 * - The robot moves in a 2 meter square
 * - The robot moves 2 meters each step, turning 90 degrees after each step
 * - The robot initially faces along the X axis (horizontal, to the right in 2D)
 * - We have full odometry between pose
 * - We have a loop closure constraint when the robot returns to the first
 * position
 */

public class Pose2SLAMExample {

    public static void main(String args[]) throws Throwable {

        // 1. Create a factor graph container and add factors to it.
        NonlinearFactorGraph graph = new NonlinearFactorGraph();

        // 2a. Add a prior on the first pose, setting it to the origin.
        //
        // A prior factor consists of a mean and a noise model (covariance matrix)
        shared_ptr<Diagonal> priorNoise = Diagonal.Sigmas(new Vector3(0.3, 0.3, 0.1));
        graph.addPrior(new Key(1), new Pose2(0, 0, 0), priorNoise);

        // For simplicity, we will use the same noise model for odometry and loop
        // closures
        shared_ptr<Diagonal> model = Diagonal.Sigmas(new Vector3(0.2, 0.2, 0.1));

        // 2b. Add odometry factors.
        //
        // Create odometry (Between) factors between consecutive poses
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(
                new Key(1), new Key(2), new Pose2(2, 0, 0), model));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(
                new Key(2), new Key(3), new Pose2(2, 0, Math.PI / 2), model));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(
                new Key(3), new Key(4), new Pose2(2, 0, Math.PI / 2), model));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(
                new Key(4), new Key(5), new Pose2(2, 0, Math.PI / 2), model));

        // 2c. Add the loop closure constraint.
        //
        // This factor encodes the fact that we have returned to the same pose. In real
        // systems, these constraints may be identified in many ways, such as
        // appearance-based techniques with camera images. We will use another Between
        // Factor to enforce this constraint:
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(
                new Key(5), new Key(2), new Pose2(2, 0, Math.PI / 2), model));

        // Print.
        graph.print("\nFactor Graph:\n");

        // 3. Create the data structure to hold the initialEstimate estimate to the
        // solution.
        //
        // For illustrative purposes, these have been deliberately set to incorrect
        // values.
        Values initialEstimate = new Values();
        initialEstimate.insert(new Key(1), new Pose2(0.5, 0.0, 0.2));
        initialEstimate.insert(new Key(2), new Pose2(2.3, 0.1, -0.2));
        initialEstimate.insert(new Key(3), new Pose2(4.1, 0.1, Math.PI / 2));
        initialEstimate.insert(new Key(4), new Pose2(4.0, 2.0, Math.PI));
        initialEstimate.insert(new Key(5), new Pose2(2.1, 2.1, -Math.PI / 2));
        initialEstimate.print("\nInitial Estimate:\n"); // print

        // 4. Optimize the initial values using a Gauss-Newton nonlinear optimizer.
        //
        // The optimizer accepts an optional set of configuration parameters,
        // controlling things like convergence criteria, the type of linear
        // system solver to use, and the amount of information displayed during
        // optimization. We will set a few parameters as a demonstration.
        GaussNewtonParams parameters = new GaussNewtonParams();

        // Stop iterating once the change in error between steps is less than this value
        parameters.relativeErrorTol(1e-5);

        // Do not perform more than N iteration steps
        parameters.maxIterations(100);

        // Create the optimizer ...
        GaussNewtonOptimizer optimizer = new GaussNewtonOptimizer(graph, initialEstimate, parameters);

        // ... and optimize
        Values result = optimizer.optimize();
        result.print("Final Result:\n");

        // 5. Calculate and print marginal covariances for all variables
        Marginals marginals = new Marginals(graph, result);
        marginals.marginalCovariance(new Key(1)).print("x1 covariance:");
        marginals.marginalCovariance(new Key(2)).print("x2 covariance:");
        marginals.marginalCovariance(new Key(3)).print("x3 covariance:");
        marginals.marginalCovariance(new Key(4)).print("x4 covariance:");
        marginals.marginalCovariance(new Key(5)).print("x5 covariance:");
    }
}
