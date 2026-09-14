package examples;

import gtsam.BearingRangeFactorPose2Point2;
import gtsam.BetweenFactorPose2;
import gtsam.Key;
import gtsam.LevenbergMarquardtOptimizer;
import gtsam.Marginals;
import gtsam.Matrix;
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
 * Like PlanarSLAMExample, but computes a "cloud" of samples around each
 * estimate, for visualization.
 */
public class PlanarSLAMExampleWithSamples {

    public static void main(String args[]) throws Throwable {

        NonlinearFactorGraph graph = new NonlinearFactorGraph();

        Key x1 = Key.X(1);
        Key x2 = Key.X(2);
        Key x3 = Key.X(3);
        Key l1 = Key.L(1);
        Key l2 = Key.L(2);

        Pose2 prior = new Pose2(0.0, 0.0, 0.0);
        shared_ptr<Diagonal> priorNoise = Diagonal.Sigmas(new Vector3(0.3, 0.3, 0.1));
        graph.addPrior(x1, prior, priorNoise);

        Pose2 odometry = new Pose2(2.0, 0.0, 0.0);
        shared_ptr<Diagonal> odometryNoise = Diagonal.Sigmas(new Vector3(0.2, 0.2, 0.1));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(x1, x2, odometry, odometryNoise));
        graph.add(BetweenFactorPose2.newBetweenFactorPose2(x2, x3, odometry, odometryNoise));

        shared_ptr<Diagonal> measurementNoise = Diagonal.Sigmas(new Vector2(0.1, 0.2));
        Rot2 bearing11 = Rot2.fromDegrees(45);
        Rot2 bearing21 = Rot2.fromDegrees(90);
        Rot2 bearing32 = Rot2.fromDegrees(90);
        double range11 = Math.sqrt(4.0 + 4.0), range21 = 2.0, range32 = 2.0;

        graph.add(BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                x1, l1, bearing11, range11, measurementNoise));
        graph.add(BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                x2, l1, bearing21, range21, measurementNoise));
        graph.add(BearingRangeFactorPose2Point2.newBearingRangeFactorPose2Point2(
                x3, l2, bearing32, range32, measurementNoise));

        graph.print("Factor Graph:\n");

        Values initialEstimate = new Values();
        initialEstimate.insert(x1, new Pose2(0.5, 0.0, 0.2));
        initialEstimate.insert(x2, new Pose2(2.3, 0.1, -0.2));
        initialEstimate.insert(x3, new Pose2(4.1, 0.1, 0.1));
        initialEstimate.insert(l1, new Point2(1.8, 2.1));
        initialEstimate.insert(l2, new Point2(4.1, 1.8));

        initialEstimate.print("Initial Estimate:\n");

        LevenbergMarquardtOptimizer optimizer = new LevenbergMarquardtOptimizer(graph, initialEstimate);
        Values result = optimizer.optimize();
        result.print("Final Result:\n");

        Marginals marginals = new Marginals(graph, result);
        Matrix x1cov = marginals.marginalCovariance(x1);
        Matrix x2cov = marginals.marginalCovariance(x2);
        Matrix x3cov = marginals.marginalCovariance(x3);
        Matrix l1cov = marginals.marginalCovariance(l1);
        Matrix l2cov = marginals.marginalCovariance(l2);

        x1cov.print("x1 covariance");
        x2cov.print("x2 covariance");
        x3cov.print("x3 covariance");
        l1cov.print("l1 covariance");
        l2cov.print("l2 covariance");

        // each sample is the result (the mean) plus a sample using the covariance.

        int N = 50;

        Pose2 p1 = result.atPose2(x1);
        Pose2 p2 = result.atPose2(x2);
        Pose2 p3 = result.atPose2(x3);
        Point2 pl1 = result.atPoint2(l1);
        Point2 pl2 = result.atPoint2(l2);

        for (int i = 0; i < N; ++i) {
            Pose2 sample = p1.expmap(new Vector3(x1cov.draw()));
            System.out.printf("x1 %9.6f %9.6f %9.6f\n", sample.x(), sample.y(), sample.theta());
        }
        for (int i = 0; i < N; ++i) {
            Pose2 sample = p2.expmap(new Vector3(x2cov.draw()));
            System.out.printf("x2 %9.6f %9.6f %9.6f\n", sample.x(), sample.y(), sample.theta());
        }
        for (int i = 0; i < N; ++i) {
            Pose2 sample = p3.expmap(new Vector3(x3cov.draw()));
            System.out.printf("x3 %9.6f %9.6f %9.6f\n", sample.x(), sample.y(), sample.theta());
        }
        for (int i = 0; i < N; ++i) {
            Point2 sample = pl1.expmap(new Vector2(l1cov.draw()));
            System.out.printf("l1 %9.6f %9.6f\n", sample.x(), sample.y());
        }
        for (int i = 0; i < N; ++i) {
            Point2 sample = pl2.expmap(new Vector2(l2cov.draw()));
            System.out.printf("l2 %9.6f %9.6f\n", sample.x(), sample.y());
        }

    }
}