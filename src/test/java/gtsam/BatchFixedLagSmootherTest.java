package gtsam;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Diagonal;
import gtsam.noiseModel.Isotropic;

/**
 * See gtsam/nonlinear/tests/testBatchFixedLagSmoother.cpp
 */
public class BatchFixedLagSmootherTest {

    boolean check_smoother(final NonlinearFactorGraph fullgraph, final Values fullinit,
            final BatchFixedLagSmoother smoother, final Key key) {

        // GaussianFactorGraph linearized = *fullgraph.linearize(fullinit);
        // VectorValues delta = linearized.optimize();
        // Values fullfinal = fullinit.retract(delta);

        // Point2 expected = fullfinal.at<Point2>(key);
        // Point2 actual = smoother.calculateEstimate<Point2>(key);

        // return assert_equal(expected, actual);

        // REMOVE
        return false;
    }

    @Test
    void testExample() throws Throwable {
        // Test the BatchFixedLagSmoother in a pure linear environment. Thus, full
        // optimization and
        // the BatchFixedLagSmoother should be identical (even with the linearized
        // approximations at
        // the end of the smoothing lag)

        // SETDEBUG("BatchFixedLagSmoother update", true);
        // SETDEBUG("BatchFixedLagSmoother reorder", true);
        // SETDEBUG("BatchFixedLagSmoother optimize", true);
        // SETDEBUG("BatchFixedLagSmoother marginalize", true);
        // SETDEBUG("BatchFixedLagSmoother calculateMarginalFactors", true);

        // Set up parameters
        shared_ptr<Diagonal> odometerNoise = Diagonal.Sigmas(new Vector2(0.1, 0.1));
        shared_ptr<Diagonal> loopNoise = Diagonal.Sigmas(new Vector2(0.1, 0.1));

        // Create a Fixed-Lag Smoother
        // typedef BatchFixedLagSmoother::KeyTimestampMap Timestamps;
        // BatchFixedLagSmoother smoother(7.0, LevenbergMarquardtParams());

        // // Create containers to keep the full graph
        Values fullinit = new Values();
        NonlinearFactorGraph fullgraph = new NonlinearFactorGraph();

        // i keeps track of the time step
        int i = 0;

        // Add a prior at time 0 and update the HMF
        {
            Key key0 = new Key(0);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            // newFactors.addPrior(key0, Point2(0.0, 0.0), odometerNoise);
            // newValues.insert(key0, Point2(0.01, 0.01));
            // newTimestamps[key0] = 0.0;

            // fullgraph.push_back(newFactors);
            // fullinit.insert(newValues);

            // // Update the smoother
            // smoother.update(newFactors, newValues, newTimestamps);

            // // Check
            // assertTrue(check_smoother(fullgraph, fullinit, smoother, key0));

            ++i;
        }

        // // Add odometry from time 0 to time 5
        while (i <= 5) {
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            // newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0),
            // odometerNoise));
            // newValues.insert(key2, Point2(double(i)+0.1, -0.1));
            // newTimestamps[key2] = double(i);

            // fullgraph.push_back(newFactors);
            // fullinit.insert(newValues);

            // // Update the smoother
            // smoother.update(newFactors, newValues, newTimestamps);

            // // Check
            // assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // Add odometry from time 5 to 6 to the HMF and a loop closure at time 5 to the
        // TSM
        {
            // Add the odometry factor to the HMF
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            // newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0),
            // odometerNoise));
            // newFactors.push_back(BetweenFactor<Point2>(Key(2), Key(5), Point2(3.5, 0.0),
            // loopNoise));
            // newValues.insert(key2, Point2(double(i)+0.1, -0.1));
            // newTimestamps[key2] = double(i);

            // fullgraph.push_back(newFactors);
            // fullinit.insert(newValues);

            // Update the smoother
            // smoother.update(newFactors, newValues, newTimestamps);

            // Check
            // assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // // Add odometry from time 6 to time 15
        while (i <= 15) {
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            // newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0),
            // odometerNoise));
            // newValues.insert(key2, Point2(double(i)+0.1, -0.1));
            // newTimestamps[key2] = double(i);

            // fullgraph.push_back(newFactors);
            // fullinit.insert(newValues);

            // Update the smoother
            // smoother.update(newFactors, newValues, newTimestamps);

            // Check
            // assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // // add/remove an extra factor
        {
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            // add 2 odometry factors
            // newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0),
            // odometerNoise));
            // newFactors.push_back(BetweenFactor<Point2>(key1, key2, Point2(1.0, 0.0),
            // odometerNoise));
            // newValues.insert(key2, Point2(double(i)+0.1, -0.1));
            // newTimestamps[key2] = double(i);

            // fullgraph.push_back(newFactors);
            // fullinit.insert(newValues);

            // Update the smoother
            // smoother.update(newFactors, newValues, newTimestamps);

            // Check
            // assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            // NonlinearFactorGraph smootherGraph = smoother.getFactors();
            // for(int i=0; i<smootherGraph.size(); i++){
            // if(smootherGraph[i]){
            // std::cout << "i:" << i << std::endl;
            // smootherGraph[i]->print();
            // }
            // }

            // now remove one of the two and try again
            // empty values and new factors for fake update in which we only remove factors
            NonlinearFactorGraph emptyNewFactors = new NonlinearFactorGraph();
            Values emptyNewValues = new Values();
            // Timestamps emptyNewTimestamps;

            int factorIndex = 6; // any index that does not break connectivity of the graph
            // FactorIndices factorToRemove;
            // factorToRemove.push_back(factorIndex);

            // final NonlinearFactorGraph smootherFactorsBeforeRemove =
            // smoother.getFactors();

            // remove factor
            // smoother.update(emptyNewFactors, emptyNewValues,
            // emptyNewTimestamps,factorToRemove);

            // // check that the factors in the smoother are right
            // NonlinearFactorGraph actual = smoother.getFactors();
            // for(int i=0; i< smootherFactorsBeforeRemove.size(); i++){
            // // check that the factors that were not removed are there
            // if(smootherFactorsBeforeRemove[i] && i != factorIndex){
            // assertTrue(smootherFactorsBeforeRemove[i]->equals(*actual[i]));
            // }
            // else{ // while the factors that were not there or were removed are no longer
            // there
            // assertTrue(!actual[i]);
            // }
            // }
        }
    }

    @Test
    void testEnforceConsistency() throws Throwable {
        // Verify that enforceConsistency_ actually preserves linearization points
        // for variables involved in marginal factors after marginalization.
        // Before the fix, linearValues_ was never populated, so this feature
        // was silently non-functional.

        shared_ptr<Isotropic> noise = Isotropic.Sigma(2, 0.1, true);

        // typedef BatchFixedLagSmoother::KeyTimestampMap Timestamps;

        // Create two smoothers: one with consistency enforcement, one without
        LevenbergMarquardtParams params = new LevenbergMarquardtParams();
        // enforceConsistency = true
        BatchFixedLagSmoother smootherOn = new BatchFixedLagSmoother(3.0, params, true);
        // enforceConsistency = false
        BatchFixedLagSmoother smootherOff = new BatchFixedLagSmoother(3.0, params, false);

        // Feed both smoothers the same data: a chain of between factors with
        // deliberately poor initial values to make relinearization matter.
        for (int i = 0; i <= 7; ++i) {
            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            // Key key_i(i);
            if (i == 0) {
                // newFactors.addPrior(key_i, Point2(0.0, 0.0), noise);
            } else {
                // Key key_prev(i - 1);
                // newFactors.push_back(BetweenFactor<Point2>(key_prev, key_i, Point2(1.0, 0.0),
                // noise));
            }

            // Use a deliberately poor initial estimate to create nonlinearity
            // newValues.insert(key_i, Point2(double(i) + 0.5, 0.5));
            // newTimestamps[key_i] = double(i);

            // smootherOn.update(newFactors, newValues, newTimestamps);
            // smootherOff.update(newFactors, newValues, newTimestamps);
        }

        // After enough steps, marginalization has occurred (lag=3, at step 7 keys
        // 0..3 are marginalized). The smoothers should still produce valid estimates
        // but may differ because consistency enforcement finalrains the optimization.
        // The key test: the enforceConsistency=true smoother should not crash and
        // should produce a reasonable estimate.
        Key lastKey = new Key(7);
        // Point2 estimateOn = smootherOn.calculateEstimate<Point2>(lastKey);
        // Point2 estimateOff = smootherOff.calculateEstimate<Point2>(lastKey);

        // // Both should be close to the ground truth (7.0, 0.0) -- the chain of
        // // unit between-factors from the origin.
        Point2 expected = new Point2(7.0, 0.0);
        // assertTrue(assert_equal(expected, estimateOn, 0.5));
        // assertTrue(assert_equal(expected, estimateOff, 0.5));
    }

    @Test
    void testNEES() throws Throwable {
        // Monte Carlo NEES evaluation comparing enforceConsistency on vs off.
        // Uses Pose2 (x, y, theta) so the problem is genuinely nonlinear --
        // the rotation makes Jacobians depend on the linearization point,
        // which is exactly where FEJ (First Estimates Jacobian) matters.

        final double transSigma = 0.5;
        final double rotSigma = 0.3; // radians (~17 degrees)
        // auto noise = noiseModel::Diagonal::Sigmas(
        // (Vector(3) << rotSigma, transSigma, transSigma).finished());

        final int numTrials = 100;
        final int numSteps = 30;
        final double lag = 3.0; // short lag forces more marginalization
        final int stateDim = 3; // Pose2: (theta, x, y)

        // // Ground truth: a curved trajectory with significant turns
        List<Pose2> groundTruth = new ArrayList<>(numSteps + 1);
        // groundTruth[0] = Pose2(0, 0, 0);
        // final Pose2 odomGT(1.0, 0.0, 0.4); // 1m forward, 0.4 rad turn (~23 deg)
        for (int i = 1; i <= numSteps; ++i) {
            // groundTruth[i] = groundTruth[i-1] * odomGT;
        }

        double neesSum_on = 0.0;
        double neesSum_off = 0.0;
        int neesCount = 0;

        // mt19937 rng(42);
        // normal_distribution<double> transDist(0.0, transSigma);
        // normal_distribution<double> rotDist(0.0, rotSigma);

        // for (int trial = 0; trial < numTrials; ++trial) {
        // typedef BatchFixedLagSmoother::KeyTimestampMap Timestamps;
        // LevenbergMarquardtParams params;
        // BatchFixedLagSmoother smootherOn(lag, params, true);
        // BatchFixedLagSmoother smootherOff(lag, params, false);

        for (int i = 0; i <= numSteps; ++i) {
            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            // Timestamps newTimestamps;

            Key key_i = new Key(i);

            if (i == 0) {
                // newFactors.addPrior(key_i, groundTruth[0], noise);
                // Pose2 initEst(groundTruth[0].x() + transDist(rng),
                // groundTruth[0].y() + transDist(rng),
                // groundTruth[0].theta() + rotDist(rng));
                // newValues.insert(key_i, initEst);
            } else {
                // // Noisy odometry measurement
                // Pose2 noisyOdom(odomGT.x() + transDist(rng),
                // odomGT.y() + transDist(rng),
                // odomGT.theta() + rotDist(rng));
                // newFactors.push_back(BetweenFactor<Pose2>(Key(i-1), key_i, noisyOdom,
                // noise));

                // // Initial estimate: perturbed ground truth
                // Pose2 initEst(groundTruth[i].x() + transDist(rng) * 2,
                // groundTruth[i].y() + transDist(rng) * 2,
                // groundTruth[i].theta() + rotDist(rng) * 2);
                // newValues.insert(key_i, initEst);
            }
            // newTimestamps[key_i] = double(i);

            // smootherOn.update(newFactors, newValues, newTimestamps);
            // smootherOff.update(newFactors, newValues, newTimestamps);
        }

        // // Compute NEES at the last key
        Key lastKey = new Key(numSteps);

        // try {
        // // enforceConsistency = true
        // Values estOn = smootherOn.calculateEstimate();
        // Marginals marginalsOn(smootherOn.getFactors(), estOn, Marginals::QR);
        // Matrix covOn = marginalsOn.marginalCovariance(lastKey);
        // Vector errOn =
        // groundTruth[numSteps].localCoordinates(estOn.at<Pose2>(lastKey));
        // neesSum_on += errOn.transpose() * covOn.inverse() * errOn;

        // // enforceConsistency = false
        // Values estOff = smootherOff.calculateEstimate();
        // Marginals marginalsOff(smootherOff.getFactors(), estOff, Marginals::QR);
        // Matrix covOff = marginalsOff.marginalCovariance(lastKey);
        // Vector errOff =
        // groundTruth[numSteps].localCoordinates(estOff.at<Pose2>(lastKey));
        // neesSum_off += errOff.transpose() * covOff.inverse() * errOff;

        // neesCount++;
        // } catch (...) {
        // continue;
        // }
        // }

        // double avgNees_on = neesSum_on / neesCount;
        // double avgNees_off = neesSum_off / neesCount;

        // cout << "NEES Evaluation (" << neesCount << "/" << numTrials << " trials,
        // Pose2):" << endl;
        // cout << " enforceConsistency=true (FEJ): avg NEES = " << avgNees_on
        // << " (expected: " << stateDim << ")" << endl;
        // cout << " enforceConsistency=false : avg NEES = " << avgNees_off
        // << " (expected: " << stateDim << ")" << endl;

        // assertTrue(neesCount > 0);
        // assertTrue(avgNees_on > 0.0);
        // assertTrue(avgNees_off > 0.0);
    }

}
