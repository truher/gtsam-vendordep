package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;
import java.util.function.DoubleSupplier;

import org.junit.jupiter.api.Test;

import gtsam.FixedLagSmoother.KeyTimestampMap;
import gtsam.noiseModel.Diagonal;
import gtsam.noiseModel.Isotropic;

/**
 * See gtsam/nonlinear/tests/testBatchFixedLagSmoother.cpp
 */
public class BatchFixedLagSmootherTest {

    boolean check_smoother(
            final NonlinearFactorGraph fullgraph,
            final Values fullinit,
            final BatchFixedLagSmoother smoother,
            final Key key) throws Throwable {

        shared_ptr<GaussianFactorGraph> linearized = fullgraph.linearize(fullinit);
        VectorValues delta = linearized.get().optimize();
        Values fullfinal = fullinit.retract(delta);

        Point2 expected = fullfinal.atPoint2(key);
        Point2 actual = smoother.calculateEstimatePoint2(key);

        return assert_equal(expected, actual);
    }

    @Test
    void testExample() throws Throwable {
        // Test the BatchFixedLagSmoother in a pure linear environment. Thus, full
        // optimization and the BatchFixedLagSmoother should be identical (even with the
        // linearized approximations at the end of the smoothing lag)

        // Set up parameters
        shared_ptr<Diagonal> odometerNoise = Diagonal.Sigmas(new Vector2(0.1, 0.1));
        shared_ptr<Diagonal> loopNoise = Diagonal.Sigmas(new Vector2(0.1, 0.1));

        // Create a Fixed-Lag Smoother
        BatchFixedLagSmoother smoother = new BatchFixedLagSmoother(
                7.0, new LevenbergMarquardtParams());

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
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.addPrior(key0, new Point2(0.0, 0.0), odometerNoise);
            newValues.insert(key0, new Point2(0.01, 0.01));
            newKeyTimestampMap.put(key0, 0.0);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key0));

            ++i;
        }

        // // Add odometry from time 0 to time 5
        while (i <= 5) {
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odometerNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

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
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odometerNoise));
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    new Key(2), new Key(5), new Point2(3.5, 0.0), loopNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // // Add odometry from time 6 to time 15
        while (i <= 15) {
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                    key1, key2, new Point2(1.0, 0.0), odometerNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            ++i;
        }

        // // add/remove an extra factor
        {
            Key key1 = new Key(i - 1);
            Key key2 = new Key(i);

            NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
            Values newValues = new Values();
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            // add 2 odometry factors
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(key1, key2, new Point2(1.0, 0.0),
                    odometerNoise));
            newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(key1, key2, new Point2(1.0, 0.0),
                    odometerNoise));
            newValues.insert(key2, new Point2(i + 0.1, -0.1));
            newKeyTimestampMap.put(key2, i);

            fullgraph.add(newFactors);
            fullinit.insert(newValues);

            // Update the smoother
            smoother.update(newFactors, newValues, newKeyTimestampMap);

            // Check
            assertTrue(check_smoother(fullgraph, fullinit, smoother, key2));

            NonlinearFactorGraph smootherGraph = smoother.getFactors();
            for (int ii = 0; ii < smootherGraph.size(); ii++) {
                if (smootherGraph.at(ii).get() != null) {
                    System.out.printf("ii 2: %d\n", ii);
                    smootherGraph.at(ii).get().print();
                }
            }

            // now remove one of the two and try again
            // empty values and new factors for fake update in which we only remove factors
            NonlinearFactorGraph emptyNewFactors = new NonlinearFactorGraph();
            Values emptyNewValues = new Values();
            KeyTimestampMap emptyNewKeyTimestampMap = new KeyTimestampMap();

            int factorIndex = 6; // any index that does not break connectivity of the graph
            FactorIndices factorToRemove = new FactorIndices();
            factorToRemove.add(new Key(factorIndex));

            final NonlinearFactorGraph smootherFactorsBeforeRemove = smoother.getFactors();

            // remove factor
            smoother.update(
                    emptyNewFactors,
                    emptyNewValues,
                    emptyNewKeyTimestampMap,
                    factorToRemove);

            // check that the factors in the smoother are right
            NonlinearFactorGraph actual = smoother.getFactors();
            for (int ii = 0; ii < smootherFactorsBeforeRemove.size(); ii++) {
                // check that the factors that were not removed are there
                if (smootherFactorsBeforeRemove.at(ii).get() != null && ii != factorIndex) {
                    assertTrue(smootherFactorsBeforeRemove.at(ii).get().equals(actual.at(ii).get()));
                } else {
                    // while the factors that were not there or were removed are no longer there
                    assertTrue(actual.at(i).get() == null);
                }
            }
        }
    }

    @Test
    void testEnforceConsistency() throws Throwable {
        // Verify that enforceConsistency_ actually preserves linearization points
        // for variables involved in marginal factors after marginalization.
        // Before the fix, linearValues_ was never populated, so this feature
        // was silently non-functional.

        shared_ptr<Isotropic> noise = Isotropic.Sigma(2, 0.1, true);

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
            KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

            Key key_i = new Key(i);
            if (i == 0) {
                newFactors.addPrior(key_i, new Point2(0.0, 0.0), noise);
            } else {
                Key key_prev = new Key(i - 1);
                newFactors.add(BetweenFactorPoint2.newBetweenFactorPoint2(
                        key_prev, key_i, new Point2(1.0, 0.0), noise));
            }

            // Use a deliberately poor initial estimate to create nonlinearity
            newValues.insert(key_i, new Point2(i + 0.5, 0.5));
            newKeyTimestampMap.put(key_i, i);

            smootherOn.update(newFactors, newValues, newKeyTimestampMap);
            smootherOff.update(newFactors, newValues, newKeyTimestampMap);
        }

        // After enough steps, marginalization has occurred (lag=3, at step 7 keys
        // 0..3 are marginalized). The smoothers should still produce valid estimates
        // but may differ because consistency enforcement finalrains the optimization.
        // The key test: the enforceConsistency=true smoother should not crash and
        // should produce a reasonable estimate.
        Key lastKey = new Key(7);
        Point2 estimateOn = smootherOn.calculateEstimatePoint2(lastKey);
        Point2 estimateOff = smootherOff.calculateEstimatePoint2(lastKey);

        // Both should be close to the ground truth (7.0, 0.0) -- the chain of
        // unit between-factors from the origin.
        Point2 expected = new Point2(7.0, 0.0);
        assertTrue(assert_equal(expected, estimateOn, 0.5));
        assertTrue(assert_equal(expected, estimateOff, 0.5));
    }

    @Test
    void testNEES() throws Throwable {
        // Monte Carlo NEES evaluation comparing enforceConsistency on vs off.
        // Uses Pose2 (x, y, theta) so the problem is genuinely nonlinear --
        // the rotation makes Jacobians depend on the linearization point,
        // which is exactly where FEJ (First Estimates Jacobian) matters.

        final double transSigma = 0.5;
        final double rotSigma = 0.3; // radians (~17 degrees)
        shared_ptr<Diagonal> noise = Diagonal.Sigmas(new Vector3(rotSigma, transSigma, transSigma));

        final int numTrials = 100;
        final int numSteps = 30;
        final double lag = 3.0; // short lag forces more marginalization
        final int stateDim = 3; // Pose2: (theta, x, y)

        // Ground truth: a curved trajectory with significant turns
        Pose2[] groundTruth = new Pose2[numSteps + 1];
        groundTruth[0] = new Pose2(0, 0, 0);
        final Pose2 odomGT = new Pose2(1.0, 0.0, 0.4); // 1m forward, 0.4 rad turn (~23 deg)
        for (int i = 1; i <= numSteps; ++i) {
            groundTruth[i] = groundTruth[i - 1].compose(odomGT);
        }

        double neesSum_on = 0.0;
        double neesSum_off = 0.0;
        int neesCount = 0;

        Random rng = new Random(42);
        DoubleSupplier transDist = () -> rng.nextGaussian(0.0, transSigma);
        DoubleSupplier rotDist = () -> rng.nextGaussian(0.0, rotSigma);

        for (int trial = 0; trial < numTrials; ++trial) {
            LevenbergMarquardtParams params = new LevenbergMarquardtParams();
            BatchFixedLagSmoother smootherOn = new BatchFixedLagSmoother(lag, params, true);
            BatchFixedLagSmoother smootherOff = new BatchFixedLagSmoother(lag, params, false);

            for (int i = 0; i <= numSteps; ++i) {
                NonlinearFactorGraph newFactors = new NonlinearFactorGraph();
                Values newValues = new Values();
                KeyTimestampMap newKeyTimestampMap = new KeyTimestampMap();

                Key key_i = new Key(i);

                if (i == 0) {
                    newFactors.addPrior(key_i, groundTruth[0], noise);
                    Pose2 initEst = new Pose2(
                            groundTruth[0].x() + transDist.getAsDouble(),
                            groundTruth[0].y() + transDist.getAsDouble(),
                            groundTruth[0].theta() + rotDist.getAsDouble());
                    newValues.insert(key_i, initEst);
                } else {
                    // Noisy odometry measurement
                    Pose2 noisyOdom = new Pose2(
                            odomGT.x() + transDist.getAsDouble(),
                            odomGT.y() + transDist.getAsDouble(),
                            odomGT.theta() + rotDist.getAsDouble());
                    newFactors.add(BetweenFactorPose2.newBetweenFactorPose2(
                            new Key(i - 1), key_i, noisyOdom, noise));

                    // Initial estimate: perturbed ground truth
                    Pose2 initEst = new Pose2(
                            groundTruth[i].x() + transDist.getAsDouble() * 2,
                            groundTruth[i].y() + transDist.getAsDouble() * 2,
                            groundTruth[i].theta() + rotDist.getAsDouble() * 2);
                    newValues.insert(key_i, initEst);
                }
                newKeyTimestampMap.put(key_i, i);

                smootherOn.update(newFactors, newValues, newKeyTimestampMap);
                smootherOff.update(newFactors, newValues, newKeyTimestampMap);
            }

            // Compute NEES at the last key
            Key lastKey = new Key(numSteps);

            // TODO: this try will not work.
            try {
                // enforceConsistency = true
                Values estOn = smootherOn.calculateEstimate();
                Marginals marginalsOn = Marginals.QR(smootherOn.getFactors(), estOn);
                Matrix covOn = marginalsOn.marginalCovariance(lastKey);
                Vector errOn = new Vector(groundTruth[numSteps].local(estOn.atPose2(lastKey)));
                neesSum_on += errOn.transpose().compose(covOn.inverse()).times(errOn).at(0);

                // enforceConsistency = false
                Values estOff = smootherOff.calculateEstimate();
                Marginals marginalsOff = Marginals.QR(smootherOff.getFactors(), estOff);
                Matrix covOff = marginalsOff.marginalCovariance(lastKey);
                Vector errOff = new Vector(groundTruth[numSteps].local(estOff.atPose2(lastKey)));
                neesSum_off += errOff.transpose().compose(covOff.inverse()).times(errOff).at(0);

                neesCount++;
            } catch (Throwable t) {
                continue;
            }
        }

        double avgNees_on = neesSum_on / neesCount;
        double avgNees_off = neesSum_off / neesCount;

        System.out.printf("NEES Evaluation (%d/%d trials, Pose2):\n", neesCount, numTrials);
        System.out.printf(" enforceConsistency=true (FEJ): avg NEES = %f (expected: %d)\n", avgNees_on, stateDim);
        System.out.printf(" enforceConsistency=false : avg NEES = %f (expected: %d)\n", avgNees_off, stateDim);

        assertTrue(neesCount > 0);
        assertTrue(avgNees_on > 0.0);
        assertTrue(avgNees_off > 0.0);
    }

}
