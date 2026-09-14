package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;
import gtsam.noiseModel.Robust;
import gtsam.noiseModel.Unit;
import gtsam.noiseModel.mEstimator.Cauchy;
import gtsam.noiseModel.mEstimator.DCS;
import gtsam.noiseModel.mEstimator.Fair;
import gtsam.noiseModel.mEstimator.GemanMcClure;
import gtsam.noiseModel.mEstimator.Huber;
import gtsam.noiseModel.mEstimator.L2WithDeadZone;
import gtsam.noiseModel.mEstimator.Tukey;
import gtsam.noiseModel.mEstimator.Welsch;

/**
 * See tests/testNonlinearFactor.cpp
 * 
 * I skipped all the tests that required examples from C++.
 */
public class NonlinearFactorTest {

    @Test
    void testWeight() throws Throwable {
        // create a values structure for the non linear factor graph
        Values values = new Values();

        // Instantiate a concrete class version of a NoiseModelFactor
        shared_ptr<PriorFactor<Point2>> factor1 = PriorFactor.PriorFactorPoint2(
                Key.X(1), new Point2(0, 0), Unit.Create(2));
        values.insert(Key.X(1), new Point2(0.1, 0.1));

        assertTrue(assert_equal(1.0, factor1.get().weight(values), 1e-6));

        // Factor with noise model
        shared_ptr<Isotropic> noise = Isotropic.Sigma(2, 0.2);
        shared_ptr<PriorFactor<Point2>> factor2 = PriorFactor.PriorFactorPoint2(
                Key.X(2), new Point2(1, 1), noise);
        values.insert(Key.X(2), new Point2(1.1, 1.1));

        assertTrue(assert_equal(1.0, factor2.get().weight(values), 1e-6));

        Point2 estimate = new Point2(3, 3);
        Point2 prior = new Point2(1, 1);
        double distance = estimate.minus(prior).norm();

        shared_ptr<Isotropic> gaussian = Isotropic.Sigma(2, 0.2, true);

        shared_ptr<PriorFactor<Point2>> factor;

        // vector to store all the robust models in so we can test iteratively.
        List<shared_ptr<? extends Robust>> robust_models = new ArrayList<>();

        // Fair noise model
        shared_ptr<Robust> fair = Robust.Create(
                Fair.Create(1.3998), gaussian);
        robust_models.add(fair);

        // Huber noise model
        shared_ptr<Robust> huber = Robust.Create(
                Huber.Create(1.345), gaussian);
        robust_models.add(huber);

        // Cauchy noise model
        shared_ptr<Robust> cauchy = Robust.Create(
                Cauchy.Create(0.1), gaussian);
        robust_models.add(cauchy);

        // Tukey noise model
        shared_ptr<Robust> tukey = Robust.Create(
                Tukey.Create(4.6851), gaussian);
        robust_models.add(tukey);

        // Welsch noise model
        shared_ptr<Robust> welsch = Robust.Create(
                Welsch.Create(2.9846), gaussian);
        robust_models.add(welsch);

        // Geman-McClure noise model
        shared_ptr<Robust> gm = Robust.Create(
                GemanMcClure.Create(1.0), gaussian);
        robust_models.add(gm);

        // DCS noise model
        shared_ptr<Robust> dcs = Robust.Create(
                DCS.Create(1.0), gaussian);
        robust_models.add(dcs);

        // L2WithDeadZone noise model
        shared_ptr<Robust> l2 = Robust.Create(
                L2WithDeadZone.Create(1.0), gaussian);
        robust_models.add(l2);

        for (shared_ptr<? extends Robust> model : robust_models) {
            factor = PriorFactor.PriorFactorPoint2(Key.X(3), prior, model);
            values.clear();
            values.insert(Key.X(3), estimate);
            assertTrue(assert_equal(
                    model.get().robust().get().weight(distance),
                    factor.get().weight(values),
                    1e-6));
        }
    }

}
