package gtsam;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;

public class PriorFactorTest {

    // Constructor scalar
    @Test
    void testConstructorScalar() {
        // SharedNoiseModel model;
        // PriorFactor<double> factor(1, 1.0, model);
    }

    // Constructor vector3
    @Test
    void testConstructorVector3() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        // PriorFactor<Vector3> factor(1, Vector3(1, 2, 3), model);
    }

    // Constructor dynamic sized vector
    @Test
    void testConstructorDynamicSizeVector() throws Throwable {
        Vector v = new Vector(new double[] { 1, 2, 3, 4, 5 });
        shared_ptr<Isotropic> model = Isotropic.Sigma(5, 1.0, true);
        // PriorFactor<Vector> factor(1, v, model);
    }

    // Vector callEvaluateError(const PriorFactor<ConstantBias>& factor,
    // const ConstantBias& bias) {
    // return factor.evaluateError(bias);
    // }

    // Test for imuBias::ConstantBias
    @Test
    void testConstantBias() throws Throwable {
        Vector3 biasAcc = new Vector3(1, 2, 3);
        Vector3 biasGyro = new Vector3(0.1, 0.2, 0.3);
        // ConstantBias bias(biasAcc, biasGyro);

        // PriorFactor<ConstantBias> factor(1, bias,
        // noiseModel::Isotropic::Sigma(6, 0.1));
        // Values values;
        // values.insert(1, bias);

        // EXPECT_DOUBLES_EQUAL(0.0, factor.error(values), 1e-8);
        // EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);

        // ConstantBias incorrectBias(
        // (Vector6() << 1.1, 2.1, 3.1, 0.2, 0.3, 0.4).finished());
        // values.clear();
        // values.insert(1, incorrectBias);
        // EXPECT_DOUBLES_EQUAL(3.0, factor.error(values), 1e-8);
        // EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
    }

}
