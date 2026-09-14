package gtsam;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;

/** I omitted the ConstantBias example since we don't use it. */
public class PriorFactorTest {

    // Constructor scalar
    @Test
    void testConstructorScalar() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(1, 1.0);
        shared_ptr<PriorFactor<Double>> factor = PriorFactor.PriorFactorDouble(
                new Key(1), 1.0, model);
    }

    // Constructor vector3
    @Test
    void testConstructorVector3() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        shared_ptr<PriorFactor<Vector3>> factor = PriorFactor.PriorFactorVector3(
                new Key(1), new Vector3(1, 2, 3), model);
    }

    // Constructor dynamic sized vector
    @Test
    void testConstructorDynamicSizeVector() throws Throwable {
        Vector v = new Vector(new double[] { 1, 2, 3, 4, 5 });
        shared_ptr<Isotropic> model = Isotropic.Sigma(5, 1.0, true);
        shared_ptr<PriorFactor<Vector>> factor = PriorFactor.PriorFactorVector(
                new Key(1), v, model);
    }
}
