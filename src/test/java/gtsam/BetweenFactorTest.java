package gtsam;

import org.junit.jupiter.api.Test;

import gtsam.noiseModel.Isotropic;

public class BetweenFactorTest {

    // /**
    // * @file testBetweenFactor.cpp
    // * @brief
    // * @author Duy-Nguyen Ta, Varun Agrawal
    // * @date Aug 2, 2013
    // */

    // #include <CppUnitLite/TestHarness.h>
    // #include <gtsam/base/TestableAssertions.h>
    // #include <gtsam/base/numericalDerivative.h>
    // #include <gtsam/geometry/Pose3.h>
    // #include <gtsam/geometry/Rot3.h>
    // #include <gtsam/inference/Symbol.h>
    // #include <gtsam/nonlinear/factorTesting.h>
    // #include <gtsam/slam/BetweenFactor.h>

    // using namespace std::placeholders;
    // using namespace gtsam;
    // using namespace gtsam::symbol_shorthand;
    // using namespace gtsam::noiseModel;

    // /**
    // * This TEST should fail. If you want it to pass, change noise to 0.
    // */
    @Test
    void testRot3() {
        // Rot3 R1 = Rot3::Rodrigues(0.1, 0.2, 0.3);
        // Rot3 R2 = Rot3::Rodrigues(0.4, 0.5, 0.6);
        // Rot3 noise = Rot3(); // Rot3::Rodrigues(0.01, 0.01, 0.01); // Uncomment to
        // make unit test fail
        // Rot3 measured = R1.between(R2)*noise ;

        // BetweenFactor<Rot3> factor(R(1), R(2), measured, Isotropic::Sigma(3, 0.05));
        // Matrix actualH1, actualH2;
        // Vector actual = factor.evaluateError(R1, R2, actualH1, actualH2);

        // Vector expected = Rot3::Logmap(measured.inverse() * R1.between(R2));
        // EXPECT(assert_equal(expected,actual/*, 1e-100*/)); // Uncomment to make unit
        // test fail

        // Matrix numericalH1 = numericalDerivative21<Vector3, Rot3, Rot3>(
        // [&factor](const Rot3& r1, const Rot3& r2) {return factor.evaluateError(r1,
        // r2);},
        // R1, R2, 1e-5);
        // EXPECT(assert_equal(numericalH1,actualH1, 1E-5));

        // Matrix numericalH2 = numericalDerivative22<Vector3,Rot3,Rot3>(
        // [&factor](const Rot3& r1, const Rot3& r2) {return factor.evaluateError(r1,
        // r2);},
        // R1, R2, 1e-5);
        // EXPECT(assert_equal(numericalH2,actualH2, 1E-5));
    }

    // Constructor scalar
    @Test
    void testConstructorScalar() {
        // SharedNoiseModel model;
        double measured = 0.0;
        // BetweenFactor<double> factor(1, 2, measured, model);
    }

    // Constructor vector3
    @Test
    void testConstructorVector3() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        Vector3 measured = new Vector3(1, 2, 3);
        // BetweenFactor<Vector3> factor(1, 2, measured, model);
    }

    // Constructor dynamic sized vector
    @Test
    void testConstructorDynamicSizeVector() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(5, 1.0, true);
        // Vector measured(5); measured << 1, 2, 3, 4, 5;
        // BetweenFactor<Vector> factor(1, 2, measured, model);
    }

    @Test
    void testPoint3Jacobians() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        Point3 measured = new Point3(1, 2, 3);
        // BetweenFactor<Point3> factor(1, 2, measured, model);

        // Values values;
        // values.insert(1, Point3(0, 0, 0));
        // values.insert(2, Point3(1, 2, 3));
        // Vector3 error = factor.evaluateError(Point3(0, 0, 0), Point3(1, 2, 3));
        // EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
        // EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
    }

    @Test
    void testRot3Jacobians() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(3, 1.0, true);
        Rot3 measured = Rot3.Ry(Math.PI / 2);
        // BetweenFactor<Rot3> factor(1, 2, measured, model);

        Values values = new Values();
        // values.insert(1, new Rot3());
        // values.insert(2, Rot3::Ry(M_PI_2));
        // Vector3 error = factor.evaluateError(Rot3(), Rot3::Ry(M_PI_2));
        // EXPECT(assert_equal<Vector3>(Vector3::Zero(), error, 1e-9));
        // EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
    }

    @Test
    void testPose3Jacobians() throws Throwable {
        shared_ptr<Isotropic> model = Isotropic.Sigma(6, 1.0, true);
        Pose3 measured = new Pose3(new Rot3(), new Point3(1, 2, 3));
        // BetweenFactor<Pose3> factor(1, 2, measured, model);

        Pose3 pose1 = new Pose3();
        Pose3 pose2 = new Pose3(new Rot3(), new Point3(1, 2, 3));
        Values values = new Values();
        values.insert(1, pose1);
        values.insert(2, pose2);
        // Vector6 error = factor.evaluateError(pose1, pose2);
        // EXPECT(assert_equal<Vector6>(Vector6::Zero(), error, 1e-9));
        // EXPECT_CORRECT_FACTOR_JACOBIANS(factor, values, 1e-7, 1e-5);
    }

}
