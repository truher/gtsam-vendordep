package gtsam;

import org.junit.jupiter.api.Test;

/**
 * See gtsam/slam/tests/testProjectionFactor.cpp
 */
public class ProjectionFactorTest {

    // // make a realistic calibration matrix
    // static double fov = 60; // degrees
    // static int w=640,h=480;
    // static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

    // // Create a noise model for the pixel error
    // static SharedNoiseModel model(noiseModel::Unit::Create(2));

    @Test
    void testnonStandard() throws Throwable {
        // GenericProjectionFactorCal3DS2 f = new GenericProjectionFactorCal3DS2();
    }

@Test
void testConstructor() throws Throwable {
//   Key poseKey(X(1));
//   Key pointKey(L(1));

//   Point2 measurement(323.0, 240.0);

//   GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K);
}

@Test
void testConstructorWithTransform() throws Throwable {
//   Key poseKey(X(1));
//   Key pointKey(L(1));

//   Point2 measurement(323.0, 240.0);
//   Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

//   GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K, body_P_sensor);
}

@Test
void testEquals () throws Throwable {
//   // Create two identical factors and make sure they're equal
//   Point2 measurement(323.0, 240.0);

//   GenericProjectionFactor<Pose3, Point3> factor1(measurement, model, X(1), L(1), K);
//   GenericProjectionFactor<Pose3, Point3> factor2(measurement, model, X(1), L(1), K);

//   CHECK(assert_equal(factor1, factor2));
}

@Test
void testEqualsWithTransform () throws Throwable {
//   // Create two identical factors and make sure they're equal
//   Point2 measurement(323.0, 240.0);
//   Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));

//   GenericProjectionFactor<Pose3, Point3> factor1(measurement, model, X(1), L(1), K, body_P_sensor);
//   GenericProjectionFactor<Pose3, Point3> factor2(measurement, model, X(1), L(1), K, body_P_sensor);

//   CHECK(assert_equal(factor1, factor2));
}

@Test
void testError () throws Throwable {
  // Create the factor with a measurement that is 3 pixels off in x
//   Key poseKey(X(1));
//   Key pointKey(L(1));
//   Point2 measurement(323.0, 240.0);
//   GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K);

//   // Set the linearization point
//   Pose3 pose(Rot3(), Point3(0,0,-6));
//   Point3 point(0.0, 0.0, 0.0);

//   // Use the factor to calculate the error
//   Vector actualError(factor.evaluateError(pose, point));

//   // The expected error is (-3.0, 0.0) pixels / UnitCovariance
//   Vector expectedError = Vector2(-3.0, 0.0);

//   // Verify we get the expected error
//   CHECK(assert_equal(expectedError, actualError, 1e-9));
}

@Test
void testErrorWithTransform () throws Throwable {
  // Create the factor with a measurement that is 3 pixels off in x
//   Key poseKey(X(1));
//   Key pointKey(L(1));
//   Point2 measurement(323.0, 240.0);
//   Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//   GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K, body_P_sensor);

//   // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
//   Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
//   Point3 point(0.0, 0.0, 0.0);

//   // Use the factor to calculate the error
//   Vector actualError(factor.evaluateError(pose, point));

//   // The expected error is (-3.0, 0.0) pixels / UnitCovariance
//   Vector expectedError = Vector2(-3.0, 0.0);

//   // Verify we get the expected error
//   CHECK(assert_equal(expectedError, actualError, 1e-9));
}

@Test
void testJacobian () throws Throwable {
  // Create the factor with a measurement that is 3 pixels off in x
//   Key poseKey(X(1));
//   Key pointKey(L(1));
//   Point2 measurement(323.0, 240.0);
//   GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K);

//   // Set the linearization point
//   Pose3 pose(Rot3(), Point3(0,0,-6));
//   Point3 point(0.0, 0.0, 0.0);

//   // Use the factor to calculate the Jacobians
//   Matrix H1Actual, H2Actual;
//   factor.evaluateError(pose, point, H1Actual, H2Actual);

//   // The expected Jacobians
//   Matrix H1Expected = (Matrix(2, 6) << 0., -554.256, 0., -92.376, 0., 0., 554.256, 0., 0., 0., -92.376, 0.).finished();
//   Matrix H2Expected = (Matrix(2, 3) << 92.376, 0., 0., 0., 92.376, 0.).finished();

//   // Verify the Jacobians are correct
//   CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
//   CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
}

@Test
void testJacobianWithTransform () throws Throwable {
  // Create the factor with a measurement that is 3 pixels off in x
//   Key poseKey(X(1));
//   Key pointKey(L(1));
//   Point2 measurement(323.0, 240.0);
//   Pose3 body_P_sensor(Rot3::RzRyRx(-M_PI_2, 0.0, -M_PI_2), Point3(0.25, -0.10, 1.0));
//   GenericProjectionFactor<Pose3, Point3> factor(measurement, model, poseKey, pointKey, K, body_P_sensor);

//   // Set the linearization point. The vehicle pose has been selected to put the camera at (-6, 0, 0)
//   Pose3 pose(Rot3(), Point3(-6.25, 0.10 , -1.0));
//   Point3 point(0.0, 0.0, 0.0);

//   // Use the factor to calculate the Jacobians
//   Matrix H1Actual, H2Actual;
//   factor.evaluateError(pose, point, H1Actual, H2Actual);

//   // The expected Jacobians
//   Matrix H1Expected = (Matrix(2, 6) << -92.376, 0., 577.350, 0., 92.376, 0., -9.2376, -577.350, 0., 0., 0., 92.376).finished();
//   Matrix H2Expected = (Matrix(2, 3) << 0., -92.376, 0., 0., 0., -92.376).finished();

//   // Verify the Jacobians are correct
//   CHECK(assert_equal(H1Expected, H1Actual, 1e-3));
//   CHECK(assert_equal(H2Expected, H2Actual, 1e-3));
}

}
