package gtsam;

import static gtsam.Testable.assert_equal;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * See gtsam/geometry/tests/testRot3.cpp
 */
public class Rot3Test {

    static Rot3 R;
    static Point3 P;
    static double error;
    static double epsilon;

    static {
        try {
            R = Rot3.Rodrigues(0.1, 0.4, 0.2);
            P = new Point3(0.2, 0.7, -2.0);
            error = 1e-9;
            epsilon = 0.001;
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
    }

    @Test
    void testchart() throws Throwable {
        Matrix3 R = new Matrix3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        new Rot3(R);
    }

    @Test
    void testconstructor() throws Throwable {
        Rot3 expected = new Rot3(Matrix3.identity());
        Point3 r1 = new Point3(1, 0, 0);
        Point3 r2 = new Point3(0, 1, 0);
        Point3 r3 = new Point3(0, 0, 1);
        Rot3 actual = new Rot3(r1, r2, r3);
        assertTrue(assert_equal(actual, expected));
    }

    @Test
    void testconstructor2() throws Throwable {
        Matrix3 R = new Matrix3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        Rot3 actual = new Rot3(R);
        Rot3 expected = new Rot3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        assertTrue(assert_equal(actual, expected));
    }

    @Test
    void testconstructor3() throws Throwable {
        Rot3 expected = new Rot3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        Point3 r1 = new Point3(0, 1, 0);
        Point3 r2 = new Point3(1, 0, 0);
        Point3 r3 = new Point3(0, 0, -1);
        assertTrue(assert_equal(expected, new Rot3(r1, r2, r3)));
    }

    @Test
    void testtranspose() throws Throwable {
        Point3 r1 = new Point3(0, 1, 0);
        Point3 r2 = new Point3(1, 0, 0);
        Point3 r3 = new Point3(0, 0, -1);
        Rot3 R = new Rot3(//
                0, 1, 0, //
                1, 0, 0, //
                0, 0, -1);
        assertTrue(assert_equal(R.inverse(), new Rot3(r1, r2, r3)));
    }

    @Test
    void testequals() throws Throwable {
        assertTrue(R.equals(R));
        Rot3 zero = new Rot3();
        assertTrue(!R.equals(zero));
    }

    // // Notice this uses J^2 whereas fast uses w*w', and has cos(t)*I + ....
    // Rot3 slow_but_correct_Rodrigues(const Vector& w) {
    // double t = w.norm();
    // Matrix3 J = skewSymmetric(w / t);
    // if (t < 1e-5) return Rot3();
    // Matrix3 R = I_3x3 + sin(t) * J + (1.0 - cos(t)) * (J * J);
    // return Rot3(R);
    // }

    @Test
    void testAxisAngle() throws Throwable {

        Point3 axis = new Point3(0., 1., 0.); // rotation around Y
        double angle = 3.14 / 4.0;
        Rot3 expected = new Rot3(//
                0.707388, 0, 0.706825, //
                0, 1, 0, //
                -0.706825, 0, 0.707388);
        Rot3 actual = Rot3.AxisAngle(axis, angle);
        assertTrue(assert_equal(expected, actual, 1e-5));
        Rot3 actual2 = Rot3.AxisAngle(axis, angle - 2 * Math.PI);
        assertTrue(assert_equal(expected, actual2, 1e-5));

        axis = new Point3(0, 50, 0);
        Rot3 actual3 = Rot3.AxisAngle(axis, angle);
        assertTrue(assert_equal(expected, actual3, 1e-5));
    }

    @Test
    void testAxisAngle2() throws Throwable {
        // constructor from a rotation matrix, as doubles in *row-major* order.
        Rot3 R1 = new Rot3(//
                -0.999957, 0.00922903, 0.00203116, //
                0.00926964, 0.999739, 0.0208927, //
                -0.0018374, 0.0209105, -0.999781);

        // convert Rot3 to quaternion using GTSAM
        // TODO: this is a "pair" return type
        // const auto [actualAxis, actualAngle] = R1.axisAngle();

        double expectedAngle = 3.1396582;
        // assertTrue(assert_equal(expectedAngle, actualAngle, 1e-5));
    }

    @Test
    void testRodrigues() throws Throwable {
        Rot3 R1 = Rot3.Rodrigues(epsilon, 0, 0);
        // Vector w = (Vector(3) << epsilon, 0., 0.).finished();
        // Rot3 R2 = slow_but_correct_Rodrigues(w);
        // assertTrue(assert_equal(R2,R1));
    }

    @Test
    void testRodrigues2() throws Throwable {
        Point3 axis = new Point3(0., 1., 0.); // rotation around Y
        double angle = 3.14 / 4.0;
        Rot3 expected = new Rot3(//
                0.707388, 0, 0.706825, //
                0, 1, 0, //
                -0.706825, 0, 0.707388);
        Rot3 actual = Rot3.AxisAngle(axis, angle);
        assertTrue(assert_equal(expected, actual, 1e-5));
        // Rot3 actual2 = Rot3::Rodrigues(angle*axis);
        // assertTrue(assert_equal(expected,actual2,1e-5));
    }

    @Test
    void testRodrigues3() throws Throwable {
        Vector3 w = new Vector3(0.1, 0.2, 0.3);
        // Rot3 R1 = Rot3::AxisAngle(w / w.norm(), w.norm());
        // Rot3 R2 = slow_but_correct_Rodrigues(w);
        // assertTrue(assert_equal(R2,R1));
    }

    @Test
    void testRodrigues4() throws Throwable {
        // Vector axis = Vector3(0., 0., 1.); // rotation around Z
        // double angle = M_PI/2.0;
        // Rot3 actual = Rot3::AxisAngle(axis, angle);
        // double c=cos(angle),s=sin(angle);
        // Rot3 expected(c,-s, 0,
        // s, c, 0,
        // 0, 0, 1);
        // assertTrue(assert_equal(expected,actual));
        // assertTrue(assert_equal(slow_but_correct_Rodrigues(axis*angle),actual));
    }

    @Test
    void testretract() throws Throwable {
        Vector3 v = new Vector3();
        assertTrue(assert_equal(R, R.retract(v)));
    }

    // namespace {

    // struct RetractNormalizationMetrics {
    // double quaternionNormError;
    // double orthogonalityError;
    // double determinantError;
    // };

    // RetractNormalizationMetrics measureRetractNormalization(
    // const Rot3& base, const Vector3& omega) {
    // const Rot3 retracted = base.retract(omega);
    // const Matrix3 matrix = retracted.matrix();
    // return {std::abs(retracted.toQuaternion().norm() - 1.0),
    // (matrix.transpose() * matrix - I_3x3).norm(),
    // std::abs(matrix.determinant() - 1.0)};
    // }

    // } // namespace

    @Test
    void testretractNormalizationAcrossMagnitudes() throws Throwable {
        // const Vector3 direction = Vector3(1.0, -2.0, 3.0).normalized();
        // const std::array<Rot3, 2> bases = {
        // Rot3(),
        // Rot3::RzRyRx(0.3, -0.2, 0.5),
        // };
        // const std::array<double, 17> magnitudes = {
        // 0.0, 1e-16, 1e-14, 1e-12, 1e-10, 1e-8, 1e-6, 1e-4, 1e-2,
        // 1.0, 1e2, 1e4, 1e6, 1e8, 1e10, 1e12, 1e14,
        // };
        // const double quaternionNormTolerance = 1e-12;
        // const double orthogonalityTolerance = 1e-10;
        // const double determinantTolerance = 1e-10;

        // for (size_t baseIndex = 0; baseIndex < bases.size(); ++baseIndex) {
        // for (const double magnitude : magnitudes) {
        // const auto metrics =
        // measureRetractNormalization(bases[baseIndex], direction * magnitude);
        // const bool quaternionOk =
        // metrics.quaternionNormError <= quaternionNormTolerance;
        // const bool orthogonalityOk =
        // metrics.orthogonalityError <= orthogonalityTolerance;
        // const bool determinantOk =
        // metrics.determinantError <= determinantTolerance;
        // if (quaternionOk && orthogonalityOk && determinantOk) continue;

        // std::ostringstream os;
        // os << std::scientific << std::setprecision(3)
        // << "Retract normalization broke down for base[" << baseIndex
        // << "] at |omega|=" << magnitude
        // << " with quaternion norm error=" << metrics.quaternionNormError
        // << ", orthogonality error=" << metrics.orthogonalityError
        // << ", determinant error=" << metrics.determinantError;
        // FAIL(os.str());
        // }
        // }
    }

    @Test
    void testlog() throws Throwable {
        // static const double PI = std::acos(-1.0);
        // Vector w;
        // Rot3 R;

        // #define CHECK_OMEGA(X, Y, Z) \
        // w = (Vector(3) << (X), (Y), (Z)).finished(); \
        // R = Rot3::Rodrigues(w); \
        // assertTrue(assert_equal(w, Rot3::Logmap(R), 1e-12));

        // // Check zero
        // CHECK_OMEGA(0, 0, 0)

        // // create a random direction:
        // double norm = sqrt(1.0 + 16.0 + 4.0);
        // double x = 1.0 / norm, y = 4.0 / norm, z = 2.0 / norm;

        // // Check very small rotation for Taylor expansion
        // // Note that tolerance above is 1e-12, so Taylor is pretty good !
        // double d = 0.0001;
        // CHECK_OMEGA(d, 0, 0)
        // CHECK_OMEGA(0, d, 0)
        // CHECK_OMEGA(0, 0, d)
        // CHECK_OMEGA(x * d, y * d, z * d)

        // // check normal rotation
        // d = 0.1;
        // CHECK_OMEGA(d, 0, 0)
        // CHECK_OMEGA(0, d, 0)
        // CHECK_OMEGA(0, 0, d)
        // CHECK_OMEGA(x * d, y * d, z * d)

        // // Check 180 degree rotations
        // CHECK_OMEGA(PI, 0, 0)
        // CHECK_OMEGA(0, PI, 0)
        // CHECK_OMEGA(0, 0, PI)

        // // Windows and Linux have flipped sign in quaternion mode
        // //#if !defined(__APPLE__) && defined(GTSAM_USE_QUATERNIONS)
        // w = (Vector(3) << x * PI, y * PI, z * PI).finished();
        // R = Rot3::Rodrigues(w);
        // assertTrue(assert_equal(Vector(-w), Rot3::Logmap(R), 1e-12));
        // //#else
        // // CHECK_OMEGA(x * PI, y * PI, z * PI)
        // //#endif

        // // Check 360 degree rotations
        // #define CHECK_OMEGA_ZERO(X, Y, Z) \
        // w = (Vector(3) << (X), (Y), (Z)).finished(); \
        // R = Rot3::Rodrigues(w); \
        // assertTrue(assert_equal((Vector)Z_3x1, Rot3::Logmap(R)));

        // CHECK_OMEGA_ZERO(2.0 * PI, 0, 0)
        // CHECK_OMEGA_ZERO(0, 2.0 * PI, 0)
        // CHECK_OMEGA_ZERO(0, 0, 2.0 * PI)
        // CHECK_OMEGA_ZERO(x * 2. * PI, y * 2. * PI, z * 2. * PI)

        // // Check problematic case from Lund dataset vercingetorix.g2o
        // // This is an almost rotation with determinant not *quite* 1.
        // Rot3 Rlund(-0.98582676, -0.03958746, -0.16303092, //
        // -0.03997006, -0.88835923, 0.45740671, //
        // -0.16293753, 0.45743998, 0.87418537);

        // // Rot3's Logmap returns different, but equivalent compacted
        // // axis-angle vectors depending on whether Rot3 is implemented
        // // by Quaternions or SO3.
        // #if defined(GTSAM_USE_QUATERNIONS)
        // // Quaternion bounds angle to [-pi, pi] resulting in ~179.9 degrees
        // assertTrue(assert_equal(Vector3(0.264451979, -0.742197651, -3.04098211),
        // (Vector)Rot3::Logmap(Rlund), 1e-8));
        // #else
        // // SO3 will be approximate because of the non-orthogonality
        // assertTrue(assert_equal(Vector3(0.264452, -0.742197708, -3.04098184),
        // (Vector)Rot3::Logmap(Rlund), 1e-8));
        // #endif
    }

    @Test
    void testretract_localCoordinates() throws Throwable {
        Vector3 d12 = new Vector3(0.1, 0.1, 0.1);
        Rot3 R2 = R.retract(d12);
        assertTrue(assert_equal(d12, R.localCoordinates(R2)));
    }

    @Test
    void testexpmap_logmap() throws Throwable {
        Vector3 d12 = new Vector3(0.1, 0.1, 0.1);
        Rot3 R2 = R.expmap(d12);
        assertTrue(assert_equal(d12, R.logmap(R2)));
    }

    @Test
    void testretract_localCoordinates2() throws Throwable {
        Rot3 t1 = R;
        Rot3 t2 = R.compose(R);
        Vector3 d12 = t1.localCoordinates(t2);
        assertTrue(assert_equal(t2, t1.retract(d12)));
        Vector3 d21 = t2.localCoordinates(t1);
        assertTrue(assert_equal(t1, t2.retract(d21)));
        assertTrue(assert_equal(d12, d21.times(-1.0)));
    }

    @Test
    void testmanifold_expmap() throws Throwable {
        Rot3 gR1 = Rot3.Rodrigues(0.1, 0.4, 0.2);
        Rot3 gR2 = Rot3.Rodrigues(0.3, 0.1, 0.7);
        Rot3 origin = new Rot3();

        // log behaves correctly
        Vector3 d12 = Rot3.traits.Logmap(gR1.between(gR2));
        Vector3 d21 = Rot3.traits.Logmap(gR2.between(gR1));

        // Check expmap
        assertTrue(assert_equal(gR2, gR1.compose(Rot3.traits.Expmap(d12))));
        assertTrue(assert_equal(gR1, gR2.compose(Rot3.traits.Expmap(d21))));

        // Check that log(t1,t2)=-log(t2,t1)
        assertTrue(assert_equal(d12, d21.times(-1)));

        // lines in canonical coordinates correspond to Abelian subgroups in SO(3)
        Vector3 d = new Vector3(0.1, 0.2, 0.3);
        // exp(-d)=inverse(exp(d))
        assertTrue(assert_equal(Rot3.traits.Expmap(d.times(-1.0)),
                Rot3.traits.Expmap(d).inverse()));
        // exp(5d)=exp(2*d+3*d)=exp(2*d)exp(3*d)=exp(3*d)exp(2*d)
        Rot3 R2 = Rot3.traits.Expmap(d.times(2));
        Rot3 R3 = Rot3.traits.Expmap(d.times(3));
        Rot3 R5 = Rot3.traits.Expmap(d.times(5));
        assertTrue(assert_equal(R5, R2.compose(R3)));
        assertTrue(assert_equal(R5, R3.compose(R2)));
    }

    // class AngularVelocity : public Vector3 {
    // public:
    // template <typename Derived>
    // inline AngularVelocity(const Eigen::MatrixBase<Derived>& v)
    // : Vector3(v) {}

    // AngularVelocity(double wx, double wy, double wz) : Vector3(wx, wy, wz) {}
    // };

    // AngularVelocity bracket(const AngularVelocity& X, const AngularVelocity& Y) {
    // return X.cross(Y);
    // }

    @Test
    void testBCH() throws Throwable {
        // Approximate exmap by BCH formula
        // AngularVelocity w1(0.2, -0.1, 0.1);
        // AngularVelocity w2(0.01, 0.02, -0.03);
        // Rot3 R1 = Rot3::Expmap (w1), R2 = Rot3::Expmap (w2);
        // Rot3 R3 = R1 * R2;
        // Vector expected = Rot3::Logmap(R3);
        // Vector actual = BCH(w1, w2);
        // assertTrue(assert_equal(expected, actual,1e-5));
    }

    @Test
    void testrotate_derivatives() throws Throwable {
        // Matrix actualDrotate1a, actualDrotate1b, actualDrotate2;
        // R.rotate(P, actualDrotate1a, actualDrotate2);
        // R.inverse().rotate(P, actualDrotate1b, {});
        // Matrix numerical1 = numericalDerivative21(testing::rotate<Rot3,Point3>, R,
        // P);
        // Matrix numerical2 = numericalDerivative21(testing::rotate<Rot3,Point3>,
        // R.inverse(), P);
        // Matrix numerical3 = numericalDerivative22(testing::rotate<Rot3,Point3>, R,
        // P);
        // assertTrue(assert_equal(numerical1,actualDrotate1a,error));
        // assertTrue(assert_equal(numerical2,actualDrotate1b,error));
        // assertTrue(assert_equal(numerical3,actualDrotate2, error));
    }

    @Test
    void testunrotate() throws Throwable {
        // Point3 w = R * P;
        Matrix H1 = new Matrix();
        Matrix H2 = new Matrix();
        // Point3 actual = R.unrotate(w,H1,H2);
        // assertTrue(assert_equal(P,actual));

        // Matrix numerical1 = numericalDerivative21(testing::unrotate<Rot3,Point3>, R,
        // w);
        // assertTrue(assert_equal(numerical1,H1,error));

        // Matrix numerical2 = numericalDerivative22(testing::unrotate<Rot3,Point3>, R,
        // w);
        // assertTrue(assert_equal(numerical2,H2,error));
    }

    @Test
    void testcompose() throws Throwable {
        Rot3 R1 = Rot3.Rodrigues(0.1, 0.2, 0.3);
        Rot3 R2 = Rot3.Rodrigues(0.2, 0.3, 0.5);

        Rot3 expected = R1.compose(R2);
        Matrix actualH1 = new Matrix();
        Matrix actualH2 = new Matrix();
        // Rot3 actual = R1.compose(R2, actualH1, actualH2);
        // assertTrue(assert_equal(expected,actual));

        // Matrix numericalH1 = numericalDerivative21(testing::compose<Rot3>, R1,
        // R2, 1e-2);
        // assertTrue(assert_equal(numericalH1,actualH1));

        // Matrix numericalH2 = numericalDerivative22(testing::compose<Rot3>, R1,
        // R2, 1e-2);
        // assertTrue(assert_equal(numericalH2,actualH2));
    }

    @Test
    void testinverse() throws Throwable {
        Rot3 R = Rot3.Rodrigues(0.1, 0.2, 0.3);
        // Rot3 I;
        // Matrix3 actualH;
        // Rot3 actual = R.inverse(actualH);
        // assertTrue(assert_equal(I,R*actual));
        // assertTrue(assert_equal(I,actual*R));
        // assertTrue(assert_equal(actual.matrix(), R.transpose()));

        // Matrix numericalH = numericalDerivative11(testing::inverse<Rot3>, R);
        // assertTrue(assert_equal(numericalH,actualH));
    }

    @Test
    void testbetween() throws Throwable {
        // Rot3 r1 = Rot3::Rz(M_PI/3.0);
        // Rot3 r2 = Rot3::Rz(2.0*M_PI/3.0);

        // Matrix expectedr1 = (Matrix(3, 3) <<
        // 0.5, -sqrt(3.0)/2.0, 0.0,
        // sqrt(3.0)/2.0, 0.5, 0.0,
        // 0.0, 0.0, 1.0).finished();
        // assertTrue(assert_equal(expectedr1, r1.matrix()));

        // Rot3 R = Rot3::Rodrigues(0.1, 0.4, 0.2);
        // Rot3 origin;
        // assertTrue(assert_equal(R, origin.between(R)));
        // assertTrue(assert_equal(R.inverse(), R.between(origin)));

        // Rot3 R1 = Rot3::Rodrigues(0.1, 0.2, 0.3);
        // Rot3 R2 = Rot3::Rodrigues(0.2, 0.3, 0.5);

        // Rot3 expected = R1.inverse() * R2;
        // Matrix actualH1, actualH2;
        // Rot3 actual = R1.between(R2, actualH1, actualH2);
        // assertTrue(assert_equal(expected,actual));

        // Matrix numericalH1 = numericalDerivative21(testing::between<Rot3> , R1, R2);
        // assertTrue(assert_equal(numericalH1,actualH1));

        // Matrix numericalH2 = numericalDerivative22(testing::between<Rot3> , R1, R2);
        // assertTrue(assert_equal(numericalH2,actualH2));
    }

    @Test
    void testxyz() throws Throwable {
        double t = 0.1;
        double st = Math.sin(t);
        double ct = Math.cos(t);

        // Make sure all counterclockwise
        // Diagrams below are all from from unchanging axis

        // // z
        // // | * Y=(ct,st)
        // // x----y
        // Rot3 expected1(1, 0, 0, 0, ct, -st, 0, st, ct);
        // assertTrue(assert_equal(expected1,Rot3::Rx(t)));

        // // x
        // // | * Z=(ct,st)
        // // y----z
        // Rot3 expected2(ct, 0, st, 0, 1, 0, -st, 0, ct);
        // assertTrue(assert_equal(expected2,Rot3::Ry(t)));

        // // y
        // // | X=* (ct,st)
        // // z----x
        // Rot3 expected3(ct, -st, 0, st, ct, 0, 0, 0, 1);
        // assertTrue(assert_equal(expected3,Rot3::Rz(t)));

        // // Check compound rotation
        // Rot3 expected = Rot3::Rz(0.3) * Rot3::Ry(0.2) * Rot3::Rx(0.1);
        // assertTrue(assert_equal(expected,Rot3::RzRyRx(0.1,0.2,0.3)));
    }

    @Test
    void testyaw_pitch_roll() throws Throwable {
        double t = 0.1;

        // // yaw is around z axis
        // assertTrue(assert_equal(Rot3::Rz(t),Rot3::Yaw(t)));

        // // pitch is around y axis
        // assertTrue(assert_equal(Rot3::Ry(t),Rot3::Pitch(t)));

        // // roll is around x axis
        // assertTrue(assert_equal(Rot3::Rx(t),Rot3::Roll(t)));

        // // Check compound rotation
        // Rot3 expected = Rot3::Yaw(0.1) * Rot3::Pitch(0.2) * Rot3::Roll(0.3);
        // assertTrue(assert_equal(expected,Rot3::Ypr(0.1,0.2,0.3)));

        // assertTrue(assert_equal((Vector)Vector3(0.1, 0.2, 0.3),expected.ypr()));
    }

    @Test
    void testRQ() throws Throwable {
        // // Try RQ on a pure rotation
        // const auto [actualK, actual] = RQ(R.matrix());
        // Vector expected = Vector3(0.14715, 0.385821, 0.231671);
        // assertTrue(assert_equal(I_3x3, (Matrix)actualK));
        // assertTrue(assert_equal(expected,actual,1e-6));

        // // Try using xyz call, asserting that Rot3::RzRyRx(x,y,z).xyz()==[x;y;z]
        // assertTrue(assert_equal(expected,R.xyz(),1e-6));
        // assertTrue(assert_equal((Vector)Vector3(0.1,0.2,0.3),Rot3::RzRyRx(0.1,0.2,0.3).xyz()));

        // // Try using ypr call, asserting that Rot3::Ypr(y,p,r).ypr()==[y;p;r]
        // assertTrue(assert_equal((Vector)Vector3(0.1,0.2,0.3),Rot3::Ypr(0.1,0.2,0.3).ypr()));
        // assertTrue(assert_equal((Vector)Vector3(0.3,0.2,0.1),Rot3::Ypr(0.1,0.2,0.3).rpy()));

        // // Try ypr for pure yaw-pitch-roll matrices
        // assertTrue(assert_equal((Vector)Vector3(0.1,0.0,0.0),Rot3::Yaw (0.1).ypr()));
        // assertTrue(assert_equal((Vector)Vector3(0.0,0.1,0.0),Rot3::Pitch(0.1).ypr()));
        // assertTrue(assert_equal((Vector)Vector3(0.0,0.0,0.1),Rot3::Roll
        // (0.1).ypr()));

        // // Try RQ to recover calibration from 3*3 sub-block of projection matrix
        // Matrix K = (Matrix(3, 3) << 500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0,
        // 1.0).finished();
        // Matrix A = K * R.matrix();
        // const auto [actualK2, actual2] = RQ(A);
        // assertTrue(assert_equal(K, actualK2));
        // assertTrue(assert_equal(expected, actual2, 1e-6));
    }

    @Test
    void testexpmapStability() throws Throwable {
        Vector3 w = new Vector3(78e-9, 5e-8, 97e-7);
        // double theta = w.norm();
        // double theta2 = theta*theta;
        // Rot3 actualR = Rot3::Expmap(w);
        // Matrix W = (Matrix(3, 3) << 0.0, -w(2), w(1),
        // w(2), 0.0, -w(0),
        // -w(1), w(0), 0.0 ).finished();
        // Matrix W2 = W*W;
        // Matrix Rmat = I_3x3 + (1.0-theta2/6.0 + theta2*theta2/120.0
        // - theta2*theta2*theta2/5040.0)*W + (0.5 - theta2/24.0 +
        // theta2*theta2/720.0)*W2 ;
        // Rot3 expectedR( Rmat );
        // assertTrue(assert_equal(expectedR, actualR, 1e-10));
    }

    @Test
    void testlogmapStability() throws Throwable {
        Vector3 w = new Vector3(1e-8, 0.0, 0.0);
        // Rot3 R = Rot3::Expmap(w);
        // // double tr = R.r1().x()+R.r2().y()+R.r3().z();
        // // std::cout.precision(5000);
        // // std::cout << "theta: " << w.norm() << std::endl;
        // // std::cout << "trace: " << tr << std::endl;
        // // R.print("R = ");
        // Vector actualw = Rot3::Logmap(R);
        // assertTrue(assert_equal(w, actualw, 1e-15));
    }

    @Test
    void testquaternion() throws Throwable {
        // // NOTE: This is also verifying the ability to convert Vector to Quaternion
        // Quaternion q1(0.710997408193224, 0.360544029310185, 0.594459869568306,
        // 0.105395217842782);
        // Rot3 R1(0.271018623057411, 0.278786459830371, 0.921318086098018,
        // 0.578529366719085, 0.717799701969298, -0.387385285854279,
        // -0.769319620053772, 0.637998195662053, 0.033250932803219);

        // Quaternion q2(0.263360579192421, 0.571813128030932, 0.494678363680335,
        // 0.599136268678053);
        // Rot3 R2(-0.207341903877828, 0.250149415542075, 0.945745528564780,
        // 0.881304914479026, -0.371869043667957, 0.291573424846290,
        // 0.424630407073532, 0.893945571198514, -0.143353873763946);

        // // Check creating Rot3 from quaternion
        // assertTrue(assert_equal(R1, Rot3(q1)));
        // assertTrue(assert_equal(R1, Rot3::Quaternion(q1.w(), q1.x(), q1.y(),
        // q1.z())));
        // assertTrue(assert_equal(R2, Rot3(q2)));
        // assertTrue(assert_equal(R2, Rot3::Quaternion(q2.w(), q2.x(), q2.y(),
        // q2.z())));

        // // Check converting Rot3 to quaterion
        // assertTrue(assert_equal(Vector(R1.toQuaternion().coeffs()),
        // Vector(q1.coeffs())));
        // assertTrue(assert_equal(Vector(R2.toQuaternion().coeffs()),
        // Vector(q2.coeffs())));

        // // Check that quaternion and Rot3 represent the same rotation
        // Point3 p1(1.0, 2.0, 3.0);
        // Point3 p2(8.0, 7.0, 9.0);

        // Point3 expected1 = R1*p1;
        // Point3 expected2 = R2*p2;

        // Point3 actual1 = Point3(q1*p1);
        // Point3 actual2 = Point3(q2*p2);

        // assertTrue(assert_equal(expected1, actual1));
        // assertTrue(assert_equal(expected2, actual2));
    }

    @Test
    void testConvertQuaternion() throws Throwable {
        // Eigen::Quaterniond eigenQuaternion;
        // eigenQuaternion.w() = 1.0;
        // eigenQuaternion.x() = 2.0;
        // eigenQuaternion.y() = 3.0;
        // eigenQuaternion.z() = 4.0;
        // EXPECT_DOUBLES_EQUAL(1, eigenQuaternion.w(), 1e-9);
        // EXPECT_DOUBLES_EQUAL(2, eigenQuaternion.x(), 1e-9);
        // EXPECT_DOUBLES_EQUAL(3, eigenQuaternion.y(), 1e-9);
        // EXPECT_DOUBLES_EQUAL(4, eigenQuaternion.z(), 1e-9);

        // Rot3 R(eigenQuaternion);
        // EXPECT_DOUBLES_EQUAL(1, R.toQuaternion().w(), 1e-9);
        // EXPECT_DOUBLES_EQUAL(2, R.toQuaternion().x(), 1e-9);
        // EXPECT_DOUBLES_EQUAL(3, R.toQuaternion().y(), 1e-9);
        // EXPECT_DOUBLES_EQUAL(4, R.toQuaternion().z(), 1e-9);
    }

    // Matrix Cayley(const Matrix& A) {
    // Matrix::Index n = A.cols();
    // const Matrix I = Matrix::Identity(n,n);
    // return (I-A)*(I+A).inverse();
    // }

    @Test
    void testCayley() throws Throwable {
        // Matrix A = skewSymmetric(1,2,-3);
        // Matrix Q = Cayley(A);
        // assertTrue(assert_equal((Matrix)I_3x3, trans(Q)*Q));
        // assertTrue(assert_equal(A, Cayley(Q)));
    }

    @Test
    void testslerp() throws Throwable {
        // // A first simple test
        // Rot3 R1 = Rot3::Rz(1), R2 = Rot3::Rz(2), R3 = Rot3::Rz(1.5);
        // assertTrue(assert_equal(R1, R1.slerp(0.0,R2)));
        // assertTrue(assert_equal(R2, R1.slerp(1.0,R2)));
        // assertTrue(assert_equal(R3, R1.slerp(0.5,R2)));
        // // Make sure other can be *this
        // assertTrue(assert_equal(R1, R1.slerp(0.5,R1)));
    }

    static Rot3 id;
    static Rot3 T1;
    static Rot3 T2;

    static {
        try {
            id = new Rot3();
            T1 = Rot3.AxisAngle(new Point3(0, 0, 1), 1);
            T2 = Rot3.AxisAngle(new Point3(0, 1, 0), 2);
        } catch (Throwable t) {
            throw new RuntimeException(t);
        }
    }

    @Test
    void testInvariants() throws Throwable {
        assertTrue(Rot3.check_group_invariants(id, id));
        assertTrue(Rot3.check_group_invariants(id, T1));
        assertTrue(Rot3.check_group_invariants(T2, id));
        assertTrue(Rot3.check_group_invariants(T2, T1));
        assertTrue(Rot3.check_group_invariants(T1, T2));

        assertTrue(Rot3.check_manifold_invariants(id, id));
        assertTrue(Rot3.check_manifold_invariants(id, T1));
        assertTrue(Rot3.check_manifold_invariants(T2, id));
        assertTrue(Rot3.check_manifold_invariants(T2, T1));
        assertTrue(Rot3.check_manifold_invariants(T1, T2));
    }

    @Test
    void testClosestTo() throws Throwable {
        // Matrix3 M;
        // M << 0.79067393, 0.6051136, -0.0930814, //
        // 0.4155925, -0.64214347, -0.64324489, //
        // -0.44948549, 0.47046326, -0.75917576;

        // Matrix expected(3, 3);
        // expected << 0.790687, 0.605096, -0.0931312, //
        // 0.415746, -0.642355, -0.643844, //
        // -0.449411, 0.47036, -0.759468;

        // auto actual = Rot3::ClosestTo(3*M);
        // assertTrue(assert_equal(expected, actual.matrix(), 1e-6));
    }

    @Test
    void testaxisAngle() throws Throwable {
        // Unit3 actualAxis;
        // double actualAngle;

        // // not a lambda as otherwise we can't trace error easily
        // #define CHECK_AXIS_ANGLE(expectedAxis, expectedAngle, rotation) \
        // std::tie(actualAxis, actualAngle) = rotation.axisAngle(); \
        // assertTrue(assert_equal(expectedAxis, actualAxis, 1e-9)); \
        // EXPECT_DOUBLES_EQUAL(expectedAngle, actualAngle, 1e-9); \
        // assertTrue(assert_equal(rotation, Rot3::AxisAngle(expectedAxis,
        // expectedAngle)))

        // // CHECK R defined at top = Rot3::Rodrigues(0.1, 0.4, 0.2)
        // Vector3 omega(0.1, 0.4, 0.2);
        // Unit3 axis(omega), _axis(-omega);
        // CHECK_AXIS_ANGLE(axis, omega.norm(), R);

        // // rotate by 90
        // CHECK_AXIS_ANGLE(Unit3(1, 0, 0), M_PI_2, Rot3::Ypr(0, 0, M_PI_2))
        // CHECK_AXIS_ANGLE(Unit3(0, 1, 0), M_PI_2, Rot3::Ypr(0, M_PI_2, 0))
        // CHECK_AXIS_ANGLE(Unit3(0, 0, 1), M_PI_2, Rot3::Ypr(M_PI_2, 0, 0))
        // CHECK_AXIS_ANGLE(axis, M_PI_2, Rot3::AxisAngle(axis, M_PI_2))

        // // rotate by -90
        // CHECK_AXIS_ANGLE(Unit3(-1, 0, 0), M_PI_2, Rot3::Ypr(0, 0, -M_PI_2))
        // CHECK_AXIS_ANGLE(Unit3(0, -1, 0), M_PI_2, Rot3::Ypr(0, -M_PI_2, 0))
        // CHECK_AXIS_ANGLE(Unit3(0, 0, -1), M_PI_2, Rot3::Ypr(-M_PI_2, 0, 0))
        // CHECK_AXIS_ANGLE(_axis, M_PI_2, Rot3::AxisAngle(axis, -M_PI_2))

        // // rotate by 270
        // const double theta270 = M_PI + M_PI / 2;
        // CHECK_AXIS_ANGLE(Unit3(-1, 0, 0), M_PI_2, Rot3::Ypr(0, 0, theta270))
        // CHECK_AXIS_ANGLE(Unit3(0, -1, 0), M_PI_2, Rot3::Ypr(0, theta270, 0))
        // CHECK_AXIS_ANGLE(Unit3(0, 0, -1), M_PI_2, Rot3::Ypr(theta270, 0, 0))
        // CHECK_AXIS_ANGLE(_axis, M_PI_2, Rot3::AxisAngle(axis, theta270))

        // // rotate by -270
        // const double theta_270 = -(M_PI + M_PI / 2); // 90 (or -270) degrees
        // CHECK_AXIS_ANGLE(Unit3(1, 0, 0), M_PI_2, Rot3::Ypr(0, 0, theta_270))
        // CHECK_AXIS_ANGLE(Unit3(0, 1, 0), M_PI_2, Rot3::Ypr(0, theta_270, 0))
        // CHECK_AXIS_ANGLE(Unit3(0, 0, 1), M_PI_2, Rot3::Ypr(theta_270, 0, 0))
        // CHECK_AXIS_ANGLE(axis, M_PI_2, Rot3::AxisAngle(axis, theta_270))

        // const double theta195 = 195 * M_PI / 180;
        // const double theta165 = 165 * M_PI / 180;

        // /// Non-trivial angle 165
        // CHECK_AXIS_ANGLE(Unit3(1, 0, 0), theta165, Rot3::Ypr(0, 0, theta165))
        // CHECK_AXIS_ANGLE(Unit3(0, 1, 0), theta165, Rot3::Ypr(0, theta165, 0))
        // CHECK_AXIS_ANGLE(Unit3(0, 0, 1), theta165, Rot3::Ypr(theta165, 0, 0))
        // CHECK_AXIS_ANGLE(axis, theta165, Rot3::AxisAngle(axis, theta165))

        // /// Non-trivial angle 195
        // CHECK_AXIS_ANGLE(Unit3(-1, 0, 0), theta165, Rot3::Ypr(0, 0, theta195))
        // CHECK_AXIS_ANGLE(Unit3(0, -1, 0), theta165, Rot3::Ypr(0, theta195, 0))
        // CHECK_AXIS_ANGLE(Unit3(0, 0, -1), theta165, Rot3::Ypr(theta195, 0, 0))
        // CHECK_AXIS_ANGLE(_axis, theta165, Rot3::AxisAngle(axis, theta195))
    }

    // Rot3 RzRyRx_proxy(double const& a, double const& b, double const& c) {
    // return Rot3::RzRyRx(a, b, c);
    // }

    @Test
    void testRzRyRx_scalars_derivative() throws Throwable {
        // const auto x = 0.1, y = 0.4, z = 0.2;
        // const auto num_x = numericalDerivative31(RzRyRx_proxy, x, y, z);
        // const auto num_y = numericalDerivative32(RzRyRx_proxy, x, y, z);
        // const auto num_z = numericalDerivative33(RzRyRx_proxy, x, y, z);

        // Vector3 act_x, act_y, act_z;
        // Rot3::RzRyRx(x, y, z, act_x, act_y, act_z);

        // assertTrue(assert_equal(num_x, act_x));
        // assertTrue(assert_equal(num_y, act_y));
        // assertTrue(assert_equal(num_z, act_z));
    }

    // Rot3 RzRyRx_proxy(Vector3 const& xyz) { return Rot3::RzRyRx(xyz); }

    @Test
    void testRzRyRx_vector_derivative() throws Throwable {
        // const auto xyz = Vector3{-0.3, 0.1, 0.7};
        // const auto num = numericalDerivative11(RzRyRx_proxy, xyz);

        // Matrix3 act;
        // Rot3::RzRyRx(xyz, act);

        // assertTrue(assert_equal(num, act));
    }

    // Rot3 Ypr_proxy(double const& y, double const& p, double const& r) {
    // return Rot3::Ypr(y, p, r);
    // }

    @Test
    void testYpr_derivative() throws Throwable {
        // const auto y = 0.7, p = -0.3, r = 0.1;
        // const auto num_y = numericalDerivative31(Ypr_proxy, y, p, r);
        // const auto num_p = numericalDerivative32(Ypr_proxy, y, p, r);
        // const auto num_r = numericalDerivative33(Ypr_proxy, y, p, r);

        // Vector3 act_y, act_p, act_r;
        // Rot3::Ypr(y, p, r, act_y, act_p, act_r);

        // assertTrue(assert_equal(num_y, act_y));
        // assertTrue(assert_equal(num_p, act_p));
        // assertTrue(assert_equal(num_r, act_r));
    }

    // Vector3 RQ_proxy(Matrix3 const& R) {
    // const auto RQ_ypr = RQ(R);
    // return RQ_ypr.second;
    // }

    @Test
    void testRQ_derivative() throws Throwable {
        // using VecAndErr = std::pair<Vector3, double>;
        // std::vector<VecAndErr> test_xyz;
        // // Test zeros and a couple of random values
        // test_xyz.push_back(VecAndErr{{0, 0, 0}, error});
        // test_xyz.push_back(VecAndErr{{0, 0.5, -0.5}, error});
        // test_xyz.push_back(VecAndErr{{0.3, 0, 0.2}, error});
        // test_xyz.push_back(VecAndErr{{-0.6, 1.3, 0}, 1e-8});
        // test_xyz.push_back(VecAndErr{{1.0, 0.7, 0.8}, error});
        // test_xyz.push_back(VecAndErr{{3.0, 0.7, -0.6}, error});
        // test_xyz.push_back(VecAndErr{{M_PI / 2, 0, 0}, error});
        // test_xyz.push_back(VecAndErr{{0, 0, M_PI / 2}, error});

        // // Test close to singularity
        // test_xyz.push_back(VecAndErr{{0, M_PI / 2 - 1e-1, 0}, 1e-7});
        // test_xyz.push_back(VecAndErr{{0, 3 * M_PI / 2 + 1e-1, 0}, 1e-7});
        // test_xyz.push_back(VecAndErr{{0, M_PI / 2 - 1.1e-2, 0}, 1e-4});
        // test_xyz.push_back(VecAndErr{{0, 3 * M_PI / 2 + 1.1e-2, 0}, 1e-4});

        // for (auto const& vec_err : test_xyz) {
        // auto const& xyz = vec_err.first;

        // const auto R = Rot3::RzRyRx(xyz).matrix();
        // const auto num = numericalDerivative11(RQ_proxy, R);
        // Matrix39 calc;
        // RQ(R, calc);

        // const auto err = vec_err.second;
        // assertTrue(assert_equal(num, calc, err));
        // }
    }

    // Vector3 xyz_proxy(Rot3 const& R) { return R.xyz(); }

    @Test
    void testxyz_derivative() throws Throwable {
        // const auto aa = Vector3{-0.6, 0.3, 0.2};
        // const auto R = Rot3::Expmap(aa);
        // const auto num = numericalDerivative11(xyz_proxy, R);
        // Matrix3 calc;
        // R.xyz(calc);

        // assertTrue(assert_equal(num, calc));
    }

    // Vector3 ypr_proxy(Rot3 const& R) { return R.ypr(); }

    @Test
    void testypr_derivative() throws Throwable {
        // const auto aa = Vector3{0.1, -0.3, -0.2};
        // const auto R = Rot3::Expmap(aa);
        // const auto num = numericalDerivative11(ypr_proxy, R);
        // Matrix3 calc;
        // R.ypr(calc);

        // assertTrue(assert_equal(num, calc));
    }

    // Vector3 rpy_proxy(Rot3 const& R) { return R.rpy(); }

    @Test
    void testrpy_derivative() throws Throwable {
        // const auto aa = Vector3{1.2, 0.3, -0.9};
        // const auto R = Rot3::Expmap(aa);
        // const auto num = numericalDerivative11(rpy_proxy, R);
        // Matrix3 calc;
        // R.rpy(calc);

        // assertTrue(assert_equal(num, calc));
    }

    // double roll_proxy(Rot3 const& R) { return R.roll(); }

    @Test
    void testroll_derivative() throws Throwable {
        Vector3 aa = new Vector3(0.8, -0.8, 0.8);
        Rot3 R = Rot3.traits.Expmap(aa);
        // const auto num = numericalDerivative11(roll_proxy, R);
        // Matrix13 calc;
        // R.roll(calc);

        // assertTrue(assert_equal(num, calc));
    }

    // double pitch_proxy(Rot3 const& R) { return R.pitch(); }

    @Test
    void testpitch_derivative() throws Throwable {
        // const auto aa = Vector3{0.01, 0.1, 0.0};
        // const auto R = Rot3::Expmap(aa);
        // const auto num = numericalDerivative11(pitch_proxy, R);
        // Matrix13 calc;
        // R.pitch(calc);

        // assertTrue(assert_equal(num, calc));
    }

    // double yaw_proxy(Rot3 const& R) { return R.yaw(); }

    @Test
    void testyaw_derivative() throws Throwable {
        // const auto aa = Vector3{0.0, 0.1, 0.6};
        // const auto R = Rot3::Expmap(aa);
        // const auto num = numericalDerivative11(yaw_proxy, R);
        // Matrix13 calc;
        // R.yaw(calc);

        // assertTrue(assert_equal(num, calc));
    }

    @Test
    void testdeterminant() throws Throwable {
        // size_t degree = 1;
        // Rot3 R_w0; // Zero rotation
        // Rot3 R_w1 = Rot3::Ry(degree * M_PI / 180);

        // Rot3 R_01, R_w2;
        // double actual, expected = 1.0;

        // for (size_t i = 2; i < 360; ++i) {
        // R_01 = R_w0.between(R_w1);
        // R_w2 = R_w1 * R_01;
        // R_w0 = R_w1;
        // R_w1 = R_w2.normalized();
        // actual = R_w2.matrix().determinant();

        // EXPECT_DOUBLES_EQUAL(expected, actual, 1e-7);
        // }
    }

    @Test
    void testExpmapChainRule() throws Throwable {
        // // Multiply with an arbitrary matrix and exponentiate
        // Matrix3 M;
        // M << 1, 2, 3, 4, 5, 6, 7, 8, 9;
        // auto g = [&](const Vector3& omega) {
        // return Rot3::Expmap(M*omega);
    };

    // // Test the derivatives at zero
    // const Matrix3 expected = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
    // assertTrue(assert_equal<Matrix3>(expected, M, 1e-5)); //
    // SO3::ExpmapDerivative(Z_3x1) is identity

    // // Test the derivatives at another value
    // const Vector3 delta{0.1,0.2,0.3};
    // const Matrix3 expected2 = numericalDerivative11<Rot3, Vector3>(g, delta);
    // assertTrue(assert_equal<Matrix3>(expected2, SO3::ExpmapDerivative(M*delta) *
    // M,
    // 1e-5));
    // }

    @Test
    void testexpmapChainRule() throws Throwable {
        // // Multiply an arbitrary rotation with exp(M*x)
        // // Perhaps counter-intuitively, this has the same derivatives as above
        // Matrix3 M;
        // M << 1, 2, 3, 4, 5, 6, 7, 8, 9;
        // const Rot3 R = Rot3::Expmap({1, 2, 3});
        // auto g = [&](const Vector3& omega) {
        // return R.expmap(M*omega);
        // };

        // // Test the derivatives at zero
        // const Matrix3 expected = numericalDerivative11<Rot3, Vector3>(g, Z_3x1);
        // assertTrue(assert_equal<Matrix3>(expected, M, 1e-5));

        // // Test the derivatives at another value
        // const Vector3 delta{0.1,0.2,0.3};
        // const Matrix3 expected2 = numericalDerivative11<Rot3, Vector3>(g, delta);
        // assertTrue(assert_equal<Matrix3>(expected2, SO3::ExpmapDerivative(M*delta) *
        // M,
        // 1e-5));
    }

}
