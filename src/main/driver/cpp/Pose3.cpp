#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

extern "C" {
/**
 * Pose3 constructor uses the implicit copy constructors for each member
 * (rotation, translation), so the arguments here can be freed.
 */
gtsam::Pose3* Pose3(const gtsam::Rot3* r, const gtsam::Point3* t) {
    return new gtsam::Pose3(*r, *t);
}
void Pose3_delete(gtsam::Pose3* p) {
    delete p;
}
gtsam::Pose3* Pose3_Pose2(const gtsam::Pose2* p) {
    return new gtsam::Pose3(*p);
}
gtsam::Pose3* Pose3_compose(const gtsam::Pose3* p,  //
                            const gtsam::Pose3* p2) {
    return new gtsam::Pose3(p->compose(*p2));
}
gtsam::Pose3* Pose3_composeH(const gtsam::Pose3* p,   //
                             const gtsam::Pose3* p2,  //
                             gtsam::Matrix* H1,       //
                             gtsam::Matrix* H2) {     //
    return new gtsam::Pose3(p->compose(*p2, *H1, *H2));
}
gtsam::Pose3* Pose3_between(const gtsam::Pose3* p,  //
                            const gtsam::Pose3* p2) {
    return new gtsam::Pose3(p->between(*p2));
}
gtsam::Pose3* Pose3_betweenH(const gtsam::Pose3* p,  //
                             const gtsam::Pose3* p2,
                             gtsam::Matrix* H1,    //
                             gtsam::Matrix* H2) {  //
    return new gtsam::Pose3(p->between(*p2, *H1, *H2));
}
gtsam::Pose3* Pose3_retract(const gtsam::Pose3* p,  //
                            const gtsam::Vector6* v) {
    return new gtsam::Pose3(p->retract(*v));
}
gtsam::Pose3* Pose3_retractH(const gtsam::Pose3* p,    //
                             const gtsam::Vector6* v,  //
                             gtsam::Matrix* H1,        //
                             gtsam::Matrix* H2) {      //
    return new gtsam::Pose3(p->retract(*v, *H1, *H2));
}
gtsam::Vector6* Pose3_localCoordinates(const gtsam::Pose3* a,
                                       const gtsam::Pose3* b) {
    return new gtsam::Vector6(a->localCoordinates(*b));
}
gtsam::Vector6* Pose3_localCoordinatesH(const gtsam::Pose3* a,  //
                                        const gtsam::Pose3* b,  //
                                        gtsam::Matrix* H1,      //
                                        gtsam::Matrix* H2) {    //
    return new gtsam::Vector6(a->localCoordinates(*b, *H1, *H2));
}
gtsam::Pose3* Pose3_inverse(const gtsam::Pose3* p) {
    return new gtsam::Pose3(p->inverse());
}
gtsam::Pose3* Pose3_inverseH(const gtsam::Pose3* p, gtsam::Matrix* H) {
    return new gtsam::Pose3(p->inverse(*H));
}
/** underlying AdjointMap returns Matrix3 but we coerce to dynamic. */
gtsam::Matrix* Pose3_AdjointMap(const gtsam::Pose3* p) {
    return new gtsam::Matrix(p->AdjointMap());
}
gtsam::Pose3* Pose3_expmapH(const gtsam::Pose3* p,    //
                            const gtsam::Vector6* v,  //
                            gtsam::Matrix* H1,        //
                            gtsam::Matrix* H2) {
    return new gtsam::Pose3(p->expmap(*v, *H1, *H2));
}
gtsam::Vector6* Pose3_logmapH(const gtsam::Pose3* p,  //
                              const gtsam::Pose3* g,  //
                              gtsam::Matrix* H1,      //
                              gtsam::Matrix* H2) {
    return new gtsam::Vector6(p->logmap(*g, *H1, *H2));
}


gtsam::Pose3* Pose3_OriginRetract(const gtsam::Vector6* v) {
    return new gtsam::Pose3(gtsam::Pose3::Retract(*v));
}
gtsam::Vector6* Pose3_OriginLocalCoordinates(const gtsam::Pose3* g) {
    return new gtsam::Vector6(gtsam::Pose3::LocalCoordinates(*g));
}
gtsam::Pose3* Pose3_OriginRetractH(const gtsam::Vector6* v, gtsam::Matrix* H) {
    return new gtsam::Pose3(gtsam::Pose3::Retract(*v, *H));
}
gtsam::Vector6* Pose3_OriginLocalCoordinatesH(const gtsam::Pose3* g, gtsam::Matrix* H) {
    return new gtsam::Vector6(gtsam::Pose3::LocalCoordinates(*g, *H));
}



gtsam::Vector6* Pose3_Logmap(const gtsam::Pose3* p) {
    return new gtsam::Vector6(gtsam::Pose3::Logmap(*p));
}
gtsam::Vector6* Pose3_LogmapH(const gtsam::Pose3* p, gtsam::Matrix* H) {
    return new gtsam::Vector6(gtsam::Pose3::Logmap(*p, *H));
}
gtsam::Pose3* Pose3_Expmap(const gtsam::Vector6* xi) {
    return new gtsam::Pose3(gtsam::Pose3::Expmap(*xi));
}
gtsam::Pose3* Pose3_ExpmapH(const gtsam::Vector6* xi, gtsam::Matrix* H) {
    return new gtsam::Pose3(gtsam::Pose3::Expmap(*xi, *H));
}
}