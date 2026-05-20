#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

extern "C" {
gtsam::Values* Values() {
    return new gtsam::Values();
}
/**
 * Note that sometimes "Values" should be owned and sometimes
 * only observed, e.g. inside the CustomFactor error function.
 */
void Values_delete(gtsam::Values* p) {
    delete p;
}
void Values_print(gtsam::Values* v) {
    v->print();
}
/** Clones val. */
void Values_insertPose2(gtsam::Values* v,    //
                        const gtsam::Key j,  //
                        const gtsam::Pose2* val) {
    v->insert(j, *val);
}
void Values_insertPose3(gtsam::Values* v,    //
                        const gtsam::Key j,  //
                        const gtsam::Pose3* val) {
    v->insert(j, *val);
}
void Values_insertCal3DS2(gtsam::Values* v,    //
                          const gtsam::Key j,  //
                          const gtsam::Cal3DS2* val) {
    v->insert(j, *val);
}
const gtsam::Pose2* Values_atPose2(const gtsam::Values* v,  //
                                   const gtsam::Key j) {
    return new gtsam::Pose2(v->at<gtsam::Pose2>(j));
}
const gtsam::Pose3* Values_atPose3(const gtsam::Values* v,  //
                                   const gtsam::Key j) {
    return new gtsam::Pose3(v->at<gtsam::Pose3>(j));
}
const gtsam::Cal3DS2* Values_atCal3DS2(const gtsam::Values* v,  //
                                       const gtsam::Key j) {
    return new gtsam::Cal3DS2(v->at<gtsam::Cal3DS2>(j));
}
bool Values_exists(const gtsam::Values* v, const gtsam::Key j) {
    return v->exists(j);
}
void Values_clear(gtsam::Values* v) {
    v->clear();
}
uint64_t Values_size(const gtsam::Values* v) {
    return v->size();
}
}