#define _USE_MATH_DEFINES
#include <cmath>
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
bool Values_insertValues(gtsam::Values* v, const gtsam::Values* u) {
    try {
        v->insert(*u);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}

bool Values_insertCal3DS2(gtsam::Values* v,             //
                          const gtsam::Key j,           //
                          const gtsam::Cal3DS2* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertDouble(gtsam::Values* v,    //
                         const gtsam::Key j,  //
                         double val) {        //
    try {
        v->insert(j, val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertPoint2(gtsam::Values* v,            //
                         const gtsam::Key j,          //
                         const gtsam::Point2* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertPoint3(gtsam::Values* v,            //
                         const gtsam::Key j,          //
                         const gtsam::Point3* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertPose2(gtsam::Values* v,           //
                        const gtsam::Key j,         //
                        const gtsam::Pose2* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertPose3(gtsam::Values* v,           //
                        const gtsam::Key j,         //
                        const gtsam::Pose3* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertRot2(gtsam::Values* v,          //
                       const gtsam::Key j,        //
                       const gtsam::Rot2* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
bool Values_insertRot3(gtsam::Values* v,          //
                       const gtsam::Key j,        //
                       const gtsam::Rot3* val) {  //
    try {
        v->insert(j, *val);
        return true;
    } catch (const gtsam::ValuesKeyAlreadyExists e) {
        std::cout << "caught ValuesKeyAlreadyExists: " << e.what() << std::endl;
        return false;
    }
}
const gtsam::KeyVector* Values_keys(const gtsam::Values* v) {
    return new gtsam::KeyVector(v->keys());
}
//
//
//
const gtsam::Vector1* Values_atDouble(const gtsam::Values* v,  //
                                      const gtsam::Key j) {    //
    // Returns Vector1 instead of double so we can pass null
    // and so we can delete the object when finshed.
    try {
        return new gtsam::Vector1(v->at<double>(j));
    } catch (const gtsam::ValuesKeyDoesNotExist e) {
        std::cout << "caught ValuesKeyDoesNotExist: " << e.what() << std::endl;
        return nullptr;
    } catch (const gtsam::ValuesIncorrectType e) {
        std::cout << "caught ValuesIncorrectType: " << e.what() << std::endl;
        return nullptr;
    }
}
gtsam::Point2* Values_atPoint2(const gtsam::Values* v,  //
                               const gtsam::Key j) {    //
    try {
        return new gtsam::Point2(v->at<gtsam::Point2>(j));
    } catch (const gtsam::ValuesKeyDoesNotExist e) {
        std::cout << "caught ValuesKeyDoesNotExist: " << e.what() << std::endl;
        return nullptr;
    } catch (const gtsam::ValuesIncorrectType e) {
        std::cout << "caught ValuesIncorrectType: " << e.what() << std::endl;
        return nullptr;
    }
}
gtsam::Point3* Values_atPoint3(const gtsam::Values* v,  //
                               const gtsam::Key j) {    //
    try {
        return new gtsam::Point3(v->at<gtsam::Point3>(j));
    } catch (const gtsam::ValuesKeyDoesNotExist e) {
        std::cout << "caught ValuesKeyDoesNotExist: " << e.what() << std::endl;
        return nullptr;
    } catch (const gtsam::ValuesIncorrectType e) {
        std::cout << "caught ValuesIncorrectType: " << e.what() << std::endl;
        return nullptr;
    }
}
gtsam::Pose2* Values_atPose2(const gtsam::Values* v,  //
                             const gtsam::Key j) {    //
    try {
        return new gtsam::Pose2(v->at<gtsam::Pose2>(j));
    } catch (const gtsam::ValuesKeyDoesNotExist e) {
        std::cout << "caught ValuesKeyDoesNotExist: " << e.what() << std::endl;
        return nullptr;
    } catch (const gtsam::ValuesIncorrectType e) {
        std::cout << "caught ValuesIncorrectType: " << e.what() << std::endl;
        return nullptr;
    }
}
gtsam::Pose3* Values_atPose3(const gtsam::Values* v,  //
                             const gtsam::Key j) {    //
    try {
        return new gtsam::Pose3(v->at<gtsam::Pose3>(j));
    } catch (const gtsam::ValuesKeyDoesNotExist e) {
        std::cout << "caught ValuesKeyDoesNotExist: " << e.what() << std::endl;
        return nullptr;
    } catch (const gtsam::ValuesIncorrectType e) {
        std::cout << "caught ValuesIncorrectType: " << e.what() << std::endl;
        return nullptr;
    }
}
gtsam::Cal3DS2* Values_atCal3DS2(const gtsam::Values* v,  //
                                 const gtsam::Key j) {    //
    try {
        return new gtsam::Cal3DS2(v->at<gtsam::Cal3DS2>(j));
    } catch (const gtsam::ValuesKeyDoesNotExist e) {
        std::cout << "caught ValuesKeyDoesNotExist: " << e.what() << std::endl;
        return nullptr;
    } catch (const gtsam::ValuesIncorrectType e) {
        std::cout << "caught ValuesIncorrectType: " << e.what() << std::endl;
        return nullptr;
    }
}
bool Values_exists(const gtsam::Values* v,  //
                   const gtsam::Key j) {    //
    return v->exists(j);
}
void Values_clear(gtsam::Values* v) {
    v->clear();
}
uint64_t Values_size(const gtsam::Values* v) {
    return v->size();
}
gtsam::Values* Values_retract(const gtsam::Values* v,  //
                              const gtsam::VectorValues* delta) {
    return new gtsam::Values(v->retract(*delta));
}
}