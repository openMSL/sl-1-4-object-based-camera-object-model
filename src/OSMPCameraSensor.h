//
// Copyright 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
// Copyright 2023 BMW AG
// Copyright (C) 2019-2023 Robert Bosch GmbH 
// SPDX-License-Identifier: MPL-2.0
// The MPL-2.0 is only an example here. You can choose any other open source license accepted by OpenMSL, or any other license if this template is used elsewhere.
//

#pragma once

#include "osi_sensordata.pb.h"
struct ObjectInfo
{
    int id;
    int age;
    int movement_state;  // 0:stationary, 1:moving, 2:stopped
};
class OSMPCameraSensor
{
  public:
    void Init(double nominal_range_in);
    osi3::SensorData Step(osi3::SensorView current_in, double time);

  private:
    double nominal_range_;
    std::vector<ObjectInfo> object_history_vector_;

    void UpdateObjectHistoryVector(ObjectInfo& current_object_history, const osi3::SensorView& input_sensor_view, int obj_idx, bool moving);
    static double GetAbsVelocity(const osi3::Vector3d& velocity_3d);
    static int GetObjectInfoIdx(std::vector<ObjectInfo> search_vector, uint64_t search_id);
    static double CalculateAngle(double r1, double r2, double r3, double g1, double g2, double g3);
    static void CalCoordNew(double trans_x, double trans_y, double trans_z, double ego_yaw, double ego_pitch, double ego_roll, double& xn, double& yn, double& zn);
    static void CalculateCoordinate(double a1, double a2, double a3, double d1, double d2, double d3, double b1, double b2, double b3, double& coord);
    static void CrossProduct(const double* vect_a, const double* vect_b, double* cross_p);
    static void Rot2env(double x, double y, double z, double yaw, double pitch, double roll, double& rx, double& ry, double& rz);
    static void Rot2veh(double x, double y, double z, double yaw, double pitch, double roll, double& rx, double& ry, double& rz);
    static void EulerAngle(double egoyaw, double egopitch, double egoroll, double objectyaw, double objectpitch, double objectroll, double& rr1, double& rr2, double& rr3);
    static double ArcTan(double num, double denom);

    /* Private File-based Logging just for Debugging */
#ifdef PRIVATE_LOG_PATH
    static ofstream private_log_file;
#endif

    static void FmiVerboseLogGlobal(const char* format, ...)
    {
#ifdef VERBOSE_FMI_LOGGING
#ifdef PRIVATE_LOG_PATH
        va_list ap;
        va_start(ap, format);
        char buffer[1024];
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH, ios::out | ios::app);
        if (private_log_file.is_open())
        {
#ifdef _WIN32
            vsnprintf_s(buffer, 1024, format, ap);
#else
            vsnprintf(buffer, 1024, format, ap);
#endif
            private_log_file << "OSMPCameraSensor"
                             << "::Global:FMI: " << buffer << endl;
            private_log_file.flush();
        }
#endif
#endif
    }

    void InternalLog(const char* category, const char* format, va_list arg)
    {
#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
        char buffer[1024];
#ifdef _WIN32
        vsnprintf_s(buffer, 1024, format, arg);
#else
        vsnprintf(buffer, 1024, format, arg);
#endif
#ifdef PRIVATE_LOG_PATH
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH, ios::out | ios::app);
        if (private_log_file.is_open())
        {
            private_log_file << "OSMPCameraSensor"
                             << "::" << instanceName << "<" << ((void*)this) << ">:" << category << ": " << buffer << endl;
            private_log_file.flush();
        }
#endif
#ifdef PUBLIC_LOGGING
        if (loggingOn && loggingCategories.count(category))
            functions.logger(functions.componentEnvironment, instanceName.c_str(), fmi2OK, category, buffer);
#endif
#endif
    }

    void FmiVerboseLog(const char* format, ...)
    {
#if defined(VERBOSE_FMI_LOGGING) && (defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING))
        va_list ap;
        va_start(ap, format);
        internal_log("FMI", format, ap);
        va_end(ap);
#endif
    }

    /* Normal Logging */
    void NormalLog(const char* category, const char* format, ...)
    {
#if defined(PRIVATE_LOG_PATH) || defined(PUBLIC_LOGGING)
        va_list ap;
        va_start(ap, format);
        internal_log(category, format, ap);
        va_end(ap);
#endif
    }
};