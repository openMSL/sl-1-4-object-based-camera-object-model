//
// Copyright 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
// Copyright 2023 BMW AG
// SPDX-License-Identifier: MPL-2.0
//

#include "OSMPCameraSensor.h"

/*
 * Debug Breaks
 *
 * If you define DEBUG_BREAKS the FMU will automatically break
 * into an attached Debugger on all major computation functions.
 * Note that the FMU is likely to break all environments if no
 * Debugger is actually attached when the breaks are triggered.
 */
#if defined(DEBUG_BREAKS) && !defined(NDEBUG)
#if defined(__has_builtin) && !defined(__ibmxl__)
#if __has_builtin(__builtin_debugtrap)
#define DEBUGBREAK() __builtin_debugtrap()
#elif __has_builtin(__debugbreak)
#define DEBUGBREAK() __debugbreak()
#endif
#endif
#if !defined(DEBUGBREAK)
#if defined(_MSC_VER) || defined(__INTEL_COMPILER)
#include <intrin.h>
#define DEBUGBREAK() __debugbreak()
#else
#include <signal.h>
#if defined(SIGTRAP)
#define DEBUGBREAK() raise(SIGTRAP)
#else
#define DEBUGBREAK() raise(SIGABRT)
#endif
#endif
#endif
#else
#define DEBUGBREAK()
#endif
#define PI 3.14159265
#define camera_FOV 50
#define camera_range 150
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

using namespace std;
const std::vector<ObjectInfo> object_history_vector;

#ifdef PRIVATE_LOG_PATH
ofstream COSMPCameraSensor::private_log_file;
#endif

/*
 * ProtocolBuffer Accessors
 */

void* DecodeIntegerToPointer(fmi2Integer hi, fmi2Integer lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union Addrconv
    {
        struct
        {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.base.lo = lo;
    myaddr.base.hi = hi;
    return reinterpret_cast<void*>(myaddr.address);
#elif PTRDIFF_MAX == INT32_MAX
    return reinterpret_cast<void*>(lo);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

void EncodePointerToInteger(const void* ptr, fmi2Integer& hi, fmi2Integer& lo)
{
#if PTRDIFF_MAX == INT64_MAX
    union Addrconv
    {
        struct
        {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr;
    myaddr.address = reinterpret_cast<unsigned long long>(ptr);
    hi = myaddr.base.hi;
    lo = myaddr.base.lo;
#elif PTRDIFF_MAX == INT32_MAX
    hi = 0;
    lo = reinterpret_cast<int>(ptr);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

bool OSMPCameraSensor::GetFmiSensorViewConfig(osi3::SensorViewConfiguration& data)
{
    if (integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX] > 0)
    {
        void* buffer = DecodeIntegerToPointer(integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX], integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX]);
        NormalLog(
            "OSMP", "Got %08X %08X, reading from %p ...", integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX], integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX], buffer);
        data.ParseFromArray(buffer, integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX]);
        return true;
    }
    return false;
}

void OSMPCameraSensor::SetFmiSensorViewConfigRequest(const osi3::SensorViewConfiguration& data)
{
    data.SerializeToString(current_config_request_buffer_);
    EncodePointerToInteger(
        current_config_request_buffer_->data(), integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX], integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX]);
    integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX] = (fmi2Integer)current_config_request_buffer_->length();
    NormalLog("OSMP",
              "Providing %08X %08X, writing from %p ...",
              integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX],
              integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX],
              current_config_request_buffer_->data());
    swap(current_config_request_buffer_, last_config_request_buffer_);
}

void OSMPCameraSensor::ResetFmiSensorViewConfigRequest()
{
    integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX] = 0;
    integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX] = 0;
    integer_vars_[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX] = 0;
}

bool OSMPCameraSensor::GetFmiSensorViewIn(osi3::SensorView& data)
{
    if (integer_vars_[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX] > 0)
    {
        void* buffer = DecodeIntegerToPointer(integer_vars_[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX], integer_vars_[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX]);
        NormalLog("OSMP", "Got %08X %08X, reading from %p ...", integer_vars_[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX], integer_vars_[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX], buffer);
        data.ParseFromArray(buffer, integer_vars_[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX]);
        return true;
    }
    return false;
}

void OSMPCameraSensor::SetFmiSensorDataOut(const osi3::SensorData& data)
{
    data.SerializeToString(current_output_buffer_);
    EncodePointerToInteger(current_output_buffer_->data(), integer_vars_[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX], integer_vars_[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]);
    integer_vars_[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = (fmi2Integer)current_output_buffer_->length();
    NormalLog("OSMP",
              "Providing %08X %08X, writing from %p ...",
              integer_vars_[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],
              integer_vars_[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX],
              current_output_buffer_->data());
    swap(current_output_buffer_, last_output_buffer_);
}

void OSMPCameraSensor::ResetFmiSensorDataOut()
{
    integer_vars_[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = 0;
    integer_vars_[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX] = 0;
    integer_vars_[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX] = 0;
}

void OSMPCameraSensor::RefreshFmiSensorViewConfigRequest()
{
    osi3::SensorViewConfiguration config;
    if (GetFmiSensorViewConfig(config))
    {
        SetFmiSensorViewConfigRequest(config);
    }
    else
    {
        config.Clear();
        config.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));
        const double field_of_view = 3.14;
        const double range_factor = 1.1;
        const uint32_t update_nanos = 20000000;
        config.set_field_of_view_horizontal(field_of_view);
        config.set_field_of_view_vertical(field_of_view);
        config.set_range(FmiNominalRange() * range_factor);
        config.mutable_update_cycle_time()->set_seconds(0);
        config.mutable_update_cycle_time()->set_nanos(update_nanos);
        config.mutable_update_cycle_offset()->Clear();
        osi3::GenericSensorViewConfiguration* generic = config.add_generic_sensor_view_configuration();
        generic->set_field_of_view_horizontal(field_of_view);
        generic->set_field_of_view_vertical(field_of_view);
        SetFmiSensorViewConfigRequest(config);
    }
}

/*
 * Actual Core Content
 */

fmi2Status OSMPCameraSensor::DoInit()
{

    /* Booleans */
    for (int& boolean_var : boolean_vars_)
    {
        boolean_var = fmi2False;
    }

    /* Integers */
    for (int& integer_var : integer_vars_)
    {
        integer_var = 0;
    }

    /* Reals */
    for (double& real_var : real_vars_)
    {
        real_var = 0.0;
    }

    /* Strings */
    for (auto& string_var : string_vars_)
    {
        string_var = "";
    }

    const double nominal_range = 135.0;
    SetFmiNominalRange(nominal_range);
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::DoStart(fmi2Boolean tolerance_defined, fmi2Real tolerance, fmi2Real start_time, fmi2Boolean stop_time_defined, fmi2Real stop_time)
{
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::DoEnterInitializationMode()
{
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::DoExitInitializationMode()
{
    osi3::SensorViewConfiguration config;
    if (!GetFmiSensorViewConfig(config))
    {
        NormalLog("OSI", "Received no valid SensorViewConfiguration from Simulation Environment, assuming everything checks out.");
    }
    else
    {
        NormalLog("OSI", "Received SensorViewConfiguration for Sensor Id %llu", config.sensor_id().value());
        NormalLog("OSI", "SVC Ground Truth FoV Horizontal %f, FoV Vertical %f, Range %f", config.field_of_view_horizontal(), config.field_of_view_vertical(), config.range());
        NormalLog("OSI",
                  "SVC Mounting Position: (%f, %f, %f)",
                  config.mounting_position().position().x(),
                  config.mounting_position().position().y(),
                  config.mounting_position().position().z());
        NormalLog("OSI",
                  "SVC Mounting Orientation: (%f, %f, %f)",
                  config.mounting_position().orientation().roll(),
                  config.mounting_position().orientation().pitch(),
                  config.mounting_position().orientation().yaw());
    }

    return fmi2OK;
}

//****** Arcus tangens function taking into account the sign of numerator and denominator ****** /
double ArcTan(double num, double denom)
{
	double result = atan(num / denom);
	if (denom < 0) {
		if (num > 0) {
			result += PI;
		}
		else {
			result -= PI;
		}
	}
	return result;
}
void EulerWinkel(double egoyaw, double egopitch, double egoroll, double objectyaw, double objectpitch, double objectroll, double &rr1, double &rr2, double &rr3)
{
	double matrixobject[3][3];
	double matrixego[3][3];
	double mr[3][3];
	double ego_cos_yaw = cos(egoyaw);		// psi		yaw
	double ego_cos_pitch = cos(egopitch);	// theta	pitch
	double ego_cos_roll = cos(egoroll);	// phi		roll
	double ego_sin_yaw = sin(egoyaw);
	double ego_sin_pitch = sin(egopitch);
	double ego_sin_roll = sin(egoroll);

	double object_cos_yaw = cos(objectyaw);		// psi		yaw
	double object_cos_pitch = cos(objectpitch);	// theta	pitch
	double object_cos_roll = cos(objectroll);	// phi		roll
	double object_sin_yaw = sin(objectyaw);
	double object_sin_pitch = sin(objectpitch);
	double object_sin_roll = sin(objectroll);

	matrixobject[0][0] = object_cos_pitch*object_cos_yaw;								matrixobject[0][1] = object_cos_pitch*object_sin_yaw;								matrixobject[0][2] = -object_sin_pitch;
	matrixobject[1][0] = object_sin_roll*object_sin_pitch*object_cos_yaw - object_cos_roll*object_sin_yaw;	matrixobject[1][1] = object_sin_roll*object_sin_pitch*object_sin_yaw + object_cos_roll*object_cos_yaw;	matrixobject[1][2] = object_sin_roll*object_cos_pitch;
	matrixobject[2][0] = object_cos_roll*object_sin_pitch*object_cos_yaw + object_sin_roll*object_sin_yaw;	matrixobject[2][1] = object_cos_roll*object_sin_pitch*object_sin_yaw - object_sin_roll*object_cos_yaw;	matrixobject[2][2] = object_cos_roll*object_cos_pitch;

	matrixego[0][0] = ego_cos_pitch*ego_cos_yaw;								matrixego[1][0] = ego_cos_pitch*ego_sin_yaw;								matrixego[2][0] = -ego_sin_pitch;
	matrixego[0][1] = ego_sin_roll*ego_sin_pitch*ego_cos_yaw - ego_cos_roll*ego_sin_yaw;	matrixego[1][1] = ego_sin_roll*ego_sin_pitch*ego_sin_yaw + ego_cos_roll*ego_cos_yaw;	matrixego[2][1] = ego_sin_roll*ego_cos_pitch;
	matrixego[0][2] = ego_cos_roll*ego_sin_pitch*ego_cos_yaw + ego_sin_roll*ego_sin_yaw;	matrixego[1][2] = ego_cos_roll*ego_sin_pitch*ego_sin_yaw - ego_sin_roll*ego_cos_yaw;	matrixego[2][2] = ego_cos_roll*ego_cos_pitch;

	mr[0][0] = matrixego[0][0] * matrixobject[0][0] + matrixego[0][1] * matrixobject[1][0] + matrixego[0][2] * matrixobject[2][0];
	mr[0][1] = matrixego[0][0] * matrixobject[0][1] + matrixego[0][1] * matrixobject[1][1] + matrixego[0][2] * matrixobject[2][1];
	mr[0][2] = matrixego[0][0] * matrixobject[0][2] + matrixego[0][1] * matrixobject[1][2] + matrixego[0][2] * matrixobject[2][2];

	mr[1][0] = matrixego[1][0] * matrixobject[0][0] + matrixego[1][1] * matrixobject[1][0] + matrixego[1][2] * matrixobject[2][0];
	mr[1][1] = matrixego[1][0] * matrixobject[0][1] + matrixego[1][1] * matrixobject[1][1] + matrixego[1][2] * matrixobject[2][1];
	mr[1][2] = matrixego[1][0] * matrixobject[0][2] + matrixego[1][1] * matrixobject[1][2] + matrixego[1][2] * matrixobject[2][2];

	mr[2][0] = matrixego[2][0] * matrixobject[0][0] + matrixego[2][1] * matrixobject[1][0] + matrixego[2][2] * matrixobject[2][0];
	mr[2][1] = matrixego[2][0] * matrixobject[0][1] + matrixego[2][1] * matrixobject[1][1] + matrixego[2][2] * matrixobject[2][1];
	mr[2][2] = matrixego[2][0] * matrixobject[0][2] + matrixego[2][1] * matrixobject[1][2] + matrixego[2][2] * matrixobject[2][2];

	/*normal_log("osi", "matrixego: %f,%f,%f", MR[1][1]);
	normal_log("osi", "matrixego: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixego: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixobject: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixobject: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixobject: %f,%f,%f", rvxx,rvxy,rvxz);*/

	//R1 = MR[0][0];
	//R2 = MR[0][1];
	//R3 = MR[0][2];



	rr1 = ArcTan(-mr[0][2], sqrt(mr[0][0] * mr[0][0] + mr[1][0] * mr[1][0]));
	rr2 = ArcTan(mr[0][1] / cos(rr1), mr[0][0] / cos(rr1));
	rr3 = ArcTan(mr[1][2] / cos(rr1), mr[2][2] / cos(rr1));
}





void TransposeRotationMatrix(double matrix_in[3][3], double matrix_trans[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            matrix_trans[j][i] = matrix_in[i][j];
        }
    }
}

void OSMPCameraSensor::RotatePoint(double x, double y, double z, double yaw, double pitch, double roll, double& rx, double& ry, double& rz)
{
    double matrix[3][3];
    double cos_yaw = cos(yaw);
    double cos_pitch = cos(pitch);
    double cos_roll = cos(roll);
    double sin_yaw = sin(yaw);
    double sin_pitch = sin(pitch);
    double sin_roll = sin(roll);

    matrix[0][0] = cos_yaw * cos_pitch;
    matrix[0][1] = cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll;
    matrix[0][2] = cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll;
    matrix[1][0] = sin_yaw * cos_pitch;
    matrix[1][1] = sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll;
    matrix[1][2] = sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll;
    matrix[2][0] = -sin_pitch;
    matrix[2][1] = cos_pitch * sin_roll;
    matrix[2][2] = cos_pitch * cos_roll;

    double matrix_trans[3][3];
    TransposeRotationMatrix(matrix, matrix_trans);

    rx = matrix_trans[0][0] * x + matrix_trans[0][1] * y + matrix_trans[0][2] * z;
    ry = matrix_trans[1][0] * x + matrix_trans[1][1] * y + matrix_trans[1][2] * z;
    rz = matrix_trans[2][0] * x + matrix_trans[2][1] * y + matrix_trans[2][2] * z;
}

/****** Rotation from environment to object coordinate system (yaw, pitch, roll = orientation of car in environment system) ******/
void Rot2veh(double x, double y, double z, double yaw, double pitch, double roll, double &rx, double &ry, double &rz)
{
	double matrix[3][3];
	double cos_yaw = cos(yaw);		// psi		yaw
	double cos_pitch = cos(pitch);	// theta	pitch
	double cos_roll = cos(roll);	// phi		roll
	double sin_yaw = sin(yaw);
	double sin_pitch = sin(pitch);
	double sin_roll = sin(roll);

	matrix[0][0] = cos_pitch*cos_yaw;								matrix[0][1] = cos_pitch*sin_yaw;								matrix[0][2] = -sin_pitch;
	matrix[1][0] = sin_roll*sin_pitch*cos_yaw - cos_roll*sin_yaw;	matrix[1][1] = sin_roll*sin_pitch*sin_yaw + cos_roll*cos_yaw;	matrix[1][2] = sin_roll*cos_pitch;
	matrix[2][0] = cos_roll*sin_pitch*cos_yaw + sin_roll*sin_yaw;	matrix[2][1] = cos_roll*sin_pitch*sin_yaw - sin_roll*cos_yaw;	matrix[2][2] = cos_roll*cos_pitch;

	rx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
	ry = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
	rz = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
}

/****** Rotation from object to environment coordinate system (yaw, pitch, roll = orientation of car in environment system) ******/
void Rot2env(double x, double y, double z, double yaw, double pitch, double roll, double &rx, double &ry, double &rz)	// Koordinatentransformation vom koerperfesten ins raumfeste Koordinatensystem
{
	double matrix[3][3];
	double cos_yaw = cos(yaw);
	double cos_pitch = cos(pitch);
	double cos_roll = cos(roll);
	double sin_yaw = sin(yaw);
	double sin_pitch = sin(pitch);
	double sin_roll = sin(roll);

	matrix[0][0] = cos_yaw*cos_pitch;  matrix[0][1] = cos_yaw*sin_pitch*sin_roll - sin_yaw*cos_roll; matrix[0][2] = cos_yaw*sin_pitch*cos_roll + sin_yaw*sin_roll;
	matrix[1][0] = sin_yaw*cos_pitch;  matrix[1][1] = sin_yaw*sin_pitch*sin_roll + cos_yaw*cos_roll; matrix[1][2] = sin_yaw*sin_pitch*cos_roll - cos_yaw*sin_roll;
	matrix[2][0] = -sin_pitch;         matrix[2][1] = cos_pitch*sin_roll;                            matrix[2][2] = cos_pitch*cos_roll;

	rx = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z;
	ry = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z;
	rz = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z;
}

// Function to find 
// cross product of two vector array. 
void CrossProduct(double vect_a[], double vect_b[], double cross_p[])

{

	cross_p[0] = vect_a[1] * vect_b[2] - vect_a[2] * vect_b[1];
	cross_p[1] = vect_a[2] * vect_b[0] - vect_a[0] * vect_b[2];
	cross_p[2] = vect_a[0] * vect_b[1] - vect_a[1] * vect_b[0];
}
void CalculateKoordinate(double a1, double a2, double a3, double d1, double d2, double d3, double b1, double b2, double b3, double &koord)
{
	double t = -1 / (a1 * a1 + a2 * a2 + a3 * a3)*(d1 * a1 + d2 * a2 + d3 * a3);
	double ff[3];
	ff[0] = d1 + t * a1;
	ff[1] = d2 + t * a2;
	ff[2] = d3 + t * a3;
	double lz = sqrt((d1 - ff[0])*(d1 - ff[0]) + (d2 - ff[1])*(d2 - ff[1]) + (d3 - ff[2])*(d3 - ff[2]));
	double h0 = d1 - ff[0];
	double h1 = d2 - ff[1];
	double h2 = d3 - ff[2];
	//double Koordinate = 0.0;
	double zaehler_lot = b1 * h0 + b2 * h1 + b3 * h2;
	double nenner_lot = lz * sqrt(b1 * b1 + b2 * b2 + b3 * b3);
	double	beta = acos(round(zaehler_lot / nenner_lot));
	double koordinate=0.0;
    const int value1=90;
    const int value2 = 180;
	if (beta > value1 / value2 * PI) { koordinate = -abs(lz); }
	else { koordinate = abs(lz); }
	//Koordinate = 3;
	//return Koordinate;
	koord = koordinate;

}

void CalKoordNew(double trans_x, double trans_y, double trans_z, double ego_yaw, double ego_pitch, double ego_roll, double &xn, double &yn, double &zn)
{

	double rvxx = 0; double rvxy = 0; double rvxz = 0;
	double rvyx = 0; double rvyy = 0; double rvyz = 0;
	double rvzx = 0; double rvzy = 0; double rvzz = 0;

	Rot2env(1, 0, 0, ego_yaw, ego_pitch, ego_roll, rvxx, rvxy, rvxz);
	Rot2env(0, 1, 0, ego_yaw, ego_pitch, ego_roll, rvyx, rvyy, rvyz);
	Rot2env(0, 0, 1, ego_yaw, ego_pitch, ego_roll, rvzx, rvzy, rvzz);
	
	double vectrvx[] = { rvxx, rvxy, rvxz }; double vectrvy[] = { rvyx, rvyy, rvyz }; double vectrvz[] = { rvzx, rvzy, rvzz };
	double vectexy[3]; double vectexz[3]; double vecteyz[3];
	CrossProduct(vectrvx, vectrvy, vectexy); CrossProduct(vectrvx, vectrvz, vectexz); CrossProduct(vectrvy, vectrvz, vecteyz);
	
	double d[] = { trans_x, trans_y, trans_z };
	//lot berechnen 	
	double znn = 0;
	double ynn = 0;
	double xnn = 0;
	CalculateKoordinate(vectexy[0], vectexy[1], vectexy[2], d[0], d[1], d[2], vectrvz[0], vectrvz[1], vectrvz[2], znn);
	CalculateKoordinate(vectexz[0], vectexz[1], vectexz[2], d[0], d[1], d[2], vectrvy[0], vectrvy[1], vectrvy[2], ynn);
	CalculateKoordinate(vecteyz[0], vecteyz[1], vecteyz[2], d[0], d[1], d[2], vectrvx[0], vectrvx[1], vectrvx[2], xnn);
	zn = znn;
	yn = ynn;
	xn = xnn;
}







float CalculateAngle(double r1, double r2, double r3, double g1, double g2, double g3)
{
    const int value3 = 180;
	double nenner1 = g1 * r1 + g2 * r2 + g3 * r3;
	double nenner2 = sqrt(g1*g1 + g2 * g2 + g3 * g3)*sqrt(r1*r1 + r2 * r2 + r3 * r3);
	double angle = acos(nenner1 / nenner2) * value3 / PI;
	return angle;
}

int GetObjectInfoIdx(std::vector<ObjectInfo> search_vector, int search_id) {
	int idx = 0;
	for (auto it = search_vector.begin(); it < search_vector.end(); it++) {
		if (it->id == search_id) {
			return idx;
		}
		idx++;
	}
	return -1;
}

double GetAbsVelocity(const osi3::Vector3d& velocity_3d) {
	return sqrt(pow(velocity_3d.x(), 2) + pow(velocity_3d.y(), 2) + pow(velocity_3d.z(), 2));
}


void UpdateObjectHistoryVector(ObjectInfo& current_object_history, const osi3::SensorView& input_sensor_view, int obj_idx, bool moving) {
	int current_object_idx=0;
	if (moving) {
            current_object_idx = GetObjectInfoIdx(object_history_vector, input_sensor_view.global_ground_truth().moving_object(obj_idx).id().value());
	}
	else {
            current_object_idx = GetObjectInfoIdx(object_history_vector, input_sensor_view.global_ground_truth().stationary_object(obj_idx).id().value());
	}
	if (current_object_idx != -1) {
		object_history_vector.at(current_object_idx).age++;
		if (moving) {
                    if (GetAbsVelocity(input_sensor_view.global_ground_truth().moving_object(obj_idx).base().velocity()) > 0.01)
                    {
				object_history_vector.at(current_object_idx).movement_state = 1;
			}
			else if (object_history_vector.at(current_object_idx).movement_state == 1) {
				object_history_vector.at(current_object_idx).movement_state = 2;
			}
		}
		current_object_history = object_history_vector.at(current_object_idx);
	}
	else {
		current_object_history.id = current_object_idx;
		current_object_history.age = 1;
		if (moving) {
                    if (GetAbsVelocity(input_sensor_view.global_ground_truth().moving_object(obj_idx).base().velocity()) > 0.01)
                    {
				current_object_history.movement_state = 1;
			}
			else {
				current_object_history.movement_state = 0;
			}
		}
		else {
			current_object_history.movement_state = 0;
		}
		g_object_history_vector.push_back(current_object_history);
	}
}











fmi2Status OSMPCameraSensor::DoCalc(fmi2Real current_communication_point, fmi2Real communication_step_size, fmi2Boolean no_set_fmu_state_prior_to_current_pointfmi_2_component)
{
DEBUGBREAK();

	osi3::SensorData current_out;
double time = current_communication_point + communication_step_size;
        NormalLog("OSI", "Calculating Camera Sensor at %f for %f (step size %f)", current_communication_point, time, communication_step_size);
#ifndef COMPILE_VTD_2_2
	// OSI standard..
	osi3::SensorView current_in;
    if (GetFmiSensorViewIn(current_in))
    {
		osi3::SensorView& current_view_in = current_in;
#else
	// VTD v2.2 receives SensorData..
	osi3::SensorData current_in;
	if (get_fmi_sensor_data_in(current_in)) {
		if (!current_in.sensor_view_size()) {
			normal_log("OSI", "No valid input for SensorData->SensorView.");
			return fmi2Fatal;
		}
		const osi3::SensorView& current_view_in = current_in.sensor_view(0);
#endif
		size_t nof_mov_obj = current_view_in.global_ground_truth().moving_object().size(); // number of vehicles (including ego vehicle)
		size_t nof_stat_obj = current_view_in.global_ground_truth().stationary_object().size(); // number of vehicles (including ego vehicle)
		size_t nof_traf_lights_obj = current_view_in.global_ground_truth().traffic_light().size();   // number of traffic lights 
		size_t nof_traf_sign_obj = current_view_in.global_ground_truth().traffic_sign().size();      // number of traffic signs 

		NormalLog("OSI", "Number of moving objects: %llu", nof_mov_obj);
                NormalLog("OSI", "Number of stationary objects: %llu", nof_stat_obj);
                NormalLog("OSI", "Number of traffic lights: %llu", nof_traf_lights_obj);
                NormalLog("OSI", "Number of traffic signs: %llu", nof_traf_sign_obj);

		//Center of the bounding box in environment coordinates 
		//Position and Orientation 
		double ego_world_x = 0;
        double ego_world_y = 0;
        double ego_world_z = 0;
        double ego_yaw = 0;
        double ego_pitch = 0;
        double ego_roll = 0;
		double ego_vel_x = 0; double ego_vel_y = 0; double ego_vel_z = 0;//ego velocity 


		double origin_host_x = 0;
                double origin_host_y = 0;
                double origin_host_z = 0;  // Position of the host vehicle Ursprung in host vehicle coordinates
		//Position of the host vehicle Ursprung in environment coordinates
                double origin_world_x = 0;
                double origin_world_y = 0;
                double origin_world_z = 0;

		double bbctr_host_x = 0;
                double bbctr_host_y = 0;
                double bbctr_host_z = 0;
                double origin_rot_x = 0;
                double origin_rot_y = 0;
                double origin_rot_z = 0;
		//Position of the sensor in environment coordinates 
		double sens_world_x = 0;
                double sens_world_y = 0;
                double sens_world_z = 0;
		//Camera view direction after calculation of all orientation - 
		double sens_sv_x = 0;
                double sens_sv_y = 0;
                double sens_sv_z = 0;
		std::vector<int>   masked(nof_mov_obj);  	// define vector with length nof_obj and initialize it to 0


		//Sensor mounting position and orientation (in host vehicle frame with reference to the bbcenter_rear)
		//Todo Mounting Position über CameraSensorViewConfig
                double mpos_x = current_view_in.mounting_position().position().x();
                double mpos_y = current_view_in.mounting_position().position().y();
                double mpos_z = current_view_in.mounting_position().position().z();
                double mpos_yaw = current_view_in.mounting_position().orientation().yaw();
                double mpos_pitch = current_view_in.mounting_position().orientation().pitch();
                double mpos_roll = current_view_in.mounting_position().orientation().roll();

		//double mpos_x = 0;        double mpos_y = 0;            double mpos_z = 0;
		//double mpos_yaw = 0; double mpos_pitch = 0; double mpos_roll = 0;
		double mpos_rot_x = 0; double mpos_rot_y = 0; double mpos_rot_z = 0; //sensor position in ego coordinate system after ego rotation orientation  

		NormalLog("DEBUG",
                          "Sensor mounting postion and orientation: x %.2f, y %.2f, z %.2f, yaw %.2f, pitch %.2f, roll %.2f",
                          mpos_x,
                          mpos_y,
                          mpos_z,
                          mpos_yaw,
                          mpos_pitch,
                          mpos_roll);


		osi3::Identifier ego_id = current_view_in.global_ground_truth().host_vehicle_id();
                NormalLog("OSI", "Looking for EgoVehicle with ID: %llu", ego_id.value());

		/*Calculating the position of the ego vehicle*/
		/*Calculating the position of the Sensor position in environment coordinates*/
		/*Calculating the camera view direction*/

		for_each(current_view_in.global_ground_truth().moving_object().begin(),
                         current_view_in.global_ground_truth().moving_object().end(),
			[this, ego_id, &bbctr_host_x, &bbctr_host_y, &bbctr_host_z, &ego_world_x, &ego_world_y, &ego_world_z, &ego_yaw, &ego_pitch, &ego_roll, &ego_vel_x, &ego_vel_y, &ego_vel_z, &origin_world_x, &origin_world_y, &origin_world_z, &origin_rot_x, &origin_rot_y, &origin_rot_z, &origin_host_x, &origin_host_y, &origin_host_z, &mpos_x, &mpos_y, &mpos_z, &mpos_rot_x, &mpos_rot_y, &mpos_rot_z, &mpos_yaw, &mpos_pitch, &mpos_roll, &sens_world_x, &sens_world_y, &sens_world_z, &sens_sv_x, &sens_sv_y, &sens_sv_z](const osi3::MovingObject& obj) {
                             NormalLog("OSI", "MovingObject with ID %llu is EgoVehicle: %d", obj.id().value(), obj.id().value() == ego_id.value());
			if (obj.id().value() == ego_id.value()) {
                            NormalLog("OSI", "Found EgoVehicle with ID: %llu", obj.id().value());

				ego_world_x = obj.base().position().x(); ego_world_y = obj.base().position().y(); ego_world_z = obj.base().position().z();
				ego_yaw = obj.base().orientation().yaw(); ego_pitch = obj.base().orientation().pitch();	ego_roll = obj.base().orientation().roll();
				
				//ego_World_x = 1; ego_World_y = 1; ego_World_z = 1; ego_yaw = PI; ego_pitch = 0; ego_roll = 0;

				NormalLog("OSI", "Current EGO position and orientation: %f,%f,%f,%f,%f,%f", ego_world_x, ego_world_y, ego_world_z, ego_yaw, ego_pitch, ego_roll);
				ego_vel_x = obj.base().velocity().x(); ego_vel_y = obj.base().velocity().y(); ego_vel_z = obj.base().velocity().z();


				//Position of the ego origin in host vehicle coordinates  (bbcenter_to_rear)
				bbctr_host_x = obj.vehicle_attributes().bbcenter_to_rear().x(); bbctr_host_y = obj.vehicle_attributes().bbcenter_to_rear().y(); bbctr_host_z = obj.vehicle_attributes().bbcenter_to_rear().z();
				
				//bbctr_Host_x = -2; bbctr_Host_y = -0.5; bbctr_Host_z = 0;
                                double h1 = 0;
                                double h2 = 0;
                                double h3 = 0;
				//Position of the ego origin in world coordinates after rotation  
				NormalLog("OSI", "Current EGO bbcenter to rear vector: %f,%f,%f", bbctr_host_x, bbctr_host_y, bbctr_host_z);
				Rot2env(bbctr_host_x, bbctr_host_y, bbctr_host_z, ego_yaw, ego_pitch, ego_roll, origin_rot_x, origin_rot_y, origin_rot_z); //rotate into coordinate system of environment
				origin_world_x = origin_rot_x + ego_world_x;
				origin_world_y = origin_rot_y + ego_world_y;
				origin_world_z = origin_rot_z + ego_world_z;
                                NormalLog("OSI", "Current EGO Origin Position in environment coordinates: %f,%f,%f", origin_world_x, origin_world_y, origin_world_z);

				
				Rot2veh(origin_world_x - ego_world_x, origin_world_y - ego_world_y, origin_world_z - ego_world_z, ego_yaw, ego_pitch, ego_roll, h1, h2, h3);
                                NormalLog("OSI", "Current Host ORigin in Host coordinates: %f,%f,%f", h1, h2, h3);
				double sens_ego_x = origin_host_x + mpos_x;	// sensor x-position in ego coordinate system before ego-orientation rotation
				double sens_ego_y = origin_host_y + mpos_y;	// sensor y-position in ego coordinate system before ego-orientation rotation
				double sens_ego_z = origin_host_z + mpos_z;	// sensor z-position in ego coordinate system before ego-orientation rotation



				Rot2env(sens_ego_x, sens_ego_y, sens_ego_z, ego_yaw, ego_pitch, ego_roll, mpos_rot_x, mpos_rot_y, mpos_rot_z);
				//Position of the sensor in environment coordinates 
				sens_world_x = ego_world_x + mpos_rot_x;
				sens_world_y = ego_world_y + mpos_rot_y;
				sens_world_z = ego_world_z + mpos_rot_z;
                                NormalLog("OSI", "Current Sensor Coordinates in host coordinate system: x %f, y %f, z %f", sens_ego_x, sens_ego_y, sens_ego_z);
                                NormalLog("OSI", "Current Sensor Coordinates in environment coordinate system: x %f, y %f, z %f", sens_world_x, sens_world_y, sens_world_z);

				
				//With sensor-rotation 
				double sens_sv_mprotx = 0; double sens_sv_mproty = 0; double sens_sv_mprotz = 0;
				
				Rot2env(1, 0, 0, mpos_yaw, mpos_pitch, mpos_roll, sens_sv_mprotx, sens_sv_mproty, sens_sv_mprotz);
				
				sens_sv_x = sens_ego_x + sens_sv_mprotx;	// Sichtvektor sensor x in ego coordinate system before ego-orientation rotation
				sens_sv_y = sens_ego_y + sens_sv_mproty;	// sensor y in ego coordinate system before ego-orientation rotation
				sens_sv_z = sens_ego_z + sens_sv_mprotz;	// sensor z in ego coordinate system before ego-orientation rotation
				
				double sens_sv_rot_x = 0; double sens_sv_rot_y = 0; double sens_sv_rot_z = 0;
				
				Rot2env(sens_sv_x, sens_sv_y, sens_sv_z, ego_yaw, ego_pitch, ego_roll, sens_sv_rot_x, sens_sv_rot_y, sens_sv_rot_z);
				sens_sv_x = ego_world_x + sens_sv_rot_x;
				sens_sv_y = ego_world_y + sens_sv_rot_y;
				sens_sv_z = ego_world_z + sens_sv_rot_z;

				//normal_log("OSI", "EGO position and orientation: x %f, y %f, z %f,yaw %f, pitch %f, roll %f", ego_World_x, ego_World_y, ego_World_z, ego_yaw, ego_pitch, ego_roll);
				//normal_log("OSI", "Current BB Position rear: %f,%f,%f", origin_Host_x, origin_Host_y, origin_Host_z);
				//normal_log("OSI", " !!! New Current Sensor Position in environment coordinates: %f,%f,%f", sens_World_x, sens_World_y, sens_World_z);

				//normal_log("OSI", "!! New Current Sensor Position Richtungsvektor in environment coordinates: %f,%f,%f", sensSV_x, sensSV_y, sensSV_z);
				//normal_log("OSI", "Current Width, Height, Length: %f,%f,%f", obj.base().dimension().width(), obj.base().dimension().height(), obj.base().dimension().length());
			}
		});





		/* Clear Output */
                const double value4 = 1000000000.0;
		current_out.Clear();
		current_out.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));
		/* Adjust Timestamps and Ids */
		current_out.mutable_timestamp()->set_seconds((long long int)floor(time));
		current_out.mutable_timestamp()->set_nanos((int)((time - floor(time))*value4));
		/* Copy of SensorView */
                current_out.add_sensor_view()->CopyFrom(current_view_in);



        const double value5 = 1.1;
		double actual_range = FmiNominalRange()*value5;
		/* Calculate vehicle FoV and distance to EGO (needed for occlusion) for all vehicles */

		std::vector<double> distance(nof_mov_obj);  // define vector with length nof_obj and initialize it to 0


		//double veh_rel2_x, veh_rel2_y, veh_rel2_z; //only for DEBUG!!!
                double trans_x;
                double trans_y;
                double trans_z;
                double veh_rel_x;
                double veh_rel_y;
                double veh_rel_z;
		std::vector<double> c1_x(nof_mov_obj);		// x coordinate of corner 1 (in sensor coord sys) (needed as reference corner for location distribution)	
		std::vector<double> c1_y(nof_mov_obj);		// y coordinate of corner 1 (in sensor coord sys)	
		std::vector<double> c1_z(nof_mov_obj);		// z coordinate of corner 1 (in sensor coord sys)	
		std::vector<double> phi_min(nof_mov_obj);   // define vector with length nof_obj and initialize it to 0
		std::vector<double> phi_max(nof_mov_obj);   // define vector with length nof_obj and initialize it to 0
		std::vector<double> phi(nof_mov_obj);
		std::vector<double> distM(nof_mov_obj);  // define vector with length nof_obj and initialize it to 0
                double rel_x=0.0;
                double rel_y = 0.0;
                double rel_z = 0.0;

		for (auto v_i = current_view_in.global_ground_truth().moving_object().begin(); v_i != current_view_in.global_ground_truth().moving_object().end(); ++v_i)
                {
			const osi3::MovingObject& veh = *v_i;
                    auto const i = v_i - current_view_in.global_ground_truth().moving_object().begin();
			if (veh.id().value() != ego_id.value()) {

				/* Get corner coordinates:
				Rotate vehicle center into its own coordinate system
				Add/substract vehicle dimensions
				Rotate corners back into environment coordinate system */
				double veh_x = veh.base().position().x(); // Coordinates of vehicle (center of bounding box)
				double veh_y = veh.base().position().y();
				double veh_z = veh.base().position().z();
				double veh_yaw = veh.base().orientation().yaw();
				double veh_pitch = veh.base().orientation().pitch();
				double veh_roll = veh.base().orientation().roll();
                                const double value10 = 2.0;



				// rotatePoint(veh_x, veh_y, veh_z, ego_yaw, ego_pitch, ego_roll, veh_rel_x, veh_rel_y, veh_rel_z); //rotate into coordinate system of ego vehicle
				Rot2veh(veh_x, veh_y, veh_z, veh_yaw, veh_pitch, veh_roll, veh_rel_x, veh_rel_y, veh_rel_z); //rotate into coordinate system of vehicle    -------------!!!! MF nicht notwendig
				//rot2env(veh_rel_x, veh_rel_y, veh_rel_z, veh_yaw, veh_pitch, veh_roll, veh_rel2_x, veh_rel2_y, veh_rel2_z); //only for DEBUG!!!
				// normal_log("DEBUG","Vehicle %d coordinates in own coord sys: %.2f,%.2f,%.2f", i,veh_rel_x,veh_rel_y,veh_rel_z);
				double veh_width = veh.base().dimension().width();
				double veh_length = veh.base().dimension().length();
				// double veh_height = veh.base().dimension().height(); // should actually be done for 3D (box - 8 corners)
                                double corner1_rel_x = veh_rel_x - veh_length / value10;  // this and the following could be done with x,y,z vectors instead
                                double corner1_rel_y = veh_rel_y + veh_width / value10;
				double corner1_rel_z = veh_rel_z;
                                double corner2_rel_x = veh_rel_x + veh_length / value10;
                                double corner2_rel_y = veh_rel_y + veh_width / value10;
				double corner2_rel_z = veh_rel_z;
                                double corner3_rel_x = veh_rel_x - veh_length / value10;
                                double corner3_rel_y = veh_rel_y - veh_width / value10;
				double corner3_rel_z = veh_rel_z;
                                double corner4_rel_x = veh_rel_x + veh_length / value10;
                                double corner4_rel_y = veh_rel_y - veh_width / value10;
				double corner4_rel_z = veh_rel_z;
                                double corner1_x=0.0;
                                double corner1_y = 0.0;
                                double corner1_z = 0.0;
                                double corner2_x = 0.0;
                                double corner2_y = 0.0;
                                double corner2_z = 0.0;
                                double corner3_x = 0.0;
                                double corner3_y = 0.0;
                                double corner3_z = 0.0;
                                double corner4_x = 0.0;
                                double corner4_y = 0.0;
                                double corner4_z = 0.0;
				Rot2env(corner1_rel_x, corner1_rel_y, corner1_rel_z, veh_yaw, veh_pitch, veh_roll, corner1_x, corner1_y, corner1_z); //rotate into environment coordinate system
				Rot2env(corner2_rel_x, corner2_rel_y, corner2_rel_z, veh_yaw, veh_pitch, veh_roll, corner2_x, corner2_y, corner2_z); //rotate into environment coordinate system
				Rot2env(corner3_rel_x, corner3_rel_y, corner3_rel_z, veh_yaw, veh_pitch, veh_roll, corner3_x, corner3_y, corner3_z); //rotate into environment coordinate system
				Rot2env(corner4_rel_x, corner4_rel_y, corner4_rel_z, veh_yaw, veh_pitch, veh_roll, corner4_x, corner4_y, corner4_z); //rotate into environment coordinate system

				/* Calculate azimuth angles of corners */
				double trans_c1_x = corner1_x - sens_world_x;	// vector from sensor to corner 1
				double trans_c1_y = corner1_y - sens_world_y;
				double trans_c1_z = corner1_z - sens_world_z;
				double trans_c2_x = corner2_x - sens_world_x;	// vector from sensor to corner 2
				double trans_c2_y = corner2_y - sens_world_y;
				double trans_c2_z = corner2_z - sens_world_z;
				double trans_c3_x = corner3_x - sens_world_x;	// vector from sensor to corner 3
				double trans_c3_y = corner3_y - sens_world_y;
				double trans_c3_z = corner3_z - sens_world_z;
				double trans_c4_x = corner4_x - sens_world_x;	// vector from sensor to corner 4
				double trans_c4_y = corner4_y - sens_world_y;
				double trans_c4_z = corner4_z - sens_world_z;
                                double rel_c1_x = 0.0;
                                double rel_c1_y = 0.0;
                                double rel_c1_z = 0.0;
                                double rel_c2_x = 0.0;
                                double rel_c2_y = 0.0;
                                double rel_c2_z = 0.0;
                                double rel_c3_x = 0.0;
                                double rel_c3_y = 0.0;
                                double rel_c3_z = 0.0;
                                double rel_c4_x = 0.0;
                                double rel_c4_y = 0.0;
                                double rel_c4_z = 0.0;
				Rot2veh(trans_c1_x, trans_c1_y, trans_c1_z, ego_yaw, ego_pitch, ego_roll, rel_c1_x, rel_c1_y, rel_c1_z); //rotate into coordinate system of ego vehicle
				Rot2veh(trans_c2_x, trans_c2_y, trans_c2_z, ego_yaw, ego_pitch, ego_roll, rel_c2_x, rel_c2_y, rel_c2_z); //rotate into coordinate system of ego vehicle
				Rot2veh(trans_c3_x, trans_c3_y, trans_c3_z, ego_yaw, ego_pitch, ego_roll, rel_c3_x, rel_c3_y, rel_c3_z); //rotate into coordinate system of ego vehicle
				Rot2veh(trans_c4_x, trans_c4_y, trans_c4_z, ego_yaw, ego_pitch, ego_roll, rel_c4_x, rel_c4_y, rel_c4_z); //rotate into coordinate system of ego vehicle
				c1_x[i] = rel_c1_x;
				c1_y[i] = rel_c1_y;
				c1_z[i] = rel_c1_z;
				double phi_c1 = ArcTan(rel_c1_y, rel_c1_x); //also valid for x<0 (rear cars)
				double phi_c2 = ArcTan(rel_c2_y, rel_c2_x);
				double phi_c3 = ArcTan(rel_c3_y, rel_c3_x);
				double phi_c4 = ArcTan(rel_c4_y, rel_c4_x);
				double phi_list[] = { phi_c1, phi_c2, phi_c3, phi_c4 };
				phi_min[i] = *std::min_element(phi_list, phi_list + 4);
				phi_max[i] = *std::max_element(phi_list, phi_list + 4);


				/** Calculate distance to EGO vehicle **/

				// Derive closest corner coordinates
				// 2do: if closest corner is out of FoV then it must be excluded (passing car)
				double distc1 = sqrt(rel_c1_x * rel_c1_x + rel_c1_y * rel_c1_y + rel_c1_z * rel_c1_z);
				double distc2 = sqrt(rel_c2_x * rel_c2_x + rel_c2_y * rel_c2_y + rel_c2_z * rel_c2_z);
				double distc3 = sqrt(rel_c3_x * rel_c3_x + rel_c3_y * rel_c3_y + rel_c3_z * rel_c3_z);
				double distc4 = sqrt(rel_c4_x * rel_c4_x + rel_c4_y * rel_c4_y + rel_c4_z * rel_c4_z);
				double cc_x = corner1_x;	// declare closest corner coordinates (default value corner 1)
				double cc_y = corner1_y;
				double cc_z = corner1_z;
				distM[i] = distc1;	// distance to closest corner
				if (distc2 < distM[i]) {
					cc_x = corner2_x;
					cc_y = corner2_y;
					cc_z = corner2_z;
					distM[i] = distc2;
				}
				if (distc3 < distance[i]) {
					cc_x = corner3_x;
					cc_y = corner3_y;
					cc_z = corner3_z;
					distM[i] = distc3;
				}
				if (distc4 < distance[i]) {
					cc_x = corner4_x;
					cc_y = corner4_y;
					cc_z = corner4_z;
					distM[i] = distc4;
				}
				trans_x = cc_x - sens_world_x; // Vector from sensor to closest corner of vehicle
				trans_y = cc_y - sens_world_y;
				trans_z = cc_z - sens_world_z;
				// normal_log("DEBUG","trans_x: %.2f, trans_y: %.2f,trans_z: %.2f",trans_x,trans_y,trans_z);
                                NormalLog("OSI",
                                          "Current Veh Position: %.2f,%.2f,%.2f (long name)",
                                          veh.base().position().x(),
                                          veh.base().position().y(),
                                          veh.base().position().z());  // test, delme
                                NormalLog("OSI", "Current Ego Position: %.2f,%.2f,%.2f", ego_world_x, ego_world_y, ego_world_z);
				// rotatePoint(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, rel_x, rel_y, rel_z); //rotate into coordinate system of ego vehicle
				Rot2veh(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, rel_x, rel_y, rel_z); //rotate into coordinate system of ego vehicle
				// distance[i] = sqrt(rel_x * rel_x + rel_y*rel_y + rel_z*rel_z); //same as with trans?
				phi[i] = ArcTan(rel_y, rel_x); // Azimuth of closest corner (was: center of bounding box)

			}
			else
			{
				// normal_log("OSI", "Ignoring EGO Vehicle from occlusion.");
			}
		}


		for (auto v_i = current_view_in.global_ground_truth().moving_object().begin(); v_i != current_view_in.global_ground_truth().moving_object().end();
			++v_i) {
			const osi3::MovingObject& veh = *v_i;
			// osi3::DetectedObject *obj = current_out.mutable_object()->Add();
                        size_t const i = v_i - current_view_in.global_ground_truth().moving_object().begin();
			// auto const i = v_i - current_view_in.global_ground_truth().moving_object().begin();
			auto const vid = veh.id().value();
			// auto const vid = i;
			// normal_log("OSI", "Vehicle %d, vid %d, begin %d, end %d", i, vid, current_view_in.global_ground_truth().moving_object().begin(), current_view_in.global_ground_truth().moving_object().end());
			if (vid != ego_id.value()) {


				/****** Occlusion (by comparison with all other vehicles) ******/

				double vis = 1;		// visibility
				double occ = 0;		// occlusion of target vehicle[i]

				// bool masked = 0;
				int occ_ind = 0;	// index of occluding car
                                double loc = 0.0;  // length of occluding car
				for (size_t j = 1; j < nof_mov_obj; ++j) { //assume j=0 =always EGO tbconfirmed! type size_t of j was auto but caused compiler warning					// if ((j!=i) && (distance[j]<distance[i]) && (phi_min[j]<phi_min[i]) && (phi_max[j]<phi_max[i])) masked = 1;
					if ((j != i) && (distM[j] < distM[i])) {  //2do: rear cars should be excluded; may cause false values						// normal_log("DEBUG","i %d, j %d, distance[j] %.2f,distance[i] %.2f, phi_min[j] %.2f, phi_min[i] %.2f, phi_max[j] %.2f, phi_max[i] %.2f, masked %d, nof_obj %d",i,j,distance[j],distance[i],phi_min[j],phi_min[i],phi_max[j],phi_max[i],masked[i],nof_obj);
						if ((phi_max[j] < phi_min[i]) || (phi_min[j] > phi_max[i])) {							//		normal_log("OSI", "Vehicle %d not occluded by vehicle %d", i, j);
						}
						else {
							double phi_min_list[] = { phi_min[i], phi_min[j] }; //surely there is a simpler command as there are only 2 elements 
							double phi_max_list[] = { phi_max[i], phi_max[j] }; //surely there is a simpler command as there are only 2 elements 
							double phi_min_ij = *std::max_element(phi_min_list, phi_min_list + 2);
							double phi_max_ij = *std::min_element(phi_max_list, phi_max_list + 2);
							double occ_temp = (phi_max_ij - phi_min_ij) / (phi_max[i] - phi_min[i]);
							if (occ_temp > occ) {
								occ = occ_temp; // Take the maximum occlusion.
                                                            loc = current_view_in.global_ground_truth().moving_object(j).base().dimension().length();  // length of occluding car
								//			normal_log("DEBUG", "Vehicle %d occluded by vehicle %d with length %.2f", i, j, loc);
								occ_ind = j;
							}
							//			normal_log("OSI", "Vehicle %d occluded (%.2f %%) by vehicle %d", i, occ*100., j);	// moved after mitigation
						}
					}
				}



				if (occ_ind > 0) {	// only if any car is occluded
					vis = 0;
					//		normal_log("OSI", "Vehicle %d mitigated occlusion: %.2f %%", i, 100 - vis * 100);
				}





				if (vis == 1.0) masked[i] = 0;
				else {
					masked[i] = 1; vis = 0;
				}
				//		normal_log("OSI", "Vehicle i %d and masked %d and vis %.2f", i, masked[i], vis);
			}
			else
			{
				// normal_log("OSI", "Ignoring EGO Vehicle %d at relative Position: %.2f,%.2f,%.2f (%.2f,%.2f,%.2f)", i, veh.base().position().x() - sens_World_x, veh.base().position().y() - sens_World_y, veh.base().position().z() - sens_World_z, veh.base().position().x(), veh.base().position().y(), veh.base().position().z());
			}
		}

		//	normal_log("OSI", "Vehicle masked %d,%d,%d,%d", masked[1], masked[2], masked[3], masked[4]);


		//Caculation of moving objects 
		int i = 0;
                for_each(
                    current_view_in.global_ground_truth().moving_object().begin(),
                    current_view_in.global_ground_truth().moving_object().end(),
                    [this,
                     &i,
                     &current_view_in,
                     &current_out,
                     &masked,
                     ego_id,
                     ego_world_x,
                     ego_world_y,
                     ego_world_z,
                     ego_vel_x,
                     ego_vel_y,
                     ego_vel_z,
                     &mpos_x,
                     &mpos_y,
                     &mpos_z,
                     &origin_world_x,
                     &origin_world_y,
                     &origin_world_z,
                     &origin_rot_x,
                     &origin_rot_y,
                     &origin_rot_z,
                     &mpos_rot_x,
                     &mpos_rot_y,
                     &mpos_rot_z,
                     ego_yaw,
                     ego_pitch,
                     ego_roll,
                     actual_range,
                     &sens_world_x,
                     &sens_world_y,
                     &sens_world_z,
                     &sens_sv_x,
                     &sens_sv_y,
                     &sens_sv_z](const osi3::MovingObject& veh) {
			if (veh.id().value() != ego_id.value()) {


				//TypeName = "Type_Unknown";
				std::string typename2 = veh.GetTypeName();
				//		normal_log("DEBUG", "moving object information id %d and masked %d", veh.id().value(), masked[i + 1]);
				//Age
				ObjectInfo current_object_history{};
				int obj_idx = i;
                                int current_object_idx = GetObjectInfoIdx(g_object_history_vector, current_view_in.global_ground_truth().moving_object(obj_idx).id().value());
                                UpdateObjectHistoryVector(current_object_history, current_view_in, obj_idx, true);


				//Calculate the Moving Objects in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
				double xxkoordinate = 0.0;	double yykoordinate = 0.0; double zzkoordinate = 0.0;  //Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 


				double trans_x = veh.base().position().x() - origin_world_x;
				double trans_y = veh.base().position().y() - origin_world_y;
				double trans_z = veh.base().position().z() - origin_world_z;


				double veh_orientation_yaw = veh.base().orientation().yaw();
				double veh_orientation_pitch = veh.base().orientation().pitch();
				double veh_orientation_roll = veh.base().orientation().roll();

			
				NormalLog("DEBUG", "Detected object bb center to mitte Hinterachse in Welt koordinaten %f,%f,%f", trans_x, trans_y, trans_z);
                                NormalLog("DEBUG",
                                          "Detected object Orientierung  %f,%f,%f",
                                          veh.base().orientation().yaw(),
                                          veh.base().orientation().pitch(),
                                          veh.base().orientation().roll());

				double rr1 = 0;
                                double rr2 = 0;
                                double rr3 = 0;
				EulerWinkel(ego_yaw, ego_pitch, ego_roll, veh_orientation_yaw, veh_orientation_pitch, veh_orientation_roll, rr1, rr2,rr3);
                                NormalLog("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", rr1, rr2, rr3);
				
				Rot2veh(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxkoordinate, yykoordinate, zzkoordinate);
                                NormalLog("DEBUG", "Detected object in Vehicle Koordinaten %f,%f,%f", xxkoordinate, yykoordinate, zzkoordinate);
				
				double distance = sqrt(trans_x*trans_x + trans_y * trans_y + trans_z * trans_z);
				//Vector camera sensor, camera view direction 
				double g1 = sens_sv_x - sens_world_x;
				double g2 = sens_sv_y - sens_world_y;
				double g3 = sens_sv_z - sens_world_z;
				//Angle between 
				double angle_to_mov_obj = CalculateAngle(trans_x, trans_y, trans_z, g1, g2, g3);
				//normal_log("OSI", "Winkel Vektors Sensor-Richtungsvektor zum Sensor-Objekt: %f", angle_to_mov_obj);


				/* Calculate radial velocity and acceleration*/
				double vx = veh.base().velocity().x();
				double vy = veh.base().velocity().y();
				double vz = veh.base().velocity().z();
				double delta_x = vx - ego_vel_x;
				double delta_y = vy - ego_vel_y;
				double delta_z = vz - ego_vel_z;
                                double vx_sens=0.0;
                                double vy_sens = 0.0;
                                double vz_sens = 0.0;  // velocity components in coordinate system of ego vehicle
				Rot2veh(delta_x, delta_y, delta_z, ego_yaw, ego_pitch, ego_roll, vx_sens, vy_sens, vz_sens); //rotate into coordinate system of ego vehicle

				NormalLog("OSI", "Velocity of moving object global coordinates: x %f ,y %f z %f \n", vx, vy, vz);
                                NormalLog("OSI", "Velocity of moving object environment coordinates: x %f ,y %f z %f \n", vx_sens, vy_sens, vz_sens);
                                NormalLog("DEBUG",
                                          "moving object information id %d and masked %d and distance %f and angke_to_mov_obj %f",
                                          veh.id().value(),
                                          masked[i + 1],
                                          distance,
                                          angle_to_mov_obj);
				
				if ((distance <= camera_range) && abs(angle_to_mov_obj) < camera_FOV) {
					if (masked[i + 1] == 0) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
						osi3::DetectedMovingObject *obj = current_out.mutable_moving_object()->Add();
						current_out.mutable_moving_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

						int max = 10;
						int min = 0;
						double randomnumber = (rand() % (max - min)) + min;

						double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;
						if (existence_prob > 100) {
							existence_prob = 100;
						}
                                                NormalLog("DEBUG", "Randomnumber is %f", randomnumber);
						double randomPos = randomnumber / 100;
                                                NormalLog("DEBUG", "Random number is %f", randomPos);
						obj->mutable_header()->add_ground_truth_id()->CopyFrom(veh.id());
						obj->mutable_header()->mutable_tracking_id()->set_value(i);
						obj->mutable_header()->set_existence_probability(existence_prob / 100);
						obj->mutable_header()->set_age(current_object_history.age);
						obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
                                                obj->mutable_header()->add_sensor_id()->CopyFrom(current_view_in.sensor_id());

						//Position in Sensor Koordinaten 
						//Input are the Coordinates in the Sensor-Environment 

						obj->mutable_base()->mutable_position()->set_x(xxkoordinate);
						obj->mutable_base()->mutable_position()->set_y(yykoordinate);
						obj->mutable_base()->mutable_position()->set_z(zzkoordinate);

						obj->mutable_base()->mutable_orientation()->set_pitch(rr1);
						obj->mutable_base()->mutable_orientation()->set_roll(rr3);
						obj->mutable_base()->mutable_orientation()->set_yaw(rr2);

						obj->mutable_base()->mutable_velocity()->set_x(vx_sens);
						obj->mutable_base()->mutable_velocity()->set_y(vy_sens);
						obj->mutable_base()->mutable_velocity()->set_z(vz_sens);
						obj->mutable_base()->mutable_dimension()->set_length(veh.base().dimension().length());
						obj->mutable_base()->mutable_dimension()->set_width(veh.base().dimension().width());
						obj->mutable_base()->mutable_dimension()->set_height(veh.base().dimension().height());


						osi3::DetectedMovingObject::CandidateMovingObject* candidate = obj->add_candidate();
						candidate->set_type(veh.type());
						candidate->mutable_vehicle_classification()->CopyFrom(veh.vehicle_classification());
						candidate->set_probability(1);



						NormalLog("DEBUG", "Detected moving object %d minimal distance to Ego/Sensor %f", i, distance);
                                                NormalLog("DEBUG", "Detected moving object %d angle to Ego/Sensor %f", i, angle_to_mov_obj);
                                                NormalLog("DEBUG", "Detected moving object type %d", veh.type());
                                                NormalLog("DEBUG", "Detected moving object type %s", candidate->GetTypeName().c_str());

						//candidate->type()
                                                NormalLog(
                                                    "OSI",
                                                    "Output Object %d[%llu] Probability %f Detected mov object position and orientation in environment coord: %f,%f,%f (%f,%f,%f)",
                                                    i,
                                                    veh.id().value(),
                                                    obj->header().existence_probability(),
                                                    veh.base().position().x(),
                                                    veh.base().position().y(),
                                                    veh.base().position().z(),
                                                    veh.base().orientation().yaw(),
                                                    veh.base().orientation().pitch(),
                                                    veh.base().orientation().roll());
                                                NormalLog("OSI",
                                                          "Output Object %d[%llu] Probability %f Detected mov object position in sensor coord: %f,%f,%f \n",
                                                          i,
                                                          veh.id().value(),
                                                          obj->header().existence_probability(),
                                                          obj->base().position().x(),
                                                          obj->base().position().y(),
                                                          obj->base().position().z());

					}
				}
				else {
                                    NormalLog("OSI",
                                              "Ignoring Vehicle %d[%llu] Outside Sensor Scope Relative Position - ego and GT Position: %f,%f,%f (%f,%f,%f)",
                                              i,
                                              veh.id().value(),
                                              veh.base().position().x() - ego_world_x,
                                              veh.base().position().y() - ego_world_y,
                                              veh.base().position().z() - ego_world_z,
                                              veh.base().position().x(),
                                              veh.base().position().y(),
                                              veh.base().position().z());
                                    NormalLog("OSI", "ENDE.......................................................................");
					//normal_log("OSI", "Ignoring Vehicle %d[%llu] Outside Sensor Scope Object trans_x,trans_y,trans_z, distance to sensor,winkel %f,%f,%f (%f,%f,%f) ,%f,%f", i, veh.id().value(), trans_x, trans_y, trans_z, rel_x, rel_y, rel_z, distance,winkel);

				}
				i++;
			}
			else
			{
                            NormalLog("OSI",
                                      "Ignoring EGO Vehicle %d[%llu] Relative Position: %f,%f,%f (%f,%f,%f)",
                                      i,
                                      veh.id().value(),
                                      veh.base().position().x() - ego_world_x,
                                      veh.base().position().y() - ego_world_y,
                                      veh.base().position().z() - ego_world_z,
                                      veh.base().position().x(),
                                      veh.base().position().y(),
                                      veh.base().position().z());
                            NormalLog("OSI", "ENDE.......................................................................");
			}
		});

		// normal_log("OSI","Mapped %d vehicles to output", i);




		//Stationary objects

		for_each(
                    current_view_in.global_ground_truth().stationary_object().begin(),
                    current_view_in.global_ground_truth().stationary_object().end(),
                    [this,
                     &i,
                     &current_view_in,
                     &current_out,
                     masked,
                     ego_id,
                     ego_world_x,
                     ego_world_y,
                     ego_world_z,
                     &origin_world_x,
                     &origin_world_y,
                     &origin_world_z,
                     &mpos_x,
                     &mpos_y,
                     &mpos_z,
                     ego_yaw,
                     ego_pitch,
                     ego_roll,
                     actual_range,
                     &sens_world_x,
                     &sens_world_y,
                     &sens_world_z,
                     &sens_sv_x,
                     &sens_sv_y,
                     &sens_sv_z](const osi3::StationaryObject& stobj) {


			double xxkoordinate = 0.0;
                        double yykoordinate = 0.0;
                        double zzkoordinate =
                            0.0;  // Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
			double trans_x = stobj.base().position().x() - origin_world_x;
			double trans_y = stobj.base().position().y() - origin_world_y;
			double trans_z = stobj.base().position().z() - origin_world_z;


			double stobj_orientation_yaw = stobj.base().orientation().yaw();
			double stobj_orientation_pitch = stobj.base().orientation().pitch();
			double stobj_orientation_roll = stobj.base().orientation().roll();


			double rr1 = 0;
                        double rr2 = 0;
                        double rr3 = 0;
			EulerWinkel(ego_yaw, ego_pitch, ego_roll, stobj_orientation_yaw, stobj_orientation_pitch, stobj_orientation_roll, rr1, rr2, rr3);
                        NormalLog("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", rr1, rr2, rr3);
			CalKoordNew(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxkoordinate, yykoordinate, zzkoordinate);
                        NormalLog("OSI", "Stationary Object in Sensor-Coordinates: %f,%f,%f", xxkoordinate, yykoordinate, zzkoordinate);


			double distance = sqrt(trans_x*trans_x + trans_y * trans_y + trans_z * trans_z);
			//Vector camera sensor, camera view direction 
			double g1 = sens_sv_x - sens_world_x;
			double g2 = sens_sv_y - sens_world_y;
			double g3 = sens_sv_z - sens_world_z;
			//Angle between 
			double angle_to_stat_obj = CalculateAngle(trans_x, trans_y, trans_z, g1, g2, g3);
                        NormalLog("OSI", "Winkel Vektors Sensor-Richtungsvektor zum Sensor-Objekt: %f", angle_to_stat_obj);



			//Define detection range //
			//abs wegen negativen werten 
			if ((distance <= camera_range) && abs(angle_to_stat_obj) < camera_FOV && !masked[i]) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
				osi3::DetectedStationaryObject *obj = current_out.mutable_stationary_object()->Add();
				current_out.mutable_stationary_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

				int max = 10;
				int min = 0;
				int randomnumber = (rand() % (max - min)) + min;

				double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;
				if (existence_prob > 100) {
					existence_prob = 100;
				}

				obj->mutable_header()->add_ground_truth_id()->CopyFrom(stobj.id());
				obj->mutable_header()->mutable_tracking_id()->set_value(i);
				obj->mutable_header()->set_existence_probability(existence_prob / 100);
			
				obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
                                obj->mutable_header()->add_sensor_id()->CopyFrom(current_view_in.sensor_id());

				//Position in Sensor Koordinaten 
				//Input are the Coordinates in the Sensor-Environment 
				obj->mutable_base()->mutable_position()->set_x(xxkoordinate);
				obj->mutable_base()->mutable_position()->set_y(yykoordinate);
				obj->mutable_base()->mutable_position()->set_z(zzkoordinate);
				obj->mutable_base()->mutable_orientation()->set_pitch(rr1);
				obj->mutable_base()->mutable_orientation()->set_roll(rr3);
				obj->mutable_base()->mutable_orientation()->set_yaw(rr2);

				obj->mutable_base()->mutable_dimension()->set_length(stobj.base().dimension().length());
				obj->mutable_base()->mutable_dimension()->set_width(stobj.base().dimension().width());
				obj->mutable_base()->mutable_dimension()->set_height(stobj.base().dimension().height());


				osi3::DetectedStationaryObject::CandidateStationaryObject* candidate = obj->add_candidate();
				
				candidate->mutable_classification()->CopyFrom(stobj.classification());
				candidate->set_probability(1);


				NormalLog("DEBUG", "Detected stationary object %d minimal distance to Ego/Sensor %f", i, distance);
                                NormalLog("DEBUG", "Detected stationary object %d angle to Ego/Sensor %f", i, angle_to_stat_obj);
                                NormalLog("OSI",
                                          "Output Object %d[%llu] Probability %f Detected stat object position and orientation in environment coord: %f,%f,%f (%f,%f,%f)",
                                          i,
                                          stobj.id().value(),
                                          obj->header().existence_probability(),
                                          stobj.base().position().x(),
                                          stobj.base().position().y(),
                                          stobj.base().position().z(),
                                          stobj.base().orientation().yaw(),
                                          stobj.base().orientation().pitch(),
                                          stobj.base().orientation().roll());
                                NormalLog("OSI",
                                          "Output Object %d[%llu] Probability %f Detected stat object position in sensor coord: %f,%f,%f \n",
                                          i,
                                          stobj.id().value(),
                                          obj->header().existence_probability(),
                                          obj->base().position().x(),
                                          obj->base().position().y(),
                                          obj->base().position().z());
				i++;
			}
			else {
                            NormalLog("OSI",
                                      "Ignoring stationary objects %d[%llu] Outside Sensor Scope Relative Position - ego and GT Position: %f,%f,%f (%f,%f,%f)",
                                      i,
                                      stobj.id().value(),
                                      stobj.base().position().x() - ego_world_x,
                                      stobj.base().position().y() - ego_world_y,
                                      stobj.base().position().z() - ego_world_z,
                                      stobj.base().position().x(),
                                      stobj.base().position().y(),
                                      stobj.base().position().z());
				//normal_log("OSI", "Ignoring stobjicle %d[%llu] Outside Sensor Scope Object trans_x,trans_y,trans_z, distance to sensor,winkel %f,%f,%f (%f,%f,%f) ,%f,%f", i, stobj.id().value(), trans_x, trans_y, trans_z, rel_x, rel_y, rel_z, distance,winkel);

			}


		});


		int itl = 0;

		//Traffic Lights 
		for_each(
                    current_view_in.global_ground_truth().traffic_light().begin(),
                    current_view_in.global_ground_truth().traffic_light().end(),
                    [this,
                     &itl,
                     &current_view_in,
                     &current_out,
                     masked,
                     ego_id,
                     ego_world_x,
                     ego_world_y,
                     ego_world_z,
                     &origin_world_x,
                     &origin_world_y,
                     &origin_world_z,
                     &mpos_x,
                     &mpos_y,
                     &mpos_z,
                     ego_yaw,
                     ego_pitch,
                     ego_roll,
                     actual_range,
                     &sens_world_x,
                     &sens_world_y,
                     &sens_world_z,
                     &sens_sv_x,
                     &sens_sv_y,
                     &sens_sv_z](const osi3::TrafficLight& trafficlight) {


			
			//Calculate the traffic light  in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
			double xxkoordinate = 0.0;
                        double yykoordinate = 0.0;
                        double zzkoordinate =
                            0.0;  // Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
			//double trans_x = trafficlight.base().position().x() - sens_World_x;
			//double trans_y = trafficlight.base().position().y() - sens_World_y;
			///double trans_z = trafficlight.base().position().z() - sens_World_z;
			double trans_x = trafficlight.base().position().x() - origin_world_x;
			double trans_y = trafficlight.base().position().y() - origin_world_y;
			double trans_z = trafficlight.base().position().z() - origin_world_z;

			double trafficlightorientationyaw = trafficlight.base().orientation().yaw();
			double trafficlightorientationpitch = trafficlight.base().orientation().pitch();
			double trafficlightorientationroll = trafficlight.base().orientation().roll();


			double rr1 = 0;
                        double rr2 = 0;
                        double rr3 = 0;
			EulerWinkel(ego_yaw, ego_pitch, ego_roll, trafficlightorientationyaw, trafficlightorientationpitch, trafficlightorientationroll, rr1, rr2, rr3);
                        NormalLog("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", rr1, rr2, rr3);
                        NormalLog("OSI",
                                  "Ground Truth Traffic Sign  %f, %f,%f ",
                                  trafficlight.base().position().x(),
                                  trafficlight.base().position().y(),
                                  trafficlight.base().position().z());
			CalKoordNew(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxkoordinate, yykoordinate, zzkoordinate);
                        NormalLog("OSI", "TrafficLight in Sensor-Coordinates: %f,%f,%f", xxkoordinate, yykoordinate, zzkoordinate);


			double distance = sqrt(trans_x*trans_x + trans_y * trans_y + trans_z * trans_z);
			//Vector camera sensor, camera view direction 
			double g1 = sens_sv_x - sens_world_x;
			double g2 = sens_sv_y - sens_world_y;
			double g3 = sens_sv_z - sens_world_z;
			//Angle between 
			double angle_to_traffic_light = CalculateAngle(trans_x, trans_y, trans_z, g1, g2, g3);
			//normal_log("OSI", "Winkel Vektors Sensor-Richtungsvektor zum Sensor-Objekt: %f", angle_to_mov_obj);


			//Define detection range //
			
			if ((distance <= camera_range) && abs(angle_to_traffic_light) < camera_FOV) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
				osi3::DetectedTrafficLight *obj = current_out.mutable_traffic_light()->Add();
				current_out.mutable_traffic_light_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

				int max = 10;
				int min = 0;
				int randomnumber = (rand() % (max - min)) + min;

				double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;
				if (existence_prob > 100) {
					existence_prob = 100;
				}

				obj->mutable_header()->add_ground_truth_id()->CopyFrom(trafficlight.id());
                                NormalLog("OSI", "TrafficLight ID: %f", trafficlight.id());
				obj->mutable_header()->mutable_tracking_id()->set_value(itl);
				obj->mutable_header()->set_existence_probability(existence_prob / 100);
                                NormalLog("OSI", "TrafficLight Existence Probability: %f", existence_prob / 100);
				//obj->mutable_header()->set_age(current_object_history.age);
				obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
                                obj->mutable_header()->add_sensor_id()->CopyFrom(current_view_in.sensor_id());

				//Position in Sensor Koordinaten 
				//Input are the Coordinates in the Sensor-Environment 
				obj->mutable_base()->mutable_position()->set_x(xxkoordinate);
				obj->mutable_base()->mutable_position()->set_y(yykoordinate);
				obj->mutable_base()->mutable_position()->set_z(zzkoordinate);
				obj->mutable_base()->mutable_orientation()->set_pitch(trafficlight.base().orientation().pitch());
				obj->mutable_base()->mutable_orientation()->set_roll(trafficlight.base().orientation().roll());
				obj->mutable_base()->mutable_orientation()->set_yaw(trafficlight.base().orientation().yaw());

				obj->mutable_base()->mutable_dimension()->set_length(trafficlight.base().dimension().length());
				obj->mutable_base()->mutable_dimension()->set_width(trafficlight.base().dimension().width());
				obj->mutable_base()->mutable_dimension()->set_height(trafficlight.base().dimension().height());


				osi3::DetectedTrafficLight::CandidateTrafficLight* candidate = obj->add_candidate();
				//candidate->set_type(stobj.type());
				candidate->mutable_classification()->CopyFrom(trafficlight.classification());
				candidate->mutable_classification()->set_color(trafficlight.classification().color());
				candidate->mutable_classification()->set_mode(trafficlight.classification().mode());
				candidate->mutable_classification()->set_icon(trafficlight.classification().icon());

				candidate->set_probability(1);

				NormalLog("DEBUG", "Detected traffic light color %d", candidate->classification().color());
                                NormalLog("DEBUG", "Detected traffic light mode %d", candidate->classification().mode());
                                NormalLog("DEBUG", "Detected traffic light icon %d", candidate->classification().icon());
                                NormalLog("DEBUG", "Detected traffic light assigned lane id %d", candidate->classification().assigned_lane_id());
                                NormalLog("DEBUG", "Detected traffic classification %d", trafficlight.classification());

				NormalLog("DEBUG", "Detected traffic light %d minimal distance to Ego/Sensor %f", itl, distance);
                                NormalLog("DEBUG", "Detected traffic light %d angle to Ego/Sensor %f", itl, angle_to_traffic_light);
                                NormalLog("OSI",
                                          "Output traffic light %d[%llu] Probability %f Detected traffic light position and orientation in environment coord: %f,%f,%f (%f,%f,%f)",
                                          itl,
                                          trafficlight.id().value(),
                                          obj->header().existence_probability(),
                                          trafficlight.base().position().x(),
                                          trafficlight.base().position().y(),
                                          trafficlight.base().position().z(),
                                          trafficlight.base().orientation().yaw(),
                                          trafficlight.base().orientation().pitch(),
                                          trafficlight.base().orientation().roll());
                                NormalLog("OSI",
                                          "Output Object %d[%llu] Probability %f Detected  traffic light position in sensor coord: %f,%f,%f \n",
                                          itl,
                                          trafficlight.id().value(),
                                          obj->header().existence_probability(),
                                          obj->base().position().x(),
                                          obj->base().position().y(),
                                          obj->base().position().z());
				
				itl++;
			}
			else {
                            NormalLog("OSI",
                                      "Ignoring traffic light %d[%llu] Outside Sensor Scope Relative Position - ego and GT Position: %f,%f,%f (%f,%f,%f)",
                                      itl,
                                      trafficlight.id().value(),
                                      trafficlight.base().position().x() - ego_world_x,
                                      trafficlight.base().position().y() - ego_world_y,
                                      trafficlight.base().position().z() - ego_world_z,
                                      trafficlight.base().position().x(),
                                      trafficlight.base().position().y(),
                                      trafficlight.base().position().z());
				//normal_log("OSI", "Ignoring stobjicle %d[%llu] Outside Sensor Scope Object trans_x,trans_y,trans_z, distance to sensor,winkel %f,%f,%f (%f,%f,%f) ,%f,%f", i, trafficlight.id().value(), trans_x, trans_y, trans_z, rel_x, rel_y, rel_z, distance,winkel);

			}


		});



		int its = 0;
                for_each(
                    current_view_in.global_ground_truth().traffic_sign().begin(),
                    current_view_in.global_ground_truth().traffic_sign().end(),
                    [this,
                     &its,
                     &current_view_in,
                     &current_out,
                     masked,
                     ego_id,
                     ego_world_x,
                     ego_world_y,
                     ego_world_z,
                     &origin_world_x,
                     &origin_world_y,
                     &origin_world_z,
                     &mpos_x,
                     &mpos_y,
                     &mpos_z,
                     ego_yaw,
                     ego_pitch,
                     ego_roll,
                     actual_range,
                     &sens_world_x,
                     &sens_world_y,
                     &sens_world_z,
                     &sens_sv_x,
                     &sens_sv_y,
                     &sens_sv_z](const osi3::TrafficSign& tsobj) {
                        NormalLog("OSI", "TrafficSign with ID %llu ", tsobj.id().value());
			//Calculate the traffic light  in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
			double xxxkoordinate = 0.0;
                        double yyykoordinate = 0.0;
                        double zzzkoordinate =
                            0.0;  // Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
		
			NormalLog("OSI",
                                  "Ground Truth Traffic Sign  %f, %f,%f ",
                                  tsobj.main_sign().base().position().x(),
                                  tsobj.main_sign().base().position().y(),
                                  tsobj.main_sign().base().position().z());
			
			double transTS_x = tsobj.main_sign().base().position().x() - origin_world_x;
			double transTS_y = tsobj.main_sign().base().position().y() - origin_world_y;
			double transTS_z = tsobj.main_sign().base().position().z() - origin_world_z;
			double tsobjmorientationyaw = tsobj.main_sign().base().orientation().yaw();
			double tsobjmorientationpitch = tsobj.main_sign().base().orientation().pitch();
			double tsobjmorientationroll = tsobj.main_sign().base().orientation().roll();


			double rr1 = 0;
                        double rr2 = 0;
                        double rr3 = 0;
			EulerWinkel(ego_yaw, ego_pitch, ego_roll, tsobjmorientationyaw, tsobjmorientationpitch, tsobjmorientationroll, rr1, rr2, rr3);
                        NormalLog("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", rr1, rr2, rr3);

			NormalLog("OSI", "TrafficSign position x %f", tsobj.main_sign().base().position().x() - sens_world_x);
                        NormalLog("OSI", "TrafficSign position y %f", tsobj.main_sign().base().position().y() - sens_world_y);
                        NormalLog("OSI", "TrafficSign position z %f", tsobj.main_sign().base().position().z() - sens_world_z);

			CalKoordNew(transTS_x, transTS_y, transTS_z, ego_yaw, ego_pitch, ego_roll, xxxkoordinate, yyykoordinate, zzzkoordinate);

			NormalLog("OSI", "TrafficSign in Sensor-Coordinates: %f,%f,%f", xxxkoordinate, yyykoordinate, zzzkoordinate);

			double distance = sqrt(transTS_x*transTS_x + transTS_y * transTS_y + transTS_z * transTS_z);
			//Vector camera sensor, camera view direction 
			double g1 = sens_sv_x - sens_world_x;
			double g2 = sens_sv_y - sens_world_y;
			double g3 = sens_sv_z - sens_world_z;
			//Angle between 
			double angle_to_traffic_sign = CalculateAngle(transTS_x, transTS_y, transTS_z, g1, g2, g3);

			if ((distance <= camera_range) && abs(angle_to_traffic_sign) < camera_FOV) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
				osi3::DetectedTrafficSign *obj = current_out.mutable_traffic_sign()->Add();
				current_out.mutable_traffic_sign_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

				int max = 10;
				int min = 0;
				int randomnumber = (rand() % (max - min)) + min;

				double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;

				obj->mutable_header()->add_ground_truth_id()->CopyFrom(tsobj.id());
				obj->mutable_header()->mutable_tracking_id()->set_value(its);
				obj->mutable_header()->set_existence_probability(existence_prob / 100);
				//obj->mutable_header()->set_age(current_object_history.age);
				obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
                                obj->mutable_header()->add_sensor_id()->CopyFrom(current_view_in.sensor_id());

				//Position in Sensor Koordinaten 
				//Input are the Coordinates in the Sensor-Environment 
				obj->mutable_main_sign()->mutable_base()->mutable_position()->set_x(xxxkoordinate);
				obj->mutable_main_sign()->mutable_base()->mutable_position()->set_y(yyykoordinate);
				obj->mutable_main_sign()->mutable_base()->mutable_position()->set_z(zzzkoordinate);
				obj->mutable_main_sign()->mutable_base()->mutable_orientation()->set_pitch(rr1);
				obj->mutable_main_sign()->mutable_base()->mutable_orientation()->set_roll(rr3);
				obj->mutable_main_sign()->mutable_base()->mutable_orientation()->set_yaw(rr2);


				obj->mutable_main_sign()->mutable_base()->mutable_dimension()->set_length(tsobj.main_sign().base().dimension().length());
				obj->mutable_main_sign()->mutable_base()->mutable_dimension()->set_width(tsobj.main_sign().base().dimension().width());
				obj->mutable_main_sign()->mutable_base()->mutable_dimension()->set_height(tsobj.main_sign().base().dimension().height());

				NormalLog("DEBUG", "Detected traffic sign %d minimal distance to Ego/Sensor %f", its, distance);
                                NormalLog("DEBUG", "Detected traffic sign %d angle to Ego/Sensor %f", its, angle_to_traffic_sign);
                                NormalLog("OSI",
                                          "Output traffic sign %d[%llu] Probability %f Detected traffic sign position\
								  								  									  		 						and orientation in environment coord:%f,%f,%f,  (%f,%f,%f)", its, tsobj.id().value(), obj->header().existence_probability(), tsobj.main_sign().base().position().x(), tsobj.main_sign().base().position().y(), tsobj.main_sign().base().position().z(), tsobj.main_sign().base().orientation().yaw(), tsobj.main_sign().base().orientation().pitch(), tsobj.main_sign().base().orientation().roll());
				//normal_log("OSI", "Output Object %d[%llu] Probability %f Detected  traffic sign position in sensor coord: %f,%f,%f \n", its, tsobj.id().value(), obj->header().existence_probability(), obj->main_sign.base().position().x(), obj->main_sign.base().position().y(), obj->main_sign.base().position().z());
				its++;
			}

		});

        NormalLog("OSI", "Mapped %d vehicles to output", i);
        /* Serialize */
        //SetFmiSensorDataOut(current_out);
        //SetFmiValid(1);
        //SetFmiCount(current_out.moving_object_size());
        SetFmiSensorDataOut(current_out);
        SetFmiValid(1);
        SetFmiCount(current_out.moving_object_size());
    }
    else
    {
        /* We have no valid input, so no valid output */
        NormalLog("OSI", "No valid input, therefore providing no valid output.");
        ResetFmiSensorDataOut();
        SetFmiValid(0);
        SetFmiCount(0);
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::DoTerm()
{
    return fmi2OK;
}

void OSMPCameraSensor::DoFree() {}

/*
 * Generic C++ Wrapper Code
 */

OSMPCameraSensor::OSMPCameraSensor(fmi2String theinstance_name,
                                   fmi2Type thefmu_type,
                                   fmi2String thefmu_guid,
                                   fmi2String thefmu_resource_location,
                                   const fmi2CallbackFunctions* thefunctions,
                                   fmi2Boolean thevisible,
                                   fmi2Boolean thelogging_on)
    : instance_name_(theinstance_name),
      fmu_type_(thefmu_type),
      fmu_guid_(thefmu_guid),
      fmu_resource_location_(thefmu_resource_location),
      functions_(*thefunctions),
      visible_(thevisible != 0),
      logging_on_(thelogging_on != 0),
      simulation_started_(false),
      current_output_buffer_(new string()),
      last_output_buffer_(new string()),
      current_config_request_buffer_(new string()),
      last_config_request_buffer_(new string())
{
    logging_categories_.clear();
    logging_categories_.insert("FMI");
    logging_categories_.insert("OSMP");
    logging_categories_.insert("OSI");
}

fmi2Status OSMPCameraSensor::SetDebugLogging(fmi2Boolean thelogging_on, size_t n_categories, const fmi2String categories[])
{
    FmiVerboseLog("fmi2SetDebugLogging(%s)", thelogging_on != 0 ? "true" : "false");
    logging_on_ = thelogging_on != 0;
    if ((categories != nullptr) && (n_categories > 0))
    {
        logging_categories_.clear();
        for (size_t i = 0; i < n_categories; i++)
        {
            if (0 == strcmp(categories[i], "FMI"))
            {
                logging_categories_.insert("FMI");
            }
            else if (0 == strcmp(categories[i], "OSMP"))
            {
                logging_categories_.insert("OSMP");
            }
            else if (0 == strcmp(categories[i], "OSI"))
            {
                logging_categories_.insert("OSI");
            }
        }
    }
    else
    {
        logging_categories_.clear();
        logging_categories_.insert("FMI");
        logging_categories_.insert("OSMP");
        logging_categories_.insert("OSI");
    }
    return fmi2OK;
}

fmi2Component OSMPCameraSensor::Instantiate(fmi2String instance_name,
                                            fmi2Type fmu_type,
                                            fmi2String fmu_guid,
                                            fmi2String fmu_resource_location,
                                            const fmi2CallbackFunctions* functions,
                                            fmi2Boolean visible,
                                            fmi2Boolean logging_on)
{
    auto* myc = new OSMPCameraSensor(instance_name, fmu_type, fmu_guid, fmu_resource_location, functions, visible, logging_on);

    if (myc == nullptr)
    {
        FmiVerboseLogGlobal(R"(fmi2Instantiate("%s",%d,"%s","%s","%s",%d,%d) = NULL (alloc failure))",
                            instance_name,
                            fmu_type,
                            fmu_guid,
                            (fmu_resource_location != nullptr) ? fmu_resource_location : "<NULL>",
                            "FUNCTIONS",
                            visible,
                            logging_on);
        return nullptr;
    }

    if (myc->DoInit() != fmi2OK)
    {
        FmiVerboseLogGlobal(R"(fmi2Instantiate("%s",%d,"%s","%s","%s",%d,%d) = NULL (DoInit failure))",
                            instance_name,
                            fmu_type,
                            fmu_guid,
                            (fmu_resource_location != nullptr) ? fmu_resource_location : "<NULL>",
                            "FUNCTIONS",
                            visible,
                            logging_on);
        delete myc;
        return nullptr;
    }
    FmiVerboseLogGlobal(R"(fmi2Instantiate("%s",%d,"%s","%s","%s",%d,%d) = %p)",
                        instance_name,
                        fmu_type,
                        fmu_guid,
                        (fmu_resource_location != nullptr) ? fmu_resource_location : "<NULL>",
                        "FUNCTIONS",
                        visible,
                        logging_on,
                        myc);
    return (fmi2Component)myc;
}

fmi2Status OSMPCameraSensor::SetupExperiment(fmi2Boolean tolerance_defined, fmi2Real tolerance, fmi2Real start_time, fmi2Boolean stop_time_defined, fmi2Real stop_time)
{
    FmiVerboseLog("fmi2SetupExperiment(%d,%g,%g,%d,%g)", tolerance_defined, tolerance, start_time, stop_time_defined, stop_time);
    return DoStart(tolerance_defined, tolerance, start_time, stop_time_defined, stop_time);
}

fmi2Status OSMPCameraSensor::EnterInitializationMode()
{
    FmiVerboseLog("fmi2EnterInitializationMode()");
    return DoEnterInitializationMode();
}

fmi2Status OSMPCameraSensor::ExitInitializationMode()
{
    FmiVerboseLog("fmi2ExitInitializationMode()");
    simulation_started_ = true;
    return DoExitInitializationMode();
}

fmi2Status OSMPCameraSensor::DoStep(fmi2Real current_communication_point, fmi2Real communication_step_size, fmi2Boolean no_set_fmu_state_prior_to_current_pointfmi_2_component)
{
    FmiVerboseLog("fmi2DoStep(%g,%g,%d)", current_communication_point, communication_step_size, no_set_fmu_state_prior_to_current_pointfmi_2_component);
    return DoCalc(current_communication_point, communication_step_size, no_set_fmu_state_prior_to_current_pointfmi_2_component);
}

fmi2Status OSMPCameraSensor::Terminate()
{
    FmiVerboseLog("fmi2Terminate()");
    return DoTerm();
}

fmi2Status OSMPCameraSensor::Reset()
{
    FmiVerboseLog("fmi2Reset()");

    DoFree();
    simulation_started_ = false;
    return DoInit();
}

void OSMPCameraSensor::FreeInstance()
{
    FmiVerboseLog("fmi2FreeInstance()");
    DoFree();
}

fmi2Status OSMPCameraSensor::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    FmiVerboseLog("fmi2GetReal(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_REAL_VARS)
        {
            value[i] = real_vars_[vr[i]];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    FmiVerboseLog("fmi2GetInteger(...)");
    bool need_refresh = !simulation_started_;
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_INTEGER_VARS)
        {
            if (need_refresh && (vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX ||
                                 vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX))
            {
                RefreshFmiSensorViewConfigRequest();
                need_refresh = false;
            }
            value[i] = integer_vars_[vr[i]];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    FmiVerboseLog("fmi2GetBoolean(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_BOOLEAN_VARS)
        {
            value[i] = boolean_vars_[vr[i]];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    FmiVerboseLog("fmi2GetString(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_STRING_VARS)
        {
            value[i] = string_vars_[vr[i]].c_str();
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    FmiVerboseLog("fmi2SetReal(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_REAL_VARS)
        {
            real_vars_[vr[i]] = value[i];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    FmiVerboseLog("fmi2SetInteger(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_INTEGER_VARS)
        {
            integer_vars_[vr[i]] = value[i];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    FmiVerboseLog("fmi2SetBoolean(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_BOOLEAN_VARS)
        {
            boolean_vars_[vr[i]] = value[i];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

fmi2Status OSMPCameraSensor::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    FmiVerboseLog("fmi2SetString(...)");
    for (size_t i = 0; i < nvr; i++)
    {
        if (vr[i] < FMI_STRING_VARS)
        {
            string_vars_[vr[i]] = value[i];
        }
        else
        {
            return fmi2Error;
        }
    }
    return fmi2OK;
}

/*
 * FMI 2.0 Co-Simulation Interface API
 */

extern "C" {

FMI2_Export const char* fmi2GetTypesPlatform()
{
    return fmi2TypesPlatform;
}

FMI2_Export const char* fmi2GetVersion()
{
    return fmi2Version;
}

FMI2_Export fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean logging_on, size_t n_categories, const fmi2String categories[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->SetDebugLogging(logging_on, n_categories, categories);
}

/*
 * Functions for Co-Simulation
 */
FMI2_Export fmi2Component fmi2Instantiate(fmi2String instance_name,
                                          fmi2Type fmu_type,
                                          fmi2String fmu_guid,
                                          fmi2String fmu_resource_location,
                                          const fmi2CallbackFunctions* functions,
                                          fmi2Boolean visible,
                                          fmi2Boolean logging_on)
{
    return OSMPCameraSensor::Instantiate(instance_name, fmu_type, fmu_guid, fmu_resource_location, functions, visible, logging_on);
}

FMI2_Export fmi2Status
fmi2SetupExperiment(fmi2Component c, fmi2Boolean tolerance_defined, fmi2Real tolerance, fmi2Real start_time, fmi2Boolean stop_time_defined, fmi2Real stop_time)
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->SetupExperiment(tolerance_defined, tolerance, start_time, stop_time_defined, stop_time);
}

FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->EnterInitializationMode();
}

FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->ExitInitializationMode();
}

FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
                                  fmi2Real current_communication_point,
                                  fmi2Real communication_step_size,
                                  fmi2Boolean no_set_fmu_state_prior_to_current_pointfmi2_component)
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->DoStep(current_communication_point, communication_step_size, no_set_fmu_state_prior_to_current_pointfmi2_component);
}

FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->Terminate();
}

FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->Reset();
}

FMI2_Export void fmi2FreeInstance(fmi2Component c)
{
    auto* myc = (OSMPCameraSensor*)c;
    myc->FreeInstance();
    delete myc;
}

/*
 * Data Exchange Functions
 */
FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->GetReal(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->GetInteger(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->GetBoolean(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->GetString(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->SetReal(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->SetInteger(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->SetBoolean(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
    auto* myc = (OSMPCameraSensor*)c;
    return myc->SetString(vr, nvr, value);
}

/*
 * Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
 */
FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* fmu_state)
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate fmu_state)
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* fmu_state)
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate fmu_state, size_t* size)
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate fmu_state, fmi2Byte serialized_state[], size_t size)
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serialized_state[], size_t size, fmi2FMUstate* fmu_state)
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2GetDirectionalDerivative(fmi2Component c,
                                                    const fmi2ValueReference v_unknown_ref[],
                                                    size_t n_unknown,
                                                    const fmi2ValueReference v_known_ref[],
                                                    size_t n_known,
                                                    const fmi2Real dv_known[],
                                                    fmi2Real dv_unknown[])
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SetRealInputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer order[], const fmi2Real value[])
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer order[], fmi2Real value[])
{
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2CancelStep(fmi2Component c)
{
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status* value)
{
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real* value)
{
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer* value)
{
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean* value)
{
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String* value)
{
    return fmi2Discard;
}
}
