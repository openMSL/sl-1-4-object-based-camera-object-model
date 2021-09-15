/*
* PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
*
* (C) 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
*
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
*
* Copyright 2020 Robert Bosch GmbH
*/

#include "OSMPCameraSensor.h"


/*
* Debug Breaks
*
* If you define DEBUG_BREAKS the FMU will automatically break
* into an attached Debugger on all major computation functions.
* Note that the FMU is likely to break all environments if no
* Debugger is actually attached when the breaks are triggered.
*/
#define PI 3.14159265
#define camera_FOV 50
#define camera_range 150
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

#include <iostream>
#include <string>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <vector>

using namespace std;
std::vector<object_info> object_history_vector;

//Check if necessary 
#ifdef PRIVATE_LOG_PATH
ofstream COSMPCameraSensor::private_log_file;
#endif
//

/*
* ProtocolBuffer Accessors
*/

void* decode_integer_to_pointer(fmi2Integer hi, fmi2Integer lo)
{
#if PTRDIFF_MAX == INT64_MAX
	union addrconv {
		struct {
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

void encode_pointer_to_integer(const void* ptr, fmi2Integer& hi, fmi2Integer& lo)
{
#if PTRDIFF_MAX == INT64_MAX
	union addrconv {
		struct {
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

bool COSMPCameraSensor::get_fmi_sensor_view_config(osi3::SensorViewConfiguration& data)
{
	if (integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX] > 0) {
		void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX]);
		normal_log("OSMP", "Got %08X %08X, reading from %p ...", integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX], buffer);
		data.ParseFromArray(buffer, integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX]);
		return true;
	}
	else {
		return false;
	}
}

void COSMPCameraSensor::set_fmi_sensor_view_config_request(const osi3::SensorViewConfiguration& data)
{
	data.SerializeToString(&currentConfigRequestBuffer);
	encode_pointer_to_integer(currentConfigRequestBuffer.data(), integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX]);
	integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX] = (fmi2Integer)currentConfigRequestBuffer.length();
	normal_log("OSMP", "Providing %08X %08X, writing from %p ...", integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX], currentConfigRequestBuffer.data());
	swap(currentConfigRequestBuffer, lastConfigRequestBuffer);
}

void COSMPCameraSensor::reset_fmi_sensor_view_config_request()
{
	integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX] = 0;
	integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX] = 0;
	integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX] = 0;
}

bool COSMPCameraSensor::get_fmi_sensor_view_in(osi3::SensorView& data)
{
	if (integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX] > 0) {
		void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX]);
		normal_log("OSMP", "Got %08X %08X, reading from %p ...", integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX], buffer);
		data.ParseFromArray(buffer, integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX]);
		return true;
	}
	else {
		return false;
	}
}

// for VTD v2.2 only
bool COSMPCameraSensor::get_fmi_sensor_data_in(osi3::SensorData& data)
{
	if (integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX] > 0) {
		void* buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX]);
		normal_log("OSMP", "Got %08X %08X, reading from %p ...", integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX], buffer);
		data.ParseFromArray(buffer, integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX]);
		return true;
	}
	else {
		return false;
	}
}

void COSMPCameraSensor::set_fmi_sensor_data_out(const osi3::SensorData& data)
{
	data.SerializeToString(&currentOutputBuffer);
	encode_pointer_to_integer(currentOutputBuffer.data(), integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]);
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = (fmi2Integer)currentOutputBuffer.length();
	normal_log("OSMP", "Providing %08X %08X, writing from %p ...", integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX], integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX], currentOutputBuffer.data());
	swap(currentOutputBuffer, lastOutputBuffer);
}

void COSMPCameraSensor::reset_fmi_sensor_data_out()
{
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = 0;
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX] = 0;
	integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX] = 0;
}

void COSMPCameraSensor::refresh_fmi_sensor_view_config_request()
{
	osi3::SensorViewConfiguration config;
	if (get_fmi_sensor_view_config(config))
		set_fmi_sensor_view_config_request(config);
	else {
		config.Clear();
		config.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));
		config.set_field_of_view_horizontal(3.14);
		config.set_field_of_view_vertical(3.14);
		config.set_range(fmi_nominal_range()*1.1);
		config.mutable_update_cycle_time()->set_seconds(0);
		config.mutable_update_cycle_time()->set_nanos(20000000);
		config.mutable_update_cycle_offset()->Clear();
		osi3::GenericSensorViewConfiguration* generic = config.add_generic_sensor_view_configuration();
		generic->set_field_of_view_horizontal(3.14);
		generic->set_field_of_view_vertical(3.14);
		set_fmi_sensor_view_config_request(config);
	}
}

/*
* Actual Core Content
*/

fmi2Status COSMPCameraSensor::doInit()
{
	DEBUGBREAK();

	/* Booleans */
	for (int i = 0; i < FMI_BOOLEAN_VARS; i++)
		boolean_vars[i] = fmi2False;

	/* Integers */
	for (int i = 0; i < FMI_INTEGER_VARS; i++)
		integer_vars[i] = 0;

	/* Reals */
	for (int i = 0; i < FMI_REAL_VARS; i++)
		real_vars[i] = 0.0;

	/* Strings */
	for (int i = 0; i < FMI_STRING_VARS; i++)
		string_vars[i] = "";

	set_fmi_nominal_range(135.0);
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
	DEBUGBREAK();

	return fmi2OK;
}

fmi2Status COSMPCameraSensor::doEnterInitializationMode()
{
	DEBUGBREAK();

	return fmi2OK;
}

fmi2Status COSMPCameraSensor::doExitInitializationMode()
{
	DEBUGBREAK();

	osi3::SensorViewConfiguration config;
	if (!get_fmi_sensor_view_config(config))
		normal_log("OSI", "Received no valid SensorViewConfiguration from Simulation Environment, assuming everything checks out.");
	else {
		normal_log("OSI", "Received SensorViewConfiguration for Sensor Id %llu", config.sensor_id().value());
		normal_log("OSI", "SVC Ground Truth FoV Horizontal %f, FoV Vertical %f, Range %f", config.field_of_view_horizontal(), config.field_of_view_vertical(), config.range());
		normal_log("OSI", "SVC Mounting Position: (%f, %f, %f)", config.mounting_position().position().x(), config.mounting_position().position().y(), config.mounting_position().position().z());
		normal_log("OSI", "SVC Mounting Orientation: (%f, %f, %f)", config.mounting_position().orientation().roll(), config.mounting_position().orientation().pitch(), config.mounting_position().orientation().yaw());
	}

	return fmi2OK;
}
//****** Arcus tangens function taking into account the sign of numerator and denominator ****** /
double arcTan(double num, double denom)
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
void EulerWinkel(double egoyaw, double egopitch, double egoroll, double objectyaw, double objectpitch, double objectroll, double &R1, double &R2, double &R3)
{
	double matrixobject[3][3];
	double matrixego[3][3];
	double MR[3][3];
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

	MR[0][0] = matrixego[0][0] * matrixobject[0][0] + matrixego[0][1] * matrixobject[1][0] + matrixego[0][2] * matrixobject[2][0];
	MR[0][1] = matrixego[0][0] * matrixobject[0][1] + matrixego[0][1] * matrixobject[1][1] + matrixego[0][2] * matrixobject[2][1];
	MR[0][2] = matrixego[0][0] * matrixobject[0][2] + matrixego[0][1] * matrixobject[1][2] + matrixego[0][2] * matrixobject[2][2];

	MR[1][0] = matrixego[1][0] * matrixobject[0][0] + matrixego[1][1] * matrixobject[1][0] + matrixego[1][2] * matrixobject[2][0];
	MR[1][1] = matrixego[1][0] * matrixobject[0][1] + matrixego[1][1] * matrixobject[1][1] + matrixego[1][2] * matrixobject[2][1];
	MR[1][2] = matrixego[1][0] * matrixobject[0][2] + matrixego[1][1] * matrixobject[1][2] + matrixego[1][2] * matrixobject[2][2];

	MR[2][0] = matrixego[2][0] * matrixobject[0][0] + matrixego[2][1] * matrixobject[1][0] + matrixego[2][2] * matrixobject[2][0];
	MR[2][1] = matrixego[2][0] * matrixobject[0][1] + matrixego[2][1] * matrixobject[1][1] + matrixego[2][2] * matrixobject[2][1];
	MR[2][2] = matrixego[2][0] * matrixobject[0][2] + matrixego[2][1] * matrixobject[1][2] + matrixego[2][2] * matrixobject[2][2];

	/*normal_log("osi", "matrixego: %f,%f,%f", MR[1][1]);
	normal_log("osi", "matrixego: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixego: %f,%f,%f", rvxx,rvxy,rvxz);

	normal_log("osi", "matrixobject: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixobject: %f,%f,%f", rvxx,rvxy,rvxz);
	normal_log("osi", "matrixobject: %f,%f,%f", rvxx,rvxy,rvxz);*/

	//R1 = MR[0][0];
	//R2 = MR[0][1];
	//R3 = MR[0][2];



	R1 = arcTan(-MR[0][2], sqrt(MR[0][0] * MR[0][0] + MR[1][0] * MR[1][0]));
	R2 = arcTan(MR[0][1] / cos(R1), MR[0][0] / cos(R1));
	R3 = arcTan(MR[1][2] / cos(R1), MR[2][2] / cos(R1));
}

/****** Rotation from environment to object coordinate system (yaw, pitch, roll = orientation of car in environment system) ******/
void rot2veh(double x, double y, double z, double yaw, double pitch, double roll, double &rx, double &ry, double &rz)
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
void rot2env(double x, double y, double z, double yaw, double pitch, double roll, double &rx, double &ry, double &rz)	// Koordinatentransformation vom koerperfesten ins raumfeste Koordinatensystem
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
void crossProduct(double vect_A[], double vect_B[], double cross_P[])

{

	cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
	cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
	cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}
void CalculateKoordinate(double a1, double a2, double a3, double d1, double d2, double d3, double b1, double b2, double b3, double &Koord)
{
	double t = -1 / (a1 * a1 + a2 * a2 + a3 * a3)*(d1 * a1 + d2 * a2 + d3 * a3);
	double F[2];
	F[0] = d1 + t * a1;
	F[1] = d2 + t * a2;
	F[2] = d3 + t * a3;
	double LZ = sqrt((d1 - F[0])*(d1 - F[0]) + (d2 - F[1])*(d2 - F[1]) + (d3 - F[2])*(d3 - F[2]));
	double h0 = d1 - F[0];
	double h1 = d2 - F[1];
	double h2 = d3 - F[2];
	//double Koordinate = 0.0;
	double zaehlerLot = b1 * h0 + b2 * h1 + b3 * h2;
	double nennerLot = LZ * sqrt(b1 * b1 + b2 * b2 + b3 * b3);
	double	beta = acos(round(zaehlerLot / nennerLot));
	double Koordinate;
	if (beta > 90 / 180 * PI) { Koordinate = -abs(LZ); }
	else { Koordinate = abs(LZ); }
	//Koordinate = 3;
	//return Koordinate;
	Koord = Koordinate;

}

void CalKoordNew(double trans_x, double trans_y, double trans_z, double ego_yaw, double ego_pitch, double ego_roll, double &xn, double &yn, double &zn)
{

	double rvxx = 0; double rvxy = 0; double rvxz = 0;
	double rvyx = 0; double rvyy = 0; double rvyz = 0;
	double rvzx = 0; double rvzy = 0; double rvzz = 0;

	rot2env(1, 0, 0, ego_yaw, ego_pitch, ego_roll, rvxx, rvxy, rvxz);
	rot2env(0, 1, 0, ego_yaw, ego_pitch, ego_roll, rvyx, rvyy, rvyz);
	rot2env(0, 0, 1, ego_yaw, ego_pitch, ego_roll, rvzx, rvzy, rvzz);
	//normal_log("osi", "rvx: %f,%f,%f", rvxx,rvxy,rvxz);
	//normal_log("osi", "rvy: %f,%f,%f", rvyx,rvyy,rvyz);
	//normal_log("osi", "rvz: %f,%f,%f", rvzx,rvzy,rvzz);

	double vectrvx[] = { rvxx, rvxy, rvxz }; double vectrvy[] = { rvyx, rvyy, rvyz }; double vectrvz[] = { rvzx, rvzy, rvzz };
	double vectexy[3]; double vectexz[3]; double vecteyz[3];
	crossProduct(vectrvx, vectrvy, vectexy); crossProduct(vectrvx, vectrvz, vectexz); crossProduct(vectrvy, vectrvz, vecteyz);
	//normal_log("osi", "cross product exy: %f,%f,%f", vectexy[0], vectexy[1], vectexy[2]);
	//normal_log("osi", "cross product exz: %f,%f,%f", vectexz[0], vectexz[1], vectexz[2]);
	//normal_log("osi", "cross product eyz: %f,%f,%f", vecteyz[0], vecteyz[1], vecteyz[2]);
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

	double nenner1 = g1 * r1 + g2 * r2 + g3 * r3;
	double nenner2 = sqrt(g1*g1 + g2 * g2 + g3 * g3)*sqrt(r1*r1 + r2 * r2 + r3 * r3);
	double angle = acos(nenner1 / nenner2) * 180 / PI;
	return angle;
}

int get_object_info_idx(std::vector<object_info> search_vector, int search_id) {
	int idx = 0;
	for (auto it = search_vector.begin(); it < search_vector.end(); it++) {
		if (it->id == search_id) {
			return idx;
		}
		idx++;
	}
	return -1;
}

double get_abs_velocity(const osi3::Vector3d& velocity_3d) {
	return sqrt(pow(velocity_3d.x(), 2) + pow(velocity_3d.y(), 2) + pow(velocity_3d.z(), 2));
}


void update_object_history_vector(object_info& current_object_history, const osi3::SensorView& input_sensor_view, int obj_idx, bool moving) {
	int current_object_idx;
	if (moving) {
		current_object_idx = get_object_info_idx(object_history_vector, input_sensor_view.global_ground_truth().moving_object(obj_idx).id().value());
	}
	else {
		current_object_idx = get_object_info_idx(object_history_vector, input_sensor_view.global_ground_truth().stationary_object(obj_idx).id().value());
	}
	if (current_object_idx != -1) {
		object_history_vector.at(current_object_idx).age++;
		if (moving) {
			if (get_abs_velocity(input_sensor_view.global_ground_truth().moving_object(obj_idx).base().velocity()) > 0.01) {
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
			if (get_abs_velocity(input_sensor_view.global_ground_truth().moving_object(obj_idx).base().velocity()) > 0.01) {
				current_object_history.movement_state = 1;
			}
			else {
				current_object_history.movement_state = 0;
			}
		}
		else {
			current_object_history.movement_state = 0;
		}
		object_history_vector.push_back(current_object_history);
	}
}



/*Main Function to calculate phenomenological effects; Input=GT-Objects, Output=Detected Moving Objects*/

fmi2Status COSMPCameraSensor::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
	DEBUGBREAK();

	osi3::SensorData currentOut;
	double time = currentCommunicationPoint + communicationStepSize;
	normal_log("OSI", "Calculating Camera Sensor at %f for %f (step size %f)", currentCommunicationPoint, time, communicationStepSize);
#ifndef COMPILE_VTD_2_2
	// OSI standard..
	osi3::SensorView currentIn;
	if (get_fmi_sensor_view_in(currentIn)) {
		osi3::SensorView& currentViewIn = currentIn;
#else
	// VTD v2.2 receives SensorData..
	osi3::SensorData currentIn;
	if (get_fmi_sensor_data_in(currentIn)) {
		if (!currentIn.sensor_view_size()) {
			normal_log("OSI", "No valid input for SensorData->SensorView.");
			return fmi2Fatal;
		}
		const osi3::SensorView& currentViewIn = currentIn.sensor_view(0);
#endif
		size_t nof_mov_obj = currentViewIn.global_ground_truth().moving_object().size(); // number of vehicles (including ego vehicle)
		size_t nof_stat_obj = currentViewIn.global_ground_truth().stationary_object().size(); // number of vehicles (including ego vehicle)
		size_t nof_trafLights_obj = currentViewIn.global_ground_truth().traffic_light().size();   // number of traffic lights 
		size_t nof_trafSign_obj = currentViewIn.global_ground_truth().traffic_sign().size();      // number of traffic signs 

		normal_log("OSI", "Number of moving objects: %llu", nof_mov_obj);
		normal_log("OSI", "Number of stationary objects: %llu", nof_stat_obj);
		normal_log("OSI", "Number of traffic lights: %llu", nof_trafLights_obj);
		normal_log("OSI", "Number of traffic signs: %llu", nof_trafSign_obj);

		//Center of the bounding box in environment coordinates 
		//Position and Orientation 
		double ego_World_x = 0, ego_World_y = 0, ego_World_z = 0, ego_yaw = 0, ego_pitch = 0, ego_roll = 0;

		double ego_vel_x = 0; double ego_vel_y = 0; double ego_vel_z = 0;//ego velocity 


		double origin_Host_x = 0, origin_Host_y = 0, origin_Host_z = 0; //Position of the host vehicle Ursprung in host vehicle coordinates
		//Position of the host vehicle Ursprung in environment coordinates
		double origin_World_x = 0, origin_World_y = 0, origin_World_z = 0;

		double bbctr_Host_x = 0, bbctr_Host_y = 0, bbctr_Host_z = 0;
		double origin_rot_x = 0, origin_rot_y = 0, origin_rot_z = 0;
		//Position of the sensor in environment coordinates 
		double sens_World_x = 0, sens_World_y = 0, sens_World_z = 0;
		//Camera view direction after calculation of all orientation - 
		double sensSV_x = 0, sensSV_y = 0, sensSV_z = 0;
		std::vector<int>   masked(nof_mov_obj);  	// define vector with length nof_obj and initialize it to 0


		//Sensor mounting position and orientation (in host vehicle frame with reference to the bbcenter_rear)
		//Todo Mounting Position über CameraSensorViewConfig
		double mpos_x = currentViewIn.mounting_position().position().x();        double mpos_y = currentViewIn.mounting_position().position().y();            double mpos_z = currentViewIn.mounting_position().position().z();
		double mpos_yaw = currentViewIn.mounting_position().orientation().yaw(); double mpos_pitch = currentViewIn.mounting_position().orientation().pitch(); double mpos_roll = currentViewIn.mounting_position().orientation().roll();

		//double mpos_x = 0;        double mpos_y = 0;            double mpos_z = 0;
		//double mpos_yaw = 0; double mpos_pitch = 0; double mpos_roll = 0;
		double mpos_rot_x = 0; double mpos_rot_y = 0; double mpos_rot_z = 0; //sensor position in ego coordinate system after ego rotation orientation  

		normal_log("DEBUG", "Sensor mounting postion and orientation: x %.2f, y %.2f, z %.2f, yaw %.2f, pitch %.2f, roll %.2f", mpos_x, mpos_y, mpos_z, mpos_yaw, mpos_pitch, mpos_roll);


		osi3::Identifier ego_id = currentViewIn.global_ground_truth().host_vehicle_id();
		normal_log("OSI", "Looking for EgoVehicle with ID: %llu", ego_id.value());

		/*Calculating the position of the ego vehicle*/
		/*Calculating the position of the Sensor position in environment coordinates*/
		/*Calculating the camera view direction*/

		for_each(currentViewIn.global_ground_truth().moving_object().begin(), currentViewIn.global_ground_truth().moving_object().end(),
			[this, ego_id, &bbctr_Host_x, &bbctr_Host_y, &bbctr_Host_z, &ego_World_x, &ego_World_y, &ego_World_z, &ego_yaw, &ego_pitch, &ego_roll, &ego_vel_x, &ego_vel_y, &ego_vel_z, &origin_World_x, &origin_World_y, &origin_World_z, &origin_rot_x, &origin_rot_y, &origin_rot_z, &origin_Host_x, &origin_Host_y, &origin_Host_z, &mpos_x, &mpos_y, &mpos_z, &mpos_rot_x, &mpos_rot_y, &mpos_rot_z, &mpos_yaw, &mpos_pitch, &mpos_roll, &sens_World_x, &sens_World_y, &sens_World_z, &sensSV_x, &sensSV_y, &sensSV_z](const osi3::MovingObject& obj) {
			normal_log("OSI", "MovingObject with ID %llu is EgoVehicle: %d", obj.id().value(), obj.id().value() == ego_id.value());
			if (obj.id().value() == ego_id.value()) {
				normal_log("OSI", "Found EgoVehicle with ID: %llu", obj.id().value());

				ego_World_x = obj.base().position().x(); ego_World_y = obj.base().position().y(); ego_World_z = obj.base().position().z();
				ego_yaw = obj.base().orientation().yaw(); ego_pitch = obj.base().orientation().pitch();	ego_roll = obj.base().orientation().roll();
				//Test
				//ego_World_x = 1; ego_World_y = 1; ego_World_z = 1; ego_yaw = PI; ego_pitch = 0; ego_roll = 0;

				normal_log("OSI", "Current EGO position and orientation: %f,%f,%f,%f,%f,%f", ego_World_x, ego_World_y, ego_World_z, ego_yaw, ego_pitch, ego_roll);
				ego_vel_x = obj.base().velocity().x(); ego_vel_y = obj.base().velocity().y(); ego_vel_z = obj.base().velocity().z();


				//Position of the ego origin in host vehicle coordinates  (bbcenter_to_rear)
				bbctr_Host_x = obj.vehicle_attributes().bbcenter_to_rear().x(); bbctr_Host_y = obj.vehicle_attributes().bbcenter_to_rear().y(); bbctr_Host_z = obj.vehicle_attributes().bbcenter_to_rear().z();
				//Test
				//bbctr_Host_x = -2; bbctr_Host_y = -0.5; bbctr_Host_z = 0;
				double h1 = 0, h2 = 0, h3 = 0;
				//Position of the ego origin in world coordinates after rotation  
				normal_log("OSI", "Current EGO bbcenter to rear vector: %f,%f,%f", bbctr_Host_x, bbctr_Host_y, bbctr_Host_z);
				rot2env(bbctr_Host_x, bbctr_Host_y, bbctr_Host_z, ego_yaw, ego_pitch, ego_roll, origin_rot_x, origin_rot_y, origin_rot_z); //rotate into coordinate system of environment
				origin_World_x = origin_rot_x + ego_World_x;
				origin_World_y = origin_rot_y + ego_World_y;
				origin_World_z = origin_rot_z + ego_World_z;
				normal_log("OSI", "Current EGO Origin Position in environment coordinates: %f,%f,%f", origin_World_x, origin_World_y, origin_World_z);

				//Test zurück
				rot2veh(origin_World_x - ego_World_x, origin_World_y - ego_World_y, origin_World_z - ego_World_z, ego_yaw, ego_pitch, ego_roll, h1, h2, h3);
				normal_log("OSI", "Current Host ORigin in Host coordinates: %f,%f,%f", h1, h2, h3);
				//Sensor -Calculations
				//Angenommen wird hier, dass die nominell die Kamera in Richtung der positiven x-Achse guckt. Also in Richtung (mposx+1,0,0)
				//Dieser Richtungsvektor muss rotiert werden mit der Roll, Pitch, Yaw der Camera 
				//Da dieser Punkt jedoch auch noch rotiert werden muss mit der Rotationsmatrix rot2env, da das Auto mit Roll Pitch und Yaw rotiert ist ergibt sich eine andere Blickrichtung.
				//In diesem Fall berechnet sich die Matrix aus der Rotationsmatrix * (1,0,0) 
				//Dann ergibt sich der Winkel zwischen dem Vektor ! Objekt - Sensor position und dem Vektor Sensor Richtungsposition und Sensor Ansatzpunkt und dem Arcos des Skalarprodukts 
				double sens_EGO_x = origin_Host_x + mpos_x;	// sensor x-position in ego coordinate system before ego-orientation rotation
				double sens_EGO_y = origin_Host_y + mpos_y;	// sensor y-position in ego coordinate system before ego-orientation rotation
				double sens_EGO_z = origin_Host_z + mpos_z;	// sensor z-position in ego coordinate system before ego-orientation rotation



				rot2env(sens_EGO_x, sens_EGO_y, sens_EGO_z, ego_yaw, ego_pitch, ego_roll, mpos_rot_x, mpos_rot_y, mpos_rot_z);
				//Position of the sensor in environment coordinates 
				sens_World_x = ego_World_x + mpos_rot_x;
				sens_World_y = ego_World_y + mpos_rot_y;
				sens_World_z = ego_World_z + mpos_rot_z;
				normal_log("OSI", "Current Sensor Coordinates in host coordinate system: x %f, y %f, z %f", sens_EGO_x, sens_EGO_y, sens_EGO_z);
				normal_log("OSI", "Current Sensor Coordinates in environment coordinate system: x %f, y %f, z %f", sens_World_x, sens_World_y, sens_World_z);

				/// Camera view-direction 
				//Alt
				//sensSV_x = sens_EGO_x+1;	// Sichtvektor sensor x in ego coordinate system before ego-orientation rotation
				//sensSV_y = sens_EGO_y;	// sensor y in ego coordinate system before ego-orientation rotation
				//sensSV_z = sens_EGO_z;	// sensor z in ego coordinate system before ego-orientation rotation
				//Alt zuende 
				//With sensor-rotation 
				double sensSV_mprotx = 0; double sensSV_mproty = 0; double sensSV_mprotz = 0;
				// Hier Änderungen 1. Version ist falsch, da Rotation um den Fixpunkt der Sensor Mounting Position ! 
				//Alt 
				//rot2env(sensSV_x, sensSV_y, sensSV_z,mpos_yaw, mpos_pitch, mpos_roll, sensSV_mprotx, sensSV_mproty, sensSV_mprotz);
				//Alt zuende 
				// SV (1,0,0)
				//Neu 
				rot2env(1, 0, 0, mpos_yaw, mpos_pitch, mpos_roll, sensSV_mprotx, sensSV_mproty, sensSV_mprotz);
				//Neu zuende 
				//Translation um den Sensor-Mounting Position relativ zur Mitte Bounding Box 
				//Neu
				sensSV_x = sens_EGO_x + sensSV_mprotx;	// Sichtvektor sensor x in ego coordinate system before ego-orientation rotation
				sensSV_y = sens_EGO_y + sensSV_mproty;	// sensor y in ego coordinate system before ego-orientation rotation
				sensSV_z = sens_EGO_z + sensSV_mprotz;	// sensor z in ego coordinate system before ego-orientation rotation
				//Neu zuende 
				double sensSV_rot_x = 0; double sensSV_rot_y = 0; double sensSV_rot_z = 0;
				//Alt
				//rot2env(sensSV_mprotx, sensSV_mproty, sensSV_mprotz, ego_yaw, ego_pitch, ego_roll, sensSV_rot_x, sensSV_rot_y, sensSV_rot_z);
				//Alt zuende 
				//Neu    - Richtung Weltkoordinaten 
				rot2env(sensSV_x, sensSV_y, sensSV_z, ego_yaw, ego_pitch, ego_roll, sensSV_rot_x, sensSV_rot_y, sensSV_rot_z);
				//Neu zuende 
				// Camera view-direction after all orientation calculations  in world coordinates 
				sensSV_x = ego_World_x + sensSV_rot_x;
				sensSV_y = ego_World_y + sensSV_rot_y;
				sensSV_z = ego_World_z + sensSV_rot_z;

				//normal_log("OSI", "EGO position and orientation: x %f, y %f, z %f,yaw %f, pitch %f, roll %f", ego_World_x, ego_World_y, ego_World_z, ego_yaw, ego_pitch, ego_roll);
				//normal_log("OSI", "Current BB Position rear: %f,%f,%f", origin_Host_x, origin_Host_y, origin_Host_z);
				//normal_log("OSI", " !!! New Current Sensor Position in environment coordinates: %f,%f,%f", sens_World_x, sens_World_y, sens_World_z);

				//normal_log("OSI", "!! New Current Sensor Position Richtungsvektor in environment coordinates: %f,%f,%f", sensSV_x, sensSV_y, sensSV_z);
				//normal_log("OSI", "Current Width, Height, Length: %f,%f,%f", obj.base().dimension().width(), obj.base().dimension().height(), obj.base().dimension().length());
			}
		});





		/* Clear Output */
		currentOut.Clear();
		currentOut.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));
		/* Adjust Timestamps and Ids */
		currentOut.mutable_timestamp()->set_seconds((long long int)floor(time));
		currentOut.mutable_timestamp()->set_nanos((int)((time - floor(time))*1000000000.0));
		/* Copy of SensorView */
		currentOut.add_sensor_view()->CopyFrom(currentViewIn);




		double actual_range = fmi_nominal_range()*1.1;
		/* Calculate vehicle FoV and distance to EGO (needed for occlusion) for all vehicles */

		std::vector<double> distance(nof_mov_obj);  // define vector with length nof_obj and initialize it to 0


		//double veh_rel2_x, veh_rel2_y, veh_rel2_z; //only for DEBUG!!!
		double trans_x, trans_y, trans_z;
		double veh_rel_x, veh_rel_y, veh_rel_z;
		std::vector<double> c1_x(nof_mov_obj);		// x coordinate of corner 1 (in sensor coord sys) (needed as reference corner for location distribution)	
		std::vector<double> c1_y(nof_mov_obj);		// y coordinate of corner 1 (in sensor coord sys)	
		std::vector<double> c1_z(nof_mov_obj);		// z coordinate of corner 1 (in sensor coord sys)	
		std::vector<double> phi_min(nof_mov_obj);   // define vector with length nof_obj and initialize it to 0
		std::vector<double> phi_max(nof_mov_obj);   // define vector with length nof_obj and initialize it to 0
		std::vector<double> phi(nof_mov_obj);
		std::vector<double> distM(nof_mov_obj);  // define vector with length nof_obj and initialize it to 0
		double rel_x, rel_y, rel_z;

		for (auto v_i = currentViewIn.global_ground_truth().moving_object().begin(); v_i != currentViewIn.global_ground_truth().moving_object().end(); ++v_i) {
			const osi3::MovingObject& veh = *v_i;
			auto const i = v_i - currentViewIn.global_ground_truth().moving_object().begin();
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




				// rotatePoint(veh_x, veh_y, veh_z, ego_yaw, ego_pitch, ego_roll, veh_rel_x, veh_rel_y, veh_rel_z); //rotate into coordinate system of ego vehicle
				rot2veh(veh_x, veh_y, veh_z, veh_yaw, veh_pitch, veh_roll, veh_rel_x, veh_rel_y, veh_rel_z); //rotate into coordinate system of vehicle    -------------!!!! MF nicht notwendig
				//rot2env(veh_rel_x, veh_rel_y, veh_rel_z, veh_yaw, veh_pitch, veh_roll, veh_rel2_x, veh_rel2_y, veh_rel2_z); //only for DEBUG!!!
				// normal_log("DEBUG","Vehicle %d coordinates in own coord sys: %.2f,%.2f,%.2f", i,veh_rel_x,veh_rel_y,veh_rel_z);
				double veh_width = veh.base().dimension().width();
				double veh_length = veh.base().dimension().length();
				// double veh_height = veh.base().dimension().height(); // should actually be done for 3D (box - 8 corners)
				double corner1_rel_x = veh_rel_x - veh_length / 2.; //this and the following could be done with x,y,z vectors instead
				double corner1_rel_y = veh_rel_y + veh_width / 2.;
				double corner1_rel_z = veh_rel_z;
				double corner2_rel_x = veh_rel_x + veh_length / 2.;
				double corner2_rel_y = veh_rel_y + veh_width / 2.;
				double corner2_rel_z = veh_rel_z;
				double corner3_rel_x = veh_rel_x - veh_length / 2.;
				double corner3_rel_y = veh_rel_y - veh_width / 2.;
				double corner3_rel_z = veh_rel_z;
				double corner4_rel_x = veh_rel_x + veh_length / 2.;
				double corner4_rel_y = veh_rel_y - veh_width / 2.;
				double corner4_rel_z = veh_rel_z;
				double corner1_x, corner1_y, corner1_z;
				double corner2_x, corner2_y, corner2_z;
				double corner3_x, corner3_y, corner3_z;
				double corner4_x, corner4_y, corner4_z;
				/*rotatePoint(corner1_rel_x, corner1_rel_y, corner1_rel_z, -ego_yaw, -ego_pitch, -ego_roll, corner1_x, corner1_y, corner1_z); //rotate into environment coordinate system
				rotatePoint(corner2_rel_x, corner2_rel_y, corner2_rel_z, -ego_yaw, -ego_pitch, -ego_roll, corner2_x, corner2_y, corner2_z); //rotate into environment coordinate system
				rotatePoint(corner3_rel_x, corner3_rel_y, corner3_rel_z, -ego_yaw, -ego_pitch, -ego_roll, corner3_x, corner3_y, corner3_z); //rotate into environment coordinate system
				rotatePoint(corner4_rel_x, corner4_rel_y, corner4_rel_z, -ego_yaw, -ego_pitch, -ego_roll, corner4_x, corner4_y, corner4_z); //rotate into environment coordinate system*/
				rot2env(corner1_rel_x, corner1_rel_y, corner1_rel_z, veh_yaw, veh_pitch, veh_roll, corner1_x, corner1_y, corner1_z); //rotate into environment coordinate system
				rot2env(corner2_rel_x, corner2_rel_y, corner2_rel_z, veh_yaw, veh_pitch, veh_roll, corner2_x, corner2_y, corner2_z); //rotate into environment coordinate system
				rot2env(corner3_rel_x, corner3_rel_y, corner3_rel_z, veh_yaw, veh_pitch, veh_roll, corner3_x, corner3_y, corner3_z); //rotate into environment coordinate system
				rot2env(corner4_rel_x, corner4_rel_y, corner4_rel_z, veh_yaw, veh_pitch, veh_roll, corner4_x, corner4_y, corner4_z); //rotate into environment coordinate system

				/* Calculate azimuth angles of corners */
				double trans_c1_x = corner1_x - sens_World_x;	// vector from sensor to corner 1
				double trans_c1_y = corner1_y - sens_World_y;
				double trans_c1_z = corner1_z - sens_World_z;
				double trans_c2_x = corner2_x - sens_World_x;	// vector from sensor to corner 2
				double trans_c2_y = corner2_y - sens_World_y;
				double trans_c2_z = corner2_z - sens_World_z;
				double trans_c3_x = corner3_x - sens_World_x;	// vector from sensor to corner 3
				double trans_c3_y = corner3_y - sens_World_y;
				double trans_c3_z = corner3_z - sens_World_z;
				double trans_c4_x = corner4_x - sens_World_x;	// vector from sensor to corner 4
				double trans_c4_y = corner4_y - sens_World_y;
				double trans_c4_z = corner4_z - sens_World_z;
				// normal_log("DEBUG", "Vehicle %d trans c1 coordinates: %.2f,%.2f,%.2f", i, trans_c1_x, trans_c1_y, trans_c1_z);
				// normal_log("DEBUG", "Vehicle %d veh_yaw, veh_pitch, veh_roll: %.2f,%.2f,%.2f", i, veh_yaw, veh_pitch, veh_roll);
				// normal_log("DEBUG", "Vehicle %d veh_x, veh_y, veh_z: %.2f,%.2f,%.2f", i, veh_x, veh_y, veh_z);
				// normal_log("DEBUG", "Vehicle %d veh_rel_x, veh_rel_y, veh_rel_z: %.2f,%.2f,%.2f", i, veh_rel_x, veh_rel_y, veh_rel_z);
				// normal_log("DEBUG", "Vehicle %d veh_rel2_x, veh_rel2_y, veh_rel2_z: %.2f,%.2f,%.2f", i, veh_rel2_x, veh_rel2_y, veh_rel2_z);
				// normal_log("DEBUG", "Vehicle %d sens_World_x, sens_World_y, sens_World_z: %.2f,%.2f,%.2f", i, sens_World_x, sens_World_y, sens_World_z);
				// normal_log("DEBUG", "Vehicle %d corner1_x, corner1_y, corner1_z: %.2f,%.2f,%.2f", i, corner1_x, corner1_y, corner1_z);
				double rel_c1_x, rel_c1_y, rel_c1_z;
				double rel_c2_x, rel_c2_y, rel_c2_z;
				double rel_c3_x, rel_c3_y, rel_c3_z;
				double rel_c4_x, rel_c4_y, rel_c4_z;
				rot2veh(trans_c1_x, trans_c1_y, trans_c1_z, ego_yaw, ego_pitch, ego_roll, rel_c1_x, rel_c1_y, rel_c1_z); //rotate into coordinate system of ego vehicle
				rot2veh(trans_c2_x, trans_c2_y, trans_c2_z, ego_yaw, ego_pitch, ego_roll, rel_c2_x, rel_c2_y, rel_c2_z); //rotate into coordinate system of ego vehicle
				rot2veh(trans_c3_x, trans_c3_y, trans_c3_z, ego_yaw, ego_pitch, ego_roll, rel_c3_x, rel_c3_y, rel_c3_z); //rotate into coordinate system of ego vehicle
				rot2veh(trans_c4_x, trans_c4_y, trans_c4_z, ego_yaw, ego_pitch, ego_roll, rel_c4_x, rel_c4_y, rel_c4_z); //rotate into coordinate system of ego vehicle
				c1_x[i] = rel_c1_x;
				c1_y[i] = rel_c1_y;
				c1_z[i] = rel_c1_z;
				double phi_c1 = arcTan(rel_c1_y, rel_c1_x); //also valid for x<0 (rear cars)
				double phi_c2 = arcTan(rel_c2_y, rel_c2_x);
				double phi_c3 = arcTan(rel_c3_y, rel_c3_x);
				double phi_c4 = arcTan(rel_c4_y, rel_c4_x);
				// normal_log("DEBUG","Vehicle %d corner 1 coordinates: %.2f,%.2f,%.2f", i,corner1_x, corner1_y, corner1_z);
				// normal_log("DEBUG","Vehicle %d corner 2 coordinates: %.2f,%.2f,%.2f", i,corner2_x, corner2_y, corner2_z);
				// normal_log("DEBUG","Vehicle %d corner 3 coordinates: %.2f,%.2f,%.2f", i,corner3_x, corner3_y, corner3_z);
				// normal_log("DEBUG","Vehicle %d corner 4 coordinates: %.2f,%.2f,%.2f", i,corner4_x, corner4_y, corner4_z);
				//normal_log("LOC", "Vehicle %d corner coordinates in sensor sys: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f", i, rel_c1_x, rel_c1_y, rel_c1_z, rel_c2_x, rel_c2_y, rel_c2_z, rel_c3_x, rel_c3_y, rel_c3_z, rel_c4_x, rel_c4_y, rel_c4_z);
				// normal_log("LOC", "Vehicle %d corner coordinates in sensor sys: %.2f; %.2f; %.2f; %.2f; %.2f; %.2f; %.2f; %.2f", i, rel_c1_x, rel_c1_y, rel_c2_x, rel_c2_y, rel_c3_x, rel_c3_y, rel_c4_x, rel_c4_y);
				// normal_log("DEBUG","Vehicle %d center of BB coordinates: %.2f,%.2f,%.2f", i,veh_x,veh_y,veh_z);
				// normal_log("DEBUG","Vehicle %d corner azimuth angels: %.2f,%.2f,%.2f,%.2f", i,phi_c1,phi_c2,phi_c3,phi_c4);
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
				// normal_log("LOC", "Vehicle %d dist %.2f", i, distance[i]);
				// trans_x = veh_x - sens_World_x; // Vector from sensor to vehicle (center of bounding box of vehicle)
				// trans_y = veh_y - sens_World_y;
				// trans_z = veh_z - sens_World_z;
				// distance[i] = sqrt(trans_x * trans_x + trans_y * trans_y + trans_z *  trans_z);	// distance to center of bounding box of vehicle
				trans_x = cc_x - sens_World_x; // Vector from sensor to closest corner of vehicle
				trans_y = cc_y - sens_World_y;
				trans_z = cc_z - sens_World_z;
				// normal_log("DEBUG","trans_x: %.2f, trans_y: %.2f,trans_z: %.2f",trans_x,trans_y,trans_z);
				normal_log("OSI", "Current Veh Position: %.2f,%.2f,%.2f (long name)", veh.base().position().x(), veh.base().position().y(), veh.base().position().z());//test, delme
				normal_log("OSI", "Current Ego Position: %.2f,%.2f,%.2f", ego_World_x, ego_World_y, ego_World_z);
				// rotatePoint(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, rel_x, rel_y, rel_z); //rotate into coordinate system of ego vehicle
				rot2veh(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, rel_x, rel_y, rel_z); //rotate into coordinate system of ego vehicle
				// distance[i] = sqrt(rel_x * rel_x + rel_y*rel_y + rel_z*rel_z); //same as with trans?
				phi[i] = arcTan(rel_y, rel_x); // Azimuth of closest corner (was: center of bounding box)

				//normal_log("DEBUG", "Output Vehicle %d BB corner coordinates: %.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f", i, corner1_x, corner1_y, corner2_x, corner2_y, corner3_x, corner3_y, corner4_x, corner4_y);
				//normal_log("DEBUG", "Output Vehicle %d BB corner coordinates: %.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f", veh.id().value(), corner1_x, corner1_y, corner2_x, corner2_y, corner3_x, corner3_y, corner4_x, corner4_y);
				// normal_log("DEBUG","Output Vehicle %d Corner 1 coordinates: %.2f %.2f %.2f and distance %.2f m",i,corner1_x,corner1_y,corner1_z,distc1);
				// normal_log("DEBUG","Output Vehicle %d Corner 2 coordinates: %.2f %.2f %.2f and distance %.2f m",i,corner2_x,corner2_y,corner2_z,distc2);
				// normal_log("DEBUG","Output Vehicle %d Corner 3 coordinates: %.2f %.2f %.2f and distance %.2f m",i,corner3_x,corner3_y,corner3_z,distc3);
				// normal_log("DEBUG","Output Vehicle %d Corner 4 coordinates: %.2f %.2f %.2f and distance %.2f m",i,corner4_x,corner4_y,corner4_z,distc4);
				//normal_log("DEBUG", "Output Vehicle %d distance to closest corner: %.2f", i, distM[i]);
				// normal_log("DEBUG","Output Vehicle %d distance from sensor to closest corner of vehicle: %.2f",i,distance[i]);v
				// normal_log("DEBUG","Output Vehicle %d Sys Coords: %.2f %.2f %.2f",i,trans_x, trans_y, trans_z);
				// normal_log("DEBUG","Output Vehicle %d senAzimuth: %.2f",i,phi[i]*180/PI);
				// normal_log("DEBUG","Output Vehicle %d EGO Coords sens: %.2f %.2f %.2f",i,rel_x, rel_y, rel_z);
			}
			else
			{
				// normal_log("OSI", "Ignoring EGO Vehicle from occlusion.");
			}
		}


		for (auto v_i = currentViewIn.global_ground_truth().moving_object().begin();
			v_i != currentViewIn.global_ground_truth().moving_object().end();
			++v_i) {
			const osi3::MovingObject& veh = *v_i;
			// osi3::DetectedObject *obj = currentOut.mutable_object()->Add();
			size_t const i = v_i - currentViewIn.global_ground_truth().moving_object().begin();
			// auto const i = v_i - currentViewIn.global_ground_truth().moving_object().begin();
			auto const vid = veh.id().value();
			// auto const vid = i;
			// normal_log("OSI", "Vehicle %d, vid %d, begin %d, end %d", i, vid, currentViewIn.global_ground_truth().moving_object().begin(), currentViewIn.global_ground_truth().moving_object().end());
			if (vid != ego_id.value()) {


				/****** Occlusion (by comparison with all other vehicles) ******/

				double vis = 1;		// visibility
				double occ = 0;		// occlusion of target vehicle[i]

				// bool masked = 0;
				int occ_ind = 0;	// index of occluding car
				double loc;			// length of occluding car
				// This algorithm gives the maximum occlusion by any other car. If occlusion is 100 % (masked) then this car is not detected.
				// This algorithm does NOT consider combined occlusion by multiple cars. I.e. if a car is totally occluded be two other cars this is NOT considered (only the higher partial occlusion).
				// 2do: consider combined occlusion
				for (size_t j = 1; j < nof_mov_obj; ++j) { //assume j=0 =always EGO tbconfirmed! type size_t of j was auto but caused compiler warning
					// if ((j!=i) && (distance[j]<distance[i]) && (phi_min[j]<phi_min[i]) && (phi_max[j]<phi_max[i])) masked = 1;
					if ((j != i) && (distM[j] < distM[i])) {  //2do: rear cars should be excluded; may cause false values
						// normal_log("DEBUG","i %d, j %d, distance[j] %.2f,distance[i] %.2f, phi_min[j] %.2f, phi_min[i] %.2f, phi_max[j] %.2f, phi_max[i] %.2f, masked %d, nof_obj %d",i,j,distance[j],distance[i],phi_min[j],phi_min[i],phi_max[j],phi_max[i],masked[i],nof_obj);
						if ((phi_max[j] < phi_min[i]) || (phi_min[j] > phi_max[i])) {
							//		normal_log("OSI", "Vehicle %d not occluded by vehicle %d", i, j);
						}
						else {
							double phi_min_list[] = { phi_min[i], phi_min[j] }; //surely there is a simpler command as there are only 2 elements 
							double phi_max_list[] = { phi_max[i], phi_max[j] }; //surely there is a simpler command as there are only 2 elements 
							double phi_min_ij = *std::max_element(phi_min_list, phi_min_list + 2);
							double phi_max_ij = *std::min_element(phi_max_list, phi_max_list + 2);
							double occ_temp = (phi_max_ij - phi_min_ij) / (phi_max[i] - phi_min[i]);
							if (occ_temp > occ) {
								occ = occ_temp; // Take the maximum occlusion.
								loc = currentViewIn.global_ground_truth().moving_object(j).base().dimension().length();			// length of occluding car
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
		for_each(currentViewIn.global_ground_truth().moving_object().begin(), currentViewIn.global_ground_truth().moving_object().end(),
			[this, &i, &currentViewIn, &currentOut, &masked, ego_id, ego_World_x, ego_World_y, ego_World_z, ego_vel_x, ego_vel_y, ego_vel_z, &mpos_x, &mpos_y, &mpos_z, &origin_World_x, &origin_World_y, &origin_World_z, &origin_rot_x, &origin_rot_y, &origin_rot_z, &mpos_rot_x, &mpos_rot_y, &mpos_rot_z, ego_yaw, ego_pitch, ego_roll, actual_range, &sens_World_x, &sens_World_y, &sens_World_z, &sensSV_x, &sensSV_y, &sensSV_z](const osi3::MovingObject& veh) {
			if (veh.id().value() != ego_id.value()) {


				//TypeName = "Type_Unknown";
				std::string TypeName2 = veh.GetTypeName();
				//		normal_log("DEBUG", "moving object information id %d and masked %d", veh.id().value(), masked[i + 1]);
				//Age
				object_info current_object_history{};
				int obj_idx = i;
				int current_object_idx = get_object_info_idx(object_history_vector, currentViewIn.global_ground_truth().moving_object(obj_idx).id().value());
				update_object_history_vector(current_object_history, currentViewIn, obj_idx, true);


				//Calculate the Moving Objects in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
				double xxKoordinate = 0;	double yyKoordinate = 0; double zzKoordinate = 0;  //Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
				//double trans_x = veh.base().position().x() - sens_World_x;
				//double trans_y = veh.base().position().y() - sens_World_y;
				//double trans_z = veh.base().position().z() - sens_World_z;
				//Test

				double trans_x = veh.base().position().x() - origin_World_x;
				double trans_y = veh.base().position().y() - origin_World_y;
				double trans_z = veh.base().position().z() - origin_World_z;


				double vehOrientationYaw = veh.base().orientation().yaw();
				double vehOrientationPitch = veh.base().orientation().pitch();
				double vehOrientationRoll = veh.base().orientation().roll();

				//Test
				//trans_x = 7;
				//trans_y = 8;
				//trans_z = 0;
				normal_log("DEBUG", "Detected object bb center to mitte Hinterachse in Welt koordinaten %f,%f,%f", trans_x, trans_y, trans_z);
				normal_log("DEBUG", "Detected object Orientierung  %f,%f,%f", veh.base().orientation().yaw(), veh.base().orientation().pitch(), veh.base().orientation().roll());

				double R1 = 0, R2 = 0, R3 = 0;
				EulerWinkel(ego_yaw, ego_pitch, ego_roll, vehOrientationYaw, vehOrientationPitch, vehOrientationRoll, R1, R2, R3);
				normal_log("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", R1, R2, R3);
				//double trans_x = veh.base().position().x() - ego_World_x;
				//double trans_y = veh.base().position().y() - ego_World_y;
				//double trans_z = veh.base().position().z() - ego_World_z;
				//                 
				//double RVXx = 0; double RVXy = 0; double RVXz = 0;
				//double RVYx = 0; double RVYy = 0; double RVYz = 0;
				//double RVZx = 0; double RVZy = 0; double RVZz = 0;

				//rot2env(1,0,0,ego_yaw,ego_pitch,ego_roll,RVXx,RVXy,RVXz);
				//rot2env(0, 1, 0, ego_yaw, ego_pitch, ego_roll, RVYx, RVYy, RVYz);
				//rot2env(0, 0,1, ego_yaw, ego_pitch, ego_roll, RVZx, RVZy, RVZz);
				////normal_log("OSI", "RVX: %f,%f,%f", RVXx,RVXy,RVXz);
				////normal_log("OSI", "RVY: %f,%f,%f", RVYx,RVYy,RVYz);
				////normal_log("OSI", "RVZ: %f,%f,%f", RVZx,RVZy,RVZz);

				//double vectRVX[] = { RVXx, RVXy, RVXz };double vectRVY[] = { RVYx, RVYy, RVYz };double vectRVZ[] = { RVZx, RVZy, RVZz };
				//double vectEXY[2]; double vectEXZ[2];double vectEYZ[2];
				//crossProduct(vectRVX, vectRVY, vectEXY);crossProduct(vectRVX, vectRVZ, vectEXZ);crossProduct(vectRVY, vectRVZ, vectEYZ);
				////normal_log("OSI", "Cross product EXY: %f,%f,%f", vectEXY[0], vectEXY[1], vectEXY[2]);
				////normal_log("OSI", "Cross product EXZ: %f,%f,%f", vectEXZ[0], vectEXZ[1], vectEXZ[2]);
				////normal_log("OSI", "Cross product EYZ: %f,%f,%f", vectEYZ[0], vectEYZ[1], vectEYZ[2]);
				//double D[] = { trans_x, trans_y, trans_z };
				////Lot berechnen 					
				//zzKoordinate= CalculateKoordinate(vectEXY[0], vectEXY[1], vectEXY[2], D[0], D[1], D[2], vectRVZ[0], vectRVZ[1], vectRVZ[2]);				
				//yyKoordinate = CalculateKoordinate(vectEXZ[0], vectEXZ[1], vectEXZ[2], D[0], D[1], D[2], vectRVY[0], vectRVY[1], vectRVY[2]);
				//xxKoordinate = CalculateKoordinate(vectEYZ[0], vectEYZ[1], vectEYZ[2], D[0], D[1], D[2], vectRVX[0], vectRVX[1], vectRVX[2]);


				//CalKoordNew(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxKoordinate, yyKoordinate, zzKoordinate);
				rot2veh(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxKoordinate, yyKoordinate, zzKoordinate);
				normal_log("DEBUG", "Detected object in Vehicle Koordinaten %f,%f,%f", xxKoordinate, yyKoordinate, zzKoordinate);
				//rot2env(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxKoordinate, yyKoordinate, zzKoordinate);
				//Test zurueck zur Mitte BB 
				/*double newxxKoordinate = xxKoordinate + mpos_rot_x;
				double newyyKoordinate = yyKoordinate + mpos_rot_y;
				double newzzKoordinate = zzKoordinate + mpos_rot_z;*/


				//normal_log("OSI", "Moving Object in Sensor-Coordinates: %f,%f,%f", xxKoordinate, yyKoordinate, zzKoordinate);
				//normal_log("OSI", "Moving Object relative to the origin of ego-car-Coordinates: %f,%f,%f", newxxKoordinate, newyyKoordinate, newzzKoordinate);

				double distance = sqrt(trans_x*trans_x + trans_y * trans_y + trans_z * trans_z);
				//Vector camera sensor, camera view direction 
				double g1 = sensSV_x - sens_World_x;
				double g2 = sensSV_y - sens_World_y;
				double g3 = sensSV_z - sens_World_z;
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
				double vx_sens, vy_sens, vz_sens; //velocity components in coordinate system of ego vehicle
				rot2veh(delta_x, delta_y, delta_z, ego_yaw, ego_pitch, ego_roll, vx_sens, vy_sens, vz_sens); //rotate into coordinate system of ego vehicle

				normal_log("OSI", "Velocity of moving object global coordinates: x %f ,y %f z %f \n", vx, vy, vz);
				normal_log("OSI", "Velocity of moving object environment coordinates: x %f ,y %f z %f \n", vx_sens, vy_sens, vz_sens);
				//Occlusion 
				//1. Define the 

				//for (size_t j = 0; j<nof_veh; ++j){ //assume j=0 =always EGO tbconfirmed!
				// if ((j!=i) && (distance[j]<distance[i]) && (phi_min[j]<phi_min[i]) && (phi_max[j]<phi_max[i])) masked = 1;
				//if ((j != i) && (j != ego_id.value()) && round(rel_yV[i]) == round(rel_yV[j]) && (distanceV[j]<distanceV[i])){//&& round(rel_yV[j])==round(rel_yV[i])  ) {  //2do: rear cars should 
				// if (i==0){

				// masked = rel_yV[1]+88;

				// }
				// if (i==1){

				// masked = rel_yV[2]+88;}
				//	masked = true;



				//}
				//}

				normal_log("DEBUG", "moving object information id %d and masked %d and distance %f and angke_to_mov_obj %f", veh.id().value(), masked[i + 1], distance, angle_to_mov_obj);
				//Define detection range //
				//abs wegen negativen werten 
				if ((distance <= camera_range) && abs(angle_to_mov_obj) < camera_FOV) {
					if (masked[i + 1] == 0) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
						osi3::DetectedMovingObject *obj = currentOut.mutable_moving_object()->Add();
						currentOut.mutable_moving_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

						int max = 10;
						int min = 0;
						double randomnumber = (rand() % (max - min)) + min;

						double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;
						if (existence_prob > 100) {
							existence_prob = 100;
						}
						normal_log("DEBUG", "Randomnumber is %f", randomnumber);
						double randomPos = randomnumber / 100;
						normal_log("DEBUG", "Random number is %f", randomPos);
						obj->mutable_header()->add_ground_truth_id()->CopyFrom(veh.id());
						obj->mutable_header()->mutable_tracking_id()->set_value(i);
						obj->mutable_header()->set_existence_probability(existence_prob / 100);
						obj->mutable_header()->set_age(current_object_history.age);
						obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
						obj->mutable_header()->add_sensor_id()->CopyFrom(currentViewIn.sensor_id());

						//Position in Sensor Koordinaten 
						//Input are the Coordinates in the Sensor-Environment 




						obj->mutable_base()->mutable_position()->set_x(xxKoordinate);
						obj->mutable_base()->mutable_position()->set_y(yyKoordinate);
						obj->mutable_base()->mutable_position()->set_z(zzKoordinate);

						obj->mutable_base()->mutable_orientation()->set_pitch(R1);
						obj->mutable_base()->mutable_orientation()->set_roll(R3);
						obj->mutable_base()->mutable_orientation()->set_yaw(R2);

						obj->mutable_base()->mutable_velocity()->set_x(vx_sens);
						obj->mutable_base()->mutable_velocity()->set_y(vy_sens);
						obj->mutable_base()->mutable_velocity()->set_z(vz_sens);
						obj->mutable_base()->mutable_dimension()->set_length(veh.base().dimension().length());
						obj->mutable_base()->mutable_dimension()->set_width(veh.base().dimension().width());
						obj->mutable_base()->mutable_dimension()->set_height(veh.base().dimension().height());


						//Alternative
						//obj->mutable_candidate(0)->set_type(veh.type);
						//Type Unknown, Other, Vehicle, PEdestrian, Animal
						// Type {TYPE_UNKNOWN = 0, TYPE_OTHER = 1, TYPE_SMALL_CAR = 2, TYPE_COMPACT_CAR = 3,TYPE_MEDIUM_CAR = 4, TYPE_LUXURY_CAR = 5, TYPE_DELIVERY_VAN = 6, TYPE_HEAVY_TRUCK = 7,TYPE_SEMITRAILER = 8, TYPE_TRAILER = 9, TYPE_MOTORBIKE = 10, TYPE_BICYCLE = 11,TYPE_BUS = 12, TYPE_TRAM = 13, TYPE_TRAIN = 14, TYPE_WHEELCHAIR = 15
						osi3::DetectedMovingObject::CandidateMovingObject* candidate = obj->add_candidate();
						candidate->set_type(veh.type());
						candidate->mutable_vehicle_classification()->CopyFrom(veh.vehicle_classification());
						candidate->set_probability(1);



						normal_log("DEBUG", "Detected moving object %d minimal distance to Ego/Sensor %f", i, distance);
						normal_log("DEBUG", "Detected moving object %d angle to Ego/Sensor %f", i, angle_to_mov_obj);
						normal_log("DEBUG", "Detected moving object type %d", veh.type());
						normal_log("DEBUG", "Detected moving object type %s", candidate->GetTypeName().c_str());

						//candidate->type()
						normal_log("OSI", "Output Object %d[%llu] Probability %f Detected mov object position and orientation in environment coord: %f,%f,%f (%f,%f,%f)", i, veh.id().value(), obj->header().existence_probability(), veh.base().position().x(), veh.base().position().y(), veh.base().position().z(), veh.base().orientation().yaw(), veh.base().orientation().pitch(), veh.base().orientation().roll());
						normal_log("OSI", "Output Object %d[%llu] Probability %f Detected mov object position in sensor coord: %f,%f,%f \n", i, veh.id().value(), obj->header().existence_probability(), obj->base().position().x(), obj->base().position().y(), obj->base().position().z());

					}
				}
				else {
					normal_log("OSI", "Ignoring Vehicle %d[%llu] Outside Sensor Scope Relative Position - ego and GT Position: %f,%f,%f (%f,%f,%f)", i, veh.id().value(), veh.base().position().x() - ego_World_x, veh.base().position().y() - ego_World_y, veh.base().position().z() - ego_World_z, veh.base().position().x(), veh.base().position().y(), veh.base().position().z());
					normal_log("OSI", "ENDE.......................................................................");
					//normal_log("OSI", "Ignoring Vehicle %d[%llu] Outside Sensor Scope Object trans_x,trans_y,trans_z, distance to sensor,winkel %f,%f,%f (%f,%f,%f) ,%f,%f", i, veh.id().value(), trans_x, trans_y, trans_z, rel_x, rel_y, rel_z, distance,winkel);

				}
				i++;
			}
			else
			{
				normal_log("OSI", "Ignoring EGO Vehicle %d[%llu] Relative Position: %f,%f,%f (%f,%f,%f)", i, veh.id().value(), veh.base().position().x() - ego_World_x, veh.base().position().y() - ego_World_y, veh.base().position().z() - ego_World_z, veh.base().position().x(), veh.base().position().y(), veh.base().position().z());
				normal_log("OSI", "ENDE.......................................................................");
			}
		});

		// normal_log("OSI","Mapped %d vehicles to output", i);




		//Stationary objects

		for_each(currentViewIn.global_ground_truth().stationary_object().begin(), currentViewIn.global_ground_truth().stationary_object().end(),
			[this, &i, &currentViewIn, &currentOut, masked, ego_id, ego_World_x, ego_World_y, ego_World_z, &origin_World_x, &origin_World_y, &origin_World_z, &mpos_x, &mpos_y, &mpos_z, ego_yaw, ego_pitch, ego_roll, actual_range, &sens_World_x, &sens_World_y, &sens_World_z, &sensSV_x, &sensSV_y, &sensSV_z](const osi3::StationaryObject& stobj) {


			//TypeName = "Type_Unknown";
			//std::string TypeName2 = stobj.GetTypeName();

			//Age
			//object_info current_object_history{};
			//int obj_idx = i;
			//int current_object_idx = get_object_info_idx(object_history_vector, currentViewIn.global_ground_truth().stationary_object(obj_idx).id().value());
			//update_object_history_vector(current_object_history, currentViewIn, obj_idx, false);


			//Calculate the Moving Objects in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
			double xxKoordinate;	double yyKoordinate; double zzKoordinate;  //Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
			/*double trans_x = stobj.base().position().x() - sens_World_x;
			double trans_y = stobj.base().position().y() - sens_World_y;
			double trans_z = stobj.base().position().z() - sens_World_z;
			*/
			double trans_x = stobj.base().position().x() - origin_World_x;
			double trans_y = stobj.base().position().y() - origin_World_y;
			double trans_z = stobj.base().position().z() - origin_World_z;


			double stobjOrientationYaw = stobj.base().orientation().yaw();
			double stobjOrientationPitch = stobj.base().orientation().pitch();
			double stobjOrientationRoll = stobj.base().orientation().roll();


			double R1 = 0, R2 = 0, R3 = 0;
			EulerWinkel(ego_yaw, ego_pitch, ego_roll, stobjOrientationYaw, stobjOrientationPitch, stobjOrientationRoll, R1, R2, R3);
			normal_log("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", R1, R2, R3);
			//double RVXx = 0; double RVXy = 0; double RVXz = 0;
			//double RVYx = 0; double RVYy = 0; double RVYz = 0;
			//double RVZx = 0; double RVZy = 0; double RVZz = 0;

			//rot2env(1, 0, 0, ego_yaw, ego_pitch, ego_roll, RVXx, RVXy, RVXz);
			//rot2env(0, 1, 0, ego_yaw, ego_pitch, ego_roll, RVYx, RVYy, RVYz);
			//rot2env(0, 0, 1, ego_yaw, ego_pitch, ego_roll, RVZx, RVZy, RVZz);
			////normal_log("OSI", "RVX: %f,%f,%f", RVXx,RVXy,RVXz);
			////normal_log("OSI", "RVY: %f,%f,%f", RVYx,RVYy,RVYz);
			////normal_log("OSI", "RVZ: %f,%f,%f", RVZx,RVZy,RVZz);

			//double vectRVX[] = { RVXx, RVXy, RVXz }; double vectRVY[] = { RVYx, RVYy, RVYz }; double vectRVZ[] = { RVZx, RVZy, RVZz };
			//double vectEXY[2]; double vectEXZ[2]; double vectEYZ[2];
			//crossProduct(vectRVX, vectRVY, vectEXY); crossProduct(vectRVX, vectRVZ, vectEXZ); crossProduct(vectRVY, vectRVZ, vectEYZ);
			////normal_log("OSI", "Cross product EXY: %f,%f,%f", vectEXY[0], vectEXY[1], vectEXY[2]);
			////normal_log("OSI", "Cross product EXZ: %f,%f,%f", vectEXZ[0], vectEXZ[1], vectEXZ[2]);
			////normal_log("OSI", "Cross product EYZ: %f,%f,%f", vectEYZ[0], vectEYZ[1], vectEYZ[2]);
			//double D[] = { trans_x, trans_y, trans_z };
			////Lot berechnen 					
			//zzKoordinate = 0;// CalculateKoordinate(vectEXY[0], vectEXY[1], vectEXY[2], D[0], D[1], D[2], vectRVZ[0], vectRVZ[1], vectRVZ[2]);
			//yyKoordinate = 0;// CalculateKoordinate(vectEXZ[0], vectEXZ[1], vectEXZ[2], D[0], D[1], D[2], vectRVY[0], vectRVY[1], vectRVY[2]);
			//xxKoordinate = 0;// CalculateKoordinate(vectEYZ[0], vectEYZ[1], vectEYZ[2], D[0], D[1], D[2], vectRVX[0], vectRVX[1], vectRVX[2]);

			//double zznewKoordinate = CalculateKoordinateNew(trans_x,trans_y,trans_z,ego_yaw,ego_pitch,ego_roll);
			CalKoordNew(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxKoordinate, yyKoordinate, zzKoordinate);
			normal_log("OSI", "Stationary Object in Sensor-Coordinates: %f,%f,%f", xxKoordinate, yyKoordinate, zzKoordinate);


			double distance = sqrt(trans_x*trans_x + trans_y * trans_y + trans_z * trans_z);
			//Vector camera sensor, camera view direction 
			double g1 = sensSV_x - sens_World_x;
			double g2 = sensSV_y - sens_World_y;
			double g3 = sensSV_z - sens_World_z;
			//Angle between 
			double angle_to_stat_obj = CalculateAngle(trans_x, trans_y, trans_z, g1, g2, g3);
			normal_log("OSI", "Winkel Vektors Sensor-Richtungsvektor zum Sensor-Objekt: %f", angle_to_stat_obj);



			//Define detection range //
			//abs wegen negativen werten 
			if ((distance <= camera_range) && abs(angle_to_stat_obj) < camera_FOV && !masked[i]) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
				osi3::DetectedStationaryObject *obj = currentOut.mutable_stationary_object()->Add();
				currentOut.mutable_stationary_object_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

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
				//obj->mutable_header()->set_age(current_object_history.age);
				obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
				obj->mutable_header()->add_sensor_id()->CopyFrom(currentViewIn.sensor_id());

				//Position in Sensor Koordinaten 
				//Input are the Coordinates in the Sensor-Environment 
				obj->mutable_base()->mutable_position()->set_x(xxKoordinate);
				obj->mutable_base()->mutable_position()->set_y(yyKoordinate);
				obj->mutable_base()->mutable_position()->set_z(zzKoordinate);
				obj->mutable_base()->mutable_orientation()->set_pitch(R1);
				obj->mutable_base()->mutable_orientation()->set_roll(R3);
				obj->mutable_base()->mutable_orientation()->set_yaw(R2);

				obj->mutable_base()->mutable_dimension()->set_length(stobj.base().dimension().length());
				obj->mutable_base()->mutable_dimension()->set_width(stobj.base().dimension().width());
				obj->mutable_base()->mutable_dimension()->set_height(stobj.base().dimension().height());


				osi3::DetectedStationaryObject::CandidateStationaryObject* candidate = obj->add_candidate();
				//candidate->set_type(stobj.type());
				candidate->mutable_classification()->CopyFrom(stobj.classification());
				candidate->set_probability(1);


				normal_log("DEBUG", "Detected stationary object %d minimal distance to Ego/Sensor %f", i, distance);
				normal_log("DEBUG", "Detected stationary object %d angle to Ego/Sensor %f", i, angle_to_stat_obj);
				normal_log("OSI", "Output Object %d[%llu] Probability %f Detected stat object position and orientation in environment coord: %f,%f,%f (%f,%f,%f)", i, stobj.id().value(), obj->header().existence_probability(), stobj.base().position().x(), stobj.base().position().y(), stobj.base().position().z(), stobj.base().orientation().yaw(), stobj.base().orientation().pitch(), stobj.base().orientation().roll());
				normal_log("OSI", "Output Object %d[%llu] Probability %f Detected stat object position in sensor coord: %f,%f,%f \n", i, stobj.id().value(), obj->header().existence_probability(), obj->base().position().x(), obj->base().position().y(), obj->base().position().z());
				i++;
			}
			else {
				normal_log("OSI", "Ignoring stationary objects %d[%llu] Outside Sensor Scope Relative Position - ego and GT Position: %f,%f,%f (%f,%f,%f)", i, stobj.id().value(), stobj.base().position().x() - ego_World_x, stobj.base().position().y() - ego_World_y, stobj.base().position().z() - ego_World_z, stobj.base().position().x(), stobj.base().position().y(), stobj.base().position().z());
				//normal_log("OSI", "Ignoring stobjicle %d[%llu] Outside Sensor Scope Object trans_x,trans_y,trans_z, distance to sensor,winkel %f,%f,%f (%f,%f,%f) ,%f,%f", i, stobj.id().value(), trans_x, trans_y, trans_z, rel_x, rel_y, rel_z, distance,winkel);

			}


		});


		int itl = 0;

		//Traffic Lights 
		for_each(currentViewIn.global_ground_truth().traffic_light().begin(), currentViewIn.global_ground_truth().traffic_light().end(),
			[this, &itl, &currentViewIn, &currentOut, masked, ego_id, ego_World_x, ego_World_y, ego_World_z, &origin_World_x, &origin_World_y, &origin_World_z, &mpos_x, &mpos_y, &mpos_z, ego_yaw, ego_pitch, ego_roll, actual_range, &sens_World_x, &sens_World_y, &sens_World_z, &sensSV_x, &sensSV_y, &sensSV_z](const osi3::TrafficLight& trafficlight) {


			//TypeName = "Type_Unknown";
			//std::string TypeName2 = trafficlight.GetTypeName();

			//Age
			/*object_info current_object_history{};
			int obj_idx = i;
			int current_object_idx = get_object_info_idx(object_history_vector, currentViewIn.global_ground_truth().traffic_light(obj_idx).id().value());
			update_object_history_vector(current_object_history, currentViewIn, obj_idx, false);
			*/

			//Calculate the traffic light  in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
			double xxKoordinate;	double yyKoordinate; double zzKoordinate;  //Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
			//double trans_x = trafficlight.base().position().x() - sens_World_x;
			//double trans_y = trafficlight.base().position().y() - sens_World_y;
			///double trans_z = trafficlight.base().position().z() - sens_World_z;
			double trans_x = trafficlight.base().position().x() - origin_World_x;
			double trans_y = trafficlight.base().position().y() - origin_World_y;
			double trans_z = trafficlight.base().position().z() - origin_World_z;

			double trafficlightOrientationYaw = trafficlight.base().orientation().yaw();
			double trafficlightOrientationPitch = trafficlight.base().orientation().pitch();
			double trafficlightOrientationRoll = trafficlight.base().orientation().roll();


			double R1 = 0, R2 = 0, R3 = 0;
			EulerWinkel(ego_yaw, ego_pitch, ego_roll, trafficlightOrientationYaw, trafficlightOrientationPitch, trafficlightOrientationRoll, R1, R2, R3);
			normal_log("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", R1, R2, R3);
			//double RVXx = 0; double RVXy = 0; double RVXz = 0;
			//double RVYx = 0; double RVYy = 0; double RVYz = 0;
			//double RVZx = 0; double RVZy = 0; double RVZz = 0;

			//rot2env(1, 0, 0, ego_yaw, ego_pitch, ego_roll, RVXx, RVXy, RVXz);
			//rot2env(0, 1, 0, ego_yaw, ego_pitch, ego_roll, RVYx, RVYy, RVYz);
			//rot2env(0, 0, 1, ego_yaw, ego_pitch, ego_roll, RVZx, RVZy, RVZz);
			////normal_log("OSI", "RVX: %f,%f,%f", RVXx,RVXy,RVXz);
			////normal_log("OSI", "RVY: %f,%f,%f", RVYx,RVYy,RVYz);
			////normal_log("OSI", "RVZ: %f,%f,%f", RVZx,RVZy,RVZz);

			//double vectRVX[] = { RVXx, RVXy, RVXz }; double vectRVY[] = { RVYx, RVYy, RVYz }; double vectRVZ[] = { RVZx, RVZy, RVZz };
			//double vectEXY[2]; double vectEXZ[2]; double vectEYZ[2];
			//crossProduct(vectRVX, vectRVY, vectEXY); crossProduct(vectRVX, vectRVZ, vectEXZ); crossProduct(vectRVY, vectRVZ, vectEYZ);
			////normal_log("OSI", "Cross product EXY: %f,%f,%f", vectEXY[0], vectEXY[1], vectEXY[2]);
			////normal_log("OSI", "Cross product EXZ: %f,%f,%f", vectEXZ[0], vectEXZ[1], vectEXZ[2]);
			////normal_log("OSI", "Cross product EYZ: %f,%f,%f", vectEYZ[0], vectEYZ[1], vectEYZ[2]);
			//double D[] = { trans_x, trans_y, trans_z };
			////Lot berechnen 					
			//zzKoordinate = 0;// CalculateKoordinate(vectEXY[0], vectEXY[1], vectEXY[2], D[0], D[1], D[2], vectRVZ[0], vectRVZ[1], vectRVZ[2]);
			//yyKoordinate = 0;// CalculateKoordinate(vectEXZ[0], vectEXZ[1], vectEXZ[2], D[0], D[1], D[2], vectRVY[0], vectRVY[1], vectRVY[2]);
			//xxKoordinate = 0;// CalculateKoordinate(vectEYZ[0], vectEYZ[1], vectEYZ[2], D[0], D[1], D[2], vectRVX[0], vectRVX[1], vectRVX[2]);

			//double zznewKoordinate = CalculateKoordinateNew(trans_x,trans_y,trans_z,ego_yaw,ego_pitch,ego_roll);

			normal_log("OSI", "Ground Truth Traffic Sign  %f, %f,%f ", trafficlight.base().position().x(), trafficlight.base().position().y(), trafficlight.base().position().z());
			CalKoordNew(trans_x, trans_y, trans_z, ego_yaw, ego_pitch, ego_roll, xxKoordinate, yyKoordinate, zzKoordinate);
			normal_log("OSI", "TrafficLight in Sensor-Coordinates: %f,%f,%f", xxKoordinate, yyKoordinate, zzKoordinate);


			double distance = sqrt(trans_x*trans_x + trans_y * trans_y + trans_z * trans_z);
			//Vector camera sensor, camera view direction 
			double g1 = sensSV_x - sens_World_x;
			double g2 = sensSV_y - sens_World_y;
			double g3 = sensSV_z - sens_World_z;
			//Angle between 
			double angle_to_traffic_light = CalculateAngle(trans_x, trans_y, trans_z, g1, g2, g3);
			//normal_log("OSI", "Winkel Vektors Sensor-Richtungsvektor zum Sensor-Objekt: %f", angle_to_mov_obj);


			//Define detection range //
			//abs wegen negativen werten 
			if ((distance <= camera_range) && abs(angle_to_traffic_light) < camera_FOV) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
				osi3::DetectedTrafficLight *obj = currentOut.mutable_traffic_light()->Add();
				currentOut.mutable_traffic_light_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

				int max = 10;
				int min = 0;
				int randomnumber = (rand() % (max - min)) + min;

				double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;
				if (existence_prob > 100) {
					existence_prob = 100;
				}

				obj->mutable_header()->add_ground_truth_id()->CopyFrom(trafficlight.id());
				normal_log("OSI", "TrafficLight ID: %f", trafficlight.id());
				obj->mutable_header()->mutable_tracking_id()->set_value(itl);
				obj->mutable_header()->set_existence_probability(existence_prob / 100);
				normal_log("OSI", "TrafficLight Existence Probability: %f", existence_prob / 100);
				//obj->mutable_header()->set_age(current_object_history.age);
				obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
				obj->mutable_header()->add_sensor_id()->CopyFrom(currentViewIn.sensor_id());

				//Position in Sensor Koordinaten 
				//Input are the Coordinates in the Sensor-Environment 
				obj->mutable_base()->mutable_position()->set_x(xxKoordinate);
				obj->mutable_base()->mutable_position()->set_y(yyKoordinate);
				obj->mutable_base()->mutable_position()->set_z(zzKoordinate);
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

				normal_log("DEBUG", "Detected traffic light color %d", candidate->classification().color());
				normal_log("DEBUG", "Detected traffic light mode %d", candidate->classification().mode());
				normal_log("DEBUG", "Detected traffic light icon %d", candidate->classification().icon());
				normal_log("DEBUG", "Detected traffic light assigned lane id %d", candidate->classification().assigned_lane_id());
				normal_log("DEBUG", "Detected traffic classification %d", trafficlight.classification());

				normal_log("DEBUG", "Detected traffic light %d minimal distance to Ego/Sensor %f", itl, distance);
				normal_log("DEBUG", "Detected traffic light %d angle to Ego/Sensor %f", itl, angle_to_traffic_light);
				normal_log("OSI", "Output traffic light %d[%llu] Probability %f Detected traffic light position and orientation in environment coord: %f,%f,%f (%f,%f,%f)", itl, trafficlight.id().value(), obj->header().existence_probability(), trafficlight.base().position().x(), trafficlight.base().position().y(), trafficlight.base().position().z(), trafficlight.base().orientation().yaw(), trafficlight.base().orientation().pitch(), trafficlight.base().orientation().roll());
				normal_log("OSI", "Output Object %d[%llu] Probability %f Detected  traffic light position in sensor coord: %f,%f,%f \n", itl, trafficlight.id().value(), obj->header().existence_probability(), obj->base().position().x(), obj->base().position().y(), obj->base().position().z());
				
				itl++;
			}
			else {
				normal_log("OSI", "Ignoring traffic light %d[%llu] Outside Sensor Scope Relative Position - ego and GT Position: %f,%f,%f (%f,%f,%f)", itl, trafficlight.id().value(), trafficlight.base().position().x() - ego_World_x, trafficlight.base().position().y() - ego_World_y, trafficlight.base().position().z() - ego_World_z, trafficlight.base().position().x(), trafficlight.base().position().y(), trafficlight.base().position().z());
				//normal_log("OSI", "Ignoring stobjicle %d[%llu] Outside Sensor Scope Object trans_x,trans_y,trans_z, distance to sensor,winkel %f,%f,%f (%f,%f,%f) ,%f,%f", i, trafficlight.id().value(), trans_x, trans_y, trans_z, rel_x, rel_y, rel_z, distance,winkel);

			}


		});



		// Traffic Signs 
		//Schleife geht über alle traffic signs, aber es werden alle Objekte als mainSign interpretiert 
		int its = 0;
		for_each(currentViewIn.global_ground_truth().traffic_sign().begin(), currentViewIn.global_ground_truth().traffic_sign().end(),
			[this, &its, &currentViewIn, &currentOut, masked, ego_id, ego_World_x, ego_World_y, ego_World_z, &origin_World_x, &origin_World_y, &origin_World_z, &mpos_x, &mpos_y, &mpos_z, ego_yaw, ego_pitch, ego_roll, actual_range, &sens_World_x, &sens_World_y, &sens_World_z, &sensSV_x, &sensSV_y, &sensSV_z](const osi3::TrafficSign& tsobj) {
			normal_log("OSI", "TrafficSign with ID %llu ", tsobj.id().value());
			//Calculate the traffic light  in Sensor coordinates (erstmal zur mounting posistion) !! Sp�ter eventuell zur Hinterachse Auto 
			double xxxKoordinate;	double yyyKoordinate; double zzzKoordinate;  //Coordinates of the moving object in sensor coordinate system (movin object center of bounding box to mounting position sensor) 
			//const osi3::TrafficSign_MainSign& tsMobj;
			//tsobj.main_sign().base().position().x();
			normal_log("OSI", "Ground Truth Traffic Sign  %f, %f,%f ", tsobj.main_sign().base().position().x(), tsobj.main_sign().base().position().y(), tsobj.main_sign().base().position().z());
			//double transTS_x = tsobj.main_sign().base().position().x() - sens_World_x;
			//double transTS_y = tsobj.main_sign().base().position().y() - sens_World_y;
			//double transTS_z = tsobj.main_sign().base().position().z() - sens_World_z;
			double transTS_x = tsobj.main_sign().base().position().x() - origin_World_x;
			double transTS_y = tsobj.main_sign().base().position().y() - origin_World_y;
			double transTS_z = tsobj.main_sign().base().position().z() - origin_World_z;
			double tsobjMOrientationYaw = tsobj.main_sign().base().orientation().yaw();
			double tsobjMOrientationPitch = tsobj.main_sign().base().orientation().pitch();
			double tsobjMOrientationRoll = tsobj.main_sign().base().orientation().roll();


			double R1 = 0, R2 = 0, R3 = 0;
			EulerWinkel(ego_yaw, ego_pitch, ego_roll, tsobjMOrientationYaw, tsobjMOrientationPitch, tsobjMOrientationRoll, R1, R2, R3);
			normal_log("DEBUG", "Detected object Orientierung zum Vehicle  %f,%f,%f", R1, R2, R3);

			normal_log("OSI", "TrafficSign position x %f", tsobj.main_sign().base().position().x() - sens_World_x);
			normal_log("OSI", "TrafficSign position y %f", tsobj.main_sign().base().position().y() - sens_World_y);
			normal_log("OSI", "TrafficSign position z %f", tsobj.main_sign().base().position().z() - sens_World_z);

			CalKoordNew(transTS_x, transTS_y, transTS_z, ego_yaw, ego_pitch, ego_roll, xxxKoordinate, yyyKoordinate, zzzKoordinate);

			normal_log("OSI", "TrafficSign in Sensor-Coordinates: %f,%f,%f", xxxKoordinate, yyyKoordinate, zzzKoordinate);

			double distance = sqrt(transTS_x*transTS_x + transTS_y * transTS_y + transTS_z * transTS_z);
			//Vector camera sensor, camera view direction 
			double g1 = sensSV_x - sens_World_x;
			double g2 = sensSV_y - sens_World_y;
			double g3 = sensSV_z - sens_World_z;
			//Angle between 
			double angle_to_traffic_sign = CalculateAngle(transTS_x, transTS_y, transTS_z, g1, g2, g3);

			if ((distance <= camera_range) && abs(angle_to_traffic_sign) < camera_FOV) {// (abs(trans_x/distance) >0.766025)){//0.766025)) {
				osi3::DetectedTrafficSign *obj = currentOut.mutable_traffic_sign()->Add();
				currentOut.mutable_traffic_sign_header()->set_data_qualifier(osi3::DetectedEntityHeader_DataQualifier_DATA_QUALIFIER_AVAILABLE);

				int max = 10;
				int min = 0;
				int randomnumber = (rand() % (max - min)) + min;

				double existence_prob = 100 - abs(distance*0.2) + 0.2*randomnumber;

				obj->mutable_header()->add_ground_truth_id()->CopyFrom(tsobj.id());
				obj->mutable_header()->mutable_tracking_id()->set_value(its);
				obj->mutable_header()->set_existence_probability(existence_prob / 100);
				//obj->mutable_header()->set_age(current_object_history.age);
				obj->mutable_header()->set_measurement_state(osi3::DetectedItemHeader_MeasurementState_MEASUREMENT_STATE_MEASURED);
				obj->mutable_header()->add_sensor_id()->CopyFrom(currentViewIn.sensor_id());

				//Position in Sensor Koordinaten 
				//Input are the Coordinates in the Sensor-Environment 
				obj->mutable_main_sign()->mutable_base()->mutable_position()->set_x(xxxKoordinate);
				obj->mutable_main_sign()->mutable_base()->mutable_position()->set_y(yyyKoordinate);
				obj->mutable_main_sign()->mutable_base()->mutable_position()->set_z(zzzKoordinate);
				obj->mutable_main_sign()->mutable_base()->mutable_orientation()->set_pitch(R1);
				obj->mutable_main_sign()->mutable_base()->mutable_orientation()->set_roll(R3);
				obj->mutable_main_sign()->mutable_base()->mutable_orientation()->set_yaw(R2);


				obj->mutable_main_sign()->mutable_base()->mutable_dimension()->set_length(tsobj.main_sign().base().dimension().length());
				obj->mutable_main_sign()->mutable_base()->mutable_dimension()->set_width(tsobj.main_sign().base().dimension().width());
				obj->mutable_main_sign()->mutable_base()->mutable_dimension()->set_height(tsobj.main_sign().base().dimension().height());

				normal_log("DEBUG", "Detected traffic sign %d minimal distance to Ego/Sensor %f", its, distance);
				normal_log("DEBUG", "Detected traffic sign %d angle to Ego/Sensor %f", its, angle_to_traffic_sign);


				//osi3::DetectedTrafficSign::DetectedMainSign::CandidateMainSign* candidate = obj->mutable_main_sign->add_candidate();


				//candidate->set_type(stobj.type());
				//candidate->mutable_classification()->CopyFrom(trafficlight.classification());
				//candidate->set_probability(1);

				//normal_log("DEBUG", "Detected traffic light color %d", candidate->classification().color());


				normal_log("OSI", "Output traffic sign %d[%llu] Probability %f Detected traffic sign position\
								  								  									  		 						and orientation in environment coord:%f,%f,%f,  (%f,%f,%f)", its, tsobj.id().value(), obj->header().existence_probability(), tsobj.main_sign().base().position().x(), tsobj.main_sign().base().position().y(), tsobj.main_sign().base().position().z(), tsobj.main_sign().base().orientation().yaw(), tsobj.main_sign().base().orientation().pitch(), tsobj.main_sign().base().orientation().roll());
				//normal_log("OSI", "Output Object %d[%llu] Probability %f Detected  traffic sign position in sensor coord: %f,%f,%f \n", its, tsobj.id().value(), obj->header().existence_probability(), obj->main_sign.base().position().x(), obj->main_sign.base().position().y(), obj->main_sign.base().position().z());
				its++;
			}

		});

		/* Serialize */
		set_fmi_sensor_data_out(currentOut);
		set_fmi_valid(true);
		set_fmi_count(currentOut.moving_object_size());
	}
	else {
		/* We have no valid input, so no valid output */
		normal_log("OSI", "No valid input, therefore providing no valid output.");
		reset_fmi_sensor_data_out();
		set_fmi_valid(false);
		set_fmi_count(0);
	}
	return fmi2OK;
	}


fmi2Status COSMPCameraSensor::doTerm()
{
	DEBUGBREAK();

	return fmi2OK;
}

void COSMPCameraSensor::doFree()
{
	DEBUGBREAK();
}

/*
* Generic C++ Wrapper Code
*/

COSMPCameraSensor::COSMPCameraSensor(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID, fmi2String thefmuResourceLocation, const fmi2CallbackFunctions* thefunctions, fmi2Boolean thevisible, fmi2Boolean theloggingOn)
	: instanceName(theinstanceName),
	fmuType(thefmuType),
	fmuGUID(thefmuGUID),
	fmuResourceLocation(thefmuResourceLocation),
	functions(*thefunctions),
	visible(!!thevisible),
	loggingOn(!!theloggingOn),
	simulation_started(false)
{
	loggingCategories.clear();
	loggingCategories.insert("FMI");
	loggingCategories.insert("OSMP");
	loggingCategories.insert("OSI");
}

COSMPCameraSensor::~COSMPCameraSensor()
{

}


fmi2Status COSMPCameraSensor::SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[])
{
	fmi_verbose_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
	loggingOn = theloggingOn ? true : false;
	if (categories && (nCategories > 0)) {
		loggingCategories.clear();
		for (size_t i = 0; i < nCategories; i++) {
			if (categories[i] == "FMI")
				loggingCategories.insert("FMI");
			else if (categories[i] == "OSMP")
				loggingCategories.insert("OSMP");
			else if (categories[i] == "OSI")
				loggingCategories.insert("OSI");
		}
	}
	else {
		loggingCategories.clear();
		loggingCategories.insert("FMI");
		loggingCategories.insert("OSMP");
		loggingCategories.insert("OSI");
	}
	return fmi2OK;
}

fmi2Component COSMPCameraSensor::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID, fmi2String fmuResourceLocation, const fmi2CallbackFunctions* functions, fmi2Boolean visible, fmi2Boolean loggingOn)
{
	COSMPCameraSensor* myc = new COSMPCameraSensor(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);

	if (myc == NULL) {
		fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (alloc failure)",
			instanceName, fmuType, fmuGUID,
			(fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
			"FUNCTIONS", visible, loggingOn);
		return NULL;
	}

	if (myc->doInit() != fmi2OK) {
		fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = NULL (doInit failure)",
			instanceName, fmuType, fmuGUID,
			(fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
			"FUNCTIONS", visible, loggingOn);
		delete myc;
		return NULL;
	}
	else {
		fmi_verbose_log_global("fmi2Instantiate(\"%s\",%d,\"%s\",\"%s\",\"%s\",%d,%d) = %p",
			instanceName, fmuType, fmuGUID,
			(fmuResourceLocation != NULL) ? fmuResourceLocation : "<NULL>",
			"FUNCTIONS", visible, loggingOn, myc);
		return (fmi2Component)myc;
	}
}

fmi2Status COSMPCameraSensor::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime)
{
	fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
	return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status COSMPCameraSensor::EnterInitializationMode()
{
	fmi_verbose_log("fmi2EnterInitializationMode()");
	return doEnterInitializationMode();
}

fmi2Status COSMPCameraSensor::ExitInitializationMode()
{
	fmi_verbose_log("fmi2ExitInitializationMode()");
	simulation_started = true;
	return doExitInitializationMode();
}

fmi2Status COSMPCameraSensor::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
{
	fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
	return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status COSMPCameraSensor::Terminate()
{
	fmi_verbose_log("fmi2Terminate()");
	return doTerm();
}

fmi2Status COSMPCameraSensor::Reset()
{
	fmi_verbose_log("fmi2Reset()");

	doFree();
	simulation_started = false;
	return doInit();
}

void COSMPCameraSensor::FreeInstance()
{
	fmi_verbose_log("fmi2FreeInstance()");
	doFree();
}

fmi2Status COSMPCameraSensor::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
{
	fmi_verbose_log("fmi2GetReal(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_REAL_VARS)
			value[i] = real_vars[vr[i]];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
{
	fmi_verbose_log("fmi2GetInteger(...)");
	bool need_refresh = !simulation_started;
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_INTEGER_VARS) {
			if (need_refresh && (vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX || vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX)) {
				refresh_fmi_sensor_view_config_request();
				need_refresh = false;
			}
			value[i] = integer_vars[vr[i]];
		}
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
{
	fmi_verbose_log("fmi2GetBoolean(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_BOOLEAN_VARS)
			value[i] = boolean_vars[vr[i]];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
{
	fmi_verbose_log("fmi2GetString(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_STRING_VARS)
			value[i] = string_vars[vr[i]].c_str();
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
{
	fmi_verbose_log("fmi2SetReal(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_REAL_VARS)
			real_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
{
	fmi_verbose_log("fmi2SetInteger(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_INTEGER_VARS)
			integer_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
{
	fmi_verbose_log("fmi2SetBoolean(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_BOOLEAN_VARS)
			boolean_vars[vr[i]] = value[i];
		else
			return fmi2Error;
	}
	return fmi2OK;
}

fmi2Status COSMPCameraSensor::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
{
	fmi_verbose_log("fmi2SetString(...)");
	for (size_t i = 0; i < nvr; i++) {
		if (vr[i] < FMI_STRING_VARS)
			string_vars[vr[i]] = value[i];
		else
			return fmi2Error;
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

	FMI2_Export fmi2Status fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->SetDebugLogging(loggingOn, nCategories, categories);
	}

	/*
	* Functions for Co-Simulation
	*/
	FMI2_Export fmi2Component fmi2Instantiate(fmi2String instanceName,
		fmi2Type fmuType,
		fmi2String fmuGUID,
		fmi2String fmuResourceLocation,
		const fmi2CallbackFunctions* functions,
		fmi2Boolean visible,
		fmi2Boolean loggingOn)
	{
		return COSMPCameraSensor::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible, loggingOn);
	}

	FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
		fmi2Boolean toleranceDefined,
		fmi2Real tolerance,
		fmi2Real startTime,
		fmi2Boolean stopTimeDefined,
		fmi2Real stopTime)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
	}

	FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->EnterInitializationMode();
	}

	FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->ExitInitializationMode();
	}

	FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
		fmi2Real currentCommunicationPoint,
		fmi2Real communicationStepSize,
		fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
	}

	FMI2_Export fmi2Status fmi2Terminate(fmi2Component c)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->Terminate();
	}

	FMI2_Export fmi2Status fmi2Reset(fmi2Component c)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->Reset();
	}

	FMI2_Export void fmi2FreeInstance(fmi2Component c)
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		myc->FreeInstance();
		delete myc;
	}

	/*
	* Data Exchange Functions
	*/
	FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->GetReal(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->GetInteger(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->GetBoolean(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->GetString(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->SetReal(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->SetInteger(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->SetBoolean(vr, nvr, value);
	}

	FMI2_Export fmi2Status fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[])
	{
		COSMPCameraSensor* myc = (COSMPCameraSensor*)c;
		return myc->SetString(vr, nvr, value);
	}

	/*
	* Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
	*/
	FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate* FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t *size)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate* FMUstate)
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2GetDirectionalDerivative(fmi2Component c,
		const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
		const fmi2ValueReference vKnown_ref[], size_t nKnown,
		const fmi2Real dvKnown[],
		fmi2Real dvUnknown[])
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2SetRealInputDerivatives(fmi2Component c,
		const  fmi2ValueReference vr[],
		size_t nvr,
		const  fmi2Integer order[],
		const  fmi2Real value[])
	{
		return fmi2Error;
	}

	FMI2_Export fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c,
		const   fmi2ValueReference vr[],
		size_t  nvr,
		const   fmi2Integer order[],
		fmi2Real value[])
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
