#include "stdafx.h"
#include "Configuration.h"

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, int numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array, got " + obj.to_str());

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");

	for (int i = 0; i < numFloats; i++)
		buf[i] = (float) arr[i].get<double>();
}

static void LoadStandby(StandbyDevice& device, picojson::value& value) {
	if (!value.is<picojson::object>()) return;
	auto& obj = value.get<picojson::object>();
	
	const auto &system = obj["tracking_system"];
	if (system.is<std::string>()) device.trackingSystem = system.get<std::string>();

	const auto& model = obj["model"];
	if (model.is<std::string>()) device.model = model.get<std::string>();

	const auto& serial = obj["serial"];
	if (serial.is<std::string>()) device.serial = serial.get<std::string>();
}

static void VisitAlignmentParams(CalibrationContext& ctx, std::function<void(const char *, double&)> MapParam) {
#define P(s) MapParam(#s, ctx.alignmentSpeedParams.s)
	P(align_speed_tiny);
	P(align_speed_small);
	P(align_speed_large);
	P(thr_trans_tiny);
	P(thr_trans_small);
	P(thr_trans_large);
	P(thr_rot_tiny);
	P(thr_rot_small);
	P(thr_rot_large);
	
	// Convert to double and back
	double tmp = ctx.continuousCalibrationThreshold;
	MapParam("continuousCalibrationThreshold", tmp);
	ctx.continuousCalibrationThreshold = (float)tmp;
}

static void LoadAlignmentParams(CalibrationContext& ctx, picojson::value& value) {
	ctx.ResetConfig();
	
	if (!value.is<picojson::object>()) return;
	auto& obj = value.get<picojson::object>();
	
	VisitAlignmentParams(ctx, [&](auto name, auto& param) {
		const picojson::value& node = obj[name];
		if (node.is<double>()) {
			param = (float)node.get<double>();
		}
	});
}

static picojson::object SaveAlignmentParams(CalibrationContext& ctx) {
	picojson::object obj;

	VisitAlignmentParams(ctx, [&](auto name, auto& param) {
		obj[name].set<double>(param);
	});

	return obj;
}

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	auto arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	auto obj = arr[0].get<picojson::object>();

	LoadAlignmentParams(ctx, obj["alignment_params"]);
	ctx.referenceTrackingSystem = obj["reference_tracking_system"].get<std::string>();
	ctx.calibratingTargetTrackingSystem = obj["target_tracking_system"].get<std::string>();

	picojson::object calibrations = obj["calibrated_spaces"].get<picojson::object>();
	for (const auto& pair : calibrations) {
		std::string n = pair.first;
		picojson::object s = pair.second.get<picojson::object>();

		ctx.calibratedRotations[n] = Eigen::Vector3d();
		ctx.calibratedTranslations[n] = Eigen::Vector3d();

		ctx.calibratedRotations[n](0) = s["roll"].get<double>();
		ctx.calibratedRotations[n](1) = s["yaw"].get<double>();
		ctx.calibratedRotations[n](2) = s["pitch"].get<double>();
		ctx.calibratedTranslations[n](0) = s["x"].get<double>();
		ctx.calibratedTranslations[n](1) = s["y"].get<double>();
		ctx.calibratedTranslations[n](2) = s["z"].get<double>();

		if (s["scale"].is<double>())
			ctx.calibratedScales[n] = s["scale"].get<double>();
		else
			ctx.calibratedScales[n] = 1.0;

		if (ctx.calibratedScales[n] <= 0.001) {
			ctx.calibratedScales[n] = 1.0;
		}
	}
	LoadStandby(ctx.referenceStandby, obj["reference_device"]);
	LoadStandby(ctx.targetStandby, obj["target_device"]);
	if (obj["autostart_continuous_calibration"].evaluate_as_boolean()) {
		ctx.state = CalibrationState::ContinuousStandby;
	}
	ctx.quashTargetInContinuous = obj["quash_target_in_continuous"].evaluate_as_boolean();

	if (obj["calibration_speed"].is<double>())
		ctx.calibrationSpeed = (CalibrationContext::Speed)(int) obj["calibration_speed"].get<double>();

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		ctx.chaperone.autoApply = chaperone["auto_apply"].get<bool>();

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>())
			throw std::runtime_error("chaperone geometry is not an array");

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		if (geometry.size() > 0)
		{
			ctx.chaperone.geometry.resize(geometry.size() * sizeof(float) / sizeof(ctx.chaperone.geometry[0]));
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), geometry.size());

			ctx.chaperone.valid = true;
		}
	}
	if (obj["relative_pos_calibrated"].is<bool>()) {
		ctx.relativePosCalibrated = obj["relative_pos_calibrated"].get<bool>();
	}
	if (obj["relative_transform"].is<picojson::object>()) {
		auto relTransform = obj["relative_transform"].get<picojson::object>();
		Eigen::Vector3d refToTargetRotation;
		Eigen::Vector3d refToTargetTranslation;

		refToTargetRotation(0) = relTransform["roll"].get<double>();
		refToTargetRotation(1) = relTransform["yaw"].get<double>();
		refToTargetRotation(2) = relTransform["pitch"].get<double>();
		refToTargetTranslation(0) = relTransform["x"].get<double>();
		refToTargetTranslation(1) = relTransform["y"].get<double>();
		refToTargetTranslation(2) = relTransform["z"].get<double>();

        Eigen::Matrix3d rotationMatrix;
        rotationMatrix =
            Eigen::AngleAxisd(refToTargetRotation[0], Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(refToTargetRotation[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(refToTargetRotation[2], Eigen::Vector3d::UnitZ());

		ctx.refToTargetPose = Eigen::AffineCompact3d::Identity();
        ctx.refToTargetPose.linear() = rotationMatrix;
        ctx.refToTargetPose.translation() = refToTargetTranslation;
	}

	ctx.validProfile = ctx.TrackingSystemHasCalibration(ctx.calibratingTargetTrackingSystem);
}


static void WriteStandby(StandbyDevice& device, picojson::value& value) {
	auto obj = picojson::object();

	obj["tracking_system"].set<std::string>(device.trackingSystem);
	obj["model"].set<std::string>(device.model);
	obj["serial"].set<std::string>(device.serial);

	value.set<picojson::object>(obj);
}


static void WriteProfile(CalibrationContext &ctx, std::ostream &out)
{
	if (!ctx.validProfile)
		return;

	picojson::object profile;
	profile["alignment_params"].set<picojson::object>(SaveAlignmentParams(ctx));
	
	profile["reference_tracking_system"].set<std::string>(ctx.referenceTrackingSystem);
	profile["target_tracking_system"].set<std::string>(ctx.calibratingTargetTrackingSystem);
	
	picojson::object calibrations;
	for (const auto& pair : ctx.calibratedRotations) {
		picojson::object s;
		std::string n = pair.first;

		s["roll"].set<double>(ctx.calibratedRotations[n](0));
		s["yaw"].set<double>(ctx.calibratedRotations[n](1));
		s["pitch"].set<double>(ctx.calibratedRotations[n](2));
		s["x"].set<double>(ctx.calibratedTranslations[n](0));
		s["y"].set<double>(ctx.calibratedTranslations[n](1));
		s["z"].set<double>(ctx.calibratedTranslations[n](2));
		s["scale"].set<double>(ctx.calibratedScales[n]);

		calibrations[n].set<picojson::object>(s);
	}
	profile["calibrated_spaces"].set<picojson::object>(calibrations);
	WriteStandby(ctx.referenceStandby, profile["reference_device"]);
	WriteStandby(ctx.targetStandby, profile["target_device"]);
	bool isInContinuousCalibrationMode = ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby;
	profile["autostart_continuous_calibration"].set<bool>(isInContinuousCalibrationMode);
	profile["quash_target_in_continuous"].set<bool>(ctx.quashTargetInContinuous);

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	if (ctx.chaperone.valid)
	{
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.geometry.data(),
			sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size()
		));

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	Eigen::Vector3d refToTragetRoation = ctx.refToTargetPose.rotation().eulerAngles(0, 1, 2);
	Eigen::Vector3d refToTargetTranslation = ctx.refToTargetPose.translation();
	picojson::object refToTarget;
	refToTarget["x"].set<double>(refToTargetTranslation(0));
	refToTarget["y"].set<double>(refToTargetTranslation(1));
	refToTarget["z"].set<double>(refToTargetTranslation(2));
	refToTarget["roll"].set<double>(refToTragetRoation(0));
	refToTarget["yaw"].set<double>(refToTragetRoation(1));
	refToTarget["pitch"].set<double>(refToTragetRoation(2));
	profile["relative_pos_calibrated"].set<bool>(ctx.relativePosCalibrated);
	profile["relative_transform"].set<picojson::object>(refToTarget);

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

static void LogRegistryResult(LSTATUS result)
{
	char *message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT, (LPSTR)&message, 0, NULL);
	std::cerr << "Opening registry key: " << message << std::endl;
}

static const char *RegistryKey = "Software\\OpenVR-SpaceCalibrator";

static std::string ReadRegistryKey()
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}
	
	str.resize(size - 1);
	return str;
}

static void WriteRegistryKey(std::string str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return;
	}

	DWORD size = str.size() + 1;

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "")
	{
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
		return;
	}

	try
	{
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Error loading profile: " << e.what() << std::endl;
	}
}

void SaveProfile(CalibrationContext &ctx)
{
	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	WriteRegistryKey(io.str());
}
