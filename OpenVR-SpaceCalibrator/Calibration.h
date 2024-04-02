#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Windows.h>
#include <openvr.h>
#include <vector>
#include <deque>
#include <map>

#include "../Protocol.h"

enum class CalibrationState
{
	None,
	Begin,
	Rotation,
	Translation,
	Editing,
	Continuous,
	ContinuousStandby,
};

struct StandbyDevice {
	std::string trackingSystem;
	std::string model, serial;
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	int32_t referenceID = -1, calibratingTargetID = -1;

	StandbyDevice targetStandby, referenceStandby;

	std::map<std::string, Eigen::Vector3d> calibratedRotations;
	std::map<std::string, Eigen::Vector3d> calibratedTranslations;
	std::map<std::string, double> calibratedScales;
	std::map<uint32_t, std::string> targetTrackingSystemFromID;

	std::string referenceTrackingSystem;
	std::string calibratingTargetTrackingSystem;

	bool enabled = false;
	bool validProfile = false;
	bool clearOnLog = false;
	bool quashTargetInContinuous = false;
	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;

	float continuousCalibrationThreshold;

	protocol::AlignmentSpeedParams alignmentSpeedParams;
	bool enableStaticRecalibration;

	Eigen::AffineCompact3d refToTargetPose = Eigen::AffineCompact3d::Identity();
	bool relativePosCalibrated = false;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	vr::DriverPose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	CalibrationContext() {
		memset(devicePoses, 0, sizeof(devicePoses));
		ResetConfig();
	}

	void ResetConfig() {
		alignmentSpeedParams.thr_rot_tiny = 0.49f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_small = 0.5f * (EIGEN_PI / 180.0f);
		alignmentSpeedParams.thr_rot_large = 5.0f * (EIGEN_PI / 180.0f);

		alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0; // mm
		alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0; // mm

		alignmentSpeedParams.align_speed_tiny = 1.0f;
		alignmentSpeedParams.align_speed_small = 1.0f;
		alignmentSpeedParams.align_speed_large = 2.0f;

		continuousCalibrationThreshold = 1.5f;

		enableStaticRecalibration = true;
	}

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter;
		vr::HmdVector2_t playSpaceSize;
	} chaperone;

	void ClearLogOnMessage() {
		clearOnLog = true;
	}

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

		calibratedRotations.clear();
		calibratedTranslations.clear();
		calibratedScales.clear();
		targetTrackingSystemFromID.clear();
		referenceTrackingSystem = "";
		calibratingTargetTrackingSystem = "";
		enabled = false;
		validProfile = false;
		refToTargetPose = Eigen::AffineCompact3d::Identity();
		relativePosCalibrated = true;
	}

	void ResetCalibrationForSystem(std::string systemName)
	{
		if (!TrackingSystemHasCalibration(systemName)) {
			return;
		}
		calibratedRotations.erase(systemName);
		calibratedTranslations.erase(systemName);
		calibratedScales.erase(systemName);
	}

	bool TrackingSystemHasCalibration(std::string systemName)
	{
		return calibratedRotations.find(systemName) != calibratedRotations.end();
	}

	bool TrackingIDHasAssociatedSystem(uint32_t id) {
		return targetTrackingSystemFromID.find(id) != targetTrackingSystemFromID.end();
	}

	size_t SampleCount()
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 100;
		case SLOW:
			return 250;
		case VERY_SLOW:
			return 500;
		}
		return 100;
	}

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type) { }

		std::string str;
		int progress, target;
	};

	std::deque<Message> messages;

	void Log(const std::string &msg)
	{
		if (clearOnLog) {
			messages.clear();
			clearOnLog = false;
		}

		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		OutputDebugStringA(msg.c_str());

		messages.back().str += msg;
		std::cerr << msg;

		while (messages.size() > 15) messages.pop_front();
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}

	bool TargetPoseIsValid() const {
		return calibratingTargetID >= 0 && calibratingTargetID <= vr::k_unMaxTrackedDeviceCount
			&& devicePoses[calibratingTargetID].poseIsValid;
	}

	bool ReferencePoseIsValid() const {
		return referenceID >= 0 && referenceID <= vr::k_unMaxTrackedDeviceCount
			&& devicePoses[referenceID].poseIsValid;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
void CalibrationTick(double time);
void StartCalibration();
void StartContinuousCalibration();
void EndContinuousCalibration();
void LoadChaperoneBounds();
void ApplyChaperoneBounds();

void PushCalibrationApplyTime();
void ShowCalibrationDebug(int r, int c);
void DebugApplyRandomOffset();