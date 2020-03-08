/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

namespace MantisParams {

typedef enum {
	PARAM_AIRFRAME_TYPE_UNSUPPORTED = 0,
	PARAM_AIRFRAME_TYPE_QUAD_X4,
	PARAM_AIRFRAME_TYPE_QUAD_P4,
	PARAM_AIRFRAME_TYPE_HEX_X6,
	PARAM_AIRFRAME_TYPE_HEX_P6,
	PARAM_AIRFRAME_TYPE_OCTO_X8,
	PARAM_AIRFRAME_TYPE_OCTO_P8
} ParamsAirframeTypeList;

typedef enum {
	PARAM_TIME_UPDATED = 0,
	PARAM_TIME_CHANGE_CONFIG,
	PARAM_TIME_CHANGE_PARAMETRIC
} ParamsTime;

const char* ParamsTimeName[] {
	"PARAM_TIME_UPDATED",
	"PARAM_TIME_CHANGE_CONFIG",
	"PARAM_TIME_CHANGE_PARAMETRIC"
};

typedef enum {
	PARAM_PWM_MIN = 0,
	PARAM_PWM_MAX
} ParamsUint16;

const char* ParamsUint16Name[] {
	"PARAM_PWM_MIN",
	"PARAM_PWM_MAX"
};

typedef enum {
	PARAM_MOTOR_NUM = 0,
	PARAM_BODY_NUM,
	PARAM_JOINT_NUM,
	PARAM_JOINT_NUM_DYNAMIC
} ParamsUint64;

const char* ParamsUint64Name[] {
	"PARAM_MOTOR_NUM",
	"PARAM_BODY_NUM",
	"PARAM_JOINT_NUM",
	"PARAM_JOINT_NUM_DYNAMIC"
};

typedef enum {
	PARAM_BASE_ARM_LENGTH = 0,
	PARAM_MOTOR_MAX_THRUST,
	PARAM_MOTOR_MAX_DRAG,
	PARAM_TOTAL_MASS
} ParamsDouble;

const char* ParamsDoubleName[] {
	"PARAM_BASE_ARM_LENGTH",
	"PARAM_MOTOR_MAX_THRUST",
	"PARAM_MOTOR_MAX_DRAG",
	"PARAM_TOTAL_MASS"
};

typedef enum {
	PARAM_AIRFRAME_TYPE = 0
} ParamsAirframeType;

const char* ParamsAirframeTypeName[] {
	"PARAM_AIRFRAME_TYPE"
};

typedef enum {
	PARAM_AIRFRAME_NAME = 0,
	PARAM_MODEL_ID
} ParamsString;

const char* ParamsStringName[] {
	"PARAM_AIRFRAME_NAME"
};

typedef enum {
	PARAM_BODY_INERTIAL = 0
} ParamsBodyInertial;

const char* ParamsBodyInertialName[] {
	"PARAM_BODY_INERTIAL"
};

typedef enum {
	PARAM_JOINT_DESCRIPTION = 0
} ParamsJointDescription;

const char* ParamsJointDescriptionName[] {
	"PARAM_JOINT_DESCRIPTION"
};

typedef enum {
	PARAM_MIXER = 0
} ParamsMatrixXd;

const char* ParamsMatrixXdName[] {
	"PARAM_MIXER"
};

};
