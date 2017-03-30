#pragma once

#include <yaml-cpp/yaml.h>

#include "extrinsics_array/ExtrinsicsCommon.h"

namespace argus
{

/*! \brief Parses a calibration from a YAML object. Returns success. */
bool ParseExtrinsicsCalibration( const YAML::Node& yaml, std::vector<RelativePose>& poses );

/*! \brief Reads an extrinsics calibration from a YAML file. Returns success. */
void PopulateExtrinsicsCalibration( const std::vector<RelativePose>& poses, YAML::Node& yaml );

/*! \brief Populates a YAML node from a extrinsics calibration. */
bool ReadExtrinsicsCalibration( const std::string& path, std::vector<RelativePose>& poses );

/*! \brief Writes an extrinsics calibration to a YAML file. Returns success. */
bool WriteExtrinsicsCalibration( const std::string& path, const std::vector<RelativePose>& poses );

}