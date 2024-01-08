#pragma once

#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <string>

std::string getMyPackagePath();

std::string getPackagePath(const std::string & package_name);
