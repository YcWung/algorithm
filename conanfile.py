################################################################################
# @author Yongchao Wang (ycw.puzzle@hotmail.com)
# @brief
# @version 0.1
# @date 2022-10-07
#
# @copyright Copyright (c) 2022
#
################################################################################

from conans import ConanFile, CMake


class AlgoirthmConanFile(ConanFile):
    name = "algorithm"
    version = "0.0.1"
    description = "Minimal implementation of common algorithms"
    settings = ["os", "compiler", "build_type", "arch"]
    options = {"shared": [True, False], "fPIC": [True, False]}
    default_options = {"shared": True, "fPIC": True}
    generators = "cmake_find_package"

    def requirements(self):
        self.requires("eigen/3.3.9")
        self.requires("gtest/1.11.0", private=True)

    def config_options(self):
        if self.settings.os == "Windows":
            del self.options.fPIC
