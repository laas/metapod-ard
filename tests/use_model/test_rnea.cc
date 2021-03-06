// Copyright 2013
//
// Olivier STASSE (LAAS/CNRS)
//
// This file is part of metapod-ard.
// metapod-ard is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod-ard is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod-ard.  If not, see <http://www.gnu.org/licenses/>.

// This test applies the rnea on a test model with a reference configuration,
// then compares the computed torques with the reference torques
// Common test tools

#include <common.hh>
#include <metapod-ard/dynamicrobot.hh>
#include "include_model.hh"
namespace msa=metapod;

BOOST_AUTO_TEST_CASE (test_rnea)
{
  current_model_ard aRobot;
}
