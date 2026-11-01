/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2001-2026 German Aerospace Center (DLR) and others.
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// https://www.eclipse.org/legal/epl-2.0/
// This Source Code may also be made available under the following Secondary
// Licenses when the conditions for such availability set forth in the Eclipse
// Public License 2.0 are satisfied: GNU General Public License, version 2
// or later which is available at
// https://www.gnu.org/licenses/old-licenses/gpl-2.0-standalone.html
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
/// @file    MSStationaryEnergyStorageExport.cpp
/// @author
/// @date    2026-10-30
///
// Realises dumping the state of stationary energy storages at charging stations
/****************************************************************************/
#include <config.h>

#include <utils/iodevices/OutputDevice.h>
#include <microsim/MSNet.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/trigger/MSStationaryEnergyStorage.h>
#include "MSStationaryEnergyStorageExport.h"


// ===========================================================================
// method definitions
// ===========================================================================
void
MSStationaryEnergyStorageExport::write(OutputDevice& of, SUMOTime timestep, int precision) {
    of.setPrecision(precision);
    of.openTag(SUMO_TAG_TIMESTEP).writeTime(SUMO_ATTR_TIME, timestep);
    for (const auto& stop : MSNet::getInstance()->getStoppingPlaces(SUMO_TAG_CHARGING_STATION)) {
        MSStationaryEnergyStorage* const storage = static_cast<MSChargingStation*>(stop.second)->getStationaryEnergyStorage();
        if (storage != nullptr) {
            storage->writeOutput(of);
        }
    }
    of.closeTag();
}
