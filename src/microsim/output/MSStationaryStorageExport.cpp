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
/// @file    MSStationaryStorageExport.cpp
/// @author
/// @date    2026-10-30
///
// Realises dumping the state of stationary storages at charging stations
/****************************************************************************/
#include <config.h>

#include <utils/iodevices/OutputDevice.h>
#include <microsim/MSNet.h>
#include <microsim/trigger/MSChargingStation.h>
#include <microsim/trigger/MSStationaryStorage.h>
#include "MSStationaryStorageExport.h"


// ===========================================================================
// method definitions
// ===========================================================================
void
MSStationaryStorageExport::write(OutputDevice& of, SUMOTime timestep, int precision) {
    of.setPrecision(precision);
    of.openTag(SUMO_TAG_TIMESTEP).writeAttr(SUMO_ATTR_TIME, time2string(timestep));
    for (const auto& stop : MSNet::getInstance()->getStoppingPlaces(SUMO_TAG_CHARGING_STATION)) {
        MSStationaryStorage* const storage = static_cast<MSChargingStation*>(stop.second)->getStationaryStorage();
        if (storage != nullptr) {
            storage->writeOutput(of);
        }
    }
    of.closeTag();
}
