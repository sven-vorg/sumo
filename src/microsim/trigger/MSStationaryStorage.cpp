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
/// @file    MSStationaryStorage.cpp
/// @author
/// @date    2026-10-30
///
// Stationary energy storage built into a charging station
/****************************************************************************/
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/Parameterised.h>
#include <utils/common/StdDefs.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>
#include <utils/iodevices/OutputDevice.h>
#include "MSStationaryStorage.h"


// ===========================================================================
// EnergyFlows
// ===========================================================================
MSStationaryStorage::EnergyFlows&
MSStationaryStorage::EnergyFlows::operator+=(const EnergyFlows& other) {
    energyDrawGrid += other.energyDrawGrid;
    energyReceived += other.energyReceived;
    energyDischargedFromStorage += other.energyDischargedFromStorage;
    receivedEnergyToStorage += other.receivedEnergyToStorage;
    receivedEnergyToVehicle += other.receivedEnergyToVehicle;
    drawnEnergyToStorage += other.drawnEnergyToStorage;
    drawnEnergyToVehicle += other.drawnEnergyToVehicle;
    curtailedEnergy += other.curtailedEnergy;
    return *this;
}


// ===========================================================================
// static method definitions
// ===========================================================================
std::unique_ptr<MSStationaryStorage>
MSStationaryStorage::buildFromParams(const std::string& chargingStationID, const Parameterised& params) {
    if (!params.hasParameter("ess.maximumStorageCapacity")) {
        return nullptr;
    }
    const std::string id = params.getParameter("ess.id", chargingStationID + "_storage");
    const double maximumStorageCapacity = readDouble(chargingStationID, params, "ess.maximumStorageCapacity", 0);
    const double stateOfChargeStartUp = readDouble(chargingStationID, params, "ess.stateOfChargeStartUp", 0);
    const double drawTarget = readDouble(chargingStationID, params, "ess.drawTarget", 0);
    const double drawPower = readDouble(chargingStationID, params, "ess.drawPower", 0);

    if (maximumStorageCapacity < 0) {
        throw ProcessError(TLF("Parameter 'ess.maximumStorageCapacity' of chargingStation '%' must not be negative (%).", chargingStationID, toString(maximumStorageCapacity)));
    }
    if (stateOfChargeStartUp < 0 || stateOfChargeStartUp > 1) {
        throw ProcessError(TLF("Parameter 'ess.stateOfChargeStartUp' of chargingStation '%' must be within [0, 1] (%).", chargingStationID, toString(stateOfChargeStartUp)));
    }
    if (drawTarget < 0 || drawTarget > 1) {
        throw ProcessError(TLF("Parameter 'ess.drawTarget' of chargingStation '%' must be within [0, 1] (%).", chargingStationID, toString(drawTarget)));
    }
    if (drawPower < 0) {
        throw ProcessError(TLF("Parameter 'ess.drawPower' of chargingStation '%' must not be negative (%).", chargingStationID, toString(drawPower)));
    }
    return std::unique_ptr<MSStationaryStorage>(new MSStationaryStorage(id, chargingStationID, maximumStorageCapacity,
            stateOfChargeStartUp, drawTarget, drawPower));
}


double
MSStationaryStorage::readDouble(const std::string& chargingStationID, const Parameterised& params, const std::string& key, double defaultValue) {
    if (!params.hasParameter(key)) {
        return defaultValue;
    }
    const std::string value = params.getParameter(key);
    try {
        return StringUtils::toDouble(value);
    } catch (NumberFormatException&) {
        throw ProcessError(TLF("Invalid value '%' for parameter '%' of chargingStation '%'.", value, key, chargingStationID));
    } catch (EmptyData&) {
        throw ProcessError(TLF("Empty value for parameter '%' of chargingStation '%'.", key, chargingStationID));
    }
}


void
MSStationaryStorage::writeFlows(OutputDevice& out, const EnergyFlows& flows, const std::string& suffix) {
    out.writeAttr("energyDrawGrid" + suffix, flows.energyDrawGrid);
    out.writeAttr("energyReceived" + suffix, flows.energyReceived);
    out.writeAttr("energyDischargedFromStorage" + suffix, flows.energyDischargedFromStorage);
    out.writeAttr("receivedEnergyToStorage" + suffix, flows.receivedEnergyToStorage);
    out.writeAttr("receivedEnergyToVehicle" + suffix, flows.receivedEnergyToVehicle);
    out.writeAttr("drawnEnergyToStorage" + suffix, flows.drawnEnergyToStorage);
    out.writeAttr("drawnEnergyToVehicle" + suffix, flows.drawnEnergyToVehicle);
    out.writeAttr("curtailedEnergy" + suffix, flows.curtailedEnergy);
}


// ===========================================================================
// member method definitions
// ===========================================================================
MSStationaryStorage::MSStationaryStorage(const std::string& id, const std::string& chargingStationID, double maximumStorageCapacity,
        double stateOfChargeStartUp, double drawTarget, double drawPower) :
    myID(id),
    myChargingStationID(chargingStationID),
    myMaximumStorageCapacity(maximumStorageCapacity),
    myStateOfChargeStartUp(stateOfChargeStartUp),
    myDrawTarget(drawTarget),
    myDrawPower(drawPower),
    myStorageCapacity(stateOfChargeStartUp * maximumStorageCapacity) {
}


MSStationaryStorage::~MSStationaryStorage() {}


void
MSStationaryStorage::stationaryStorageLoop(double energyRequested, double energyReceived, double stepLength) {
    myStep = EnergyFlows();
    myStep.energyReceived = energyReceived;
    if (!directEnergyUtilisation(energyRequested, energyReceived)) {
        if (!storageSupportedEnergy(energyRequested, energyReceived)) {
            gridSupportedEnergy(energyRequested, energyReceived);
        }
    }
    passiveDraw(stepLength);
    myStep.energyDrawGrid = myStep.drawnEnergyToStorage + myStep.drawnEnergyToVehicle;
    myPeriod += myStep;
    myTotal += myStep;
}


bool
MSStationaryStorage::directEnergyUtilisation(double energyRequested, double energyReceived) {
    if (energyRequested > energyReceived) {
        return false;
    }
    const double surplus = energyReceived - energyRequested;
    const double stored = MIN2(surplus, myMaximumStorageCapacity - myStorageCapacity);
    myStorageCapacity += stored;
    myStep.receivedEnergyToVehicle = energyRequested;
    myStep.receivedEnergyToStorage = stored;
    myStep.curtailedEnergy = surplus - stored;
    return true;
}


bool
MSStationaryStorage::storageSupportedEnergy(double energyRequested, double energyReceived) {
    const double remainingEnergyDemand = energyRequested - energyReceived;
    if (myStorageCapacity < remainingEnergyDemand) {
        return false;
    }
    myStorageCapacity -= remainingEnergyDemand;
    myStep.receivedEnergyToVehicle = energyReceived;
    myStep.energyDischargedFromStorage = remainingEnergyDemand;
    return true;
}


void
MSStationaryStorage::gridSupportedEnergy(double energyRequested, double energyReceived) {
    const double remainingEnergyDemand = energyRequested - energyReceived - myStorageCapacity;
    myStep.receivedEnergyToVehicle = energyReceived;
    myStep.energyDischargedFromStorage = myStorageCapacity;
    myStorageCapacity = 0;
    myStep.drawnEnergyToVehicle = remainingEnergyDemand;
}


void
MSStationaryStorage::passiveDraw(double stepLength) {
    const double target = myDrawTarget * myMaximumStorageCapacity;
    if (myDrawPower <= 0 || myStorageCapacity >= target) {
        return;
    }
    const double powerDraw = MIN2(myDrawPower * stepLength / 3600., target - myStorageCapacity);
    myStorageCapacity += powerDraw;
    myStep.drawnEnergyToStorage = powerDraw;
}


void
MSStationaryStorage::writeOutput(OutputDevice& out) {
    out.openTag("station");
    out.writeAttr("id", myID);
    out.writeAttr("chargingStationId", myChargingStationID);
    out.writeAttr("maximumStorageCapacity", myMaximumStorageCapacity);
    out.writeAttr("stateOfChargeStartUp", myStateOfChargeStartUp);
    out.writeAttr("drawTarget", myDrawTarget);
    out.writeAttr("drawPower", myDrawPower);
    out.writeAttr("storageCapacity", myStorageCapacity);
    writeFlows(out, myPeriod, "");
    writeFlows(out, myTotal, "Total");
    out.closeTag();
    // the next output starts a new period
    myPeriod = EnergyFlows();
}


/****************************************************************************/
