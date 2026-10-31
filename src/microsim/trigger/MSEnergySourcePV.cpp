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
/// @file    MSEnergySourcePV.cpp
/// @author
/// @date    2026-10-30
///
// PV energy source of a charging station, driven by a list of energy values per time frame
/****************************************************************************/
#include <config.h>

#include <utils/common/MsgHandler.h>
#include <utils/common/Parameterised.h>
#include <utils/common/StringTokenizer.h>
#include <utils/common/StringUtils.h>
#include <utils/common/ToString.h>
#include <utils/common/UtilExceptions.h>
#include "MSEnergySourcePV.h"


// ===========================================================================
// static method definitions
// ===========================================================================
std::unique_ptr<MSEnergySourcePV>
MSEnergySourcePV::buildFromParams(const std::string& chargingStationID, const Parameterised& params) {
    if (!params.hasParameter("pv.energyValueList")) {
        return nullptr;
    }
    std::vector<double> energyValueList;
    for (const std::string& value : StringTokenizer(params.getParameter("pv.energyValueList")).getVector()) {
        try {
            energyValueList.push_back(StringUtils::toDouble(value));
        } catch (ProcessError&) {
            throw ProcessError(TLF("Invalid value '%' in parameter 'pv.energyValueList' of chargingStation '%'.", value, chargingStationID));
        }
        if (energyValueList.back() < 0) {
            throw ProcessError(TLF("Parameter 'pv.energyValueList' of chargingStation '%' contains a negative value (%).", chargingStationID, value));
        }
    }
    if (energyValueList.empty()) {
        throw ProcessError(TLF("Parameter 'pv.energyValueList' of chargingStation '%' is empty.", chargingStationID));
    }

    SUMOTime timeFrame = TIME2STEPS(3600);
    if (params.hasParameter("pv.timeFrame")) {
        const std::string value = params.getParameter("pv.timeFrame");
        try {
            timeFrame = string2time(value);
        } catch (ProcessError&) {
            throw ProcessError(TLF("Invalid value '%' for parameter 'pv.timeFrame' of chargingStation '%'.", value, chargingStationID));
        }
        if (timeFrame <= 0) {
            throw ProcessError(TLF("Parameter 'pv.timeFrame' of chargingStation '%' must be positive (%).", chargingStationID, value));
        }
    }
    return std::unique_ptr<MSEnergySourcePV>(new MSEnergySourcePV(chargingStationID, energyValueList, timeFrame));
}


// ===========================================================================
// member method definitions
// ===========================================================================
MSEnergySourcePV::MSEnergySourcePV(const std::string& chargingStationID, const std::vector<double>& energyValueList, SUMOTime timeFrame) :
    myChargingStationID(chargingStationID),
    myEnergyValueList(energyValueList),
    myTimeFrame(timeFrame),
    myLastEnergyGenerated(0),
    myWarnedEndOfList(false) {
}


MSEnergySourcePV::~MSEnergySourcePV() {}


double
MSEnergySourcePV::getCurrentValue(SUMOTime time) const {
    if (time <= 0) {
        return 0;
    }
    // frame k covers (k * timeFrame, (k + 1) * timeFrame], SUMOTime is integer [ms]
    const SUMOTime index = (time - 1) / myTimeFrame;
    if (index >= (SUMOTime)myEnergyValueList.size()) {
        return 0;
    }
    return myEnergyValueList[(int)index];
}


double
MSEnergySourcePV::energyGenerated(SUMOTime currentTime, SUMOTime stepLength) {
    // the whole step belongs to the time frame containing its time
    myLastEnergyGenerated = getCurrentValue(currentTime) * (double)stepLength / (double)myTimeFrame;
    const SUMOTime endOfList = myTimeFrame * (SUMOTime)myEnergyValueList.size();
    if (!myWarnedEndOfList && currentTime > endOfList) {
        WRITE_WARNINGF(TL("PV energy value list of chargingStation '%' ends at time %; no PV energy is generated afterwards."),
                       myChargingStationID, time2string(endOfList));
        myWarnedEndOfList = true;
    }
    return myLastEnergyGenerated;
}


/****************************************************************************/
