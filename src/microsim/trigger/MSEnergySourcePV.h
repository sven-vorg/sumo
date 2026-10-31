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
/// @file    MSEnergySourcePV.h
/// @author
/// @date    2026-10-30
///
// PV energy source of a charging station, driven by a list of energy values per time frame
/****************************************************************************/
#pragma once
#include <config.h>

#include <memory>
#include <string>
#include <vector>
#include <utils/common/SUMOTime.h>


// ===========================================================================
// class declarations
// ===========================================================================
class Parameterised;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSEnergySourcePV
 * @brief PV generation at a charging station
 *
 * The generation is given as a list of energy values, one per time frame (default: one
 *  per hour), starting at simulation time 0. Each simulation step is assigned completely
 *  to one time frame by its time t: frame k covers (k * timeFrame, (k + 1) * timeFrame],
 *  e.g. for hourly frames the steps at 1..3600 s use position 0, 3601..7200 s position 1.
 *  This matches the periods of the stationary storage output. A step receives
 *  value * stepLength / timeFrame. Outside of the list (time <= 0 or after its end)
 *  nothing is generated.
 *
 * Every simulation step, before the stationary storage is advanced, energyGenerated()
 *  is called and its result handed to the storage as energyReceived.
 *
 * Configured via the following parameters of the charging station:
 *  - pv.energyValueList  space-separated energy generated per time frame [Wh] (mandatory, enables the source)
 *  - pv.timeFrame        length of a time frame, in s or as time string (default 3600)
 */
class MSEnergySourcePV {
public:
    /** @brief Builds a PV source from the parameters of a charging station
     * @param[in] chargingStationID The id of the charging station
     * @param[in] params The parameters of the charging station
     * @return the PV source or nullptr if pv.energyValueList is not given
     * @exception ProcessError if a parameter is invalid
     */
    static std::unique_ptr<MSEnergySourcePV> buildFromParams(const std::string& chargingStationID, const Parameterised& params);

    /// @brief constructor
    MSEnergySourcePV(const std::string& chargingStationID, const std::vector<double>& energyValueList, SUMOTime timeFrame);

    /// @brief destructor
    ~MSEnergySourcePV();

    /** @brief Get the energy value of the time frame containing the given time
     * @param[in] time The simulation time
     * @return the energy of the whole time frame [Wh], 0 outside of the list
     */
    double getCurrentValue(SUMOTime time) const;

    /** @brief Computes the energy generated in the step beginning at currentTime
     * @param[in] currentTime The begin of the step
     * @param[in] stepLength The length of the step
     * @return the energy generated [Wh]
     */
    double energyGenerated(SUMOTime currentTime, SUMOTime stepLength);

    /// @name Getters
    /// @{
    const std::vector<double>& getEnergyValueList() const {
        return myEnergyValueList;
    }

    SUMOTime getTimeFrame() const {
        return myTimeFrame;
    }

    /// @brief energy generated in the last call of energyGenerated() [Wh]
    double getLastEnergyGenerated() const {
        return myLastEnergyGenerated;
    }
    /// @}

private:
    /// @brief id of the charging station (for messages)
    const std::string myChargingStationID;

    /// @brief energy generated per time frame [Wh]
    const std::vector<double> myEnergyValueList;

    /// @brief length of a time frame
    const SUMOTime myTimeFrame;

    /// @brief energy generated in the last step [Wh]
    double myLastEnergyGenerated;

    /// @brief whether the warning about running past the end of the list was given
    bool myWarnedEndOfList;

private:
    /// @brief Invalidated copy constructor.
    MSEnergySourcePV(const MSEnergySourcePV&) = delete;

    /// @brief Invalidated assignment operator.
    MSEnergySourcePV& operator=(const MSEnergySourcePV&) = delete;
};
