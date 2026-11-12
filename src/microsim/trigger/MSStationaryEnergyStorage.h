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
/// @file    MSStationaryEnergyStorage.h
/// @author
/// @date    2026-10-30
///
// Stationary energy storage built into a charging station
/****************************************************************************/
#pragma once
#include <config.h>

#include <memory>
#include <string>


// ===========================================================================
// class declarations
// ===========================================================================
class OutputDevice;
class Parameterised;


// ===========================================================================
// class definitions
// ===========================================================================
/**
 * @class MSStationaryEnergyStorage
 * @brief Battery built into a charging station
 *
 * The storage does not generate energy itself. Once per simulation step it is handed
 *  the energy requested by the vehicles and the energy received from another source
 *  (e.g. PV) and distributes it in this order:
 *   1. received energy -> vehicles, surplus -> storage (rest is curtailed)
 *   2. storage -> vehicles for what the received energy could not cover
 *   3. grid -> vehicles for what the storage could not cover
 *   4. passive draw: grid -> storage while below drawTarget
 *
 * All energies are in Wh, all powers in W.
 *
 * Configured via the following parameters of the charging station:
 *  - ses.maximumStorageCapacity  capacity [Wh] (mandatory, enables the storage)
 *  - ses.stateOfChargeStartUp    initial state of charge as fraction of capacity [0..1] (default 0)
 *  - ses.drawTarget              passive draw until this fraction of capacity [0..1] (default 0)
 *  - ses.drawPower               passive draw power [W] (default 0)
 *  - ses.id                      id of the storage (default <chargingStationID>_ses)
 */
class MSStationaryEnergyStorage {
public:
    /// @brief energy flows [Wh], used for the current step and for the accumulated totals
    struct EnergyFlows {
        /// @brief drawnEnergyToStorage + drawnEnergyToVehicle
        double energyDrawGrid = 0;
        /// @brief handed over by the energy source
        double energyReceived = 0;
        /// @brief storage -> vehicle
        double energyDischargedFromStorage = 0;
        /// @brief source -> storage
        double receivedEnergyToStorage = 0;
        /// @brief source -> vehicle
        double receivedEnergyToVehicle = 0;
        /// @brief grid -> storage (passive draw)
        double drawnEnergyToStorage = 0;
        /// @brief grid -> vehicle
        double drawnEnergyToVehicle = 0;
        /// @brief received, but neither used nor storable
        double curtailedEnergy = 0;

        EnergyFlows& operator+=(const EnergyFlows& other);
    };

    /** @brief Builds a storage from the parameters of a charging station
     * @param[in] chargingStationID The id of the charging station
     * @param[in] params The parameters of the charging station
     * @return the storage or nullptr if ses.maximumStorageCapacity is not given
     * @exception ProcessError if a parameter is invalid
     */
    static std::unique_ptr<MSStationaryEnergyStorage> buildFromParams(const std::string& chargingStationID, const Parameterised& params);

    /// @brief constructor
    MSStationaryEnergyStorage(const std::string& id, const std::string& chargingStationID, double maximumStorageCapacity,
                              double stateOfChargeStartUp, double drawTarget, double drawPower);

    /// @brief destructor
    ~MSStationaryEnergyStorage();

    /** @brief Advances the storage by one simulation step
     * @param[in] energyRequested The energy requested by vehicles during this step [Wh]
     * @param[in] energyReceived The energy received from the energy source during this step [Wh]
     * @param[in] stepLength The length of the step [s]
     */
    void stationaryEnergyStorageLoop(double energyRequested, double energyReceived, double stepLength);

    /** @brief Writes the current state as a <station> element
     *
     * The flows without suffix are summed up since the previous call (one step if
     *  written every step), the flows with suffix "Total" since the simulation start.
     *  Starts a new output period afterwards.
     */
    void writeOutput(OutputDevice& out);

    /// @name Getters
    /// @{
    const std::string& getID() const {
        return myID;
    }

    const std::string& getChargingStationID() const {
        return myChargingStationID;
    }

    double getMaximumStorageCapacity() const {
        return myMaximumStorageCapacity;
    }

    /// @brief current state of charge [Wh]
    double getStorageCapacity() const {
        return myStorageCapacity;
    }

    /// @brief current state of charge as fraction of capacity [0..1]
    double getStateOfCharge() const {
        return myMaximumStorageCapacity > 0 ? myStorageCapacity / myMaximumStorageCapacity : 0.;
    }

    const EnergyFlows& getStepFlows() const {
        return myStep;
    }

    /// @brief flows summed up since the last output
    const EnergyFlows& getPeriodFlows() const {
        return myPeriod;
    }

    const EnergyFlows& getTotalFlows() const {
        return myTotal;
    }
    /// @}

    /// @name Single flows of the last step [Wh], e.g. for FunctionBinding in the GUI
    /// @{
    double getStepEnergyDrawGrid() const {
        return myStep.energyDrawGrid;
    }

    double getStepEnergyReceived() const {
        return myStep.energyReceived;
    }

    double getStepEnergyDischargedFromStorage() const {
        return myStep.energyDischargedFromStorage;
    }

    double getStepReceivedEnergyToStorage() const {
        return myStep.receivedEnergyToStorage;
    }

    double getStepDrawnEnergyToStorage() const {
        return myStep.drawnEnergyToStorage;
    }

    double getStepCurtailedEnergy() const {
        return myStep.curtailedEnergy;
    }
    /// @}

    /// @name Single accumulated flows since the simulation start [Wh]
    /// @{
    double getTotalEnergyDrawGrid() const {
        return myTotal.energyDrawGrid;
    }

    double getTotalEnergyReceived() const {
        return myTotal.energyReceived;
    }

    double getTotalEnergyDischargedFromStorage() const {
        return myTotal.energyDischargedFromStorage;
    }

    double getTotalCurtailedEnergy() const {
        return myTotal.curtailedEnergy;
    }
    /// @}

private:
    /// @brief received energy covers the whole demand, surplus goes to the storage
    bool directEnergyUtilisation(double energyRequested, double energyReceived);

    /// @brief received energy plus storage cover the demand
    bool storageSupportedEnergy(double energyRequested, double energyReceived);

    /// @brief received energy and the whole storage are used, the grid covers the rest
    void gridSupportedEnergy(double energyRequested, double energyReceived);

    /// @brief charge the storage from the grid while below drawTarget
    void passiveDraw(double stepLength);

    /// @brief write all values of flows, appending suffix to the attribute names
    static void writeFlows(OutputDevice& out, const EnergyFlows& flows, const std::string& suffix);

    /// @brief Read a double parameter, throwing a ProcessError naming station and key if invalid
    static double readDouble(const std::string& chargingStationID, const Parameterised& params, const std::string& key, double defaultValue);

private:
    /// @name Persistent parameters
    /// @{
    const std::string myID;
    const std::string myChargingStationID;
    /// @brief capacity [Wh]
    const double myMaximumStorageCapacity;
    /// @brief initial state of charge as fraction of capacity [0..1]
    const double myStateOfChargeStartUp;
    /// @brief passive draw until this fraction of capacity [0..1]
    const double myDrawTarget;
    /// @brief passive draw power [W]
    const double myDrawPower;
    /// @}

    /// @name Runtime parameters
    /// @{
    /// @brief current state of charge [Wh]
    double myStorageCapacity;
    /// @brief energy flows of the current step
    EnergyFlows myStep;
    /// @brief energy flows summed up since the last output
    EnergyFlows myPeriod;
    /// @brief accumulated energy flows
    EnergyFlows myTotal;
    /// @}

private:
    /// @brief Invalidated copy constructor.
    MSStationaryEnergyStorage(const MSStationaryEnergyStorage&) = delete;

    /// @brief Invalidated assignment operator.
    MSStationaryEnergyStorage& operator=(const MSStationaryEnergyStorage&) = delete;
};
