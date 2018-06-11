//
// Created by jemin on 22.06.16.
//

#ifndef ENUMERATION_HPP
#define ENUMERATION_HPP

/* This enumeration only stores enumeration that is common for the whole RAI framework*/
namespace rai {

  enum class TerminationType {
    not_terminated = 0,
    timeout,
    terminalState,
    stationary_state,        // termination due to a stationary state
    missionSuccess
  };

  enum VisualizationOption {
    showEverything = 0,
    dontShowAnything,
    showOnlyTheNoiselessTrajectory,
    showNoiselessAndTheFirstNoisyTrajectory
  };

  enum ControllerType {
    controller_deterministric = 0,
    controller_stochastic
  };

  enum class TaskStochasticity {
    determinisitic = 0,
    stochastic
  };

}
#endif //ENUMERATION_HPP
