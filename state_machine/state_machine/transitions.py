from __future__ import annotations

from typing import TYPE_CHECKING

from state_machine.state_types import StateType

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine


def dummy_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            if state_machine._low_bat:
                return StateType.LOW_BAT
            else:
                return StateType.GB_TRACK
        case StateType.LOW_BAT:
            return StateType.LOW_BAT
        case default:
            return StateType.GB_TRACK
        
        
def timetrials_transition(state_machine: StateMachine)->str:
    return StateType.GB_TRACK

def _gb_or_recovery(state_machine: StateMachine) -> StateType:
    """Route every return to GB_TRACK through RECOVERY first, so the controller can
    extend its lookahead and ease back onto the raceline instead of snapping. RECOVERY
    exits to GB_TRACK on the next tick if already close (see SpliniRecoveryTransition).
    Disabled via enable_recovery_state."""
    if getattr(state_machine.params, 'enable_recovery_state', False):
        return StateType.RECOVERY
    return StateType.GB_TRACK

def head_to_head_transition(state_machine: StateMachine)->str:
    match state_machine.state:
        case StateType.GB_TRACK:
            return SpliniGlobalTrackingTransition(state_machine)
        case StateType.TRAILING:
            return SpliniTrailingTransition(state_machine)
        case StateType.OVERTAKE:
            return SpliniOvertakingTransition(state_machine)
        case StateType.FTGONLY:
            return SpliniFTGOnlyTransition(state_machine)
        case StateType.RECOVERY:
            return SpliniRecoveryTransition(state_machine)
        case default:
            raise ValueError(f"Invalid state {state_machine.state}")


def SpliniGlobalTrackingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.GB_TRACK`"""
    if not state_machine._check_only_ftg_zone:
        if state_machine._check_gbfree:
            if (getattr(state_machine.params, 'enable_recovery_state', False)
                    and state_machine.cur_d is not None
                    and abs(state_machine.cur_d - state_machine._lane_d_ref) > state_machine.params.recovery_enter_d_m):
                return StateType.RECOVERY
            return StateType.GB_TRACK
        else:
            if (getattr(state_machine.params, 'direct_static_overtake', False)
                    and state_machine._dynamic_blocker is None
                    and state_machine._check_ot_entry_allowed
                    and state_machine._check_availability_splini_wpts
                    and state_machine._check_ofree):
                return StateType.OVERTAKE
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniTrailingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.TRAILING`"""
    gb_free = state_machine._check_gbfree
    ot_allowed = state_machine._check_ot_entry_allowed

    if not state_machine._check_only_ftg_zone:
        # If we have been sitting around in TRAILING for a while then FTG
        if state_machine._check_ftg:
            return StateType.FTGONLY
        if not gb_free and not ot_allowed:
            return StateType.TRAILING
        elif gb_free and state_machine._check_close_to_raceline:
            return _gb_or_recovery(state_machine)
        elif (
            not gb_free
            and ot_allowed
            and state_machine._check_availability_splini_wpts
            and state_machine._check_ofree
        ):
            return StateType.OVERTAKE
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniOvertakingTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.OVERTAKE`"""
    if not state_machine._check_only_ftg_zone:
        in_ot_sector = state_machine._check_ot_keep_allowed
        spline_valid = state_machine._check_availability_splini_wpts
        o_free = state_machine._check_ofree

        # If spline is on an obstacle we trail
        if not o_free:
            return StateType.TRAILING
        if in_ot_sector and o_free and spline_valid:
            return StateType.OVERTAKE
        # If spline becomes unvalid while overtaking, we trail
        elif in_ot_sector and not spline_valid and not o_free:
            return StateType.TRAILING
        # go to GB_TRACK if not in ot_sector and the GB is free
        elif not in_ot_sector and state_machine._check_gbfree:
            return _gb_or_recovery(state_machine)
        # go to Trailing if not in ot_sector and the GB is not free
        else:
            return StateType.TRAILING
    else:
        return StateType.FTGONLY


def SpliniFTGOnlyTransition(state_machine: StateMachine) -> StateType:
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    else:
        if state_machine._check_gbfree:
            return _gb_or_recovery(state_machine)
        else:
            return StateType.FTGONLY


def SpliniRecoveryTransition(state_machine: StateMachine) -> StateType:
    """Transitions for being in `StateType.RECOVERY` -- tracking the
    global raceline with an extended controller lookahead until back on the line.
    Mirrors GB_TRACK's transition so obstacle handling keeps working mid-recovery;
    exits to GB_TRACK once |d| < recovery_exit_d_m (hysteresis vs recovery_enter_d_m)."""
    if state_machine._check_only_ftg_zone:
        return StateType.FTGONLY
    if not state_machine._check_gbfree:
        return StateType.TRAILING
    if (state_machine.cur_d is not None
            and abs(state_machine.cur_d - state_machine._lane_d_ref) < state_machine.params.recovery_exit_d_m):
        return StateType.GB_TRACK
    if not getattr(state_machine.params, 'enable_recovery_state', False):
        return StateType.GB_TRACK
    return StateType.RECOVERY
