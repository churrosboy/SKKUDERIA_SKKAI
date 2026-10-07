import enum

class StateType(enum.Enum):
    GB_TRACK = 'GB_TRACK' 
    TRAILING = 'TRAILING' 
    OVERTAKE = 'OVERTAKE' 
    FTGONLY = 'FTGONLY'
    COLLISION = 'COLLISION'
    RECOVERY = 'RECOVERY'
    # LOW_BAT = 'LOW_BAT'
