# C-scan scanning status message

uint8 STATUS_IDLE = 0
uint8 STATUS_PATH_PLANNING = 1
uint8 STATUS_READY = 2
uint8 STATUS_SCANNING = 3
uint8 STATUS_DWELL = 4
uint8 STATUS_COMPLETE = 5
uint8 STATUS_ERROR = 6

uint8 status          # Current scanning state
uint32 current_point  # Current grid point index
uint32 total_points   # Total number of grid points
float32 progress      # Progress percentage (0.0 - 100.0)
string message        # Status message for debugging