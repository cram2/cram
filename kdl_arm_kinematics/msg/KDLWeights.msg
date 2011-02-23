byte INVALID_MODE = 0
byte SET_TS = 1
byte SET_JS = 2
byte SET_LAMBDA = 4
byte SET_TS_JS = 3

byte mode                  # or-combination of values to set
float64[36] weight_ts      # the 6x6 matrix of task space weights in row-major order
float64[] weight_js        # the joints x joints matrix of joint space weights in row-major order
float64 lambda             # Lambda value
