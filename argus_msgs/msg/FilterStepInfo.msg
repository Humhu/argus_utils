# Message detailing a filter predict or update step
#
# Fields
# ======
# header/stamp     : Time at which this step ends or occurs at
# header/frame_id  : This observation source's name [unused for predict]
# step_num  : Filter step index
# info_type : Enumerated active payload type
# predict   : Predict info payload
# update    : Update info payload

# Defines the active payload field
uint8 PREDICT_STEP = 0
uint8 UPDATE_STEP = 1

std_msgs/Header header
uint64 step_num
uint8 info_type
argus_msgs/FilterPredictStep predict
argus_msgs/FilterUpdateStep update
