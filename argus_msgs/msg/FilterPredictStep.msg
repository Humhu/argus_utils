# Message detailing a filter predict step
#
# Fields
# ======
# step_dt         : The predict time step size
# trans_jacobian  : Transition function jacobian
# trans_noise_cov : Transition noise covariance
# prior_state_cov : State covariance before predict
# post_state_cov  : State covariance after predict 

float64 step_dt 
MatrixFloat64 trans_jacobian
MatrixFloat64 trans_noise_cov
MatrixFloat64 prior_state_cov
MatrixFloat64 post_state_cov